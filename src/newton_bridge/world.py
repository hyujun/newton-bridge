"""NewtonWorld: wraps Newton ModelBuilder + solver + double-buffered state.

IMPORTANT: Newton's Python API is still stabilizing (~1.x). The sections marked
`# --- NEWTON API SURFACE ---` are the places to adjust if a future release
moves things around.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Iterable

import numpy as np
import warp as wp
import newton
from newton.selection import ArticulationView

from .softbody import SoftbodySpec, load_softbody_npz, parse_softbodies

log = logging.getLogger(__name__)


# JointTargetMode name (yaml) -> enum. Accepted on pack['drive.mode'] and
# pack['joints.<name>.drive.mode']. Case-insensitive at parse time.
_DRIVE_MODE = {
    "position": newton.JointTargetMode.POSITION,
    "velocity": newton.JointTargetMode.VELOCITY,
    "effort": newton.JointTargetMode.EFFORT,
    "position_velocity": newton.JointTargetMode.POSITION_VELOCITY,
    "none": newton.JointTargetMode.NONE,
}


def parse_drive_mode(name: str) -> newton.JointTargetMode:
    try:
        return _DRIVE_MODE[str(name).strip().lower()]
    except KeyError:
        raise ValueError(
            f"unknown drive.mode: {name!r}. "
            f"expected one of {sorted(_DRIVE_MODE.keys())}"
        ) from None


# Newton 1.1.0 has no native particle↔body constraint. The franka softbody
# example pins particle_mass=0 (in PR 2) and overwrites particle_q/qd from
# the attached body's transform every substep — this kernel is the overwrite.
# Spatial vector layout for body_qd is (omega, v_lin) in Newton 1.x.
@wp.kernel
def update_attach_particles(
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    body_qd: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
    attach_particle_idx: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    attach_body_idx: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    attach_local_pos: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    particle_q: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    particle_qd: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
):
    tid = wp.tid()
    pi = attach_particle_idx[tid]
    bi = attach_body_idx[tid]
    local = attach_local_pos[tid]

    xform = body_q[bi]
    p = wp.transform_get_translation(xform)
    q = wp.transform_get_rotation(xform)

    r_world = wp.quat_rotate(q, local)
    particle_q[pi] = p + r_world

    twist = body_qd[bi]
    omega = wp.spatial_top(twist)
    v_lin = wp.spatial_bottom(twist)
    particle_qd[pi] = v_lin + wp.cross(omega, r_world)


class NewtonWorld:
    """Thin wrapper: build model, hold solver + double-buffered state."""

    def __init__(self, pack: dict, device: str = "cuda:0") -> None:
        self.pack = pack
        self.device = device
        self.physics_dt: float = 1.0 / float(pack["sim"]["physics_hz"])
        self.substeps: int = int(pack["sim"].get("substeps", 1))

        # Parsed once up-front so _build_model/_build_solver can branch on
        # `self.softbodies` without re-walking the yaml. Empty list when the
        # pack has no `softbodies:` block.
        self.softbodies: list[SoftbodySpec] = parse_softbodies(
            pack, Path(pack["_pack_dir"])
        )

        self._build_model()
        self._build_solver()
        self._build_view()
        # Pre-allocate the contacts buffer so model.collide() can write
        # in-place. Mandatory for CUDA graph capture (graphs cannot allocate).
        # Sensors read this same buffer via self.last_contacts.
        self.contacts = self.model.contacts()
        self.last_contacts = self.contacts
        self._control_target_pos_host = np.zeros(self.total_dof, dtype=np.float32)
        self._control_target_vel_host = np.zeros(self.total_dof, dtype=np.float32)
        self._control_effort_host = np.zeros(self.total_dof, dtype=np.float32)
        self.sim_time: float = 0.0

        # builder.joint_q / joint_target_pos already put us at home, but
        # mirror into _control_target_pos_host + re-assert for reset() parity.
        self._apply_home_pose()

        # CUDA graph for the per-step kernel sequence. Captured lazily on
        # the first cuda step(); set back to None whenever the model changes
        # (reset / set_gravity / etc.) to force a recapture.
        self._step_graph: wp.Graph | None = None
        self._capture_step_graph()

    # -- NEWTON API SURFACE -------------------------------------------------
    def _build_model(self) -> None:
        src_fmt = self.pack["robot"]["source"]
        src_path = str(self.pack["_pack_dir"] / self.pack["robot"]["source_rel"])
        base_pos = self.pack["robot"].get("base_position", [0.0, 0.0, 0.0])

        builder = newton.ModelBuilder()
        xform = wp.transform(base_pos, wp.quat_identity())
        if src_fmt == "urdf":
            # enable_self_collisions=False: URDF mesh colliders otherwise
            # overflow the contact buffer (seen on UR5e) and freeze the step.
            builder.add_urdf(
                src_path, xform=xform, floating=False, enable_self_collisions=False
            )
        elif src_fmt == "xacro":
            # Process xacro in-process → URDF XML string, then hand it to the
            # URDF importer via a tempfile. (Passing XML directly trips a bug
            # where newton's importer calls os.path.abspath on the string for
            # mesh URI resolution, producing nonsense paths.)
            #
            # xacro_loader rewrites every `$(find ur_description)` to the
            # pack's own models/ dir and asserts the output URDF contains no
            # `package://` / `/opt/ros/` references — so meshes resolve to
            # robots/<name>/models/meshes/... and the runtime never touches
            # the host or container ROS share.
            import tempfile
            from .xacro_loader import process_xacro
            urdf_xml = process_xacro(
                src_path, args=self.pack["robot"].get("source_args") or {}
            )
            # Write to the OS temp dir (not pack_dir — it may be a read-only
            # bind-mount). The URDF's on-disk location doesn't matter because
            # all mesh URIs are absolute file:// paths into the pack tree.
            with tempfile.NamedTemporaryFile(
                "w", suffix=".urdf", delete=False
            ) as fh:
                fh.write(urdf_xml)
                urdf_tmp = fh.name
            try:
                builder.add_urdf(
                    urdf_tmp, xform=xform, floating=False, enable_self_collisions=False
                )
            finally:
                try:
                    Path(urdf_tmp).unlink()
                except OSError:
                    pass
        elif src_fmt == "mjcf":
            builder.add_mjcf(src_path, xform=xform, floating=False)
        else:
            raise ValueError(
                f"unknown robot.source: {src_fmt!r} (expected urdf|xacro|mjcf)"
            )

        if self.pack["sim"].get("ground_plane", True):
            builder.add_ground_plane()

        # Gains/mode/home must be written to the builder BEFORE finalize;
        # post-finalize assigns to model.joint_target_ke/kd were no-ops in
        # Newton 1.1.0 testing (the solver captures these at finalize time).
        self._configure_builder_drive(builder)

        # Softbody assets are added to the same builder before finalize.
        # `_build_softbodies` records per-spec (start_vertex, vertex_count)
        # plus the resolved attach (body_index, local_offset) tuples that
        # the post-finalize step needs to pin particle_mass.
        self._build_softbodies(builder)

        # SolverVBD requires `model.particle_color_groups` populated by
        # `builder.color()` (graph coloring of the soft-mesh constraint
        # graph). Newton's example_softbody_franka does this immediately
        # before finalize. No-op when no soft particles are present.
        if self.softbodies:
            builder.color()

        self.model = builder.finalize(device=self.device)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

        # Pin attach particles by zeroing their mass — Newton 1.1.0's only
        # kinematic-particle hook. The actual per-step transform overwrite
        # lands in PR 3 (`update_attach_particles` warp kernel).
        self._finalize_softbody_attach()

    # -- NEWTON API SURFACE -------------------------------------------------
    def _dof_names_from_builder(self, builder) -> list[str]:
        """Derive per-DOF short names from builder (pre-finalize; view unavailable).

        For URDF/MJCF revolute/prismatic joints (1 DOF each) this returns the
        joint's short label. Multi-DOF joints (D6/ball) get suffixed indices.
        ArticulationView later yields the same names, allowing pack overrides
        to be indexed by joint name.
        """
        names: list[str] = []
        for j_idx, label in enumerate(builder.joint_label):
            dims = builder.joint_dof_dim[j_idx]
            total = int(dims[0]) + int(dims[1])
            if total == 0:
                continue
            short = label.rsplit("/", 1)[-1]
            if total == 1:
                names.append(short)
            else:
                names.extend([f"{short}_{d}" for d in range(total)])
        return names

    def _resolve_joint_drive(self, joint_name: str) -> dict:
        """Merge top-level drive with per-joint override in pack yaml."""
        default = dict(self.pack.get("drive", {}) or {})
        override = (self.pack.get("joints", {}) or {}).get(joint_name, {}) or {}
        out = {
            "mode": default.get("mode", "position"),
            "stiffness": float(default.get("stiffness", 0.0)),
            "damping": float(default.get("damping", 0.0)),
        }
        joint_drive = override.get("drive") or {}
        if "mode" in joint_drive:
            out["mode"] = joint_drive["mode"]
        if "stiffness" in joint_drive:
            out["stiffness"] = float(joint_drive["stiffness"])
        if "damping" in joint_drive:
            out["damping"] = float(joint_drive["damping"])
        # Non-drive per-joint scalars (armature, effort_limit, ...) are read
        # directly from `override` by _configure_builder_drive.
        return out

    # -- NEWTON API SURFACE -------------------------------------------------
    def _configure_builder_drive(self, builder) -> None:
        """Apply drive/mode/limit params per DOF; then home pose.

        Sources are merged in priority order: per-joint override (pack's
        `joints.<name>.*`) overlays top-level `drive.*` defaults.
        """
        dof_names = self._dof_names_from_builder(builder)
        joints_override = self.pack.get("joints", {}) or {}
        dof_count = int(builder.joint_dof_count)

        for i, name in enumerate(dof_names):
            if i >= dof_count:
                break
            drive = self._resolve_joint_drive(name)
            builder.joint_target_ke[i] = drive["stiffness"]
            builder.joint_target_kd[i] = drive["damping"]
            builder.joint_target_mode[i] = int(parse_drive_mode(drive["mode"]))

            over = joints_override.get(name, {}) or {}
            if "armature" in over:
                builder.joint_armature[i] = float(over["armature"])
            if "effort_limit" in over:
                builder.joint_effort_limit[i] = float(over["effort_limit"])
            if "velocity_limit" in over:
                builder.joint_velocity_limit[i] = float(over["velocity_limit"])
            if "friction" in over:
                builder.joint_friction[i] = float(over["friction"])
            if "limit_ke" in over:
                builder.joint_limit_ke[i] = float(over["limit_ke"])
            if "limit_kd" in over:
                builder.joint_limit_kd[i] = float(over["limit_kd"])

        # Home pose: pack-exposed name -> DOF index (via name lookup), not
        # positional. This tolerates pack subset ordering.
        home = self.pack.get("home_pose", {}) or {}
        if home:
            name_to_i = {n: i for i, n in enumerate(dof_names)}
            for name, val in home.items():
                i = name_to_i.get(name)
                if i is None or i >= dof_count:
                    continue
                val_f = float(val)
                builder.joint_q[i] = val_f
                builder.joint_target_pos[i] = val_f

    # -- NEWTON API SURFACE -------------------------------------------------
    def _build_softbodies(self, builder) -> None:
        """Add each `softbodies:` entry to `builder` before finalize.

        Per-spec we record:
            (start_vertex, vertex_count)        # offset into model.particle_q
            attach.body_index                   # index into model.body_q
            attach.local_offset (3,) float32    # body-frame pin location

        The body index is resolved against the builder's pre-finalize body
        label list (NOT the post-finalize ArticulationView, which doesn't
        exist yet at this stage). Local offsets are computed from the
        builder's initial body transform: local = R0^T · (vertex_world - p0).

        Vertex_world is the npz vertex with the spec's xform applied — i.e.
        the position the particle will sit at right after finalize, which
        keeps the attach-kernel offset consistent with the actual particle
        spawn position.
        """
        if not self.softbodies:
            self._softbody_layout = []
            self._softbody_attach_meta = []
            return

        body_labels = self._builder_body_labels(builder)
        layout: list[tuple[int, int]] = []
        attach_meta: list[dict] = []

        for spec in self.softbodies:
            arrays = load_softbody_npz(spec.asset_path)

            # particle_q starts at the current vertex count — this is the
            # offset newton.ModelBuilder appends new soft-mesh particles at.
            start = int(builder.particle_count)
            mesh = newton.TetMesh(
                arrays.vertices,
                arrays.tet_indices,
                k_mu=spec.material.k_mu,
                k_lambda=spec.material.k_lambda,
                k_damp=spec.material.k_damp,
                density=spec.material.density,
            )
            # Newton 1.1.0: add_soft_mesh dispatches into warp built-ins, so
            # pos/rot/vel must be wp.vec3 / wp.quat — passing lists/tuples
            # raises "Built-in functions cannot be called with non-Warp array
            # types" at finalize time.
            kwargs = dict(
                pos=wp.vec3(
                    float(spec.pos[0]), float(spec.pos[1]), float(spec.pos[2])
                ),
                rot=wp.quat(
                    float(spec.rot[0]),
                    float(spec.rot[1]),
                    float(spec.rot[2]),
                    float(spec.rot[3]),
                ),
                scale=float(spec.scale),
                vel=wp.vec3(0.0, 0.0, 0.0),
                density=spec.material.density,
                k_mu=spec.material.k_mu,
                k_lambda=spec.material.k_lambda,
                k_damp=spec.material.k_damp,
            )
            if spec.particle_radius is not None:
                kwargs["particle_radius"] = spec.particle_radius
            builder.add_soft_mesh(mesh=mesh, **kwargs)
            count = int(builder.particle_count) - start
            if count <= 0:
                raise RuntimeError(
                    f"softbody {spec.name!r}: builder.add_soft_mesh added "
                    f"{count} particles (expected > 0)"
                )
            layout.append((start, count))

            # Resolve the URDF/MJCF link name to a body index.
            try:
                body_index = body_labels.index(spec.attach.body)
            except ValueError:
                # Allow short-name match (`tool0` matching `arm/tool0`) since
                # importers commonly prepend the articulation prefix.
                shorts = [b.rsplit("/", 1)[-1] for b in body_labels]
                if spec.attach.body in shorts:
                    body_index = shorts.index(spec.attach.body)
                else:
                    raise ValueError(
                        f"softbody {spec.name!r}: attach.body={spec.attach.body!r} "
                        f"not found among bodies: {body_labels}"
                    ) from None

            # Body initial transform from the builder. body_q is laid out as
            # [px, py, pz, qx, qy, qz, qw] per body (Newton convention).
            bq = np.asarray(builder.body_q, dtype=np.float32)
            if bq.ndim == 1:
                bq = bq.reshape(-1, 7)
            p0 = bq[body_index, 0:3].astype(np.float32)
            q0 = bq[body_index, 3:7].astype(np.float32)  # xyzw

            # Particle world spawn = npz vertex * scale, rotated by spec.rot,
            # then translated by spec.pos. We mirror that math here so the
            # local offset we hand PR 3 lines up exactly with the particle
            # the kernel will overwrite.
            spec_R = self._quat_to_matrix(np.asarray(spec.rot, dtype=np.float32))
            spec_p = np.asarray(spec.pos, dtype=np.float32)

            local_offsets = []
            for vi in spec.attach.vertex_indices:
                if vi >= arrays.vertex_count:
                    raise ValueError(
                        f"softbody {spec.name!r}: attach.vertex_indices contains "
                        f"{vi} but mesh only has {arrays.vertex_count} vertices"
                    )
                v = arrays.vertices[vi].astype(np.float32) * float(spec.scale)
                world = spec_R @ v + spec_p
                # local = R0^T · (world - p0)
                R0 = self._quat_to_matrix(q0)
                local = R0.T @ (world - p0)
                local_offsets.append(local)

            attach_meta.append(
                {
                    "spec_name": spec.name,
                    "body_index": int(body_index),
                    "particle_indices": np.asarray(
                        [start + int(vi) for vi in spec.attach.vertex_indices],
                        dtype=np.int32,
                    ),
                    "local_offsets": np.asarray(local_offsets, dtype=np.float32),
                }
            )

        self._softbody_layout = layout
        self._softbody_attach_meta = attach_meta

    @staticmethod
    def _builder_body_labels(builder) -> list[str]:
        # Newton's ModelBuilder exposes body labels as either `body_key`
        # (newer builds) or `body_label` (older). Try both, then fall back
        # to bare indices so a missing list still yields a useful error.
        for attr in ("body_key", "body_label"):
            v = getattr(builder, attr, None)
            if v:
                return list(v)
        return [f"body_{i}" for i in range(int(getattr(builder, "body_count", 0)))]

    @staticmethod
    def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
        """xyzw quaternion → 3x3 rotation. Local copy avoids a scipy dep."""
        x, y, z, w = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        n = x * x + y * y + z * z + w * w
        if n <= 0.0:
            return np.eye(3, dtype=np.float32)
        s = 2.0 / n
        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s
        return np.asarray(
            [
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ],
            dtype=np.float32,
        )

    def _finalize_softbody_attach(self) -> None:
        """Pin attach particles by zeroing their mass; cache wp.array views.

        Runs immediately after `builder.finalize`. `model.particle_mass` is
        a wp.array on the simulation device; we read it back, zero the
        attach indices, and write it back. We also stash device-side wp
        arrays for `attach_particle_idx`, `attach_body_idx`, and
        `attach_local_pos`, which PR 3's warp kernel will consume.
        """
        if not self._softbody_attach_meta:
            self.attach_particle_idx = None
            self.attach_body_idx = None
            self.attach_local_pos = None
            return

        all_particle_idx = np.concatenate(
            [m["particle_indices"] for m in self._softbody_attach_meta]
        ).astype(np.int32)
        all_local_pos = np.concatenate(
            [m["local_offsets"] for m in self._softbody_attach_meta], axis=0
        ).astype(np.float32)
        all_body_idx = np.concatenate(
            [
                np.full(
                    len(m["particle_indices"]), m["body_index"], dtype=np.int32
                )
                for m in self._softbody_attach_meta
            ]
        )

        # Zero particle_mass at the attach indices (kinematic pinning).
        pmass_host = self.model.particle_mass.numpy().copy()
        pmass_host[all_particle_idx] = 0.0
        self.model.particle_mass.assign(pmass_host)

        self.attach_particle_idx = wp.array(
            all_particle_idx, dtype=wp.int32, device=self.device
        )
        self.attach_body_idx = wp.array(
            all_body_idx, dtype=wp.int32, device=self.device
        )
        # vec3 layout for the warp kernel consumer in PR 3.
        self.attach_local_pos = wp.array(
            all_local_pos, dtype=wp.vec3, device=self.device
        )

    # -- NEWTON API SURFACE -------------------------------------------------
    def _build_solver(self) -> None:
        sim_cfg = self.pack["sim"]
        solver_name = sim_cfg.get("solver", "xpbd").lower()
        kwargs = dict(sim_cfg.get("solver_params", {}) or {})

        solver_map = {
            "xpbd": newton.solvers.SolverXPBD,
            "mujoco": newton.solvers.SolverMuJoCo,
            "featherstone": newton.solvers.SolverFeatherstone,
            "semi_implicit": newton.solvers.SolverSemiImplicit,
            "style3d": newton.solvers.SolverStyle3D,
            "vbd": newton.solvers.SolverVBD,
        }
        cls = solver_map.get(solver_name)
        if cls is None:
            raise ValueError(
                f"unknown solver: {solver_name!r}. "
                f"expected one of {sorted(solver_map.keys())}"
            )
        try:
            self.solver = cls(self.model, **kwargs)
        except TypeError as exc:
            raise ValueError(
                f"solver {solver_name!r} rejected solver_params={kwargs!r}: {exc}"
            ) from None

        # Only SolverMuJoCo honors <mimic> and <equality>. Other solvers
        # silently ignore them, which manifests as passive joints not moving.
        # See docs/CONSTRAINTS.md.
        if solver_name != "mujoco":
            n_eq = int(getattr(self.model, "equality_constraint_count", 0) or 0)
            n_mimic = int(getattr(self.model, "constraint_mimic_count", 0) or 0)
            if n_eq or n_mimic:
                log.warning(
                    "solver=%r silently ignores %d equality + %d mimic "
                    "constraint(s); passive joints will not move. "
                    "Use solver=mujoco for grasping / closed-chain robots "
                    "(see docs/CONSTRAINTS.md).",
                    solver_name, n_eq, n_mimic,
                )

        # Hybrid mode: when softbodies are present, a separate VBD solver
        # owns the soft particles and a CollisionPipeline produces soft
        # contacts. Step kernels still run rigid-only in PR 2; PR 3 wires
        # the franka-pattern co-step.
        if self.softbodies:
            self._build_soft_solver()
        else:
            self.soft_solver = None
            self.collision_pipeline = None

    # -- NEWTON API SURFACE -------------------------------------------------
    def _build_soft_solver(self) -> None:
        sim_cfg = self.pack["sim"]
        soft_cfg = dict(sim_cfg.get("soft_solver_params", {}) or {})
        soft_cfg.setdefault("integrate_with_external_rigid_solver", True)

        try:
            self.soft_solver = newton.solvers.SolverVBD(self.model, **soft_cfg)
        except TypeError as exc:
            raise ValueError(
                f"SolverVBD rejected soft_solver_params={soft_cfg!r}: {exc}"
            ) from None

        pipeline_cfg = dict(sim_cfg.get("collision_pipeline_params", {}) or {})
        try:
            self.collision_pipeline = newton.CollisionPipeline(
                self.model, **pipeline_cfg
            )
        except TypeError as exc:
            raise ValueError(
                f"CollisionPipeline rejected params={pipeline_cfg!r}: {exc}"
            ) from None

        # Per-spec contact override: write soft_contact_ke/kd/mu on the model
        # if the spec sets them. Newton applies these globally (no per-mesh
        # override in 1.1.0) — last spec wins. Documented in the YAML schema.
        for spec in self.softbodies:
            if spec.contact.ke is not None:
                self.model.soft_contact_ke = float(spec.contact.ke)
            if spec.contact.kd is not None:
                self.model.soft_contact_kd = float(spec.contact.kd)
            if spec.contact.mu is not None:
                self.model.soft_contact_mu = float(spec.contact.mu)

        # Pre-allocate the soft contacts buffer that the captured graph will
        # write to in-place (graphs cannot allocate). Mirrors the rigid
        # `self.contacts` buffer pattern. Distinct buffer because the rigid
        # solver and CollisionPipeline don't share contact storage in 1.1.0.
        self.soft_contacts = self.collision_pipeline.contacts()

        # Gravity-toggle buffers used by the hybrid step. Newton's
        # model.gravity is a wp.array(vec3); we swap two pre-built arrays
        # in/out to disable gravity during the rigid kinematic phase. Pulls
        # the configured gravity from the model so the toggle is consistent
        # with whatever set_gravity() / yaml configured.
        g = np.asarray(self.model.gravity.numpy(), dtype=np.float32).reshape(-1, 3)[0]
        self._gravity_zero_buf = wp.array(
            [(0.0, 0.0, 0.0)], dtype=wp.vec3, device=self.device
        )
        self._gravity_active_buf = wp.array(
            [(float(g[0]), float(g[1]), float(g[2]))],
            dtype=wp.vec3,
            device=self.device,
        )

        # Attach kernel launch dim — int(0) when there are no attachments
        # (the kernel branch then becomes a no-op via `if self._attach_count`).
        self._attach_count = (
            0 if self.attach_particle_idx is None
            else int(self.attach_particle_idx.shape[0])
        )

    def set_gravity(self, gravity: tuple[float, float, float]) -> None:
        """Runtime gravity change (Phase 6a). Wraps model.set_gravity(vec3)."""
        g = [float(gravity[0]), float(gravity[1]), float(gravity[2])]
        self.model.set_gravity(g)
        # Hybrid mode caches the active gravity in a wp.array so the
        # captured graph can swap it in/out per substep — keep it in sync.
        if self.softbodies:
            self._gravity_active_buf.assign([(g[0], g[1], g[2])])
        # Newton solvers cache model state at construction; tell the solver
        # to re-read, then recapture the graph so it picks up the new params.
        notify = getattr(self.solver, "notify_model_changed", None)
        if callable(notify):
            try:
                notify()
            except TypeError:
                # Some Newton revisions take a flags arg; recapture anyway.
                pass
        self._capture_step_graph()

    # -- NEWTON API SURFACE -------------------------------------------------
    def _build_view(self) -> None:
        """Create an ArticulationView for name-based DOF/link access.

        The pattern is an fnmatch glob (not regex) matched against
        `model.articulation_label[*]`. Default `*` picks up every
        articulation in the model (fine for single-robot packs).
        """
        pattern = str(self.pack.get("articulation_pattern", "*"))
        self.view = ArticulationView(self.model, pattern=pattern)

        dof_names: list[str] = list(self.view.joint_dof_names)
        configured: list[str] = list(self.pack["joint_names"])

        view_set = set(dof_names)
        missing = [n for n in configured if n not in view_set]
        if missing:
            raise RuntimeError(
                "pack['joint_names'] references joints not in ArticulationView DOFs.\n"
                f"  missing:    {missing}\n"
                f"  configured: {configured}\n"
                f"  actual:     {dof_names}\n"
                f"  pattern:    {pattern!r}"
            )

        self.joint_dof_names: list[str] = dof_names
        self.exposed_joint_names: list[str] = configured
        self.total_dof: int = int(self.view.joint_dof_count)
        self._dof_index: dict[str, int] = {n: i for i, n in enumerate(dof_names)}

    def _apply_home_pose(self) -> None:
        home = self.pack.get("home_pose", {}) or {}
        if not home:
            return

        q = np.zeros(self.total_dof, dtype=np.float32)
        for name, val in home.items():
            i = self._dof_index.get(name)
            if i is None:
                continue
            q[i] = float(val)

        shaped = q.reshape(1, 1, self.total_dof)
        self.view.set_dof_positions(self.state_0, shaped)
        self.view.set_dof_positions(self.state_1, shaped)
        self.state_0.joint_qd.zero_()
        self.state_1.joint_qd.zero_()

        self._control_target_pos_host[:] = q
        self._control_target_vel_host[:] = 0.0
        self._control_effort_host[:] = 0.0
        self.control.joint_target_pos.assign(self._control_target_pos_host)
        self.control.joint_target_vel.assign(self._control_target_vel_host)
        self.control.joint_f.assign(self._control_effort_host)

    # ----------------------------------------------------------------------
    def sync_device(self) -> None:
        """Block until all queued GPU work on the simulation device is done.

        Diagnostic helper: when the physics step is captured into a CUDA graph,
        `step()` returns immediately and the actual compute completes async.
        Calling this before any readback isolates "wait for step compute" cost
        from the readback (DtoH copy) cost itself.
        """
        wp.synchronize_device(self.device)

    def read_joint_positions(self) -> dict[str, float]:
        q = self.view.get_dof_positions(self.state_0).numpy().reshape(-1)
        return {n: float(q[self._dof_index[n]]) for n in self.exposed_joint_names}

    def read_joint_velocities(self) -> dict[str, float]:
        qd = self.view.get_dof_velocities(self.state_0).numpy().reshape(-1)
        return {n: float(qd[self._dof_index[n]]) for n in self.exposed_joint_names}

    def read_joint_efforts(self) -> dict[str, float]:
        """Return **commanded** joint force per exposed joint.

        Newton 1.1.0 `State` does not expose `joint_f` (the applied torque);
        `get_dof_forces(control)` reads the commanded `control.joint_f` buffer.
        That's what we feed /joint_states.effort — a readback of what the
        controller sent us, not the solver's post-step force. For POSITION/
        VELOCITY modes this stays at 0 since we never write to joint_f.
        """
        f = self.view.get_dof_forces(self.control).numpy().reshape(-1)
        return {n: float(f[self._dof_index[n]]) for n in self.exposed_joint_names}

    def read_body_transforms(self) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        """Return {body_label: ((px, py, pz), (qx, qy, qz, qw))} for every body.

        Used by the /tf publisher. Coordinates are world frame (Newton native).
        """
        # shape (n_worlds, n_arts, n_bodies, 7). Single-art pack: flatten to (n_bodies, 7).
        arr = self.view.get_link_transforms(self.state_0).numpy().reshape(-1, 7)
        return {
            name: ((float(row[0]), float(row[1]), float(row[2])),
                   (float(row[3]), float(row[4]), float(row[5]), float(row[6])))
            for name, row in zip(self.view.body_names, arr)
        }

    def set_joint_targets(
        self,
        names: Iterable[str],
        positions: Iterable[float] | None = None,
        velocities: Iterable[float] | None = None,
        efforts: Iterable[float] | None = None,
    ) -> None:
        """Write command channels. Any of pos/vel/effort may be None to leave untouched.

        The joint's configured `drive.mode` determines which channel actually
        drives it; extra channels are stored but inert. Unknown joint names
        are silently ignored.
        """
        names = list(names)
        if positions is not None:
            for name, v in zip(names, positions):
                i = self._dof_index.get(name)
                if i is not None:
                    self._control_target_pos_host[i] = float(v)
            # -- NEWTON API SURFACE ---------------------------------------
            # Newton 1.1.0: POSITION setpoints live on control.joint_target_pos.
            self.control.joint_target_pos.assign(self._control_target_pos_host)
        if velocities is not None:
            for name, v in zip(names, velocities):
                i = self._dof_index.get(name)
                if i is not None:
                    self._control_target_vel_host[i] = float(v)
            self.control.joint_target_vel.assign(self._control_target_vel_host)
        if efforts is not None:
            for name, v in zip(names, efforts):
                i = self._dof_index.get(name)
                if i is not None:
                    self._control_effort_host[i] = float(v)
            # joint_f is commanded torque; joint_act is additive feedforward.
            # For EFFORT mode, writing joint_f is the intended channel.
            self.control.joint_f.assign(self._control_effort_host)

    # -- NEWTON API SURFACE -------------------------------------------------
    def _step_kernels(self) -> None:
        """Per-tick GPU kernel sequence — the body that gets graph-captured.

        Mirrors the Newton 1.x official robot examples (anymal_d, h1, g1 …):
        for each substep, refresh contacts in-place, clear per-state forces,
        run one solver step, then swap state_0/state_1. Because state swap
        runs *inside* the captured sequence, the graph encodes a fixed
        ping-pong over two pre-allocated state buffers — every launch
        produces the same input→output memory mapping.

        Hybrid (softbody) mode follows Newton's `example_softbody_franka.py`:
        rigid solver runs as a kinematic integrator (particle_count=0,
        gravity=0, shape_contact_pair_count=0) so it produces body
        velocities without touching particles or fighting contacts;
        attach kernel pins the kinematic particles to those fresh body
        transforms; CollisionPipeline + SolverVBD then advance the soft
        bodies. Single state-swap at the end matches the rigid-only path.
        """
        if self.softbodies:
            self._step_kernels_hybrid()
        else:
            self._step_kernels_rigid()

    def _step_kernels_rigid(self) -> None:
        dt = self.physics_dt / self.substeps
        n = self.substeps
        odd = n % 2 == 1
        for i in range(n):
            self.model.collide(self.state_0, self.contacts)
            self.state_0.clear_forces()
            self.solver.step(
                self.state_0, self.state_1, self.control, self.contacts, dt
            )
            # On the final substep when n is odd, copy the result back into
            # state_0 instead of rebinding Python refs. The Python swap is
            # not recorded into a captured CUDA graph, so an odd number of
            # swaps leaves the graph with baked-in (state_in=s0_orig,
            # state_out=s1_orig) and s0_orig never receives new input —
            # state never progresses. See newton.State.assign() docstring.
            if odd and i == n - 1:
                self.state_0.assign(self.state_1)
            else:
                self.state_0, self.state_1 = self.state_1, self.state_0

    def _step_kernels_hybrid(self) -> None:
        dt = self.physics_dt / self.substeps
        soft_solver = self.soft_solver
        collision_pipeline = self.collision_pipeline
        assert soft_solver is not None and collision_pipeline is not None, (
            "_step_kernels_hybrid invoked without soft_solver/collision_pipeline; "
            "this is an invariant violation — softbodies non-empty implies both."
        )
        # rebuild_bvh is a per-frame (not per-substep) op in the franka
        # example; cheap to run once before the substep loop.
        soft_solver.rebuild_bvh(self.state_0)

        # shape_contact_pair_count is restored each substep via this
        # captured value. Reading once outside the loop is fine — the value
        # is constant after finalize.
        spcc = int(self.model.shape_contact_pair_count)

        n = self.substeps
        odd = n % 2 == 1
        for i in range(n):
            self.state_0.clear_forces()
            self.state_1.clear_forces()
            self.model.collide(self.state_0, self.contacts)

            # --- rigid kinematic phase: disable particles, gravity, rigid
            # contacts so Featherstone/MuJoCo just integrates joint targets
            # into body_q/body_qd without disturbing the soft system.
            pc = self.model.particle_count
            self.model.particle_count = 0
            self.model.gravity.assign(self._gravity_zero_buf)
            self.model.shape_contact_pair_count = 0

            self.solver.step(
                self.state_0, self.state_1, self.control, self.contacts, dt
            )

            # state_1 holds the freshly integrated rigid result. Restore
            # model fields and zero any spurious particle_f the rigid
            # solver may have written into state_1.
            self.state_1.particle_f.zero_()
            self.model.particle_count = pc
            self.model.gravity.assign(self._gravity_active_buf)
            self.model.shape_contact_pair_count = spcc

            # --- attach: overwrite kinematic particles to follow body_q.
            if self._attach_count:
                wp.launch(
                    update_attach_particles,
                    dim=self._attach_count,
                    inputs=[
                        self.state_1.body_q,
                        self.state_1.body_qd,
                        self.attach_particle_idx,
                        self.attach_body_idx,
                        self.attach_local_pos,
                        self.state_1.particle_q,
                        self.state_1.particle_qd,
                    ],
                    device=self.device,
                )

            # --- soft phase: collision against the freshly-posed scene,
            # then VBD step. Soft solver writes state_0 from state_1 (note
            # the input/output swap) so the trailing rename keeps state_0
            # as "current" — same convention as the rigid-only path.
            collision_pipeline.collide(self.state_1, self.soft_contacts)
            soft_solver.step(
                self.state_1, self.state_0, self.control, self.soft_contacts, dt
            )
            # See _step_kernels_rigid for the odd-substep / graph-capture
            # rationale. The soft phase already writes state_1→state_0, so
            # the trailing operation here is the same swap-vs-assign choice.
            if odd and i == n - 1:
                self.state_0.assign(self.state_1)
            else:
                self.state_0, self.state_1 = self.state_1, self.state_0

    def _capture_step_graph(self) -> None:
        """Capture `_step_kernels` into a CUDA graph (cuda devices only).

        Falls back silently on CPU: `step()` then runs the eager path.
        Call again whenever `self.model` or `self.solver` configuration
        changes (e.g. set_gravity) — the captured graph references the
        device buffers as they were at capture time.
        """
        self._step_graph = None
        try:
            dev = wp.get_device(self.device)
        except Exception:
            return
        if not dev.is_cuda:
            return
        with wp.ScopedCapture() as capture:
            self._step_kernels()
        self._step_graph = capture.graph

    def step(self) -> None:
        if self._step_graph is not None:
            wp.capture_launch(self._step_graph)
        else:
            self._step_kernels()
        # Sensors read self.last_contacts; the buffer is pre-allocated and
        # rewritten in-place by collide(), so the reference stays valid.
        self.last_contacts = self.contacts
        self.sim_time += self.physics_dt

    def reset(self) -> None:
        self.sim_time = 0.0
        self._apply_home_pose()
        # Home pose write touches state_0 / state_1 buffers. The captured
        # graph references those same buffers, so no recapture is needed
        # here — but keep this comment as a tripwire if reset() ever starts
        # rebuilding state objects.
