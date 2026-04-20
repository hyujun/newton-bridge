"""NewtonWorld: wraps Newton ModelBuilder + solver + double-buffered state.

IMPORTANT: Newton's Python API is still stabilizing (~1.x). The sections marked
`# --- NEWTON API SURFACE ---` are the places to adjust if a future release
moves things around.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np
import warp as wp
import newton
from newton.selection import ArticulationView


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


class NewtonWorld:
    """Thin wrapper: build model, hold solver + double-buffered state."""

    def __init__(self, pack: dict, device: str = "cuda:0") -> None:
        self.pack = pack
        self.device = device
        self.physics_dt: float = 1.0 / float(pack["sim"]["physics_hz"])
        self.substeps: int = int(pack["sim"].get("substeps", 1))

        self._build_model()
        self._build_solver()
        self._build_view()
        self._control_target_pos_host = np.zeros(self.total_dof, dtype=np.float32)
        self._control_target_vel_host = np.zeros(self.total_dof, dtype=np.float32)
        self._control_effort_host = np.zeros(self.total_dof, dtype=np.float32)
        self.sim_time: float = 0.0

        # builder.joint_q / joint_target_pos already put us at home, but
        # mirror into _control_target_pos_host + re-assert for reset() parity.
        self._apply_home_pose()

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
        elif src_fmt == "mjcf":
            builder.add_mjcf(src_path, xform=xform, floating=False)
        else:
            raise ValueError(f"unknown robot.source: {src_fmt!r} (expected urdf|mjcf)")

        if self.pack["sim"].get("ground_plane", True):
            builder.add_ground_plane()

        # Gains/mode/home must be written to the builder BEFORE finalize;
        # post-finalize assigns to model.joint_target_ke/kd were no-ops in
        # Newton 1.1.0 testing (the solver captures these at finalize time).
        self._configure_builder_drive(builder)

        self.model = builder.finalize(device=self.device)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

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
    def _build_solver(self) -> None:
        solver_name = self.pack["sim"].get("solver", "xpbd").lower()
        if solver_name == "xpbd":
            self.solver = newton.solvers.SolverXPBD(self.model)
        elif solver_name == "mujoco":
            self.solver = newton.solvers.SolverMuJoCo(self.model)
        elif solver_name == "featherstone":
            self.solver = newton.solvers.SolverFeatherstone(self.model)
        else:
            raise ValueError(f"unknown solver: {solver_name!r}")

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
    def step(self) -> None:
        dt = self.physics_dt / self.substeps
        for _ in range(self.substeps):
            contacts = self.model.collide(self.state_0)
            self.state_0.clear_forces()
            self.solver.step(self.state_0, self.state_1, self.control, contacts, dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
        self.sim_time += self.physics_dt

    def reset(self) -> None:
        self.sim_time = 0.0
        self._apply_home_pose()
