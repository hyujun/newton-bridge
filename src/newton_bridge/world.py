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
        self._control_target_host = np.zeros(self.total_dof, dtype=np.float32)
        self.sim_time: float = 0.0

        # builder.joint_q / joint_target_pos already put us at home, but
        # mirror into _control_target_host + re-assert for reset() parity.
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
        # This matches the panda_hydro example pattern.
        self._configure_builder_drive(builder)

        self.model = builder.finalize(device=self.device)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

    # -- NEWTON API SURFACE -------------------------------------------------
    def _configure_builder_drive(self, builder) -> None:
        """Set per-DOF PD gains, POSITION mode, and home pose on the builder."""
        ke = float(self.pack["drive"]["stiffness"])
        kd = float(self.pack["drive"]["damping"])
        pos_mode = int(newton.JointTargetMode.POSITION)
        dof_count = int(builder.joint_dof_count)

        for i in range(dof_count):
            builder.joint_target_ke[i] = ke
            builder.joint_target_kd[i] = kd
            builder.joint_target_mode[i] = pos_mode

        # Synthetic layout for home pose: pack's joint_names[i] -> DOF index i.
        # This matches the order ArticulationView.joint_dof_names will expose
        # after finalize, so resetting is consistent.
        home = self.pack.get("home_pose", {}) or {}
        if not home:
            return
        configured = list(self.pack["joint_names"])
        for i, name in enumerate(configured):
            if i >= dof_count or name not in home:
                continue
            val = float(home[name])
            builder.joint_q[i] = val
            builder.joint_target_pos[i] = val

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

        The pattern is an fnmatch glob (not regex) that matches against
        `model.articulation_label[*]`. Default `*` picks up every
        articulation in the model (fine for single-robot packs). A pack can
        override via `articulation_pattern:` to select a specific
        articulation when multiple are loaded.
        """
        pattern = str(self.pack.get("articulation_pattern", "*"))
        self.view = ArticulationView(self.model, pattern=pattern)

        dof_names: list[str] = list(self.view.joint_dof_names)
        configured: list[str] = list(self.pack["joint_names"])

        # Pack's joint_names is the ROS contract — a subset of what the model
        # actually simulates. Extras (e.g. unexposed gripper fingers) are fine
        # and remain at home_pose / 0. Missing names are fatal.
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

        self.joint_dof_names: list[str] = dof_names  # full view order
        self.exposed_joint_names: list[str] = configured  # ROS contract subset
        self.total_dof: int = int(self.view.joint_dof_count)
        # dof-name -> flat index over the full view (covers both exposed and
        # unexposed joints, so set_joint_targets can address any of them).
        self._dof_index: dict[str, int] = {n: i for i, n in enumerate(dof_names)}

    def _apply_home_pose(self) -> None:
        home = self.pack.get("home_pose", {}) or {}
        if not home:
            return

        # Build full-dof array in the view's order; unspecified joints stay 0.
        q = np.zeros(self.total_dof, dtype=np.float32)
        for name, val in home.items():
            i = self._dof_index.get(name)
            if i is None:
                continue
            q[i] = float(val)

        # ArticulationView.set_dof_positions expects shape
        # (n_worlds, n_arts_per_world, dof_count). For our single-art case
        # that's (1, 1, total_dof).
        shaped = q.reshape(1, 1, self.total_dof)
        self.view.set_dof_positions(self.state_0, shaped)
        self.view.set_dof_positions(self.state_1, shaped)
        self.state_0.joint_qd.zero_()
        self.state_1.joint_qd.zero_()

        # Mirror home into the PD setpoint so reset() restores drive targets
        # too — not just q.
        self._control_target_host[:] = q
        self.control.joint_target_pos.assign(self._control_target_host)

    # ----------------------------------------------------------------------
    def read_joint_positions(self) -> dict[str, float]:
        """Return {pack-exposed joint name: position}. Extras are hidden."""
        # ArticulationView returns wp.array of shape
        # (n_worlds, n_arts_per_world, dof_count). Single-art pack: flatten.
        q = self.view.get_dof_positions(self.state_0).numpy().reshape(-1)
        return {n: float(q[self._dof_index[n]]) for n in self.exposed_joint_names}

    def set_joint_targets(self, names: Iterable[str], positions: Iterable[float]) -> None:
        """Write position targets into control. Unknown names are silently ignored."""
        for name, pos in zip(names, positions):
            i = self._dof_index.get(name)
            if i is None:
                continue
            self._control_target_host[i] = float(pos)
        # -- NEWTON API SURFACE ---------------------------------------------
        # Newton 1.1.0: position setpoints live on control.joint_target_pos.
        # joint_act is feedforward (additive), not a PD setpoint.
        self.control.joint_target_pos.assign(self._control_target_host)

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
