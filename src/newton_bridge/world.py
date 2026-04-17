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


class NewtonWorld:
    """Thin wrapper: build model, hold solver + double-buffered state."""

    def __init__(self, pack: dict, device: str = "cuda:0") -> None:
        self.pack = pack
        self.device = device
        self.physics_dt: float = 1.0 / float(pack["sim"]["physics_hz"])
        self.substeps: int = int(pack["sim"].get("substeps", 1))

        self._build_model()
        self._build_solver()
        self._resolve_joint_layout()
        self._apply_drive_gains()
        self._control_target_host = np.zeros(self.total_dof, dtype=np.float32)
        self.sim_time: float = 0.0

        self._apply_home_pose()

    # -- NEWTON API SURFACE -------------------------------------------------
    def _build_model(self) -> None:
        src_fmt = self.pack["robot"]["source"]
        src_path = str(self.pack["_pack_dir"] / self.pack["robot"]["source_rel"])
        base_pos = self.pack["robot"].get("base_position", [0.0, 0.0, 0.0])

        builder = newton.ModelBuilder()
        xform = wp.transform(base_pos, wp.quat_identity())
        if src_fmt == "urdf":
            builder.add_urdf(src_path, xform=xform, floating=False)
        elif src_fmt == "mjcf":
            builder.add_mjcf(src_path, xform=xform, floating=False)
        else:
            raise ValueError(f"unknown robot.source: {src_fmt!r} (expected urdf|mjcf)")

        if self.pack["sim"].get("ground_plane", True):
            builder.add_ground_plane()

        self.model = builder.finalize(device=self.device)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

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
    def _resolve_joint_layout(self) -> None:
        """Map each configured joint name -> (q_start, q_count) into joint_q.

        Uses model.joint_name + model.joint_q_start if present, otherwise
        falls back to a revolute-only assumption (1 DOF per joint in order).
        """
        configured: list[str] = list(self.pack["joint_names"])
        joint_names: list[str] = list(getattr(self.model, "joint_name", []))

        layout: dict[str, tuple[int, int]] = {}

        if joint_names:
            q_start = np.asarray(
                getattr(self.model, "joint_q_start", np.arange(len(joint_names)))
            )
            for i, name in enumerate(joint_names):
                start = int(q_start[i])
                end = int(q_start[i + 1]) if i + 1 < len(q_start) else int(
                    self.state_0.joint_q.shape[0]
                )
                layout[name] = (start, end - start)
        else:
            for i, name in enumerate(configured):
                layout[name] = (i, 1)

        missing = [n for n in configured if n not in layout]
        if missing:
            raise RuntimeError(
                f"joints in robot.yaml not found in Newton model: {missing}\n"
                f"available joints: {list(layout.keys())}"
            )

        self.joint_layout = {n: layout[n] for n in configured}
        self.total_dof = int(self.state_0.joint_q.shape[0])

    # -- NEWTON API SURFACE -------------------------------------------------
    def _apply_drive_gains(self) -> None:
        """Set per-DOF PD gains from pack.drive.{stiffness,damping}."""
        ke = float(self.pack["drive"]["stiffness"])
        kd = float(self.pack["drive"]["damping"])

        if hasattr(self.model, "joint_target_ke") and self.model.joint_target_ke is not None:
            np_ke = np.full(self.model.joint_target_ke.shape, ke, dtype=np.float32)
            np_kd = np.full(self.model.joint_target_kd.shape, kd, dtype=np.float32)
            self.model.joint_target_ke.assign(np_ke)
            self.model.joint_target_kd.assign(np_kd)
        # else: solver (e.g. MuJoCo) took gains from the MJCF actuator block.

    def _apply_home_pose(self) -> None:
        home = self.pack.get("home_pose", {}) or {}
        if not home:
            return
        q = self.state_0.joint_q.numpy().copy()
        for name, val in home.items():
            if name not in self.joint_layout:
                continue
            start, count = self.joint_layout[name]
            q[start] = float(val)
        self.state_0.joint_q.assign(q)

        # mirror into state_1 so a double-buffer swap doesn't undo the reset
        q1 = self.state_1.joint_q.numpy().copy()
        q1[:] = q
        self.state_1.joint_q.assign(q1)
        self.state_0.joint_qd.zero_()
        self.state_1.joint_qd.zero_()

    # ----------------------------------------------------------------------
    def read_joint_positions(self) -> dict[str, float]:
        q = self.state_0.joint_q.numpy()
        return {n: float(q[s]) for n, (s, _) in self.joint_layout.items()}

    def set_joint_targets(self, names: Iterable[str], positions: Iterable[float]) -> None:
        """Write position targets into control. Unknown names are silently ignored."""
        for name, pos in zip(names, positions):
            slot = self.joint_layout.get(name)
            if slot is None:
                continue
            start, _ = slot
            self._control_target_host[start] = float(pos)
        # -- NEWTON API SURFACE ---------------------------------------------
        if hasattr(self.control, "joint_target") and self.control.joint_target is not None:
            self.control.joint_target.assign(self._control_target_host)
        elif hasattr(self.control, "joint_act") and self.control.joint_act is not None:
            self.control.joint_act.assign(self._control_target_host)

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
        self._control_target_host[:] = 0.0
        home = self.pack.get("home_pose", {}) or {}
        self.set_joint_targets(home.keys(), home.values())
