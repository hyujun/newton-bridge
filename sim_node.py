#!/usr/bin/env python3
"""newton-bridge sim_node: standalone Newton physics + ROS 2 Jazzy bridge.

Robot-agnostic entry point. Reads a robot pack from ROBOT_PACK (container path,
default /workspace/robots/ur5e) and exposes its joints on ROS 2 topics.

Sync modes (SYNC_MODE env):
    freerun    Sim owns its own step loop, paced by FREERUN_RATE (realtime|max).
               /joint_states publishes at pack.ros.publish_rate_hz.
               /joint_command (latest-wins) feeds drive targets.
    handshake  Sim does NOT auto-step. An external controller advances the
               world by calling /sim/step. /joint_states publishes once per
               step. /sim/reset returns the sim to pack.home_pose.

Topics (standard types, stable contract):
    pub  /clock           rosgraph_msgs/Clock    (every physics step)
    pub  /joint_states    sensor_msgs/JointState (rate-limited in freerun,
                                                  per-step in handshake)
    sub  /joint_command   sensor_msgs/JointState (position targets, rad;
                                                  partial name match OK)

Services (handshake mode only):
    /sim/step   std_srvs/Trigger   advance one physics step
    /sim/reset  std_srvs/Trigger   restore pack.home_pose, publish state

IMPORTANT: Newton's Python API is still stabilizing (~1.x). The sections marked
`# --- NEWTON API SURFACE ---` are the places to adjust if a future release
moves things around. The ROS 2 side is stable.
"""

from __future__ import annotations

import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml

# -- Newton / Warp -----------------------------------------------------------
import warp as wp
import newton

# -- ROS 2 -------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger


# ============================================================================
# Robot pack loading
# ============================================================================

def load_pack(pack_dir: Path) -> dict:
    cfg_path = pack_dir / "robot.yaml"
    if not cfg_path.is_file():
        raise FileNotFoundError(f"robot.yaml not found under {pack_dir}")
    with cfg_path.open() as fh:
        cfg = yaml.safe_load(fh)
    cfg["_pack_dir"] = pack_dir
    return cfg


# ============================================================================
# Newton Model bootstrap
# ============================================================================

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

        # Optional ground plane for visual grounding.
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
        """Map each configured joint name -> (q_start, q_count) index into joint_q.

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
            # Fallback: assume order matches and each is 1-DOF revolute.
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
            # (count > 1 multi-DOF joints would need per-component handling.)
        self.state_0.joint_q.assign(q)

        # mirror into state_1 so a double-buffer swap doesn't undo the reset
        q1 = self.state_1.joint_q.numpy().copy()
        q1[:] = q
        self.state_1.joint_q.assign(q1)
        # Zero velocities.
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
        # pre-load targets from home pose so the drive doesn't yank to zero.
        home = self.pack.get("home_pose", {}) or {}
        self.set_joint_targets(home.keys(), home.values())


# ============================================================================
# ROS 2 side
# ============================================================================

class SimBridgeNode(Node):
    def __init__(self, world: NewtonWorld, sync_mode: str, viewer=None) -> None:
        super().__init__("newton_bridge")
        self.world = world
        self.sync_mode = sync_mode
        self.viewer = viewer
        self.pack = world.pack

        ros_cfg = self.pack["ros"]
        self._joint_names: list[str] = list(self.pack["joint_names"])
        self._publish_rate_hz: float = float(ros_cfg.get("publish_rate_hz", 100.0))
        self._pub_interval: float = 1.0 / self._publish_rate_hz

        # latest command (mutable; callback writes, main loop reads)
        self._latest_cmd: dict = {"names": None, "positions": None}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_clock = self.create_publisher(Clock, "/clock", qos)
        self.pub_state = self.create_publisher(
            JointState, ros_cfg["joint_states_topic"], qos
        )
        self.sub_cmd = self.create_subscription(
            JointState, ros_cfg["joint_command_topic"], self._on_cmd, qos
        )

        if self.sync_mode == "handshake":
            self.srv_step = self.create_service(Trigger, "/sim/step", self._on_step)
            self.srv_reset = self.create_service(Trigger, "/sim/reset", self._on_reset)
            self.get_logger().info(
                "handshake mode: call /sim/step to advance, /sim/reset to home"
            )
        else:
            self.get_logger().info(
                f"freerun mode: stepping @ {1.0/world.physics_dt:.0f}Hz, "
                f"publishing /joint_states @ {self._publish_rate_hz:.0f}Hz"
            )

        self._last_pub_wall: float = 0.0

    # -- topic callbacks ----------------------------------------------------
    def _on_cmd(self, msg: JointState) -> None:
        self._latest_cmd["names"] = list(msg.name)
        self._latest_cmd["positions"] = list(msg.position)

    # -- service callbacks (handshake) --------------------------------------
    def _on_step(self, request, response):
        self._apply_latest_cmd()
        self.world.step()
        self._publish_state(force=True)
        self._render_viewer()
        response.success = True
        response.message = f"sim_time={self.world.sim_time:.6f}"
        return response

    def _on_reset(self, request, response):
        self.world.reset()
        self._latest_cmd["names"] = None
        self._latest_cmd["positions"] = None
        self._publish_state(force=True)
        self._render_viewer()
        response.success = True
        response.message = "reset to home_pose"
        return response

    # -- helpers ------------------------------------------------------------
    def _apply_latest_cmd(self) -> None:
        positions = self._latest_cmd["positions"]
        if positions is None:
            return
        self.world.set_joint_targets(self._latest_cmd["names"], positions)
        self._latest_cmd["positions"] = None

    def _publish_clock(self) -> None:
        sec = int(self.world.sim_time)
        nsec = int((self.world.sim_time - sec) * 1_000_000_000)
        msg = Clock()
        msg.clock.sec = sec
        msg.clock.nanosec = nsec
        self.pub_clock.publish(msg)

    def _publish_state(self, force: bool = False) -> None:
        now_wall = time.monotonic()
        if not force and now_wall < self._last_pub_wall + self._pub_interval:
            return
        self._last_pub_wall = now_wall

        q = self.world.read_joint_positions()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [float(q[n]) for n in self._joint_names]
        self.pub_state.publish(msg)
        self._publish_clock()

    def _render_viewer(self) -> None:
        if self.viewer is None:
            return
        self.viewer.begin_frame(self.world.sim_time)
        self.viewer.log_state(self.world.state_0)
        self.viewer.end_frame()

    # -- freerun main loop --------------------------------------------------
    def run_freerun(self, rate_mode: str) -> None:
        realtime = rate_mode != "max"
        next_wall = time.monotonic()
        while rclpy.ok():
            if self.viewer is not None and not self.viewer.is_running():
                self.get_logger().info("viewer window closed; shutting down")
                break
            paused = self.viewer is not None and self.viewer.is_paused()
            rclpy.spin_once(self, timeout_sec=0.0)
            if not paused:
                self._apply_latest_cmd()
                self.world.step()
                self._publish_state(force=False)
            self._render_viewer()
            if realtime and not paused:
                next_wall += self.world.physics_dt
                sleep_for = next_wall - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    # fell behind; reset pacing reference
                    next_wall = time.monotonic()
            elif paused:
                # viewer pumps events in end_frame; throttle spin so we don't busy-loop
                time.sleep(1.0 / 60.0)
                next_wall = time.monotonic()


# ============================================================================
# main
# ============================================================================

def _resolve_pack_dir() -> Path:
    raw = os.environ.get("ROBOT_PACK", "/workspace/robots/ur5e")
    p = Path(raw)
    if not p.is_dir():
        raise FileNotFoundError(f"ROBOT_PACK not a directory: {p}")
    return p


def _env_truthy(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}


def _build_viewer(world: "NewtonWorld"):
    """Construct newton.viewer.ViewerGL and bind it to the model.

    Imported lazily so headless runs don't pay the GL import cost.
    """
    from newton.viewer import ViewerGL  # noqa: WPS433 (intentional local import)

    width = int(os.environ.get("VIEWER_WIDTH", "1280"))
    height = int(os.environ.get("VIEWER_HEIGHT", "720"))
    viewer = ViewerGL(width=width, height=height, vsync=False)
    viewer.set_model(world.model)
    return viewer


def main() -> int:
    pack_dir = _resolve_pack_dir()
    sync_mode = os.environ.get("SYNC_MODE", "freerun").lower()
    rate_mode = os.environ.get("FREERUN_RATE", "realtime").lower()
    if sync_mode not in {"freerun", "handshake"}:
        print(f"[sim_node] invalid SYNC_MODE={sync_mode!r}", file=sys.stderr)
        return 2

    print(f"[sim_node] loading pack: {pack_dir}", flush=True)
    pack = load_pack(pack_dir)

    wp.init()
    print(f"[sim_node] Warp CUDA devices: {wp.get_cuda_devices()}", flush=True)

    world = NewtonWorld(pack)
    print(
        f"[sim_node] Newton world ready: dof={world.total_dof}, "
        f"joints={len(world.joint_layout)}, dt={world.physics_dt:.6f}s, "
        f"solver={pack['sim'].get('solver', 'xpbd')}",
        flush=True,
    )

    viewer = None
    if _env_truthy("ENABLE_VIEWER"):
        try:
            viewer = _build_viewer(world)
            print("[sim_node] GL viewer enabled (close window to stop sim)", flush=True)
        except Exception as exc:  # noqa: BLE001 — surface viewer init failures clearly
            print(
                f"[sim_node] ENABLE_VIEWER=1 but viewer init failed: {exc!r}\n"
                f"[sim_node] continuing headless. Check DISPLAY + xhost + nvidia GL.",
                file=sys.stderr,
                flush=True,
            )
            viewer = None
        if viewer is not None and sync_mode == "handshake":
            print(
                "[sim_node] note: handshake mode renders only on /sim/step or /sim/reset; "
                "the window will appear frozen until a controller calls those services.",
                flush=True,
            )

    rclpy.init(args=None)
    node = SimBridgeNode(world, sync_mode, viewer=viewer)

    # graceful shutdown on SIGINT/SIGTERM (docker stop, Ctrl-C)
    def _sigint(*_):
        node.get_logger().warning("shutdown signal received")
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        if sync_mode == "handshake":
            rclpy.spin(node)
        else:
            node.run_freerun(rate_mode)
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception:  # noqa: BLE001
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
