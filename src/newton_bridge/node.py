"""SimBridgeNode: rclpy node that connects NewtonWorld to ROS 2 topics/services.

Topics (standard types, stable contract):
    pub  /clock           rosgraph_msgs/Clock    (every physics step)
    pub  /joint_states    sensor_msgs/JointState (rate-limited in freerun,
                                                  per-step in handshake)
    sub  /joint_command   sensor_msgs/JointState (position targets, rad;
                                                  partial name match OK)

Services (handshake mode only):
    /sim/step   std_srvs/Trigger   advance one physics step
    /sim/reset  std_srvs/Trigger   restore pack.home_pose, publish state
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

from .world import NewtonWorld


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
                    next_wall = time.monotonic()
            elif paused:
                time.sleep(1.0 / 60.0)
                next_wall = time.monotonic()
