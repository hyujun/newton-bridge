"""SimBridgeNode: rclpy node that connects NewtonWorld to ROS 2 topics/services.

Topics (standard types, stable contract):
    pub  /clock           rosgraph_msgs/Clock    (every physics step)
    pub  /joint_states    sensor_msgs/JointState (rate-limited in freerun,
                                                  per-step in sync +
                                                  watchdog republish when idle)
    sub  /joint_command   sensor_msgs/JointState (position targets, rad;
                                                  partial name match OK).
                                                  In sync mode, arrival of a
                                                  command triggers one physics
                                                  step.

Services:
    /sim/reset  std_srvs/Trigger   restore pack.home_pose, publish state
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Imu
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped, Vector3, WrenchStamped
from tf2_msgs.msg import TFMessage

from .world import NewtonWorld
from .ticks import CommandWatchdog, RenderTicker
from .sensors import (
    SensorBundle,
    build_sensors,
    contact_force_vec3,
    imu_readings,
)


class SimBridgeNode(Node):
    def __init__(self, world: NewtonWorld, sync_mode: str, viewer=None) -> None:
        super().__init__("newton_bridge")
        self.world = world
        self.sync_mode = sync_mode
        self.viewer = viewer
        self.pack = world.pack
        self.shutdown_requested = False
        self.ready_log_info: str | None = None
        self._ready_logged = False

        ros_cfg = self.pack["ros"]
        sim_cfg = self.pack["sim"]
        self._joint_names: list[str] = list(self.pack["joint_names"])
        self._publish_rate_hz: float = float(ros_cfg.get("publish_rate_hz", 100.0))
        self._pub_interval: float = 1.0 / self._publish_rate_hz

        # Render cadence decoupled from physics step rate. viewer_hz=0/None
        # means "render every physics step".
        viewer_hz = sim_cfg.get("viewer_hz", 60)
        self._render_ticker = RenderTicker(viewer_hz, world.physics_dt)

        # Watchdog used only in sync mode — armed lazily in run_sync().
        self._sync_timeout_s: float = float(ros_cfg.get("sync_timeout_ms", 100)) / 1000.0
        self._cmd_watchdog = CommandWatchdog(self._sync_timeout_s)

        # /tf config (Phase 4). Default ON per product decision D.
        self._publish_tf_enabled: bool = bool(ros_cfg.get("publish_tf", True))
        self._tf_root_frame: str = str(ros_cfg.get("tf_root_frame", "world"))
        # Empty list means "publish every body except the root frame itself".
        self._publish_frames: list[str] = list(ros_cfg.get("publish_frames", []) or [])

        # latest command (mutable; callback writes, main loop reads).
        # Any of positions/velocities/efforts may be None (= field was empty
        # on the incoming JointState and should be left untouched).
        self._latest_cmd: dict = {
            "names": None,
            "positions": None,
            "velocities": None,
            "efforts": None,
        }

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
        # Phase 6a: runtime gravity change via topic (Trigger can't carry vec).
        self.sub_gravity = self.create_subscription(
            Vector3, "/sim/set_gravity", self._on_set_gravity, qos
        )
        self.pub_tf = (
            self.create_publisher(TFMessage, "/tf", qos)
            if self._publish_tf_enabled
            else None
        )

        # Phase 5: sensors. Build after model is up; publishers are per-sensor.
        self.sensors: SensorBundle = build_sensors(self.pack, world.model)
        self._contact_pubs = {
            spec.label: self.create_publisher(WrenchStamped, spec.topic, qos)
            for spec in self.sensors.contact
        }
        self._imu_pubs = {
            spec.label: self.create_publisher(Imu, spec.topic, qos)
            for spec in self.sensors.imu
        }
        if not self.sensors.empty():
            self.get_logger().info(
                f"sensors: {len(self.sensors.contact)} contact, "
                f"{len(self.sensors.imu)} imu"
            )

        # /sim/reset is always available — useful in both modes.
        self.srv_reset = self.create_service(Trigger, "/sim/reset", self._on_reset)

        render_note = (
            f"render @ {viewer_hz:.0f}Hz" if viewer_hz else "render every step"
        )
        if self.sync_mode == "sync":
            self.get_logger().info(
                f"sync mode: /joint_command drives one step each; "
                f"watchdog republishes after {self._sync_timeout_s*1000:.0f}ms idle; "
                f"{render_note}"
            )
        else:
            self.get_logger().info(
                f"freerun mode: stepping @ {1.0/world.physics_dt:.0f}Hz, "
                f"publishing /joint_states @ {self._publish_rate_hz:.0f}Hz, "
                f"{render_note}"
            )

        self._last_pub_wall: float = 0.0

    # -- topic callbacks ----------------------------------------------------
    def _on_set_gravity(self, msg: Vector3) -> None:
        self.world.set_gravity((msg.x, msg.y, msg.z))
        self.get_logger().info(f"gravity -> ({msg.x}, {msg.y}, {msg.z})")

    def _on_cmd(self, msg: JointState) -> None:
        self._latest_cmd["names"] = list(msg.name)
        # Each field is only forwarded if the publisher populated it — empty
        # arrays mean "do not drive this channel this tick". Length mismatches
        # (other than empty) are passed through; set_joint_targets iterates
        # with zip so extras are truncated.
        n = len(msg.name)
        self._latest_cmd["positions"] = list(msg.position) if len(msg.position) == n else None
        self._latest_cmd["velocities"] = list(msg.velocity) if len(msg.velocity) == n else None
        self._latest_cmd["efforts"] = list(msg.effort) if len(msg.effort) == n else None

        if self.sync_mode == "sync":
            self._cmd_watchdog.note_command(time.monotonic())
            self._apply_latest_cmd()
            self.world.step()
            self._log_ready_once()
            self._publish_state(force=True)
            self._render_if_due()

    # -- service callbacks --------------------------------------------------
    def _on_reset(self, request, response):
        self.world.reset()
        self._latest_cmd["names"] = None
        self._latest_cmd["positions"] = None
        self._publish_state(force=True)
        self._render_if_due()
        response.success = True
        response.message = "reset to home_pose"
        return response

    def _log_ready_once(self) -> None:
        if self._ready_logged:
            return
        if self.ready_log_info is not None:
            self.get_logger().info(self.ready_log_info)
        self._ready_logged = True

    def _render_if_due(self) -> None:
        if self.viewer is None:
            return
        if self._render_ticker.tick():
            self._render_viewer()

    # -- helpers ------------------------------------------------------------
    def _apply_latest_cmd(self) -> None:
        names = self._latest_cmd["names"]
        if names is None:
            return
        if any(
            self._latest_cmd[k] is not None for k in ("positions", "velocities", "efforts")
        ):
            self.world.set_joint_targets(
                names,
                positions=self._latest_cmd["positions"],
                velocities=self._latest_cmd["velocities"],
                efforts=self._latest_cmd["efforts"],
            )
        # Consume: clear so we don't re-apply stale commands next tick.
        self._latest_cmd["names"] = None
        self._latest_cmd["positions"] = None
        self._latest_cmd["velocities"] = None
        self._latest_cmd["efforts"] = None

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
        qd = self.world.read_joint_velocities()
        eff = self.world.read_joint_efforts()
        now_stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = now_stamp
        msg.name = self._joint_names
        msg.position = [float(q[n]) for n in self._joint_names]
        msg.velocity = [float(qd[n]) for n in self._joint_names]
        # effort is the commanded control.joint_f readback; solver-applied
        # torque is not exposed on State in Newton 1.1.0.
        msg.effort = [float(eff[n]) for n in self._joint_names]
        self.pub_state.publish(msg)
        self._publish_clock()
        self._publish_tf(now_stamp)
        self._publish_sensors(now_stamp)

    def _publish_tf(self, stamp) -> None:
        if self.pub_tf is None:
            return
        poses = self.world.read_body_transforms()
        # Filter: default = every body except the root frame itself.
        # If publish_frames is set, publish only those names (still minus root).
        if self._publish_frames:
            names = [n for n in self._publish_frames if n in poses]
        else:
            names = [n for n in poses.keys() if n != self._tf_root_frame]
        if not names:
            return

        tfm = TFMessage()
        for name in names:
            (px, py, pz), (qx, qy, qz, qw) = poses[name]
            tr = TransformStamped()
            tr.header.stamp = stamp
            tr.header.frame_id = self._tf_root_frame
            tr.child_frame_id = name
            tr.transform.translation.x = px
            tr.transform.translation.y = py
            tr.transform.translation.z = pz
            tr.transform.rotation.x = qx
            tr.transform.rotation.y = qy
            tr.transform.rotation.z = qz
            tr.transform.rotation.w = qw
            tfm.transforms.append(tr)
        self.pub_tf.publish(tfm)

    def _publish_sensors(self, stamp) -> None:
        if self.sensors.empty():
            return
        contacts = self.world.last_contacts
        state = self.world.state_0

        for spec in self.sensors.contact:
            if contacts is None:
                continue
            spec.sensor.update(state, contacts)
            fx, fy, fz = contact_force_vec3(spec)
            msg = WrenchStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = spec.frame_id
            msg.wrench.force.x = fx
            msg.wrench.force.y = fy
            msg.wrench.force.z = fz
            # torque stays zero — SensorContact.total_force is vec3, not wrench
            self._contact_pubs[spec.label].publish(msg)

        for spec in self.sensors.imu:
            spec.sensor.update(state)
            r = imu_readings(spec)
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = spec.frame_id
            ax, ay, az = r["linear_acceleration"]
            gx, gy, gz = r["angular_velocity"]
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            # orientation unknown -> -1 on [0] signals the field is unset
            # (sensor_msgs/Imu convention)
            msg.orientation_covariance[0] = -1.0
            self._imu_pubs[spec.label].publish(msg)

    def _render_viewer(self) -> None:
        if self.viewer is None:
            return
        self.viewer.begin_frame(self.world.sim_time)
        self.viewer.log_state(self.world.state_0)
        self.viewer.end_frame()

    def request_shutdown(self) -> None:
        self.shutdown_requested = True

    # -- freerun main loop --------------------------------------------------
    def run_freerun(self, rate_mode: str) -> None:
        realtime = rate_mode != "max"
        next_wall = time.monotonic()
        while rclpy.ok() and not self.shutdown_requested:
            if self.viewer is not None and not self.viewer.is_running():
                self.get_logger().info("viewer window closed; shutting down")
                break
            paused = self.viewer is not None and self.viewer.is_paused()
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.shutdown_requested:
                break
            if not paused:
                self._apply_latest_cmd()
                self.world.step()
                self._log_ready_once()
                self._publish_state(force=False)
            self._render_if_due()
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

    # -- sync main loop -----------------------------------------------------
    def run_sync(self) -> None:
        """Externally driven loop: /joint_command triggers step via _on_cmd.

        The main loop only spins ROS callbacks and runs the idle watchdog,
        which republishes /joint_states (without stepping) when no command
        has arrived for sync_timeout_ms. Viewer window close / pause is
        honored here too so the process exits cleanly when the user closes
        the viewer.
        """
        # Publish initial state so late subscribers get at least one sample.
        self._publish_state(force=True)
        self._render_if_due()
        # Spin timeout chosen to divide the watchdog window ~5 ways so the
        # republish jitter stays below ~20% of sync_timeout_ms.
        spin_timeout = min(0.02, self._sync_timeout_s / 5.0)
        while rclpy.ok() and not self.shutdown_requested:
            if self.viewer is not None and not self.viewer.is_running():
                self.get_logger().info("viewer window closed; shutting down")
                break
            rclpy.spin_once(self, timeout_sec=spin_timeout)
            if self.shutdown_requested:
                break
            if self._cmd_watchdog.is_stale(time.monotonic()):
                # Stale: republish current state (no step, no render) so
                # downstream tools that polled for /joint_states keep seeing
                # fresh timestamps. Render stays tied to actual simulation
                # progress.
                self._publish_state(force=True)
