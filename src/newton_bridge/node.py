"""SimBridgeNode: rclpy node that connects NewtonWorld to ROS 2 topics/services.

Topics (standard types, stable contract):
    pub  /clock             rosgraph_msgs/Clock    (every physics step)
    pub  /joint_states      sensor_msgs/JointState (every physics step in both
                                                    modes; sync also republishes
                                                    on watchdog idle)
    pub  /sim/diagnostics   diagnostic_msgs/DiagnosticArray  (1Hz telemetry)
    sub  /joint_command     sensor_msgs/JointState (position targets, rad;
                                                    partial name match OK).
                                                    In sync mode, arrival of a
                                                    command triggers one physics
                                                    step.

Services:
    /sim/reset  std_srvs/Trigger   restore pack.home_pose, publish state

Threading:
    Physics + ROS callbacks run on the main thread. Rendering runs on the
    `ViewerThread` (see viewer_thread.py); the main thread publishes a state
    snapshot via `StateSnapshot.publish()` after every step. This keeps a
    slow viewer from stalling physics and lets the viewer keep animating at
    viewer_hz wall-clock when physics is idle.
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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .world import NewtonWorld
from .ticks import CommandWatchdog
from .snapshot import StateSnapshot
from .viewer_thread import ViewerThread
from .telemetry import (
    STATE_STALL,
    StatusLogger,
    TelemetryRegistry,
)
from .sensors import (
    SensorBundle,
    build_sensors,
    contact_force_vec3,
    imu_readings,
)


class SimBridgeNode(Node):
    def __init__(
        self,
        world: NewtonWorld,
        sync_mode: str,
        *,
        snapshot: StateSnapshot | None = None,
        viewer_thread: ViewerThread | None = None,
        telemetry: TelemetryRegistry | None = None,
        status_logger: StatusLogger | None = None,
    ) -> None:
        super().__init__("newton_bridge")
        self.world = world
        self.sync_mode = sync_mode
        self.snapshot = snapshot
        self.viewer_thread = viewer_thread
        self.pack = world.pack
        self.shutdown_requested = False
        self.ready_log_info: str | None = None
        self._ready_logged = False

        ros_cfg = self.pack["ros"]
        sim_cfg = self.pack["sim"]
        self._joint_names: list[str] = list(self.pack["joint_names"])

        # Telemetry — created here if caller didn't pass one (eases tests).
        self.telemetry = telemetry or TelemetryRegistry(
            physics_dt=world.physics_dt,
            sync_mode=(sync_mode == "sync"),
            sync_timeout_s=float(ros_cfg.get("sync_timeout_ms", 100)) / 1000.0,
        )
        self.status_logger = status_logger or StatusLogger(period_s=0)

        # Watchdog used only in sync mode — armed lazily in run_sync().
        self._sync_timeout_s: float = float(ros_cfg.get("sync_timeout_ms", 100)) / 1000.0
        self._cmd_watchdog = CommandWatchdog(self._sync_timeout_s)

        # /tf config (Phase 4). Default ON per product decision D.
        self._publish_tf_enabled: bool = bool(ros_cfg.get("publish_tf", True))
        self._tf_root_frame: str = str(ros_cfg.get("tf_root_frame", "world"))
        self._publish_frames: list[str] = list(ros_cfg.get("publish_frames", []) or [])

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
        self.pub_diagnostics = self.create_publisher(
            DiagnosticArray, "/sim/diagnostics", qos
        )
        self.sub_cmd = self.create_subscription(
            JointState, ros_cfg["joint_command_topic"], self._on_cmd, qos
        )
        self.sub_gravity = self.create_subscription(
            Vector3, "/sim/set_gravity", self._on_set_gravity, qos
        )
        self.pub_tf = (
            self.create_publisher(TFMessage, "/tf", qos)
            if self._publish_tf_enabled
            else None
        )

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

        self.srv_reset = self.create_service(Trigger, "/sim/reset", self._on_reset)

        viewer_hz = sim_cfg.get("viewer_hz", 60)
        render_note = (
            f"render @ {viewer_hz:.0f}Hz wall (viewer thread)"
            if viewer_hz
            else "render every tick (viewer thread)"
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
                f"publishing /joint_states per step, "
                f"{render_note}"
            )

        self._last_diag_wall: float = 0.0

    # -- topic callbacks ----------------------------------------------------
    def _on_set_gravity(self, msg: Vector3) -> None:
        self.world.set_gravity((msg.x, msg.y, msg.z))
        self.get_logger().info(f"gravity -> ({msg.x}, {msg.y}, {msg.z})")

    def _on_cmd(self, msg: JointState) -> None:
        now = time.monotonic()
        self.telemetry.note_cmd(now)
        self._latest_cmd["names"] = list(msg.name)
        n = len(msg.name)
        self._latest_cmd["positions"] = list(msg.position) if len(msg.position) == n else None
        self._latest_cmd["velocities"] = list(msg.velocity) if len(msg.velocity) == n else None
        self._latest_cmd["efforts"] = list(msg.effort) if len(msg.effort) == n else None

        if self.sync_mode == "sync":
            self._cmd_watchdog.note_command(now)
            self._apply_latest_cmd()
            self._step_and_publish()
            self._log_ready_once()

    # -- service callbacks --------------------------------------------------
    def _on_reset(self, request, response):
        self._do_reset()
        response.success = True
        response.message = "reset to home_pose"
        return response

    def _do_reset(self) -> None:
        """Internal reset path; reused by ViewerThread reset queue."""
        self.world.reset()
        self._latest_cmd["names"] = None
        self._latest_cmd["positions"] = None
        self._latest_cmd["velocities"] = None
        self._latest_cmd["efforts"] = None
        self._publish_state()
        self._publish_snapshot()

    def _log_ready_once(self) -> None:
        if self._ready_logged:
            return
        if self.ready_log_info is not None:
            self.get_logger().info(self.ready_log_info)
        self._ready_logged = True

    # -- helpers ------------------------------------------------------------
    def _step_and_publish(self) -> None:
        """One physics step + the bookkeeping that goes with it."""
        t0 = time.perf_counter()
        self.world.step()
        t1 = time.perf_counter()
        now = time.monotonic()
        self.telemetry.note_step(now)
        self.telemetry.note_phase("step", t1 - t0, now)
        self._publish_state()
        t2 = time.perf_counter()
        self.telemetry.note_phase("publish_state", t2 - t1, time.monotonic())
        self._publish_snapshot()
        t3 = time.perf_counter()
        self.telemetry.note_phase("publish_snapshot", t3 - t2, time.monotonic())

    def _publish_snapshot(self) -> None:
        if self.snapshot is None:
            return
        now = time.monotonic()
        telemetry_dict = self.telemetry.snapshot(now, self.world.sim_time).as_dict()
        self.snapshot.publish(
            self.world.state_0,
            sim_time=self.world.sim_time,
            telemetry=telemetry_dict,
            now=now,
        )

    def _apply_latest_cmd(self) -> None:
        names = self._latest_cmd["names"]
        if names is None:
            return
        t0 = time.perf_counter()
        if any(
            self._latest_cmd[k] is not None for k in ("positions", "velocities", "efforts")
        ):
            self.world.set_joint_targets(
                names,
                positions=self._latest_cmd["positions"],
                velocities=self._latest_cmd["velocities"],
                efforts=self._latest_cmd["efforts"],
            )
        self._latest_cmd["names"] = None
        self._latest_cmd["positions"] = None
        self._latest_cmd["velocities"] = None
        self._latest_cmd["efforts"] = None
        self.telemetry.note_phase(
            "apply_cmd", time.perf_counter() - t0, time.monotonic()
        )

    def _publish_clock(self) -> None:
        sec = int(self.world.sim_time)
        nsec = int((self.world.sim_time - sec) * 1_000_000_000)
        msg = Clock()
        msg.clock.sec = sec
        msg.clock.nanosec = nsec
        self.pub_clock.publish(msg)

    def _publish_state(self) -> None:
        now_wall = time.monotonic()

        q = self.world.read_joint_positions()
        qd = self.world.read_joint_velocities()
        eff = self.world.read_joint_efforts()
        now_stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = now_stamp
        msg.name = self._joint_names
        msg.position = [float(q[n]) for n in self._joint_names]
        msg.velocity = [float(qd[n]) for n in self._joint_names]
        msg.effort = [float(eff[n]) for n in self._joint_names]
        self.pub_state.publish(msg)
        self.telemetry.note_pub(now_wall)
        self._publish_clock()
        self._publish_tf(now_stamp)
        self._publish_sensors(now_stamp)

    def _publish_tf(self, stamp) -> None:
        if self.pub_tf is None:
            return
        poses = self.world.read_body_transforms()
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
            msg.orientation_covariance[0] = -1.0
            self._imu_pubs[spec.label].publish(msg)

    def _publish_diagnostics(self, now: float) -> None:
        """1Hz /sim/diagnostics with the same fields as the status line."""
        if now < self._last_diag_wall + 1.0:
            return
        self._last_diag_wall = now
        snap = self.telemetry.snapshot(now, self.world.sim_time)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "newton_bridge"
        status.hardware_id = "newton"
        if snap.state == STATE_STALL:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = snap.state
        for k, v in snap.as_dict().items():
            kv = KeyValue()
            kv.key = str(k)
            kv.value = "" if v is None else f"{v}"
            status.values.append(kv)
        msg.status.append(status)
        self.pub_diagnostics.publish(msg)

    def _periodic_telemetry(self, now: float) -> None:
        """Status line + /sim/diagnostics fanout, called once per main-loop tick."""
        self.status_logger.maybe_log(now, self.telemetry, self.world.sim_time)
        self._publish_diagnostics(now)

    def _drain_viewer_signals(self) -> bool:
        """Honor ESC/window-close + UI Reset from the viewer thread.

        Returns True if a single-step pulse was consumed (caller may use this
        in sync mode while paused). Window close flips shutdown_requested.
        """
        vt = self.viewer_thread
        if vt is None:
            return False
        if vt.closed_externally():
            self.get_logger().info("viewer window closed; shutting down")
            self.shutdown_requested = True
            return False
        if vt.consume_reset_request():
            self.get_logger().info("viewer Reset clicked; resetting world")
            self._do_reset()
        return vt.consume_step_request()

    def request_shutdown(self) -> None:
        self.shutdown_requested = True

    # -- freerun main loop --------------------------------------------------
    def run_freerun(self, rate_mode: str) -> None:
        realtime = rate_mode != "max"
        next_wall = time.monotonic()
        # Seed the snapshot so the viewer thread has something to render
        # before the first step lands.
        self._publish_snapshot()
        while rclpy.ok() and not self.shutdown_requested:
            t_spin0 = time.perf_counter()
            step_pulse = self._drain_viewer_signals()
            if self.shutdown_requested:
                break
            paused = self.viewer_thread is not None and self.viewer_thread.is_paused()
            rclpy.spin_once(self, timeout_sec=0.0)
            self.telemetry.note_phase(
                "spin", time.perf_counter() - t_spin0, time.monotonic()
            )
            if self.shutdown_requested:
                break
            if not paused:
                self._apply_latest_cmd()
                if self.shutdown_requested:
                    break
                self._step_and_publish()
                self._log_ready_once()
            elif step_pulse:
                # Paused + '.' single-step → advance exactly one frame.
                self._apply_latest_cmd()
                if self.shutdown_requested:
                    break
                self._step_and_publish()
                self._log_ready_once()
            self._periodic_telemetry(time.monotonic())
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

        The main loop spins ROS callbacks and runs the idle watchdog (which
        republishes /joint_states without stepping when no command has
        arrived for sync_timeout_ms). Viewer signals (close / reset / SPACE
        pause / `.` step) are also drained here.
        """
        # Publish initial state + snapshot so late subscribers and the viewer
        # thread both see something immediately.
        self._publish_state()
        self._publish_snapshot()
        # Spin timeout: divide watchdog window ~5 ways for republish jitter
        # control. With viewer in its own thread, we no longer cap by the
        # render period (the viewer thread paces itself).
        spin_timeout = min(0.02, self._sync_timeout_s / 5.0)
        while rclpy.ok() and not self.shutdown_requested:
            step_pulse = self._drain_viewer_signals()
            if self.shutdown_requested:
                break
            paused = self.viewer_thread is not None and self.viewer_thread.is_paused()
            rclpy.spin_once(self, timeout_sec=spin_timeout)
            if self.shutdown_requested:
                break
            if paused and step_pulse:
                # Paused + '.' single-step → advance one frame even without a cmd.
                if self.shutdown_requested:
                    break
                self._step_and_publish()
                self._log_ready_once()
            if self.shutdown_requested:
                break
            if self._cmd_watchdog.is_stale(time.monotonic()):
                self._publish_state()
            self._periodic_telemetry(time.monotonic())
