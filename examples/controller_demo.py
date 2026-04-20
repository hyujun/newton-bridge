#!/usr/bin/env python3
"""Minimal external controller for end-to-end verification.

Freerun mode:   pub /joint_command = home + sine(t); observe /joint_states.
Sync mode:      pub /joint_command N times, each publish advances one step on
                the sim side and the reply arrives on /joint_states.

Run on the HOST (not inside the container).
Example:
    source /opt/ros/jazzy/setup.bash
    python3 examples/controller_demo.py --mode freerun --robot ur5e
    python3 examples/controller_demo.py --mode sync --robot franka --steps 200
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def load_home(repo_root: Path, robot: str) -> tuple[list[str], list[float]]:
    cfg = yaml.safe_load((repo_root / "robots" / robot / "robot.yaml").read_text())
    names = list(cfg["joint_names"])
    home = cfg.get("home_pose", {}) or {}
    positions = [float(home.get(n, 0.0)) for n in names]
    return names, positions


class Controller(Node):
    def __init__(self, names: list[str], home: list[float]) -> None:
        super().__init__("controller_demo")
        self.names = names
        self.home = home
        self.latest: JointState | None = None
        self.pub = self.create_publisher(JointState, "/joint_command", 10)
        self.sub = self.create_subscription(
            JointState, "/joint_states", self._on_state, 10
        )

    def _on_state(self, msg: JointState) -> None:
        self.latest = msg

    def make_target(self, t: float, amp: float = 0.2) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        # per-joint phase shift so the motion is visible.
        msg.position = [h + amp * math.sin(t + i) for i, h in enumerate(self.home)]
        return msg


def run_freerun(node: Controller, duration: float) -> int:
    print(f"[demo] freerun: publishing /joint_command for {duration}s")
    t0 = time.monotonic()
    period = 1.0 / 50.0
    next_t = t0
    while rclpy.ok() and time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.0)
        t = time.monotonic() - t0
        node.pub.publish(node.make_target(t))
        next_t += period
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
    if node.latest:
        print(f"[demo] last /joint_states positions: {list(node.latest.position)}")
    return 0


def run_sync(node: Controller, steps: int) -> int:
    """Each /joint_command publish triggers exactly one step on the sim side.

    We wait for /joint_states between publishes to confirm the step landed —
    otherwise a burst of commands could arrive before sim consumes them and
    the "one publish = one step" invariant would be observed only approximately.
    """
    print(f"[demo] sync: {steps} /joint_command publishes")
    # Wait for the sim's startup /joint_states so we have a baseline stamp.
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest is not None:
            break
    if node.latest is None:
        print("[demo] no /joint_states received (is sim running?)", file=sys.stderr)
        return 1

    for i in range(steps):
        t = i * 0.01
        prev_stamp = node.latest.header.stamp if node.latest is not None else None
        node.pub.publish(node.make_target(t))
        # Wait for a fresh /joint_states with a newer stamp (= step happened).
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.latest is not None and node.latest.header.stamp != prev_stamp:
                break
        else:
            print(f"[demo] step {i} timed out waiting for /joint_states", file=sys.stderr)
            return 2
    print(f"[demo] {steps} publishes done")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["freerun", "sync"], default="freerun")
    ap.add_argument("--robot", default="ur5e")
    ap.add_argument("--duration", type=float, default=5.0, help="freerun seconds")
    ap.add_argument("--steps", type=int, default=200, help="sync step count")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    names, home = load_home(repo_root, args.robot)

    rclpy.init()
    node = Controller(names, home)
    try:
        if args.mode == "freerun":
            return run_freerun(node, args.duration)
        return run_sync(node, args.steps)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
