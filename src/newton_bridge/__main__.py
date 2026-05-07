"""`python -m newton_bridge` entry point.

Reads env:
    ROBOT_PACK      container path to robots/<name>/ (default /workspace/robots/ur5e)
    SYNC_MODE       freerun | sync (default freerun). Legacy "handshake" is
                    accepted with a deprecation warning and treated as "sync".
    FREERUN_RATE    realtime | max (freerun only, default realtime)
    VIEWER          rerun | gl | usd | file | null | none (default rerun)
    STATUS_LOG_HZ   1Hz status line cadence (default 1.0; 0 disables)
    LOG_LEVEL       stdlib logging level (default INFO; DEBUG/WARNING/ERROR)
    NEWTON_BRIDGE_PUBLISH_TF
                    1/true/on or 0/false/off — overrides pack ros.publish_tf.
                    Equivalent to --publish-tf / --no-publish-tf.

CLI flags:
    --publish-tf / --no-publish-tf
                    Override pack ros.publish_tf. Highest precedence; falls
                    back to env var, then yaml, then default true.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
from pathlib import Path

import warp as wp
import rclpy

from .robot_pack import load_pack
from .world import NewtonWorld
from .node import SimBridgeNode
from .snapshot import StateSnapshot
from .telemetry import StatusLogger, TelemetryRegistry
from .viewer import build_viewer, resolve_mode
from .viewer_thread import ViewerThread


def _resolve_pack_dir() -> Path:
    raw = os.environ.get("ROBOT_PACK", "/workspace/robots/ur5e")
    p = Path(raw)
    if not p.is_dir():
        raise FileNotFoundError(f"ROBOT_PACK not a directory: {p}")
    return p


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _env_bool(name: str) -> bool | None:
    """Return True/False for common boolean spellings, or None if unset/unparseable.

    Accepts: 1/0, true/false, yes/no, on/off (case-insensitive). Anything else
    -> None so we fall through to the next precedence layer instead of silently
    coercing typos to False.
    """
    raw = os.environ.get(name)
    if raw is None:
        return None
    s = raw.strip().lower()
    if s in {"1", "true", "yes", "on"}:
        return True
    if s in {"0", "false", "no", "off"}:
        return False
    return None


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="python -m newton_bridge",
        description="Newton ↔ ROS 2 bridge entry point.",
    )
    p.add_argument(
        "--publish-tf",
        dest="publish_tf",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable/disable /tf publishing. Overrides pack ros.publish_tf "
        "and NEWTON_BRIDGE_PUBLISH_TF.",
    )
    return p.parse_args(argv)


def _configure_logging() -> None:
    """Install a stderr handler on the root logger.

    Without this, stdlib `logging` drops everything below WARNING (no handler
    means `lastResort` kicks in, which only emits WARN+). The status logger
    in `telemetry.py` uses INFO for healthy ticks, so users would only ever
    see the line when the pipeline stalled. Also picks up
    `viewer_thread.log.exception(...)` calls.
    """
    level_name = os.environ.get("LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    logging.basicConfig(
        level=level,
        format="[%(name)s] %(levelname)s: %(message)s",
        stream=sys.stderr,
    )


def _ros_logger_emit(node):
    """Adapter from `StatusLogger`'s `(level, msg)` callable to ROS logging.

    StatusLogger uses stdlib level ints (INFO=20, WARNING=30). ROS rclpy
    loggers expose `.info() / .warn() / .error() / .debug()`. Map by level so
    STALL lines arrive at WARN and show up in ROS log viewers / bag tooling.
    """
    rl = node.get_logger()

    def emit(level: int, msg: str) -> None:
        if level >= logging.ERROR:
            rl.error(msg)
        elif level >= logging.WARNING:
            rl.warn(msg)
        elif level >= logging.INFO:
            rl.info(msg)
        else:
            rl.debug(msg)

    return emit


def main(argv: list[str] | None = None) -> int:
    _configure_logging()
    args = _parse_args(argv)
    pack_dir = _resolve_pack_dir()
    sync_mode = os.environ.get("SYNC_MODE", "freerun").lower()
    rate_mode = os.environ.get("FREERUN_RATE", "realtime").lower()
    if sync_mode == "handshake":
        print(
            "[newton_bridge] SYNC_MODE=handshake is deprecated; use SYNC_MODE=sync",
            file=sys.stderr,
            flush=True,
        )
        sync_mode = "sync"
    if sync_mode not in {"freerun", "sync"}:
        print(f"[newton_bridge] invalid SYNC_MODE={sync_mode!r}", file=sys.stderr)
        return 2

    viewer_mode = resolve_mode()  # may SystemExit on bad VIEWER / legacy ENABLE_VIEWER

    print(f"[newton_bridge] loading pack: {pack_dir}", flush=True)
    pack = load_pack(pack_dir)

    # publish_tf precedence: CLI > env > yaml > default(True). Resolved here
    # (not in node.py) so the value passed into the node is the final one.
    if args.publish_tf is not None:
        pack["ros"]["publish_tf"] = args.publish_tf
    else:
        env_tf = _env_bool("NEWTON_BRIDGE_PUBLISH_TF")
        if env_tf is not None:
            pack["ros"]["publish_tf"] = env_tf

    wp.init()
    print(f"[newton_bridge] Warp CUDA devices: {wp.get_cuda_devices()}", flush=True)

    world = NewtonWorld(pack)
    print(
        f"[newton_bridge] world ready: dof={world.total_dof}, "
        f"joints={len(world.joint_dof_names)}, dt={world.physics_dt:.6f}s, "
        f"solver={pack['sim'].get('solver', 'xpbd')}",
        flush=True,
    )

    # Snapshot buffers: two extra State()s so the viewer thread always reads
    # a slot the producer is not currently writing into.
    snapshot = StateSnapshot(world.model.state(), world.model.state())

    sim_cfg = pack["sim"]
    ros_cfg = pack["ros"]
    viewer_hz = sim_cfg.get("viewer_hz", 60)
    sync_timeout_s = float(ros_cfg.get("sync_timeout_ms", 100)) / 1000.0

    telemetry = TelemetryRegistry(
        physics_dt=world.physics_dt,
        sync_mode=(sync_mode == "sync"),
        sync_timeout_s=sync_timeout_s,
    )

    # ViewerThread: the factory closure is called inside the thread so the
    # GL context (if any) is owned by the thread. mode='none' returns None
    # and the thread idles cleanly.
    def viewer_factory():
        if viewer_mode == "none":
            return None
        try:
            v = build_viewer(world, mode=viewer_mode)
            print(f"[newton_bridge] viewer: {viewer_mode}", flush=True)
            return v
        except Exception as exc:  # noqa: BLE001
            print(
                f"[newton_bridge] VIEWER={viewer_mode} init failed: {exc!r}\n"
                f"[newton_bridge] continuing headless. Set VIEWER=none to silence.",
                file=sys.stderr,
                flush=True,
            )
            return None

    viewer_thread = ViewerThread(
        snapshot, viewer_factory, viewer_hz=viewer_hz, telemetry=telemetry
    )
    viewer_thread.start()
    viewer_thread.wait_ready(timeout=10.0)
    if viewer_thread.build_error is not None:
        # build_error already printed by the factory; thread will idle.
        pass

    if viewer_mode == "none":
        # No viewer at all → stop the thread; nothing for it to do.
        viewer_thread.stop(timeout=1.0)
        viewer_thread = None
    elif viewer_mode in {"rerun", "gl"} and sync_mode == "sync":
        print(
            "[newton_bridge] note: sync mode advances only on /joint_command "
            "or /sim/reset; the viewer will appear frozen until a controller "
            "publishes commands.",
            flush=True,
        )

    rclpy.init(args=None)
    node = SimBridgeNode(
        world,
        sync_mode,
        snapshot=snapshot,
        viewer_thread=viewer_thread,
        telemetry=telemetry,
    )
    # Wire the status logger to the ROS node's logger so the 1Hz line shows
    # up alongside other bridge logs (and at WARN on STALL). Has to happen
    # after node construction since rclpy logger lookup needs the node.
    node.status_logger = StatusLogger(
        period_s=_env_float("STATUS_LOG_HZ", 1.0),
        logger=_ros_logger_emit(node),
    )
    node.ready_log_info = (
        f"Newton ready: dof={world.total_dof}, "
        f"joints={len(world.joint_dof_names)}, dt={world.physics_dt:.6f}s, "
        f"solver={pack['sim'].get('solver', 'xpbd')}, "
        f"sync={sync_mode}, viewer={viewer_mode}"
    )

    # Track ^C count: first one requests graceful shutdown, second one
    # forces immediate exit. Newton/Warp + rclpy + GL teardown can stack
    # several seconds of latency on a clean shutdown; users who hit ^C twice
    # are telling us they don't want to wait.
    sigint_count = 0

    def _sigint(*_):
        nonlocal sigint_count
        sigint_count += 1
        if sigint_count >= 2:
            node.get_logger().warning("second shutdown signal — forcing exit")
            os._exit(130)
        node.get_logger().warning("shutdown signal received")
        node.request_shutdown()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        if sync_mode == "sync":
            node.run_sync()
        else:
            node.run_freerun(rate_mode)
    finally:
        if viewer_thread is not None:
            viewer_thread.stop(timeout=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
