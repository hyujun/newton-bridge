"""`python -m newton_bridge` entry point.

Reads env:
    ROBOT_PACK      container path to robots/<name>/ (default /workspace/robots/ur5e)
    SYNC_MODE       freerun | sync (default freerun). Legacy "handshake" is
                    accepted with a deprecation warning and treated as "sync".
    FREERUN_RATE    realtime | max (freerun only, default realtime)
    VIEWER          rerun | gl | usd | file | null | none (default rerun)
    STATUS_LOG_HZ   1Hz status line cadence (default 1.0; 0 disables)
"""

from __future__ import annotations

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


def main() -> int:
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
    status_logger = StatusLogger(period_s=_env_float("STATUS_LOG_HZ", 1.0))

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

    viewer_thread = ViewerThread(snapshot, viewer_factory, viewer_hz=viewer_hz)
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
        status_logger=status_logger,
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
