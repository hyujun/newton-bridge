"""`python -m newton_bridge` entry point.

Reads env:
    ROBOT_PACK      container path to robots/<name>/ (default /workspace/robots/ur5e)
    SYNC_MODE       freerun | handshake (default freerun)
    FREERUN_RATE    realtime | max (freerun only, default realtime)
    ENABLE_VIEWER   0 | 1 (optional GL viewer window)
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
from .viewer import build_viewer


def _resolve_pack_dir() -> Path:
    raw = os.environ.get("ROBOT_PACK", "/workspace/robots/ur5e")
    p = Path(raw)
    if not p.is_dir():
        raise FileNotFoundError(f"ROBOT_PACK not a directory: {p}")
    return p


def _env_truthy(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}


def main() -> int:
    pack_dir = _resolve_pack_dir()
    sync_mode = os.environ.get("SYNC_MODE", "freerun").lower()
    rate_mode = os.environ.get("FREERUN_RATE", "realtime").lower()
    if sync_mode not in {"freerun", "handshake"}:
        print(f"[newton_bridge] invalid SYNC_MODE={sync_mode!r}", file=sys.stderr)
        return 2

    print(f"[newton_bridge] loading pack: {pack_dir}", flush=True)
    pack = load_pack(pack_dir)

    wp.init()
    print(f"[newton_bridge] Warp CUDA devices: {wp.get_cuda_devices()}", flush=True)

    world = NewtonWorld(pack)
    print(
        f"[newton_bridge] world ready: dof={world.total_dof}, "
        f"joints={len(world.joint_layout)}, dt={world.physics_dt:.6f}s, "
        f"solver={pack['sim'].get('solver', 'xpbd')}",
        flush=True,
    )

    viewer = None
    if _env_truthy("ENABLE_VIEWER"):
        try:
            viewer = build_viewer(world)
            print("[newton_bridge] GL viewer enabled (close window to stop sim)", flush=True)
        except Exception as exc:  # noqa: BLE001 — surface viewer init failures clearly
            print(
                f"[newton_bridge] ENABLE_VIEWER=1 but viewer init failed: {exc!r}\n"
                f"[newton_bridge] continuing headless. Check DISPLAY + xhost + nvidia GL.",
                file=sys.stderr,
                flush=True,
            )
            viewer = None
        if viewer is not None and sync_mode == "handshake":
            print(
                "[newton_bridge] note: handshake mode renders only on /sim/step or /sim/reset; "
                "the window will appear frozen until a controller calls those services.",
                flush=True,
            )

    rclpy.init(args=None)
    node = SimBridgeNode(world, sync_mode, viewer=viewer)

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
