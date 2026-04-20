"""Viewer factory (Phase 7).

`VIEWER` env var picks which Newton viewer to attach. Default is **rerun**
(web UI on host port 9090, no X11 required). The legacy `ENABLE_VIEWER=1`
variable is rejected with a migration hint.

Supported modes:
    rerun    newton.viewer.ViewerRerun  web UI on RERUN_WEB_PORT (default 9090)
    gl       newton.viewer.ViewerGL     X11 window (needs DISPLAY + nvidia GL)
    usd      newton.viewer.ViewerUSD    writes USD to workspace/runs/<ts>.usd
    file     newton.viewer.ViewerFile   records to workspace/runs/<ts>.nvpr
    null     newton.viewer.ViewerNull   no output (benchmarking)
    none     no viewer at all           (explicitly disable)

All Newton imports are lazy to keep headless runs (VIEWER=none) from
pulling GL / Rerun / USD dependencies.
"""

from __future__ import annotations

import datetime as _dt
import os
from pathlib import Path


VALID_MODES = {"rerun", "gl", "usd", "file", "null", "none"}


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _ts_output(ext: str) -> str:
    """Timestamped path under workspace/runs/<UTC>.ext."""
    root = Path(os.environ.get("VIEWER_OUTPUT_DIR", "/workspace/workspace/runs"))
    root.mkdir(parents=True, exist_ok=True)
    ts = _dt.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    return str(root / f"sim_{ts}.{ext}")


def resolve_mode() -> str:
    """Read VIEWER env; reject the deprecated ENABLE_VIEWER."""
    if os.environ.get("ENABLE_VIEWER", "").strip().lower() in {"1", "true", "yes", "on"}:
        raise SystemExit(
            "[newton_bridge] ENABLE_VIEWER was removed in Phase 7. "
            "Use VIEWER=rerun (default) | gl | usd | file | null | none instead."
        )
    mode = os.environ.get("VIEWER", "rerun").strip().lower()
    if mode not in VALID_MODES:
        raise SystemExit(
            f"[newton_bridge] unknown VIEWER={mode!r}. "
            f"expected one of {sorted(VALID_MODES)}"
        )
    return mode


def build_viewer(world, mode: str | None = None):
    """Dispatch to the right Newton viewer; returns None for mode='none'."""
    mode = mode or resolve_mode()

    if mode == "none":
        return None

    if mode == "gl":
        from newton.viewer import ViewerGL
        width = _env_int("VIEWER_WIDTH", 1280)
        height = _env_int("VIEWER_HEIGHT", 720)
        v = ViewerGL(width=width, height=height, vsync=False)
        v.set_model(world.model)
        return v

    if mode == "rerun":
        from newton.viewer import ViewerRerun
        v = ViewerRerun(
            app_id=os.environ.get("RERUN_APP_ID", "newton_bridge"),
            serve_web_viewer=True,
            web_port=_env_int("RERUN_WEB_PORT", 9090),
            grpc_port=_env_int("RERUN_GRPC_PORT", 9876),
            record_to_rrd=os.environ.get("RERUN_RECORD_TO") or None,
        )
        v.set_model(world.model)
        return v

    if mode == "usd":
        from newton.viewer import ViewerUSD
        out = os.environ.get("VIEWER_OUTPUT_PATH") or _ts_output("usd")
        v = ViewerUSD(
            output_path=out,
            fps=_env_int("VIEWER_FPS", 60),
            up_axis=os.environ.get("VIEWER_UP_AXIS", "Z"),
        )
        v.set_model(world.model)
        return v

    if mode == "file":
        from newton.viewer import ViewerFile
        out = os.environ.get("VIEWER_OUTPUT_PATH") or _ts_output("nvpr")
        v = ViewerFile(output_path=out, auto_save=True)
        v.set_model(world.model)
        return v

    # mode == "null"
    from newton.viewer import ViewerNull
    v = ViewerNull()
    v.set_model(world.model)
    return v
