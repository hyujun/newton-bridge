"""Viewer factory (Phase 7, revised in Phase 8).

Viewer settings live in `config/config.yaml` (see `config.py`). This module
just dispatches to the right Newton viewer with the resolved cfg dict.

Supported modes:
    gl       newton.viewer.ViewerGL     X11 window (needs DISPLAY + nvidia GL)
    rerun    newton.viewer.ViewerRerun  web UI on rerun.web_port (default 9090)
    usd      newton.viewer.ViewerUSD    writes USD to viewer.output_dir/<ts>.usd
    file     newton.viewer.ViewerFile   records to viewer.output_dir/<ts>.nvpr
    null     newton.viewer.ViewerNull   no output (benchmarking)
    none     no viewer at all           (explicitly disable)

All Newton imports are lazy to keep headless runs (mode='none') from
pulling GL / Rerun / USD dependencies.
"""

from __future__ import annotations

import datetime as _dt
from pathlib import Path


VALID_MODES = {"rerun", "gl", "usd", "file", "null", "none"}


def _ts_output(output_dir: str, ext: str) -> str:
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    ts = _dt.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    return str(root / f"sim_{ts}.{ext}")


def build_viewer(world, viewer_cfg: dict):
    """Dispatch to the right Newton viewer; returns None for mode='none'.

    `viewer_cfg` is the resolved `viewer:` subtree of config.py output.
    """
    mode = viewer_cfg["mode"]
    if mode not in VALID_MODES:
        raise SystemExit(
            f"[newton_bridge] unknown viewer mode={mode!r}. "
            f"expected one of {sorted(VALID_MODES)}"
        )

    if mode == "none":
        return None

    if mode == "gl":
        from newton.viewer import ViewerGL
        v = ViewerGL(
            width=int(viewer_cfg["width"]),
            height=int(viewer_cfg["height"]),
            vsync=bool(viewer_cfg["gl"]["vsync"]),
        )
        v.set_model(world.model)
        return v

    if mode == "rerun":
        from newton.viewer import ViewerRerun
        rr = viewer_cfg["rerun"]
        v = ViewerRerun(
            app_id=rr["app_id"],
            serve_web_viewer=True,
            web_port=int(rr["web_port"]),
            grpc_port=int(rr["grpc_port"]),
            record_to_rrd=rr["record_to_rrd"] or None,
        )
        v.set_model(world.model)
        return v

    if mode == "usd":
        from newton.viewer import ViewerUSD
        usd = viewer_cfg["usd"]
        out = usd["output_path"] or _ts_output(viewer_cfg["output_dir"], "usd")
        v = ViewerUSD(
            output_path=out,
            fps=int(usd["fps"]),
            up_axis=usd["up_axis"],
        )
        v.set_model(world.model)
        return v

    if mode == "file":
        from newton.viewer import ViewerFile
        f = viewer_cfg["file"]
        out = f["output_path"] or _ts_output(viewer_cfg["output_dir"], "nvpr")
        v = ViewerFile(output_path=out, auto_save=True)
        v.set_model(world.model)
        return v

    # mode == "null"
    from newton.viewer import ViewerNull
    v = ViewerNull()
    v.set_model(world.model)
    return v
