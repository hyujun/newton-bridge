"""Session config loader (Phase 8).

Resolves runtime/viewer settings with precedence **CLI > env > yaml > code default**.
Pack-level `robot.yaml` / `scene.yaml` keep robot facts (joints, drives, gravity);
this module owns session-level knobs (sync mode, viewer mode, viewer geometry,
status log cadence, …).

Two pack-level keys are migrated here: `sim.viewer_hz` and `ros.sync_timeout_ms`.
`__main__.py` injects the resolved values into the pack dict before constructing
the node, so downstream readers (node.py, viewer_thread.py) keep their existing
`pack["sim"]["viewer_hz"]` / `pack["ros"]["sync_timeout_ms"]` access pattern.
"""

from __future__ import annotations

import argparse
import copy
import os
from pathlib import Path
from typing import Any

import yaml


_DEFAULTS: dict[str, Any] = {
    "sim": {
        "sync_mode": "freerun",        # freerun | sync
        "freerun_rate": "realtime",    # realtime | max
        "status_log_hz": 0.0,          # 0 = off
        "viewer_hz": 60,               # injected into pack['sim']
        "sync_timeout_ms": 100,        # injected into pack['ros']
    },
    "logging": {
        "level": "INFO",
    },
    "viewer": {
        "mode": "gl",                  # gl | rerun | usd | file | null | none
        "width": 1280,
        "height": 720,
        "output_dir": "/workspace/workspace/runs",
        "ready_timeout_s": 60.0,       # cold-boot raycast compile ~11s
        "gl": {
            "vsync": False,
        },
        "rerun": {
            "app_id": "newton_bridge",
            "web_port": 9090,
            "grpc_port": 9876,
            "record_to_rrd": None,
        },
        "usd": {
            "fps": 60,
            "up_axis": "Z",
            "output_path": None,       # None = timestamped under output_dir
        },
        "file": {
            "output_path": None,
        },
    },
}


VALID_VIEWER_MODES = {"rerun", "gl", "usd", "file", "null", "none"}
VALID_SYNC_MODES = {"freerun", "sync"}
VALID_FREERUN_RATES = {"realtime", "max"}


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Return base with overlay merged in (recursive for nested dicts).

    `None` values in overlay are skipped — they mean "no override" so the
    CLI/env layers can pass None for unset flags without clobbering yaml.
    """
    out = copy.deepcopy(base)
    for k, v in overlay.items():
        if v is None:
            continue
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_yaml(path: Path | None) -> dict:
    """Load a yaml config file. Returns {} if path is None or missing."""
    if path is None or not path.is_file():
        return {}
    with path.open() as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError(f"config yaml must be a mapping at the top level: {path}")
    return data


def _parse_bool(raw: str) -> bool | None:
    s = raw.strip().lower()
    if s in {"1", "true", "yes", "on"}:
        return True
    if s in {"0", "false", "no", "off"}:
        return False
    return None


def _env_overlay() -> dict:
    """Translate known env vars into an overlay dict matching _DEFAULTS shape.

    Unset env vars contribute no keys (so yaml/defaults remain). Unparseable
    values are skipped silently rather than coerced — match the existing
    `_env_bool` / `_env_int` behavior from the pre-refactor entrypoint.
    """
    overlay: dict[str, Any] = {"sim": {}, "logging": {}, "viewer": {"gl": {}, "rerun": {}, "usd": {}, "file": {}}}

    # Legacy ENABLE_VIEWER guard — kept here so error surfaces during resolve(),
    # not at the viewer factory call.
    enable = os.environ.get("ENABLE_VIEWER", "").strip().lower()
    if enable in {"1", "true", "yes", "on"}:
        raise SystemExit(
            "[newton_bridge] ENABLE_VIEWER was removed in Phase 7. "
            "Use VIEWER=gl (default) | rerun | usd | file | null | none instead."
        )

    def _str(name: str) -> str | None:
        raw = os.environ.get(name)
        return raw.strip() if raw is not None else None

    def _int(name: str) -> int | None:
        raw = os.environ.get(name)
        if raw is None:
            return None
        try:
            return int(raw)
        except ValueError:
            return None

    def _float(name: str) -> float | None:
        raw = os.environ.get(name)
        if raw is None:
            return None
        try:
            return float(raw)
        except ValueError:
            return None

    # sim
    if (v := _str("SYNC_MODE")) is not None:
        overlay["sim"]["sync_mode"] = v.lower()
    if (v := _str("FREERUN_RATE")) is not None:
        overlay["sim"]["freerun_rate"] = v.lower()
    if (vf := _float("STATUS_LOG_HZ")) is not None:
        overlay["sim"]["status_log_hz"] = vf

    # logging
    if (v := _str("LOG_LEVEL")) is not None:
        overlay["logging"]["level"] = v.upper()

    # viewer
    if (v := _str("VIEWER")) is not None:
        overlay["viewer"]["mode"] = v.lower()
    if (vi := _int("VIEWER_WIDTH")) is not None:
        overlay["viewer"]["width"] = vi
    if (vi := _int("VIEWER_HEIGHT")) is not None:
        overlay["viewer"]["height"] = vi
    if (v := _str("VIEWER_OUTPUT_DIR")) is not None:
        overlay["viewer"]["output_dir"] = v

    # rerun
    if (v := _str("RERUN_APP_ID")) is not None:
        overlay["viewer"]["rerun"]["app_id"] = v
    if (vi := _int("RERUN_WEB_PORT")) is not None:
        overlay["viewer"]["rerun"]["web_port"] = vi
    if (vi := _int("RERUN_GRPC_PORT")) is not None:
        overlay["viewer"]["rerun"]["grpc_port"] = vi
    if (v := _str("RERUN_RECORD_TO")) is not None:
        overlay["viewer"]["rerun"]["record_to_rrd"] = v or None

    # usd / file output
    if (vi := _int("VIEWER_FPS")) is not None:
        overlay["viewer"]["usd"]["fps"] = vi
    if (v := _str("VIEWER_UP_AXIS")) is not None:
        overlay["viewer"]["usd"]["up_axis"] = v
    if (v := _str("VIEWER_OUTPUT_PATH")) is not None:
        # Shared by usd + file modes — applied to both layers; build_viewer
        # only reads the one that matches the active mode.
        overlay["viewer"]["usd"]["output_path"] = v
        overlay["viewer"]["file"]["output_path"] = v

    return overlay


def _cli_overlay(args: argparse.Namespace) -> dict:
    """Translate argparse Namespace into an overlay dict. None == no override."""
    overlay: dict[str, Any] = {"sim": {}, "logging": {}, "viewer": {}}
    sync_mode = getattr(args, "sync_mode", None)
    if sync_mode is not None:
        overlay["sim"]["sync_mode"] = sync_mode
    freerun_rate = getattr(args, "freerun_rate", None)
    if freerun_rate is not None:
        overlay["sim"]["freerun_rate"] = freerun_rate
    status_log_hz = getattr(args, "status_log_hz", None)
    if status_log_hz is not None:
        overlay["sim"]["status_log_hz"] = status_log_hz
    log_level = getattr(args, "log_level", None)
    if log_level is not None:
        overlay["logging"]["level"] = log_level.upper()
    viewer = getattr(args, "viewer", None)
    if viewer is not None:
        overlay["viewer"]["mode"] = viewer
    viewer_width = getattr(args, "viewer_width", None)
    if viewer_width is not None:
        overlay["viewer"]["width"] = viewer_width
    viewer_height = getattr(args, "viewer_height", None)
    if viewer_height is not None:
        overlay["viewer"]["height"] = viewer_height
    return overlay


def _validate(cfg: dict) -> None:
    sim = cfg["sim"]
    if sim["sync_mode"] == "handshake":
        # Soft-migrate legacy spelling — same shim the old entrypoint had.
        sim["sync_mode"] = "sync"
    if sim["sync_mode"] not in VALID_SYNC_MODES:
        raise SystemExit(f"[newton_bridge] invalid sync_mode={sim['sync_mode']!r}")
    if sim["freerun_rate"] not in VALID_FREERUN_RATES:
        raise SystemExit(
            f"[newton_bridge] invalid freerun_rate={sim['freerun_rate']!r}"
        )
    mode = cfg["viewer"]["mode"]
    if mode not in VALID_VIEWER_MODES:
        raise SystemExit(
            f"[newton_bridge] unknown viewer mode={mode!r}. "
            f"expected one of {sorted(VALID_VIEWER_MODES)}"
        )


def resolve(
    cli_args: argparse.Namespace | None = None,
    config_path: Path | None = None,
) -> dict:
    """Compose final config: defaults → yaml → env → cli."""
    cfg = _deep_merge(_DEFAULTS, load_yaml(config_path))
    cfg = _deep_merge(cfg, _env_overlay())
    if cli_args is not None:
        cfg = _deep_merge(cfg, _cli_overlay(cli_args))
    _validate(cfg)
    return cfg


def default_config_path() -> Path:
    """Repo-relative default. Returns /workspace/config/config.yaml in-container,
    or the repo's config/config.yaml on the host when imported from a checkout."""
    in_container = Path("/workspace/config/config.yaml")
    if in_container.is_file():
        return in_container
    return Path(__file__).resolve().parents[2] / "config" / "config.yaml"
