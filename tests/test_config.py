"""config.resolve() precedence chain: defaults < yaml < env < cli."""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest

from newton_bridge.config import _DEFAULTS, resolve


def _args(**kwargs) -> argparse.Namespace:
    return argparse.Namespace(**kwargs)


def _write_yaml(tmp_path: Path, body: str) -> Path:
    p = tmp_path / "config.yaml"
    p.write_text(body)
    return p


def _clear_env(monkeypatch) -> None:
    for k in (
        "VIEWER", "ENABLE_VIEWER", "SYNC_MODE", "FREERUN_RATE",
        "STATUS_LOG_HZ", "LOG_LEVEL",
        "VIEWER_WIDTH", "VIEWER_HEIGHT", "VIEWER_OUTPUT_DIR",
        "RERUN_APP_ID", "RERUN_WEB_PORT", "RERUN_GRPC_PORT", "RERUN_RECORD_TO",
        "VIEWER_FPS", "VIEWER_UP_AXIS", "VIEWER_OUTPUT_PATH",
    ):
        monkeypatch.delenv(k, raising=False)


def test_defaults_only(monkeypatch):
    _clear_env(monkeypatch)
    cfg = resolve(_args(), config_path=None)
    assert cfg["sim"]["sync_mode"] == _DEFAULTS["sim"]["sync_mode"]
    assert cfg["viewer"]["mode"] == _DEFAULTS["viewer"]["mode"]
    assert cfg["viewer"]["width"] == _DEFAULTS["viewer"]["width"]


def test_yaml_overrides_defaults(monkeypatch, tmp_path):
    _clear_env(monkeypatch)
    yml = _write_yaml(tmp_path, "sim:\n  sync_mode: sync\nviewer:\n  width: 1920\n")
    cfg = resolve(_args(), config_path=yml)
    assert cfg["sim"]["sync_mode"] == "sync"
    assert cfg["viewer"]["width"] == 1920
    # unrelated keys still take defaults
    assert cfg["viewer"]["mode"] == _DEFAULTS["viewer"]["mode"]


def test_env_overrides_yaml(monkeypatch, tmp_path):
    _clear_env(monkeypatch)
    yml = _write_yaml(tmp_path, "sim:\n  sync_mode: sync\nviewer:\n  width: 1920\n")
    monkeypatch.setenv("SYNC_MODE", "freerun")
    monkeypatch.setenv("VIEWER_WIDTH", "1600")
    cfg = resolve(_args(), config_path=yml)
    assert cfg["sim"]["sync_mode"] == "freerun"
    assert cfg["viewer"]["width"] == 1600


def test_cli_overrides_env(monkeypatch, tmp_path):
    _clear_env(monkeypatch)
    yml = _write_yaml(tmp_path, "sim:\n  sync_mode: sync\n")
    monkeypatch.setenv("SYNC_MODE", "freerun")
    args = _args(sync_mode="sync", viewer="rerun", viewer_width=2000)
    cfg = resolve(args, config_path=yml)
    assert cfg["sim"]["sync_mode"] == "sync"     # CLI wins over env
    assert cfg["viewer"]["mode"] == "rerun"
    assert cfg["viewer"]["width"] == 2000


def test_missing_config_file_falls_back_to_defaults(monkeypatch, tmp_path):
    _clear_env(monkeypatch)
    cfg = resolve(_args(), config_path=tmp_path / "nope.yaml")
    assert cfg["sim"]["sync_mode"] == _DEFAULTS["sim"]["sync_mode"]


def test_unparseable_env_is_skipped(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv("VIEWER_WIDTH", "not-a-number")
    cfg = resolve(_args(), config_path=None)
    assert cfg["viewer"]["width"] == _DEFAULTS["viewer"]["width"]


def test_handshake_sync_mode_softmigrates(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv("SYNC_MODE", "handshake")
    cfg = resolve(_args(), config_path=None)
    assert cfg["sim"]["sync_mode"] == "sync"


def test_invalid_sync_mode_raises(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv("SYNC_MODE", "bogus")
    with pytest.raises(SystemExit, match="invalid sync_mode"):
        resolve(_args(), config_path=None)


def test_invalid_freerun_rate_raises(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv("FREERUN_RATE", "bogus")
    with pytest.raises(SystemExit, match="invalid freerun_rate"):
        resolve(_args(), config_path=None)


def test_cli_none_values_dont_clobber(monkeypatch, tmp_path):
    """argparse default=None for an unspecified flag must not override yaml."""
    _clear_env(monkeypatch)
    yml = _write_yaml(tmp_path, "viewer:\n  mode: rerun\n")
    args = _args(viewer=None, viewer_width=None, sync_mode=None)
    cfg = resolve(args, config_path=yml)
    assert cfg["viewer"]["mode"] == "rerun"


def test_rerun_ports_from_env(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv("RERUN_WEB_PORT", "9091")
    monkeypatch.setenv("RERUN_GRPC_PORT", "9877")
    cfg = resolve(_args(), config_path=None)
    assert cfg["viewer"]["rerun"]["web_port"] == 9091
    assert cfg["viewer"]["rerun"]["grpc_port"] == 9877


def test_status_log_hz_cli_wins(monkeypatch):
    _clear_env(monkeypatch)
    monkeypatch.setenv("STATUS_LOG_HZ", "2.0")
    cfg = resolve(_args(status_log_hz=0.5), config_path=None)
    assert cfg["sim"]["status_log_hz"] == 0.5


def test_empty_env_string_is_treated_as_unset(monkeypatch, tmp_path):
    """`docker compose ${VAR-}` forwards a missing host var as VAR="" inside
    the container. That must NOT clobber the yaml value with empty string."""
    _clear_env(monkeypatch)
    yml = _write_yaml(tmp_path, "viewer:\n  mode: rerun\n  width: 1920\n")
    monkeypatch.setenv("VIEWER", "")
    monkeypatch.setenv("VIEWER_WIDTH", "")
    monkeypatch.setenv("STATUS_LOG_HZ", "")
    cfg = resolve(_args(), config_path=yml)
    assert cfg["viewer"]["mode"] == "rerun"
    assert cfg["viewer"]["width"] == 1920
    assert cfg["sim"]["status_log_hz"] == _DEFAULTS["sim"]["status_log_hz"]
