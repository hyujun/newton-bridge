"""Viewer mode resolution moved to config.resolve() in Phase 8.

These tests cover the same surface that lived in viewer.resolve_mode() before:
default mode, case-insensitivity, legacy ENABLE_VIEWER rejection, and unknown
mode rejection. The viewer factory itself (build_viewer) needs newton.viewer
and a built model, so it's not unit-tested here.
"""

from __future__ import annotations

import argparse

import pytest

from newton_bridge.config import VALID_VIEWER_MODES, resolve
from newton_bridge.viewer import VALID_MODES


def _empty_args() -> argparse.Namespace:
    return argparse.Namespace()


def test_viewer_factory_modes_match_config_modes():
    assert VALID_MODES == VALID_VIEWER_MODES


def test_default_mode_is_gl(monkeypatch):
    monkeypatch.delenv("VIEWER", raising=False)
    monkeypatch.delenv("ENABLE_VIEWER", raising=False)
    cfg = resolve(_empty_args(), config_path=None)
    assert cfg["viewer"]["mode"] == "gl"


@pytest.mark.parametrize("m", sorted(VALID_VIEWER_MODES))
def test_valid_modes_round_trip(m: str, monkeypatch):
    monkeypatch.delenv("ENABLE_VIEWER", raising=False)
    monkeypatch.setenv("VIEWER", m.upper())  # case-insensitive
    cfg = resolve(_empty_args(), config_path=None)
    assert cfg["viewer"]["mode"] == m


def test_legacy_enable_viewer_is_rejected(monkeypatch):
    monkeypatch.delenv("VIEWER", raising=False)
    monkeypatch.setenv("ENABLE_VIEWER", "1")
    with pytest.raises(SystemExit, match="ENABLE_VIEWER was removed"):
        resolve(_empty_args(), config_path=None)


def test_unknown_viewer_raises(monkeypatch):
    monkeypatch.delenv("ENABLE_VIEWER", raising=False)
    monkeypatch.setenv("VIEWER", "bogus")
    with pytest.raises(SystemExit, match="unknown viewer mode"):
        resolve(_empty_args(), config_path=None)
