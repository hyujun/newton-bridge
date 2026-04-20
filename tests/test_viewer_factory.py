"""Phase 7: viewer mode resolution (no Newton import needed)."""

from __future__ import annotations

import pytest

from newton_bridge.viewer import VALID_MODES, resolve_mode


def test_default_mode_is_rerun(monkeypatch):
    monkeypatch.delenv("VIEWER", raising=False)
    monkeypatch.delenv("ENABLE_VIEWER", raising=False)
    assert resolve_mode() == "rerun"


@pytest.mark.parametrize("m", sorted(VALID_MODES))
def test_valid_modes_round_trip(m: str, monkeypatch):
    monkeypatch.delenv("ENABLE_VIEWER", raising=False)
    monkeypatch.setenv("VIEWER", m.upper())  # case-insensitive
    assert resolve_mode() == m


def test_legacy_enable_viewer_is_rejected(monkeypatch):
    monkeypatch.delenv("VIEWER", raising=False)
    monkeypatch.setenv("ENABLE_VIEWER", "1")
    with pytest.raises(SystemExit, match="ENABLE_VIEWER was removed"):
        resolve_mode()


def test_unknown_viewer_raises(monkeypatch):
    monkeypatch.delenv("ENABLE_VIEWER", raising=False)
    monkeypatch.setenv("VIEWER", "bogus")
    with pytest.raises(SystemExit, match="unknown VIEWER"):
        resolve_mode()
