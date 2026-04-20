"""Unit tests for RenderTicker and CommandWatchdog.

Pure Python, no Newton/rclpy. Covers Phase 2 and Phase 3 of the sync-mode
redesign.
"""

from __future__ import annotations

import pytest

from newton_bridge.ticks import CommandWatchdog, RenderTicker


# ---------- RenderTicker ---------------------------------------------------

def test_render_60hz_over_500hz_ratio() -> None:
    t = RenderTicker(render_hz=60, physics_dt=1.0 / 500)
    hits = sum(1 for _ in range(5000) if t.tick())
    # 5000 physics steps at 500Hz = 10s sim time. At 60Hz that is 600 frames.
    assert 590 <= hits <= 610, hits


def test_render_hz_zero_is_passthrough() -> None:
    t = RenderTicker(render_hz=0, physics_dt=1.0 / 500)
    assert t.passthrough is True
    assert all(t.tick() for _ in range(100))


def test_render_hz_none_is_passthrough() -> None:
    t = RenderTicker(render_hz=None, physics_dt=1.0 / 500)
    assert t.passthrough is True
    assert all(t.tick() for _ in range(100))


def test_render_exact_boundary_triggers_once() -> None:
    # render_hz == physics_hz: every tick should fire.
    t = RenderTicker(render_hz=500, physics_dt=1.0 / 500)
    hits = sum(1 for _ in range(50) if t.tick())
    assert hits == 50


def test_render_ticker_half_rate() -> None:
    # render_hz is half of physics_hz: fire every other tick.
    t = RenderTicker(render_hz=250, physics_dt=1.0 / 500)
    results = [t.tick() for _ in range(10)]
    # Pattern: True/False alternation (first tick hits at dt == render_dt).
    assert sum(results) == 5


def test_render_ticker_rejects_zero_dt() -> None:
    with pytest.raises(ValueError):
        RenderTicker(render_hz=60, physics_dt=0.0)


def test_render_ticker_custom_dt_override() -> None:
    t = RenderTicker(render_hz=60, physics_dt=1.0 / 500)
    # Feed a giant dt — should still fire exactly once per tick call.
    assert t.tick(dt=1.0) is True


# ---------- CommandWatchdog ------------------------------------------------

def test_watchdog_not_stale_before_timeout() -> None:
    w = CommandWatchdog(timeout_s=0.1)
    w.note_command(now=0.0)
    assert w.is_stale(now=0.05) is False


def test_watchdog_stale_after_timeout() -> None:
    w = CommandWatchdog(timeout_s=0.1)
    w.note_command(now=0.0)
    assert w.is_stale(now=0.1) is True


def test_watchdog_rearms_after_stale_read() -> None:
    # Once stale fires, it shouldn't fire again every tick — caller wants
    # periodic republishes at roughly timeout cadence.
    w = CommandWatchdog(timeout_s=0.1)
    w.note_command(now=0.0)
    assert w.is_stale(now=0.1) is True
    assert w.is_stale(now=0.15) is False
    assert w.is_stale(now=0.2) is True


def test_watchdog_command_resets() -> None:
    w = CommandWatchdog(timeout_s=0.1)
    w.note_command(now=0.0)
    w.note_command(now=0.09)
    assert w.is_stale(now=0.1) is False
    assert w.is_stale(now=0.19) is True


def test_watchdog_initial_read_arms_clock() -> None:
    # Before any command arrives, the first is_stale() call should NOT fire —
    # otherwise every subscriber would see a duplicate on startup.
    w = CommandWatchdog(timeout_s=0.1)
    assert w.is_stale(now=0.0) is False
    assert w.is_stale(now=0.1) is True


def test_watchdog_rejects_zero_timeout() -> None:
    with pytest.raises(ValueError):
        CommandWatchdog(timeout_s=0.0)
