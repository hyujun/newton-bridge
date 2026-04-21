"""Unit tests for RenderTicker and CommandWatchdog.

Pure Python, no Newton/rclpy. Covers Phase 2 and Phase 3 of the sync-mode
redesign.
"""

from __future__ import annotations

import pytest

from newton_bridge.ticks import CommandWatchdog, RenderTicker


# ---------- RenderTicker ---------------------------------------------------

def test_render_first_tick_fires_immediately() -> None:
    # First tick should always return True so initial state renders right away.
    t = RenderTicker(render_hz=60)
    assert t.tick(now=100.0) is True


def test_render_waits_render_dt_between_fires() -> None:
    t = RenderTicker(render_hz=60)
    t.tick(now=0.0)  # arm first frame
    # Just under render_dt: no fire.
    assert t.tick(now=1.0 / 60 - 0.001) is False
    # Just over render_dt: fire.
    assert t.tick(now=1.0 / 60 + 0.001) is True


def test_render_longrun_rate_matches_target() -> None:
    # Poll at 1 kHz for 10s wall time; 60Hz target should yield ~600 fires.
    t = RenderTicker(render_hz=60)
    now = 0.0
    hits = 0
    for _ in range(10_000):
        if t.tick(now=now):
            hits += 1
        now += 1.0 / 1000
    assert 595 <= hits <= 605, hits


def test_render_decoupled_from_poll_rate() -> None:
    # Polling at 200Hz or 1000Hz should both produce ~60Hz fires over the
    # same wall window — the whole point of wall-clock decoupling.
    def hits_at(poll_hz: int, seconds: float) -> int:
        t = RenderTicker(render_hz=60)
        step = 1.0 / poll_hz
        now = 0.0
        n = 0
        for _ in range(int(poll_hz * seconds)):
            if t.tick(now=now):
                n += 1
            now += step
        return n

    assert abs(hits_at(200, 5.0) - 300) <= 5
    assert abs(hits_at(1000, 5.0) - 300) <= 5


def test_render_hz_zero_is_passthrough() -> None:
    t = RenderTicker(render_hz=0)
    assert t.passthrough is True
    assert all(t.tick() for _ in range(100))


def test_render_hz_none_is_passthrough() -> None:
    t = RenderTicker(render_hz=None)
    assert t.passthrough is True
    assert all(t.tick() for _ in range(100))


def test_render_dt_property() -> None:
    assert RenderTicker(render_hz=60).render_dt == pytest.approx(1.0 / 60)
    assert RenderTicker(render_hz=0).render_dt == 0.0


def test_render_hitch_does_not_burst() -> None:
    # Long gap (simulated pause) should yield one fire on resume, not a
    # burst of catch-up frames.
    t = RenderTicker(render_hz=60)
    t.tick(now=0.0)
    # Jump forward 10 seconds — 600 periods elapsed.
    assert t.tick(now=10.0) is True
    # Immediately after, we should be back to normal cadence (no burst).
    assert t.tick(now=10.0 + 0.001) is False
    assert t.tick(now=10.0 + 1.0 / 60 + 0.001) is True


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
