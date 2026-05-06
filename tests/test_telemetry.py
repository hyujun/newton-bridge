"""Tests for telemetry primitives — RateMeter, TelemetryRegistry, StatusLogger.

All timing uses an injected fake clock. No threads, no rclpy, no Newton.
"""

from __future__ import annotations

import logging

import pytest

from newton_bridge.telemetry import (
    STATE_IDLE,
    STATE_INIT,
    STATE_RUNNING,
    STATE_STALL,
    PhaseTimer,
    RateMeter,
    StatusLogger,
    TelemetryRegistry,
    format_status,
)


# ---------- RateMeter -----------------------------------------------------


def test_rate_meter_empty_returns_zero() -> None:
    m = RateMeter()
    assert m.rate(now=0.0) == 0.0
    assert m.last_tick is None
    assert m.time_since_last(now=10.0) is None


def test_rate_meter_counts_within_window() -> None:
    m = RateMeter(window_s=1.0)
    for i in range(10):
        m.tick(now=i * 0.1)  # 10 ticks in 1s window
    assert m.rate(now=0.95) == pytest.approx(10.0)


def test_rate_meter_drops_old_samples() -> None:
    m = RateMeter(window_s=1.0)
    # 5 ticks at t=0..0.4, then 5 ticks at t=2.0..2.4
    for i in range(5):
        m.tick(now=i * 0.1)
    for i in range(5):
        m.tick(now=2.0 + i * 0.1)
    # At now=2.5, only the second batch is in-window.
    assert m.rate(now=2.5) == pytest.approx(5.0)


def test_rate_meter_time_since_last() -> None:
    m = RateMeter()
    m.tick(now=10.0)
    assert m.time_since_last(now=10.5) == pytest.approx(0.5)
    m.tick(now=11.0)
    assert m.time_since_last(now=11.2) == pytest.approx(0.2)


def test_rate_meter_rejects_zero_window() -> None:
    with pytest.raises(ValueError):
        RateMeter(window_s=0)


# ---------- TelemetryRegistry: rates & counts -----------------------------


def _reg(**overrides: object) -> TelemetryRegistry:
    kwargs: dict[str, object] = dict(physics_dt=1 / 240, sync_mode=False, sync_timeout_s=0.5)
    kwargs.update(overrides)
    return TelemetryRegistry(**kwargs)  # type: ignore[arg-type]


def test_registry_snapshot_basic_rates() -> None:
    r = _reg()
    for i in range(240):
        r.note_step(now=i / 240)
    for i in range(100):
        r.note_pub(now=i / 100)
    snap = r.snapshot(now=0.99, sim_time=1.0)
    assert snap.step_count == 240
    assert snap.step_hz == pytest.approx(240.0)
    assert snap.pub_hz == pytest.approx(100.0)
    assert snap.realtime_factor == pytest.approx(1.0, rel=0.01)


def test_registry_step_count_monotonic() -> None:
    r = _reg()
    assert r.step_count == 0
    r.note_step(now=0.0)
    r.note_step(now=0.001)
    assert r.step_count == 2


# ---------- TelemetryRegistry: state classification -----------------------


def test_state_init_before_any_step() -> None:
    r = _reg()
    assert r.classify(now=0.0) == STATE_INIT
    assert r.classify(now=10.0) == STATE_INIT


def test_state_running_when_stepping() -> None:
    r = _reg()
    for i in range(10):
        r.note_step(now=i * (1 / 240))
    assert r.classify(now=10 / 240 + 1 / 240) == STATE_RUNNING


def test_state_stall_when_step_gap_exceeds_threshold() -> None:
    """Plan: STALL if now - last_step > max(0.5, 50*dt). For 240Hz, threshold is 0.5s."""
    r = _reg(physics_dt=1 / 240)
    r.note_step(now=0.0)
    assert r.classify(now=0.4) == STATE_RUNNING
    assert r.classify(now=0.6) == STATE_STALL


def test_state_stall_threshold_scales_with_dt() -> None:
    """At 1kHz dt=0.001, threshold should be max(0.5, 50*0.001=0.05) = 0.5s (floor wins)."""
    r = _reg(physics_dt=0.001)
    r.note_step(now=0.0)
    assert r.classify(now=0.4) == STATE_RUNNING
    assert r.classify(now=0.6) == STATE_STALL


def test_state_stall_threshold_scales_for_slow_sim() -> None:
    """At dt=0.05 (20Hz), threshold should be 50*0.05=2.5s (ceiling wins)."""
    r = _reg(physics_dt=0.05)
    r.note_step(now=0.0)
    assert r.classify(now=2.0) == STATE_RUNNING
    assert r.classify(now=3.0) == STATE_STALL


def test_state_idle_in_sync_mode_when_no_recent_command() -> None:
    r = _reg(sync_mode=True, sync_timeout_s=0.5)
    # One command at t=0 then silence; keep stepping so step is fresh.
    r.note_cmd(now=0.0)
    for i in range(0, 300):
        r.note_step(now=i * (1 / 240))
    # now=1.3: cmd was 1.3s ago AND outside the 1s rate window so cmd_rate==0.
    # Last step was ~1.246, so time_since_step≈0.054 → not stall. → IDLE.
    assert r.classify(now=1.3) == STATE_IDLE


def test_state_running_in_sync_with_active_commands() -> None:
    r = _reg(sync_mode=True, sync_timeout_s=0.5)
    for i in range(240):
        r.note_step(now=i / 240)
        r.note_cmd(now=i / 240)
    assert r.classify(now=1.0) == STATE_RUNNING


def test_sync_soft_stall_after_one_second_of_cmds_without_steps() -> None:
    """Sync substall: cmd_rate>0 but step_rate==0 for >1s → STALL.

    Need physics_dt large enough that the hard-stall floor (0.5s) doesn't
    fire before the 1s soft-stall accumulator does.
    """
    r = _reg(sync_mode=True, sync_timeout_s=0.5, physics_dt=0.1)  # hard-stall floor=5s

    def pump_cmds_through(t_end: float) -> None:
        # Tick cmds at 100Hz from t=0 up to t_end so cmd_rate stays > 0.
        i = 0
        while i / 100.0 <= t_end + 1e-9:
            r.note_cmd(now=i / 100.0)
            i += 1

    r.note_step(now=0.0)
    # First arming classify must be late enough that the lone step at t=0 has
    # already aged out of the 1s rate window (so step_rate==0).
    pump_cmds_through(2.0)
    assert r.classify(now=2.0) == STATE_RUNNING  # arming, not yet >1s

    # >1s later, still pumping commands, still no step → STALL.
    pump_cmds_through(3.5)
    assert r.classify(now=3.5) == STATE_STALL


def test_sync_soft_stall_clears_when_step_resumes() -> None:
    r = _reg(sync_mode=True, sync_timeout_s=0.5, physics_dt=0.1)
    r.note_step(now=0.0)
    # Pump cmds at 100Hz so cmd_rate stays >0 across both classify() calls.
    for i in range(0, 350):
        r.note_cmd(now=i * 0.01)
    # Arm soft-stall (step at t=0 is now outside the 1s window).
    r.classify(now=2.0)
    # Step resumes — step_rate becomes >0, soft-stall path resets.
    r.note_step(now=2.6)
    r.note_step(now=2.7)
    r.note_step(now=2.8)
    assert r.classify(now=2.9) == STATE_RUNNING


# ---------- StatusLogger --------------------------------------------------


def test_status_logger_disabled_when_period_zero() -> None:
    sl = StatusLogger(period_s=0)
    assert sl.enabled is False
    r = _reg()
    r.note_step(now=0.0)
    assert sl.maybe_log(now=10.0, registry=r, sim_time=0.0) is False


def test_status_logger_first_call_arms_does_not_emit() -> None:
    """First call only sets the deadline so we don't burst on startup."""
    emissions: list[tuple[int, str]] = []
    sl = StatusLogger(period_s=1.0, logger=lambda lvl, msg: emissions.append((lvl, msg)))
    r = _reg()
    r.note_step(now=0.0)
    assert sl.maybe_log(now=10.0, registry=r, sim_time=0.0) is False
    assert emissions == []


def test_status_logger_emits_at_period() -> None:
    emissions: list[tuple[int, str]] = []
    sl = StatusLogger(period_s=1.0, logger=lambda lvl, msg: emissions.append((lvl, msg)))
    r = _reg()

    def step_through(t: float) -> None:
        # Keep stepping at 240Hz up to time `t` so classify stays RUNNING.
        i = 0
        while i / 240.0 <= t + 1e-9:
            r.note_step(now=i / 240.0)
            i += 1

    step_through(0.0)
    sl.maybe_log(now=0.0, registry=r, sim_time=0.0)  # arm
    step_through(0.5)
    sl.maybe_log(now=0.5, registry=r, sim_time=0.5)  # too early
    step_through(1.0)
    sl.maybe_log(now=1.0, registry=r, sim_time=1.0)  # emit
    step_through(1.9)
    sl.maybe_log(now=1.9, registry=r, sim_time=1.9)  # too early
    step_through(2.0)
    sl.maybe_log(now=2.0, registry=r, sim_time=2.0)  # emit
    assert len(emissions) == 2
    assert all(lvl == logging.INFO for lvl, _ in emissions)


def test_status_logger_warns_on_stall() -> None:
    emissions: list[tuple[int, str]] = []
    sl = StatusLogger(period_s=1.0, logger=lambda lvl, msg: emissions.append((lvl, msg)))
    r = _reg()
    r.note_step(now=0.0)
    sl.maybe_log(now=0.0, registry=r, sim_time=0.0)
    # Step is stale (>0.5s), so classify returns STALL and we emit at WARN.
    sl.maybe_log(now=1.5, registry=r, sim_time=0.0)
    assert len(emissions) == 1
    lvl, msg = emissions[0]
    assert lvl == logging.WARNING
    assert "STALL" in msg


def test_status_logger_resyncs_after_long_pause() -> None:
    """If we don't poll for many periods (e.g., paused), we shouldn't burst-emit."""
    emissions: list[tuple[int, str]] = []
    sl = StatusLogger(period_s=1.0, logger=lambda lvl, msg: emissions.append((lvl, msg)))
    r = _reg()
    r.note_step(now=0.0)
    sl.maybe_log(now=0.0, registry=r, sim_time=0.0)  # arm at t=0, deadline=1
    # Skip ahead 10s — only one emit, not 10.
    sl.maybe_log(now=10.0, registry=r, sim_time=0.0)
    sl.maybe_log(now=10.1, registry=r, sim_time=0.0)
    assert len(emissions) == 1
    # And the next deadline has been resynced near 10+1=11.
    sl.maybe_log(now=11.05, registry=r, sim_time=0.0)
    assert len(emissions) == 2


# ---------- format_status -------------------------------------------------


def test_format_status_includes_all_metrics() -> None:
    r = _reg()
    for i in range(240):
        r.note_step(now=i / 240)
    snap = r.snapshot(now=1.0, sim_time=1.0)
    s = format_status(snap)
    assert "sim 1.00s" in s
    assert "step" in s and "Hz" in s
    assert "rt" in s
    assert "cmd" in s
    assert "pub" in s
    assert "render" in s
    assert f"state={snap.state}" in s


# ---------- PhaseTimer ----------------------------------------------------


def test_phase_timer_empty() -> None:
    pt = PhaseTimer()
    assert pt.avg(now=0.0) == 0.0
    assert pt.max(now=0.0) == 0.0
    assert pt.count(now=0.0) == 0


def test_phase_timer_avg_and_max() -> None:
    pt = PhaseTimer(window_s=1.0)
    pt.sample(0.001, now=0.1)
    pt.sample(0.003, now=0.2)
    pt.sample(0.002, now=0.3)
    assert pt.count(now=0.5) == 3
    assert pt.avg(now=0.5) == pytest.approx(0.002)
    assert pt.max(now=0.5) == pytest.approx(0.003)


def test_phase_timer_drops_old_samples() -> None:
    pt = PhaseTimer(window_s=1.0)
    pt.sample(0.005, now=0.0)  # will fall out of window
    pt.sample(0.001, now=2.0)
    pt.sample(0.002, now=2.5)
    assert pt.count(now=2.7) == 2
    assert pt.max(now=2.7) == pytest.approx(0.002)


def test_phase_timer_rejects_zero_window() -> None:
    with pytest.raises(ValueError):
        PhaseTimer(window_s=0)


# ---------- TelemetryRegistry.note_phase / snapshot phases ----------------


def test_registry_note_phase_lazy_creation() -> None:
    r = _reg()
    r.note_phase("step", 0.001, now=0.1)
    r.note_phase("publish_state", 0.002, now=0.1)
    r.note_phase("step", 0.003, now=0.2)
    snap = r.snapshot(now=0.5, sim_time=0.5)
    assert snap.phases_avg_s is not None
    assert snap.phases_avg_s["step"] == pytest.approx(0.002)
    assert snap.phases_avg_s["publish_state"] == pytest.approx(0.002)
    assert snap.phases_max_s["step"] == pytest.approx(0.003)


def test_registry_snapshot_no_phases_when_unused() -> None:
    r = _reg()
    snap = r.snapshot(now=0.5, sim_time=0.5)
    assert snap.phases_avg_s is None
    assert snap.phases_max_s is None
    # as_dict should not contain any phase_* keys.
    assert not any(k.startswith("phase_") for k in snap.as_dict())


def test_registry_snapshot_phase_keys_in_dict() -> None:
    r = _reg()
    r.note_phase("step", 0.0015, now=0.1)
    r.note_phase("publish_state", 0.0035, now=0.1)
    d = r.snapshot(now=0.5, sim_time=0.5).as_dict()
    assert d["phase_step_avg_ms"] == pytest.approx(1.5)
    assert d["phase_publish_state_avg_ms"] == pytest.approx(3.5)
    assert d["phase_step_max_ms"] == pytest.approx(1.5)


def test_format_status_appends_phase_line_when_present() -> None:
    r = _reg()
    r.note_step(now=0.5)
    r.note_phase("step", 0.0008, now=0.5)
    r.note_phase("publish_state", 0.0032, now=0.5)
    r.note_phase("publish_snapshot", 0.0006, now=0.5)
    snap = r.snapshot(now=0.6, sim_time=0.6)
    s = format_status(snap)
    assert "\n[newton_bridge] phases:" in s
    # Sorted by avg desc → publish_state should come before step in the line.
    phase_line = s.split("\n", 1)[1]
    assert phase_line.index("publish_state") < phase_line.index("step ")
    assert "3.20ms" in phase_line
    assert "0.80ms" in phase_line


def test_format_status_omits_phase_line_when_no_phases() -> None:
    r = _reg()
    r.note_step(now=0.5)
    snap = r.snapshot(now=0.6, sim_time=0.6)
    s = format_status(snap)
    assert "phases:" not in s
    assert "\n" not in s
