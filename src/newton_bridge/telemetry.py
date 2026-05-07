"""Telemetry primitives: rate meters, registry, status logger.

Once the viewer runs in its own thread (`viewer_thread.ViewerThread`), the
fact that the viewer is rendering no longer implies physics is alive — and
in sync mode, "physics not stepping" can mean either healthy idle (no command
has arrived yet) or an actual stall. This module exposes:

* `RateMeter` — sliding-window Hz over a fixed window (default 1s).
* `TelemetryRegistry` — owns step/cmd/pub/render meters, classifies the
  pipeline state, and serializes a per-tick `snapshot()` dict that's shipped
  alongside `StateSnapshot` and to ROS `/sim/diagnostics`.
* `StatusLogger` — emits one terminal line per second; WARN level on STALL.

Pure-Python so host pytest can exercise the timing logic with a fake clock.
"""

from __future__ import annotations

import collections
import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable


# Pipeline state values surfaced to logs / diagnostics / overlays.
STATE_INIT = "INIT"
STATE_IDLE = "IDLE"
STATE_STALL = "STALL"
STATE_RUNNING = "RUNNING"


class RateMeter:
    """Tracks events-per-second over a sliding window.

    The window defaults to 1.0s; any tick older than `window_s` before `now`
    is dropped on the next `rate()` call. `tick()` is O(1) amortized.

    Thread safety: producer (physics/main) calls `tick()` while consumer
    (viewer thread) may concurrently call `rate()` for the HUD. `rate()`
    mutates the deque (popleft of stale stamps), so all access is guarded
    by an internal lock. The lock is uncontended in practice — both sides
    call ~hundreds of times/sec at most.
    """

    def __init__(self, window_s: float = 1.0) -> None:
        if window_s <= 0:
            raise ValueError(f"window_s must be > 0 (got {window_s})")
        self.window_s = float(window_s)
        self._stamps: collections.deque[float] = collections.deque()
        self._last_tick: float | None = None
        self._lock = threading.Lock()

    def tick(self, now: float) -> None:
        with self._lock:
            self._stamps.append(float(now))
            self._last_tick = float(now)

    def rate(self, now: float) -> float:
        cutoff = float(now) - self.window_s
        with self._lock:
            while self._stamps and self._stamps[0] < cutoff:
                self._stamps.popleft()
            return len(self._stamps) / self.window_s

    @property
    def last_tick(self) -> float | None:
        return self._last_tick

    def time_since_last(self, now: float) -> float | None:
        last = self._last_tick
        if last is None:
            return None
        return float(now) - last


class PhaseTimer:
    """Sliding-window per-phase wall-time tracker.

    Producer calls `sample(dt_seconds, now)` once per occurrence; consumer
    reads `avg(now)` / `max(now)` over the trailing `window_s`. Same locking
    model as RateMeter — a single internal lock guards the deque.
    """

    def __init__(self, window_s: float = 1.0) -> None:
        if window_s <= 0:
            raise ValueError(f"window_s must be > 0 (got {window_s})")
        self.window_s = float(window_s)
        # entries: (timestamp, dt_seconds)
        self._samples: collections.deque[tuple[float, float]] = collections.deque()
        self._lock = threading.Lock()

    def sample(self, dt_seconds: float, now: float) -> None:
        with self._lock:
            self._samples.append((float(now), float(dt_seconds)))

    def _evict(self, now: float) -> None:
        cutoff = float(now) - self.window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def avg(self, now: float) -> float:
        with self._lock:
            self._evict(now)
            if not self._samples:
                return 0.0
            return sum(dt for _, dt in self._samples) / len(self._samples)

    def max(self, now: float) -> float:
        with self._lock:
            self._evict(now)
            if not self._samples:
                return 0.0
            return max(dt for _, dt in self._samples)

    def count(self, now: float) -> int:
        with self._lock:
            self._evict(now)
            return len(self._samples)


@dataclass
class TelemetrySnapshot:
    """Plain-data view of the registry, safe to ship across threads / topics."""

    sim_time: float
    step_count: int
    step_hz: float
    realtime_factor: float
    cmd_hz: float
    pub_hz: float
    render_hz: float
    time_since_cmd: float | None
    time_since_step: float | None
    state: str
    # Per-phase wall time (seconds). Empty until any sample lands.
    phases_avg_s: dict[str, float] | None = None
    phases_max_s: dict[str, float] | None = None

    def as_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "sim_time": self.sim_time,
            "step_count": self.step_count,
            "step_hz": self.step_hz,
            "rt_factor": self.realtime_factor,
            "cmd_hz": self.cmd_hz,
            "pub_hz": self.pub_hz,
            "render_hz": self.render_hz,
            "time_since_cmd": self.time_since_cmd,
            "time_since_step": self.time_since_step,
            "state": self.state,
        }
        if self.phases_avg_s:
            for name, v in self.phases_avg_s.items():
                d[f"phase_{name}_avg_ms"] = v * 1000.0
        if self.phases_max_s:
            for name, v in self.phases_max_s.items():
                d[f"phase_{name}_max_ms"] = v * 1000.0
        return d


class TelemetryRegistry:
    """Bundles all rate meters + state classification.

    The producer (physics/main) calls `note_step`, `note_cmd`, `note_pub`.
    The viewer thread calls `note_render`. Either side can call `snapshot()`
    to get a self-consistent view at a given `now`.
    """

    # Stall thresholds (plan §"State classification").
    STALL_FLOOR_S = 0.5  # GC/swap false-positive guard
    STALL_DT_MULTIPLIER = 50  # ceiling scales with sim rate

    def __init__(
        self,
        physics_dt: float,
        *,
        sync_mode: bool = False,
        sync_timeout_s: float = 0.5,
        window_s: float = 1.0,
    ) -> None:
        if physics_dt <= 0:
            raise ValueError(f"physics_dt must be > 0 (got {physics_dt})")
        self.physics_dt = float(physics_dt)
        self.sync_mode = bool(sync_mode)
        self.sync_timeout_s = float(sync_timeout_s)

        self.step = RateMeter(window_s)
        self.cmd = RateMeter(window_s)
        self.pub = RateMeter(window_s)
        self.render = RateMeter(window_s)

        # Per-phase wall-time timers for the main physics tick. Phases are
        # added lazily via `note_phase` so callers don't need to register
        # names up front.
        self._phases: dict[str, PhaseTimer] = {}
        self._phase_window_s = float(window_s)

        self._step_count = 0
        # Tracks "cmd_rate>0 but step_rate==0" sub-stall — when this becomes
        # true we record the time so we can wait >1s before flipping to STALL.
        self._sync_stall_started: float | None = None

        # Cached snapshot for hot-path callers (see snapshot_cached). The
        # snapshot itself does Hz/phase aggregations across all RateMeters and
        # PhaseTimers; recomputing it every physics tick is wasted work since
        # the values it summarizes only change meaningfully on a 1Hz scale.
        # `_cached_dict` is also stashed so as_dict() doesn't have to re-flatten
        # phases on every cache hit — sim_time is patched into both on hit.
        self._cached_snap: TelemetrySnapshot | None = None
        self._cached_dict: dict[str, Any] | None = None
        self._cached_snap_at: float = 0.0

    # --- producer-side ticks --------------------------------------------------

    def note_step(self, now: float) -> None:
        self.step.tick(now)
        self._step_count += 1

    def note_cmd(self, now: float) -> None:
        self.cmd.tick(now)

    def note_pub(self, now: float) -> None:
        self.pub.tick(now)

    def note_render(self, now: float) -> None:
        self.render.tick(now)

    def note_phase(self, name: str, dt_seconds: float, now: float) -> None:
        timer = self._phases.get(name)
        if timer is None:
            timer = PhaseTimer(self._phase_window_s)
            self._phases[name] = timer
        timer.sample(dt_seconds, now)

    @property
    def step_count(self) -> int:
        return self._step_count

    # --- snapshot / classification -------------------------------------------

    def classify(self, now: float) -> str:
        """Return one of INIT/IDLE/STALL/RUNNING — see plan §state-classification."""
        if self.step.last_tick is None:
            return STATE_INIT

        time_since_step = now - self.step.last_tick
        cmd_rate = self.cmd.rate(now)
        step_rate = self.step.rate(now)

        # Sync-mode IDLE: command pipeline silent, watchdog window expired.
        if self.sync_mode:
            time_since_cmd = self.cmd.time_since_last(now)
            if (
                time_since_cmd is not None
                and time_since_cmd > self.sync_timeout_s
                and cmd_rate == 0.0
            ):
                # Reset the cmd-vs-step stall tracker: we're idle, not stalled.
                self._sync_stall_started = None
                return STATE_IDLE

        # Hard stall: physics hasn't stepped in too long.
        stall_threshold = max(
            self.STALL_FLOOR_S, self.STALL_DT_MULTIPLIER * self.physics_dt
        )
        if time_since_step > stall_threshold:
            return STATE_STALL

        # Sync-mode soft stall: commands flowing but steps aren't, for >1s.
        if self.sync_mode and cmd_rate > 0 and step_rate == 0:
            if self._sync_stall_started is None:
                self._sync_stall_started = now
            elif now - self._sync_stall_started > 1.0:
                return STATE_STALL
        else:
            self._sync_stall_started = None

        return STATE_RUNNING

    def snapshot_cached(
        self, now: float, sim_time: float, *, max_age_s: float = 1.0
    ) -> TelemetrySnapshot:
        """Like snapshot(), but reuse the previous result if it's fresh enough.

        Hot-path producers (per-physics-step) call this to avoid re-aggregating
        Hz/phase stats every tick. The returned object's `sim_time` is updated
        to the current value even on cache hits so consumers that only care
        about the latest sim_time still see live data; the rate/phase fields
        update at most every `max_age_s`.
        """
        cached = self._cached_snap
        if cached is not None and (now - self._cached_snap_at) < max_age_s:
            cached.sim_time = float(sim_time)
            return cached
        snap = self.snapshot(now, sim_time)
        self._cached_snap = snap
        self._cached_dict = None  # invalidate downstream dict cache
        self._cached_snap_at = now
        return snap

    def snapshot_dict_cached(
        self, now: float, sim_time: float, *, max_age_s: float = 1.0
    ) -> dict[str, Any]:
        """Cached `snapshot(...).as_dict()` for hot-path callers.

        Returns the same dict object across cache hits with `sim_time` updated
        in-place — caller must not mutate other keys. Saves both the snapshot
        re-aggregation and the as_dict() flatten step (~0.2ms combined).
        """
        snap = self.snapshot_cached(now, sim_time, max_age_s=max_age_s)
        d = self._cached_dict
        if d is None:
            d = snap.as_dict()
            self._cached_dict = d
        else:
            d["sim_time"] = float(sim_time)
        return d

    def snapshot(self, now: float, sim_time: float) -> TelemetrySnapshot:
        step_hz = self.step.rate(now)
        phases_avg: dict[str, float] = {}
        phases_max: dict[str, float] = {}
        for name, timer in self._phases.items():
            if timer.count(now) == 0:
                continue
            phases_avg[name] = timer.avg(now)
            phases_max[name] = timer.max(now)
        return TelemetrySnapshot(
            sim_time=float(sim_time),
            step_count=self._step_count,
            step_hz=step_hz,
            realtime_factor=step_hz * self.physics_dt,
            cmd_hz=self.cmd.rate(now),
            pub_hz=self.pub.rate(now),
            render_hz=self.render.rate(now),
            time_since_cmd=self.cmd.time_since_last(now),
            time_since_step=self.step.time_since_last(now),
            state=self.classify(now),
            phases_avg_s=phases_avg or None,
            phases_max_s=phases_max or None,
        )


class StatusLogger:
    """1Hz terminal status line; WARN level on STALL.

    Call `maybe_log(now, registry, sim_time)` from the main loop as often as
    you like — it self-throttles to `period_s`. `period_s=0` disables.
    """

    def __init__(
        self,
        period_s: float = 1.0,
        *,
        logger: logging.Logger | Callable[[int, str], None] | None = None,
    ) -> None:
        self.period_s = float(period_s)
        self._next_emit: float | None = None
        if logger is None:
            self._log = logging.getLogger("newton_bridge.telemetry")
            self._emit = self._emit_via_logger
        elif isinstance(logger, logging.Logger):
            self._log = logger
            self._emit = self._emit_via_logger
        else:
            # Test injection: callable(level, message)
            self._log = None
            self._user_emit = logger
            self._emit = self._emit_via_callable

    def _emit_via_logger(self, level: int, msg: str) -> None:
        assert self._log is not None
        self._log.log(level, msg)

    def _emit_via_callable(self, level: int, msg: str) -> None:
        self._user_emit(level, msg)

    @property
    def enabled(self) -> bool:
        return self.period_s > 0

    def maybe_log(
        self, now: float, registry: TelemetryRegistry, sim_time: float
    ) -> bool:
        """Emit if the period has elapsed; return True if a line was emitted."""
        if not self.enabled:
            return False
        if self._next_emit is None:
            self._next_emit = float(now) + self.period_s
            return False
        if now < self._next_emit:
            return False
        self._next_emit += self.period_s
        # If we fell way behind (pause/swap), resync to now+period.
        if self._next_emit < now:
            self._next_emit = float(now) + self.period_s

        snap = registry.snapshot(now, sim_time)
        level = logging.WARNING if snap.state == STATE_STALL else logging.INFO
        self._emit(level, format_status(snap))
        return True


def format_status(snap: TelemetrySnapshot) -> str:
    """Format a TelemetrySnapshot into the one-line status string from the plan.

    When per-phase timings are present, append a second line breaking down
    average wall time per tick phase (ms). Helps diagnose RTF<1 by pinpointing
    which phase blew the per-tick budget.
    """
    parts = [
        f"sim {snap.sim_time:.2f}s",
        f"step {snap.step_hz:.0f}Hz (rt {snap.realtime_factor:.2f})",
        f"cmd {snap.cmd_hz:.0f}Hz",
        f"pub {snap.pub_hz:.0f}Hz",
        f"render {snap.render_hz:.0f}Hz",
        f"state={snap.state}",
    ]
    line = "[newton_bridge] " + " | ".join(parts)
    if snap.phases_avg_s:
        # Stable order: largest avg first (most actionable for the operator).
        items = sorted(
            snap.phases_avg_s.items(), key=lambda kv: kv[1], reverse=True
        )
        phase_parts = [f"{name} {dt * 1000.0:.2f}ms" for name, dt in items]
        line += "\n[newton_bridge] phases: " + " | ".join(phase_parts)
    return line
