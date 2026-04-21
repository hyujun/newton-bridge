"""Pure-Python tick helpers used by SimBridgeNode.

Kept dependency-free (no rclpy, warp, or newton) so host-side pytest can cover
the timing logic directly. The node wraps instances of these classes; the
classes themselves know nothing about ROS or physics.
"""

from __future__ import annotations

import time


class RenderTicker:
    """Throttles viewer render calls to a target wall-clock rate.

    Decoupled from physics progress: poll `tick()` as often as the caller
    likes and it fires True roughly every 1/render_hz seconds of wall time.
    This keeps viewer FPS at the configured rate whether the physics loop
    runs faster than realtime (sync mode with slow commands, FREERUN_RATE=max)
    or slower (freerun realtime missing its budget).

    First call returns True so the initial frame is drawn immediately.
    A render_hz of 0 or None disables throttling — every tick returns True.
    """

    def __init__(self, render_hz: float | None) -> None:
        self.render_hz = render_hz
        self._render_dt = 0.0 if not render_hz else 1.0 / float(render_hz)
        self._last_wall: float | None = None

    @property
    def passthrough(self) -> bool:
        return self._render_dt == 0.0

    @property
    def render_dt(self) -> float:
        return self._render_dt

    def tick(self, now: float | None = None) -> bool:
        if self.passthrough:
            return True
        t = time.monotonic() if now is None else float(now)
        if self._last_wall is None:
            self._last_wall = t
            return True
        elapsed = t - self._last_wall
        if elapsed + 1e-12 < self._render_dt:
            return False
        # Advance the deadline by exactly render_dt so long-run rate matches
        # render_hz. If we fell more than one period behind (hitch/pause),
        # resync to `now` to avoid a burst of catch-up frames.
        if elapsed > 2 * self._render_dt:
            self._last_wall = t
        else:
            self._last_wall += self._render_dt
        return True


class CommandWatchdog:
    """Tracks whether a /joint_command has arrived within the timeout window.

    `note_command(now)` records an activity timestamp. `is_stale(now)` returns
    True once `timeout_s` has elapsed since the last note. After a stale read
    the watchdog re-arms on the current `now` so the caller republishes at
    roughly timeout_s cadence rather than every tick.
    """

    def __init__(self, timeout_s: float) -> None:
        if timeout_s <= 0:
            raise ValueError(f"timeout_s must be > 0 (got {timeout_s})")
        self.timeout_s = float(timeout_s)
        self._last: float | None = None

    def note_command(self, now: float) -> None:
        self._last = float(now)

    def is_stale(self, now: float) -> bool:
        if self._last is None:
            # No activity ever — arm the clock so the first stale read happens
            # one full window from now, not immediately.
            self._last = float(now)
            return False
        if now - self._last >= self.timeout_s:
            self._last = float(now)
            return True
        return False
