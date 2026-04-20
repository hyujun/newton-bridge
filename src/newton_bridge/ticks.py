"""Pure-Python tick helpers used by SimBridgeNode.

Kept dependency-free (no rclpy, warp, or newton) so host-side pytest can cover
the timing logic directly. The node wraps instances of these classes; the
classes themselves know nothing about ROS or physics.
"""

from __future__ import annotations


class RenderTicker:
    """Throttles viewer render calls to a target rate independent of physics_dt.

    Call `tick(dt)` once per physics step (pass the physics_dt that was just
    integrated). Returns True when the accumulated sim time has crossed the
    render interval, meaning the caller should run one render frame.

    A render_hz of 0 or None disables throttling — every tick returns True.
    """

    def __init__(self, render_hz: float | None, physics_dt: float) -> None:
        if physics_dt <= 0:
            raise ValueError(f"physics_dt must be > 0 (got {physics_dt})")
        self.render_hz = render_hz
        self.physics_dt = physics_dt
        self._render_dt = 0.0 if not render_hz else 1.0 / float(render_hz)
        self._accum = 0.0

    @property
    def passthrough(self) -> bool:
        return self._render_dt == 0.0

    def tick(self, dt: float | None = None) -> bool:
        if self.passthrough:
            return True
        self._accum += self.physics_dt if dt is None else float(dt)
        if self._accum + 1e-12 >= self._render_dt:
            # Subtract rather than reset so fractional remainder carries over —
            # keeps long-run average at the requested render_hz.
            self._accum -= self._render_dt
            return True
        return False


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
