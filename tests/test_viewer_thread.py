"""Unit tests for ViewerThread using a FakeViewer.

Pure Python — no Newton, no GL. The FakeViewer mimics the viewer surface
ViewerThread depends on (is_running, is_paused, should_step, set_reset_callback,
begin_frame/log_state/end_frame, is_key_down, close).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import pytest

from newton_bridge.snapshot import StateSnapshot
from newton_bridge.viewer_thread import ViewerThread


# ---------- fakes ---------------------------------------------------------


class FakeState:
    def __init__(self, payload: int = 0) -> None:
        self.payload = payload

    def assign(self, other: "FakeState") -> None:
        self.payload = other.payload


class FakeViewer:
    """Drives a ViewerThread end-to-end without GL/Rerun.

    All state is guarded by a lock so the test thread can flip flags
    (close, pause, request_step, click_reset) while the viewer thread polls.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running = True
        self._paused = False
        self._step_pending = False
        self._reset_cb: Callable[[], None] | None = None
        self._keys: set[Any] = set()
        self.frames: list[tuple[float, int]] = []  # (sim_time, payload)
        self.closed = False

    # --- viewer surface (called from viewer thread) ----------------------
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def is_paused(self) -> bool:
        with self._lock:
            return self._paused

    def should_step(self) -> bool:
        with self._lock:
            v = self._step_pending
            self._step_pending = False
            return v

    def set_reset_callback(self, cb: Callable[[], None]) -> None:
        with self._lock:
            self._reset_cb = cb

    def is_key_down(self, key: Any) -> bool:
        with self._lock:
            return key in self._keys

    def begin_frame(self, sim_time: float) -> None:
        self._pending_sim = sim_time

    def log_state(self, state: FakeState) -> None:
        self._pending_payload = state.payload

    def end_frame(self) -> None:
        self.frames.append((self._pending_sim, self._pending_payload))

    def close(self) -> None:
        self.closed = True

    # --- test-side controls (called from test/main thread) ---------------
    def request_close(self) -> None:
        with self._lock:
            self._running = False

    def set_paused(self, p: bool) -> None:
        with self._lock:
            self._paused = p

    def request_step(self) -> None:
        with self._lock:
            self._step_pending = True

    def click_reset(self) -> None:
        with self._lock:
            cb = self._reset_cb
        if cb is not None:
            cb()

    def press_key(self, key: Any) -> None:
        with self._lock:
            self._keys.add(key)

    def release_key(self, key: Any) -> None:
        with self._lock:
            self._keys.discard(key)


def _make_thread(
    viewer: Any | Exception | None,
    viewer_hz: float | None = 1000.0,
) -> tuple[ViewerThread, StateSnapshot]:
    snap = StateSnapshot(FakeState(-1), FakeState(-2))

    def factory() -> Any:
        if isinstance(viewer, BaseException):
            raise viewer
        return viewer

    vt = ViewerThread(snap, factory, viewer_hz=viewer_hz)
    return vt, snap


def _wait_until(pred: Callable[[], bool], timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.005)
    return False


# ---------- lifecycle ----------------------------------------------------


def test_starts_and_stops_cleanly() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    assert vt.wait_ready(timeout=1.0)
    assert vt.build_error is None
    assert vt.is_alive
    vt.stop(timeout=1.0)
    assert not vt.is_alive
    assert fv.closed is True  # close() called on shutdown


def test_factory_failure_surfaces_to_main() -> None:
    boom = RuntimeError("GL init failed")
    vt, _ = _make_thread(boom)
    vt.start()
    assert vt.wait_ready(timeout=1.0)
    assert isinstance(vt.build_error, RuntimeError)
    assert "GL init failed" in str(vt.build_error)
    vt.stop(timeout=1.0)


def test_none_viewer_thread_still_runs_and_stops() -> None:
    """mode='none' returns None; thread should idle without rendering and stop cleanly."""
    vt, _ = _make_thread(None)
    vt.start()
    assert vt.wait_ready(timeout=1.0)
    assert vt.build_error is None
    # No viewer → no closed signal, no paused.
    time.sleep(0.05)
    assert not vt.closed_externally()
    assert not vt.is_paused()
    vt.stop(timeout=1.0)
    assert not vt.is_alive


# ---------- closed / paused signalling -----------------------------------


def test_window_close_sets_closed_event() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    fv.request_close()
    assert _wait_until(vt.closed_externally, timeout=1.0)
    vt.stop(timeout=1.0)


def test_pause_state_mirrored() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    assert not vt.is_paused()
    fv.set_paused(True)
    assert _wait_until(vt.is_paused, timeout=1.0)
    fv.set_paused(False)
    assert _wait_until(lambda: not vt.is_paused(), timeout=1.0)
    vt.stop(timeout=1.0)


# ---------- step / reset pulses ------------------------------------------


def test_step_request_consumed_once() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    fv.request_step()
    assert _wait_until(vt.consume_step_request, timeout=1.0)
    # No more pending — second consume should be False.
    assert vt.consume_step_request() is False
    vt.stop(timeout=1.0)


def test_repeated_step_requests_coalesce() -> None:
    """Plan §"Open risks": maxsize=1 + put_nowait means rapid presses don't queue up."""
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    for _ in range(50):
        fv.request_step()
    # Even with 50 requests, main consumes at most 1 per tick — but it must
    # eventually drain to False.
    saw = False
    for _ in range(200):
        if vt.consume_step_request():
            saw = True
        else:
            if saw:
                break
        time.sleep(0.002)
    assert saw
    assert vt.consume_step_request() is False
    vt.stop(timeout=1.0)


def test_reset_button_signals_via_queue() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    fv.click_reset()
    assert _wait_until(vt.consume_reset_request, timeout=1.0)
    assert vt.consume_reset_request() is False
    vt.stop(timeout=1.0)


# ---------- key passthrough ----------------------------------------------


def test_is_key_down_passthrough() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    assert vt.is_key_down("W") is False
    fv.press_key("W")
    # is_key_down asks the viewer directly each call — should pick up immediately.
    assert _wait_until(lambda: vt.is_key_down("W"), timeout=0.2)
    fv.release_key("W")
    assert _wait_until(lambda: not vt.is_key_down("W"), timeout=0.2)
    vt.stop(timeout=1.0)


def test_is_key_down_false_when_no_viewer() -> None:
    vt, _ = _make_thread(None)
    vt.start()
    vt.wait_ready(timeout=1.0)
    assert vt.is_key_down("W") is False
    vt.stop(timeout=1.0)


def test_is_key_down_after_stop() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    vt.stop(timeout=1.0)
    # Viewer reference cleared on shutdown → False, not a crash.
    assert vt.is_key_down("W") is False


# ---------- rendering ----------------------------------------------------


def test_renders_published_snapshots() -> None:
    fv = FakeViewer()
    vt, snap = _make_thread(fv, viewer_hz=1000.0)  # high cap so we don't wait long
    vt.start()
    vt.wait_ready(timeout=1.0)

    # Publish a few snapshots and let the viewer thread pick them up.
    for i in range(1, 6):
        snap.publish(FakeState(payload=i), sim_time=float(i))
        time.sleep(0.02)

    # Wait until at least one frame is rendered.
    assert _wait_until(lambda: len(fv.frames) >= 1, timeout=1.0)
    vt.stop(timeout=1.0)

    # Each rendered frame should match a real (sim_time, payload) pair we
    # published — nothing torn or fabricated. Sim times we sent: 1..5.
    for sim_t, payload in fv.frames:
        assert int(sim_t) == payload
        assert 1 <= payload <= 5


def test_does_not_render_when_no_snapshot() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv, viewer_hz=1000.0)
    vt.start()
    vt.wait_ready(timeout=1.0)
    time.sleep(0.05)
    vt.stop(timeout=1.0)
    assert fv.frames == []


def test_viewer_hz_zero_does_not_busy_spin() -> None:
    """Decision #4: viewer_hz=0 must enforce MIN_SLEEP between iterations."""
    fv = FakeViewer()
    vt, snap = _make_thread(fv, viewer_hz=0)
    vt.start()
    vt.wait_ready(timeout=1.0)

    # Drive the thread for 50ms; with MIN_SLEEP=0.5ms we'd expect at most ~100
    # iterations. We can't observe iteration count directly, but we can
    # observe rendered frames are bounded — publish 1 snapshot, see how many
    # times begin_frame is called (should be exactly 1, since after consume
    # there's nothing new).
    snap.publish(FakeState(payload=1), sim_time=0.0)
    time.sleep(0.05)
    vt.stop(timeout=1.0)
    assert len(fv.frames) == 1


# ---------- shutdown idempotence -----------------------------------------


def test_stop_is_idempotent() -> None:
    fv = FakeViewer()
    vt, _ = _make_thread(fv)
    vt.start()
    vt.wait_ready(timeout=1.0)
    vt.stop(timeout=1.0)
    vt.stop(timeout=1.0)  # second call: no-op
    assert not vt.is_alive
