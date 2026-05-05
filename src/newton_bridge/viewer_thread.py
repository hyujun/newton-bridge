"""ViewerThread — runs the Newton viewer in its own daemon thread.

The main thread builds the world, steps physics, services ROS, and calls
`StateSnapshot.publish()` after every step. A ViewerThread spins on the side,
consumes snapshots, and renders at viewer_hz wall-clock. This keeps physics
and viewer independent: a slow viewer can't stall physics, and the viewer
keeps animating at viewer_hz even when physics is idle in sync mode.

Lifecycle
---------
1. `start()` launches the daemon thread.
2. The thread calls `viewer_factory()` (so any GL context lives on the
   viewer thread — decision #3 of the thread-separation plan).
3. Caller waits on `ready_event` to know whether the viewer built or failed
   (`build_error` is populated on failure).
4. Main loop polls `closed_event`, `is_paused()`, `consume_step_request()`,
   `consume_reset_request()`, and `is_key_down(key)`.
5. `stop()` sets `_shutdown_event`, joins the thread, closes the viewer.

Communication channels (matches plan section "Communication channels"):

| Signal              | Direction           | Mechanism                              |
|---------------------|---------------------|----------------------------------------|
| state snapshot      | main → viewer       | StateSnapshot double buffer            |
| closed (ESC/window) | viewer → main       | threading.Event (closed_event)         |
| paused              | viewer → main       | threading.Event (paused_event)         |
| single-step pulse   | viewer → main       | queue.Queue(maxsize=1)                 |
| reset button        | viewer → main       | queue.Queue(maxsize=1)                 |
| shutdown            | main → viewer       | threading.Event (_shutdown_event)      |
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Any, Callable

from .snapshot import StateSnapshot
from .ticks import RenderTicker

# Floor for viewer_hz=0 passthrough so the thread doesn't busy-spin (decision #4).
MIN_SLEEP = 0.0005


log = logging.getLogger(__name__)


class ViewerThread:
    """Owns a daemon thread that drives a Newton viewer at viewer_hz.

    Parameters
    ----------
    snapshot : StateSnapshot
        Shared double-buffered handoff. The viewer thread polls
        `snapshot.consume()` each render tick.
    viewer_factory : Callable[[], viewer | None]
        Called inside the thread to build the viewer. Returning None disables
        rendering (mode='none') but the thread still runs to honor input.
        Most callers pass a closure: `lambda: build_viewer(world, mode)`.
    viewer_hz : float | None
        Render rate. 0 or None = passthrough (render every loop, throttled
        only by MIN_SLEEP).
    sleeper : Callable[[float], None]
        Injection point for tests; defaults to `time.sleep`.
    clock : Callable[[], float]
        Injection point for tests; defaults to `time.monotonic`.
    """

    def __init__(
        self,
        snapshot: StateSnapshot,
        viewer_factory: Callable[[], Any],
        viewer_hz: float | None = 60.0,
        *,
        sleeper: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._snapshot = snapshot
        self._factory = viewer_factory
        self._viewer_hz = viewer_hz
        self._sleep = sleeper
        self._clock = clock

        # Lifecycle / signalling
        self.ready_event = threading.Event()
        self.closed_event = threading.Event()
        self.paused_event = threading.Event()
        self._shutdown_event = threading.Event()

        # Step / reset pulses. maxsize=1 + put_nowait(ignore-on-full) gives
        # exactly-once semantics per keypress at the consume side: extra
        # pulses while one is already pending coalesce into a single trigger.
        self._step_q: queue.Queue[bool] = queue.Queue(maxsize=1)
        self._reset_q: queue.Queue[bool] = queue.Queue(maxsize=1)

        # Viewer reference + lock. The viewer is built inside the thread but
        # main-thread helpers (is_key_down) need to read it safely.
        self._viewer: Any = None
        self._viewer_lock = threading.Lock()

        # Build error surface — checked by caller after ready_event fires.
        self.build_error: BaseException | None = None

        self._thread = threading.Thread(
            target=self._run,
            name="newton-viewer",
            daemon=True,
        )

    # ---------- lifecycle ----------------------------------------------------

    def start(self) -> None:
        self._thread.start()

    def wait_ready(self, timeout: float | None = None) -> bool:
        """Block until the viewer has been built (or failed). Returns True if ready."""
        return self.ready_event.wait(timeout=timeout)

    def stop(self, timeout: float | None = 0.5) -> None:
        """Signal the viewer thread to stop and best-effort join.

        The thread is a daemon, so if join times out the process can still
        exit — we prefer a snappy ^C over guaranteed clean teardown of GL
        resources. A render in progress (begin_frame/log_state/end_frame)
        may hold the thread inside a GPU call past the timeout; that's fine.
        """
        self._shutdown_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)

    @property
    def is_alive(self) -> bool:
        return self._thread.is_alive()

    # ---------- main-thread queries -----------------------------------------

    def is_paused(self) -> bool:
        return self.paused_event.is_set()

    def closed_externally(self) -> bool:
        """True if viewer signalled ESC / window close (not shutdown_event)."""
        return self.closed_event.is_set()

    def consume_step_request(self) -> bool:
        """Pop one pending '.' keypress, return True if there was one."""
        try:
            self._step_q.get_nowait()
            return True
        except queue.Empty:
            return False

    def consume_reset_request(self) -> bool:
        """Pop one pending UI Reset, return True if there was one."""
        try:
            self._reset_q.get_nowait()
            return True
        except queue.Empty:
            return False

    def is_key_down(self, key: Any) -> bool:
        """Pass-through to viewer.is_key_down (decision #5). False if no viewer."""
        with self._viewer_lock:
            v = self._viewer
        if v is None:
            return False
        fn = getattr(v, "is_key_down", None)
        if fn is None:
            return False
        try:
            return bool(fn(key))
        except Exception:
            return False

    # ---------- thread body --------------------------------------------------

    def _run(self) -> None:
        try:
            viewer = self._factory()
        except BaseException as e:  # noqa: BLE001 — surface anything to main
            self.build_error = e
            self.ready_event.set()
            log.exception("viewer factory failed")
            return

        with self._viewer_lock:
            self._viewer = viewer

        # Wire UI Reset button (decision #5: registers in viewer thread,
        # signals via reset queue, main is responsible for actually resetting
        # via the existing /sim/reset path).
        if viewer is not None and hasattr(viewer, "set_reset_callback"):
            try:
                viewer.set_reset_callback(self._on_reset_clicked)
            except Exception:
                log.exception("set_reset_callback failed; UI Reset will not work")

        self.ready_event.set()

        ticker = RenderTicker(render_hz=self._viewer_hz)

        try:
            while not self._shutdown_event.is_set():
                # Window-close / ESC → tell main to shut down.
                if viewer is not None:
                    if not _safe_call(viewer.is_running, default=True):
                        self.closed_event.set()
                        break

                    # SPACE pause state mirrored to main.
                    if _safe_call(viewer.is_paused, default=False):
                        self.paused_event.set()
                    else:
                        self.paused_event.clear()

                    # '.' single-step pulse.
                    if _safe_call(getattr(viewer, "should_step", lambda: False), default=False):
                        try:
                            self._step_q.put_nowait(True)
                        except queue.Full:
                            pass  # main hasn't consumed last pulse yet — coalesce

                # Render one frame if (a) we have a viewer, (b) ticker says go,
                # (c) snapshot has something fresh. Stale-but-no-new is fine —
                # we just skip; the viewer's own UI keeps interacting.
                if viewer is not None and ticker.tick(now=self._clock()):
                    self._render_one(viewer)

                # Sleep — never busy-spin even at viewer_hz=0.
                self._sleep(MIN_SLEEP if ticker.passthrough else MIN_SLEEP)
        finally:
            # Best-effort close (Newton viewers expose .close() on most modes).
            with self._viewer_lock:
                v = self._viewer
                self._viewer = None
            if v is not None:
                close = getattr(v, "close", None)
                if callable(close):
                    try:
                        close()
                    except Exception:
                        log.exception("viewer.close() failed")

    def _render_one(self, viewer: Any) -> None:
        view = self._snapshot.consume()
        if view is None:
            return
        with view:
            try:
                viewer.begin_frame(view.sim_time)
                viewer.log_state(view.state)
                # Telemetry overlay hook (filled in by PR #3).
                end = getattr(viewer, "end_frame", None)
                if callable(end):
                    end()
            except Exception:
                log.exception("viewer render failed")

    # ---------- viewer-thread → main signalling helpers ----------------------

    def _on_reset_clicked(self) -> None:
        """Called by the GL UI thread when the Reset button is clicked."""
        try:
            self._reset_q.put_nowait(True)
        except queue.Full:
            pass


def _safe_call(fn: Callable[[], Any], *, default: Any) -> Any:
    """Call a viewer method that may raise during teardown; swallow errors."""
    try:
        return fn()
    except Exception:
        return default
