"""Double-buffered state snapshot for main → viewer thread handoff.

The producer (physics/main thread) calls `publish(state, sim_time, telemetry)`
after each step. The consumer (viewer thread) calls `consume()` to obtain a
`SnapshotView` that references one of two pre-allocated `newton.State`
buffers. The producer writes into whichever buffer the consumer is not
currently reading, so the consumer always sees a consistent state.

Design notes
------------
* Two `State` slots are allocated up-front by the caller (kept Newton-free
  here so host-side pytest does not need warp/newton). The snapshot copies
  the live state into the back buffer with `back.assign(live_state)` — an
  in-place GPU memcpy in Newton 1.1.0, no per-step allocation.
* A short critical section (`threading.Lock`) guards only index bookkeeping
  and metadata reads; the heavy GPU work in `assign()` happens outside the
  lock.
* `consume()` returns `None` when no new publish has happened since the last
  consume — viewer threads can poll cheaply.
* `SnapshotView` carries `sim_time`, `publish_wall`, optional `telemetry`,
  and an `age()` helper. `release()` (or context manager exit) frees the
  slot back to the producer.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SnapshotView:
    """One read of a published state. Hold via context manager or call `release()`.

    While held, the producer will not overwrite `state` — it writes into the
    other buffer instead. Holding for too long stalls the producer's swap if
    a publish lands while the view is in use; the producer waits only as long
    as the lock is held during a swap (microseconds), not for the view's life.
    """

    state: Any
    sim_time: float
    publish_wall: float
    telemetry: dict[str, Any] | None
    seq: int
    _owner: "StateSnapshot | None" = field(default=None, repr=False)

    def age(self, now: float | None = None) -> float:
        """Wall-clock seconds since the producer published this snapshot."""
        t = time.monotonic() if now is None else float(now)
        return t - self.publish_wall

    def release(self) -> None:
        owner, self._owner = self._owner, None
        if owner is not None:
            owner._release(self)

    def __enter__(self) -> "SnapshotView":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.release()


class StateSnapshot:
    """Single-producer / single-consumer double-buffered state handoff.

    Parameters
    ----------
    buf_a, buf_b : newton.State
        Two pre-allocated state buffers. The snapshot copies the live state
        into one of them on each `publish`, alternating so the consumer can
        always read a coherent buffer.
    """

    def __init__(self, buf_a: Any, buf_b: Any) -> None:
        self._buffers: tuple[Any, Any] = (buf_a, buf_b)
        # _front_idx: which buffer the latest publish lives in. None until first publish.
        self._front_idx: int | None = None
        self._reader_idx: int | None = None  # buffer currently held by a SnapshotView
        self._sim_time: float = 0.0
        self._publish_wall: float = 0.0
        self._telemetry: dict[str, Any] | None = None
        self._seq: int = 0  # incremented on each publish; consumer compares to detect new
        self._last_consumed_seq: int = 0
        self._lock = threading.Lock()

    # ---------- producer side -------------------------------------------------

    def publish(
        self,
        state: Any,
        sim_time: float,
        telemetry: dict[str, Any] | None = None,
        *,
        now: float | None = None,
    ) -> None:
        """Copy `state` into the back buffer and atomically promote it to front.

        The back buffer is whichever slot the consumer is not currently
        reading. If the consumer holds the only "free" slot we still pick the
        other buffer (the consumer's slot is sacrosanct), which means the
        consumer's previous front becomes stale but is never torn.
        """
        with self._lock:
            back_idx = self._pick_back_idx_locked()

        # Heavy work (GPU memcpy) outside the lock so consumers can keep reading.
        self._buffers[back_idx].assign(state)
        wall = time.monotonic() if now is None else float(now)

        with self._lock:
            self._front_idx = back_idx
            self._sim_time = float(sim_time)
            self._publish_wall = wall
            self._telemetry = telemetry
            self._seq += 1

    def _pick_back_idx_locked(self) -> int:
        # Never write into the slot the reader holds.
        if self._reader_idx == 0:
            return 1
        if self._reader_idx == 1:
            return 0
        # No reader: alternate from current front (default to slot 0 on first call).
        if self._front_idx is None:
            return 0
        return 1 - self._front_idx

    # ---------- consumer side -------------------------------------------------

    def consume(self) -> SnapshotView | None:
        """Return the latest published snapshot, or None if nothing new.

        Caller must `release()` (or use as a context manager) so the producer
        can reuse the slot.
        """
        with self._lock:
            if self._front_idx is None:
                return None
            if self._seq == self._last_consumed_seq:
                return None
            if self._reader_idx is not None:
                # Caller didn't release the previous view. Refuse rather than
                # silently double-claim a buffer; tests cover this path.
                raise RuntimeError(
                    "previous SnapshotView not released; call release() or use a `with` block"
                )
            idx = self._front_idx
            self._reader_idx = idx
            self._last_consumed_seq = self._seq
            view = SnapshotView(
                state=self._buffers[idx],
                sim_time=self._sim_time,
                publish_wall=self._publish_wall,
                telemetry=self._telemetry,
                seq=self._seq,
                _owner=self,
            )
        return view

    def _release(self, view: SnapshotView) -> None:
        with self._lock:
            if self._reader_idx is not None and self._buffers[self._reader_idx] is view.state:
                self._reader_idx = None

    # ---------- introspection (mostly for tests) -----------------------------

    @property
    def published_seq(self) -> int:
        with self._lock:
            return self._seq

    @property
    def has_unread(self) -> bool:
        with self._lock:
            return self._front_idx is not None and self._seq != self._last_consumed_seq
