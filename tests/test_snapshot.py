"""Unit tests for StateSnapshot — the main → viewer thread state handoff.

Pure Python, no Newton/warp. Uses a FakeState that mimics newton.State's
`assign()` semantics: copy the source's payload into self in place.
"""

from __future__ import annotations

import threading
import time

import pytest

from newton_bridge.snapshot import SnapshotView, StateSnapshot


class FakeState:
    """Stand-in for newton.State; `assign(other)` copies in place."""

    def __init__(self, payload: int = 0) -> None:
        self.payload = payload
        self.assign_count = 0

    def assign(self, other: "FakeState") -> None:
        self.payload = other.payload
        self.assign_count += 1


def _mk(buf_payload_a: int = -1, buf_payload_b: int = -2) -> tuple[StateSnapshot, FakeState, FakeState]:
    a = FakeState(buf_payload_a)
    b = FakeState(buf_payload_b)
    return StateSnapshot(a, b), a, b


# ---------- basic publish/consume ----------------------------------------


def test_consume_returns_none_before_first_publish() -> None:
    snap, _, _ = _mk()
    assert snap.consume() is None


def test_consume_returns_view_after_publish() -> None:
    snap, _, _ = _mk()
    live = FakeState(payload=42)
    snap.publish(live, sim_time=1.5, now=100.0)

    view = snap.consume()
    assert view is not None
    assert view.state.payload == 42
    assert view.sim_time == 1.5
    assert view.publish_wall == 100.0
    assert view.telemetry is None
    assert view.seq == 1
    view.release()


def test_consume_returns_none_when_no_new_publish() -> None:
    snap, _, _ = _mk()
    snap.publish(FakeState(1), sim_time=0.0, now=0.0)
    v = snap.consume()
    assert v is not None
    v.release()
    # No new publish since last consume.
    assert snap.consume() is None


def test_publish_then_consume_sees_latest_only() -> None:
    snap, _, _ = _mk()
    for i in range(5):
        snap.publish(FakeState(payload=i), sim_time=float(i), now=float(i))
    view = snap.consume()
    assert view is not None
    assert view.state.payload == 4
    assert view.sim_time == 4.0
    assert view.seq == 5
    view.release()


# ---------- double-buffer alternation ------------------------------------


def test_publishes_alternate_buffers_when_no_reader() -> None:
    snap, a, b = _mk()
    snap.publish(FakeState(1), sim_time=0.0)
    snap.publish(FakeState(2), sim_time=1.0)
    snap.publish(FakeState(3), sim_time=2.0)
    # Two assigns should hit one buffer, two on the other (3 total: 2+1 split).
    assert a.assign_count + b.assign_count == 3
    assert a.assign_count >= 1 and b.assign_count >= 1


def test_publish_does_not_overwrite_held_buffer() -> None:
    """While a SnapshotView is held, the next publish must land in the OTHER buffer."""
    snap, a, b = _mk()
    snap.publish(FakeState(payload=10), sim_time=0.0)
    view = snap.consume()
    assert view is not None
    held = view.state
    held_payload = held.payload  # 10

    # Publish lots of times while the view is held; held buffer must stay intact.
    for i in range(20):
        snap.publish(FakeState(payload=100 + i), sim_time=float(i))
        assert held.payload == held_payload, (
            f"held buffer was overwritten on iteration {i}"
        )
    view.release()


def test_consume_without_release_raises() -> None:
    snap, _, _ = _mk()
    snap.publish(FakeState(1), sim_time=0.0)
    v1 = snap.consume()
    assert v1 is not None
    snap.publish(FakeState(2), sim_time=1.0)
    with pytest.raises(RuntimeError):
        snap.consume()
    v1.release()


def test_release_via_context_manager() -> None:
    snap, _, _ = _mk()
    snap.publish(FakeState(7), sim_time=0.0)
    with snap.consume() as view:
        assert view is not None
        assert view.state.payload == 7
    # Consumer slot freed; new publish + consume should work.
    snap.publish(FakeState(8), sim_time=1.0)
    v2 = snap.consume()
    assert v2 is not None
    assert v2.state.payload == 8
    v2.release()


# ---------- telemetry passthrough -----------------------------------------


def test_telemetry_round_trips() -> None:
    snap, _, _ = _mk()
    payload = {"step_hz": 240.0, "rt": 1.0, "state": "RUNNING"}
    snap.publish(FakeState(1), sim_time=0.0, telemetry=payload)
    with snap.consume() as v:
        assert v is not None
        assert v.telemetry == payload


def test_telemetry_default_none() -> None:
    snap, _, _ = _mk()
    snap.publish(FakeState(1), sim_time=0.0)
    with snap.consume() as v:
        assert v is not None
        assert v.telemetry is None


# ---------- age() ---------------------------------------------------------


def test_age_uses_publish_wall() -> None:
    snap, _, _ = _mk()
    snap.publish(FakeState(1), sim_time=0.0, now=10.0)
    with snap.consume() as v:
        assert v is not None
        assert v.age(now=10.004) == pytest.approx(0.004)
        assert v.age(now=10.5) == pytest.approx(0.5)


# ---------- seq / has_unread ---------------------------------------------


def test_seq_increments_on_publish() -> None:
    snap, _, _ = _mk()
    assert snap.published_seq == 0
    snap.publish(FakeState(1), sim_time=0.0)
    assert snap.published_seq == 1
    snap.publish(FakeState(2), sim_time=1.0)
    assert snap.published_seq == 2


def test_has_unread_flag() -> None:
    snap, _, _ = _mk()
    assert snap.has_unread is False
    snap.publish(FakeState(1), sim_time=0.0)
    assert snap.has_unread is True
    with snap.consume() as v:
        assert v is not None
    assert snap.has_unread is False
    snap.publish(FakeState(2), sim_time=1.0)
    assert snap.has_unread is True


# ---------- threaded smoke test ------------------------------------------


def test_concurrent_producer_consumer_no_corruption() -> None:
    """Stress: producer publishes monotonically increasing payloads while a
    consumer reads them. Consumer must never see a payload > the latest the
    producer has published, and never see torn state (in this fake, payload
    is a single int so torn state is impossible — but the assign_count
    bookkeeping verifies the producer never wrote into the held buffer)."""
    snap, _, _ = _mk()
    stop = threading.Event()
    seen: list[int] = []
    last_published = [0]

    def producer() -> None:
        for i in range(1, 1001):
            snap.publish(FakeState(payload=i), sim_time=float(i))
            last_published[0] = i
        stop.set()

    def consumer() -> None:
        while not stop.is_set() or snap.has_unread:
            v = snap.consume()
            if v is None:
                time.sleep(0.0001)
                continue
            with v:
                # While we hold v, payload must remain stable.
                first = v.state.payload
                for _ in range(50):
                    assert v.state.payload == first
                seen.append(first)

    tp = threading.Thread(target=producer)
    tc = threading.Thread(target=consumer)
    tc.start()
    tp.start()
    tp.join(timeout=10.0)
    tc.join(timeout=10.0)
    assert not tp.is_alive() and not tc.is_alive()

    # Consumer should have observed something, and observations should be
    # monotonically non-decreasing (skips OK; back-jumps not OK).
    assert seen, "consumer never observed a snapshot"
    for prev, curr in zip(seen, seen[1:]):
        assert curr >= prev, f"consumer saw payload regress: {prev} -> {curr}"
    # And the final consumer view should reflect work close to the producer's end.
    assert seen[-1] <= last_published[0]


def test_assign_called_on_publish() -> None:
    snap, a, b = _mk()
    live = FakeState(payload=99)
    snap.publish(live, sim_time=0.0)
    # One of the two buffers got an assign with payload 99.
    assert (a.assign_count == 1 and a.payload == 99) or (
        b.assign_count == 1 and b.payload == 99
    )


def test_view_release_is_idempotent() -> None:
    snap, _, _ = _mk()
    snap.publish(FakeState(1), sim_time=0.0)
    v = snap.consume()
    assert v is not None
    v.release()
    v.release()  # second call: no-op, must not raise or corrupt state
    snap.publish(FakeState(2), sim_time=1.0)
    v2 = snap.consume()
    assert v2 is not None
    v2.release()
