"""Unit tests for PickingOverlay (HUD + wireframe).

Pure-numpy fakes: no warp, no newton. The overlay's `_draw_lines` branch
that touches `warp` is bypassed by a FakeViewer.log_lines that captures
numpy arrays directly.
"""

from __future__ import annotations

import sys
import types
from typing import Any

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Stub `warp` so picking_overlay can import its `wp.array` call path. The
# overlay's _draw_lines wraps everything in try/except; tests assert on the
# log_lines call counts and arguments rather than touching warp directly.
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _stub_warp(monkeypatch):
    if "warp" in sys.modules:
        return
    fake = types.ModuleType("warp")

    class _FakeArray:
        def __init__(self, data, **_kwargs):
            self.data = np.asarray(data)

        def numpy(self):
            return self.data

    fake.array = lambda data, **kwargs: _FakeArray(data, **kwargs)
    fake.vec3 = "vec3"
    monkeypatch.setitem(sys.modules, "warp", fake)


# Late import so the warp stub is in place before module import.
from newton_bridge.picking_overlay import (  # noqa: E402
    PickingOverlay,
    _box_edges,
    _capsule_edges,
    _compose_transforms,
    _mesh_edges_from_indices,
    _sphere_edges,
    _transform_points,
)


# ---------- pure geometry helpers -----------------------------------------


def test_box_edges_count_and_extent():
    s, e = _box_edges(0.5, 1.0, 2.0)
    assert s.shape == (12, 3)
    assert e.shape == (12, 3)
    # The extreme corner should be reachable from at least one endpoint.
    assert np.any(np.all(np.isclose(s, [0.5, 1.0, 2.0]), axis=1) | np.all(np.isclose(e, [0.5, 1.0, 2.0]), axis=1))


def test_sphere_edges_three_great_circles():
    s, e = _sphere_edges(2.0, segments=12)
    # 3 axes × 12 segments
    assert s.shape == (36, 3)
    assert e.shape == (36, 3)
    # Points lie on radius 2.0 within tolerance.
    np.testing.assert_allclose(np.linalg.norm(s, axis=1), 2.0, atol=1e-5)


def test_capsule_edges_geometry():
    s, e = _capsule_edges(0.1, 0.5, segments=8)
    # 2 rings of 8 + 4 longitudinal
    assert s.shape[0] == 2 * 8 + 4
    assert s.shape == e.shape


def test_mesh_edges_dedup():
    verts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
    idxs = np.array([0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3], dtype=np.int32)  # tet, 4 faces
    s, e = _mesh_edges_from_indices(verts, idxs)
    # Tetrahedron has exactly 6 unique edges.
    assert s.shape == (6, 3)
    assert e.shape == (6, 3)


def test_transform_points_identity():
    pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    identity = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
    np.testing.assert_allclose(_transform_points(pts, identity), pts)


def test_transform_points_translation_only():
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    xf = np.array([10.0, 20.0, 30.0, 0, 0, 0, 1], dtype=np.float32)
    np.testing.assert_allclose(_transform_points(pts, xf), [[11.0, 20.0, 30.0]])


def test_transform_points_rotation_z_90():
    # Quaternion for 90deg about +Z: (x,y,z,w) = (0, 0, sin(45), cos(45))
    s2 = np.sqrt(0.5)
    xf = np.array([0, 0, 0, 0, 0, s2, s2], dtype=np.float32)
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    out = _transform_points(pts, xf)
    np.testing.assert_allclose(out, [[0.0, 1.0, 0.0]], atol=1e-5)


def test_compose_transforms_pure_translation():
    body = np.array([1, 2, 3, 0, 0, 0, 1], dtype=np.float32)
    shape = np.array([10, 20, 30, 0, 0, 0, 1], dtype=np.float32)
    out = _compose_transforms(body, shape)
    np.testing.assert_allclose(out[:3], [11, 22, 33])
    np.testing.assert_allclose(out[3:], [0, 0, 0, 1])


# ---------- overlay integration with fakes -------------------------------


class _FakeWarpArray:
    """Mimics the readback path: .numpy() returns the stored ndarray."""

    def __init__(self, arr):
        self._arr = np.asarray(arr)

    def numpy(self):
        return self._arr


class FakePicking:
    def __init__(
        self,
        body_idx: int = -1,
        active: bool = False,
        pick_max_acceleration: float = 5.0,
        effective_mass: float = 1.0,
        picked_point: tuple[float, float, float] = (0.0, 0.0, 0.0),
        picked_local: tuple[float, float, float] | None = None,
        target: tuple[float, float, float] = (0.0, 0.0, 0.0),
        stiffness: float = 50.0,
    ) -> None:
        self.pick_body = _FakeWarpArray(np.array([body_idx], dtype=np.int64))
        self._active = active
        # Match Newton's PickingState numpy dtype access pattern (record fields).
        ps = np.zeros(
            1,
            dtype=[
                ("picked_point_local", np.float32, 3),
                ("picked_point_world", np.float32, 3),
                ("picking_target_world", np.float32, 3),
                ("pick_stiffness", np.float32),
                ("pick_damping", np.float32),
                ("pick_max_acceleration", np.float32),
            ],
        )
        ps[0]["picked_point_local"] = picked_local if picked_local is not None else picked_point
        ps[0]["picked_point_world"] = picked_point
        ps[0]["picking_target_world"] = target
        ps[0]["pick_stiffness"] = stiffness
        ps[0]["pick_damping"] = 5.0
        ps[0]["pick_max_acceleration"] = pick_max_acceleration
        self.pick_state = _FakeWarpArray(ps)
        self._pick_effective_mass = _FakeWarpArray(
            np.array([effective_mass], dtype=np.float32)
        )

    def is_picking(self) -> bool:
        return self._active


class FakeState:
    def __init__(self, body_q: np.ndarray, body_f: np.ndarray) -> None:
        self.body_q = _FakeWarpArray(body_q)
        self.body_f = _FakeWarpArray(body_f)


class FakeModel:
    def __init__(
        self,
        body_count: int,
        body_label: list[str],
        body_shapes: dict[int, list[int]],
        shape_type: np.ndarray,
        shape_scale: np.ndarray,
        shape_transform: np.ndarray,
        body_mass: np.ndarray | None = None,
        body_com: np.ndarray | None = None,
    ) -> None:
        self.body_count = body_count
        self.body_label = body_label
        self.body_shapes = body_shapes
        self.shape_count = len(shape_type)
        self.shape_type = _FakeWarpArray(shape_type)
        self.shape_scale = _FakeWarpArray(shape_scale)
        self.shape_transform = _FakeWarpArray(shape_transform)
        self.shape_source = [None] * len(shape_type)
        self.body_mass = _FakeWarpArray(
            body_mass if body_mass is not None else np.ones(body_count, dtype=np.float32)
        )
        self.body_com = _FakeWarpArray(
            body_com if body_com is not None else np.zeros((body_count, 3), dtype=np.float32)
        )


class FakeRenderer:
    def __init__(self) -> None:
        self.arrow_scale = 1.0


class FakeViewer:
    def __init__(self, model: FakeModel, with_arrows: bool = False) -> None:
        self.model = model
        self.device = "cpu"
        self.picking = FakePicking()
        self.picking_enabled = True
        self.log_lines_calls: list[tuple[str, Any, Any, Any, bool]] = []
        self.log_arrows_calls: list[tuple[str, Any, Any, Any, bool]] = []
        self.ui_callbacks: list[tuple[Any, str]] = []
        self.renderer = FakeRenderer() if with_arrows else None
        if with_arrows:
            self.log_arrows = self._log_arrows

    def register_ui_callback(self, cb, position="side") -> None:
        self.ui_callbacks.append((cb, position))

    def log_lines(self, name: str, starts, ends, colors, hidden: bool = False) -> None:
        self.log_lines_calls.append((name, starts, ends, colors, hidden))

    def _log_arrows(self, name: str, starts, ends, colors, hidden: bool = False) -> None:
        self.log_arrows_calls.append((name, starts, ends, colors, hidden))


def _make_one_box_model() -> FakeModel:
    # Single body 'link0' with one box shape (half-extents 0.1, 0.2, 0.3).
    shape_type = np.array([7], dtype=np.int32)  # BOX
    shape_scale = np.array([[0.1, 0.2, 0.3]], dtype=np.float32)
    shape_transform = np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32)
    return FakeModel(
        body_count=1,
        body_label=["link0"],
        body_shapes={0: [0]},
        shape_type=shape_type,
        shape_scale=shape_scale,
        shape_transform=shape_transform,
    )


class _FakeImgui:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def text(self, s: str) -> None:
        self.lines.append(s)


def test_overlay_no_pick_draws_nothing_and_hud_says_none():
    model = _make_one_box_model()
    viewer = FakeViewer(model)
    overlay = PickingOverlay(viewer)
    overlay.register_hud()

    assert len(viewer.ui_callbacks) == 1 and viewer.ui_callbacks[0][1] == "stats"

    state = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state)

    # No active pick: should call log_lines with None args (clear).
    assert viewer.log_lines_calls[-1] == ("picking_overlay/wireframe", None, None, None, False)

    imgui = _FakeImgui()
    overlay._draw_hud(imgui)
    assert any("none" in s.lower() for s in imgui.lines)


def test_overlay_active_pick_draws_box_wireframe_and_force():
    model = _make_one_box_model()
    viewer = FakeViewer(model)
    # Drive the spring with: picked_local = (0, 0.1, 0), target = (0.01, 0.1, 0).
    # mass=1, stiffness=50 → F = (10+1)*50*(0.01,0,0) = (5.5, 0, 0), |F|=5.5.
    # lever (pick_world - COM) = (0, 0.1, 0) → torque = lever × F = (0, 0, -0.55).
    viewer.picking = FakePicking(
        body_idx=0,
        active=True,
        picked_local=(0.0, 0.1, 0.0),
        picked_point=(0.0, 0.1, 0.0),
        target=(0.01, 0.1, 0.0),
        stiffness=50.0,
    )
    overlay = PickingOverlay(viewer)
    overlay.register_hud()

    # body at origin, identity orientation
    body_q = np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32)
    state = FakeState(body_q=body_q, body_f=np.zeros((1, 6), dtype=np.float32))

    overlay.update(state)

    # Last log_lines call should carry 12 box edges, magenta colored.
    name, starts, ends, colors, hidden = viewer.log_lines_calls[-1]
    assert name == "picking_overlay/wireframe"
    assert hidden is False
    assert starts is not None and ends is not None and colors is not None
    s_np = starts.numpy()
    assert s_np.shape == (12, 3)

    # HUD reflects link name and spring force/torque magnitudes.
    imgui = _FakeImgui()
    overlay._draw_hud(imgui)
    joined = "\n".join(imgui.lines)
    assert "link0" in joined
    assert "5.50 N" in joined
    assert "0.550 Nm" in joined


def test_overlay_handles_multiple_shapes_per_body():
    # Body 0 has a box and a sphere; expect 12 + 3*16 = 60 edges total.
    shape_type = np.array([7, 3], dtype=np.int32)  # BOX, SPHERE
    shape_scale = np.array(
        [[0.1, 0.2, 0.3], [0.5, 0.0, 0.0]],
        dtype=np.float32,
    )
    shape_transform = np.array(
        [[0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1]],
        dtype=np.float32,
    )
    model = FakeModel(
        body_count=1,
        body_label=["multi"],
        body_shapes={0: [0, 1]},
        shape_type=shape_type,
        shape_scale=shape_scale,
        shape_transform=shape_transform,
    )
    viewer = FakeViewer(model)
    viewer.picking = FakePicking(body_idx=0, active=True)
    overlay = PickingOverlay(viewer)

    state = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state)

    starts = viewer.log_lines_calls[-1][1].numpy()
    # 12 box edges + 3 great-circle rings × 16 segments
    assert starts.shape == (12 + 3 * 16, 3)


def test_overlay_clears_when_pick_released():
    model = _make_one_box_model()
    viewer = FakeViewer(model)
    overlay = PickingOverlay(viewer)

    # First: active
    viewer.picking = FakePicking(body_idx=0, active=True)
    state = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state)
    assert viewer.log_lines_calls[-1][1] is not None  # drew something

    # Then: released → cleared
    viewer.picking = FakePicking(body_idx=-1, active=False)
    overlay.update(state)
    assert viewer.log_lines_calls[-1] == ("picking_overlay/wireframe", None, None, None, False)


# ---------- force arrow + thickness modulation ---------------------------


def test_force_arrow_drawn_with_endpoints_and_color_when_active():
    model = _make_one_box_model()
    viewer = FakeViewer(model, with_arrows=True)
    # Large Δ → spring force saturates F_max → ratio=1.0 → red color.
    viewer.picking = FakePicking(
        body_idx=0,
        active=True,
        pick_max_acceleration=5.0,
        effective_mass=1.0,
        picked_point=(0.1, 0.2, 0.3),
        picked_local=(0.1, 0.2, 0.3),
        target=(0.5, 0.2, 0.3),
        stiffness=50.0,
    )
    overlay = PickingOverlay(viewer)

    state = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state)

    assert viewer.log_arrows_calls, "force arrow should be drawn"
    name, starts, ends, color, hidden = viewer.log_arrows_calls[-1]
    assert name == "picking_overlay/force_arrow"
    assert hidden is False
    np.testing.assert_allclose(starts.numpy().reshape(3), [0.1, 0.2, 0.3], atol=1e-5)
    np.testing.assert_allclose(ends.numpy().reshape(3), [0.5, 0.2, 0.3], atol=1e-5)
    # ratio=1.0 (saturated) → red (1,0,0)
    assert color == (1.0, 0.0, 0.0)


def test_arrow_thickness_modulated_and_restored():
    model = _make_one_box_model()
    viewer = FakeViewer(model, with_arrows=True)
    base = viewer.renderer.arrow_scale
    # Large Δ saturates F_max → ratio=1.0 → arrow_scale = base * (1 + gain) = base * 4.
    viewer.picking = FakePicking(
        body_idx=0,
        active=True,
        pick_max_acceleration=5.0,
        effective_mass=1.0,
        picked_point=(0.0, 0.0, 0.0),
        picked_local=(0.0, 0.0, 0.0),
        target=(0.1, 0.0, 0.0),
        stiffness=50.0,
    )
    overlay = PickingOverlay(viewer)

    state_full = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state_full)
    assert viewer.renderer.arrow_scale == pytest.approx(base * 4.0)

    # Release: arrow_scale restored, force_arrow cleared.
    viewer.picking = FakePicking(body_idx=-1, active=False)
    overlay.update(state_full)
    assert viewer.renderer.arrow_scale == pytest.approx(base)
    assert viewer.log_arrows_calls[-1] == (
        "picking_overlay/force_arrow",
        None,
        None,
        None,
        False,
    )


def test_force_arrow_skipped_for_degenerate_segment():
    model = _make_one_box_model()
    viewer = FakeViewer(model, with_arrows=True)
    viewer.picking = FakePicking(
        body_idx=0,
        active=True,
        picked_point=(0.5, 0.5, 0.5),
        target=(0.5, 0.5, 0.5),  # zero-length
    )
    overlay = PickingOverlay(viewer)

    state = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state)

    # log_arrows called, but with None (degenerate clear) — last entry is the clear.
    assert viewer.log_arrows_calls[-1][1] is None


def test_hud_includes_force_torque_components():
    model = _make_one_box_model()
    viewer = FakeViewer(model, with_arrows=True)
    # Large Δ saturates the clamp: F = (F_max, 0, 0) = (49.05, 0, 0).
    # picked_local=(0,0.1,0) → lever × F = (0, 0, -0.1 * 49.05) ≈ -4.905.
    viewer.picking = FakePicking(
        body_idx=0,
        active=True,
        pick_max_acceleration=5.0,
        effective_mass=1.0,
        picked_point=(0.0, 0.1, 0.0),
        picked_local=(0.0, 0.1, 0.0),
        target=(0.3, 0.1, 0.0),
        stiffness=50.0,
    )
    overlay = PickingOverlay(viewer)

    state = FakeState(
        body_q=np.array([[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32),
        body_f=np.zeros((1, 6), dtype=np.float32),
    )
    overlay.update(state)

    imgui = _FakeImgui()
    overlay._draw_hud(imgui)
    joined = "\n".join(imgui.lines)
    # Force magnitude (clamped to F_max = 49.05)
    assert "49.05 N" in joined
    # Per-axis force components: F = (+49.05, 0, 0)
    assert "+49.05" in joined
    # Per-axis torque components: τ ≈ (0, 0, -4.905) — rounded to 3 decimals
    assert "-4.905" in joined
    # F_max % indicator: ratio=1.0 → 100%
    assert "100% F_max" in joined
