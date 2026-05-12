"""GUI overlay for ViewerGL picking: HUD + magenta wireframe.

When a user right-clicks a body in the GL viewer, Newton's built-in picking
machinery (`viewer.picking`) applies a spring-damper force to that body. This
module adds two visual cues so the interaction is legible:

1. **HUD text** in the GL stats panel: picked link name and |F|, |tau|
   computed from the wrench Newton wrote into `state.body_f[pick_body]`.
2. **Magenta wireframe overlay** drawn on top of every shape attached to the
   picked body, rendered via the viewer's existing `log_lines` segment API.

Both run inside the viewer thread (no cross-thread state). Wireframe edges
are precomputed once per shape index and cached; per-frame work is just a
GPU upload of transformed line endpoints for the active body.

The module is GL-viewer-only: `attach()` is a no-op when the viewer lacks
the picking + log_lines surface (Rerun/USD/File/Null).
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

log = logging.getLogger(__name__)


MAGENTA = (1.0, 0.0, 1.0)
_OVERLAY_NAME = "picking_overlay/wireframe"

# Newton GeoType ints; mirrored locally so this module can be imported in
# host-side tests without newton installed.
_GEO_NONE = 0
_GEO_PLANE = 1
_GEO_SPHERE = 3
_GEO_CAPSULE = 4
_GEO_ELLIPSOID = 5
_GEO_CYLINDER = 6
_GEO_BOX = 7
_GEO_MESH = 8
_GEO_CONE = 9
_GEO_CONVEX_MESH = 10


def _box_edges(hx: float, hy: float, hz: float) -> tuple[np.ndarray, np.ndarray]:
    """12 edges of an axis-aligned box centered at origin, half-extents (hx,hy,hz)."""
    c = np.array(
        [
            [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
            [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
        ],
        dtype=np.float32,
    )
    e = np.array(
        [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ],
        dtype=np.int32,
    )
    return c[e[:, 0]], c[e[:, 1]]


def _ring(radius: float, axis: int, offset: float, segments: int = 24) -> tuple[np.ndarray, np.ndarray]:
    """Closed polygon ring of `segments` edges, in plane perpendicular to `axis`."""
    t = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False, dtype=np.float32)
    a, b = np.cos(t) * radius, np.sin(t) * radius
    pts = np.zeros((segments, 3), dtype=np.float32)
    other = [(1, 2), (0, 2), (0, 1)][axis]
    pts[:, other[0]] = a
    pts[:, other[1]] = b
    pts[:, axis] = offset
    nxt = np.roll(pts, -1, axis=0)
    return pts, nxt


def _sphere_edges(r: float, segments: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Three great circles (XY/XZ/YZ) — cheap recognizable wireframe."""
    s = []
    e = []
    for axis in (0, 1, 2):
        a, b = _ring(r, axis, 0.0, segments=segments)
        s.append(a)
        e.append(b)
    return np.concatenate(s, axis=0), np.concatenate(e, axis=0)


def _capsule_edges(r: float, half_height: float, segments: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Capsule along local Z: two end-cap rings + 4 longitudinal lines."""
    s, e = [], []
    for z in (-half_height, half_height):
        a, b = _ring(r, 2, z, segments=segments)
        s.append(a)
        e.append(b)
    # 4 longitudinal lines connecting the rings at +/-x, +/-y
    longs = np.array(
        [
            [(r, 0.0, -half_height), (r, 0.0, half_height)],
            [(-r, 0.0, -half_height), (-r, 0.0, half_height)],
            [(0.0, r, -half_height), (0.0, r, half_height)],
            [(0.0, -r, -half_height), (0.0, -r, half_height)],
        ],
        dtype=np.float32,
    )
    s.append(longs[:, 0, :])
    e.append(longs[:, 1, :])
    return np.concatenate(s, axis=0), np.concatenate(e, axis=0)


def _cylinder_edges(r: float, half_height: float, segments: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Same as capsule (matches what GL viewer renders for cylinders)."""
    return _capsule_edges(r, half_height, segments=segments)


def _mesh_edges_from_indices(vertices: np.ndarray, indices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Extract unique edges from a triangle mesh; returns (start_pts, end_pts)."""
    tri = indices.reshape(-1, 3)
    pairs = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]], axis=0)
    pairs = np.sort(pairs, axis=1)
    # Deduplicate
    pairs_view = np.ascontiguousarray(pairs).view(np.dtype((np.void, pairs.dtype.itemsize * 2)))
    _, idx = np.unique(pairs_view, return_index=True)
    uniq = pairs[idx]
    return vertices[uniq[:, 0]].astype(np.float32), vertices[uniq[:, 1]].astype(np.float32)


def _transform_points(pts: np.ndarray, xf: np.ndarray) -> np.ndarray:
    """Apply a (px,py,pz, qx,qy,qz,qw) transform to Nx3 points (numpy only)."""
    p = xf[:3].astype(np.float32)
    qx, qy, qz, qw = float(xf[3]), float(xf[4]), float(xf[5]), float(xf[6])
    # Rotate via quaternion-matrix (row-major). Newton uses (x,y,z,w).
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    R = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float32,
    )
    return pts @ R.T + p


def _compose_transforms(xf_body: np.ndarray, xf_shape: np.ndarray) -> np.ndarray:
    """body_q ∘ shape_transform → world-space (7,) transform (numpy)."""
    # Rotate xf_shape's translation by xf_body's quat, add xf_body's translation;
    # multiply quaternions.
    rotated = _transform_points(xf_shape[:3][None, :], xf_body)[0]
    bq = xf_body[3:7]
    sq = xf_shape[3:7]
    bx, by, bz, bw = float(bq[0]), float(bq[1]), float(bq[2]), float(bq[3])
    sx, sy, sz, sw = float(sq[0]), float(sq[1]), float(sq[2]), float(sq[3])
    qx = bw * sx + bx * sw + by * sz - bz * sy
    qy = bw * sy - bx * sz + by * sw + bz * sx
    qz = bw * sz + bx * sy - by * sx + bz * sw
    qw = bw * sw - bx * sx - by * sy - bz * sz
    return np.array([rotated[0], rotated[1], rotated[2], qx, qy, qz, qw], dtype=np.float32)


class PickingOverlay:
    """Per-frame HUD + magenta wireframe for the picked body.

    Parameters
    ----------
    viewer : ViewerGL
        Must expose `.picking`, `.log_lines`, `.register_ui_callback`, `.model`,
        and `.device`. Non-GL viewers should not instantiate this.
    body_names : list[str] | None
        Optional override for the per-body name list (length == model.body_count).
        Defaults to `model.body_label`.
    """

    def __init__(self, viewer: Any, body_names: list[str] | None = None) -> None:
        self.viewer = viewer
        model = viewer.model
        self.model = model

        # Precompute body -> list[shape_idx] map; body_shapes is dict[int, list].
        self._body_shapes: dict[int, list[int]] = {
            int(b): list(s) for b, s in (model.body_shapes or {}).items() if int(b) >= 0
        }

        # body_label is a list[str], length == body_count.
        labels = body_names if body_names is not None else getattr(model, "body_label", None)
        self._body_names: list[str] = list(labels) if labels else [f"body_{i}" for i in range(model.body_count)]

        # Cache shape-local wireframe edges keyed by shape index.
        self._shape_edges: dict[int, tuple[np.ndarray, np.ndarray]] = {}

        # Latest readbacks (kept on viewer thread; HUD callback reads these).
        self._latest_pick_body: int = -1
        self._latest_link_name: str = ""
        self._latest_force_mag: float = 0.0
        self._latest_torque_mag: float = 0.0

        # Cached numpy snapshots of model arrays (host-side, immutable per model).
        self._shape_type = model.shape_type.numpy() if model.shape_count > 0 else np.array([], dtype=np.int32)
        self._shape_scale = (
            model.shape_scale.numpy() if model.shape_count > 0 else np.zeros((0, 3), dtype=np.float32)
        )
        self._shape_transform = (
            model.shape_transform.numpy() if model.shape_count > 0 else np.zeros((0, 7), dtype=np.float32)
        )

    # ---- attachment ---------------------------------------------------------

    def register_hud(self) -> None:
        """Register the HUD imgui callback on the GL viewer's stats panel."""
        register = getattr(self.viewer, "register_ui_callback", None)
        if callable(register):
            register(self._draw_hud, position="stats")

    # ---- per-frame update ---------------------------------------------------

    def update(self, state: Any) -> None:
        """Read picking state, refresh HUD numbers, redraw wireframe overlay.

        Must be called on the viewer thread after `log_state(state)` so that
        the picking-line and overlay live in the same frame's record stream.
        """
        picking = getattr(self.viewer, "picking", None)
        if picking is None or not getattr(self.viewer, "picking_enabled", True):
            self._clear()
            return

        try:
            pick_body = int(picking.pick_body.numpy()[0])
        except Exception:  # noqa: BLE001
            pick_body = -1

        if pick_body < 0 or not picking.is_picking():
            self._clear()
            return

        self._latest_pick_body = pick_body
        self._latest_link_name = (
            self._body_names[pick_body] if 0 <= pick_body < len(self._body_names) else f"body_{pick_body}"
        )

        # Force/torque magnitudes from state.body_f[pick_body]
        try:
            body_f = state.body_f.numpy()[pick_body]
            # spatial_vector layout in Newton is (top=linear, bottom=angular) when
            # accessed via .numpy() — float[6]. Linear is what apply_picking_force
            # writes via wp.spatial_top, so [:3] = force, [3:] = torque.
            f = body_f[:3]
            t = body_f[3:6]
            self._latest_force_mag = float(np.linalg.norm(f))
            self._latest_torque_mag = float(np.linalg.norm(t))
        except Exception:  # noqa: BLE001
            self._latest_force_mag = 0.0
            self._latest_torque_mag = 0.0

        # Magenta wireframe overlay
        try:
            body_q = state.body_q.numpy()[pick_body]
        except Exception:  # noqa: BLE001
            self._draw_lines(None, None)
            return

        starts_list: list[np.ndarray] = []
        ends_list: list[np.ndarray] = []
        for shape_idx in self._body_shapes.get(pick_body, []):
            edges = self._get_shape_edges(shape_idx)
            if edges is None:
                continue
            s_local, e_local = edges
            xf_world = _compose_transforms(body_q, self._shape_transform[shape_idx])
            starts_list.append(_transform_points(s_local, xf_world))
            ends_list.append(_transform_points(e_local, xf_world))

        if not starts_list:
            self._draw_lines(None, None)
            return

        starts = np.concatenate(starts_list, axis=0)
        ends = np.concatenate(ends_list, axis=0)
        self._draw_lines(starts, ends)

    def _clear(self) -> None:
        self._latest_pick_body = -1
        self._latest_link_name = ""
        self._latest_force_mag = 0.0
        self._latest_torque_mag = 0.0
        self._draw_lines(None, None)

    # ---- wireframe edge cache ----------------------------------------------

    def _get_shape_edges(self, shape_idx: int) -> tuple[np.ndarray, np.ndarray] | None:
        """Return (start_pts, end_pts) in shape-local coords. Cached per shape."""
        cached = self._shape_edges.get(shape_idx)
        if cached is not None:
            return cached

        if shape_idx < 0 or shape_idx >= len(self._shape_type):
            return None
        geo_type = int(self._shape_type[shape_idx])
        scale = self._shape_scale[shape_idx]

        edges: tuple[np.ndarray, np.ndarray] | None
        if geo_type == _GEO_BOX:
            edges = _box_edges(float(scale[0]), float(scale[1]), float(scale[2]))
        elif geo_type == _GEO_SPHERE:
            edges = _sphere_edges(float(scale[0]))
        elif geo_type == _GEO_CAPSULE:
            edges = _capsule_edges(float(scale[0]), float(scale[1]))
        elif geo_type == _GEO_CYLINDER:
            edges = _cylinder_edges(float(scale[0]), float(scale[1]))
        elif geo_type == _GEO_ELLIPSOID:
            # Use mean radius — visual approximation.
            r = float((scale[0] + scale[1] + scale[2]) / 3.0)
            edges = _sphere_edges(r)
        elif geo_type in (_GEO_MESH, _GEO_CONVEX_MESH):
            edges = self._mesh_edges(shape_idx, scale)
        else:
            edges = None

        if edges is not None:
            self._shape_edges[shape_idx] = edges
        return edges

    def _mesh_edges(self, shape_idx: int, scale: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
        source = getattr(self.model, "shape_source", None)
        if source is None or shape_idx >= len(source):
            return None
        mesh = source[shape_idx]
        verts = getattr(mesh, "vertices", None)
        idxs = getattr(mesh, "indices", None)
        if verts is None or idxs is None or len(verts) == 0 or len(idxs) < 3:
            return None
        verts_np = np.asarray(verts, dtype=np.float32) * np.asarray(scale, dtype=np.float32)
        idxs_np = np.asarray(idxs, dtype=np.int32).reshape(-1)
        if idxs_np.size % 3 != 0:
            return None
        return _mesh_edges_from_indices(verts_np, idxs_np)

    # ---- log_lines wrapper --------------------------------------------------

    def _draw_lines(self, starts: np.ndarray | None, ends: np.ndarray | None) -> None:
        """Push a magenta line batch (or clear) to the viewer."""
        log_lines = getattr(self.viewer, "log_lines", None)
        if not callable(log_lines):
            return
        try:
            if starts is None or ends is None or len(starts) == 0:
                # Clear: pass length-0 arrays so the viewer drops the entry.
                # Newton's log_lines on None,None,None hides the layer.
                log_lines(_OVERLAY_NAME, None, None, None)
                return
            import warp as wp  # noqa: PLC0415

            device = getattr(self.viewer, "device", None)
            n = len(starts)
            starts_wp = wp.array(starts.reshape(-1, 3).astype(np.float32), dtype=wp.vec3, device=device)
            ends_wp = wp.array(ends.reshape(-1, 3).astype(np.float32), dtype=wp.vec3, device=device)
            colors_np = np.tile(np.asarray(MAGENTA, dtype=np.float32), (n, 1))
            colors_wp = wp.array(colors_np, dtype=wp.vec3, device=device)
            log_lines(_OVERLAY_NAME, starts_wp, ends_wp, colors_wp, hidden=False)
        except Exception:  # noqa: BLE001
            log.exception("picking overlay log_lines failed")

    # ---- HUD ---------------------------------------------------------------

    def _draw_hud(self, imgui: Any) -> None:
        try:
            if self._latest_pick_body < 0:
                imgui.text("pick: none (right-drag a link)")
                return
            imgui.text(f"pick: {self._latest_link_name}")
            imgui.text(f" |F| = {self._latest_force_mag:.2f} N")
            imgui.text(f" |t| = {self._latest_torque_mag:.3f} Nm")
        except Exception:  # noqa: BLE001
            log.exception("picking HUD draw failed")
