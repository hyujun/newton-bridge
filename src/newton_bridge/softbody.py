"""Softbody pack loading + spec parsing (PR 1).

A pack may declare zero or more soft bodies that get added to the model
alongside the rigid articulation. Each soft body is a tetrahedral mesh
loaded from a `.npz` file produced offline.

Pack layout (per the design discussion):

    robots/<name>/
      models/             # URDF/MJCF + meshes (existing)
      robot.yaml          # adds optional `softbodies:` list (see below)
      softbody/
        <asset>.npz       # vertices + tet_indices

YAML schema (per articulation; carried through `_promote_robot_yaml`
into the canonical scene under each articulation entry):

    softbodies:
      - name: left_fingertip_pad
        asset_rel: softbody/a.npz   # path relative to the pack dir
        pos: [0.0, 0.0, 0.0]        # attach.body local frame offset
        rot: [0.0, 0.0, 0.0, 1.0]   # attach.body local frame (xyzw, default I)
        scale: 1.0                  # default 1.0
        material:
          density: 100.0
          k_mu: 1.0e6
          k_lambda: 1.0e6
          k_damp: 1.0e-6
        particle_radius: 0.003      # optional
        attach:
          body: left_fingertip      # link name (must exist in URDF/MJCF)
          vertex_indices: [0, 1, 2] # which TetMesh vertices are pinned
        contact:                    # optional, applied at finalize
          ke: 2.0e6
          kd: 1.0e-7
          mu: 0.5

`pos`/`rot`/`scale` describe the pad's pose in `attach.body`'s LOCAL frame
— independent of base_position or home pose. NewtonWorld composes them
with the link's builder-initial world transform to derive the actual
spawn pose (see `_build_softbodies` in `world.py`).

This module is intentionally Newton-free so it can run in host pytest
without the Newton container. The only required keys in the .npz are
`vertices` (N, 3) float and `tet_indices` (T, 4) int. They are coerced
to float32 / int32 at load time. The loader also runs three geometry
sanity checks (inverted-tet auto-fix, disconnected-mesh rejection,
mm-scale warning); see `_fix_inverted_tets`, `_assert_single_component`,
and `_warn_if_suspicious_scale` below.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

log = logging.getLogger(__name__)


# -- npz schema --------------------------------------------------------------

REQUIRED_NPZ_KEYS = ("vertices", "tet_indices")

# Bbox larger than this on any axis triggers the "did you mean mm?" warning.
# 1m is a generous upper bound for any realistic softbody pad/finger asset.
_SCALE_WARN_THRESHOLD_M = 1.0


@dataclass(frozen=True)
class TetMeshArrays:
    """Plain-numpy view of a softbody asset, ready to feed into newton.TetMesh.

    Sizes:
        vertices: (N, 3) float32
        tet_indices: (T, 4) int32
    """

    vertices: np.ndarray
    tet_indices: np.ndarray

    @property
    def vertex_count(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def tet_count(self) -> int:
        return int(self.tet_indices.shape[0])


def load_softbody_npz(path: Path) -> TetMeshArrays:
    """Load a `.npz` softbody asset.

    Required keys: `vertices` (N,3) and `tet_indices` (T,4). Both are coerced
    to float32 / int32. Unknown extra keys are ignored — future versions may
    use them, but PR 1 keeps the schema minimal.
    """
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"softbody asset not found: {p}")
    with np.load(p) as npz:
        keys = set(npz.files)
        missing = [k for k in REQUIRED_NPZ_KEYS if k not in keys]
        if missing:
            raise ValueError(
                f"{p}: missing keys {missing} in .npz; required: {list(REQUIRED_NPZ_KEYS)}; "
                f"present: {sorted(keys)}"
            )
        vertices = np.asarray(npz["vertices"])
        tet_indices = np.asarray(npz["tet_indices"])

    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError(
            f"{p}: vertices must have shape (N, 3); got {vertices.shape}"
        )
    if tet_indices.ndim != 2 or tet_indices.shape[1] != 4:
        raise ValueError(
            f"{p}: tet_indices must have shape (T, 4); got {tet_indices.shape}"
        )
    if vertices.shape[0] == 0:
        raise ValueError(f"{p}: vertices is empty")
    if tet_indices.shape[0] == 0:
        raise ValueError(f"{p}: tet_indices is empty")

    vertices = np.ascontiguousarray(vertices, dtype=np.float32)
    tet_indices = np.ascontiguousarray(tet_indices, dtype=np.int32)

    n = vertices.shape[0]
    if int(tet_indices.min()) < 0 or int(tet_indices.max()) >= n:
        raise ValueError(
            f"{p}: tet_indices reference vertices outside [0,{n}); "
            f"min={int(tet_indices.min())} max={int(tet_indices.max())}"
        )

    # --- sanity checks. Mesh has the right schema; now verify the geometry
    # won't blow up the solver. Inverted-tet auto-fix is non-destructive
    # (just swaps two vertex indices), so we apply it inline. Disconnected
    # meshes are a hard error because picking the largest component would
    # silently discard vertices and re-index everything — better to fail
    # loud and force the user to re-mesh.
    tet_indices = _fix_inverted_tets(vertices, tet_indices, source=str(p))
    _assert_single_component(vertices, tet_indices, source=str(p))
    _warn_if_suspicious_scale(vertices, source=str(p))

    return TetMeshArrays(vertices=vertices, tet_indices=tet_indices)


# -- sanity helpers ---------------------------------------------------------


def _tet_volumes(vertices: np.ndarray, tet_indices: np.ndarray) -> np.ndarray:
    """Signed tet volume V = det([e1, e2, e3]) / 6, vectorized over tets.

    Negative or zero values mean inverted or degenerate. Result is (T,) float64.
    """
    v0 = vertices[tet_indices[:, 0]].astype(np.float64)
    v1 = vertices[tet_indices[:, 1]].astype(np.float64)
    v2 = vertices[tet_indices[:, 2]].astype(np.float64)
    v3 = vertices[tet_indices[:, 3]].astype(np.float64)
    mats = np.stack([v1 - v0, v2 - v0, v3 - v0], axis=2)  # (T, 3, 3)
    return np.linalg.det(mats) / 6.0


def _fix_inverted_tets(
    vertices: np.ndarray, tet_indices: np.ndarray, *, source: str
) -> np.ndarray:
    """Detect tets with non-positive signed volume; swap nodes 1↔2 to flip
    orientation. Returns the (possibly modified) tet_indices array.

    Truly degenerate (`|V|<eps`) tets are reported but not removable here —
    they survive in the returned array. The solver will treat them as
    near-zero stiffness contributors; if you want to fail hard, the caller
    can post-check with `_tet_volumes(...)`.
    """
    vols = _tet_volumes(vertices, tet_indices)
    eps = float(np.finfo(np.float64).eps) * max(1.0, float(np.abs(vols).max()))
    inverted = vols < -eps
    degenerate = np.abs(vols) <= eps
    n_inv = int(inverted.sum())
    n_deg = int(degenerate.sum())

    if n_inv:
        out = tet_indices.copy()
        col1 = out[inverted, 1].copy()
        out[inverted, 1] = out[inverted, 2]
        out[inverted, 2] = col1
        log.warning(
            "%s: fixed %d inverted tet(s) by swapping nodes 1↔2",
            source, n_inv,
        )
        tet_indices = np.ascontiguousarray(out, dtype=np.int32)

    if n_deg:
        log.warning(
            "%s: %d degenerate tet(s) (|V|≈0) detected; "
            "softbody simulation may be unstable. Consider remeshing.",
            source, n_deg,
        )

    return tet_indices


def _assert_single_component(
    vertices: np.ndarray, tet_indices: np.ndarray, *, source: str
) -> None:
    """Verify the tet mesh forms a single connected component via BFS over
    the vertex adjacency induced by tet edges.

    Pure numpy (no scipy) to keep this module dependency-light and host-test
    friendly. Disconnected meshes are rejected outright — silently keeping
    the largest component would discard particles the YAML may reference.
    """
    n = int(vertices.shape[0])
    if n == 0:
        return

    # Build adjacency as a list-of-sets via vectorized edge extraction.
    # Each tet contributes 6 undirected edges (4 choose 2).
    edges_a = np.concatenate(
        [tet_indices[:, i] for i in (0, 0, 0, 1, 1, 2)]
    )
    edges_b = np.concatenate(
        [tet_indices[:, j] for j in (1, 2, 3, 2, 3, 3)]
    )
    adj: list[list[int]] = [[] for _ in range(n)]
    for a, b in zip(edges_a.tolist(), edges_b.tolist()):
        adj[a].append(b)
        adj[b].append(a)

    visited = np.zeros(n, dtype=bool)
    # BFS from vertex 0. Any vertex unreached at the end is in a different
    # component. (Tet-referenced vertices only — isolated bare vertices in
    # the npz vertex list are caught by the same test since they'd never be
    # visited.)
    stack = [0]
    visited[0] = True
    while stack:
        u = stack.pop()
        for v in adj[u]:
            if not visited[v]:
                visited[v] = True
                stack.append(v)

    if not visited.all():
        n_unreached = int((~visited).sum())
        raise ValueError(
            f"{source}: tet mesh is disconnected — {n_unreached} of {n} "
            f"vertices unreachable from vertex 0. Newton solvers assume a "
            f"single connected component; re-mesh or split the asset."
        )


def _warn_if_suspicious_scale(vertices: np.ndarray, *, source: str) -> None:
    """Emit a WARNING if the mesh bbox exceeds 1m on any axis.

    Newton is SI (meters). A bbox in the tens or hundreds usually means the
    asset is in millimeters — the user should rescale at export time. We
    only warn (not error) because someone could legitimately ship a meter-
    scale soft object.
    """
    bbox = vertices.max(axis=0) - vertices.min(axis=0)
    if float(bbox.max()) > _SCALE_WARN_THRESHOLD_M:
        log.warning(
            "%s: bbox %s exceeds %.1fm — Newton uses SI units (m). "
            "If this asset is in mm, rescale with `vertices *= 0.001` "
            "before saving the npz.",
            source, tuple(float(x) for x in bbox), _SCALE_WARN_THRESHOLD_M,
        )


# -- yaml parsing ------------------------------------------------------------


@dataclass(frozen=True)
class SoftbodyMaterial:
    density: float = 100.0
    k_mu: float = 1.0e6
    k_lambda: float = 1.0e6
    k_damp: float = 1.0e-6


@dataclass(frozen=True)
class SoftbodyContact:
    ke: float | None = None
    kd: float | None = None
    mu: float | None = None


@dataclass(frozen=True)
class SoftbodyAttach:
    body: str
    vertex_indices: tuple[int, ...]


@dataclass(frozen=True)
class SoftbodySpec:
    """One parsed `softbodies:` entry, ready for NewtonWorld to build."""

    name: str
    asset_path: Path                # absolute, resolved against pack dir
    pos: tuple[float, float, float]
    rot: tuple[float, float, float, float]   # xyzw
    scale: float
    material: SoftbodyMaterial
    attach: SoftbodyAttach
    particle_radius: float | None
    contact: SoftbodyContact = field(default_factory=SoftbodyContact)


def _as_float(x: Any, key: str) -> float:
    try:
        return float(x)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"softbody.{key}: expected float, got {x!r}") from exc


def _as_vec3(x: Any, key: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if x is None:
        return default
    seq = list(x)
    if len(seq) != 3:
        raise ValueError(f"softbody.{key}: expected length-3 sequence, got {x!r}")
    return (_as_float(seq[0], key), _as_float(seq[1], key), _as_float(seq[2], key))


def _as_quat(x: Any, key: str) -> tuple[float, float, float, float]:
    if x is None:
        return (0.0, 0.0, 0.0, 1.0)
    seq = list(x)
    if len(seq) != 4:
        raise ValueError(f"softbody.{key}: expected length-4 quaternion (xyzw), got {x!r}")
    return tuple(_as_float(v, key) for v in seq)  # type: ignore[return-value]


def _parse_one(entry: dict, pack_dir: Path, idx: int) -> SoftbodySpec:
    if not isinstance(entry, dict):
        raise ValueError(f"softbodies[{idx}]: must be a mapping, got {type(entry).__name__}")

    name = entry.get("name")
    if not isinstance(name, str) or not name:
        raise ValueError(f"softbodies[{idx}].name: required non-empty string")

    asset_rel = entry.get("asset_rel")
    if not isinstance(asset_rel, str) or not asset_rel:
        raise ValueError(f"softbodies[{name!r}].asset_rel: required non-empty string")
    asset_path = (pack_dir / asset_rel).resolve()

    pos = _as_vec3(entry.get("pos"), f"{name}.pos", (0.0, 0.0, 0.0))
    rot = _as_quat(entry.get("rot"), f"{name}.rot")
    scale = _as_float(entry.get("scale", 1.0), f"{name}.scale")
    if scale <= 0.0:
        raise ValueError(f"softbodies[{name!r}].scale: must be > 0; got {scale}")

    mat_raw = entry.get("material") or {}
    if not isinstance(mat_raw, dict):
        raise ValueError(f"softbodies[{name!r}].material: must be a mapping")
    material = SoftbodyMaterial(
        density=_as_float(mat_raw.get("density", SoftbodyMaterial.density), f"{name}.material.density"),
        k_mu=_as_float(mat_raw.get("k_mu", SoftbodyMaterial.k_mu), f"{name}.material.k_mu"),
        k_lambda=_as_float(mat_raw.get("k_lambda", SoftbodyMaterial.k_lambda), f"{name}.material.k_lambda"),
        k_damp=_as_float(mat_raw.get("k_damp", SoftbodyMaterial.k_damp), f"{name}.material.k_damp"),
    )

    attach_raw = entry.get("attach")
    if not isinstance(attach_raw, dict):
        raise ValueError(f"softbodies[{name!r}].attach: required mapping")
    body = attach_raw.get("body")
    if not isinstance(body, str) or not body:
        raise ValueError(f"softbodies[{name!r}].attach.body: required non-empty string")
    vidx_raw = attach_raw.get("vertex_indices")
    if not isinstance(vidx_raw, (list, tuple)) or not vidx_raw:
        raise ValueError(
            f"softbodies[{name!r}].attach.vertex_indices: required non-empty list of ints"
        )
    try:
        vertex_indices = tuple(int(i) for i in vidx_raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"softbodies[{name!r}].attach.vertex_indices: must be ints, got {vidx_raw!r}"
        ) from exc
    if any(i < 0 for i in vertex_indices):
        raise ValueError(
            f"softbodies[{name!r}].attach.vertex_indices: indices must be >= 0"
        )
    if len(set(vertex_indices)) != len(vertex_indices):
        raise ValueError(
            f"softbodies[{name!r}].attach.vertex_indices: duplicates not allowed"
        )

    radius_raw = entry.get("particle_radius")
    particle_radius = None if radius_raw is None else _as_float(radius_raw, f"{name}.particle_radius")
    if particle_radius is not None and particle_radius <= 0.0:
        raise ValueError(
            f"softbodies[{name!r}].particle_radius: must be > 0 if set; got {particle_radius}"
        )

    contact_raw = entry.get("contact") or {}
    if not isinstance(contact_raw, dict):
        raise ValueError(f"softbodies[{name!r}].contact: must be a mapping")
    contact = SoftbodyContact(
        ke=_as_float(contact_raw["ke"], f"{name}.contact.ke") if "ke" in contact_raw else None,
        kd=_as_float(contact_raw["kd"], f"{name}.contact.kd") if "kd" in contact_raw else None,
        mu=_as_float(contact_raw["mu"], f"{name}.contact.mu") if "mu" in contact_raw else None,
    )

    return SoftbodySpec(
        name=name,
        asset_path=asset_path,
        pos=pos,
        rot=rot,
        scale=scale,
        material=material,
        attach=SoftbodyAttach(body=body, vertex_indices=vertex_indices),
        particle_radius=particle_radius,
        contact=contact,
    )


def parse_softbodies(articulation: dict, pack_dir: Path) -> list[SoftbodySpec]:
    """Parse the `softbodies:` block of one articulation.

    Returns `[]` when the key is absent or empty — softbody is fully optional
    and existing packs without the key remain unaffected.

    The asset path is resolved against `pack_dir` but **not** required to
    exist here; PR 2 (NewtonWorld build) is what loads the .npz. This keeps
    yaml validation and asset I/O separable.
    """
    raw = articulation.get("softbodies") if isinstance(articulation, dict) else None
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError(
            f"{pack_dir}: softbodies must be a list; got {type(raw).__name__}"
        )

    seen_names: set[str] = set()
    specs: list[SoftbodySpec] = []
    for i, entry in enumerate(raw):
        spec = _parse_one(entry, pack_dir, i)
        if spec.name in seen_names:
            raise ValueError(
                f"{pack_dir}: softbodies[{i}].name={spec.name!r} duplicates an earlier entry"
            )
        seen_names.add(spec.name)
        specs.append(spec)
    return specs
