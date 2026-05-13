"""PR 1: softbody .npz loader + yaml spec parser.

Newton-free unit tests. They exercise:
- `load_softbody_npz`: schema validation on the (vertices, tet_indices) keys
- `parse_softbodies`: the per-articulation `softbodies:` yaml block
- `load_pack` carry-through: legacy `robot.yaml` with `softbodies:` lands in
  the canonical scene under the primary articulation and as a top-level alias

Each soft body asset is built in-memory (one tet, four vertices) so we don't
need a real fingertip mesh on disk.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
import pytest
import yaml

from newton_bridge.robot_pack import load_pack
from newton_bridge.softbody import (
    SoftbodySpec,
    load_softbody_npz,
    parse_softbodies,
)


def _write_minimal_npz(path: Path) -> None:
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,  # loader must coerce to float32
    )
    tet_indices = np.array([[0, 1, 2, 3]], dtype=np.int64)  # loader coerces to int32
    np.savez(path, vertices=vertices, tet_indices=tet_indices)


# -- npz loader --------------------------------------------------------------


def test_load_npz_happy_path(tmp_path: Path) -> None:
    p = tmp_path / "pad.npz"
    _write_minimal_npz(p)

    arr = load_softbody_npz(p)
    assert arr.vertices.dtype == np.float32
    assert arr.tet_indices.dtype == np.int32
    assert arr.vertices.shape == (4, 3)
    assert arr.tet_indices.shape == (1, 4)
    assert arr.vertex_count == 4
    assert arr.tet_count == 1


def test_load_npz_missing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_softbody_npz(tmp_path / "nope.npz")


def test_load_npz_missing_key(tmp_path: Path) -> None:
    p = tmp_path / "bad.npz"
    np.savez(p, vertices=np.zeros((4, 3), dtype=np.float32))  # tet_indices missing
    with pytest.raises(ValueError, match="missing keys.*tet_indices"):
        load_softbody_npz(p)


def test_load_npz_wrong_vertex_shape(tmp_path: Path) -> None:
    p = tmp_path / "bad.npz"
    np.savez(
        p,
        vertices=np.zeros((4, 4), dtype=np.float32),
        tet_indices=np.zeros((1, 4), dtype=np.int32),
    )
    with pytest.raises(ValueError, match=r"vertices must have shape \(N, 3\)"):
        load_softbody_npz(p)


def test_load_npz_wrong_tet_shape(tmp_path: Path) -> None:
    p = tmp_path / "bad.npz"
    np.savez(
        p,
        vertices=np.zeros((4, 3), dtype=np.float32),
        tet_indices=np.zeros((1, 3), dtype=np.int32),  # triangle, not tet
    )
    with pytest.raises(ValueError, match=r"tet_indices must have shape \(T, 4\)"):
        load_softbody_npz(p)


def test_load_npz_index_out_of_range(tmp_path: Path) -> None:
    p = tmp_path / "bad.npz"
    np.savez(
        p,
        vertices=np.zeros((4, 3), dtype=np.float32),
        tet_indices=np.array([[0, 1, 2, 7]], dtype=np.int32),
    )
    with pytest.raises(ValueError, match="reference vertices outside"):
        load_softbody_npz(p)


def test_load_npz_empty_arrays(tmp_path: Path) -> None:
    p = tmp_path / "bad.npz"
    np.savez(
        p,
        vertices=np.zeros((0, 3), dtype=np.float32),
        tet_indices=np.zeros((0, 4), dtype=np.int32),
    )
    with pytest.raises(ValueError, match="vertices is empty"):
        load_softbody_npz(p)


# -- sanity checks ----------------------------------------------------------


def _capture_softbody_logs() -> tuple[logging.Handler, list[logging.LogRecord]]:
    """Attach a list-handler directly to the softbody logger.

    pytest's `caplog` fixture relies on propagation to root, which some
    container logging configs disable. A directly-attached handler captures
    records regardless. Caller is responsible for `removeHandler` cleanup.
    """
    records: list[logging.LogRecord] = []

    class _ListHandler(logging.Handler):
        def emit(self, record: logging.LogRecord) -> None:
            records.append(record)

    handler = _ListHandler(level=logging.WARNING)
    logger = logging.getLogger("newton_bridge.softbody")
    logger.addHandler(handler)
    logger.setLevel(logging.WARNING)
    return handler, records


def test_inverted_tet_is_auto_fixed(tmp_path: Path) -> None:
    """A tet with negative signed volume should be silently re-oriented and
    the loader should log a WARNING with the count.
    """
    p = tmp_path / "inv.npz"
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    tet_indices = np.array([[0, 2, 1, 3]], dtype=np.int32)  # det < 0
    np.savez(p, vertices=vertices, tet_indices=tet_indices)

    handler, records = _capture_softbody_logs()
    try:
        arr = load_softbody_npz(p)
    finally:
        logging.getLogger("newton_bridge.softbody").removeHandler(handler)
    # After fix: nodes 1↔2 swapped back → original positive orientation.
    np.testing.assert_array_equal(
        arr.tet_indices, np.array([[0, 1, 2, 3]], dtype=np.int32)
    )
    msgs = [r.getMessage() for r in records]
    assert any("inverted tet" in m for m in msgs), f"got: {msgs}"


def test_disconnected_mesh_is_rejected(tmp_path: Path) -> None:
    """Two unconnected tets (sharing no vertices) → ValueError."""
    p = tmp_path / "split.npz"
    # 8 vertices, 2 tets, no shared vertices → 2 components.
    vertices = np.array(
        [
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
            [5.0, 0.0, 0.0], [6.0, 0.0, 0.0], [5.0, 1.0, 0.0], [5.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    tet_indices = np.array([[0, 1, 2, 3], [4, 5, 6, 7]], dtype=np.int32)
    np.savez(p, vertices=vertices, tet_indices=tet_indices)
    with pytest.raises(ValueError, match="disconnected"):
        load_softbody_npz(p)


def test_suspicious_scale_warns(tmp_path: Path) -> None:
    """Mesh in mm (bbox = 10m equivalent) should WARN, not fail."""
    p = tmp_path / "big.npz"
    vertices = np.array(
        [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]],
        dtype=np.float32,
    )
    tet_indices = np.array([[0, 1, 2, 3]], dtype=np.int32)
    np.savez(p, vertices=vertices, tet_indices=tet_indices)

    handler, records = _capture_softbody_logs()
    try:
        arr = load_softbody_npz(p)
    finally:
        logging.getLogger("newton_bridge.softbody").removeHandler(handler)
    assert arr.vertex_count == 4  # load succeeds
    msgs = [r.getMessage() for r in records]
    assert any("bbox" in m and "SI" in m for m in msgs), f"got: {msgs}"


def test_normal_mesh_no_warnings(tmp_path: Path) -> None:
    """The canonical positive-orientation tet must load cleanly: no
    inverted-fix log, no scale warning.
    """
    p = tmp_path / "ok.npz"
    _write_minimal_npz(p)
    handler, records = _capture_softbody_logs()
    try:
        load_softbody_npz(p)
    finally:
        logging.getLogger("newton_bridge.softbody").removeHandler(handler)
    msgs = [r.getMessage() for r in records]
    assert not any("inverted" in m for m in msgs), msgs
    assert not any("bbox" in m for m in msgs), msgs


# -- spec parser -------------------------------------------------------------


def _minimal_entry(name: str = "pad", asset_rel: str = "softbody/a.npz") -> dict:
    return {
        "name": name,
        "asset_rel": asset_rel,
        "attach": {"body": "fingertip", "vertex_indices": [0, 1, 2]},
    }


def test_parse_softbodies_empty_or_missing(tmp_path: Path) -> None:
    assert parse_softbodies({}, tmp_path) == []
    assert parse_softbodies({"softbodies": []}, tmp_path) == []
    assert parse_softbodies({"softbodies": None}, tmp_path) == []


def test_parse_softbodies_defaults(tmp_path: Path) -> None:
    art = {"softbodies": [_minimal_entry()]}
    [spec] = parse_softbodies(art, tmp_path)
    assert isinstance(spec, SoftbodySpec)
    assert spec.name == "pad"
    assert spec.asset_path == (tmp_path / "softbody/a.npz").resolve()
    assert spec.pos == (0.0, 0.0, 0.0)
    assert spec.rot == (0.0, 0.0, 0.0, 1.0)
    assert spec.scale == 1.0
    assert spec.material.density == 100.0
    assert spec.material.k_mu == 1.0e6
    assert spec.attach.body == "fingertip"
    assert spec.attach.vertex_indices == (0, 1, 2)
    assert spec.particle_radius is None
    assert spec.contact.ke is None and spec.contact.kd is None and spec.contact.mu is None


def test_parse_softbodies_full_overrides(tmp_path: Path) -> None:
    entry = {
        "name": "left_pad",
        "asset_rel": "softbody/left.npz",
        "pos": [0.1, -0.2, 0.3],
        "rot": [0.0, 0.0, 0.7071, 0.7071],
        "scale": 1.5,
        "material": {"density": 50.0, "k_mu": 2e5, "k_lambda": 3e5, "k_damp": 1e-5},
        "particle_radius": 0.004,
        "attach": {"body": "left_fingertip", "vertex_indices": [10, 20, 30, 40]},
        "contact": {"ke": 1.0e6, "kd": 1.0e-6, "mu": 0.4},
    }
    [spec] = parse_softbodies({"softbodies": [entry]}, tmp_path)
    assert spec.pos == (0.1, -0.2, 0.3)
    assert spec.rot == (0.0, 0.0, 0.7071, 0.7071)
    assert spec.scale == 1.5
    assert spec.material.density == 50.0
    assert spec.material.k_lambda == 3e5
    assert spec.particle_radius == 0.004
    assert spec.attach.vertex_indices == (10, 20, 30, 40)
    assert spec.contact.ke == 1.0e6
    assert spec.contact.mu == 0.4


def test_parse_softbodies_duplicate_name_rejected(tmp_path: Path) -> None:
    art = {"softbodies": [_minimal_entry("pad"), _minimal_entry("pad", "softbody/b.npz")]}
    with pytest.raises(ValueError, match="duplicates"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_missing_attach(tmp_path: Path) -> None:
    art = {"softbodies": [{"name": "pad", "asset_rel": "x.npz"}]}
    with pytest.raises(ValueError, match="attach: required mapping"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_missing_vertex_indices(tmp_path: Path) -> None:
    art = {
        "softbodies": [
            {"name": "pad", "asset_rel": "x.npz", "attach": {"body": "ft", "vertex_indices": []}}
        ]
    }
    with pytest.raises(ValueError, match="vertex_indices"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_negative_index(tmp_path: Path) -> None:
    art = {
        "softbodies": [
            {
                "name": "pad",
                "asset_rel": "x.npz",
                "attach": {"body": "ft", "vertex_indices": [0, -1]},
            }
        ]
    }
    with pytest.raises(ValueError, match=">= 0"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_duplicate_indices(tmp_path: Path) -> None:
    art = {
        "softbodies": [
            {
                "name": "pad",
                "asset_rel": "x.npz",
                "attach": {"body": "ft", "vertex_indices": [0, 1, 1]},
            }
        ]
    }
    with pytest.raises(ValueError, match="duplicates not allowed"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_bad_quat(tmp_path: Path) -> None:
    art = {
        "softbodies": [
            {**_minimal_entry(), "rot": [0.0, 0.0, 0.0]},  # missing w
        ]
    }
    with pytest.raises(ValueError, match="length-4 quaternion"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_bad_scale(tmp_path: Path) -> None:
    art = {"softbodies": [{**_minimal_entry(), "scale": 0.0}]}
    with pytest.raises(ValueError, match="scale: must be > 0"):
        parse_softbodies(art, tmp_path)


def test_parse_softbodies_bad_top_level_type(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="softbodies must be a list"):
        parse_softbodies({"softbodies": {"name": "pad"}}, tmp_path)


# -- robot_pack carry-through ------------------------------------------------


def _write_legacy_pack(pack_dir: Path) -> None:
    """Write a minimal legacy robot.yaml with a softbodies block."""
    pack_dir.mkdir(parents=True, exist_ok=True)
    (pack_dir / "models").mkdir(exist_ok=True)
    (pack_dir / "models" / "robot.urdf").write_text(
        "<robot name='custom_hand'><link name='base_link'/></robot>"
    )
    (pack_dir / "softbody").mkdir(exist_ok=True)
    _write_minimal_npz(pack_dir / "softbody" / "a.npz")
    cfg = {
        "robot": {
            "source": "urdf",
            "source_rel": "models/robot.urdf",
            "base_position": [0.0, 0.0, 0.0],
        },
        "sim": {"physics_hz": 60, "substeps": 10, "solver": "featherstone"},
        "joint_names": [],
        "home_pose": {},
        "drive": {"mode": "position", "stiffness": 100.0, "damping": 10.0},
        "softbodies": [
            {
                "name": "left_fingertip_pad",
                "asset_rel": "softbody/a.npz",
                "attach": {"body": "left_fingertip", "vertex_indices": [0, 1]},
                "material": {"density": 100.0, "k_mu": 1e6, "k_lambda": 1e6, "k_damp": 1e-6},
            }
        ],
    }
    (pack_dir / "robot.yaml").write_text(yaml.safe_dump(cfg))


def test_load_pack_carries_softbodies_through(tmp_path: Path) -> None:
    pack = tmp_path / "custom_hand"
    _write_legacy_pack(pack)
    scene = load_pack(pack)

    # Carried into the canonical articulation entry
    art = scene["worlds"][0]["articulations"][0]
    assert "softbodies" in art and len(art["softbodies"]) == 1
    assert art["softbodies"][0]["name"] == "left_fingertip_pad"

    # And mirrored to the top level for legacy single-articulation readers
    assert scene["softbodies"][0]["name"] == "left_fingertip_pad"

    # parse_softbodies on the articulation entry resolves the path
    [spec] = parse_softbodies(art, scene["_pack_dir"])
    assert spec.asset_path == (pack / "softbody/a.npz").resolve()

    # And we can actually load the asset that was written
    arr = load_softbody_npz(spec.asset_path)
    assert arr.vertex_count == 4 and arr.tet_count == 1


def test_load_pack_without_softbodies_is_empty(tmp_path: Path) -> None:
    """Existing packs (no softbodies key) must end up with an empty list."""
    pack = tmp_path / "plain_robot"
    pack.mkdir()
    (pack / "models").mkdir()
    (pack / "models" / "robot.urdf").write_text(
        "<robot name='plain'><link name='base_link'/></robot>"
    )
    cfg = {
        "robot": {"source": "urdf", "source_rel": "models/robot.urdf"},
        "sim": {"physics_hz": 60, "solver": "xpbd"},
        "joint_names": [],
        "home_pose": {},
        "drive": {"mode": "position", "stiffness": 0.0, "damping": 0.0},
    }
    (pack / "robot.yaml").write_text(yaml.safe_dump(cfg))

    scene = load_pack(pack)
    assert scene["softbodies"] == []
    assert scene["worlds"][0]["articulations"][0]["softbodies"] == []
    assert parse_softbodies(scene["worlds"][0]["articulations"][0], pack) == []
