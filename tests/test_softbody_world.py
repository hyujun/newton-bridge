"""PR 2: NewtonWorld softbody build path.

Verifies that a pack with a `softbodies:` block finalizes successfully
and that the resulting world exposes the attach metadata that PR 3's
warp kernel will consume:

- particle count grows by the loaded mesh's vertex count
- attach particle indices map to the configured `vertex_indices`
- `model.particle_mass` is zero at those indices (kinematic pinning)
- a separate `soft_solver` (SolverVBD) and `collision_pipeline` exist

These tests touch Newton/warp APIs directly, so they `pytest.skip` on
the host (mirroring `test_drive_schema.py`'s container-only pattern).
"""

from __future__ import annotations

import importlib
from pathlib import Path

import numpy as np
import pytest


# Skip the whole module on host (no Newton/warp). The container CI runs it.
try:
    importlib.import_module("newton")
    importlib.import_module("warp")
    world_mod = importlib.import_module("newton_bridge.world")
except Exception:
    pytest.skip(
        "world.py imports Newton/warp — container-only integration check",
        allow_module_level=True,
    )


# Minimal single-link URDF: one body called "tip" so attach.body resolves.
# Newton's URDF importer requires a base, so we ship a base + a fixed joint.
_URDF_XML = """<?xml version="1.0"?>
<robot name="rig">
  <link name="base"/>
  <link name="tip"/>
  <joint name="base_to_tip" type="fixed">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
</robot>
"""


def _write_npz(path: Path) -> None:
    # Single tet, four vertices in [0,1]^3 — keeps the soft mesh trivially
    # well-formed for the loader and lets us pin specific indices.
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    tet_indices = np.array([[0, 1, 2, 3]], dtype=np.int32)
    np.savez(path, vertices=vertices, tet_indices=tet_indices)


def _make_pack(tmp_path: Path) -> dict:
    """Build the in-memory `pack` dict NewtonWorld consumes."""
    urdf_path = tmp_path / "rig.urdf"
    urdf_path.write_text(_URDF_XML)
    npz_path = tmp_path / "pad.npz"
    _write_npz(npz_path)

    return {
        "_pack_dir": tmp_path,
        "robot": {
            "source": "urdf",
            "source_rel": "rig.urdf",
            "base_position": [0.0, 0.0, 0.0],
        },
        "sim": {
            "physics_hz": 100,
            "substeps": 1,
            "solver": "xpbd",
            "ground_plane": False,
        },
        "joint_names": [],
        "home_pose": {},
        "drive": {},
        "joints": {},
        "articulation_pattern": "*",
        "softbodies": [
            {
                "name": "fingertip_pad",
                "asset_rel": "pad.npz",
                "pos": [0.0, 0.0, 0.5],
                "rot": [0.0, 0.0, 0.0, 1.0],
                "scale": 1.0,
                "material": {
                    "density": 100.0,
                    "k_mu": 1.0e6,
                    "k_lambda": 1.0e6,
                    "k_damp": 1.0e-6,
                },
                "particle_radius": 0.01,
                "attach": {
                    "body": "tip",
                    "vertex_indices": [0, 2],
                },
                "contact": {"ke": 2.0e6, "kd": 1.0e-7, "mu": 0.5},
            }
        ],
    }


def _build_world(pack: dict):
    """Try cuda first, fall back to cpu — CI may or may not have a GPU."""
    NewtonWorld = world_mod.NewtonWorld
    try:
        return NewtonWorld(pack, device="cuda:0")
    except Exception:
        return NewtonWorld(pack, device="cpu")


def test_finalize_creates_particles(tmp_path: Path) -> None:
    pack = _make_pack(tmp_path)
    w = _build_world(pack)

    # 4 vertices in the npz → 4 particles in the model.
    assert int(w.model.particle_count) == 4
    assert len(w.softbodies) == 1
    assert w._softbody_layout == [(0, 4)]


def test_attach_indices_match_yaml(tmp_path: Path) -> None:
    pack = _make_pack(tmp_path)
    w = _build_world(pack)

    meta = w._softbody_attach_meta[0]
    assert meta["spec_name"] == "fingertip_pad"
    # vertex_indices [0, 2] with start_vertex=0 → particle indices [0, 2].
    np.testing.assert_array_equal(
        meta["particle_indices"], np.asarray([0, 2], dtype=np.int32)
    )
    assert meta["body_index"] >= 0


def test_attach_particles_are_pinned(tmp_path: Path) -> None:
    pack = _make_pack(tmp_path)
    w = _build_world(pack)

    pmass = w.model.particle_mass.numpy()
    # Particles 0 and 2 are pinned (mass=0); 1 and 3 stay non-zero.
    assert float(pmass[0]) == 0.0
    assert float(pmass[2]) == 0.0
    assert float(pmass[1]) > 0.0
    assert float(pmass[3]) > 0.0


def test_soft_solver_and_pipeline_exist(tmp_path: Path) -> None:
    pack = _make_pack(tmp_path)
    w = _build_world(pack)

    assert w.soft_solver is not None
    assert w.collision_pipeline is not None
    # Rigid solver kept under self.solver for non-softbody compat.
    assert w.solver is not None


def test_no_softbody_keeps_existing_behavior(tmp_path: Path) -> None:
    pack = _make_pack(tmp_path)
    pack["softbodies"] = []
    w = _build_world(pack)

    assert int(w.model.particle_count) == 0
    assert w.soft_solver is None
    assert w.collision_pipeline is None
    assert w.attach_particle_idx is None
