"""PR 3: hybrid step path for softbody worlds.

Verifies that `world.step()` in softbody mode actually pins the attach
particles to the configured body's transform, that the rigid-only path
still works for packs without softbodies, and that the captured CUDA
graph (when available) doesn't crash on second launch.

These tests touch Newton/warp APIs directly, so they `pytest.skip` on
the host (mirroring `test_softbody_world.py`).
"""

from __future__ import annotations

import importlib
from pathlib import Path

import numpy as np
import pytest


try:
    importlib.import_module("newton")
    importlib.import_module("warp")
    world_mod = importlib.import_module("newton_bridge.world")
except Exception:
    pytest.skip(
        "world.py imports Newton/warp — container-only integration check",
        allow_module_level=True,
    )


# Fixed-joint URDF: `tip` is offset 0.5m above `base` along z. Because the
# joint is fixed and the URDF importer collapses fixed joints, `tip`'s body
# transform is deterministic — we can assert the attach particle landed at
# exactly `tip_pos + R · local_offset`.
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
    vertices = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    tet_indices = np.array([[0, 1, 2, 3]], dtype=np.int32)
    np.savez(path, vertices=vertices, tet_indices=tet_indices)


def _make_pack(tmp_path: Path, with_softbody: bool = True) -> dict:
    urdf_path = tmp_path / "rig.urdf"
    urdf_path.write_text(_URDF_XML)
    pack: dict = {
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
        "softbodies": [],
    }
    if with_softbody:
        npz_path = tmp_path / "pad.npz"
        _write_npz(npz_path)
        pack["softbodies"] = [
            {
                "name": "pad",
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
                "attach": {"body": "tip", "vertex_indices": [0, 2]},
                "contact": {"ke": 2.0e6, "kd": 1.0e-7, "mu": 0.5},
            }
        ]
    return pack


def _build_world(pack: dict):
    NewtonWorld = world_mod.NewtonWorld
    try:
        return NewtonWorld(pack, device="cuda:0")
    except Exception:
        return NewtonWorld(pack, device="cpu")


def test_hybrid_step_runs(tmp_path: Path) -> None:
    """One step() call in hybrid mode must not raise and must advance time."""
    w = _build_world(_make_pack(tmp_path))
    assert w.soft_solver is not None  # sanity: hybrid mode active
    t0 = w.sim_time
    w.step()
    assert w.sim_time > t0


def test_attach_particles_track_body(tmp_path: Path) -> None:
    """After step(), the pinned particles must sit at body_pos + R·local.

    The URDF's `base_to_tip` is fixed, so `tip` body transform is identical
    every step (no joint motion). With identity attach.rot and pos=[0,0,0.5]
    we know the particle world positions analytically from PR 2's local
    offset math, so we just assert the kernel reproduced them.
    """
    w = _build_world(_make_pack(tmp_path))
    w.step()

    meta = w._softbody_attach_meta[0]
    body_index = meta["body_index"]
    local_offsets = meta["local_offsets"]  # (N, 3) float32
    particle_idx = meta["particle_indices"]  # (N,) int32

    body_q = w.state_0.body_q.numpy()  # (B, 7) — px py pz qx qy qz qw
    if body_q.ndim == 1:
        body_q = body_q.reshape(-1, 7)
    p = body_q[body_index, 0:3].astype(np.float32)
    q = body_q[body_index, 3:7].astype(np.float32)
    R = world_mod.NewtonWorld._quat_to_matrix(q)

    expected = (R @ local_offsets.T).T + p

    pq = w.state_0.particle_q.numpy()
    actual = pq[particle_idx]

    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-4)


def test_step_is_idempotent_under_capture(tmp_path: Path) -> None:
    """Two step() calls (potentially under graph capture) must not crash."""
    w = _build_world(_make_pack(tmp_path))
    w.step()
    w.step()
    # If we reach here without raising, the captured graph (or eager fallback)
    # is replayable. Particle data should still be finite.
    pq = w.state_0.particle_q.numpy()
    assert np.all(np.isfinite(pq))


def test_rigid_only_path_unchanged(tmp_path: Path) -> None:
    """Pack without softbodies must run the original rigid-only kernel."""
    w = _build_world(_make_pack(tmp_path, with_softbody=False))
    assert w.soft_solver is None
    assert w.collision_pipeline is None
    t0 = w.sim_time
    w.step()
    assert w.sim_time > t0
