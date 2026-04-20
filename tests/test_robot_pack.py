"""Smoke tests for pack loading — run-of-the-mill YAML parse path.

These tests do not touch Newton/Warp/rclpy, so they can run anywhere with
just `pip install -e .[dev]`.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from newton_bridge.robot_pack import load_pack

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOTS = REPO_ROOT / "robots"


@pytest.mark.parametrize("name", ["ur5e", "franka", "kuka_iiwa_14"])
def test_pack_parses(name: str) -> None:
    cfg = load_pack(ROBOTS / name)
    assert cfg["_pack_dir"] == ROBOTS / name
    assert cfg["robot"]["source"] in {"urdf", "mjcf"}
    assert cfg["robot"]["source_rel"].startswith("models/")
    assert cfg["sim"]["physics_hz"] > 0
    assert cfg["joint_names"], "joint_names must be non-empty"
    for joint in cfg.get("home_pose", {}):
        assert joint in cfg["joint_names"], f"home_pose key {joint!r} not in joint_names"


def test_missing_pack_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_pack(tmp_path)


@pytest.mark.parametrize("name", ["ur5e", "franka", "kuka_iiwa_14"])
def test_articulation_pattern_optional(name: str) -> None:
    """Phase 1: articulation_pattern is an optional pack field; default `.*`."""
    cfg = load_pack(ROBOTS / name)
    pattern = cfg.get("articulation_pattern", ".*")
    assert isinstance(pattern, str) and pattern, "articulation_pattern must be a non-empty string"
