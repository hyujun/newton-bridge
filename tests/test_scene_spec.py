"""Phase 2: scene.yaml parsing + robot.yaml auto-shim.

The loader emits a unified "scene" dict regardless of which file is on disk.
Top-level aliases (robot, joint_names, home_pose, drive, joints,
articulation_pattern) mirror the primary articulation for backward compat
with single-articulation consumers (NewtonWorld, SimBridgeNode).
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from newton_bridge.robot_pack import load_pack


REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOTS = REPO_ROOT / "robots"


@pytest.mark.parametrize("name", ["ur5e", "franka", "kuka_iiwa_14"])
def test_legacy_robot_yaml_shims_to_scene(name: str) -> None:
    scene = load_pack(ROBOTS / name)
    # Scene shape
    assert "worlds" in scene and len(scene["worlds"]) == 1
    world = scene["worlds"][0]
    assert len(world["articulations"]) == 1
    art = world["articulations"][0]
    # Label defaults to the pack dir name
    assert art["label"] == name
    assert scene["ros"]["primary_articulation"] == name

    # Legacy top-level aliases still present
    assert scene["robot"]["source"] in {"urdf", "xacro", "mjcf"}
    assert scene["joint_names"], "joint_names must be mirrored"
    assert "drive" in scene


def test_scene_yaml_direct_load(tmp_path: Path) -> None:
    # Build a minimal pack with scene.yaml only (no robot.yaml)
    pack = tmp_path / "mini"
    pack.mkdir()
    (pack / "models").mkdir()
    # Not executed, just needs to exist for relative-path expectations.
    (pack / "models" / "fake.urdf").write_text("<robot/>\n")

    scene_cfg = {
        "sim": {"physics_hz": 240, "solver": "xpbd", "ground_plane": True},
        "worlds": [
            {
                "label": "env0",
                "gravity": [0, 0, -9.81],
                "articulations": [
                    {
                        "label": "arm_a",
                        "source": "urdf",
                        "source_rel": "models/fake.urdf",
                        "xform": {"pos": [0.1, 0.2, 0.3], "rot": [0, 0, 0, 1]},
                        "joint_names": ["j1"],
                        "home_pose": {"j1": 0.5},
                        "drive": {"mode": "position", "stiffness": 100, "damping": 10},
                    }
                ],
            }
        ],
        "ros": {"publish_rate_hz": 50, "primary_articulation": "arm_a"},
    }
    (pack / "scene.yaml").write_text(yaml.safe_dump(scene_cfg))

    s = load_pack(pack)
    assert s["ros"]["primary_articulation"] == "arm_a"
    assert s["robot"]["source"] == "urdf"
    assert s["robot"]["base_position"] == [0.1, 0.2, 0.3]
    assert s["joint_names"] == ["j1"]
    assert s["home_pose"] == {"j1": 0.5}


def test_missing_pack_raises(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_pack(tmp_path)


def test_multi_world_rejected(tmp_path: Path) -> None:
    pack = tmp_path / "mw"
    pack.mkdir()
    scene = {
        "sim": {"physics_hz": 240},
        "worlds": [
            {"label": "env0", "articulations": [{"label": "a", "source": "urdf",
                                                 "source_rel": "m.urdf", "joint_names": ["j"]}]},
            {"label": "env1", "articulations": [{"label": "b", "source": "urdf",
                                                 "source_rel": "m.urdf", "joint_names": ["j"]}]},
        ],
        "ros": {},
    }
    (pack / "scene.yaml").write_text(yaml.safe_dump(scene))
    with pytest.raises(NotImplementedError, match="multi-world"):
        load_pack(pack)


def test_multi_articulation_rejected(tmp_path: Path) -> None:
    pack = tmp_path / "ma"
    pack.mkdir()
    scene = {
        "sim": {"physics_hz": 240},
        "worlds": [
            {
                "label": "env0",
                "articulations": [
                    {"label": "first", "source": "urdf", "source_rel": "m.urdf",
                     "joint_names": ["j"]},
                    {"label": "second", "source": "urdf", "source_rel": "m.urdf",
                     "joint_names": ["j"]},
                ],
            }
        ],
        "ros": {},
    }
    (pack / "scene.yaml").write_text(yaml.safe_dump(scene))
    with pytest.raises(NotImplementedError, match="multi-articulation"):
        load_pack(pack)


def test_primary_articulation_defaults_to_only_one(tmp_path: Path) -> None:
    pack = tmp_path / "def"
    pack.mkdir()
    scene = {
        "sim": {"physics_hz": 240},
        "worlds": [
            {
                "label": "env0",
                "articulations": [
                    {"label": "solo", "source": "urdf", "source_rel": "m.urdf",
                     "joint_names": ["j"]},
                ],
            }
        ],
        "ros": {},  # no primary_articulation specified
    }
    (pack / "scene.yaml").write_text(yaml.safe_dump(scene))
    s = load_pack(pack)
    assert s["ros"]["primary_articulation"] == "solo"
