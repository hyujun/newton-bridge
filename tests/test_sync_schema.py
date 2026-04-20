"""Sync-mode schema defaults: viewer_hz, sync_timeout_ms.

Newton-free — exercises only the yaml -> dict defaulting path.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from newton_bridge.robot_pack import load_pack

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOTS = REPO_ROOT / "robots"


@pytest.mark.parametrize("name", ["ur5e", "franka", "kuka_iiwa_14"])
def test_defaults_injected(name: str) -> None:
    cfg = load_pack(ROBOTS / name)
    assert cfg["sim"]["viewer_hz"] == 60
    assert cfg["ros"]["sync_timeout_ms"] == 100


def test_explicit_values_preserved(tmp_path: Path) -> None:
    pack = tmp_path / "custom"
    (pack / "models").mkdir(parents=True)
    (pack / "models" / "x.urdf").write_text("<robot name='x'/>")
    (pack / "robot.yaml").write_text(yaml.safe_dump({
        "robot": {"source": "urdf", "source_rel": "models/x.urdf"},
        "sim": {"physics_hz": 500, "viewer_hz": 30},
        "joint_names": ["j1"],
        "ros": {
            "joint_states_topic": "/joint_states",
            "joint_command_topic": "/joint_command",
            "sync_timeout_ms": 250,
        },
    }))
    cfg = load_pack(pack)
    assert cfg["sim"]["viewer_hz"] == 30
    assert cfg["ros"]["sync_timeout_ms"] == 250


def test_zero_viewer_hz_passes_through(tmp_path: Path) -> None:
    pack = tmp_path / "custom"
    (pack / "models").mkdir(parents=True)
    (pack / "models" / "x.urdf").write_text("<robot name='x'/>")
    (pack / "robot.yaml").write_text(yaml.safe_dump({
        "robot": {"source": "urdf", "source_rel": "models/x.urdf"},
        "sim": {"physics_hz": 500, "viewer_hz": 0},
        "joint_names": ["j1"],
        "ros": {"joint_states_topic": "/a", "joint_command_topic": "/b"},
    }))
    cfg = load_pack(pack)
    assert cfg["sim"]["viewer_hz"] == 0
