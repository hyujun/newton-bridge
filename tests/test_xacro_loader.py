"""Smoke test for the xacro → URDF string loader.

Skips gracefully when the `xacro` package is not importable (e.g. on a
dev machine without ROS sourced). Run inside the container or with
`source /opt/ros/jazzy/setup.bash` to exercise this path.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


xacro = pytest.importorskip("xacro")


def test_process_ur5e_xacro() -> None:
    from newton_bridge.xacro_loader import process_xacro

    xml = process_xacro(
        REPO_ROOT / "robots/ur5e/models/ur.urdf.xacro",
        args={"name": "ur5e", "ur_type": "ur5e"},
    )
    assert xml.lstrip().startswith("<?xml") or "<robot" in xml
    # The xacro expands 6 revolute UR5e joints; they should all appear.
    for joint in (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ):
        assert joint in xml, f"joint {joint!r} missing from expanded URDF"


def test_missing_args_raises() -> None:
    from newton_bridge.xacro_loader import process_xacro

    # ur_type default in the upstream xacro is a placeholder ("ur5x") that
    # points at non-existent config files — loading must surface a clean
    # RuntimeError rather than silently producing garbage.
    with pytest.raises(RuntimeError):
        process_xacro(REPO_ROOT / "robots/ur5e/models/ur.urdf.xacro", args={})
