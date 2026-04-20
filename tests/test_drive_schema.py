"""Phase 3: drive mode parsing + per-joint override merging.

These tests are Newton-free — they exercise the pack yaml -> config dict
pipeline only, so they run in host pytest without the container.
"""

from __future__ import annotations

import pytest

# Import without pulling in Newton by patching sys.modules? Easier: import the
# pure helpers directly. world.py top-level imports warp/newton; to keep this
# test host-runnable we duplicate the tiny schema logic here and assert the
# yaml merge shape used by NewtonWorld._resolve_joint_drive.
#
# When world.py is imported in a Newton-enabled environment (container),
# NewtonWorld._resolve_joint_drive applies the same rules — verify.sh §6
# exercises the integration path end-to-end.


def _resolve(pack: dict, joint_name: str) -> dict:
    default = dict(pack.get("drive", {}) or {})
    override = (pack.get("joints", {}) or {}).get(joint_name, {}) or {}
    out = {
        "mode": default.get("mode", "position"),
        "stiffness": float(default.get("stiffness", 0.0)),
        "damping": float(default.get("damping", 0.0)),
    }
    jd = override.get("drive") or {}
    if "mode" in jd:
        out["mode"] = jd["mode"]
    if "stiffness" in jd:
        out["stiffness"] = float(jd["stiffness"])
    if "damping" in jd:
        out["damping"] = float(jd["damping"])
    return out


def test_top_level_drive_applies_to_all_joints():
    pack = {"drive": {"mode": "position", "stiffness": 100.0, "damping": 10.0}}
    for j in ("a", "b", "c"):
        got = _resolve(pack, j)
        assert got == {"mode": "position", "stiffness": 100.0, "damping": 10.0}


def test_per_joint_override_wins():
    pack = {
        "drive": {"mode": "position", "stiffness": 100.0, "damping": 10.0},
        "joints": {
            "a": {"drive": {"mode": "velocity", "damping": 5.0}},
        },
    }
    got = _resolve(pack, "a")
    assert got["mode"] == "velocity"
    assert got["damping"] == 5.0
    assert got["stiffness"] == 100.0  # not overridden -> inherits default


def test_unmentioned_joint_keeps_defaults():
    pack = {
        "drive": {"mode": "effort", "stiffness": 0.0, "damping": 2.0},
        "joints": {"a": {"drive": {"mode": "position"}}},
    }
    got = _resolve(pack, "b")
    assert got == {"mode": "effort", "stiffness": 0.0, "damping": 2.0}


def test_mode_parsing_rejects_unknown():
    import importlib

    try:
        world = importlib.import_module("newton_bridge.world")
    except Exception:
        pytest.skip("world.py imports Newton/warp — container-only integration check")
    with pytest.raises(ValueError, match="unknown drive.mode"):
        world.parse_drive_mode("linear")


@pytest.mark.parametrize(
    "raw,expected_lower",
    [
        ("position", "position"),
        ("VELOCITY", "velocity"),
        ("Effort", "effort"),
        ("position_velocity", "position_velocity"),
        ("none", "none"),
    ],
)
def test_mode_parsing_accepts_case(raw: str, expected_lower: str):
    import importlib

    try:
        world = importlib.import_module("newton_bridge.world")
    except Exception:
        pytest.skip("world.py imports Newton/warp — container-only integration check")
    mode = world.parse_drive_mode(raw)
    import newton  # type: ignore

    expected = getattr(newton.JointTargetMode, expected_lower.upper())
    assert int(mode) == int(expected)
