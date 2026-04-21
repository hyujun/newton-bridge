"""Pack loader.

Historically a pack is `robots/<name>/robot.yaml` describing one robot. Phase 2
introduces `scene.yaml` as the canonical multi-articulation format. The loader
accepts either — when only `robot.yaml` exists it is auto-promoted to a
scene-shaped dict, so downstream consumers (NewtonWorld, SimBridgeNode) see a
uniform structure.

Scene shape (canonical):
    sim: {...}                      # physics_hz, solver, substeps, ground_plane, gravity
    worlds:
      - label: str
        gravity: [x,y,z] | None     # overrides sim.gravity
        articulations:
          - label: str              # unique within scene
            source: urdf | mjcf | xacro
            source_rel: str         # relative to pack dir (xacro: .xacro file)
            source_args: {key: val} # xacro-only, passed as xacro mappings
            xform: {pos:[x,y,z], rot:[x,y,z,w]}   # default identity
            articulation_pattern: str (fnmatch, default "*")
            joint_names: [str, ...] # ROS-exposed subset
            home_pose: {name: rad}
            drive: {mode, stiffness, damping}
            joints: {name: {drive: {...}, armature, effort_limit, ...}}  # per-joint override
    ros: {...}                       # joint_states_topic, ..., primary_articulation, publish_tf, ...
    _pack_dir: Path

In addition, the loader mirrors the **primary articulation**'s fields (robot,
joint_names, home_pose, drive, joints, articulation_pattern) at the top level.
This lets existing single-articulation consumers read them without knowing the
scene structure, while scene-aware code can walk `worlds`.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


def _deep_merge(dst: dict, src: dict) -> dict:
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def _promote_robot_yaml(cfg: dict, pack_dir: Path) -> dict:
    """Take a legacy `robot.yaml` dict and return an equivalent scene dict."""
    robot = cfg.get("robot", {}) or {}
    base_pos = robot.get("base_position", [0.0, 0.0, 0.0])
    articulation_label = pack_dir.name  # synthetic, unique within the scene

    articulation: dict[str, Any] = {
        "label": articulation_label,
        "source": robot.get("source"),
        "source_rel": robot.get("source_rel"),
        "source_args": dict(robot.get("source_args", {}) or {}),
        "xform": {"pos": list(base_pos), "rot": [0.0, 0.0, 0.0, 1.0]},
        "articulation_pattern": cfg.get("articulation_pattern", "*"),
        "joint_names": list(cfg.get("joint_names", []) or []),
        "home_pose": dict(cfg.get("home_pose", {}) or {}),
        "drive": dict(cfg.get("drive", {}) or {}),
        "joints": dict(cfg.get("joints", {}) or {}),
    }

    scene: dict[str, Any] = {
        "sim": dict(cfg.get("sim", {}) or {}),
        "worlds": [
            {
                "label": "env0",
                "gravity": (cfg.get("sim", {}) or {}).get("gravity"),
                "articulations": [articulation],
            }
        ],
        "ros": dict(cfg.get("ros", {}) or {}),
    }
    scene["ros"].setdefault("primary_articulation", articulation_label)
    # Optional scene-level blocks introduced in later phases — carry through if
    # present on a legacy robot.yaml (e.g. sensors: from Phase 5).
    if "sensors" in cfg:
        scene["sensors"] = cfg["sensors"]
    return scene


def _validate_scene(scene: dict, pack_dir: Path) -> None:
    worlds = scene.get("worlds") or []
    if not worlds:
        raise ValueError(f"{pack_dir}: scene has no worlds")
    if len(worlds) > 1:
        raise NotImplementedError(
            f"{pack_dir}: multi-world scenes are not yet supported "
            f"(found {len(worlds)} worlds). Use a single world for now."
        )
    for w in worlds:
        arts = w.get("articulations") or []
        if not arts:
            raise ValueError(f"{pack_dir}: world {w.get('label')!r} has no articulations")
        if len(arts) > 1:
            raise NotImplementedError(
                f"{pack_dir}: world {w.get('label')!r} has {len(arts)} articulations; "
                "multi-articulation worlds are not yet supported (Phase 2 scope is "
                "single-robot scenes). Use one articulation per world for now."
            )

    ros = scene.get("ros") or {}
    primary = ros.get("primary_articulation")
    if primary is None:
        # Default: first articulation of first world.
        ros["primary_articulation"] = worlds[0]["articulations"][0].get("label")
        scene["ros"] = ros

    labels = [a.get("label") for a in worlds[0]["articulations"]]
    if ros["primary_articulation"] not in labels:
        raise ValueError(
            f"{pack_dir}: ros.primary_articulation={ros['primary_articulation']!r} "
            f"not among world articulations {labels}"
        )


def _flatten_primary_aliases(scene: dict) -> None:
    """Mirror the primary articulation's fields at the top level for legacy readers."""
    primary_label = scene["ros"]["primary_articulation"]
    primary = next(
        a for a in scene["worlds"][0]["articulations"] if a.get("label") == primary_label
    )
    # Top-level `robot:` slice for backward compat with NewtonWorld._build_model.
    scene["robot"] = {
        "source": primary["source"],
        "source_rel": primary["source_rel"],
        "source_args": dict(primary.get("source_args", {}) or {}),
        "base_position": list(primary.get("xform", {}).get("pos", [0.0, 0.0, 0.0])),
    }
    scene["articulation_pattern"] = primary.get("articulation_pattern", "*")
    scene["joint_names"] = list(primary.get("joint_names", []) or [])
    scene["home_pose"] = dict(primary.get("home_pose", {}) or {})
    scene["drive"] = dict(primary.get("drive", {}) or {})
    scene["joints"] = dict(primary.get("joints", {}) or {})


def load_pack(pack_dir: Path) -> dict:
    scene_path = pack_dir / "scene.yaml"
    robot_path = pack_dir / "robot.yaml"

    if scene_path.is_file():
        with scene_path.open() as fh:
            scene = yaml.safe_load(fh) or {}
    elif robot_path.is_file():
        with robot_path.open() as fh:
            cfg = yaml.safe_load(fh) or {}
        scene = _promote_robot_yaml(cfg, pack_dir)
    else:
        raise FileNotFoundError(f"neither scene.yaml nor robot.yaml under {pack_dir}")

    scene = copy.deepcopy(scene)
    scene.setdefault("sim", {})
    scene.setdefault("ros", {})
    # Defaults injected here so downstream readers don't re-encode them.
    # viewer_hz: target viewer FPS in **wall-clock** time (decoupled from
    # physics_hz and command rate). 0/None disables throttling.
    # sync_timeout_ms: in sync mode, republish last state if no /joint_command
    # arrives within this window.
    scene["sim"].setdefault("viewer_hz", 60)
    scene["ros"].setdefault("sync_timeout_ms", 100)
    _validate_scene(scene, pack_dir)
    _flatten_primary_aliases(scene)
    scene["_pack_dir"] = pack_dir
    return scene
