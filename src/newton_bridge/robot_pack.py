"""Robot pack loader: parses `robots/<name>/robot.yaml` into a plain dict."""

from __future__ import annotations

from pathlib import Path

import yaml


def load_pack(pack_dir: Path) -> dict:
    cfg_path = pack_dir / "robot.yaml"
    if not cfg_path.is_file():
        raise FileNotFoundError(f"robot.yaml not found under {pack_dir}")
    with cfg_path.open() as fh:
        cfg = yaml.safe_load(fh)
    cfg["_pack_dir"] = pack_dir
    return cfg
