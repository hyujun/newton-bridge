"""Sync-mode schema: viewer_hz / sync_timeout_ms ownership moved to config.yaml.

Before Phase 8, robot_pack.load_pack() injected defaults for these two keys.
They are now config-yaml only — pack yaml does NOT carry them, and __main__.py
injects the resolved config values into the pack dict at start (see
src/newton_bridge/__main__.py). publish_tf stays a per-pack policy.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from newton_bridge.robot_pack import load_pack

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOTS = REPO_ROOT / "robots"


@pytest.mark.parametrize("name", ["ur5e", "franka", "kuka_iiwa_14"])
def test_viewer_hz_and_sync_timeout_not_injected_by_pack(name: str) -> None:
    """Pack loader must not inject session knobs — config.yaml owns them."""
    cfg = load_pack(ROBOTS / name)
    assert "viewer_hz" not in cfg["sim"]
    assert "sync_timeout_ms" not in cfg["ros"]


@pytest.mark.parametrize("name", ["ur5e", "franka", "kuka_iiwa_14"])
def test_publish_tf_key_present(name: str) -> None:
    """All shipped packs declare the /tf toggle keys so users can discover them.

    The boolean default ships per-pack (true or false) — we only assert the
    keys exist and have the right shape, not their value.
    """
    cfg = load_pack(ROBOTS / name)
    assert isinstance(cfg["ros"]["publish_tf"], bool)
    assert isinstance(cfg["ros"]["tf_root_frame"], str)
    assert isinstance(cfg["ros"]["publish_frames"], list)
