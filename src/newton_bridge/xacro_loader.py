"""Xacro → URDF in-process conversion.

Thin wrapper over the `xacro` Python module so `NewtonWorld._build_model` can
feed the resulting URDF XML string directly to `builder.add_urdf`, which
accepts either a file path or XML content.

The pack's xacro files have every `$(find ur_description)` rewritten to the
sentinel `@@NEWTON_BRIDGE_PACK_DIR@@` by `scripts/host/fetch_assets.sh`. We
substitute the sentinel for `<pack>/models/` before running xacro so all
includes and config-yaml refs resolve INSIDE the pack.

But `force_abs_paths=true` causes the upstream ur_macro to also emit mesh
URIs of the form `file://$(find ur_description)/meshes/ur5e/visual/base.dae`
where `ur_description` is loaded as a string out of visual_parameters.yaml
(i.e. NOT a literal `$(find ur_description)` token we could patch in the
xacro file). xacro evaluates that `$(find ...)` against ament_index, which
points at the container's ROS share. We post-process the output URDF to
rewrite every `file://<ros-share>/{meshes,config,urdf}/...` segment to the
matching pack-local path, then assert no /opt/ros or package:// references
remain — silent fallback to the ROS share at mesh-load time is exactly the
failure mode this module exists to prevent.
"""

from __future__ import annotations

import re
import tempfile
from pathlib import Path
from typing import Mapping

PACK_DIR_SENTINEL = "@@NEWTON_BRIDGE_PACK_DIR@@"

# Forbidden patterns in the post-xacro URDF. Hitting any of these means a
# `$(find ur_description)` reference slipped past fetch_assets.sh, or the
# pack was assembled by hand and is not self-contained.
_FORBIDDEN_URI_PATTERNS = (
    re.compile(r"package://"),
    re.compile(r"/opt/ros/"),
    re.compile(re.escape(PACK_DIR_SENTINEL)),
)


def process_xacro(path: Path | str, args: Mapping[str, object] | None = None) -> str:
    """Return URDF XML content produced by running xacro on `path`.

    `path` must live inside a robot pack (`robots/<name>/models/<file>.xacro`).
    The pack's parent of `models/` is treated as the resolution root for the
    `@@NEWTON_BRIDGE_PACK_DIR@@` sentinel that `fetch_assets.sh` injects in
    place of `$(find ur_description)`.

    Raises ImportError if the `xacro` package is not installed (apt:
    ros-jazzy-xacro or pip: xacro). Raises RuntimeError on xacro syntax
    errors, unresolved refs, or if the output contains `package://` /
    `/opt/ros/` paths (i.e. the pack is not self-contained).
    """
    try:
        import xacro
    except ImportError as exc:
        raise ImportError(
            "xacro package not installed. In the container: "
            "`apt install ros-jazzy-xacro`; on host: `pip install xacro`."
        ) from exc

    src = Path(path).resolve()
    # `<pack>/models/<file>.xacro` → pack_models_dir = `<pack>/models`. The
    # ur_description tree we copied in mirrors `<pack>/models/{urdf,meshes,
    # config}` against ur_description's `share/{urdf,meshes,config}`, so
    # substituting the sentinel with `<pack>/models` keeps `$(find
    # ur_description)/meshes/...` style paths valid.
    pack_models_dir = src.parent
    if pack_models_dir.name != "models":
        raise RuntimeError(
            f"xacro source must live under <pack>/models/, got: {src}"
        )

    # Materialize a sentinel-substituted copy in a temp dir so xacro reads
    # real paths. We can't pass XML content to xacro.process_file(); it must
    # be a file (and resolves includes relative to that file's dir).
    raw = src.read_text()
    if PACK_DIR_SENTINEL not in raw and "find ur_description" in raw:
        raise RuntimeError(
            f"{src} still contains `$(find ur_description)` — re-run "
            "scripts/host/fetch_assets.sh to apply the sentinel patch."
        )
    patched = raw.replace(PACK_DIR_SENTINEL, str(pack_models_dir))

    # Also need to patch sibling includes the same way, since xacro reads
    # them from disk. Easiest: stage a sibling-mirrored tempdir.
    with tempfile.TemporaryDirectory(prefix="newton_bridge_xacro_") as tmp:
        tmp_dir = Path(tmp)
        for f in pack_models_dir.rglob("*.xacro"):
            rel = f.relative_to(pack_models_dir)
            dst = tmp_dir / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            text = f.read_text().replace(PACK_DIR_SENTINEL, str(pack_models_dir))
            dst.write_text(text)
        # The substituted file we'll actually feed to xacro
        tmp_src = tmp_dir / src.name
        tmp_src.write_text(patched)

        mappings = {str(k): str(v) for k, v in (args or {}).items()}
        try:
            doc = xacro.process_file(str(tmp_src), mappings=mappings)
        except xacro.XacroException as exc:
            raise RuntimeError(f"xacro processing failed for {src}: {exc}") from exc
        urdf_xml = doc.toxml()

    urdf_xml = _rewrite_ros_share_to_pack(urdf_xml, pack_models_dir)
    _assert_self_contained(urdf_xml, src)
    return urdf_xml


def _rewrite_ros_share_to_pack(urdf_xml: str, pack_models_dir: Path) -> str:
    """Rewrite `file://<ros-share>/{meshes,config,urdf}/...` to pack-local paths.

    `force_abs_paths=true` plus a `package: ur_description` in
    visual_parameters.yaml makes xacro emit mesh URIs that point at the ROS
    share via ament resolution. fetch_assets.sh has already mirrored the
    relevant subtrees into `<pack>/models/`, so we just swap the prefix.
    """
    # Match e.g. `file:///opt/ros/jazzy/share/ur_description/meshes/...`.
    # `<package-name>` is alphanumeric + underscore.
    pattern = re.compile(
        r"file://(?:/opt/ros/[^/]+)?/share/[A-Za-z0-9_]+/(?=(?:meshes|config|urdf)/)"
    )
    pack_prefix = f"file://{pack_models_dir}/"
    return pattern.sub(pack_prefix, urdf_xml)


def _assert_self_contained(urdf_xml: str, src: Path) -> None:
    """Fail loud if the URDF still references ROS share or package:// URIs.

    Silent fallback to /opt/ros at mesh-load time is exactly what we're
    trying to prevent — robot packs must be self-contained.
    """
    for pattern in _FORBIDDEN_URI_PATTERNS:
        m = pattern.search(urdf_xml)
        if m:
            raise RuntimeError(
                f"xacro output for {src} contains forbidden reference "
                f"{m.group()!r} — pack is not self-contained. Re-run "
                "scripts/host/fetch_assets.sh and verify robots/<name>/models/ "
                "has been fully patched."
            )
