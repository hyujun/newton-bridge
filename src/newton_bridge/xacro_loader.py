"""Xacro → URDF in-process conversion.

Thin wrapper over the `xacro` Python module so `NewtonWorld._build_model` can
feed the resulting URDF XML string directly to `builder.add_urdf`, which
accepts either a file path or XML content.

The `xacro` package ships with `ros-jazzy-xacro`. `$(find <pkg>)` substitutions
are resolved via `ament_index_python` using `AMENT_PREFIX_PATH` — the Docker
entrypoint already sources `/opt/ros/jazzy/setup.bash` so that happens for
free. `source_args` values are stringified before being passed as xacro
`mappings=` (xacro only accepts str→str).
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping


def process_xacro(path: Path | str, args: Mapping[str, object] | None = None) -> str:
    """Return URDF XML content produced by running xacro on `path`.

    Raises ImportError if the `xacro` package is not installed (apt:
    ros-jazzy-xacro or pip: xacro). Raises RuntimeError on xacro syntax
    errors or unresolved `$(find ...)` references.
    """
    try:
        import xacro
    except ImportError as exc:
        raise ImportError(
            "xacro package not installed. In the container: "
            "`apt install ros-jazzy-xacro`; on host: `pip install xacro`."
        ) from exc

    mappings = {str(k): str(v) for k, v in (args or {}).items()}
    try:
        doc = xacro.process_file(str(path), mappings=mappings)
    except xacro.XacroException as exc:
        raise RuntimeError(f"xacro processing failed for {path}: {exc}") from exc
    return doc.toxml()
