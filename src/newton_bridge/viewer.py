"""Optional Newton GL viewer. Imported lazily so headless runs skip GL init."""

from __future__ import annotations

import os


def build_viewer(world):
    """Construct newton.viewer.ViewerGL and bind it to the model."""
    from newton.viewer import ViewerGL  # noqa: WPS433 (intentional local import)

    width = int(os.environ.get("VIEWER_WIDTH", "1280"))
    height = int(os.environ.get("VIEWER_HEIGHT", "720"))
    viewer = ViewerGL(width=width, height=height, vsync=False)
    viewer.set_model(world.model)
    return viewer
