"""
Mirrors: private copies of a scene in one collision backend each, for planners.

- ! A mirror belongs to one thread, usually a planner's worker. Backend ids never leave it.
- ! A disabled body stands at HIDDEN_POSITION in the backend: leaving it out of the checks is not enough, since a
  PyBullet window and code reading the backend directly (tamp after `lend`) would still see it.
"""

from __future__ import annotations

import sys
from os import environ

#: Where mirrors put disabled bodies (and PyBulletMirror absent robots): far from everything that moves, and from
#: PARKED_POSITION, where the compas_fab cell parks absent robots.
HIDDEN_POSITION = (-50.0, -50.0, -50.0)


def check_display() -> None:
    """Raise unless a PyBullet window can open: on Linux without an X display, opening one exits the whole process.

    ? Windows and macOS need no X display.

    Raises:
        RuntimeError: On Linux, if DISPLAY is not set.
    """
    if sys.platform.startswith("linux") and not environ.get("DISPLAY"):
        raise RuntimeError("no X display to open a PyBullet window on")
