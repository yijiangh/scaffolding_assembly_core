"""Tests for the core on Python 3.9 (Rhino 8), in an environment without ROS: syntax, and imports via uv."""

import ast
import os
import shutil
import subprocess
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CORE = PACKAGE_ROOT / "bar_assembly_core"
#: Pure-core modules: they must import with numpy, scipy, trimesh and yourdfpy alone.
PURE = ("bar_assembly_core", "bar_assembly_core.geometry", "bar_assembly_core.ids", "bar_assembly_core.urdf",
        "bar_assembly_core.kinematics", "bar_assembly_core.robot", "bar_assembly_core.scene",
        "bar_assembly_core.process", "bar_assembly_core.legacy", "bar_assembly_core.legacy.timing",
        *(f"bar_assembly_core.process.{p.stem}" for p in sorted((CORE / "process").glob("*.py")) if p.stem != "__init__"))
#: Modules with extra dependencies (the extras in `pyproject.toml`). legacy.export and legacy.conversion also need
#: rs_data_structure.
EXTRAS = ("bar_assembly_core.mirrors",
          "bar_assembly_core.mirrors.compas", "bar_assembly_core.mirrors.pp_client",
          "bar_assembly_core.mirrors.pybullet",
          "bar_assembly_core.mirrors.compas_fab")
#: The pure core's dependencies, at Rhino's pins.
CORE_PACKAGES = ("numpy", "scipy", "trimesh==4.12.2", "yourdfpy==0.0.60")
#: The mirrors extra (pyproject.toml); compas_fab from its `wip_process` branch.
COMPAS_FAB = "compas_fab @ git+https://github.com/compas-dev/compas_fab.git@995122dd467415971ca842772573a07d6c942145"
EXTRA_PACKAGES = ("pybullet", "compas_robots", COMPAS_FAB, "pybullet_planning==0.6.1", "roslibpy==1.8.1")


def test_syntax_is_python_39():
    """Every core file parses as Python 3.9 and postpones annotations."""
    for path in sorted(CORE.rglob("*.py")):
        source = path.read_text()
        tree = ast.parse(source, str(path), feature_version=(3, 9))
        futures = {alias.name for node in tree.body if isinstance(node, ast.ImportFrom)
                   and node.module == "__future__" for alias in node.names}
        assert "annotations" in futures, f"{path.relative_to(CORE)} lacks `from __future__ import annotations`"


def _run_39(packages: tuple, modules: tuple, extra_check: str = "") -> None:
    """Import `modules` under Python 3.9 with only `packages` and no PYTHONPATH (so no ROS); skip without uv."""
    if shutil.which("uv") is None:
        pytest.skip("uv is not installed")
    code = (f"import sys; sys.path.insert(0, {str(PACKAGE_ROOT)!r})\n"
            + "".join(f"import {name}\n" for name in modules)
            + "assert sys.version_info[:2] == (3, 9), sys.version\n"
            "assert 'rclpy' not in sys.modules and 'husky_assembly_teleop' not in sys.modules\n" + extra_check)
    command = ["uv", "run", "--python", "3.9", "--no-project",
               *(arg for package in packages for arg in ("--with", package)), "python", "-c", code]
    env = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=900, cwd=str(PACKAGE_ROOT), env=env)
    except subprocess.TimeoutExpired:
        pytest.skip("uv took too long (offline?)")
    # ? No Traceback means uv itself failed: Python 3.9 not available, or no network to fetch packages.
    if result.returncode != 0 and "Traceback" not in result.stderr:
        pytest.skip(f"Python 3.9 environment unavailable: {result.stderr.strip()[-300:]}")
    assert result.returncode == 0, result.stderr


def test_core_imports_under_python_39():
    """The pure core imports and runs under Python 3.9 with only numpy, scipy, trimesh and yourdfpy, without compas.

    It writes, reads and plan-checks the test process, and builds a scene of it.
    """
    run = (f"import tempfile, pathlib; sys.path.insert(0, {str(PACKAGE_ROOT / 'tests')!r})\n"
           "from process_fixtures import build_process\n"
           "from bar_assembly_core.process import write\n"
           "from bar_assembly_core.process.plan_check import check_plan\n"
           "folder = pathlib.Path(tempfile.mkdtemp())\n"
           "process = write(build_process(folder), folder / 'out')\n"
           "assert check_plan(process).ok\n"
           "scene = process.scene_at(next(movement for _, movement in process.movements()))\n"
           "assert 'bars/B2' in scene.world_poses and scene.robots['robots/cindy'].unmeasured\n"
           "from bar_assembly_core.scene import scene_parts\n"
           "assert 'robots/cindy/base_link' in {part_id for part_id, _, _ in scene_parts(scene)}\n"
           "assert 'compas' not in sys.modules\n")
    _run_39(CORE_PACKAGES, PURE, run)


@pytest.mark.slow
def test_mirrors_import_under_python_39():
    """The mirrors and the compas conversions import under Python 3.9 with their extras."""
    _run_39(CORE_PACKAGES + EXTRA_PACKAGES, PURE + EXTRAS)
