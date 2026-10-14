# scaffolding_assembly_core

The shared core of the scaffolding assembly tools, used by the Rhino plugin (`bar_joint_rhino_design_workflow`), the planners
(`husky_assembly_tamp`) and the monitor (`husky-assembly-teleop`). It holds:

- `process/`: the process format, schema 2.1 (`doc/process_format.md`): `read`, `write`, `validate` (file checks A1–A14),
  `plan_check.check_plan` (plan checks B1–B15), `relations`, `scenes` (`Process.scene_at`), `solutions`;
- `geometry`, `ids`, `urdf` (URDFs read with yourdfpy: names, joints, each link's shapes), `kinematics`
  (`link_pose`, `link_poses`), `robot`, `scene` (`scene_parts`: what a viewer draws);
- `mirrors/`: a scene in PyBullet or compas_fab, for planners, and its compas_fab cell without PyBullet, for viewers
  (`mirrors.cell.CompasFabCell(robot, visual=True)`) (extra `[mirrors]`);
- `legacy/`: converts the old compas_fab export into schema 2.1 (extra `[mirrors]` plus `rs_data_structure`).

The layers and their import rules are in `scaffolding_assembly_core/__init__.py`.

- ! Runs on Python 3.9 (Rhino 8's CPython) with numpy, scipy, trimesh and yourdfpy only; known good pins: numpy
  1.24.4, scipy 1.13.1, trimesh 4.12.2, yourdfpy 0.0.60 with lxml 6.1.3 and six 1.17.0. yourdfpy asks for
  `trimesh[easy]`; it needs none of those extras (`pip install --no-deps yourdfpy lxml six` where they fail).
- ! Imports nothing from the monitor, ROS or viser.

## Install

```bash
pip install -e .              # pure core
pip install -e ".[mirrors]"   # plus PyBullet and compas_fab mirrors
```

In Rhino the plugin adds this folder (its submodule `external/scaffolding_assembly_core`) to `sys.path` instead.

## Test

```bash
pip install -e ".[test]"
python -m pytest            # quick set
python -m pytest -m ""      # also slow: Python 3.9 environments built with uv, legacy conversions
```

`tests/test_legacy_conversion.py` needs `HUSKY_DESIGN_STUDY` (the folder with the 260814 and 260920 exports) and
`HUSKY_DATA` (the monitor's `data/`, holding `husky_urdf/`). Tests whose extras are missing are skipped
(`tests/conftest.py`).

## Source

Copied from `yijiangh/husky-assembly-teleop`, branch `jg/viser-cleanup`, commit
`7bea80e404e0967022bb0462c1c7419b43440018` (2026-10-09): the package `scaffolding_assembly_core/`, the core tests from
`test/` that do not need the monitor, and `doc/process_format.md`. Changes for standalone use:

- `pyproject.toml` replaces `scaffolding_assembly_core/requirements.txt`;
- tests: `tests/conftest.py` skips modules whose extras are missing; the legacy conversion test reads its robot files
  from `HUSKY_DATA`; the 3.9 mirror test installs compas_fab from git and is marked slow.

CI (`.github/workflows/tests.yml`) runs the quick set on Python 3.9 with Rhino's pins and on 3.10.
