# bar_assembly_core

The shared core of the bar assembly tools, used by the Rhino plugin (`bar_joint_rhino_design_workflow`), the planners
(`husky_assembly_tamp`) and the monitor (`husky-assembly-teleop`). It holds:

- `design/`: the design format, schema 2 (`doc/design_format.md`): `read`, `write`, `validate` (file checks A1–A14),
  `plan_check.check_plan` (plan checks B1–B14), `relations`, `scenes` (`Design.scene_at`), `solutions`;
- `geometry`, `ids`, `urdf`, `kinematics` (`link_pose`, numpy only), `robot`, `scene`;
- `mirrors/`: a scene in PyBullet or compas_fab, for planners (extra `[mirrors]`);
- `legacy/`: converts the old compas_fab export into schema 2 (extra `[mirrors]` plus `rs_data_structure`).

The layers and their import rules are in `bar_assembly_core/__init__.py`.

- ! Runs on Python 3.9 (Rhino 8's CPython) with numpy, scipy and trimesh only; known good pins: numpy 1.24.4,
  scipy 1.13.1, trimesh 4.12.2.
- ! Imports nothing from the monitor, ROS or viser.

## Install

```bash
pip install -e .              # pure core
pip install -e ".[mirrors]"   # plus PyBullet and compas_fab mirrors
```

In Rhino the plugin adds this folder (its submodule `external/bar_assembly_core`) to `sys.path` instead.

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
`7bea80e404e0967022bb0462c1c7419b43440018` (2026-10-09): the package `bar_assembly_core/`, the core tests from
`test/` that do not need the monitor, and `doc/design_format.md`. Changes for standalone use:

- `pyproject.toml` replaces `bar_assembly_core/requirements.txt`;
- tests: `tests/conftest.py` skips modules whose extras are missing; the legacy conversion test reads its robot files
  from `HUSKY_DATA`; the 3.9 mirror test installs compas_fab from git and is marked slow.

Known gap: `writer.library` is still `"design_io"` (`design/version.py`).
