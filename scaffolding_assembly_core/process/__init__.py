"""
The process file format, schema 2.1 (doc/process_format.md): the `Process` in memory, and reading, writing and validating.

Not imported here: `process.scenes` (scenes from a process, `Process.scene_at`), `process.plan_check` (the plan checks
B1–B15) and `process.solutions` (planner results), so `import process` stays free of scenes and robots.

! Runs on Python 3.9 (Rhino 8) with numpy, scipy, trimesh and yourdfpy only.
"""

from __future__ import annotations

from .carry import assumed_joints, assumed_start_all
from .read import read
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, ENDS_ON, PATHS, Action, ActionType, BodySpec, Process,
                    ProcessError, Holder, LineSpec, Movement, Producer, RobotSpec, RobotState, RobotTarget, SchemaMismatch,
                    State, Target, ToolState, Writer)
from .validate import validate
from .version import LIBRARY, SCHEMA, writer_info
from .vocabulary import GRIP, TOOL_KINDS
from .write import content_hash, process_hashes, write

__all__ = [
    "ACTION_TYPES", "BODY_PREFIXES", "CONTROLLERS", "ENDS_ON", "GRIP", "LIBRARY", "PATHS", "SCHEMA", "TOOL_KINDS",
    "Action", "ActionType", "BodySpec", "Process", "ProcessError", "Holder", "LineSpec", "Movement",
    "Producer", "RobotSpec", "RobotState", "RobotTarget", "SchemaMismatch", "State", "Target", "ToolState", "Writer", "assumed_joints",
    "assumed_start_all", "content_hash", "process_hashes", "read", "validate", "write", "writer_info",
]
