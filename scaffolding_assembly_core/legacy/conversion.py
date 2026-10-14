"""
Convert an export in the old compas_fab format into a schema 2 process, written next to it in `<export>_process`.

The robots are the calibrated URDFs and SRDFs from the data directory, copied into the process with their meshes.
! Slow (~15 s, the cells are ~350 MB each): call it off the main thread.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Callable, Optional

from ..process.types import Process
from ..process.version import SCHEMA
from ..process.write import write
from .export import CONVERTER, from_export, load_export
from .timing import Stopwatch

#: A folder holding this is an export in the old compas_fab format.
OLD_EXPORT_FILE = "ActionSchedule.json"
#: A folder holding this is a process (its schema is in `writer.schema`).
PROCESS_FILE = "process.json"
#: Added to an export's folder name for its converted copy.
CONVERTED_SUFFIX = "_process"

#: Robot name in the schedule -> (URDF, SRDF) under the data directory, as the Rhino workflow's
#: `core/config.py` uses them. ! The calibrated files, not the monitor's `_StockUrFrames` variants.
ROBOT_FILES = {
    "Cindy": ("husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf",
              "husky_urdf/mt_husky_dual_ur5_e_moveit_config/config/dual_arm_husky.srdf"),
    "Alice": ("husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf",
              "husky_urdf/mt_husky_moveit_config/config/husky.srdf"),
    "Belle": ("husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Belle_Calibrated.urdf",
              "husky_urdf/mt_husky_moveit_config/config/belle.srdf"),
}

#: Robot name -> hardware serial (config._ROBOTS_BY_SERIAL).
SERIALS = {"Cindy": "0806", "Alice": "0804", "Belle": "0805"}


def is_old_export(folder: Path) -> bool:
    """Whether a folder is an old compas_fab export and not a schema 1 process."""
    folder = Path(folder)
    return (folder / OLD_EXPORT_FILE).is_file() and not (folder / PROCESS_FILE).is_file()


def converted_folder(export: Path) -> Path:
    """The sibling folder `<export>_process` an export is converted into."""
    export = Path(export).expanduser().resolve()
    return export.with_name(export.name + CONVERTED_SUFFIX)


def is_up_to_date(export: Path) -> bool:
    """Whether the export's converted copy exists, was made by this converter version, and is newer than the export.

    Args:
        export: The export folder.
    """
    process_file = converted_folder(export) / PROCESS_FILE
    if not process_file.is_file():
        return False
    try:
        manifest = json.loads(process_file.read_text(encoding="utf-8"))
        schema, command = manifest["writer"]["schema"], manifest["producer"]["command"]
    except (ValueError, KeyError, TypeError):
        return False
    if schema != SCHEMA or command != CONVERTER:
        return False
    written = process_file.stat().st_mtime
    return all(path.stat().st_mtime <= written for path in Path(export).rglob("*.json"))


def robot_files(data_directory: Path) -> dict:
    """ROBOT_FILES as absolute paths under a data directory.

    Args:
        data_directory: The monitor's data directory, holding `husky_urdf/`.

    Returns:
        dict[str, tuple[Path, Path]]: Robot name -> (URDF, SRDF).
    """
    return {name: (data_directory / urdf, data_directory / srdf) for name, (urdf, srdf) in ROBOT_FILES.items()}


def convert_in_memory(export: Path, data_directory: Path, report: Callable[[str], None] = print,
                      watch: Optional[Stopwatch] = None) -> Process:
    """Load an export and convert it into an in-memory process (`folder` None), writing nothing.

    Args:
        export: The export folder, holding ActionSchedule.json.
        data_directory: The monitor's data directory, holding `husky_urdf/`.
        report: Told what is being done.
        watch: Gets a lap per step, if given.
    """
    watch = watch if watch is not None else Stopwatch()
    loaded = load_export(export, report, watch)
    process = from_export(loaded, robot_files(data_directory), serials=SERIALS, report=report)
    watch.lap("convert")
    return process


def convert_export(export: Path, destination: Path, data_directory: Path,
                   report: Callable[[str], None] = print, watch: Optional[Stopwatch] = None) -> Path:
    """Convert an export into a schema 2 process folder, replacing what is there.

    Args:
        export: The export folder, holding ActionSchedule.json.
        destination: The process folder to write.
        data_directory: The monitor's data directory, holding `husky_urdf/`.
        report: Told what is being done.
        watch: Gets a lap per step, if given.

    Returns:
        Path: The process folder.
    """
    watch = watch if watch is not None else Stopwatch()
    process = convert_in_memory(export, data_directory, report, watch)
    report(f"writing {destination}")
    write(process, destination, overwrite=True, package_dirs=[data_directory / "husky_urdf"])
    watch.lap("write (copy robots, read back, validate)")
    return Path(destination)
