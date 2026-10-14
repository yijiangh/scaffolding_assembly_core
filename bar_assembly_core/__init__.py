"""
The shared core of bar assembly, used alike by the monitor, the Rhino plugin and the planners.

Layers, each importing only the ones above it:
    geometry, ids     poses, shapes, ids                                          numpy, scipy, trimesh
    urdf, kinematics  URDF/SRDF files, UR conventions, link shapes;            + yourdfpy
                      forward kinematics
    robot, scene      RobotModel, RobotObject; Body, Scene: the world at one moment, and what to draw of it
    process            the file format and its file checks; process.scenes turns a process into scenes,
                      process.plan_check runs the plan checks, process.solutions reads and writes planner results
    mirrors           a scene in PyBullet or compas_fab, for planners; a compas_fab  + pybullet, compas_fab
                      cell to draw (mirrors.cell, no PyBullet)
    legacy            the old compas_fab export (optional)                       + compas, rs_data_structure

- ! Imports nothing from `husky_assembly_teleop`, ROS or viser (`tests/test_core_isolation.py`).
- ! Runs on Python 3.9 (Rhino 8).
"""

from __future__ import annotations
