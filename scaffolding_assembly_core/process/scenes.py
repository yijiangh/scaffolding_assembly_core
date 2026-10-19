"""
Scenes from a process: the world at one movement (`scene_at`), or once a bar is built (`scene_after`).

`seeded` makes a scene plannable for a robot whose base or joints the process leaves open.

A scene shares nothing mutable with the process: bodies, robots, joints and touches are new; only `Geometry` and
`RobotModel` objects are shared, and a robot's model is the same object for every scene of the same robot.
"""

from __future__ import annotations

from functools import lru_cache
from typing import Dict, Mapping, Optional, Set, Tuple

from ..geometry import Pose
from ..ids import split_link_id
from ..kinematics import link_pose
from ..robot import RobotModel, RobotObject, Tool, robot_model
from ..scene import Attachment, Body, Scene, world_poses
from .relations import allowed_contacts, is_present, placement
from .types import Process, Holder, Movement, State


def process_model(process: Process, robot_id: str) -> RobotModel:
    """The model of a process robot: its URDF and SRDF, its tools, and the links each tool may touch.

    ! The same object for an unchanged robot, also across `dataclasses.replace(process, …)`: mirrors compare models
      by identity.

    Args:
        process: The process.
        robot_id: E.g. "robots/cindy".

    Returns:
        RobotModel: Cached by the robot's files and tool objects.
    """
    spec = process.robots[robot_id]
    tools = tuple(sorted((flange, process.tools[tool_id]) for flange, tool_id in spec.tools.items()))
    return _model(robot_id, spec.urdf, spec.srdf, tools)


@lru_cache(maxsize=64)
def _model(robot_id: str, urdf, srdf, tools: Tuple[Tuple[str, Tool], ...]) -> RobotModel:
    """`process_model`, cached: a tool's `Geometry` hashes by identity, so a new process object is a new model."""
    return robot_model(robot_id.split("/", 1)[1], urdf, srdf, dict(tools))


def scene_at(process: Process, movement: Movement, at_end: bool = False) -> Scene:
    """The world at the start of a movement, as the process gives it; with `at_end`, its target joints applied.

    - A robot absent from the state is disabled; one with `joints: null` has every movable joint `unmeasured`
      (at 0), one with `base: null` is not `base_tracked`, so planners refuse it until its owner fills them in.
    - Bodies follow the pose rule (`relations.placement`): an attached, unbuilt bar and its halves are attached to
      the first holder's link; every other body has a world pose. An absent body is disabled.
    - `touches` hold every allowed contact both ways, derived (`relations.allowed_contacts`); mount contacts are on
      the robot model.

    Args:
        process: The process.
        movement: One of its movements.
        at_end: Apply the movement's target joints, for the world as the movement ends. Bodies keep their start
            placement; held ones move with the robot holding them.

    Returns:
        Scene: A new scene, `tick` -1.
    """
    target = movement.target.joints if at_end and movement.target is not None else None
    return _scene(process, movement.start, target)


def scene_after(process: Process, bar: str) -> Scene:
    """The world once a bar is built, following the schedule.

    That is the start of the first movement after the last action of that bar; after the schedule's last action, its
    last movement's start with the target joints applied.

    Args:
        process: The process.
        bar: A bar id, e.g. "bars/B3".

    Returns:
        Scene: A new scene.

    Raises:
        KeyError: If no action places that bar.
    """
    indices = [index for index, action_id in enumerate(process.schedule) if process.actions[action_id].bar == bar]
    if not indices:
        raise KeyError(f"no action in the schedule places {bar!r}")
    following = indices[-1] + 1
    if following < len(process.schedule):
        return _scene(process, process.actions[process.schedule[following]].movements[0].start)
    last = process.actions[process.schedule[indices[-1]]].movements[-1]
    return _scene(process, last.start, last.target.joints if last.target is not None else None)


def _scene(process: Process, state: State, target_joints: Optional[Dict[str, Dict[str, float]]] = None) -> Scene:
    """A new scene of one process state; `target_joints` (robot id -> joints) override the state's."""
    robots: Dict[str, RobotObject] = {}
    for robot_id, spec in process.robots.items():
        model = process_model(process, robot_id)
        robot_state = state.robots.get(robot_id)
        if robot_state is None:
            robots[robot_id] = RobotObject(robot_id, model, Pose(), {}, enabled=False, label=spec.name)
            continue
        joints = dict(robot_state.joints or {})
        joints.update((target_joints or {}).get(robot_id, {}))
        unknown = frozenset() if robot_state.joints is not None else frozenset(model.movable_joints) - set(joints)
        # ? A base the process leaves open counts as not tracked: planners refuse it until its owner fills it in.
        robots[robot_id] = RobotObject(robot_id, model, robot_state.base or Pose(), joints,
                                       base_tracked=robot_state.base is not None, unmeasured=unknown, label=spec.name)

    # * Allowed contacts are derived, never stored: both ways, on every body that takes part.
    contacts: Dict[str, Set[str]] = {}
    for a, b in allowed_contacts(process, state):
        contacts.setdefault(a, set()).add(b)
        contacts.setdefault(b, set()).add(a)

    bodies: Dict[str, Body] = {}
    for body_id, spec in process.bodies.items():
        where = placement(process, state, body_id)
        if isinstance(where, Holder):
            robot_id, link = split_link_id(where.to)
            where = Attachment(robot_id, link, where.grasp)
        bodies[body_id] = Body(body_id, spec.geometry, where, tuple(sorted(contacts.get(body_id, ()))),
                               spec.label, enabled=is_present(process, state, body_id))

    scene = Scene(bodies=bodies, world_poses={}, robots=robots)
    scene.world_poses = _world_poses(scene)
    return scene


def _world_poses(scene: Scene) -> Dict[str, Pose]:
    """Every body's world pose in a scene, attached ones by forward kinematics of their robot."""
    return world_poses(scene.bodies, scene.robots,
                       lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))


def seeded(scene: Scene, robot_id: str, joints: Optional[Mapping[str, float]] = None,
           base: Optional[Pose] = None) -> None:
    """Fill in a robot's unknown joints and base in a scene, in place, so a planner can start from them.

    `null` in a process means "not decided by the process": a planner may choose the values, and writes what it chose
    into its solution (`start`, `bases`). Other robots with unknown state are parked by the compas_fab mirror.

    Args:
        scene: A scene from `scene_at`; changed in place.
        robot_id: The robot to fill in; it is enabled.
        joints: Values for its unknown joints; the rest are 0. Known joints keep their value.
        base: Its base, if the process leaves it open; else the origin (a planner sampling bases replaces it).
    """
    robot = scene.robots[robot_id]
    robot.enabled = True
    if robot.unmeasured:
        given = dict(joints or {})
        robot.joints = {**robot.joints, **{name: given.get(name, 0.0) for name in robot.unmeasured}}
        robot.unmeasured = frozenset()
    if not robot.base_tracked:
        robot.base, robot.base_tracked = base or Pose(), True
    scene.world_poses = _world_poses(scene)
