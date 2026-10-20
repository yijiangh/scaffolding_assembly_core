"""
Joints to assume, for display and planner seeding only, when a movement's acting robot has `joints: null`.

* Per robot, in schedule order:
  - an authored start sets its joints, then the movement's target joints (those named) move it;
  - before its first known joints, a movement uses its own target joints;
  - failing that, the robot's next known start.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

from .types import Process, Movement

#: Assumed joints (or None) and where they came from: a movement id, "own target", "<id> (later)" or "".
Assumed = Tuple[Optional[Dict[str, float]], str]


def _authored(movement: Movement, robot: str) -> Dict[str, float]:
    """The authored start joints of a robot in a movement; empty if None or the robot is absent."""
    robot_state = movement.start.robots.get(robot)
    return dict(robot_state.joints) if robot_state is not None and robot_state.joints else {}


def _target(movement: Movement, robot: str) -> Dict[str, float]:
    """The target joints a movement names for a robot; empty if none."""
    return dict(movement.target.joints.get(robot, {})) if movement.target is not None else {}


def assumed_start_all(process: Process) -> dict[tuple[str, str], Assumed]:
    """Assumed start joints of every movement, by the rule in the module docstring.

    Args:
        process: The process.

    Returns:
        dict[tuple[str, str], tuple[dict[str, float] | None, str]]: (action id, movement id) ->
            (joints, source). Movements with authored joints, or with nothing known, get (None, "").
    """
    steps = [(action.id, action.robot, movement) for action, movement in process.movements()]
    result: Dict[Tuple[str, str], Assumed] = {}
    # Robot id -> (last known joint values, id of the movement they came from).
    last: Dict[str, Tuple[Dict[str, float], str]] = {}
    for action_id, robot, movement in steps:
        authored, target = _authored(movement, robot), _target(movement, robot)
        assumed: Assumed = (None, "")
        if authored:
            start = authored
        elif robot in last:
            assumed = last[robot]
            start = assumed[0]
        elif target:
            assumed = (target, "own target")
            start = target
        else:
            start = {}
        result[(action_id, movement.id)] = assumed
        # Where the robot is after this movement.
        if start or target:
            last[robot] = ({**start, **target}, movement.id)

    # * Movements with no pose and no target: take the robot's next known start (walk backwards).
    upcoming: Dict[str, Tuple[Dict[str, float], str]] = {}
    for action_id, robot, movement in reversed(steps):
        key = (action_id, movement.id)
        known = _authored(movement, robot) or result[key][0]
        if known:
            upcoming[robot] = (known, movement.id)
        elif robot in upcoming:
            joints, source = upcoming[robot]
            result[key] = (joints, f"{source} (later)")
    return result


def assumed_joints(process: Process, action_id: str, movement_id: str) -> tuple[dict[str, float] | None, str]:
    """Assumed start joints of one movement. Walks the whole schedule; use `assumed_start_all` for many.

    Args:
        process: The process.
        action_id: The action.
        movement_id: One of its movements.

    Returns:
        tuple[dict[str, float] | None, str]: The joints and their source (a movement id,
            "own target" or "<id> (later)"), or (None, "") if the start is authored or nothing is known.

    Raises:
        KeyError: If the action has no such movement.
    """
    return assumed_start_all(process)[(action_id, movement_id)]


def joints_at(process: Process, action_id: str, movement_id: str, at_end: bool = False,
              assumed: Optional[Dict[Tuple[str, str], Assumed]] = None) -> Dict[str, Dict[str, float]]:
    """The joints to show each robot of a movement at: its start, or with `at_end` its target joints applied.

    Authored joints first; the acting robot's `joints: null` takes its assumed start (this module's rules), or an
    empty dict when nothing is known. A robot absent from the start state is left out, also when the target names it.
    * The assumed view, for display and seeding: `scene_at` instead leaves such joints `unmeasured`, at 0.

    Args:
        process: The process.
        action_id: The action.
        movement_id: One of its movements.
        at_end: Apply the movement's target joints.
        assumed: `assumed_start_all(process)`, keyed (action id, movement id), when stepping through many movements;
            computed here if None.

    Returns:
        dict[str, dict[str, float]]: Robot id -> joint name -> value.

    Raises:
        KeyError: If the action has no such movement, or the action is not in the schedule.
    """
    action = process.actions[action_id]
    movement = process.find_movement(action_id, movement_id)
    if movement is None:
        raise KeyError(f"action {action_id!r} has no movement {movement_id!r}")
    joints: Dict[str, Dict[str, float]] = {}
    for robot_id, robot_state in movement.start.robots.items():
        if robot_state is None:
            continue
        given = robot_state.joints
        if given is None and robot_id == action.robot:
            given = (assumed if assumed is not None else assumed_start_all(process))[(action_id, movement_id)][0]
        joints[robot_id] = dict(given or {})
    if at_end and movement.target is not None:
        for robot_id, values in movement.target.joints.items():
            if robot_id in joints:
                joints[robot_id].update(values)
    return joints
