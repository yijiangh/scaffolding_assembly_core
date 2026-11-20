"""
Shoulder lifts across a process moved into the window every user of the core keeps to (`urdf.SHOULDER_LIFT_RANGE`).

A lift a whole turn outside the window gives the same pose but is out of reach. Exports write lifts inside it; a
reader may move the ones it finds outside (`lifts_in_range`, the monitor on load), and plan check B16 warns of them.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, List, Mapping, Optional, Tuple, Union

import numpy as np

from ..urdf import shoulder_lift_in_range
from .types import Movement, Process, RobotState, RobotTarget, State, Target


@dataclass(frozen=True)
class MovedLift:
    """One shoulder lift written outside the window.

    Attributes:
        where: The movement id, with " start" for a start state.
        robot: The robot id, e.g. "robots/alice".
        joint: The joint's name.
        before: The value written, radians.
        after: The same pose inside the window, radians.
    """

    where: str
    robot: str
    joint: str
    before: float
    after: float

    def __str__(self) -> str:
        return (f"{self.where}: {self.robot} {self.joint} {np.degrees(self.before):.1f}° -> "
                f"{np.degrees(self.after):.1f}°")


def lifts_in_range(process: Process) -> tuple[Process, tuple[MovedLift, ...]]:
    """The process with every shoulder lift in its start states and targets moved into the window; the same poses.

    Equal values move alike, so each start still equals the end before it (B11).

    Args:
        process: The process.

    Returns:
        tuple[Process, tuple[MovedLift, ...]]: The process (the same object when nothing moved), and each value written
            outside the window, in schedule order: every target's, and every start's that is not the value before it
            (a start that copies an earlier end is not repeated).
    """
    moved: List[MovedLift] = []
    #: (robot, joint) -> the value last seen, in a start or a target.
    seen: Dict[Tuple[str, str], float] = {}
    for _action, movement in process.movements():
        _note(movement.start.robots, f"{movement.id} start", True, seen, moved)
        if movement.target is not None:
            _note(movement.target.robots, movement.id, False, seen, moved)
    if not moved:
        return process, ()
    actions = {action_id: replace(action, movements=tuple(_movement(m) for m in action.movements))
               for action_id, action in process.actions.items()}
    return replace(process, actions=actions), tuple(moved)


def _note(parts: Mapping[str, Optional[Union[RobotState, RobotTarget]]], where: str, start: bool,
          seen: Dict[Tuple[str, str], float], moved: List[MovedLift]) -> None:
    """Add each lift outside the window to `moved`, except a start's that repeats the value seen before it."""
    for robot, part in parts.items():
        joints = (part.joints if part is not None else None) or {}
        for name, value in joints.items():
            after = shoulder_lift_in_range([name], [value])[0]
            if after != value and not (start and seen.get((robot, name)) == value):
                moved.append(MovedLift(where, robot, name, value, after))
            seen[(robot, name)] = value


def _in_range(joints: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
    """One robot's joints with their lifts in the window."""
    if not joints:
        return joints
    names = list(joints)
    return dict(zip(names, shoulder_lift_in_range(names, [joints[name] for name in names])))


def _state(state: State) -> State:
    """A state with every robot's lifts in the window."""
    robots = {robot: None if part is None else replace(part, joints=_in_range(part.joints))
              for robot, part in state.robots.items()}
    return replace(state, robots=robots)


def _target(target: Optional[Target]) -> Optional[Target]:
    """A target with every robot's lifts in the window."""
    if target is None:
        return None
    robots = {robot: replace(part, joints=_in_range(part.joints) or {}) for robot, part in target.robots.items()}
    return replace(target, robots=robots)


def _movement(movement: Movement) -> Movement:
    """A movement with the lifts of its start and its target in the window."""
    return replace(movement, start=_state(movement.start), target=_target(movement.target))
