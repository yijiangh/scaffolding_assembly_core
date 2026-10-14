"""
Hold scenes, a planning helper: where a support robot's hold is solved, and where its release is checked.

Both are plain `scene_at` scenes, because every process state is the cell at that moment: the hold is solved at the
start of the movement that closes the gripper (Cindy still holds the bar at its assembled pose), and its release is
checked at the start of the matching `bar_holding_release` (every bar in `supports_until` built).
"""

from __future__ import annotations

from typing import Tuple

from ..scene import Scene
from .types import Process, Movement


def closing_movement(process: Process, action_id: str) -> Movement:
    """The movement of a `bar_holding` action that attaches the held bar to the support robot.

    Raises:
        ValueError: If the action is not a `bar_holding` action, or none of its movements attaches its bar.
    """
    action = process.actions[action_id]
    if action.type != "bar_holding":
        raise ValueError(f"{action_id} is a {action.type} action, not bar_holding")
    for movement in action.movements:
        attached = movement.target.attached if movement.target is not None else None
        if attached is not None and any(holder.to.startswith(f"{action.robot}/")
                                        for holder in attached.get(action.bar, ())):
            return movement
    raise ValueError(f"{action_id}: no movement attaches {action.bar} to {action.robot}")


def release_movement(process: Process, action_id: str) -> Movement:
    """The first movement of the `bar_holding_release` that ends a `bar_holding` action.

    Raises:
        ValueError: If no later release of the same bar by the same robot is in the schedule.
    """
    action = process.actions[action_id]
    later = process.schedule[process.schedule.index(action_id) + 1:]
    for other_id in later:
        other = process.actions[other_id]
        if other.type == "bar_holding_release" and other.robot == action.robot and other.bar == action.bar \
                and other.movements:
            return other.movements[0]
    raise ValueError(f"{action_id}: no later bar_holding_release of {action.bar} by {action.robot}")


def hold_scenes(process: Process, action_id: str) -> Tuple[Scene, Scene]:
    """The two scenes of a hold: solve it in the first, and check each candidate in the second.

    Args:
        process: The process.
        action_id: A `bar_holding` action.

    Returns:
        tuple[Scene, Scene]: `scene_at(closing_movement)` and `scene_at(release_movement)`.

    Raises:
        ValueError: As `closing_movement` and `release_movement`.
    """
    return (process.scene_at(closing_movement(process, action_id)),
            process.scene_at(release_movement(process, action_id)))
