"""
What follows from a state and the process without being stored (format §8): where a body is, mate status, contacts.

- Where a bar is: built -> its assembly pose; attached and not built -> its first holder's link times the grasp;
  otherwise the pose in `State.poses`, else its assembly pose. A mounted half follows its bar.
- Allowed contacts: a tool with the whole part it is on, a half with its bar, a pending or engaged mate's halves with
  each other and with each other's bar, a ground-mated half and every robot's ground links with every ground body.
  Mount contacts belong to the robot model (`RobotModel`).
"""

from __future__ import annotations

from typing import Dict, Optional, Set, Tuple, Union

from ..geometry import Pose, compose, invert
from .types import BAR_PREFIX, GROUND_PREFIX, HALF_PREFIX, Process, Holder, State

#: Mate status (format §8.3): which contacts a mate allows at one moment.
OPEN, PENDING, ENGAGED, NOT_RELEVANT = "open", "pending", "engaged", "not relevant"

#: A mate: two body ids, sorted.
Mate = Tuple[str, str]


def bar_of(process: Process, body: str) -> Optional[str]:
    """The bar a body belongs to: a bar itself, a half its mount; None for ground, obstacles and unmounted halves."""
    if body.startswith(BAR_PREFIX):
        return body
    spec = process.bodies.get(body)
    return spec.mount if spec is not None else None


def is_present(process: Process, state: State, body: str) -> bool:
    """Whether a body exists in a state: when `present` lists it, or, for a mounted body (a half), when it lists the
    bar it is mounted on. ? No override: a present bar has all its halves."""
    spec = process.bodies.get(body)
    if spec is not None and spec.mount is not None:
        return spec.mount in state.present
    return body in state.present


def mount_offset(process: Process, half: str) -> Pose:
    """A half's pose in its bar's frame, from the two assembly poses."""
    spec = process.bodies[half]
    return compose(invert(process.bodies[spec.mount].assembly_pose), spec.assembly_pose)


def placement(process: Process, state: State, body: str) -> Union[Pose, Holder]:
    """Where a body is by the pose rule: a world pose, or the holding link and the body's pose in that link's frame.

    Args:
        process: The process.
        state: The state; only meaningful for a present body.
        body: A body id.

    Returns:
        Pose | Holder: A Holder for an attached, unbuilt bar (its first holder) and for the halves mounted on it.
    """
    bar = bar_of(process, body)
    holders = state.attached.get(bar, ()) if bar is not None else ()
    staged = state.poses.get(bar) if bar is not None else None
    if bar is None or bar in state.built or not (holders or staged):
        return process.bodies[body].assembly_pose  # ? its own assembly pose, also for a half: exact, not composed
    if holders:
        holder = holders[0]
        return holder if body == bar else Holder(holder.to, compose(holder.grasp, mount_offset(process, body)))
    return staged if body == bar else compose(staged, mount_offset(process, body))


def mate_status(process: Process, state: State, mate: Mate) -> str:
    """The status of one mate in a state, from the `built` and `attached` flags of its two sides.

    A ground body counts as built. Engaged: both sides built. Pending: one built, the other present, attached and not
    built (being joined or removed). Not relevant: neither side present. Anything else is open: no contact allowed.
    """
    sides = [bar_of(process, side) or side for side in mate]

    def built(side: str) -> bool:
        return side.startswith(GROUND_PREFIX) or side in state.built

    present = [side in state.present for side in sides]
    if not any(present):
        return NOT_RELEVANT
    if all(built(side) and side in state.present for side in sides):
        return ENGAGED
    for one, other in (sides, sides[::-1]):
        if built(one) and one in state.present and other in state.present and other in state.attached \
                and not built(other):
            return PENDING
    return OPEN


def mate_statuses(process: Process, state: State) -> Dict[Mate, str]:
    """The status of every mate of the process in a state."""
    return {mate: mate_status(process, state, mate) for mate in sorted(process.mates)}


def allowed_contacts(process: Process, state: State) -> Set[Tuple[str, str]]:
    """Every allowed contact in a state, each pair sorted (mount contacts excepted: they are the robot model's).

    - A tool on a body may touch that body's whole part: its bar and every half mounted on it.
    - A half may touch its bar; the halves of a pending or engaged mate each other, and each other's bar.
    - A half mated to a ground body, and every robot's ground links, may touch every present ground body. A robot
      the state leaves out still stands on the floor in the real world.

    ? Collision models are coarser than the parts: a tool's convex hull overlaps the bar it holds by about 1 mm.

    Args:
        process: The process.
        state: The state.

    Returns:
        set[tuple[str, str]]: Pairs of ids: bodies, tools, and robot links "robots/<robot>/<link>".
    """
    pairs: Set[Tuple[str, str]] = set()

    def add(a: str, b: str) -> None:
        if a != b:
            pairs.add((a, b) if a < b else (b, a))

    def part(body: str) -> Tuple[str, ...]:
        bar = bar_of(process, body)
        return (bar, *process.halves_of(bar)) if bar is not None else (body,)

    grounds = [body for body in state.present if body.startswith(GROUND_PREFIX)]
    for tool, tool_state in state.tools.items():
        if tool_state is not None and tool_state.on is not None and is_present(process, state, tool_state.on):
            for body in part(tool_state.on):
                add(tool, body)
    for body_id, body in process.bodies.items():
        if body.mount is not None and is_present(process, state, body_id):
            add(body_id, body.mount)
    for mate in process.mates:
        if mate_status(process, state, mate) in (PENDING, ENGAGED):
            add(*mate)
            for side, other in (mate, mate[::-1]):
                if side.startswith(HALF_PREFIX) and other.startswith(HALF_PREFIX):
                    add(side, process.bodies[other].mount)
        for side, other in (mate, mate[::-1]):
            if side.startswith(HALF_PREFIX) and other.startswith(GROUND_PREFIX) and is_present(process, state, side):
                for ground in grounds:
                    add(side, ground)
    for robot_id, robot in process.robots.items():
        for link in robot.ground_links:
            for ground in grounds:
                add(f"{robot_id}/{link}", ground)
    return pairs
