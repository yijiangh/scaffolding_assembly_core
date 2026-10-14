"""
The plan checks B1–B15 of format §11: does the plan make sense? Run on demand: before export, planning or execution.

A file can pass the file checks (`validate`) and still fail here, e.g. a half-finished plan saved from Rhino. Errors
block execution; warnings are reported. Only B13 (SRDF groups), B14 and B15 (forward kinematics) read the robot files.

`unreached_targets` is B15 alone, as data: the targets whose joints no longer reach their link poses, e.g. a bar moved
after its inverse kinematics was solved. A process source asks it which of its solutions are stale.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, Iterator, List, Optional, Set, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from ..geometry import Pose, compose, invert
from ..ids import split_link_id
from ..kinematics import chain_joints, link_pose, link_poses
from ..urdf import srdf_group_tips
from .relations import ENGAGED, bar_of, mate_status, mount_offset
from .types import BAR_PREFIX, GROUND_PREFIX, HALF_PREFIX, Action, Process, Holder, Movement, RobotState, State

#: How far a grasp may be from the geometry (B14), metres and radians.
GRASP_TOLERANCE: Tuple[float, float] = (1e-4, 1e-3)
#: How far a movement's end joints may be from the next start (B11), radians or metres.
JOINT_TOLERANCE = 1e-6


@dataclass(frozen=True)
class UnreachedTarget:
    """One link of a movement's target that the target's own joints do not bring there (B15).

    Attributes:
        action, movement: Their ids.
        bar: The action's bar, e.g. "bars/B25".
        link: The link id, e.g. "robots/cindy/left_ur_arm_tool0".
        distance: From the target pose to where the joints put the link, metres.
        angle: The same, radians.
    """

    action: str
    movement: str
    bar: str
    link: str
    distance: float
    angle: float


@dataclass(frozen=True)
class PlanReport:
    """The result of `check_plan`: one line per problem, each starting with its check, e.g. "B7: ...".

    `unreached` holds B15's errors as data too, for callers that mark the bars instead of printing the lines.
    """

    errors: Tuple[str, ...]
    warnings: Tuple[str, ...]
    unreached: Tuple[UnreachedTarget, ...] = ()

    @property
    def ok(self) -> bool:
        """bool: No errors (warnings allowed)."""
        return not self.errors


def check_plan(process: Process, *, geometry: bool = True) -> PlanReport:
    """Run the plan checks B1–B15 on a process that passed the file checks.

    Args:
        process: The process.
        geometry: Also run B13, B14 and B15, which read the robot files.

    Returns:
        PlanReport: Errors and warnings, each check's messages in schedule order.
    """
    checker = _PlanChecker(process)
    checker.run(geometry)
    return PlanReport(tuple(checker.errors), tuple(checker.warnings), tuple(checker.unreached))


def unreached_targets(process: Process, tolerance: Tuple[float, float] = GRASP_TOLERANCE) -> Tuple[UnreachedTarget, ...]:
    """B15 alone: every target link that the target's own joints do not reach, in schedule order.

    A target may give both the joints a movement ends at and the world poses its links end at (a solved keyframe:
    the flange targets from the geometry, the joints from inverse kinematics). They agree when the solution is
    valid. They stop agreeing when the geometry changes afterwards: the bar, a connector or a tool was moved, or
    the bar was copied with its solution.

    - A link is compared where the robot's base is known and the target or the start gives every joint between the
      base and the link; other links cannot be judged and are left out.
    - ! It says the solution misses its target, not that it collides: that needs the scene (`mirrors`).

    Args:
        process: The process; its robot files are read.
        tolerance: Largest distance (metres) and angle (radians) that still count as reached.

    Returns:
        tuple[UnreachedTarget, ...]: Empty when every comparable target is reached.
    """
    found = []
    for action, movement in process.movements():
        target = movement.target
        if target is None or not target.link_poses or not target.joints:
            continue
        by_robot: Dict[str, List[str]] = {}
        for link_id in target.link_poses:
            by_robot.setdefault(split_link_id(link_id)[0], []).append(link_id)
        for robot, link_ids in by_robot.items():
            given, state, spec = target.joints.get(robot), movement.start.robots.get(robot), process.robots.get(robot)
            # * Where the base is at the end: the target's, if the movement drives it there.
            base = None if state is None else target.bases.get(robot, None if robot in movement.base else state.base)
            if not given or state is None or base is None or spec is None:
                continue
            joints = {**(state.joints or {}), **given}
            try:
                judged = [(link_id, name) for link_id, name in ((i, split_link_id(i)[1]) for i in link_ids)
                          if all(joint in joints for joint in chain_joints(spec.urdf, name))]
                poses = link_poses(spec.urdf, base, joints, [name for _, name in judged])
            except KeyError:
                continue  # ? a link the URDF lacks: the file checks' business (A), not this one's
            for link_id, name in judged:
                wanted = target.link_poses[link_id]
                distance = float(np.linalg.norm(np.subtract(wanted.position, poses[name].position)))
                angle = _angle(wanted, poses[name])
                if distance > tolerance[0] or angle > tolerance[1]:
                    found.append(UnreachedTarget(action.id, movement.id, action.bar, link_id, distance, angle))
    return tuple(found)


def end_state(movement: Movement) -> State:
    """The state a movement ends in, as far as the process says: the start with the target applied.

    The target's bases, joints, grips, tool `on`, `attached` and `built` replace the start's; the rest stays.
    ! The acting robot's joints are unknown (None) after an arm motion whose target gives none of its joints, and
      its base after a base motion whose target gives none.
    """
    start, target = movement.start, movement.target
    if target is None:
        return start
    robots = dict(start.robots)
    moving = {split_link_id(arm)[0] for arm in movement.arms}
    bases = target.bases
    for robot in moving | set(target.joints) | set(movement.base) | set(bases):
        robot_state = robots.get(robot)
        if robot_state is None:
            continue
        joints = robot_state.joints
        if robot in moving or robot in target.joints:
            given = target.joints.get(robot)
            joints = None if given is None or robot_state.joints is None else {**robot_state.joints, **given}
        base = bases.get(robot) if robot in movement.base or robot in bases else robot_state.base
        robots[robot] = RobotState(base, joints)
    tools = dict(start.tools)
    for tool, grip in target.tools.items():
        if tools.get(tool) is not None:
            tools[tool] = replace(tools[tool], grip=grip)
    for tool, body in target.on.items():
        if tools.get(tool) is not None:
            tools[tool] = replace(tools[tool], on=body)
    attached = start.attached if target.attached is None else target.attached
    built = start.built if target.built is None else target.built
    return replace(start, robots=robots, tools=tools, attached=dict(attached), built=frozenset(built),
                   present=start.present | set(attached) | built)


class _PlanChecker:
    """One run of the plan checks: the process and the messages found so far."""

    def __init__(self, process: Process):
        self.process = process
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.unreached: List[UnreachedTarget] = []
        # Robot id -> {flange link name: tool id}.
        self.tool_at = {robot_id: dict(robot.tools) for robot_id, robot in process.robots.items()}
        self.steps: List[Tuple[Action, Movement]] = list(process.movements())

    def error(self, check: int, where: str, text: str) -> None:
        """Note one error."""
        self.errors.append(f"B{check}: {where}: {text}")

    def warn(self, check: int, where: str, text: str) -> None:
        """Note one warning."""
        self.warnings.append(f"B{check}: {where}: {text}")

    def holder_tool(self, holder: Holder) -> Optional[str]:
        """The tool mounted at a holder's link, if any."""
        robot, link = split_link_id(holder.to)
        return self.tool_at.get(robot, {}).get(link)

    def states(self) -> Iterator[Tuple[str, State]]:
        """Every start state, then the end of the last movement, with where they are for messages."""
        for _, movement in self.steps:
            yield f"{movement.id} start", movement.start
        if self.steps:
            yield f"{self.steps[-1][1].id} end", end_state(self.steps[-1][1])

    def run(self, geometry: bool) -> None:
        """Run every check."""
        engaged: Set[Tuple[str, str]] = set()
        for where, state in self.states():
            self.check_structure(state, where, engaged)
        self.check_unmated(engaged)
        for index, (action, movement) in enumerate(self.steps):
            at = f"{movement.id}"
            self.check_ending(movement, at)
            self.check_carrying(action, movement, at)
            self.check_changes(movement, at)
            if index + 1 < len(self.steps):
                self.check_chain(action, movement, *self.steps[index + 1])
        self.check_holds()
        if geometry:
            self.check_arms()
            self.check_grasps()
            self.check_targets()

    # --- --- --- --- --- STRUCTURE (B1–B5) --- --- --- --- ---

    def check_structure(self, state: State, where: str, engaged: Set[Tuple[str, str]]) -> None:
        """B1–B4 in one state; collects the mates engaged in it for B5."""
        process = self.process
        for bar in sorted(state.present):
            if bar.startswith(BAR_PREFIX) and bar not in state.built and bar not in state.attached \
                    and bar not in state.poses:
                self.warn(1, where, f"{bar} is neither built nor attached and has no pose: it floats at its process "
                                    f"pose")
        # * B2: every built bar has an engaged mate, and the built bars reach the ground through engaged mates.
        links: Dict[str, Set[str]] = {}
        for mate in process.mates:
            if mate_status(process, state, mate) != ENGAGED:
                continue
            engaged.add(mate)
            a, b = (bar_of(process, side) or side for side in mate)
            links.setdefault(a, set()).add(b)
            links.setdefault(b, set()).add(a)
        reached = {node for node in links if node.startswith(GROUND_PREFIX)}
        frontier = list(reached)
        while frontier:
            for other in links.get(frontier.pop(), ()):
                if other not in reached:
                    reached.add(other)
                    frontier.append(other)
        for bar in sorted(state.built):
            if bar not in links:
                self.error(2, where, f"{bar} is built but none of its mates is engaged")
            elif bar not in reached:
                self.error(2, where, f"{bar} is built but does not reach the ground through engaged mates")
        for bar, holders in sorted(state.attached.items()):
            robots = {split_link_id(holder.to)[0] for holder in holders}
            if bar not in state.built and len(robots) > 1:
                self.error(3, where, f"{bar} is not built but held by several robots {sorted(robots)}")
            part = {bar, *process.halves_of(bar)}
            for robot in sorted(robots):
                on = {tool_state.on for tool, tool_state in state.tools.items()
                      if tool_state is not None and tool in self.tool_at.get(robot, {}).values()}
                if not on & part:
                    self.error(4, where, f"{robot} holds {bar} but none of its tools is on {bar} or its halves")

    def check_unmated(self, engaged: Set[Tuple[str, str]]) -> None:
        """B5: mates never engaged in the schedule, and halves in no mate."""
        for mate in sorted(self.process.mates - engaged):
            self.warn(5, "mates", f"{list(mate)} is never engaged in the schedule")
        mated = {side for mate in self.process.mates for side in mate}
        for half in sorted(body for body in self.process.bodies if body.startswith(HALF_PREFIX) and body not in mated):
            self.warn(5, "mates", f"{half} (on {self.process.bodies[half].mount}) is in no mate: its partner is "
                                  f"not in the process")

    # --- --- --- --- --- PROCEDURE (B6–B12) --- --- --- --- ---

    def check_ending(self, movement: Movement, at: str) -> None:
        """B6: `ends_on: tools` comes with a tool part, and with arm motion needs a compliant controller."""
        if movement.ends_on != "tools":
            return
        if not movement.has_tool_part:
            self.error(6, at, "ends_on tools needs a tool part (a grip change or a drive)")
        if movement.arms and movement.controller != "compliant":
            self.error(6, at, "ends_on tools with arm motion needs controller compliant: a position-controlled arm "
                              "never stops short of its target")

    def check_carrying(self, action: Action, movement: Movement, at: str) -> None:
        """B7–B9: what an arm motion may do given what its robot holds and where its tools are."""
        if not movement.arms:
            return  # ? Manual steps and tool moves are exempt: the operator mounts the bar before the grasp.
        state, robot = movement.start, action.robot
        moving = set(movement.arms)
        for bar, holders in sorted(state.attached.items()):
            own = [holder for holder in holders if split_link_id(holder.to)[0] == robot]
            if not own:
                continue
            if bar in state.built:
                # * B8: a robot holding a built bar is frozen, except in a compliant movement releasing it.
                releases = any(movement.grip_change.get(self.holder_tool(holder)) == "open" for holder in own) or \
                    any(direction == "loosen" and self.on_part(state, tool, bar)
                        for tool, direction in movement.drives.items())
                if movement.controller != "compliant" or not releases:
                    self.error(8, at, f"{robot} holds the built {bar}: it may move only in a compliant movement "
                                      f"that releases it")
                continue
            if not moving & {holder.to for holder in own}:
                continue
            # * B7: carrying an unbuilt bar needs closed grips, and several holders move coupled.
            for holder in own:
                tool = self.holder_tool(holder)
                tool_state = state.tools.get(tool) if tool else None
                if tool_state is not None and tool_state.grip != "closed":
                    self.error(7, at, f"moves {bar} while {tool} grip is {tool_state.grip!r}")
            if len(own) > 1 and not ({holder.to for holder in own} <= moving and movement.coupled):
                self.error(7, at, f"{bar} has {len(own)} holders: they must all move, coupled")
        # * B9: a tool on a body its arm does not hold allows only a linear retreat.
        for arm in movement.arms:
            robot_id, link = split_link_id(arm)
            tool = self.tool_at.get(robot_id, {}).get(link)
            tool_state = state.tools.get(tool) if tool else None
            if tool_state is None or tool_state.on is None:
                continue
            held = bar_of(self.process, tool_state.on)
            holds = held is not None and any(holder.to == arm for holder in state.attached.get(held, ()))
            if not holds and movement.path != "linear":
                self.error(9, at, f"{tool} is on {tool_state.on}, which {arm} does not hold: only a linear retreat "
                                  f"may move it")

    def on_part(self, state: State, tool: str, bar: str) -> bool:
        """Whether a tool is on a bar or one of its halves."""
        tool_state = state.tools.get(tool)
        return tool_state is not None and tool_state.on is not None and bar_of(self.process, tool_state.on) == bar

    def check_changes(self, movement: Movement, at: str) -> None:
        """B10: `attached` and `built` change only as the operations allow."""
        start, end = movement.start, end_state(movement)

        def held(state: State) -> Set[Tuple[str, str]]:
            return {(bar, holder.to) for bar, holders in state.attached.items() for holder in holders}

        for bar, link in sorted(held(end) - held(start)):
            tool = self.holder_tool(Holder(link, Pose()))
            if movement.ends_on != "operator" and movement.grip_change.get(tool) != "closed":
                self.error(10, at, f"{bar} becomes attached to {link} without a manual mount or a grip closing there")
        for bar, link in sorted(held(start) - held(end)):
            tool = self.holder_tool(Holder(link, Pose()))
            if movement.grip_change.get(tool) != "open":
                self.error(10, at, f"{bar} stops being attached to {link} without its grip opening (an ungrasp)")
        for bar in sorted(end.built - start.built):
            # ? A manual step may build a bar: the operator places it by hand (a `bar_placing` action).
            if movement.ends_on != "operator" and not any(
                    direction == "tighten" and self.on_part(start, tool, bar)
                    for tool, direction in movement.drives.items()):
                self.error(10, at, f"{bar} becomes built without an insert (a tighten drive on its part)")
        for bar in sorted(start.built - end.built):
            if not any(direction == "loosen" and self.on_part(start, tool, bar)
                       for tool, direction in movement.drives.items()):
                self.error(10, at, f"{bar} stops being built without an untighten (a loosen drive on its part)")

    def check_chain(self, action: Action, movement: Movement, next_action: Action, following: Movement) -> None:
        """B11: a movement ends where the next one starts, across actions in schedule order.

        Joints, grips, tool `on`, `present`, `attached` and `built` are compared; bases only within one action,
        because robots drive between actions (a movement that lists the robot in `base` ends at its target's base). Values the process leaves open (None) are not compared.
        """
        if movement.target is None:
            return  # ? By definition: it ends where the next one starts.
        end, start = end_state(movement), following.start
        at = f"{movement.id} -> {following.id}"
        for robot, robot_state in end.robots.items():
            other = start.robots.get(robot)
            if robot_state is None or other is None:
                continue
            if robot_state.joints is not None and other.joints is not None:
                shared = set(robot_state.joints) & set(other.joints)
                gap = max((abs(robot_state.joints[name] - other.joints[name]) for name in shared), default=0.0)
                if gap > JOINT_TOLERANCE:
                    self.error(11, at, f"{robot} joints jump by {gap:.3g}")
            if action is next_action and robot_state.base is not None and other.base is not None \
                    and _distance(robot_state.base, other.base) > JOINT_TOLERANCE:
                self.error(11, at, f"{robot} base moves between two movements of one action")
        for tool, tool_state in end.tools.items():
            other = start.tools.get(tool)
            if tool_state is not None and other is not None and None not in (tool_state.grip, other.grip) \
                    and tool_state.grip != other.grip:
                self.error(11, at, f"{tool} grip changes from {tool_state.grip} to {other.grip} between movements")
            if tool_state is not None and other is not None and tool_state.on != other.on:
                self.error(11, at, f"{tool} on changes from {tool_state.on} to {other.on} between movements: a "
                                   f"target sets it")
        bars = {body for body in end.present | start.present if body.startswith(BAR_PREFIX)}
        if {bar for bar in bars if bar in end.present} != {bar for bar in bars if bar in start.present}:
            self.error(11, at, f"present bars change between movements: "
                               f"{sorted((end.present ^ start.present) & bars)}")
        if end.built != start.built:
            self.error(11, at, f"built changes between movements: {sorted(end.built ^ start.built)}")
        before = {(bar, holder.to): holder.grasp for bar, holders in end.attached.items() for holder in holders}
        after = {(bar, holder.to): holder.grasp for bar, holders in start.attached.items() for holder in holders}
        if set(before) != set(after):
            self.error(11, at, f"attached changes between movements: {sorted(set(before) ^ set(after))}")
        else:
            for key in sorted(before):
                if _distance(before[key], after[key]) > GRASP_TOLERANCE[0]:
                    self.error(11, at, f"the grasp of {key[0]} on {key[1]} changes between movements")

    def check_holds(self) -> None:
        """B12: a hold is released only after every bar in its `supports_until` is built."""
        holds: Dict[Tuple[str, str], Action] = {}
        for action_id in self.process.schedule:
            action = self.process.actions[action_id]
            if action.type == "bar_holding":
                holds[(action.robot, action.bar)] = action
            elif action.type == "bar_holding_release" and action.movements:
                hold = holds.get((action.robot, action.bar))
                if hold is None:
                    self.warn(12, action.id, f"no earlier bar_holding of {action.bar} by {action.robot}")
                    continue
                missing = sorted(set(hold.supports_until) - action.movements[0].start.built)
                if missing:
                    self.error(12, action.id, f"releases {action.bar} before {missing} are built "
                                              f"({hold.id} supports until them)")

    # --- --- --- --- --- GEOMETRY (B13, B14, B15) --- --- --- --- ---

    def check_arms(self) -> None:
        """B13: `arms` name flange links that end an SRDF group of the acting robot."""
        tips = {robot_id: set(srdf_group_tips(robot.srdf).values()) for robot_id, robot in self.process.robots.items()}
        for action, movement in self.steps:
            for arm in movement.arms:
                robot, link = split_link_id(arm)
                if robot != action.robot or link not in tips.get(robot, ()):
                    self.error(13, movement.id, f"{arm!r} does not end an SRDF group of {action.robot}")

    def check_grasps(self) -> None:
        """B14: grasps agree with the geometry: the half's `pose_in_tcp` for a tool on it, forward kinematics otherwise."""
        process = self.process
        seen: Set[tuple] = set()
        for _, movement in self.steps:
            state, where = movement.start, f"{movement.id} start"
            for bar, holders in sorted(state.attached.items()):
                reference: Optional[Pose] = process.bodies[bar].assembly_pose if bar in state.built else None
                for holder in holders:
                    tool = self.holder_tool(holder)
                    tool_state = state.tools.get(tool) if tool else None
                    on = tool_state.on if tool_state is not None else None
                    key = (bar, holder.to, holder.grasp, on)
                    if on is not None and on.startswith(HALF_PREFIX) and bar_of(process, on) == bar and key not in seen:
                        seen.add(key)
                        in_tcp = process.bodies[on].pose_in_tcp
                        if in_tcp is None:
                            self.error(14, where, f"{tool} is on {on}, which has no pose_in_tcp: no tool goes on it")
                        else:
                            tcp = process.tools[tool].tcp
                            expected = compose(compose(tcp, in_tcp), invert(mount_offset(process, on)))
                            what = f"the grasp of {bar} on {holder.to} (tool on {on})"
                            self._compare(expected, holder.grasp, where, what)
                    robot, link = split_link_id(holder.to)
                    robot_state = state.robots.get(robot)
                    if robot_state is None or robot_state.base is None or robot_state.joints is None:
                        continue
                    world = compose(link_pose(process.robots[robot].urdf, robot_state.base, robot_state.joints, link),
                                    holder.grasp)
                    if reference is None:
                        reference = world
                    else:
                        self._compare(reference, world, where, f"{bar} as held by {holder.to}")

    def check_targets(self) -> None:
        """B15: a target's joints bring its links to their target poses."""
        self.unreached = list(unreached_targets(self.process))
        for miss in self.unreached:
            self.error(15, miss.movement, f"the target's joints leave {miss.link} {miss.distance * 1000:.3g} mm and "
                       f"{np.degrees(miss.angle):.3g}\u00b0 from its target pose: the solution is stale (the geometry "
                       "changed after it was solved)")

    def _compare(self, expected: Pose, found: Pose, where: str, what: str) -> None:
        """Note a B14 error if two poses differ by more than GRASP_TOLERANCE."""
        distance = float(np.linalg.norm(np.subtract(expected.position, found.position)))
        angle = _angle(expected, found)
        if distance > GRASP_TOLERANCE[0] or angle > GRASP_TOLERANCE[1]:
            self.error(14, where, f"{what} is {distance * 1000:.3g} mm and {np.degrees(angle):.3g}° from the geometry")


def _angle(a: Pose, b: Pose) -> float:
    """The rotation angle between two poses' orientations, radians."""
    return float((Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude())


def _distance(a: Pose, b: Pose) -> float:
    """The larger of two poses' distance (metres) and rotation angle (radians)."""
    return max(float(np.linalg.norm(np.subtract(a.position, b.position))), _angle(a, b))
