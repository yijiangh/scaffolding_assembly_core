"""Tests for the joint space (urdf.joint_limits), shoulder lifts moved into their window, and plan check B16."""

from dataclasses import replace

import numpy as np
import pytest
from process_fixtures import build_process

from scaffolding_assembly_core.process import Process
from scaffolding_assembly_core.process.joint_range import lifts_in_range
from scaffolding_assembly_core.process.plan_check import check_plan
from scaffolding_assembly_core.urdf import SHOULDER_LIFT_RANGE, joint_limits, shoulder_lift_in_range

LIFT = "left_ur_arm_shoulder_lift_joint"

#: One UR arm's lift, elbow and wrist 1 (continuous), then a joint of no UR arm.
UR_URDF = """<?xml version="1.0"?>
<robot name="ur">
  <link name="l0"/><link name="l1"/><link name="l2"/><link name="l3"/><link name="l4"/>
  <joint name="arm_shoulder_lift_joint" type="revolute"><parent link="l0"/><child link="l1"/><axis xyz="0 1 0"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="1" velocity="1"/></joint>
  <joint name="arm_elbow_joint" type="revolute"><parent link="l1"/><child link="l2"/><axis xyz="0 1 0"/>
    <limit lower="-6.2831853" upper="6.2831853" effort="1" velocity="1"/></joint>
  <joint name="arm_wrist_1_joint" type="continuous"><parent link="l2"/><child link="l3"/><axis xyz="0 1 0"/></joint>
  <joint name="turn" type="revolute"><parent link="l3"/><child link="l4"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
</robot>
"""


def test_joint_limits_keep_to_the_ur_controller_and_the_lift_window(tmp_path):
    """URDF limits, within the UR controller's (elbow ±180°, a continuous wrist ±360°) and the lift's -270° to +90°."""
    urdf = tmp_path / "ur.urdf"
    urdf.write_text(UR_URDF)
    lower, upper = joint_limits(urdf, ["arm_shoulder_lift_joint", "arm_elbow_joint", "arm_wrist_1_joint", "turn"])
    assert np.degrees(lower) == pytest.approx([-270.0, -180.0, -360.0, np.degrees(-3.0)])
    assert np.degrees(upper) == pytest.approx([90.0, 180.0, 360.0, np.degrees(3.0)])


@pytest.mark.parametrize("before, after", [(328.3, -31.7), (252.6, -107.4), (-300.0, 60.0), (630.0, -90.0),
                                           (90.0, 90.0), (-270.0, -270.0), (-31.7, -31.7)])
def test_a_lift_moves_by_whole_turns_into_the_window(before, after):
    """Values outside move by whole turns; values inside, both edges included, stay; other joints never move."""
    lift, pan, bare = shoulder_lift_in_range([LIFT, "left_ur_arm_shoulder_pan_joint", "shoulder_lift_joint"],
                                             np.radians([before, 400.0, before]))
    assert np.degrees([lift, bare]) == pytest.approx([after, after])
    assert np.degrees(pan) == pytest.approx(400.0)


def test_rounding_past_an_edge_stays():
    """A lift a hair past +90° is the edge, not a whole turn away."""
    edge = SHOULDER_LIFT_RANGE[1] + 1e-12
    assert shoulder_lift_in_range([LIFT], [edge]) == [edge]


def _with_lift(process: Process, robot: str, value: float) -> Process:
    """The process with `robot`'s shoulder lift at `value`, radians, wherever a state or a target gives its joints."""
    def joints(given):
        return {**given, LIFT: value} if given else given

    actions = {}
    for action_id, action in process.actions.items():
        movements = []
        for movement in action.movements:
            robots = {name: part if name != robot or part is None else replace(part, joints=joints(part.joints))
                      for name, part in movement.start.robots.items()}
            target = movement.target
            if target is not None and robot in target.robots:
                part = target.robots[robot]
                target = replace(target, robots={**target.robots, robot: replace(part, joints=joints(part.joints))})
            movements.append(replace(movement, start=replace(movement.start, robots=robots), target=target))
        actions[action_id] = replace(action, movements=tuple(movements))
    return replace(process, actions=actions)


def _b16(process: Process) -> list:
    """The B16 warnings of a process whose plan checks pass."""
    report = check_plan(process)
    assert report.ok, report.errors
    return [warning for warning in report.warnings if warning.startswith("B16:")]


def test_a_process_with_lifts_a_turn_off(tmp_path):
    """B16 warns of each value written outside; `lifts_in_range` moves them all alike, so the plan still holds."""
    process = build_process(tmp_path)
    assert lifts_in_range(process) == (process, ())
    off = _with_lift(process, "robots/cindy", np.radians(328.3))
    warnings = _b16(off)
    assert warnings and all("328.3°" in w and "-31.7°" in w for w in warnings), warnings

    fixed, moved = lifts_in_range(off)
    assert len(moved) == len(warnings)
    assert str(moved[0]).endswith(f"robots/cindy {LIFT} 328.3° -> -31.7°")
    assert not _b16(fixed)
    parts = [part for _, movement in fixed.movements()
             for part in (movement.start.robots["robots/cindy"],
                          *(movement.target.robots.values() if movement.target is not None else ()))]
    values = [part.joints[LIFT] for part in parts if part is not None and part.joints and LIFT in part.joints]
    assert values and np.degrees(values) == pytest.approx([-31.7] * len(values))


def test_a_start_that_copies_no_earlier_end_is_moved_and_reported(tmp_path):
    """A later start holding its own lift (after a target that gives no joints) is found too, not only targets."""
    process = build_process(tmp_path)
    action, index, movement = next((action, index, movement) for action in process.actions.values()
                                   for index, movement in enumerate(action.movements)
                                   if index and (movement.start.robots.get("robots/cindy") or None) is not None
                                   and movement.start.robots["robots/cindy"].joints)
    part = movement.start.robots["robots/cindy"]
    turned = replace(part, joints={**part.joints, LIFT: np.radians(328.3)})
    start = replace(movement.start, robots={**movement.start.robots, "robots/cindy": turned})
    movements = list(action.movements)
    movements[index] = replace(movement, start=start)
    off = replace(process, actions={**process.actions, action.id: replace(action, movements=tuple(movements))})
    fixed, moved = lifts_in_range(off)
    assert [str(lift) for lift in moved] == [f"{movement.id} start: robots/cindy {LIFT} 328.3° -> -31.7°"]
    lift = fixed.actions[action.id].movements[index].start.robots["robots/cindy"].joints[LIFT]
    assert np.degrees(lift) == pytest.approx(-31.7)
