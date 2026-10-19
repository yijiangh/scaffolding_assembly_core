"""Tests for process carry (joints assumed for movements without authored start joints), joints_at and lookups."""

from process_fixtures import WRITER

import pytest

from scaffolding_assembly_core.process import (Action, Process, Movement, RobotState, State, Target, assumed_joints,
                                               joints_at)
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process.carry import assumed_start_all

ROBOTS = ("robots/a", "robots/b")


def _movement(movement_id: str, robot: str, start=None, target=None) -> Movement:
    """A free movement of `robot`: its start joints (or None) and target joints (or None)."""
    robots = {r: RobotState(Pose(), start if r == robot else None) for r in ROBOTS}
    return Movement(id=movement_id, arms=(f"{robot}/tool0",), path="free", controller="position",
                    start=State(robots=robots, present=frozenset(), poses={}),
                    target=Target.of(joints={robot: target}) if target else None)


def _process(*actions) -> Process:
    """A process in memory with only a schedule of actions (carry needs nothing else)."""
    return Process(folder=None, writer=WRITER, robots={}, tools={}, bodies={},
                  schedule=tuple(a.id for a in actions), actions={a.id: a for a in actions})


def _action(action_id: str, robot: str, *movements) -> Action:
    """An action of `robot`."""
    return Action(id=action_id, type="bar_jointing", robot=robot, bar="bars/B1", movements=movements)


def test_authored_and_carried_from_previous_target():
    """An authored start is used as is; the next movement starts where it ended (start + target)."""
    process = _process(_action("A1", "robots/a",
                             _movement("M0", "robots/a", start={"j1": 1.0, "j2": 2.0}, target={"j2": 5.0}),
                             _movement("M1", "robots/a")))
    assert assumed_joints(process, "A1", "M0") == (None, "")
    assert assumed_joints(process, "A1", "M1") == ({"j1": 1.0, "j2": 5.0}, "M0")


def test_own_target_before_anything_known():
    """Before the robot's first known pose, a movement uses its own target."""
    process = _process(_action("A1", "robots/a", _movement("M0", "robots/a", target={"j1": 3.0}),
                             _movement("M1", "robots/a")))
    assert assumed_joints(process, "A1", "M0") == ({"j1": 3.0}, "own target")
    assert assumed_joints(process, "A1", "M1") == ({"j1": 3.0}, "M0")


def test_look_ahead_and_robots_kept_apart():
    """With nothing before it, a movement takes the robot's next known start; other robots don't count."""
    process = _process(_action("A1", "robots/a", _movement("M0", "robots/a")),
                     _action("A2", "robots/b", _movement("M1", "robots/b", start={"k": 9.0})),
                     _action("A3", "robots/a", _movement("M2", "robots/a", start={"j1": 7.0})))
    everything = assumed_start_all(process)
    assert everything[("A1", "M0")] == ({"j1": 7.0}, "M2 (later)")
    assert everything[("A2", "M1")] == (None, "")


def test_nothing_known():
    """A robot with no start or target anywhere has nothing to assume."""
    process = _process(_action("A1", "robots/a", _movement("M0", "robots/a"), _movement("M1", "robots/a")),
                     _action("A2", "robots/b", _movement("M2", "robots/b", start={"k": 1.0})))
    assert assumed_joints(process, "A1", "M0") == (None, "")
    assert assumed_joints(process, "A1", "M1") == (None, "")


def test_joints_at_fills_only_the_acting_robot_with_its_assumed_start():
    """Authored joints as given; the acting robot's null start takes its assumed start; other null robots stay empty."""
    process = _process(_action("A1", "robots/a",
                               _movement("M0", "robots/a", start={"j1": 1.0}, target={"j1": 2.0}),
                               _movement("M1", "robots/a")))
    assert joints_at(process, "A1", "M0") == {"robots/a": {"j1": 1.0}, "robots/b": {}}
    assert joints_at(process, "A1", "M1") == {"robots/a": {"j1": 2.0}, "robots/b": {}}


def test_joints_at_end_applies_the_target_and_takes_a_precomputed_carry():
    """With `at_end` the target's joints override; a precomputed `assumed_start_all` gives the same answer."""
    process = _process(_action("A1", "robots/a",
                               _movement("M0", "robots/a", start={"j1": 1.0, "j2": 0.5}, target={"j1": 2.0})))
    expected = {"robots/a": {"j1": 2.0, "j2": 0.5}, "robots/b": {}}
    assert joints_at(process, "A1", "M0", at_end=True) == expected
    assert joints_at(process, "A1", "M0", at_end=True, assumed=assumed_start_all(process)) == expected
    with pytest.raises(KeyError):
        joints_at(process, "A1", "missing")


def test_joints_at_end_leaves_out_a_robot_absent_from_the_start():
    """A robot the target names but the start state leaves out stays out, as in `scene_at`."""
    movement = _movement("M0", "robots/a", start={"j1": 1.0})
    absent_b = Movement(id="M0", arms=movement.arms, path="free", controller="position",
                        start=State(robots={"robots/a": RobotState(Pose(), {"j1": 1.0})}, present=frozenset(),
                                    poses={}),
                        target=Target.of(joints={"robots/a": {"j1": 2.0}, "robots/b": {"k": 1.0}}))
    process = _process(_action("A1", "robots/a", absent_b))
    assert joints_at(process, "A1", "M0", at_end=True) == {"robots/a": {"j1": 2.0}}


def test_find_movement_and_movement_after_follow_the_schedule():
    """Lookup by ids, and the next movement across actions; None after the last or for unknown ids."""
    first, second, third = (_movement("M0", "robots/a"), _movement("M1", "robots/a"), _movement("M2", "robots/b"))
    process = _process(_action("A1", "robots/a", first, second), _action("A2", "robots/b", third))
    assert process.find_movement("A1", "M1") is second
    assert process.find_movement("A1", "M2") is None and process.find_movement("A9", "M0") is None
    assert process.movement_after(first) is second and process.movement_after(second) is third
    assert process.movement_after(third) is None
