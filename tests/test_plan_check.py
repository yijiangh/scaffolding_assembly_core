"""Tests for the plan checks B1–B15 (design.plan_check): one broken plan per check, errors apart from warnings."""

from dataclasses import replace
from pathlib import Path

import pytest
from design_fixtures import build_design, joints, with_movement, with_start

from bar_assembly_core.design import Design, Holder, RobotState, Target, ToolState
from bar_assembly_core.design.plan_check import check_plan, end_state, unreached_targets
from bar_assembly_core.geometry import Pose

CINDY, ALICE = "robots/cindy/left_tool0", "robots/alice/left_tool0"


@pytest.fixture
def design(tmp_path: Path) -> Design:
    """The fixture design: a plan that passes every check."""
    return build_design(tmp_path)


def _errors(design: Design, check: int, text: str) -> None:
    """Assert the plan checks report `text` as an error of B<check>."""
    report = check_plan(design)
    assert any(e.startswith(f"B{check}:") and text in e for e in report.errors), report.errors


def _warns(design: Design, check: int, text: str) -> None:
    """Assert the plan checks report `text` as a warning of B<check>, and no error."""
    report = check_plan(design)
    assert report.ok, report.errors
    assert any(w.startswith(f"B{check}:") and text in w for w in report.warnings), report.warnings


def _start(design: Design, action: str, index: int):
    """One movement's start state."""
    return design.actions[action].movements[index].start


def test_fixture_passes(design):
    """The fixture plan has no error and no warning."""
    report = check_plan(design)
    assert report.ok and report.errors == () and report.warnings == ()


def test_b1_floating_bar(design):
    """A present bar neither built nor attached, without a pose, floats: a warning."""
    _warns(with_start(design, "B1_J_joint", 0, poses={}), 1, "bars/B2 is neither built nor attached")


def test_b2_built_bars_reach_the_ground(design):
    """Without B1's ground mate: B1 alone has no engaged mate; B1 and B2 built together do not reach the ground."""
    no_ground = replace(design, mates=frozenset({("joints/J1_female", "joints/J1_male")}))
    _errors(no_ground, 2, "B1_H_M0_approach start: bars/B1 is built but none of its mates is engaged")
    _errors(no_ground, 2, "B1_HR_M0_open start: bars/B1 is built but does not reach the ground")


def test_b3_holders_of_an_unbuilt_bar(design):
    """An unbuilt bar held by two robots."""
    start = _start(design, "B2_J_joint", 2)
    attached = {**start.attached, "bars/B2": (*start.attached["bars/B2"], Holder(ALICE, Pose()))}
    _errors(with_start(design, "B2_J_joint", 2, attached=attached), 3, "held by several robots")


def test_b4_holder_has_a_tool_on_the_part(design):
    """A robot holding a bar while none of its tools is on it or its halves."""
    start = _start(design, "B2_J_joint", 1)
    _errors(with_start(design, "B2_J_joint", 1, tools={**start.tools, "tools/cindy/AT3L": ToolState("open", None)}), 4,
            "robots/cindy holds bars/B2 but none of its tools is on")


def test_b5_unmated_halves_and_mates_never_engaged(design):
    """A half in no mate, and a mate whose bar is never built: warnings."""
    report = check_plan(replace(design, mates=frozenset({("ground/WG0", "joints/G1_ground")})))
    assert any(w.startswith("B5:") and "joints/J1_male (on bars/B2) is in no mate" in w for w in report.warnings)
    bodies = dict(design.bodies)
    bodies["bars/B3"] = replace(bodies["bars/B2"], id="bars/B3")
    bodies["joints/J2_male"] = replace(bodies["joints/J1_male"], id="joints/J2_male", mount="bars/B3")
    bodies["joints/J2_female"] = replace(bodies["joints/J1_female"], id="joints/J2_female")
    never = replace(design, bodies=bodies, mates=design.mates | {("joints/J2_female", "joints/J2_male")})
    _warns(never, 5, "['joints/J2_female', 'joints/J2_male'] is never engaged")


def test_b6_ends_on_tools(design):
    """`ends_on: tools` needs a tool part, and with arms a compliant controller."""
    _errors(with_movement(design, "B1_J_joint", 1, target=None), 6, "needs a tool part")
    _errors(with_movement(design, "B1_J_joint", 2, controller="position"), 6, "needs controller compliant")


def test_b7_carrying_needs_closed_grips(design):
    """Moving an unbuilt bar with an open grip; several holders that do not move coupled."""
    start = _start(design, "B2_J_joint", 2)
    opened = {**start.tools, "tools/cindy/AT3L": ToolState("open", "joints/J1_male")}
    _errors(with_start(design, "B2_J_joint", 2, tools=opened), 7, "moves bars/B2 while tools/cindy/AT3L grip is 'open'")
    attached = {**start.attached, "bars/B2": (*start.attached["bars/B2"], Holder("robots/cindy/right_tool0", Pose()))}
    _errors(with_start(design, "B2_J_joint", 2, attached=attached), 7, "they must all move, coupled")


def test_b8_holding_a_built_bar_freezes_the_robot(design):
    """Cindy holds the built B1 (before her ungrasp): moving her arm is refused, a compliant release allowed."""
    retreat = design.actions["B1_R_release"].movements[1]
    start = _start(design, "B1_R_release", 0)
    frozen = with_start(design, "B1_R_release", 1, attached=start.attached, robots=start.robots)
    _errors(frozen, 8, "robots/cindy holds the built bars/B1")
    form_a = with_movement(design, "B1_R_release", 0, arms=(CINDY,), path="linear", controller="compliant",
                           line=retreat.line)
    assert not any(e.startswith("B8") for e in check_plan(form_a).errors)


def test_b9_tool_on_a_body_it_does_not_hold(design):
    """After her ungrasp Cindy's tool is still on G1: only a linear retreat may move it."""
    _errors(with_movement(design, "B1_R_release", 1, path="free", line={}), 9,
            "tools/cindy/AT3L is on joints/G1_ground, which robots/cindy/left_tool0 does not hold")


def test_b10_attached_and_built_change_only_by_operations(design):
    """Attaching without a grasp, releasing without a grip opening, building without a drive."""
    transfer = design.actions["B2_J_joint"].movements[2]
    both = design.actions["B1_H_hold"].movements[1].target.attached
    _errors(with_movement(design, "B1_J_joint", 1, target=Target(tools={"tools/cindy/AT3L": "closed"}, attached=both)),
            10, "bars/B1 becomes attached to robots/alice/left_tool0 without")
    _errors(with_movement(design, "B2_J_joint", 2, target=replace(transfer.target, attached={})), 10,
            "bars/B2 stops being attached")
    _errors(with_movement(design, "B2_J_joint", 3, drives={}, ends_on="target"), 10,
            "bars/B2 becomes built without an insert")


def test_b10_a_manual_step_may_build_a_bar(design):
    """A bar placed by hand: no tighten drive, but the operator confirms the step."""
    by_hand = with_movement(design, "B2_J_joint", 3, drives={}, ends_on="operator")
    assert not [e for e in check_plan(by_hand).errors if "becomes built without an insert" in e]


def test_b11_movements_chain(design):
    """A target that is not where the next movement starts; a grip that changes between movements."""
    _errors(with_movement(design, "B2_J_joint", 2, target=Target.of(joints={"robots/cindy": joints(0.9)})), 11,
            "robots/cindy joints jump by 0.6")
    # * A movement that drives the base ends at its target's base: the next start must agree with it.
    grasp = design.actions["B1_J_joint"].movements[1]
    driven = Target.of(bases={"robots/cindy": Pose((5.0, 0.0, 0.0))}, tools=grasp.target.tools)
    _errors(with_movement(design, "B1_J_joint", 1, target=driven, base=("robots/cindy",)), 11,
            "robots/cindy base moves between two movements of one action")
    there = end_state(replace(grasp, target=driven, base=("robots/cindy",)))
    assert there.robots["robots/cindy"].base == Pose((5.0, 0.0, 0.0))
    start = _start(design, "B1_J_joint", 2)
    opened = {**start.tools, "tools/cindy/AT3L": ToolState("open", "joints/G1_ground")}
    _errors(with_start(design, "B1_J_joint", 2, tools=opened), 11, "tools/cindy/AT3L grip changes from closed to open")
    # ? A target of null ends where the next movement starts, by definition.
    assert check_plan(with_movement(design, "B2_J_joint", 2, target=None)).ok


def test_b12_hold_released_too_early(design):
    """Alice lets go of B1 before B2, which her hold supports, is built."""
    start = _start(design, "B1_HR_hold_release", 0)
    _errors(with_start(design, "B1_HR_hold_release", 0, built=frozenset({"bars/B1"})), 12,
            "releases bars/B1 before ['bars/B2'] are built")
    assert start.built == {"bars/B1", "bars/B2"}


def test_b13_arms_end_srdf_groups(design):
    """An arm that ends no SRDF group, and an arm of another robot."""
    _errors(with_movement(design, "B2_J_joint", 2, arms=("robots/cindy/left_link2",)), 13, "left_link2")
    _errors(with_movement(design, "B2_J_joint", 2, arms=(ALICE,)), 13, "robots/alice/left_tool0")


def test_b14_grasps_agree_with_the_geometry(design):
    """A grasp off the half's pose in the TCP; a second holder whose forward kinematics disagrees."""
    start = _start(design, "B2_J_joint", 1)
    moved = {**start.attached, "bars/B2": (Holder(CINDY, Pose((0.0, 0.0, 0.2))),)}
    _errors(with_start(design, "B2_J_joint", 1, attached=moved), 14, "the grasp of bars/B2 on robots/cindy/left_tool0")
    start = _start(design, "B1_R_release", 0)
    elsewhere = {**start.robots, "robots/alice": RobotState(start.robots["robots/alice"].base, joints(0.5))}
    _errors(with_start(design, "B1_R_release", 0, robots=elsewhere), 14, "bars/B1 as held by robots/alice/left_tool0")
    bare = replace(design.bodies["joints/J1_male"], pose_in_tcp=None)
    _errors(replace(design, bodies={**design.bodies, "joints/J1_male": bare}), 14,
            "tools/cindy/AT3L is on joints/J1_male, which has no pose_in_tcp")


def test_b15_target_joints_reach_the_target_links(design):
    """A solved insert whose flange target moved afterwards (the bar was moved or copied): the solution is stale."""
    assert unreached_targets(design) == ()
    insert = design.actions["B1_J_joint"].movements[2]
    reached = insert.target.link_poses[CINDY]
    moved = Pose(tuple(p + d for p, d in zip(reached.position, (0.0, 0.002, 0.0))), reached.orientation)
    stale = with_movement(design, "B1_J_joint", 2, target=insert.target.having(link_poses={CINDY: moved}))
    _errors(stale, 15, "robots/cindy/left_tool0 2 mm and")
    (miss,) = check_plan(stale).unreached
    assert (miss.action, miss.movement, miss.bar, miss.link) == ("B1_J_joint", "B1_M2_insert", "bars/B1", CINDY)
    assert miss.distance == pytest.approx(0.002) and miss.angle == pytest.approx(0.0, abs=1e-9)
    assert unreached_targets(stale, tolerance=(0.01, 0.01)) == ()


def test_b15_judges_only_what_is_known(design):
    """No verdict without the base, or without every joint up to the link; none for a target without links."""
    insert = design.actions["B1_J_joint"].movements[2]
    off = insert.target.having(link_poses={CINDY: Pose((9.0, 9.0, 9.0))})
    assert len(unreached_targets(with_movement(design, "B1_J_joint", 2, target=off))) == 1
    cindy = insert.start.robots["robots/cindy"]
    for state in (RobotState(None, cindy.joints), RobotState(cindy.base, None)):
        unknown = replace(insert.start, robots={**insert.start.robots, "robots/cindy": state})
        assert unreached_targets(with_movement(design, "B1_J_joint", 2, start=unknown, target=off)) == ()
    assert unreached_targets(with_movement(design, "B1_J_joint", 2, target=off.having(joints={}))) == ()


def test_geometry_checks_can_be_left_out(design):
    """Without robot files B13, B14 and B15 do not run."""
    bad = with_movement(design, "B2_J_joint", 2, arms=("robots/cindy/left_link2",))
    assert not any(e.startswith("B13") for e in check_plan(bad, geometry=False).errors)


def test_end_state_applies_the_target(design):
    """The end of an insert: target joints over the start's, the bar built; `on` unchanged."""
    insert = design.actions["B1_J_joint"].movements[2]
    end = end_state(insert)
    assert end.built == {"bars/B1"} and end.robots["robots/cindy"].joints["left_joint1"] == 0.5
    assert end.tools["tools/cindy/AT3L"] == insert.start.tools["tools/cindy/AT3L"]


def test_b11_on_changes_only_in_targets(design):
    """A tool that arrives between two movements, instead of in a target, breaks the chain."""
    mount = design.actions["B1_J_joint"].movements[0]
    _errors(with_movement(design, "B1_J_joint", 0, target=replace(mount.target, on={})), 11,
            "tools/cindy/AT3L on changes from None to joints/G1_ground between movements")
    assert end_state(mount).tools["tools/cindy/AT3L"].on == "joints/G1_ground"


def test_b14_reads_the_halfs_pose_in_tcp(design):
    """The grasp is checked against the `pose_in_tcp` of the half the tool is on."""
    turned = replace(design.bodies["joints/J1_male"], pose_in_tcp=Pose(orientation=(0.0, 0.0, 1.0, 0.0)))
    _errors(replace(design, bodies={**design.bodies, "joints/J1_male": turned}), 14,
            "the grasp of bars/B2 on robots/cindy/left_tool0")
    assert check_plan(design).ok
