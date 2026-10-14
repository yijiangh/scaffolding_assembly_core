"""Tests for scenes from processes: scene_at, scene_after, id maps with retarget, and hold scenes."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from process_fixtures import build_process

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.process.hold import closing_movement, hold_scenes, release_movement
from scaffolding_assembly_core.process.scenes import seeded
from scaffolding_assembly_core.ids import IdMap
from scaffolding_assembly_core.scene import retarget
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.scene import Attachment


@pytest.fixture
def process(tmp_path):
    """The fixture process: Cindy builds B1 and B2; Alice holds B1 until B2 is built."""
    return build_process(tmp_path)


def _movement(process, movement_id):
    """One movement of the process, by id."""
    return next(movement for _, movement in process.movements() if movement.id == movement_id)


def test_scene_at_robots_and_held_body(process):
    """An absent robot is disabled, unknown joints are unmeasured; an attached bar and its halves follow its link."""
    scene = process.scene_at(_movement(process, "B1_M0_mount"))
    alice, cindy = scene.robots["robots/alice"], scene.robots["robots/cindy"]
    assert not alice.enabled and cindy.enabled
    assert cindy.unmeasured == frozenset(cindy.model.movable_joints) and cindy.acting_problems()
    assert not scene.bodies["bars/B1"].enabled and not scene.bodies["joints/G1_ground"].enabled

    movement = _movement(process, "B1_M1_grasp")
    scene = process.scene_at(movement)
    cindy = scene.robots["robots/cindy"]
    grasp = movement.start.attached["bars/B1"][0].grasp
    assert scene.bodies["bars/B1"].placement == Attachment("robots/cindy", "left_tool0", grasp)
    offset = compose(invert(process.bodies["bars/B1"].assembly_pose), process.bodies["joints/G1_ground"].assembly_pose)
    on_flange = Attachment("robots/cindy", "left_tool0", compose(grasp, offset))
    assert scene.bodies["joints/G1_ground"].placement == on_flange
    tool0 = link_pose(cindy.model.urdf, cindy.base, cindy.joints, "left_tool0")
    np.testing.assert_allclose(scene.world_poses["bars/B1"].position, compose(tool0, grasp).position, atol=1e-9)
    # * Derived contacts: the tool with the half it is on, halves with their bar, the pending ground mate, wheels.
    assert set(scene.bodies["joints/G1_ground"].touches) == {"tools/cindy/AT3L", "bars/B1", "ground/WG0"}
    assert set(scene.bodies["bars/B1"].touches) == {"joints/G1_ground", "joints/J1_female", "tools/cindy/AT3L"}
    assert set(scene.bodies["ground/WG0"].touches) == {"joints/G1_ground", "robots/cindy/wheel_link"}
    assert cindy.model.tool_touches["left_tool0"] == ("left_tool0",), "a tool may touch the last links of its arm"


def test_scene_at_built_and_staged_bars(process):
    """A built bar stands at its assembly pose while held; a staged bar at its written pose."""
    scene = process.scene_at(_movement(process, "B1_R_M0_ungrasp"))
    assert scene.bodies["bars/B1"].placement == process.bodies["bars/B1"].assembly_pose
    assert scene.bodies["bars/B2"].placement == Pose((2.0, 0.0, 0.1)) and scene.bodies["bars/B2"].enabled


def test_scene_at_unknown_base(process):
    """A base the process leaves open makes the robot refuse to act until its owner sets one."""
    cindy = process.scene_at(_movement(process, "B1_R_M1_retreat")).robots["robots/cindy"]
    assert not cindy.base_tracked and "its base is not tracked" in cindy.acting_problems()


def test_scene_shares_only_geometry_and_models(process):
    """Editing a scene never reaches the process or another scene; geometry and models are the same objects."""
    movement = _movement(process, "B1_M1_grasp")
    first, second = process.scene_at(movement), process.scene_at(movement)
    first.robots["robots/cindy"].joints["left_joint1"] = 9.0
    first.bodies["ground/WG0"].enabled = False
    assert movement.start.robots["robots/cindy"].joints["left_joint1"] != 9.0
    assert second.robots["robots/cindy"].joints["left_joint1"] != 9.0 and second.bodies["ground/WG0"].enabled
    assert first.bodies["bars/B2"] is not second.bodies["bars/B2"]
    assert first.bodies["bars/B2"].geometry is process.bodies["bars/B2"].geometry
    assert first.robots["robots/cindy"].model is second.robots["robots/cindy"].model
    # ? A new Process object with the same robots and tools keeps the same model: mirrors don't rebuild.
    edited = replace(process, schedule=process.schedule)
    assert edited.scene_at(movement).robots["robots/cindy"].model is first.robots["robots/cindy"].model


def test_scene_after_follows_the_schedule(process):
    """After B2: the next action's start. After B1 (its hold release ends the schedule): the last target's joints."""
    after_b2 = process.scene_after("bars/B2")
    assert after_b2.robots["robots/alice"].enabled and after_b2.bodies["bars/B2"].enabled
    assert after_b2.bodies["bars/B2"].placement == process.bodies["bars/B2"].assembly_pose
    after_b1 = process.scene_after("bars/B1")
    assert set(after_b1.robots["robots/alice"].joints.values()) == {0.9}
    with pytest.raises(KeyError):
        process.scene_after("bars/none")


def test_hold_scenes(process):
    """Alice's hold of B1 is solved where her grip closes (Cindy still holds B1) and checked where she lets go."""
    assert closing_movement(process, "B1_H_hold").id == "B1_H_M1_close"
    assert release_movement(process, "B1_H_hold").id == "B1_HR_M0_open"
    solve, release = hold_scenes(process, "B1_H_hold")
    assert solve.bodies["bars/B1"].enabled and solve.bodies["bars/B2"].placement == Pose((2.0, 0.0, 0.1))
    assert isinstance(solve.bodies["bars/B1"].placement, Pose)
    assert "tools/alice/Grip" in solve.bodies["joints/J1_female"].touches
    assert isinstance(release.bodies["bars/B2"].placement, Pose) and release.bodies["bars/B2"].placement != Pose(
        (2.0, 0.0, 0.1))
    with pytest.raises(ValueError):
        closing_movement(process, "B1_J_joint")


def test_id_map_refuses_unmapped_ids():
    """A mapped robot maps its links; anything not listed is refused."""
    ids = IdMap({"robots/cindy": "robots/a200-0806", "bars/B1": "cell/bars/B1"})
    assert ids("robots/cindy/left_tool0") == "robots/a200-0806/left_tool0" and ids("bars/B1") == "cell/bars/B1"
    assert "robots/alice/left_tool0" not in ids
    with pytest.raises(KeyError):
        ids("robots/alice")


def test_retarget_swaps_the_robot_only(process):
    """The grasp and link carry over to the mapped robot; an unmapped robot or a missing link is refused."""
    scene = process.scene_at(_movement(process, "B1_M1_grasp"))
    model = scene.robots["robots/cindy"].model
    grasp = scene.bodies["bars/B1"].placement.offset
    held = {"bars/B1": scene.bodies["bars/B1"].placement, "t/on_body": Attachment("bars/B1", None, Pose())}
    moved = retarget(held, IdMap({"robots/cindy": "robots/real"}), {"robots/real": model})
    assert moved["bars/B1"] == Attachment("robots/real", "left_tool0", grasp)
    assert moved["t/on_body"] == held["t/on_body"]
    with pytest.raises(KeyError):
        retarget(held, IdMap({}), {})
    other = SimpleNamespace(links=frozenset({"base_link"}))  # ? only `links` is read
    with pytest.raises(ValueError, match="left_tool0"):
        retarget(held, IdMap({"robots/cindy": "robots/real"}), {"robots/real": other})


def test_seeded_fills_in_what_the_process_leaves_open(process):
    """Cindy's joints are null at her mount: seeded, she can be planned for; known joints keep their value."""
    scene = process.scene_at(_movement(process, "B1_M0_mount"))
    cindy = scene.robots["robots/cindy"]
    assert cindy.acting_problems()
    seeded(scene, "robots/cindy", {"left_joint1": 0.4})
    assert not cindy.acting_problems() and cindy.joints["left_joint1"] == 0.4 and cindy.joints["left_joint2"] == 0.0
    scene = process.scene_at(_movement(process, "B1_M1_grasp"))
    before = dict(scene.robots["robots/cindy"].joints)
    seeded(scene, "robots/cindy", {"left_joint1": 0.4})
    assert scene.robots["robots/cindy"].joints == before
