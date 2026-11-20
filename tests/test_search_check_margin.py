"""`SearchCheck` with a margin: a pair closer than the margin is a hit, though nothing touches.

The fake robot's left arm reaches 0.5 m and carries a 0.2 m tetrahedron (AT3L), whose tip sits at (0.7, 0, 0); a
static box starts 5 mm beyond it. PyBullet pads each convex shape by about 1 mm, so it measures that gap as 3 mm.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

import pytest
from process_fixtures import TETRA, write_robot_files

from scaffolding_assembly_core.geometry import BoxShape, Geometry, Pose, TriMesh
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.robot import RobotObject, Tool, robot_model
from scaffolding_assembly_core.scene import Attachment, Body, Scene, world_poses

CINDY = "robots/cindy"
TOOL_MESH = TriMesh.from_arrays(TETRA.vertices * 2, TETRA.faces)
AT3L = Tool("tools/cindy/AT3L", Geometry((TOOL_MESH,), (TOOL_MESH,)), Pose((0.0, 0.0, 0.12)), "scaffolding_v3")
BOX = BoxShape((0.1, 0.1, 0.1))
#: The box, its near face 5 mm beyond AT3L's tip.
NEAR = Body("obstacles/box", Geometry((BOX,), (BOX,)), Pose((0.755, 0.0, 0.0)))
#: The gap as PyBullet measures it, metres.
GAP = 0.003


@pytest.fixture
def mirror(tmp_path: Path) -> Iterator[CompasFabMirror]:
    """Cindy's mirror, synced with AT3L on her left arm, the right arm turned away, and the box near AT3L."""
    urdf, srdf = write_robot_files(tmp_path, reach=0.5)
    model = robot_model("cindy", urdf, srdf, {"left_tool0": AT3L})
    robot = RobotObject(CINDY, model, Pose(), {"right_joint1": 3.14159})
    scene = Scene(bodies={NEAR.id: NEAR}, robots={CINDY: robot})
    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    made = CompasFabMirror(CINDY)
    made.sync(scene)
    yield made
    made.close()


def test_a_margin_larger_than_the_gap_hits(mirror):
    """3 mm apart as measured: clear for contact and a 2 mm margin; a 1 cm margin reports the tool and the box."""
    assert mirror.collisions(full_report=True) == []
    names = ("left_joint1",)
    assert mirror.search_check(names)([0.0]) is None
    assert mirror.search_check(names, GAP - 0.001)([0.0]) is None
    assert mirror.search_check(names, 0.01)([0.0]) == ("tools/cindy/AT3L", "obstacles/box")


def test_the_margin_clears_once_the_arm_turns_away(mirror):
    """Turned 0.5 rad, the tool is well clear of the box even for a 1 cm margin."""
    assert mirror.search_check(("left_joint1",), 0.01)([0.5]) is None


def test_the_self_margin_applies_to_the_robot_against_itself_only(mirror):
    """The box still hits under a 1 cm margin with no self margin; a 1 m self margin hits the robot's own parts only."""
    names = ("left_joint1",)
    assert mirror.search_check(names, 0.01, self_margin=0.0)([0.0]) == ("tools/cindy/AT3L", "obstacles/box")
    hit = mirror.search_check(names, 0.0, self_margin=1.0)([0.5])
    assert hit is not None and "obstacles/box" not in hit


def test_a_pair_margin_replaces_the_margin_of_that_pair_only(mirror):
    """Under a 1 cm margin the box hits; given its own 2 mm, it is clear, and every other pair keeps the 1 cm."""
    names = ("left_joint1",)
    pair = frozenset(("tools/cindy/AT3L", "obstacles/box"))
    assert mirror.search_check(names, 0.01, pair_margins={pair: GAP - 0.001})([0.0]) is None
    assert mirror.search_check(names, 0.01, pair_margins={pair: 0.01})([0.0]) == ("tools/cindy/AT3L", "obstacles/box")
    # * The robot against itself still keeps the 1 m self margin: the box's own margin changes nothing there.
    hit = mirror.search_check(names, 0.0, self_margin=1.0, pair_margins={pair: 0.0})([0.5])
    assert hit is not None and "obstacles/box" not in hit


def test_distances_lists_the_pairs_within_their_margin_and_how_close(mirror):
    """At the start only the tool and the box are within 1 cm, at the measured gap; turned away, nothing is."""
    check = mirror.search_check(("left_joint1",), 0.01, self_margin=0.0)
    close = check.distances([0.0])
    assert list(close) == [("tools/cindy/AT3L", "obstacles/box")]
    assert close[("tools/cindy/AT3L", "obstacles/box")] == pytest.approx(GAP, abs=5e-4)
    assert check.distances([0.5]) == {}


def test_signed_distances_give_the_depth_of_an_overlap(tmp_path: Path):
    """The box moved 10 mm towards the tool: its tip is 5 mm inside, which PyBullet's padding makes 7 mm deep.

    The padding shifts every distance by the same 2 mm (the 5 mm gap above measures 3 mm), so the signed distance
    runs on through contact without a jump. Unsigned, the pair is in contact (0); turned away, nothing is near.
    """
    urdf, srdf = write_robot_files(tmp_path, reach=0.5)
    model = robot_model("cindy", urdf, srdf, {"left_tool0": AT3L})
    robot = RobotObject(CINDY, model, Pose(), {"right_joint1": 3.14159})
    inside = Body("obstacles/box", Geometry((BOX,), (BOX,)), Pose((0.745, 0.0, 0.0)))
    scene = Scene(bodies={inside.id: inside}, robots={CINDY: robot})
    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    made = CompasFabMirror(CINDY)
    try:
        made.sync(scene)
        pair = ("tools/cindy/AT3L", "obstacles/box")
        check = made.search_check(("left_joint1",), 0.01, self_margin=0.0)
        assert check.distances([0.0]) == {pair: 0.0}
        assert check.distances([0.0], signed=True)[pair] == pytest.approx(-0.005 - (0.005 - GAP), abs=5e-4)
        assert check.distances([0.5], signed=True) == {}
    finally:
        made.close()


def test_base_moves_checks_the_robot_body_against_the_scene_too(tmp_path: Path):
    """A box on Cindy's base mesh: a check over the left arm alone never sees it; with `base_moves` it does."""
    urdf, srdf = write_robot_files(tmp_path, reach=0.5)
    model = robot_model("cindy", urdf, srdf, {"left_tool0": AT3L})
    robot = RobotObject(CINDY, model, Pose(), {"right_joint1": 3.14159})
    small = BoxShape((0.02, 0.02, 0.02))
    on_base = Body("obstacles/crate", Geometry((small,), (small,)), Pose((0.02, 0.02, 0.02)))
    scene = Scene(bodies={on_base.id: on_base}, robots={CINDY: robot})
    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    made = CompasFabMirror(CINDY)
    try:
        made.sync(scene)
        pair = (f"{CINDY}/base_link", "obstacles/crate")
        assert pair in made.collisions(full_report=True)
        names = ("left_joint1",)
        assert pair not in made.search_check(names, 0.01).distances([0.0])
        assert made.search_check(names, 0.01, base_moves=True).distances([0.0], signed=True)[pair] < 0.0
    finally:
        made.close()


def test_base_moves_leaves_a_body_on_the_root_link_where_the_sync_put_it(tmp_path: Path):
    """A body held by the root link (PyBullet index -1, no link state): `base_moves` checks it, never re-poses it."""
    urdf, srdf = write_robot_files(tmp_path, reach=0.5)
    model = robot_model("cindy", urdf, srdf, {"left_tool0": AT3L})
    robot = RobotObject(CINDY, model, Pose(), {"right_joint1": 3.14159})
    small = BoxShape((0.02, 0.02, 0.02))
    held = Body("obstacles/plate", Geometry((small,), (small,)), Attachment(CINDY, "base_footprint", Pose((0.3, 0, 0))))
    scene = Scene(bodies={held.id: held, NEAR.id: NEAR}, robots={CINDY: robot})
    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    made = CompasFabMirror(CINDY)
    try:
        made.sync(scene)
        check = made.search_check(("left_joint1",), 0.01, base_moves=True)
        assert check.distances([0.0], signed=True) == check.distances([0.0], signed=True)  # posing it never raises
    finally:
        made.close()
