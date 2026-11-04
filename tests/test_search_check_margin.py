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
from scaffolding_assembly_core.scene import Body, Scene, world_poses

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
