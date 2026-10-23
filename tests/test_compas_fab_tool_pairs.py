"""CC.6 in the compas_fab mirror: the acting robot's tools against other robots and against each other.

compas_fab never checks a tool against a tool, and another robot is one tool in the cell. The mirror adds that check
to its planner (`with_tool_pairs`), so `collisions` and the lent planner report it, and `SearchCheck` checks the same
pairs. The scenes are built by hand: the fake robot with arms reaching 0.5 m (`write_robot_files(reach=0.5)`), Cindy
at the origin with AT3L (left, at +X) and AT3R (right, turned back to -X), Alice with a Grip box facing Cindy.
"""

from __future__ import annotations

from math import pi
from pathlib import Path
from typing import Dict, Iterator, Optional, Tuple

import numpy as np
import pytest
from compas_fab.backends import CollisionCheckError, PyBulletPlanner
from compas_robots import ToolModel
from process_fixtures import TETRA, write_robot_files

from scaffolding_assembly_core.geometry import BoxShape, Geometry, Pose, TriMesh, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror, _check_collision_with_tool_pairs
from scaffolding_assembly_core.robot import RobotModel, RobotObject, Tool, robot_model
from scaffolding_assembly_core.scene import Body, Scene, world_poses

CINDY, ALICE = "robots/cindy", "robots/alice"
TOOL_MESH = TriMesh.from_arrays(TETRA.vertices * 2, TETRA.faces)
AT3L = Tool("tools/cindy/AT3L", Geometry((TOOL_MESH,), (TOOL_MESH,)), Pose((0.0, 0.0, 0.12)), "scaffolding_v3")
AT3R = Tool("tools/cindy/AT3R", Geometry((TOOL_MESH,), (TOOL_MESH,)), Pose((0.0, 0.0, 0.12)), "scaffolding_v3")
GRIP = Tool("tools/alice/Grip", Geometry((BoxShape((0.1, 0.1, 0.2)),), (BoxShape((0.1, 0.1, 0.2)),)),
            Pose((0.0, 0.0, 0.2)), "robotiq")

# * Cindy's AT3L is a 0.2 m tetrahedron from her left flange at (0.5, 0, 0): x 0.5 to 0.7, y and z 0 to 0.2.
#: Cindy's joints in most tests: the right arm turned back, so AT3R sits 1 m from AT3L.
RIGHT_AWAY = {"right_joint1": pi}
#: Where Alice's flange (the Grip box's centre) goes: inside AT3L, 0.15 m beside it, 3 m away.
GRIP_IN_AT3L, GRIP_BESIDE, GRIP_FAR = (0.55, 0.05, 0.05), (0.5, -0.2, 0.05), (3.55, 0.05, 0.05)
#: Cindy's tool against Alice, as each mirror names it.
CINDY_HIT, ALICE_HIT = ("tools/cindy/AT3L", ALICE), ("tools/alice/Grip", CINDY)


@pytest.fixture
def models(tmp_path: Path) -> Tuple[Path, RobotModel, RobotModel]:
    """The fake robot's URDF, then Cindy's model (AT3L, AT3R) and Alice's (Grip), each built once."""
    urdf, srdf = write_robot_files(tmp_path, reach=0.5)
    return (urdf, robot_model("cindy", urdf, srdf, {"left_tool0": AT3L, "right_tool0": AT3R}),
            robot_model("alice", urdf, srdf, {"left_tool0": GRIP}))


@pytest.fixture
def mirrors() -> Iterator[Dict[str, CompasFabMirror]]:
    """Cindy's and Alice's mirrors, closed after the test."""
    made = {robot_id: CompasFabMirror(robot_id) for robot_id in (CINDY, ALICE)}
    yield made
    for mirror in made.values():
        mirror.close()


def _scene(models: Tuple[Path, RobotModel, RobotModel], grip_at: Tuple[float, float, float],
           cindy_joints: Optional[Dict[str, float]] = None, bodies: Tuple[Body, ...] = ()) -> Scene:
    """Cindy at the origin and Alice facing her (-X) with her flange at `grip_at`, so her base is 0.5 m behind it.

    Args:
        models: From the `models` fixture.
        grip_at: World position of Alice's left flange.
        cindy_joints: Cindy's joints; None: `RIGHT_AWAY`.
        bodies: Static bodies.
    """
    urdf, cindy, alice = models
    facing_cindy = Pose(grip_at, (0.0, 0.0, 1.0, 0.0))
    alice_base = compose(facing_cindy, invert(link_pose(urdf, Pose(), {}, "left_tool0")))
    robots = {CINDY: RobotObject(CINDY, cindy, Pose(), dict(RIGHT_AWAY if cindy_joints is None else cindy_joints)),
              ALICE: RobotObject(ALICE, alice, alice_base, {})}
    scene = Scene(bodies={body.id: body for body in bodies}, robots=robots)
    scene.world_poses = world_poses(scene.bodies, scene.robots,
                                    lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    return scene


def _sync(mirrors: Dict[str, CompasFabMirror], scene: Scene) -> None:
    """Sync every mirror to one scene."""
    for mirror in mirrors.values():
        mirror.sync(scene)


def test_only_tools_overlap_is_reported(models, mirrors):
    """Alice's Grip inside Cindy's AT3L, bases apart: compas_fab alone sees nothing; each mirror reports its tool."""
    _sync(mirrors, _scene(models, GRIP_IN_AT3L))
    for mirror in mirrors.values():
        # ? compas_fab's own check (CC.1 to CC.5) passes: nothing but the two tools overlaps.
        PyBulletPlanner.check_collision(mirror.planner, mirror.state, {"full_report": True})
    assert mirrors[CINDY].collisions(full_report=True) == [CINDY_HIT]
    assert mirrors[ALICE].collisions(full_report=True) == [ALICE_HIT]


def test_separated_tools_are_clear(models, mirrors):
    """Alice 3 m away: clear on both sides."""
    _sync(mirrors, _scene(models, GRIP_FAR))
    assert mirrors[CINDY].collisions(full_report=True) == []
    assert mirrors[ALICE].collisions(full_report=True) == []


def test_own_tools_against_each_other(models, mirrors):
    """Cindy's two flanges at the same place: AT3L hits AT3R; with the right arm turned back they are clear."""
    mirror = mirrors[CINDY]
    mirror.sync(_scene(models, GRIP_FAR, cindy_joints={}))
    assert mirror.collisions(full_report=True) == [("tools/cindy/AT3L", "tools/cindy/AT3R")]
    mirror.sync(_scene(models, GRIP_FAR, cindy_joints={"right_joint1": pi}))
    assert mirror.collisions(full_report=True) == []


def test_search_check_agrees_with_collisions(models, mirrors):
    """Cindy's left arm swings past Alice's Grip: the fast check is clear exactly when `collisions` is."""
    mirror = mirrors[CINDY]
    mirror.sync(_scene(models, GRIP_BESIDE))
    assert mirror.collisions(full_report=True) == []
    names = ("left_joint1", "left_joint2")
    check = mirror.search_check(names)
    samples = np.random.default_rng(0).uniform((-1.0, -3.0), (1.0, 3.0), size=(200, 2))
    fast, slow = [], []
    for values in samples:
        fast.append(check(values))
        slow.append(mirror.collisions({name: float(value) for name, value in zip(names, values)}))
    assert all((hit is None) == (pairs == []) for hit, pairs in zip(fast, slow))
    assert CINDY_HIT in fast
    clear = sum(hit is None for hit in fast)
    assert 5 < clear < 195


def test_lent_planner_raises_on_tool_overlap(models, mirrors, capsys):
    """The lent planner checks CC.6 too, skips it with `_skip_cc2`, and never prints it."""
    mirror = mirrors[CINDY]
    mirror.sync(_scene(models, GRIP_IN_AT3L))
    with mirror.lend() as planner:
        with pytest.raises(CollisionCheckError, match="CC.6"):
            planner.check_collision(mirror.state)
        planner.check_collision(mirror.state, {"_skip_cc2": True})
        capsys.readouterr()
        with pytest.raises(CollisionCheckError) as raised:
            planner.check_collision(mirror.state, {"verbose": True, "full_report": True})
    printed = capsys.readouterr().out
    assert "CC.2" in printed and "CC.6" not in printed
    pairs = raised.value.collision_pairs
    assert pairs == [(mirror.cell.tool_models["tools/cindy/AT3L"], mirror.cell.tool_models[ALICE])]
    assert all(isinstance(model, ToolModel) for pair in pairs for model in pair)
    assert mirror.planner.check_collision.__func__ is _check_collision_with_tool_pairs


def test_handoff_on_one_bar(models, mirrors):
    """Both tools touch a static bar but not each other: clear; once the Grip is inside AT3L, only the tool pair."""
    bar = BoxShape((0.4, 0.02, 0.02))
    # * From x 0.55 (inside AT3L) to 0.95 (inside the Grip beside it at 0.9).
    held = Body("bars/B1", Geometry((bar,), (bar,)), Pose((0.75, 0.03, 0.03)),
                touches=("tools/cindy/AT3L", "tools/alice/Grip"))
    _sync(mirrors, _scene(models, (0.9, 0.05, 0.05), bodies=(held,)))
    assert mirrors[CINDY].collisions(full_report=True) == []
    assert mirrors[ALICE].collisions(full_report=True) == []
    _sync(mirrors, _scene(models, (0.6, 0.05, 0.05), bodies=(held,)))
    assert mirrors[CINDY].collisions(full_report=True) == [CINDY_HIT]
    assert mirrors[ALICE].collisions(full_report=True) == [ALICE_HIT]
