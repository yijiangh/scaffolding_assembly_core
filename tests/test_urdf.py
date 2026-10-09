"""Tests for reading URDFs with yourdfpy: names, link geometry, forward kinematics, scene parts, caching."""

from __future__ import annotations

import os
from pathlib import Path
from xml.etree.ElementTree import ParseError

import numpy as np
import pytest
from design_fixtures import TETRA, build_design, joints, write_robot_files
from scipy.spatial.transform import Rotation

from bar_assembly_core.design.meshes import write_mesh
from bar_assembly_core.geometry import BoxShape, CylinderShape, Pose, TriMesh
from bar_assembly_core.kinematics import link_pose, link_poses
from bar_assembly_core.robot import robot_model
from bar_assembly_core.scene import robot_parts, scene_parts
from bar_assembly_core.urdf import (URDFReadError, link_geometry, load_urdf, mesh_references, movable_joints,
                                    stock_frame_problem, urdf_joints, urdf_links)

#: A robot with every shape type, origins, a scaled mesh, a prismatic joint and a mimic joint.
SHAPES_URDF = """<?xml version="1.0"?>
<robot name="shapes">
  <link name="base"/>
  <link name="body">
    <visual><origin xyz="0 0 0.5" rpy="0 0 1.5707963267948966"/><geometry><mesh filename="m/tetra.obj" scale="2 2 2"/></geometry></visual>
    <visual><origin xyz="1 0 0"/><geometry><box size="0.1 0.2 0.3"/></geometry></visual>
    <collision><origin xyz="0 1 0"/><geometry><cylinder radius="0.05" length="0.4"/></geometry></collision>
    <collision><origin xyz="0 0 2"/><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
  <link name="slider"/>
  <link name="follower"/>
  <joint name="fix" type="fixed"><parent link="base"/><child link="body"/><origin xyz="0 0 1"/></joint>
  <joint name="slide" type="prismatic">
    <parent link="body"/><child link="slider"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="turn" type="revolute">
    <parent link="slider"/><child link="follower"/><origin xyz="0 0.3 0" rpy="0.1 0.2 0.3"/><axis xyz="0 0 1"/>
    <mimic joint="slide" multiplier="2" offset="0.5"/><limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>
"""


@pytest.fixture
def shapes(tmp_path):
    """The shapes robot, with its mesh."""
    write_mesh(TETRA, tmp_path / "m" / "tetra.obj")
    urdf = tmp_path / "shapes.urdf"
    urdf.write_text(SHAPES_URDF)
    return urdf


def test_names_and_joints(tmp_path):
    """Links, joints in file order, movable joints without passive and mimic ones, mesh references."""
    urdf, srdf = write_robot_files(tmp_path)
    assert urdf_links(urdf) == {"base_footprint", "base_link", "wheel_link", "left_link1", "left_link2",
                                "left_tool0", "right_link1", "right_link2", "right_tool0"}
    assert list(urdf_joints(urdf))[:2] == ["base_joint", "wheel_joint"]
    assert urdf_joints(urdf)["left_joint1"] == "revolute"
    assert movable_joints(urdf, srdf) == ("left_joint1", "left_joint2", "right_joint1", "right_joint2")
    assert mesh_references(urdf) == ["meshes/base.obj", "meshes/base.obj"]


def test_load_urdf_cached_until_changed(tmp_path):
    """The same parsed model until the file changes; then a new one with the change."""
    urdf, _ = write_robot_files(tmp_path)
    model = load_urdf(urdf)
    assert load_urdf(urdf) is model
    urdf.write_text(urdf.read_text().replace("</robot>", '  <link name="extra"/>\n</robot>'))
    os.utime(urdf, ns=(1, 1))  # ? A rewrite within the clock's resolution keeps the mtime: force a new one.
    assert load_urdf(urdf) is not model and "extra" in urdf_links(urdf)


def test_broken_urdfs(tmp_path):
    """Broken XML raises ParseError (yourdfpy alone would recover); a robot yourdfpy cannot read, URDFReadError."""
    broken = tmp_path / "broken.urdf"
    broken.write_text("<robot name='x'><link name='a'></robot>")
    with pytest.raises(ParseError):
        urdf_links(broken)
    headless = tmp_path / "headless.urdf"
    headless.write_text("<robot name='x'><link name='a'/><joint name='j' type='fixed'><child link='a'/></joint>"
                        "</robot>")
    with pytest.raises(URDFReadError):
        urdf_links(headless)


def test_link_geometry(shapes):
    """Every shape type in the link frame: mesh scaled and moved by its origin, primitives keeping theirs."""
    geometry = link_geometry(shapes)
    assert set(geometry) == {"body"}
    (mesh, box), (cylinder, sphere) = geometry["body"].visual, geometry["body"].collision
    assert isinstance(mesh, TriMesh) and isinstance(box, BoxShape) and isinstance(cylinder, CylinderShape)
    turn = Rotation.from_euler("z", np.pi / 2).as_matrix()
    np.testing.assert_allclose(mesh.vertices, TETRA.vertices * 2 @ turn.T + [0, 0, 0.5], atol=1e-12)
    assert box.size == (0.1, 0.2, 0.3) and box.origin.position == (1.0, 0.0, 0.0)
    assert (cylinder.radius, cylinder.height) == (0.05, 0.4) and cylinder.origin.position == (0.0, 1.0, 0.0)
    assert isinstance(sphere, TriMesh)
    np.testing.assert_allclose(np.linalg.norm(sphere.vertices - [0, 0, 2], axis=1), 0.1, atol=1e-9)


def test_link_geometry_shared_until_a_mesh_changes(shapes):
    """The same objects on every call; new ones once a mesh file changes."""
    first = link_geometry(shapes)
    assert link_geometry(shapes) is first
    write_mesh(TriMesh.from_arrays(TETRA.vertices * 3, TETRA.faces), shapes.parent / "m" / "tetra.obj")
    os.utime(shapes.parent / "m" / "tetra.obj", ns=(1, 1))
    second = link_geometry(shapes)
    assert second is not first and second["body"].visual[0].vertices.max() > first["body"].visual[0].vertices.max()


def test_link_geometry_missing_mesh(tmp_path):
    """A mesh that is not there is an error, not an empty link."""
    urdf, _ = write_robot_files(tmp_path, write_mesh_file=False)
    with pytest.raises(FileNotFoundError):
        link_geometry(urdf)


def test_link_poses_match_link_pose(shapes, tmp_path):
    """All links at once give the same poses as one at a time, mimic and prismatic joints included."""
    base = Pose((1.0, 2.0, 3.0), tuple(Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_quat()))
    values = {"slide": 0.25}
    poses = link_poses(shapes, base, values)
    assert set(poses) == urdf_links(shapes)
    for link, pose in poses.items():
        one = link_pose(shapes, base, values, link)
        np.testing.assert_allclose(pose.matrix(), one.matrix(), atol=1e-12)
    # * The slider moved 0.25 along x; the follower turned 2 * 0.25 + 0.5 = 1 rad about its z.
    in_root = link_poses(shapes, Pose(), values)
    np.testing.assert_allclose(in_root["slider"].position, (0.25, 0.0, 1.0), atol=1e-12)
    expected = Rotation.from_euler("xyz", [0.1, 0.2, 0.3]) * Rotation.from_euler("z", 1.0)
    assert (Rotation.from_quat(in_root["follower"].orientation) * expected.inv()).magnitude() < 1e-9

    urdf, _ = write_robot_files(tmp_path / "fake")
    assert set(link_poses(urdf, Pose(), joints(0.4), ["left_tool0"])) == {"left_tool0"}
    with pytest.raises(KeyError):
        link_poses(urdf, Pose(), {}, ["nowhere"])


def test_stock_frames(tmp_path):
    """The stock UR base frames pass; a turned joint below base_link is named with its rpy."""
    urdf = tmp_path / "ur.urdf"
    text = """<robot name="ur"><link name="arm_base_link"/><link name="arm_base_link_inertia"/>
      <joint name="arm_base_link-base_link_inertia" type="fixed"><parent link="arm_base_link"/>
        <child link="arm_base_link_inertia"/><origin xyz="0 0 0" rpy="0 0 {yaw}"/></joint></robot>"""
    urdf.write_text(text.format(yaw=np.pi))
    assert stock_frame_problem(urdf, "arm") is None
    urdf.write_text(text.format(yaw=0.5) + " ")
    assert "arm_base_link-base_link_inertia (rpy 0 0 0.5)" in stock_frame_problem(urdf, "arm")


def test_robot_parts_and_scene_parts(tmp_path):
    """A scene's parts: enabled bodies, then each enabled robot's links with shapes and its tools at their flanges."""
    design = build_design(tmp_path)
    movement = next(m for _, m in design.movements() if m.id == "B1_M1_grasp")
    scene = design.scene_at(movement)
    parts = {part_id: (geometry, pose) for part_id, geometry, pose in scene_parts(scene)}
    cindy = scene.robots["robots/cindy"]
    assert not scene.robots["robots/alice"].enabled and not any(p.startswith("robots/alice/") for p in parts)
    assert "robots/cindy/base_link" in parts and "robots/cindy/left_link1" not in parts  # * no shapes
    assert parts["robots/cindy/base_link"][0] is cindy.model.link_geometry["base_link"]
    for flange, tool in cindy.model.tools.items():
        np.testing.assert_allclose(parts[tool.id][1].matrix(),
                                   link_pose(cindy.model.urdf, cindy.base, cindy.joints, flange).matrix(), atol=1e-12)
    for body_id, body in scene.bodies.items():
        assert (body_id in parts) == (body.enabled and body_id in scene.world_poses)

    model = robot_model("fake", *write_robot_files(tmp_path / "fake"))
    given = {link: Pose((float(i), 0.0, 0.0)) for i, link in enumerate(sorted(model.links))}
    robot = scene.robots["robots/cindy"].copy()
    robot.model = model
    assert {p: pose for p, _, pose in robot_parts(robot, given)} == {"robots/cindy/base_link": given["base_link"]}


HUSKY_ROBOTS = {
    "cindy": ("mt_husky_dual_ur5_e_moveit_config", "husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf", 37, 12),
    "alice": ("mt_husky_moveit_config", "husky_ur5_e_no_base_joint_Alice_Calibrated.urdf", 28, 6),
    "belle": ("mt_husky_moveit_config", "husky_ur5_e_no_base_joint_Belle_Calibrated.urdf", 28, 6),
}


@pytest.mark.parametrize("name", sorted(HUSKY_ROBOTS))
def test_husky_robots(name):
    """The calibrated Husky robots: every link read, meshes found through their packages, arm joints movable."""
    data = os.environ.get("HUSKY_DATA")
    if not data:
        pytest.skip("HUSKY_DATA (the folder holding husky_urdf/) is not set")
    package, filename, link_count, arm_joints = HUSKY_ROBOTS[name]
    packages = Path(data) / "husky_urdf"
    urdf = packages / package / "urdf" / filename
    assert len(urdf_links(urdf)) == link_count
    assert sum(kind == "revolute" for kind in urdf_joints(urdf).values()) >= arm_joints
    geometry = link_geometry(urdf, [packages])
    assert geometry and all(g.visual or g.collision for g in geometry.values())
    assert link_geometry(urdf, [packages]) is geometry
