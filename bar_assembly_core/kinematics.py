"""
Forward kinematics from URDF files, in numpy: link poses at given joint values, for scenes without a live robot.

The joint tree comes from yourdfpy (`urdf.load_urdf`), re-read when the file changes. Revolute, continuous and
prismatic joints move, mimic joints follow their source; any other type counts as fixed. Safe to call from several
threads: nothing parsed is ever changed (the yourdfpy model is never posed).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation
from yourdfpy import URDF

from .geometry import Pose, compose
from .urdf import load_urdf

#: Joint types that move: rotate about the axis, or slide along it.
_ROTATING, _SLIDING = ("revolute", "continuous"), ("prismatic",)


@dataclass(frozen=True)
class _Joint:
    """One URDF joint: its parent link, the child's origin in the parent frame, and how it moves."""

    name: str
    parent: str
    origin: np.ndarray
    kind: str
    axis: np.ndarray
    # (source joint, multiplier, offset), or None.
    mimic: Optional[Tuple[str, float, float]]


@dataclass(frozen=True)
class _Tree:
    """A URDF's joints by child link, and its link names, built from one yourdfpy model."""

    model: URDF
    joints: Dict[str, _Joint]
    links: frozenset


#: Resolved path -> the tree of the model `load_urdf` last returned for it.
_TREES: Dict[Path, _Tree] = {}


def _tree(urdf_file: Path) -> _Tree:
    """The joint tree of a URDF, rebuilt only when `load_urdf` parses the file again."""
    path = Path(urdf_file).resolve()
    model = load_urdf(path)
    tree = _TREES.get(path)
    if tree is not None and tree.model is model:
        return tree
    joints = {}
    for joint in model.robot.joints:
        axis = np.array([1.0, 0.0, 0.0]) if joint.axis is None else np.asarray(joint.axis, dtype=float)
        mimic = joint.mimic
        joints[joint.child] = _Joint(
            name=joint.name, parent=joint.parent,
            origin=np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float),
            kind=joint.type, axis=axis / np.linalg.norm(axis),
            mimic=None if mimic is None else (mimic.joint, 1.0 if mimic.multiplier is None else float(mimic.multiplier),
                                              0.0 if mimic.offset is None else float(mimic.offset)))
    tree = _TREES[path] = _Tree(model, joints, frozenset(model.link_map))
    return tree


def _motion(joint: _Joint, joints: Mapping[str, float]) -> np.ndarray:
    """The 4x4 transform a joint adds at the given joint values (identity for a fixed joint)."""
    matrix = np.eye(4)
    if joint.kind not in _ROTATING + _SLIDING:
        return matrix
    if joint.mimic is not None:
        source, multiplier, offset = joint.mimic
        value = multiplier * float(joints.get(source, 0.0)) + offset
    else:
        value = float(joints.get(joint.name, 0.0))
    if joint.kind in _ROTATING:
        matrix[:3, :3] = Rotation.from_rotvec(joint.axis * value).as_matrix()
    else:
        matrix[:3, 3] = joint.axis * value
    return matrix


def link_pose(urdf_file: Path, base: Pose, joints: Mapping[str, float], link: str) -> Pose:
    """World pose of one link's frame.

    Args:
        urdf_file: The robot's URDF.
        base: World pose of the URDF's root link.
        joints: Values by joint name; joints not given are 0, unknown names are ignored.
        link: A URDF link name, e.g. "left_ur_arm_tool0".

    Returns:
        Pose: The link frame in the world.

    Raises:
        KeyError: If the URDF has no such link.
    """
    tree = _tree(urdf_file)
    if link not in tree.links:
        raise KeyError(f"{Path(urdf_file).name} has no link {link!r}")
    matrix = np.eye(4)
    while link in tree.joints:
        joint = tree.joints[link]
        matrix = joint.origin @ _motion(joint, joints) @ matrix
        link = joint.parent
    return compose(base, Pose.from_matrix(matrix))


def link_poses(urdf_file: Path, base: Pose, joints: Mapping[str, float],
               links: Optional[Iterable[str]] = None) -> Dict[str, Pose]:
    """World poses of many link frames at once, each joint's transform computed once.

    Args:
        urdf_file: The robot's URDF.
        base: World pose of the URDF's root link.
        joints: Values by joint name; joints not given are 0, unknown names are ignored.
        links: The links wanted; None: every link.

    Returns:
        dict[str, Pose]: Link name -> its frame in the world.

    Raises:
        KeyError: If the URDF lacks a link asked for.
    """
    tree = _tree(urdf_file)
    wanted = tree.links if links is None else tuple(links)
    missing = [link for link in wanted if link not in tree.links]
    if missing:
        raise KeyError(f"{Path(urdf_file).name} has no links {missing}")
    in_root: Dict[str, np.ndarray] = {}

    def resolve(link: str) -> np.ndarray:
        """The link frame in the root link's frame, parents first (iterative: chains may be long)."""
        chain = []
        while link not in in_root and link in tree.joints:
            chain.append(link)
            link = tree.joints[link].parent
        matrix = in_root.setdefault(link, np.eye(4))
        for child in reversed(chain):
            joint = tree.joints[child]
            matrix = in_root[child] = matrix @ joint.origin @ _motion(joint, joints)
        return matrix

    return {link: compose(base, Pose.from_matrix(resolve(link))) for link in wanted}
