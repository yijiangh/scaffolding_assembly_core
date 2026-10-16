"""
URDF and SRDF files, and the conventions of the UR arms every user of the core shares.

URDFs are read with yourdfpy (`load_urdf`, as the monitor's viewer does): names, joints, mesh references and each
link's shapes (`link_geometry`). SRDFs, copying and path rewriting use plain `xml.etree`, so comments survive.

! A process URDF's mesh paths are relative to the URDF: resolve them against its folder, not the working directory.
"""

from __future__ import annotations

import os
import re
import shutil
from dataclasses import dataclass
from hashlib import sha256
from io import BytesIO
from pathlib import Path, PurePath
from threading import Lock
from typing import Dict, List, Mapping, Optional, Sequence, Tuple
from xml.etree.ElementTree import Element, ElementTree, TreeBuilder, XMLParser, parse, tostring

import numpy as np
from scipy.spatial.transform import Rotation
from trimesh.creation import icosphere
from yourdfpy import URDF

from .geometry import BoxShape, CylinderShape, Geometry, Pose, Shape, TriMesh, read_mesh

PACKAGE_SCHEME, FILE_SCHEME = "package://", "file://"


class URDFReadError(ValueError):
    """A URDF that is well-formed XML but not a robot yourdfpy can read (e.g. a joint without a parent)."""


# --- --- --- --- --- READING --- --- --- --- ---

def _root(path: Path) -> Element:
    """The root element of an XML file, comments kept so a copied URDF keeps them."""
    parser = XMLParser(target=TreeBuilder(insert_comments=True))
    return parse(str(path), parser=parser).getroot()


#: Resolved path -> ((mtime_ns, size), parsed URDF).
_URDFS: Dict[Path, Tuple[Tuple[int, int], URDF]] = {}
_URDFS_LOCK = Lock()


def load_urdf(urdf: Path) -> URDF:
    """A URDF parsed by yourdfpy, without meshes; parsed again only when the file changes.

    - ! Shared by every caller: never pose it (`update_cfg`) or change it. For forward kinematics use `kinematics`.
    - ? Checked as strict XML first: yourdfpy recovers from broken XML instead of failing.

    Raises:
        xml.etree.ElementTree.ParseError: If the file is not well-formed XML.
        URDFReadError: If yourdfpy cannot read it as a robot.
    """
    path = Path(urdf).resolve()
    status = os.stat(path)
    stamp = (status.st_mtime_ns, status.st_size)
    cached = _URDFS.get(path)
    if cached is not None and cached[0] == stamp:
        return cached[1]
    parse(str(path))
    try:
        model = URDF.load(str(path), build_scene_graph=False, build_collision_scene_graph=False,
                          load_meshes=False, load_collision_meshes=False)
    except Exception as error:  # noqa: BLE001  (yourdfpy raises AttributeError, KeyError, ... on a broken robot)
        raise URDFReadError(f"{path.name}: yourdfpy cannot read it ({type(error).__name__}: {error})") from error
    with _URDFS_LOCK:
        _URDFS[path] = (stamp, model)
    return model


def urdf_links(urdf: Path) -> set[str]:
    """Every link name of a URDF."""
    return set(load_urdf(urdf).link_map)


def urdf_joints(urdf: Path) -> dict[str, str]:
    """Joint name -> type ("revolute", "fixed", ...) of a URDF, in file order."""
    return {joint.name: joint.type for joint in load_urdf(urdf).robot.joints}


def movable_joints(urdf: Path, srdf: Path) -> tuple[str, ...]:
    """The joints a process state must list, in URDF order: all but fixed, SRDF-passive (wheels) and mimic ones.

    Args:
        urdf: The URDF file.
        srdf: The SRDF file of the same robot.
    """
    passive = {joint.get("name") for joint in _root(srdf).findall("passive_joint")}
    return tuple(joint.name for joint in load_urdf(urdf).robot.joints
                 if joint.type != "fixed" and joint.name not in passive and joint.mimic is None)


def srdf_group_tips(srdf: Path) -> dict[str, str]:
    """Group name -> tip link of every SRDF group made of a `<chain>`."""
    tips = {}
    for group in _root(srdf).findall("group"):
        chain = group.find("chain")
        if chain is not None and chain.get("tip_link"):
            tips[group.get("name")] = chain.get("tip_link")
    return tips


def mesh_references(urdf: Path) -> list[str]:
    """The `filename` of every `<mesh>` in a URDF, as written, link by link, visuals first (repeats included)."""
    return [element.geometry.mesh.filename or "" for link in load_urdf(urdf).robot.links
            for element in (*link.visuals, *link.collisions)
            if element.geometry is not None and element.geometry.mesh is not None]


def is_relative_reference(filename: str) -> bool:
    """Whether a `<mesh filename=...>` is a plain relative path (no `package://`, `file://`, or absolute path)."""
    return not (filename.startswith(PACKAGE_SCHEME) or filename.startswith(FILE_SCHEME)
                or Path(filename).is_absolute())


def resolved_urdf_text(urdf: Path) -> str:
    """The URDF text with relative mesh paths made absolute; other references are left as they are.

    ? compas_robots resolves plain paths against the working directory, so compas callers load this instead.
    """
    root = _root(urdf)
    folder = Path(urdf).resolve().parent
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename and is_relative_reference(filename):
            mesh.set("filename", str((folder / filename).resolve()))
    return tostring(root, encoding="unicode")


# --- --- --- --- --- COPYING --- --- --- --- ---

def _find_package(name: str, package_dirs: Sequence[Path]) -> Optional[Path]:
    """The folder of a ROS package: `<dir>/<name>`, or a dir that is the package itself; None if not found."""
    for folder in package_dirs:
        folder = Path(folder)
        if folder.name == name and folder.is_dir():
            return folder
        if (folder / name).is_dir():
            return folder / name
    return None


def _without_anchor(path: Path) -> str:
    """An absolute path as a relative one, e.g. "/opt/meshes/a.stl" -> "opt/meshes/a.stl"."""
    return PurePath(*path.parts[1:]).as_posix()


def _locate(filename: str, urdf_folder: Path, package_dirs: Sequence[Path]) -> Tuple[Path, str]:
    """Find a referenced mesh and choose its path below `meshes/`.

    The path is `<pkg>/<rest>` for `package://`, the relative path for files inside the URDF's folder,
    else the absolute path without its leading `/`.

    Args:
        filename: The reference as written in the URDF.
        urdf_folder: Folder of the source URDF.
        package_dirs: Folders to find packages in.

    Returns:
        tuple[Path, str]: The source file, and its path below `meshes/` (with `/`).

    Raises:
        FileNotFoundError: If the package or the file is missing.
    """
    source = resolve_mesh(filename, urdf_folder, package_dirs)
    if filename.startswith(PACKAGE_SCHEME):
        below = filename[len(PACKAGE_SCHEME):]
    else:
        # ? A process's own URDF already keeps its meshes under `meshes/`: keep that path, or a rewrite nests it again.
        for base in (urdf_folder.resolve() / "meshes", urdf_folder.resolve()):
            try:
                below = source.relative_to(base).as_posix()
                break
            except ValueError:
                continue
        else:
            below = _without_anchor(source)
    return source, below


def resolve_mesh(filename: str, urdf_folder: Path, package_dirs: Sequence[Path] = ()) -> Path:
    """The file a `<mesh filename=...>` names: `package://` in `package_dirs`, `file://`, absolute, or relative to
    the URDF's folder.

    Raises:
        FileNotFoundError: If the package or the file is missing.
    """
    if filename.startswith(PACKAGE_SCHEME):
        package, _, rest = filename[len(PACKAGE_SCHEME):].partition("/")
        package_dir = _find_package(package, package_dirs)
        if package_dir is None:
            raise FileNotFoundError(f"mesh {filename!r}: package {package!r} not found in {list(package_dirs)}")
        source = package_dir / rest
    else:
        path = Path(filename[len(FILE_SCHEME):] if filename.startswith(FILE_SCHEME) else filename)
        source = (path if path.is_absolute() else (Path(urdf_folder) / path)).resolve()
    if not source.is_file():
        raise FileNotFoundError(f"mesh {filename!r} not found (looked at {source})")
    return source


def _robot_copy(urdf: Path, srdf: Path, package_dirs: Sequence[Path]) -> Tuple[bytes, bytes, Dict[str, Path]]:
    """What `copy_robot` writes, in memory: the URDF with its mesh paths rewritten, the SRDF, and the meshes.

    Returns:
        tuple: URDF bytes, SRDF bytes, and each mesh's path below `meshes/` -> its source file.
    """
    urdf = Path(urdf)
    tree = ElementTree(_root(urdf))
    copies: Dict[str, Tuple[Path, str]] = {}
    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")
        if filename not in copies:
            copies[filename] = _locate(filename, urdf.parent, package_dirs)
        mesh.set("filename", f"meshes/{copies[filename][1]}")
    text = BytesIO()
    tree.write(text, encoding="utf-8", xml_declaration=True)
    return text.getvalue(), Path(srdf).read_bytes(), {below: source for source, below in copies.values()}


def copy_robot(urdf: Path, srdf: Path, dest_dir: Path, package_dirs: Sequence[Path] = ()) -> tuple[Path, Path]:
    """Copy a robot into a process: `robot.urdf`, `robot.srdf`, and every mesh under `meshes/`, named relatively.

    Args:
        urdf: Source URDF.
        srdf: Source SRDF.
        dest_dir: The robot's folder in the process, e.g. `<process>/robots/cindy`. Created if missing.
        package_dirs: Folders holding packages named by `package://`, or package folders themselves.

    Returns:
        tuple[Path, Path]: The copied URDF and SRDF.

    Raises:
        FileNotFoundError: If a referenced mesh (or its package) is missing.
    """
    dest_dir = Path(dest_dir)
    # ? Read everything before writing: the source may already be the destination.
    urdf_text, srdf_text, meshes = _robot_copy(urdf, srdf, package_dirs)
    dest_dir.mkdir(parents=True, exist_ok=True)
    for below, source in meshes.items():
        target = dest_dir / "meshes" / below
        if target.exists() and target.resolve() == source.resolve():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
    urdf_out, srdf_out = dest_dir / "robot.urdf", dest_dir / "robot.srdf"
    urdf_out.write_bytes(urdf_text)
    srdf_out.write_bytes(srdf_text)
    return urdf_out, srdf_out


#: (file stats of a robot's URDF, SRDF and meshes) -> its files hash; reading 40 MB of meshes takes a while.
_FILES_HASHES: Dict[tuple, str] = {}


def robot_files_hash(urdf: Path, srdf: Path, package_dirs: Sequence[Path] = ()) -> str:
    """The SHA-256 of a robot's files as `copy_robot` writes them: URDF, SRDF and every mesh with its path.

    The same for a source robot and for its copy in a process, so a process hashes the same in memory and on disk.

    Raises:
        FileNotFoundError: If a referenced mesh (or its package) is missing.
    """
    urdf_text, srdf_text, meshes = _robot_copy(urdf, srdf, package_dirs)
    stats = tuple((below, source.stat().st_mtime_ns, source.stat().st_size) for below, source in sorted(meshes.items()))
    key = (urdf_text, srdf_text, stats)
    if key not in _FILES_HASHES:
        digest = sha256(urdf_text + b"\0" + srdf_text)
        for below, source in sorted(meshes.items()):
            digest.update(b"\0" + below.encode("utf-8") + b"\0" + source.read_bytes())
        _FILES_HASHES[key] = digest.hexdigest()
    return _FILES_HASHES[key]


# --- --- --- --- --- LINK GEOMETRY --- --- --- --- ---

#: Subdivisions of the icosphere a URDF `<sphere>` becomes (the core has no sphere shape).
SPHERE_SUBDIVISIONS = 2

#: (urdf path, stamp, package dirs) -> (link -> Geometry, {mesh file: its stamp when read}).
_LINK_GEOMETRY: Dict[tuple, Tuple[Dict[str, Geometry], Dict[Path, Tuple[int, int]]]] = {}
#: (mesh path, (mtime_ns, size), scale, origin) -> TriMesh: robots sharing a mesh file share the object.
_MESHES: Dict[tuple, TriMesh] = {}


def link_geometry(urdf: Path, package_dirs: Sequence[Path] = ()) -> Dict[str, Geometry]:
    """Each link's shapes from its `<visual>` and `<collision>` elements, in the link frame, metres.

    Meshes are read with trimesh, scaled, and their `<origin>` applied to the vertices; boxes and cylinders keep
    theirs; spheres become icospheres. Links with neither element are left out.

    - ! The same objects while the URDF and its meshes are unchanged: mirrors and viewers cache per object.
    - ? Material colours are not read: `Geometry` has none.

    Args:
        urdf: The URDF.
        package_dirs: Folders holding packages named by `package://` (a process's URDF needs none).

    Returns:
        dict[str, Geometry]: Link name -> its shapes.

    Raises:
        FileNotFoundError: If a mesh (or its package) is missing.
        xml.etree.ElementTree.ParseError, URDFReadError: As `load_urdf`.
    """
    path = Path(urdf).resolve()
    model = load_urdf(path)
    key = (path, _URDFS[path][0], tuple(str(Path(d).resolve()) for d in package_dirs))
    found = _LINK_GEOMETRY.get(key)
    if found is not None and all(_stamp(file) == stamp for file, stamp in found[1].items()):
        return found[0]
    out: Dict[str, Geometry] = {}
    files: Dict[Path, Tuple[int, int]] = {}
    for link in model.robot.links:
        visual = tuple(_shape(e.geometry, e.origin, path.parent, package_dirs, files) for e in link.visuals
                       if e.geometry)
        collision = tuple(_shape(e.geometry, e.origin, path.parent, package_dirs, files) for e in link.collisions
                          if e.geometry)
        if visual or collision:
            out[link.name] = Geometry(visual, collision)
    _LINK_GEOMETRY[key] = (out, files)
    return out


def _stamp(file: Path) -> Optional[Tuple[int, int]]:
    """A file's (mtime_ns, size), or None if it is gone."""
    try:
        status = os.stat(file)
    except OSError:
        return None
    return status.st_mtime_ns, status.st_size


def _shape(geometry, origin: Optional[np.ndarray], urdf_folder: Path, package_dirs: Sequence[Path],
           files: Dict[Path, Tuple[int, int]]) -> Shape:
    """One URDF `<geometry>` (yourdfpy's) as a core shape, its `<origin>` (4x4 or None) applied; mesh files read
    are noted in `files` with their stamps."""
    pose = Pose() if origin is None else Pose.from_matrix(origin)
    if geometry.box is not None:
        return BoxShape(tuple(float(v) for v in geometry.box.size), pose)
    if geometry.cylinder is not None:
        return CylinderShape(float(geometry.cylinder.radius), float(geometry.cylinder.length), pose)
    if geometry.sphere is not None:
        sphere = icosphere(subdivisions=SPHERE_SUBDIVISIONS, radius=float(geometry.sphere.radius))
        sphere.apply_transform(pose.matrix())
        return TriMesh.from_arrays(sphere.vertices, sphere.faces)
    if geometry.mesh is None:
        raise URDFReadError("a <geometry> without box, cylinder, sphere or mesh")
    source = resolve_mesh(geometry.mesh.filename, urdf_folder, package_dirs)
    stamp = files[source] = _stamp(source)
    scale = geometry.mesh.scale
    scale = (1.0, 1.0, 1.0) if scale is None else tuple(float(v) for v in np.broadcast_to(scale, 3))
    matrix = None if origin is None else np.asarray(origin, dtype=float)
    key = (source, stamp, scale, None if matrix is None else matrix.tobytes())
    mesh = _MESHES.get(key)
    if mesh is None:
        plain = read_mesh(source)
        vertices = plain.vertices * np.asarray(scale)
        if matrix is not None:
            vertices = vertices @ matrix[:3, :3].T + matrix[:3, 3]
        mesh = _MESHES[key] = TriMesh.from_arrays(vertices, plain.faces)
    return mesh


# --- --- --- --- --- UR ARMS --- --- --- --- ---

#: Joint names in the UR driver's order. The URDF has the same names with the arm's prefix ("left_ur_arm_").
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

#: Links of its arm ("<arm>_<suffix>") a mounted tool may touch. The SRDFs don't list these pairs,
#: so collision checkers add them themselves.
TOOL_TOUCHES_ARM_LINKS = ("wrist_2_link", "wrist_3_link", "flange", "tool0")

#: Stock ur_description turns both joints below `<arm>_base_link` 180 deg about z.
STOCK_YAW = np.pi


def stock_frame_problem(urdf_file: Path, arm_name: str) -> Optional[str]:
    """Check the two joints below an arm's base_link are the stock ones.

    Args:
        urdf_file: The robot's URDF.
        arm_name: The arm's prefix, e.g. "ur_arm".

    Returns:
        str | None: None if stock, otherwise what is wrong and how to fix it.
    """
    wrong = []
    for joint in load_urdf(urdf_file).robot.joints:
        if joint.parent != f"{arm_name}_base_link":
            continue
        if joint.child not in (f"{arm_name}_base_link_inertia", f"{arm_name}_base"):
            continue
        origin = np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=float)
        rotation = Rotation.from_matrix(origin[:3, :3])
        turn = (Rotation.from_euler("z", STOCK_YAW).inv() * rotation).magnitude()
        if np.linalg.norm(origin[:3, 3]) > 1e-6 or turn > 1e-6:
            rpy = " ".join(f"{v:.6g}" for v in rotation.as_euler("xyz"))
            wrong.append(f"{joint.name} (rpy {rpy})")
    if not wrong:
        return None
    return (f"URDF {Path(urdf_file).name}, arm {arm_name}: joints below {arm_name}_base_link are not the stock UR "
            f"ones (xyz 0 0 0, rpy 0 0 pi): {', '.join(wrong)}. So {arm_name}_base_link is not the robot "
            f"controller's base_link. The arm's mounting belongs in the joint above it. "
            f"Fix with scripts/fix_ur_base_frames.py; see doc/ur_frames.md.")


# --- --- --- --- --- UR FACTORY CALIBRATION --- --- --- --- ---

#: Section of a UR calibration (the `kinematics` of the driver's calibration.yaml) -> the arm joint whose origin it
#: is, as ur_description's ur_macro.xacro builds the URDF from it.
UR_CALIBRATION_JOINTS = {"shoulder": "shoulder_pan_joint", "upper_arm": "shoulder_lift_joint",
                         "forearm": "elbow_joint", "wrist_1": "wrist_1_joint", "wrist_2": "wrist_2_joint",
                         "wrist_3": "wrist_3_joint"}
#: Origin differences below these count as the same: metres, radians. Far below what a calibration changes.
UR_CALIBRATION_POSITION_TOLERANCE = 1e-5
UR_CALIBRATION_ANGLE_TOLERANCE = 1e-5


@dataclass(frozen=True)
class JointOriginChange:
    """How far one joint origin moved when a calibration was applied.

    Attributes:
        joint: The joint's name, e.g. "left_ur_arm_shoulder_pan_joint".
        distance: Change of its origin position, metres.
        angle: Change of its origin orientation, radians.
    """

    joint: str
    distance: float
    angle: float


def apply_ur_calibration(urdf_text: str, arm_name: str,
                         calibration: Mapping) -> Tuple[str, List[JointOriginChange]]:
    """Set an arm's six joint origins to a UR factory calibration; a URDF that already has it comes back unchanged.

    Only origins that differ beyond the tolerances are rewritten, so applying the calibration a URDF was built with
    changes nothing, and the returned changes say how out of date a URDF is.

    Args:
        urdf_text: The robot's URDF.
        arm_name: The arm's prefix, e.g. "ur_arm".
        calibration: The parsed calibration file: `{"kinematics": {...}}` or its `kinematics` section, with x, y, z,
            roll, pitch, yaw per section of UR_CALIBRATION_JOINTS.

    Returns:
        tuple[str, list[JointOriginChange]]: The URDF text, and each joint whose origin changed, with how much.

    Raises:
        KeyError: If a section or value is missing from the calibration, or an arm joint from the URDF.
    """
    kinematics = calibration.get("kinematics", calibration)
    changes = []
    for section, joint_suffix in UR_CALIBRATION_JOINTS.items():
        values = kinematics[section]
        position = np.array([float(values[k]) for k in ("x", "y", "z")])
        rpy = np.array([float(values[k]) for k in ("roll", "pitch", "yaw")])
        joint = f"{arm_name}_{joint_suffix}"
        # ! Comments are blanked before searching: the files keep old origins, and whole transmission blocks naming
        #   the same joints, commented out. Blanking keeps every position, so edits apply to the real text.
        live_text = re.sub(r"<!--.*?-->", lambda m: " " * len(m.group(0)), urdf_text, flags=re.S)
        block = re.search(r'<joint\s+name="' + re.escape(joint) + r'".*?</joint>', live_text, re.S)
        if block is None:
            raise KeyError(f"no joint {joint} in the URDF")
        body = urdf_text[block.start():block.end()]
        tag = re.search(r"<origin\b[^>]*/>", live_text[block.start():block.end()])
        old_position, old_rpy = np.zeros(3), np.zeros(3)
        if tag is not None:
            for name, target in (("xyz", old_position), ("rpy", old_rpy)):
                found = re.search(name + r'="([^"]*)"', tag.group(0))
                if found:
                    target[:] = [float(v) for v in found.group(1).split()]
        distance = float(np.linalg.norm(position - old_position))
        angle = float((Rotation.from_euler("xyz", old_rpy).inv() * Rotation.from_euler("xyz", rpy)).magnitude())
        if distance <= UR_CALIBRATION_POSITION_TOLERANCE and angle <= UR_CALIBRATION_ANGLE_TOLERANCE:
            continue
        new_tag = (f'<origin rpy="{" ".join(repr(float(v)) for v in rpy)}" '
                   f'xyz="{" ".join(repr(float(v)) for v in position)}" />')
        body = (body[:tag.start()] + new_tag + body[tag.end():] if tag is not None
                else body.replace("</joint>", f"  {new_tag}\n    </joint>"))
        urdf_text = urdf_text[:block.start()] + body + urdf_text[block.end():]
        changes.append(JointOriginChange(joint, distance, angle))
    return urdf_text, changes
