"""
A compas_fab `RobotCell` for one acting robot, built from scenes, and its states: no PyBullet.

The acting robot is the cell's robot, each of its mounted tools a `ToolModel` attached to the SRDF group ending at
its flange. Every other robot is one `ToolModel` keyed by its robot id, parked far away while absent; bodies are
`RigidBody`s keyed by our id, attached when held by the acting robot. A contact a body may have with any link or
mounted tool of another robot allows that whole robot: compas_fab allows only whole tools. `CompasFabMirror` plans
on such a cell; a viewer draws one built with `visual=True`.

- ! `update` rebuilds the cell only when a model, body id or geometry object changed (`scene.same_source`).
- ! compas_fab builds each collision mesh as its convex hull.
- ? `visual=False` loads no visual shapes (robots) and uses the collision shapes as visuals (bodies): faster, and a
  PyBullet window shows what is checked. `visual=True` is for drawing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, Mapping, Optional, Tuple, Union

from compas.geometry import Frame
from compas_fab.robots import RigidBody, RigidBodyState, RobotCell, RobotCellState, RobotSemantics, ToolState
from compas_robots import ToolModel

from ..geometry import Pose
from ..ids import ROBOTS
from ..robot import RobotModel
from ..scene import Attachment, Scene, same_source
from .compas import PARKED_POSITION, filled, frame_from_pose, load_model, planning_group, rigid_body, robot_as_tool, \
    tool_model

if TYPE_CHECKING:
    from compas_robots import RobotModel as CompasRobotModel
    from compas_robots.model import Link

    from ..geometry import Geometry


@dataclass(eq=False)
class _Acting:
    """The acting robot as compas_fab objects, built once per `RobotModel`.

    Attributes:
        source: The model they were built from.
        model: Its URDF (with visual shapes if the cell is visual).
        semantics: Its SRDF.
        tools: Tool id -> its compas_fab tool, one per mounted tool.
        flanges: Tool id -> the flange it is mounted on.
    """

    source: RobotModel
    model: CompasRobotModel
    semantics: RobotSemantics
    tools: Dict[str, ToolModel]
    flanges: Dict[str, str]


class CompasFabCell:
    """The compas_fab cell of one acting robot, following the scenes given to `update`, and their states."""

    def __init__(self, robot_id: str, visual: bool = False) -> None:
        """An empty cell; the first `update` loads everything (slow: seconds per robot).

        Args:
            robot_id: The acting robot, e.g. "robots/cindy"; every other robot becomes a tool.
            visual: Load the robots' visual shapes and use the bodies' visual shapes: for drawing.
        """
        self.robot_id = robot_id
        self.visual = visual
        #: The cell as last built (None before the first `update`). A new object on every rebuild.
        self.cell: RobotCell | None = None
        # * Cached for this object's lifetime: loading and converting is the slow part.
        self._acting: _Acting | None = None
        # (our id, model) of other robots -> their tool models.
        self._others: Dict[Tuple[str, RobotModel], ToolModel] = {}
        # Geometry -> rigid body. ? Keyed by the object itself (identity), never `id(obj)`.
        self._bodies: Dict[Geometry, RigidBody] = {}
        # What the cell was built from: acting model, other models by id, geometry by id.
        self._built: tuple | None = None
        #: Tool id -> SRDF group its tool is attached to, for the cell as built.
        self.groups: Dict[str, str] = {}
        #: Other robots in the scene but parked at the last `state`, because their base or joints are unknown.
        self.parked: Tuple[str, ...] = ()
        # id() of each tool and rigid body model in the cell -> our id. ? Safe: the cell keeps them alive.
        self._names: Dict[int, str] = {}

    # --- --- --- --- --- BUILD --- --- --- --- ---

    def update(self, scene: Scene) -> Optional[str]:
        """Rebuild the cell if the scene's models, body ids or geometry differ from what it was built from.

        Args:
            scene: The world; it is not modified.

        Returns:
            str | None: Why the cell was rebuilt (e.g. "first build"), or None if it was kept.

        Raises:
            KeyError: If the acting robot is not in the scene.
            ValueError: If the acting robot has no SRDF.
        """
        built = self._sources(scene)
        if self._same(built):
            return None
        why = self._rebuild_reason(built)
        self._build(built)
        return why

    def forget(self) -> None:
        """Count the cell as unknown: the next `update` rebuilds it (models already loaded are kept)."""
        self._built = None

    def _sources(self, scene: Scene) -> tuple:
        """(acting model, other models by id, geometry by id) of a scene: what a cell is built from."""
        acting = scene.robots[self.robot_id]
        others = {key: robot.model for key, robot in scene.robots.items() if key != self.robot_id}
        # ? Disabled bodies are built too, but hidden: switching them on and off never rebuilds the cell.
        geometries = {key: body.geometry for key, body in scene.bodies.items() if body.geometry.collision}
        return acting.model, others, geometries

    def _same(self, built: tuple) -> bool:
        """Whether the cell was built from these models: the same model and geometry objects, by `same_source`."""
        if self._built is None:
            return False
        acting, others, geometries = built
        old_acting, old_others, old_geometries = self._built
        return same_source(acting, old_acting) and all(
            new.keys() == old.keys() and all(same_source(value, old[key]) for key, value in new.items())
            for new, old in ((others, old_others), (geometries, old_geometries)))

    def _rebuild_reason(self, built: tuple) -> str:
        """Why the cell must be rebuilt for `built`, in the terms `_same` compares; for logs."""
        if self._built is None:
            return "first build"
        acting, others, geometries = built
        old_acting, old_others, old_geometries = self._built
        reasons = [] if same_source(acting, old_acting) else ["the acting robot's model changed"]
        reasons += _changes("robots", others, old_others)
        reasons += _changes("bodies", geometries, old_geometries)
        return "; ".join(reasons)

    def _build(self, built: tuple) -> None:
        """Build a new cell. Slow the first time: loads models.

        Raises:
            ValueError: If the acting robot has no SRDF.
        """
        acting, others, geometries = built
        if self._acting is None or not same_source(acting, self._acting.source):
            if acting.srdf is None:
                raise ValueError(f"{self.robot_id} has no SRDF (RobotModel.srdf); compas_fab needs one")
            model = load_model(acting.urdf, visual=self.visual)
            self._acting = _Acting(acting, model, RobotSemantics.from_srdf_file(str(acting.srdf), model),
                                   {tool.id: tool_model(tool, tool.id) for tool in acting.tools.values()},
                                   {tool.id: flange for flange, tool in acting.tools.items()})

        self._others = {(key, model): self._others.get((key, model))
                        or robot_as_tool(model.urdf, model.tools, key, visual=self.visual)
                        for key, model in others.items()}
        self._bodies = {geometry: self._bodies.get(geometry) or rigid_body(geometry, draw_visual=self.visual)
                        for geometry in geometries.values()}
        tools = {**self._acting.tools, **{key: self._others[key, model] for key, model in others.items()}}
        rigid_bodies = {key: self._bodies[geometry] for key, geometry in geometries.items()}

        self.cell = RobotCell(self._acting.model, self._acting.semantics, tool_models=tools,
                              rigid_body_models=rigid_bodies)
        self._built = built
        self.groups = {key: planning_group(self.cell, flange) for key, flange in self._acting.flanges.items()}
        self._names = {id(value): key for key, value in [*tools.items(), *rigid_bodies.items()]}

    # --- --- --- --- --- STATES --- --- --- --- ---

    def state(self, scene: Scene, strict: bool = True) -> RobotCellState:
        """The scene as a state of the current cell (call `update` with it first).

        Args:
            scene: The world; it is not modified.
            strict: Planning: the acting robot must be present with a tracked base and known joints. Not strict
                (drawing): an acting robot that is absent or whose base is not tracked is parked far away, and its
                unknown joints are 0.

        Every other robot that is absent, or whose base or joints are unknown, is parked far away instead of standing
        at a guess; `parked` lists the ones in the scene parked for that reason.

        Raises:
            ValueError: If `strict` and the acting robot can't be planned for (`RobotObject.acting_problems`).
        """
        acting = scene.robots[self.robot_id]
        problems = acting.acting_problems()
        if strict and problems:
            raise ValueError(f"cannot plan for {self.robot_id}: {'; '.join(problems)}")
        model = acting.model
        tool_states = {key: ToolState(frame=None, attached_to_group=self.groups[key],
                                      touch_links=list(model.tool_touches.get(self._acting.flanges[key], ())),
                                      attachment_frame=Frame.worldXY())
                       for key in self._acting.tools}
        parked_others = []
        for key, robot in scene.robots.items():
            if key == self.robot_id:
                continue
            zero = self.cell.tool_models[key].zero_configuration()
            # ? compas_fab needs every tool in every state: an absent robot, or one whose base or joints are unknown,
            #   is parked far away instead of standing at a guess.
            if robot.enabled and robot.base_tracked and not robot.unmeasured:
                tool_states[key] = ToolState(frame=frame_from_pose(robot.base), configuration=filled(zero, robot.joints))
            else:
                tool_states[key] = ToolState(frame=Frame(PARKED_POSITION), configuration=zero)
                if robot.enabled:
                    parked_others.append(key)
        #: Other robots in the last scene given to `state`, parked because their base or joints are unknown.
        self.parked = tuple(parked_others)
        # ? Strict: a body without a world pose (attached to a missing parent) is an error (KeyError).
        bodies = {key: (scene.world_poses[key] if strict else scene.world_poses.get(key), body.placement,
                        body.touches, body.enabled)
                  for key, body in scene.bodies.items() if body.geometry.collision}
        # * Another robot is one tool here, its mounted tools included: a contact with one of them is with that robot.
        owners = {tool.id: key for key, robot in scene.robots.items() if key != self.robot_id
                  for tool in robot.model.tools.values()}
        parked = not strict and (not acting.enabled or not acting.base_tracked)
        return RobotCellState(
            robot_base_frame=Frame(PARKED_POSITION) if parked else frame_from_pose(acting.base),
            robot_configuration=filled(self.cell.zero_full_configuration(), acting.joints),
            tool_states=tool_states,
            rigid_body_states={key: self._body_state(*placed, owners) for key, placed in bodies.items()})

    @property
    def acting_tools(self) -> Dict[str, str]:
        """Tool id -> flange link of each tool mounted on the acting robot, as built."""
        return dict(self._acting.flanges) if self._acting is not None else {}

    def _body_state(self, pose: Optional[Pose], placement: Union[Pose, Attachment], touches: Tuple[str, ...],
                    enabled: bool, owners: Mapping[str, str]) -> RigidBodyState:
        """A body's compas_fab state: attached to one of our links, or stationary at its world pose.

        Args:
            pose: Its world pose; None if it has none (attached to a missing parent): then hidden.
            placement: As in the scene.
            touches: As in the scene.
            enabled: As in the scene; a disabled body is hidden, so nothing collides with it.
            owners: Tool id -> robot id of each other robot's mounted tools.
        """
        own = self.robot_id
        touch_links, touch_bodies = [], []
        for entry in touches:
            if entry.startswith(f"{own}/"):
                touch_links.append(entry.split("/", 2)[2])
            elif entry == own:
                # The whole robot: every link and every mounted tool.
                touch_links.extend(link.name for link in self.cell.robot_model.links)
                touch_bodies.extend(self._acting.tools)
            elif entry.startswith(f"{ROBOTS}/"):
                # Another robot is one tool: any of its links means the whole robot.
                touch_bodies.append("/".join(entry.split("/", 2)[:2]))
            elif entry in owners:
                touch_bodies.append(owners[entry])
            else:
                touch_bodies.append(entry)

        state = RigidBodyState(frame=frame_from_pose(pose if pose is not None else Pose()), touch_links=touch_links,
                               touch_bodies=touch_bodies, is_hidden=not enabled or pose is None)
        if isinstance(placement, Attachment):
            if placement.parent == own and placement.link is not None:
                state.attached_to_link = placement.link
                state.attachment_frame = frame_from_pose(placement.offset)
            elif placement.parent != own:
                # Held by another robot or fixed to a body: stationary here, but it may touch its holder.
                state.touch_bodies.append(placement.parent)
            # ? Held by our base: stationary, since the base does not move while an arm plans.
        return state

    # --- --- --- --- --- NAMES --- --- --- --- ---

    def our_id(self, model: Link | ToolModel | RigidBody) -> str:
        """Our id of one side of a compas_fab collision pair: a body or tool id, or "<robot id>/<link>"."""
        name = self._names.get(id(model))
        return name if name is not None else f"{self.robot_id}/{model.name}"


def _changes(kind: str, new: Mapping, old: Mapping) -> list[str]:
    """What differs between two maps of our id -> model, for the rebuild log.

    Args:
        kind: What the ids are, e.g. "bodies".
        new: The models now.
        old: The models the cell was built from.

    Returns:
        list[str]: Up to three parts, e.g. "bodies added: bars/3, bars/4".
    """
    parts = []
    for label, ids in (("added", new.keys() - old.keys()), ("removed", old.keys() - new.keys()),
                       ("changed", {key for key in new.keys() & old.keys() if not same_source(new[key], old[key])})):
        if ids:
            ids = sorted(ids)
            shown = ", ".join(ids[:5]) + (f" and {len(ids) - 5} more" if len(ids) > 5 else "")
            parts.append(f"{kind} {label}: {shown}")
    return parts
