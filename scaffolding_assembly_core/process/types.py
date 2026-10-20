"""
The process in memory, schema 2.1: one frozen dataclass per object of doc/process_format.md, fields named as the file keys.

! Never change the dicts inside after construction; build a new object with `dataclasses.replace`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Dict, FrozenSet, Literal, Mapping, Optional, Tuple, Union

from ..geometry import Geometry, Pose
from ..ids import ROBOTS, TOOLS
from ..robot import Tool

if TYPE_CHECKING:
    from ..scene import Scene

ActionType = Literal["bar_jointing", "bar_release", "bar_holding", "bar_holding_release", "bar_placing"]
Controller = Literal["position", "compliant"]
EndsOn = Literal["target", "tools", "operator"]

ACTION_TYPES: Tuple[str, ...] = ("bar_jointing", "bar_release", "bar_holding", "bar_holding_release", "bar_placing")
#: The action type no robot acts in: the bar is put into the structure by hand, at its place in the schedule.
BY_HAND: str = "bar_placing"
PATHS: Tuple[str, ...] = ("free", "linear")
CONTROLLERS: Tuple[str, ...] = ("position", "compliant")
ENDS_ON: Tuple[str, ...] = ("target", "tools", "operator")

#: Body id prefixes (format §3.1).
BAR_PREFIX, HALF_PREFIX, GROUND_PREFIX, OBSTACLE_PREFIX = "bars/", "joints/", "ground/", "obstacles/"
BODY_PREFIXES: Tuple[str, ...] = (BAR_PREFIX, HALF_PREFIX, GROUND_PREFIX, OBSTACLE_PREFIX)
ROBOT_PREFIX, TOOL_PREFIX = f"{ROBOTS}/", f"{TOOLS}/"

#: A note value: flat only.
Note = Union[str, int, float, bool]


# --- --- --- --- --- ERRORS --- --- --- --- ---

class ProcessError(ValueError):
    """A process breaks the format. `problems` lists every broken rule found, one line each."""

    def __init__(self, problems):
        """Collect `problems`, one string per broken rule."""
        self.problems = list(problems)
        super().__init__("invalid process:\n  " + "\n  ".join(self.problems))


class SchemaMismatch(ProcessError):
    """A file was written with another schema. Read it with the library at `commit`."""

    def __init__(self, path: str, schema: float, commit: str, expected: float):
        """Name the file, its schema and the commit that wrote it."""
        self.schema, self.commit = schema, commit
        super().__init__([f"{path} has schema {schema}, this library reads schema {expected}; "
                          f"check out scaffolding_assembly_core at commit {commit} to read it"])


# --- --- --- --- --- PROCESS-LEVEL OBJECTS --- --- --- --- ---

@dataclass(frozen=True)
class Writer:
    """Which library wrote a file (format §9)."""

    schema: float
    library: str
    commit: str
    dirty: bool


@dataclass(frozen=True)
class Producer:
    """Which code made the process (format §9), e.g. Rhino's `RSExportProcess`."""

    repo: str
    commit: str
    dirty: bool
    command: str


@dataclass(frozen=True)
class RobotSpec:
    """A robot (format §4.1). `urdf`/`srdf` are absolute once read.

    Attributes:
        tools: Flange link name -> tool id.
        ground_links: Links the robot stands on (its wheels): they may touch any `ground/` body.
    """

    id: str
    urdf: Path
    srdf: Path
    serial: Optional[str]
    tools: Dict[str, str]
    ground_links: Tuple[str, ...] = ()

    @property
    def name(self) -> str:
        """str: The id without its prefix, e.g. "cindy"."""
        return self.id[len(ROBOT_PREFIX):]


@dataclass(frozen=True)
class BodySpec:
    """A body (format §4.3). `assembly_pose` is where it stands in the finished assembly, in world; geometry and markers are in the body frame.

    Attributes:
        part: What kind of half it is, e.g. "T20/Female": a label, for people and mocap; empty if none.
        markers: Marker label -> (x, y, z), metres, for placement checks and mocap registration.
        mount: A connector half's bar: the half is fixed to it for the whole process and follows it. None otherwise.
        pose_in_tcp: A connector half a tool acts on: the half's pose in the frame of the tool centre point on it
            (identity when the two frames coincide). None: no tool may be put on it. B14 checks grasps against it.
    """

    id: str
    assembly_pose: Pose
    geometry: Geometry
    label: str = ""
    part: str = ""
    markers: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    mount: Optional[str] = None
    pose_in_tcp: Optional[Pose] = None


# --- --- --- --- --- STATES --- --- --- --- ---

@dataclass(frozen=True)
class RobotState:
    """A robot at one moment; `base` and `joints` are None where the process does not decide them."""

    base: Optional[Pose]
    joints: Optional[Dict[str, float]]


@dataclass(frozen=True)
class Holder:
    """A robot link holding a bar: `to` is "robots/<robot>/<link>", `grasp` the bar pose in that link's frame."""

    to: str
    grasp: Pose


@dataclass(frozen=True)
class ToolState:
    """A tool at one moment: its grip ("open", "closed", or None: not decided) and the body it is on, if any."""

    grip: Optional[str] = None
    on: Optional[str] = None


@dataclass(frozen=True)
class State:
    """One complete moment (format §5.2): readable on its own, together with the process.

    Attributes:
        robots: Every robot; None = not in the scene.
        present: Bars and ground in the scene; halves are present with their bar, obstacles always.
        attached: Bar -> the robot links holding it, the first one setting its pose while it is not built.
        built: Bars fixed in the structure: they take their assembly pose.
        poses: World poses of present bars that are neither built nor attached, where away from the assembly pose.
        tools: Every mounted tool's state; None = its robot is absent.
    """

    robots: Dict[str, Optional[RobotState]]
    present: FrozenSet[str]
    attached: Dict[str, Tuple[Holder, ...]] = field(default_factory=dict)
    built: FrozenSet[str] = frozenset()
    poses: Dict[str, Pose] = field(default_factory=dict)
    tools: Dict[str, Optional[ToolState]] = field(default_factory=dict)


@dataclass(frozen=True)
class LineSpec:
    """A linear path of one flange: `direction` (unit, world frame) and `distance` (metres)."""

    direction: Tuple[float, float, float]
    distance: float


@dataclass(frozen=True)
class RobotTarget:
    """What a target says of one robot (format §5.3): the keys of a `RobotState`, only those the movement sets.

    Attributes:
        base: Where the base ends; None: it does not move (only a movement listing the robot in `base` gives one).
        joints: Joint -> value at the end; a subset of joints is allowed.
        link_poses: Link id ("robots/cindy/left_ur_arm_tool0") -> world pose at the end, e.g. of a flange.
    """

    base: Optional[Pose] = None
    joints: Dict[str, float] = field(default_factory=dict)
    link_poses: Dict[str, Pose] = field(default_factory=dict)


@dataclass(frozen=True)
class Target:
    """Where a movement should end (format §5.3): a partial `State`, holding only what the movement decides.

    The state at a movement's end is its start with the target laid over it (`plan_check.end_state`).

    Attributes:
        robots: Robot id -> its base, joints and link poses at the end; only the robots the movement moves.
        tools: Tool id -> the grip it is commanded to, only for the tools the movement commands.
        on: Tool id -> the body it is on at the end (None: on nothing), only for the tools whose `on` changes.
        attached: The whole `attached` map at the end, if the movement changes it; None: unchanged.
        built: The whole `built` set at the end, if the movement changes it; None: unchanged.
    """

    robots: Dict[str, RobotTarget] = field(default_factory=dict)
    tools: Dict[str, str] = field(default_factory=dict)
    on: Dict[str, Optional[str]] = field(default_factory=dict)
    attached: Optional[Dict[str, Tuple[Holder, ...]]] = None
    built: Optional[FrozenSet[str]] = None

    @classmethod
    def of(cls, joints: Optional[Mapping[str, Mapping[str, float]]] = None,
           link_poses: Optional[Mapping[str, Pose]] = None, bases: Optional[Mapping[str, Pose]] = None, **rest) -> Target:
        """A target from maps over all robots: robot id -> joints, link id -> pose, robot id -> base.

        Args:
            joints: Robot id -> joint -> value.
            link_poses: Link id -> world pose; each goes to the robot its id names.
            bases: Robot id -> base pose.
            **rest: The other fields (`tools`, `on`, `attached`, `built`).
        """
        joints, link_poses, bases = dict(joints or {}), dict(link_poses or {}), dict(bases or {})
        of_link = {link: "/".join(link.split("/")[:2]) for link in link_poses}
        robots = {robot: RobotTarget(base=bases.get(robot), joints=dict(joints.get(robot, {})),
                                     link_poses={link: pose for link, pose in link_poses.items()
                                                 if of_link[link] == robot})
                  for robot in dict.fromkeys([*joints, *of_link.values(), *bases])}
        return cls(robots=robots, **rest)

    def having(self, joints=None, link_poses=None, bases=None) -> Target:
        """A copy with the given maps (as for `of`) in place of its own; None keeps what it has."""
        return Target.of(self.joints if joints is None else joints,
                         self.link_poses if link_poses is None else link_poses,
                         self.bases if bases is None else bases, tools=self.tools, on=self.on,
                         attached=self.attached, built=self.built)

    @property
    def joints(self) -> Dict[str, Dict[str, float]]:
        """dict: Robot id -> joint -> value, for the robots whose joints the target gives."""
        return {robot: dict(part.joints) for robot, part in self.robots.items() if part.joints}

    @property
    def link_poses(self) -> Dict[str, Pose]:
        """dict: Link id -> world pose, over every robot."""
        return {link: pose for part in self.robots.values() for link, pose in part.link_poses.items()}

    @property
    def bases(self) -> Dict[str, Pose]:
        """dict: Robot id -> base pose, for the robots whose base the movement moves."""
        return {robot: part.base for robot, part in self.robots.items() if part.base is not None}


@dataclass(frozen=True)
class Movement:
    """One movement (format §5.1): a segment that runs without stopping, described by independent parts.

    Attributes:
        arms: Flange link ids that move; empty: no arm moves.
        base: Robot ids whose base drives during the movement; empty: none. ! No planner or path type handles a
            moving base yet: the format allows it, planners report it as not supported.
        path: "free" or "linear", when arms move.
        coupled: The arms keep their relative pose (they hold one bar).
        controller: "position" or "compliant", when arms move.
        line: Per moving flange of a linear path: direction and distance.
        drives: Tool id -> "tighten" or "loosen": the jointing screw runs during the movement.
        ends_on: "target" (arms reached it, tools done), "tools" (tools done) or "operator" (confirmed).
        target: None: end where the next movement starts.
        notes: For people only; flat values. No program reads them.
    """

    id: str
    start: State
    arms: Tuple[str, ...] = ()
    base: Tuple[str, ...] = ()
    path: Optional[str] = None
    coupled: bool = False
    controller: Optional[str] = None
    line: Dict[str, LineSpec] = field(default_factory=dict)
    drives: Dict[str, str] = field(default_factory=dict)
    ends_on: str = "target"
    target: Optional[Target] = None
    label: str = ""
    notes: Dict[str, Note] = field(default_factory=dict)

    @property
    def grip_change(self) -> Dict[str, str]:
        """dict: Tool id -> the grip it ends with, for the tools this movement commands."""
        return dict(self.target.tools) if self.target is not None else {}

    @property
    def has_tool_part(self) -> bool:
        """bool: Whether the movement changes a grip or runs a drive."""
        return bool(self.grip_change or self.drives)


@dataclass(frozen=True)
class Action:
    """One scheduled action (format §5): one robot's movements on one bar.

    A `bar_placing` action has no robot (`robot` is ""): one manual step that ends with the bar built.
    """

    id: str
    type: ActionType
    robot: str
    bar: str
    movements: Tuple[Movement, ...]
    ground: Tuple[str, ...] = ()
    supports_until: Tuple[str, ...] = ()
    label: str = ""
    notes: Dict[str, Note] = field(default_factory=dict)


@dataclass(frozen=True)
class Process:
    """A whole process folder (format §2).

    Attributes:
        folder: Where it was read from or written to; None for a process only in memory.
        actions: Keyed by action id; `schedule` gives their order.
        mates: Joints of the finished structure, each pair sorted: a male and a female half, or a ground half and a
            ground body. They never move anything; their status follows from the state (`relations.mate_status`).
        producer: The code that made the process, or None.
    """

    folder: Optional[Path]
    writer: Writer
    robots: Dict[str, RobotSpec]
    tools: Dict[str, Tool]
    bodies: Dict[str, BodySpec]
    schedule: Tuple[str, ...]
    actions: Dict[str, Action]
    mates: FrozenSet[Tuple[str, str]] = frozenset()
    producer: Optional[Producer] = None

    def halves_of(self, bar: str) -> Tuple[str, ...]:
        """The connector halves mounted on a bar, sorted."""
        return tuple(sorted(body_id for body_id, body in self.bodies.items() if body.mount == bar))

    def movements(self):
        """Every (action, movement) in execution order.

        Yields:
            tuple[Action, Movement]: Each movement with the action it belongs to.
        """
        for action_id in self.schedule:
            action = self.actions[action_id]
            for movement in action.movements:
                yield action, movement

    def find_movement(self, action_id: str, movement_id: str) -> Optional[Movement]:
        """A movement by its action and movement ids, or None if there is no such action or movement."""
        action = self.actions.get(action_id)
        if action is None:
            return None
        return next((movement for movement in action.movements if movement.id == movement_id), None)

    def movement_after(self, movement: Movement) -> Optional[Movement]:
        """The movement after `movement` in the schedule, or None after the last or for one not in the process.

        ! Matched by identity, not equality: pass a movement taken from this process, not a copy.
        """
        found = False
        for _action, candidate in self.movements():
            if found:
                return candidate
            found = candidate is movement
        return None

    def scene_at(self, movement: Movement, at_end: bool = False) -> Scene:
        """The world at the start of one movement, or with `at_end` its target joints applied (`scenes.scene_at`).

        It shares nothing mutable with the process. ! Run it when the process or the movement changes, not every tick.
        """
        # ? Imported here: the scene and its robots stay out of `import process`.
        from .scenes import scene_at
        return scene_at(self, movement, at_end)

    def scene_after(self, bar: str) -> Scene:
        """The world once a bar is built, following the schedule (`scenes.scene_after`)."""
        from .scenes import scene_after
        return scene_after(self, bar)
