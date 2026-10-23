"""
A compas_fab planning world for one acting robot, filled from a `Scene`.

The cell and its states come from `cell.CompasFabCell` (acting robot with its tools, other robots as tools, bodies
by our id), built without visual shapes. Other robots are parked far away while absent or while their base or joints
are unknown (`parked`). `sync` rebuilds the cell (~2 s) only when a model, body id or geometry
object changed. `collisions` is compas_fab's own check plus tool against tool (CC.6, `tool_pairs`); `search_check`
is a fast copy of both for searches. `lend` hands the planner, CC.6 included, to other code, such as tamp.
`set_gui(True)` shows the world in PyBullet's own window (one per process) for debugging.

- ! One thread only: create, sync, query and lend a mirror on the same thread.
- ! Every contact is a collision unless the scene allows it (`touches`, a holder), between other robots and bodies too.
- ! compas_fab allows only whole tools, and another robot is one tool: a contact allowed with any link or tool of
  another robot allows that whole robot. Such pairs never move while this robot plans: only the start check is coarse.
- ! compas_fab builds each collision mesh as its convex hull.
- ! compas_fab only skips a hidden body in its own checks and leaves it where it last stood: `sync` moves it to
  HIDDEN_POSITION.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass
from itertools import combinations
from types import MethodType
from typing import TYPE_CHECKING, Callable, Iterable, Iterator, Mapping, Optional, Sequence

import pybullet as p
from compas_fab.backends import CollisionCheckError, PyBulletClient, PyBulletPlanner
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.robots import RobotCell, RobotCellState
from compas_robots import Configuration, ToolModel

from ..scene import Scene
from . import HIDDEN_POSITION, check_display
from .cell import CompasFabCell
from .compas import filled, subtree
from .pp_client import pp_client

if TYPE_CHECKING:
    from compas_fab.robots import RigidBody, ToolState
    from compas_robots.model import Link


class CompasFabMirror:
    """A compas_fab PyBullet world planning for one robot, following the scenes given to `sync`."""

    def __init__(self, robot_id: str, log: Callable[[str], None] | None = None) -> None:
        """Connect an empty world without a window; the first `sync` loads everything (slow).

        Args:
            robot_id: The acting robot, e.g. "robots/a200-0806"; every other robot becomes an obstacle.
            log: Called on this mirror's thread with one line per full rebuild of the cell: why, and how long.
        """
        self.robot_id = robot_id
        self._log = log
        #: compas_fab's client and planner. Use them only on the owning thread.
        self.client: PyBulletClient | None = None
        # * The cell and its states, kept for the mirror's lifetime: loading and converting is the slow part.
        self._cells = CompasFabCell(robot_id, visual=False)
        self._connect(gui=False)

    def _connect(self, gui: bool) -> None:
        """Replace the world with a new, empty one, with PyBullet's own window if asked for.

        Raises:
            RuntimeError: If a window is asked for without an X display.
            pybullet.error: If another PyBullet window is open in this process. The old world stays.
        """
        if gui:
            check_display()
        client = PyBulletClient("gui" if gui else "direct", verbose=False)
        client.__enter__()
        if self.client is not None:
            self.close()
        #: Whether PyBullet's own window shows this world.
        self.client, self.gui = client, gui
        self.planner = with_tool_pairs(PyBulletPlanner(self.client))
        #: The state as last synced (None before the first sync).
        self.state: RobotCellState | None = None
        #: Other robots in the scene but parked at the last sync, because their base or joints are unknown.
        self.parked: tuple[str, ...] = ()
        # ? A new world holds no cell: the next sync hands one to it.
        self._cells.forget()
        # Lent since the last sync: the borrower may have left any state, or another cell.
        self._dirty = False

    @property
    def cell(self) -> RobotCell | None:
        """RobotCell | None: The cell as last built; a new object on every rebuild."""
        return self._cells.cell

    def set_gui(self, gui: bool) -> None:
        """Open or close PyBullet's own window, for debugging. The world is empty until the next `sync`.

        - ! Closing the window by hand ends the world; `set_gui(False)` brings it back.

        Raises:
            RuntimeError: If a window is asked for without an X display.
            pybullet.error: If another PyBullet window is open in this process.
        """
        if gui != self.gui:
            self._connect(gui)

    # --- --- --- --- --- SYNC --- --- --- --- ---

    def sync(self, scene: Scene) -> None:
        """Make the world match a scene: rebuild the cell if its models changed, then set the state.

        Args:
            scene: The world to copy. It is not modified.

        Raises:
            KeyError: If the acting robot is not in the scene.
            ValueError: If it can't be planned for (`RobotObject.acting_problems`), or has no SRDF.
        """
        acting = scene.robots[self.robot_id]
        problems = acting.acting_problems()
        if problems:
            raise ValueError(f"cannot plan for {self.robot_id}: {'; '.join(problems)}")
        if self._dirty and self.client.robot_cell is not self.cell:
            self._cells.forget()  # the borrower set a cell of its own
        started = time.monotonic()
        why = self._cells.update(scene)
        if why is not None:
            self.planner.set_robot_cell(self.cell)
            if self._log is not None:
                self._log(f"rebuilt the planning cell for {self.robot_id} in {time.monotonic() - started:.1f} s; "
                          f"why: {why}")
        state = self._cells.state(scene)
        self.parked = self._cells.parked
        # * Always the full state: after `lend` nothing in the world can be trusted.
        self.planner.set_robot_cell_state(state)
        self._move_hidden_away(state)
        self.state = state
        self._dirty = False

    def _move_hidden_away(self, state: RobotCellState) -> None:
        """Move every hidden body far away in PyBullet; compas_fab leaves it where the last state put it."""
        for name, body in state.rigid_body_states.items():
            if body.is_hidden:
                for part in self.client.rigid_bodies_puids.get(name, ()):
                    p.resetBasePositionAndOrientation(part, HIDDEN_POSITION, (0.0, 0.0, 0.0, 1.0),
                                                      physicsClientId=self.client.client_id)

    @contextmanager
    def lend(self) -> Iterator[PyBulletPlanner]:
        """Hand this world's compas_fab planner to other code (e.g. tamp), with pybullet_planning pointed at it.

        - ! The borrower may leave the world in any state: the mirror counts as changed, and the next `sync` writes
          the full state (and rebuilds the cell if the borrower set its own). Build a `search_check` after it.
        - ! One pybullet_planning user at a time in the whole process (`pp_client`).
        - * Its `check_collision` adds CC.6 (`with_tool_pairs`), also when compas_fab's own IK and planners call it.

        Yields:
            PyBulletPlanner: The planner, its cell and state as of the last `sync`.
        """
        self._dirty = True
        with pp_client(self.client.client_id):
            yield self.planner

    # --- --- --- --- --- QUERIES --- --- --- --- ---

    def configuration(self, joints: Mapping[str, float]) -> Configuration:
        """The acting robot's full configuration: `joints` where given, else the synced value.

        Args:
            joints: Values by joint name.

        Returns:
            Configuration: Every configurable joint.
        """
        return filled(self.state.robot_configuration, joints)

    def state_at(self, joints: Mapping[str, float]) -> RobotCellState:
        """The synced state with some joints changed; shares everything else with `state`.

        Args:
            joints: Values by joint name.

        Returns:
            RobotCellState: A new state. ! Treat its tool and body states as read-only.
        """
        return RobotCellState(robot_base_frame=self.state.robot_base_frame,
                              robot_configuration=self.configuration(joints),
                              tool_states=self.state.tool_states, rigid_body_states=self.state.rigid_body_states)

    def collisions(self, joints: Mapping[str, float] | None = None, full_report: bool = False) -> list[tuple[str, str]]:
        """What collides with the acting robot or its tools at the synced state, some joints changed.

        compas_fab's check, then each of the acting robot's tools against every other tool (CC.6, `tool_pairs`),
        which compas_fab never checks.

        Args:
            joints: Values by joint name to check at, or None for the synced state.
            full_report: Find every pair; otherwise stop at the first.

        Returns:
            list[tuple[str, str]]: Colliding pairs as our ids, e.g. ("robots/a200-0806/left_ur_arm_wrist_3_link",
                "obstacles/tables/A"); a mounted tool by its tool id, another robot by its robot id, e.g.
                ("tools/cindy/AT3L", "robots/alice"). Empty if clear.
        """
        state = self.state if joints is None else self.state_at(joints)
        try:
            self.planner.check_collision(state, {"full_report": full_report})
        except CollisionCheckError as error:
            return [(self._our_id(a), self._our_id(b)) for a, b in error.collision_pairs]
        return []

    def tool_pairs(self) -> list[tuple[str, str]]:
        """Each shown tool of the acting robot with every other shown tool: the pairs compas_fab leaves unchecked.

        Other robots are tools here, so this is also the acting robot's tools against other robots, and on a robot
        with two arms one gripper against the other.

        Returns:
            list[tuple[str, str]]: Tool ids, the acting robot's first.
        """
        return tool_pairs(self.state.tool_states, self.client.tools_puids)

    def search_check(self, joint_names: Sequence[str]) -> SearchCheck:
        """A fast collision check for a search that moves only these joints, from the synced state.

        Args:
            joint_names: The joints the search changes, e.g. one arm's six.

        Returns:
            SearchCheck: Call it with joint values in `joint_names` order.
        """
        return SearchCheck(self, joint_names)

    def _our_id(self, model: Link | ToolModel | RigidBody) -> str:
        """Our id of one side of a compas_fab collision pair."""
        return self._cells.our_id(model)

    @property
    def connected(self) -> bool:
        """bool: Whether the world still exists. False once closed, or once its window was closed."""
        return self.client.client_id is not None and bool(p.isConnected(physicsClientId=self.client.client_id))

    def close(self) -> None:
        """Disconnect this world and delete its files, unless already gone."""
        if self.client.client_id is not None:
            if self.connected:
                self.client.__exit__()
            else:
                self.client._cache_dir.cleanup()
            self.client.client_id = None


# --- --- --- --- --- CC.6: TOOLS AGAINST TOOLS --- --- --- --- ---

def tool_pairs(tool_states: Mapping[str, ToolState], tools: Iterable[str]) -> list[tuple[str, str]]:
    """Each shown tool of the acting robot with every other shown tool, once: the pairs compas_fab leaves unchecked.

    Args:
        tool_states: Tool id -> its state; the acting robot's tools have `attached_to_group` set.
        tools: The tool ids in the world, in the order to check them.

    Returns:
        list[tuple[str, str]]: Tool ids, the acting robot's first. ? Two other robots never pair up: neither moves
            while the acting robot plans.
    """
    shown = [name for name in tools if not tool_states[name].is_hidden]
    acting = [name for name in shown if tool_states[name].attached_to_group is not None]
    return [(a, b) for i, a in enumerate(acting) for b in shown if b != a and (b not in acting or b in acting[i:])]


def _check_collision_with_tool_pairs(planner: PyBulletPlanner, state: RobotCellState,
                                     options: Optional[dict] = None) -> None:
    """compas_fab's `check_collision` (CC.1 to CC.5), then CC.6: the acting robot's tools against the other tools.

    The options are compas_fab's; its `_skip_cc2` (no tool checks) skips CC.6 too.

    - ! Never prints, even with `verbose`: tamp counts the printed "CC.<n>" lines for CC.1 to CC.5 only.

    Args:
        planner: The planner whose world to check.
        state: The state to check, as for compas_fab's `check_collision`.
        options: compas_fab's options, e.g. {"full_report": True}.

    Raises:
        CollisionCheckError: If anything collides: compas_fab's messages and pairs first, then the CC.6 ones, e.g.
            "CC.6 between tool 'tools/cindy/AT3L' and tool 'robots/alice' - COLLISION" with the two tool models.
    """
    options = options or {}
    full_report = options.get("full_report", False)
    messages, pairs = [], []
    try:
        # ! The class's method, never `planner.check_collision`: that is this function again.
        PyBulletPlanner.check_collision(planner, state, options)  # ? poses the world at `state`
    except CollisionCheckError as error:
        if not full_report:
            raise
        messages, pairs = [error.message], list(error.collision_pairs)
    if not options.get("_skip_cc2", False):
        client = planner.client
        tools = client.tools_puids
        for a, b in tool_pairs(client.robot_cell_state.tool_states, tools):
            if p.getClosestPoints(tools[a], tools[b], 0.0, physicsClientId=client.client_id):
                messages.append(f"CC.6 between tool '{a}' and tool '{b}' - COLLISION")
                pairs.append((client.robot_cell.tool_models[a], client.robot_cell.tool_models[b]))
                if not full_report:
                    break
    if messages:
        raise CollisionCheckError("\n".join(messages), pairs)


def with_tool_pairs(planner: PyBulletPlanner) -> PyBulletPlanner:
    """Make this planner's `check_collision` also check tools against tools (CC.6, `_check_collision_with_tool_pairs`).

    * Bound on this one object: compas_fab's own IK and planners call `check_collision` on the planner, so a borrower
      (`CompasFabMirror.lend`) gets CC.6 too.

    Args:
        planner: The planner to change.

    Returns:
        PyBulletPlanner: The same planner.
    """
    planner.check_collision = MethodType(_check_collision_with_tool_pairs, planner)
    return planner


# --- --- --- --- --- FAST CHECKS FOR SEARCHES --- --- --- --- ---

@dataclass(frozen=True)
class _Part:
    """One side of a pair to check: a PyBullet body, or one link of it.

    Attributes:
        body: PyBullet body id.
        link: Link index, or None for the whole body.
        name: Our id, for reports.
    """

    body: int
    link: Optional[int]
    name: str


def _offset(client_id: int, robot: int, link: int, body: int) -> tuple:
    """Where a body sits in a robot link's frame now, as (position, quaternion)."""
    state = p.getLinkState(robot, link, computeForwardKinematics=True, physicsClientId=client_id)
    inverse = p.invertTransform(state[4], state[5])
    return p.multiplyTransforms(*inverse, *p.getBasePositionAndOrientation(body, physicsClientId=client_id))


class SearchCheck:
    """The rules of `CompasFabMirror.collisions` for a search over some joints, resolved once and checked fast.

    * Why ours: compas_fab's `check_collision` resets the whole cell and walks every pair on each call
      (3.6 ms vs 0.16 ms here; arm plans ran ~12x slower and hit the search time limit). Replace it once
      compas_fab offers a fast repeated check.

    - ! Only pairs with a moving side are checked: check the start once with `collisions`.
    - ! It leaves the world at the last configuration checked; `sync` and `collisions` put it back.
    - ! Build it after each `sync`.
    """

    def __init__(self, mirror: CompasFabMirror, joint_names: Sequence[str]):
        """Resolve the pairs to check for a search over `joint_names`.

        Args:
            mirror: A synced mirror; its current state is the one searched from.
            joint_names: The joints the search changes.
        """
        client, state, model = mirror.client, mirror.state, mirror.cell.robot_model
        self._client_id = client.client_id
        self._robot = client.robot_puid
        self._joints = [client.robot_joint_puids[name] for name in joint_names]
        own = mirror.robot_id

        # * What moves: every link below a searched joint, and the bodies and tools attached to those links.
        moving = set().union(*(subtree(model, model.get_joint_by_name(name).child.link) for name in joint_names))
        bodies = {name: body for name, body in state.rigid_body_states.items()
                  if not body.is_hidden and name in client.rigid_bodies_puids}
        carried = {name for name, body in bodies.items() if body.attached_to_link in moving}
        #: (PyBullet parts, link index, attachment as (position, quaternion)) of each carried body.
        self._carried = [(client.rigid_bodies_puids[name], client.robot_link_puids[bodies[name].attached_to_link],
                          pose_from_frame(bodies[name].attachment_frame)) for name in carried]
        # ? compas_fab placed the acting robot's tools at `sync`: keep each where it sits in its flange's frame.
        flanges = {name: mirror.cell.get_end_effector_link_name(tool.attached_to_group)
                   for name, tool in state.tool_states.items() if tool.attached_to_group is not None}
        carried_tools = {name for name, flange in flanges.items()
                         if flange in moving and not state.tool_states[name].is_hidden}
        self._carried += [([client.tools_puids[name]], client.robot_link_puids[flanges[name]],
                           _offset(self._client_id, self._robot, client.robot_link_puids[flanges[name]],
                                   client.tools_puids[name])) for name in carried_tools]

        # ? Links without collision shapes (tool0, flange, …) never hit anything: leave them out.
        links = {name: _Part(self._robot, index, f"{own}/{name}") for name, index in client.robot_link_puids.items()
                 if p.getCollisionShapeData(self._robot, index, physicsClientId=self._client_id)}
        tools = {name: _Part(body, None, name) for name, body in client.tools_puids.items()
                 if not state.tool_states[name].is_hidden}
        parts = {name: [_Part(body, None, name) for body in client.rigid_bodies_puids[name]] for name in bodies}

        # * The pairs `collisions` checks (compas_fab's CC.1 to CC.5, then `tool_pairs`), with a moving side only.
        disabled = client.unordered_disabled_collisions
        pairs = [(links[a], links[b]) for a, b in combinations(links, 2)
                 if (a in moving or b in moving) and frozenset((a, b)) not in disabled]
        pairs += [(links[link], tool) for link in links for name, tool in tools.items()
                  if (link in moving or name in carried_tools) and link not in state.tool_states[name].touch_links]
        pairs += [(links[link], part) for link in links for name, body in bodies.items()
                  if (link in moving or name in carried) and link not in body.touch_links for part in parts[name]]
        for name in sorted(carried):
            for other, body in bodies.items():
                # ? Two carried bodies once, not twice.
                if other == name or (other in carried and other < name) or \
                        name in body.touch_bodies or other in bodies[name].touch_bodies:
                    continue
                pairs += [(a, b) for a in parts[name] for b in parts[other]]
        pairs += [(tool, part) for tool_name, tool in tools.items() for name, body in bodies.items()
                  if (name in carried or tool_name in carried_tools)
                  and body.attached_to_tool != tool_name and tool_name not in body.touch_bodies for part in parts[name]]
        pairs += [(tools[a], tools[b]) for a, b in mirror.tool_pairs() if a in carried_tools or b in carried_tools]
        #: (body A, body B, getClosestPoints link arguments, names) per pair.
        self._pairs = [(a.body, b.body, {**({"linkIndexA": a.link} if a.link is not None else {}),
                                         **({"linkIndexB": b.link} if b.link is not None else {})}, (a.name, b.name))
                       for a, b in dict.fromkeys(pairs)]

    def __call__(self, values: Sequence[float]) -> tuple[str, str] | None:
        """Set the searched joints and report the first colliding pair.

        Args:
            values: Joint values, in the order of the joint names given.

        Returns:
            tuple[str, str] | None: The first pair hit, as our ids; None if clear.
        """
        client_id = self._client_id
        for joint, value in zip(self._joints, values):
            p.resetJointState(self._robot, joint, float(value), physicsClientId=client_id)
        for bodies, link, (position, orientation) in self._carried:
            state = p.getLinkState(self._robot, link, computeForwardKinematics=True, physicsClientId=client_id)
            world = p.multiplyTransforms(state[4], state[5], position, orientation)
            for body in bodies:
                p.resetBasePositionAndOrientation(body, *world, physicsClientId=client_id)
        for body_a, body_b, links, names in self._pairs:
            if p.getClosestPoints(body_a, body_b, 0.0, physicsClientId=client_id, **links):
                return names
        return None
