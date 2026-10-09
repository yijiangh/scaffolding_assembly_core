"""`mirrors.cell.CompasFabCell`: the compas_fab cell of a scene, kept while its models are; strict and drawing states.

Also the mirror built on it: `CompasFabMirror.sync` plans on the same cell and keeps it while nothing changed.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from design_fixtures import build_design

from bar_assembly_core.mirrors.cell import CompasFabCell
from bar_assembly_core.mirrors.compas import PARKED_POSITION


def _movement(design, movement_id):
    return next(m for a in design.actions.values() for m in a.movements if m.id == movement_id)


@pytest.fixture
def design(tmp_path: Path):
    return build_design(tmp_path)


def test_cell_is_kept_until_a_geometry_changes(design):
    """A second update with the same models keeps the cell; a new geometry object rebuilds it."""
    cells = CompasFabCell("robots/cindy")
    scene = design.scene_at(_movement(design, "B1_M2_insert"))
    assert cells.update(scene) == "first build"
    cell = cells.cell
    assert set(cell.rigid_body_models) == {body for body, b in scene.bodies.items() if b.geometry.collision}
    assert set(cell.tool_models) == {"tools/AT3L", "robots/alice"}
    assert cells.update(design.scene_at(_movement(design, "B1_R_M0_ungrasp"))) is None
    assert cells.cell is cell

    from dataclasses import replace
    from bar_assembly_core.geometry import Geometry
    bodies = dict(design.bodies)
    old = bodies["bars/B2"].geometry
    bodies["bars/B2"] = replace(bodies["bars/B2"], geometry=Geometry(old.visual, old.collision))
    changed = replace(design, bodies=bodies)
    assert cells.update(changed.scene_at(_movement(changed, "B1_M2_insert"))) == "bodies changed: bars/B2"
    assert cells.cell is not cell


def test_visual_cell_keeps_visual_shapes(design):
    """Drawing cells load the robots' visual shapes; planning cells drop them."""
    scene = design.scene_at(_movement(design, "B1_M2_insert"))
    shown, planned = CompasFabCell("robots/cindy", visual=True), CompasFabCell("robots/cindy")
    shown.update(scene)
    planned.update(scene)
    base = lambda cell: cell.robot_model.get_link_by_name("base_link")  # noqa: E731
    assert base(shown.cell).visual and not base(planned.cell).visual
    assert base(shown.cell).collision and base(planned.cell).collision


def test_strict_state_refuses_what_a_drawing_parks(design):
    """Unknown joints: strict raises; a drawing state uses 0. Absent robots are parked; held bodies attached."""
    cells = CompasFabCell("robots/cindy", visual=True)
    mount = design.scene_at(_movement(design, "B1_M0_mount"))  # Cindy's joints unknown, Alice absent
    cells.update(mount)
    with pytest.raises(ValueError, match="joints not known"):
        cells.state(mount)
    state = cells.state(mount, strict=False)
    assert list(state.tool_states["robots/alice"].frame.point) == list(PARKED_POSITION)
    assert state.rigid_body_states["bars/B1"].is_hidden  # not mounted yet

    insert = design.scene_at(_movement(design, "B1_M2_insert"))
    state = cells.state(insert)
    assert state.rigid_body_states["bars/B1"].attached_to_link == "left_tool0"
    assert state.tool_states["tools/AT3L"].attached_to_group is not None


def test_drawing_state_parks_a_robot_without_base(design):
    """Cindy's base is open during her retreat: a drawing parks her, a plan refuses her."""
    cells = CompasFabCell("robots/cindy")
    retreat = design.scene_at(_movement(design, "B1_R_M1_retreat"))
    cells.update(retreat)
    with pytest.raises(ValueError, match="base is not tracked"):
        cells.state(retreat)
    assert list(cells.state(retreat, strict=False).robot_base_frame.point) == list(PARKED_POSITION)


def test_unknown_other_robots_are_parked(design):
    """Alice arrives with unknown joints: she is parked, not drawn at a guess, and `parked` names her."""
    for visual in (False, True):
        cells = CompasFabCell("robots/cindy", visual=visual)
        approach = design.scene_at(_movement(design, "B1_H_M0_approach"))
        cells.update(approach)
        state = cells.state(approach, strict=not visual)
        assert list(state.tool_states["robots/alice"].frame.point) == list(PARKED_POSITION)
        assert cells.parked == ("robots/alice",)
        cells.state(design.scene_at(_movement(design, "B1_H_M1_close")), strict=not visual)
        assert cells.parked == ()


def test_mirror_plans_on_the_cell(design):
    """The mirror syncs, keeps its cell across syncs, and names collision pairs by our ids."""
    from bar_assembly_core.mirrors.compas_fab import CompasFabMirror

    mirror = CompasFabMirror("robots/cindy")
    try:
        mirror.sync(design.scene_at(_movement(design, "B1_M2_insert")))
        cell = mirror.cell
        assert mirror.client.robot_cell is cell
        mirror.sync(design.scene_at(_movement(design, "B1_R_M0_ungrasp")))
        assert mirror.cell is cell and mirror.parked == ()
        mirror.sync(design.scene_at(_movement(design, "B1_H_M0_approach")))
        assert mirror.parked == ("robots/alice",)
        pairs = mirror.collisions(full_report=True)
        assert all(isinstance(a, str) and isinstance(b, str) for a, b in pairs)
        with mirror.lend() as planner:
            assert planner is mirror.planner
    finally:
        mirror.close()
