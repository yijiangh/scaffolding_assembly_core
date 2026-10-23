"""Tests for what follows from a state without being stored: the pose rule, mate status, allowed contacts."""

from dataclasses import replace
from pathlib import Path

import pytest
from process_fixtures import build_process

from scaffolding_assembly_core.process import Process, Holder, State
from scaffolding_assembly_core.process.relations import (ENGAGED, NOT_RELEVANT, OPEN, PENDING, allowed_contacts, is_present,
                                                mate_status, mount_offset, placement)
from scaffolding_assembly_core.geometry import Pose, compose

G1_MATE, J1_MATE = ("ground/WG0", "joints/G1_ground"), ("joints/J1_female", "joints/J1_male")


@pytest.fixture
def process(tmp_path: Path) -> Process:
    """The fixture process."""
    return build_process(tmp_path)


def _start(process: Process, movement_id: str) -> State:
    """The start state of one movement, by id."""
    return next(movement.start for _, movement in process.movements() if movement.id == movement_id)


def test_pose_rule(process):
    """Built: assembly pose (even while held); attached and unbuilt: the first holder; staged: its pose, else its assembly pose."""
    held_by_two = _start(process, "B1_R_M0_ungrasp")
    assert len(held_by_two.attached["bars/B1"]) == 2
    assert placement(process, held_by_two, "bars/B1") == process.bodies["bars/B1"].assembly_pose
    carried = _start(process, "B2_M1_grasp")
    assert placement(process, carried, "bars/B2") == carried.attached["bars/B2"][0]
    staged = _start(process, "B1_M0_place_bar")
    assert placement(process, staged, "bars/B2") == Pose((2.0, 0.0, 0.1))
    assert placement(process, replace(staged, poses={}), "bars/B2") == process.bodies["bars/B2"].assembly_pose


def test_halves_follow_their_bar(process):
    """A half is placed by its bar's placement times the mount offset, and is present with its bar."""
    offset = mount_offset(process, "joints/J1_male")
    carried = _start(process, "B2_M1_grasp")
    holder = carried.attached["bars/B2"][0]
    assert placement(process, carried, "joints/J1_male") == Holder(holder.to, compose(holder.grasp, offset))
    staged = _start(process, "B1_M0_place_bar")
    assert placement(process, staged, "joints/J1_male") == compose(Pose((2.0, 0.0, 0.1)), offset)
    assert not is_present(process, staged, "joints/G1_ground") and is_present(process, staged, "joints/J1_male")
    assert process.halves_of("bars/B1") == ("joints/G1_ground", "joints/J1_female")


def test_mate_status(process):
    """Open, pending while being joined, engaged once both sides are built, not relevant when neither is present."""
    assert mate_status(process, _start(process, "B1_M0_place_bar"), G1_MATE) == OPEN
    assert mate_status(process, _start(process, "B1_M2_insert"), G1_MATE) == PENDING
    assert mate_status(process, _start(process, "B1_H_M0_approach"), G1_MATE) == ENGAGED
    assert mate_status(process, _start(process, "B1_H_M0_approach"), J1_MATE) == OPEN
    assert mate_status(process, _start(process, "B2_M3_insert"), J1_MATE) == PENDING
    assert mate_status(process, _start(process, "B1_HR_M0_open"), J1_MATE) == ENGAGED
    empty = replace(_start(process, "B1_M0_place_bar"), present=frozenset({"ground/WG0"}), poses={})
    assert mate_status(process, empty, J1_MATE) == NOT_RELEVANT


def test_allowed_contacts_are_derived(process):
    """At B2's insert: tools with their whole part, halves with their bars, mates with each other's bar, wheels.

    A pending or engaged mate's halves touch each other and each other's bar.
    """
    contacts = allowed_contacts(process, _start(process, "B2_M3_insert"))
    assert contacts == {
        ("joints/J1_male", "tools/cindy/AT3L"), ("bars/B2", "tools/cindy/AT3L"),
        ("bars/B1", "tools/alice/Grip"), ("joints/G1_ground", "tools/alice/Grip"), ("joints/J1_female", "tools/alice/Grip"),
        ("bars/B1", "joints/G1_ground"), ("bars/B1", "joints/J1_female"), ("bars/B2", "joints/J1_male"),
        G1_MATE, J1_MATE, ("bars/B1", "joints/J1_male"), ("bars/B2", "joints/J1_female"),
        ("ground/WG0", "robots/alice/wheel_link"), ("ground/WG0", "robots/cindy/wheel_link"),
    }
    # * Before B1 is mounted: B1's halves are absent, the open J1 mate allows nothing; Alice is absent from the state
    # * but still stands on the floor, so her wheels may touch it.
    before = _start(process, "B1_M0_place_bar")
    assert before.robots.get("robots/alice") is None
    assert allowed_contacts(process, before) == {
        ("bars/B2", "joints/J1_male"), ("ground/WG0", "robots/alice/wheel_link"),
        ("ground/WG0", "robots/cindy/wheel_link")}
