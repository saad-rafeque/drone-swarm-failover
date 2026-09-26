"""V formation: slot geometry, parity-arm assignment, reassignment behaviour, control law."""
from __future__ import annotations

import itertools
import math

import pytest

from swarm_agent.formation import (LEFT, RIGHT, assign_slots, follower_slot, follower_velocity,
                                   formation_errors, initial_layout, rms, slot_position, v_slot_body)
from swarm_agent.geometry import dist

SP, HALF = 10.0, math.radians(45.0)
NORTH = math.pi / 2


def test_slot_geometry_spacing_and_symmetry():
    l1, l2 = v_slot_body((LEFT, 1), SP, HALF), v_slot_body((LEFT, 2), SP, HALF)
    r1 = v_slot_body((RIGHT, 1), SP, HALF)
    assert math.dist(l1, l2) == pytest.approx(SP)            # neighbours along an arm
    assert math.dist((0.0, 0.0), l1) == pytest.approx(SP)    # master to first slot
    assert l1[0] == pytest.approx(r1[0]) and l1[1] == pytest.approx(-r1[1])
    assert l1[0] < 0 and l1[1] > 0                           # behind and to the left
    for bad in ((LEFT, 0), (0, 1)):
        with pytest.raises(ValueError):
            v_slot_body(bad, SP, HALF)


def test_slot_rotated_into_heading():
    # heading north: left arm is to the west, behind is south
    p = slot_position((100.0, 200.0, 30.0), NORTH, (LEFT, 1), SP, HALF, alt_offset_m=8.0)
    assert p[0] < 100.0 and p[1] < 200.0 and p[2] == 38.0
    east = slot_position((0.0, 0.0, 0.0), 0.0, (LEFT, 1), SP, HALF)
    assert east[0] < 0 and east[1] > 0  # heading east: behind is west, left is north


def test_assign_slots_parity_arms_rank_by_id():
    slots = assign_slots(range(1, 11), master_id=1)
    assert {i: s for i, s in slots.items() if s[0] == LEFT} == {2: (LEFT, 1), 4: (LEFT, 2), 6: (LEFT, 3),
                                                               8: (LEFT, 4), 10: (LEFT, 5)}
    assert {i: s for i, s in slots.items() if s[0] == RIGHT} == {3: (RIGHT, 1), 5: (RIGHT, 2), 7: (RIGHT, 3),
                                                                9: (RIGHT, 4)}
    assert 1 not in slots


def test_follower_loss_only_slides_same_arm_drones_behind_it():
    """F4: removing drone 4 moves only 6, 8, 10 (one slot inward); the other arm is untouched."""
    before = assign_slots(range(1, 11), 1)
    after = assign_slots(set(range(1, 11)) - {4}, 1)
    moved = {i for i in after if after[i] != before[i]}
    assert moved == {6, 8, 10}
    assert all(after[i] == (LEFT, before[i][1] - 1) for i in moved)


def test_master_loss_new_master_keeps_its_arm_in_place():
    """F1: master 1 lost, 2 (was LEFT row 1) leads. The left arm keeps its absolute positions and
    the right arm shifts by exactly one slot (10 m), so nobody crosses the formation."""
    ids = list(range(1, 11))
    apex = (0.0, 0.0, 30.0)
    old = {i: slot_position(apex, NORTH, s, SP, HALF) for i, s in assign_slots(ids, 1).items()}
    new_master_pos = old[2]
    new = {i: slot_position(new_master_pos, NORTH, s, SP, HALF) for i, s in assign_slots(set(ids) - {1}, 2).items()}
    for i in (4, 6, 8, 10):
        assert dist(old[i], new[i]) == pytest.approx(0.0, abs=1e-9)
    for i in (3, 5, 7, 9):
        assert dist(old[i], new[i]) == pytest.approx(SP)


def test_arms_rebalance_only_when_imbalance_exceeds_two():
    slots = assign_slots([1, 3, 5, 7, 9, 11], master_id=1)       # all odd followers: 5 right, 0 left
    arms = [sum(1 for s in slots.values() if s[0] == side) for side in (LEFT, RIGHT)]
    assert arms == [2, 3]
    # outermost right drones move, one at a time, until the imbalance is <= 2
    assert {i: s for i, s in slots.items() if s[0] == LEFT} == {11: (LEFT, 1), 9: (LEFT, 2)}
    even = assign_slots([1, 2, 4, 6, 8, 10, 12], master_id=1)     # 6 left, 0 right
    assert sorted(i for i, s in even.items() if s[0] == RIGHT) == [10, 12]
    assert assign_slots([1, 2, 4, 3], 1) == {2: (LEFT, 1), 4: (LEFT, 2), 3: (RIGHT, 1)}  # 2 vs 1: kept


def test_follower_slot_member_and_orphan():
    members = frozenset({1, 2, 3, 4, 5})
    assert follower_slot(4, 1, members, heard_ids=[1, 2, 3, 5, 9]) == ((LEFT, 2), False)
    # 9 (odd, right arm holds 3 and 5) is not a member -> orphan behind the right arm
    assert follower_slot(9, 1, members, heard_ids=[1, 2, 3, 4, 5]) == ((RIGHT, 3), True)
    # 7 and 9 both orphans on the right arm -> ranked among themselves by ID
    assert follower_slot(9, 1, members, heard_ids=[1, 2, 3, 4, 5, 7]) == ((RIGHT, 4), True)
    assert follower_slot(7, 1, members, heard_ids=[9]) == ((RIGHT, 3), True)
    # an even orphan goes behind the left arm (2, 4)
    assert follower_slot(8, 1, members, heard_ids=[]) == ((LEFT, 3), True)


def test_follower_velocity_feedforward_and_saturation():
    mv = (0.0, 5.0, 0.0)
    at_slot = follower_velocity((0.0, 0.0, 30.0), (0.0, 0.0, 30.0), mv, 0.6, 3.0, 10.0, 2.0, 1.5)
    assert at_slot == pytest.approx(mv)
    far = follower_velocity((0.0, 0.0, 30.0), (100.0, 0.0, 30.0), mv, 0.6, 3.0, 10.0, 2.0, 1.5)
    assert far == pytest.approx((3.0, 5.0, 0.0))                  # horizontal P-term capped at 3
    down = follower_velocity((0.0, 0.0, 30.0), (100.0, 0.0, 20.0), mv, 0.6, 1.0, 10.0, 2.0, 1.5, gain_z=1.0)
    assert down[2] == pytest.approx(-1.5)                         # vertical limited by descent rate only
    assert math.hypot(down[0], down[1] - 5.0) == pytest.approx(1.0)
    fast = follower_velocity((0.0, 0.0, 30.0), (0.0, 100.0, 30.0), (0.0, 9.0, 0.0), 0.6, 3.0, 10.0, 2.0)
    assert math.hypot(fast[0], fast[1]) == pytest.approx(10.0)    # total speed cap


@pytest.mark.parametrize("n", [1, 2, 3, 5, 10])
def test_initial_layout_is_the_formation_and_well_spaced(n):
    ids = list(range(1, n + 1))
    layout = initial_layout(ids, NORTH, SP, HALF)
    assert layout[1] == (0.0, 0.0)
    for a, b in itertools.combinations(ids, 2):
        assert math.dist(layout[a], layout[b]) >= SP - 1e-9
    pos = {i: (e, nn, 30.0) for i, (e, nn) in layout.items()}
    errs = formation_errors(pos, 1, NORTH, ids, SP, HALF)
    assert all(v == pytest.approx(0.0, abs=1e-9) for v in errs.values())


def test_rms():
    assert rms([3.0, 4.0]) == pytest.approx(math.sqrt(12.5))
    assert rms([]) == 0.0
