"""Formations: slot geometry, slot assignment and the follower velocity command (pure Python).

Shapes (config formation.shape): "V" (the default and the only one used in Phases 0-4), "line" (line abreast:
followers side by side with the leader in the middle), "column" (single file behind the leader) and
"echelon" (one diagonal line behind the leader, to its right). All shapes share the slot model below:
two arms by ID parity, a row per arm. In "column" and "echelon" the two arms are interleaved into one line
(left row r -> place 2r-1, right row r -> place 2r), so neighbours stay spacing_m apart.

Conventions: ENU, heading measured from East counter-clockwise (geometry.heading_of). A slot
offset is defined in the master's body frame (forward, left) and rotated into the master's
heading (docs/SPECIFICATION.md §6).

Slot assignment (docs/SPECIFICATION.md §6: "follower slot = rank among alive drones sorted by ID"):
the arm is fixed by ID parity (even IDs left, odd IDs right) and the row is the rank by ID
within that arm among the alive members. Losing a drone makes only the drones behind it on
the same arm slide one slot inward, and a new master that flew at row 1 keeps its own arm in
place. A plain alternating left/right rank mapping swaps every higher rank across the
formation instead (up to 64 m lateral moves with 10 drones; see reports/PHASE_2.md).
Arms are rebalanced (outermost drones move to the other arm's tail) only when their lengths
differ by more than MAX_ARM_IMBALANCE.
"""
from __future__ import annotations

import math
from collections.abc import Iterable

from .geometry import Vec3, add, body_to_enu, clamp_norm, clamp_xy_z, dist, scale, sub

LEFT, RIGHT = 1, -1
MAX_ARM_IMBALANCE = 2
SHAPES = ("V", "line", "column", "echelon")

Slot = tuple[int, int]  # (side: LEFT/RIGHT, row >= 1)


def v_slot_body(slot: Slot, spacing_m: float, half_angle_rad: float) -> tuple[float, float]:
    """(forward, left) offset of a slot relative to the master at the apex. Neighbours along an
    arm are spacing_m apart; each arm makes half_angle_rad with the master's reverse heading."""
    side, row = slot
    if row < 1 or side not in (LEFT, RIGHT):
        raise ValueError(f"invalid slot {slot}")
    return (-row * spacing_m * math.cos(half_angle_rad), side * row * spacing_m * math.sin(half_angle_rad))


def slot_body(slot: Slot, spacing_m: float, half_angle_rad: float, shape: str = "V") -> tuple[float, float]:
    """(forward, left) offset of a slot relative to the leader for any shape in SHAPES."""
    if shape == "V":
        return v_slot_body(slot, spacing_m, half_angle_rad)
    side, row = slot
    if row < 1 or side not in (LEFT, RIGHT):
        raise ValueError(f"invalid slot {slot}")
    if shape == "line":
        return 0.0, side * row * spacing_m
    place = 2 * row - (1 if side == LEFT else 0)          # the two arms interleaved into one line
    if shape == "column":
        return -place * spacing_m, 0.0
    if shape == "echelon":
        return -place * spacing_m * math.cos(half_angle_rad), -place * spacing_m * math.sin(half_angle_rad)
    raise ValueError(f"unknown formation shape {shape!r} (one of {SHAPES})")


def slot_position(master_pos: Vec3, heading: float, slot: Slot, spacing_m: float,
                  half_angle_rad: float, alt_offset_m: float = 0.0, shape: str = "V") -> Vec3:
    f, l = slot_body(slot, spacing_m, half_angle_rad, shape)
    de, dn = body_to_enu(f, l, heading)
    return (master_pos[0] + de, master_pos[1] + dn, master_pos[2] + alt_offset_m)


def _arms(follower_ids: Iterable[int]) -> tuple[list[int], list[int]]:
    ids = sorted(follower_ids)
    left = [i for i in ids if i % 2 == 0]
    right = [i for i in ids if i % 2 == 1]
    while len(left) - len(right) > MAX_ARM_IMBALANCE:
        right.append(left.pop())
    while len(right) - len(left) > MAX_ARM_IMBALANCE:
        left.append(right.pop())
    return left, right


def assign_slots(member_ids: Iterable[int], master_id: int) -> dict[int, Slot]:
    """Slot of every member follower (master excluded)."""
    left, right = _arms(set(member_ids) - {master_id})
    slots = {i: (LEFT, row) for row, i in enumerate(left, start=1)}
    slots.update({i: (RIGHT, row) for row, i in enumerate(right, start=1)})
    return slots


def follower_slot(my_id: int, master_id: int, members: set[int] | frozenset[int],
                  heard_ids: Iterable[int]) -> tuple[Slot, bool]:
    """(slot, is_orphan) for a follower.

    Slots come from the MASTER's member list (carried in its heartbeat), so every follower
    derives the same assignment even when their own views of who is alive differ. A drone
    missing from that list cannot be heard by the master: it is an orphan, placed behind the
    last member of its own arm (by ID among the orphans it hears) on a separate altitude layer.
    """
    slots = assign_slots(members, master_id)
    if my_id in slots:
        return slots[my_id], False
    side = LEFT if my_id % 2 == 0 else RIGHT
    arm_len = sum(1 for s, _ in slots.values() if s == side)
    same_arm_orphans = sorted({my_id} | {i for i in heard_ids
                                         if i not in members and i != master_id and i % 2 == my_id % 2})
    return (side, arm_len + same_arm_orphans.index(my_id) + 1), True


def follower_velocity(my_pos: Vec3, slot_pos: Vec3, master_vel: Vec3, gain: float,
                      max_correction_mps: float, max_speed_mps: float, max_climb_mps: float,
                      max_descent_mps: float | None = None, gain_z: float | None = None) -> Vec3:
    """Master velocity (feed-forward) + saturated P-control on the slot position error.

    The horizontal P-term is saturated at max_correction_mps; the vertical command is limited
    by the climb/descent rates only (so a slow horizontal limit never slows a layer change).
    """
    err = sub(slot_pos, my_pos)
    cx, cy, _ = clamp_norm((gain * err[0], gain * err[1], 0.0), max_correction_mps)
    cz = (gain if gain_z is None else gain_z) * err[2]
    return clamp_xy_z((master_vel[0] + cx, master_vel[1] + cy, master_vel[2] + cz),
                      max_speed_mps, max_climb_mps, max_descent_mps)


def initial_layout(ids: Iterable[int], heading: float, spacing_m: float,
                   half_angle_rad: float, shape: str = "V") -> dict[int, tuple[float, float]]:
    """Ground spawn offsets (east, north) from the origin: the formation itself with the lowest
    ID as the leader, so every drone takes off already in the slot it will fly."""
    ids = sorted(ids)
    master = ids[0]
    out = {master: (0.0, 0.0)}
    for did, slot in assign_slots(ids, master).items():
        f, l = slot_body(slot, spacing_m, half_angle_rad, shape)
        out[did] = body_to_enu(f, l, heading)
    return out


def formation_errors(positions: dict[int, Vec3], master_id: int, heading: float, members: Iterable[int],
                     spacing_m: float, half_angle_rad: float, shape: str = "V") -> dict[int, float]:
    """3-D distance of every member follower from its ideal slot around the master's position."""
    master_pos = positions[master_id]
    return {
        did: dist(positions[did], slot_position(master_pos, heading, slot, spacing_m, half_angle_rad, shape=shape))
        for did, slot in assign_slots(members, master_id).items() if did in positions
    }


def rms(values: Iterable[float]) -> float:
    vals = list(values)
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0
