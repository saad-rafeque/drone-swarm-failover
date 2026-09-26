"""V formation: slot geometry, rank assignment and the follower velocity command (pure Python).

Conventions: ENU, heading measured from East counter-clockwise (geometry.heading_of).
A slot offset is defined in the master's body frame (forward, left) and rotated into the
master's heading (docs/SPECIFICATION.md §6).
"""
from __future__ import annotations

import math
from collections.abc import Iterable

from .geometry import Vec3, add, body_to_enu, clamp_norm, clamp_xy_z, dist, scale, sub


def v_slot_body(rank: int, spacing_m: float, half_angle_rad: float) -> tuple[float, float]:
    """(forward, left) offset of follower `rank` (1-based) relative to the master at the apex.

    Odd ranks take the left arm, even ranks the right arm; row = ceil(rank / 2). Neighbours
    along an arm are spacing_m apart, and each arm makes half_angle_rad with the master's
    reverse heading.
    """
    if rank < 1:
        raise ValueError(f"rank must be >= 1, got {rank}")
    row = (rank + 1) // 2
    side = 1.0 if rank % 2 == 1 else -1.0
    return (-row * spacing_m * math.cos(half_angle_rad), side * row * spacing_m * math.sin(half_angle_rad))


def slot_position(master_pos: Vec3, heading: float, rank: int, spacing_m: float,
                  half_angle_rad: float, alt_offset_m: float = 0.0) -> Vec3:
    f, l = v_slot_body(rank, spacing_m, half_angle_rad)
    de, dn = body_to_enu(f, l, heading)
    return (master_pos[0] + de, master_pos[1] + dn, master_pos[2] + alt_offset_m)


def assign_ranks(member_ids: Iterable[int], master_id: int) -> dict[int, int]:
    """Follower slot = rank among the member drones sorted by ID (master excluded)."""
    followers = sorted(set(member_ids) - {master_id})
    return {fid: rank for rank, fid in enumerate(followers, start=1)}


def follower_rank(my_id: int, master_id: int, members: set[int], heard_ids: Iterable[int]) -> tuple[int, bool]:
    """(rank, is_orphan) for a follower.

    Ranks come from the MASTER's member list (carried in its heartbeat), so every follower
    derives the same slots even when their own views of who is alive differ. A drone missing
    from that list cannot be heard by the master: it is an orphan, ranked after all members
    among the other non-members it can hear, and flown on a separate altitude layer.
    """
    if my_id in members:
        return assign_ranks(members, master_id)[my_id], False
    n_members = len(set(members) - {master_id})
    orphans = sorted({my_id} | {i for i in heard_ids if i not in members and i != master_id})
    return n_members + orphans.index(my_id) + 1, True


def follower_velocity(my_pos: Vec3, slot_pos: Vec3, master_vel: Vec3, gain: float,
                      max_correction_mps: float, max_speed_mps: float, max_climb_mps: float) -> Vec3:
    """Master velocity (feed-forward) + saturated P-control on the slot position error."""
    correction = clamp_norm(scale(sub(slot_pos, my_pos), gain), max_correction_mps)
    return clamp_xy_z(add(master_vel, correction), max_speed_mps, max_climb_mps)


def initial_layout(ids: Iterable[int], heading: float, spacing_m: float,
                   half_angle_rad: float) -> dict[int, tuple[float, float]]:
    """Ground spawn offsets (east, north) from the origin: the V formation itself, lowest ID at
    the apex, so every drone takes off already in the slot it will fly."""
    ids = sorted(ids)
    master = ids[0]
    out = {master: (0.0, 0.0)}
    for did, rank in assign_ranks(ids, master).items():
        f, l = v_slot_body(rank, spacing_m, half_angle_rad)
        out[did] = body_to_enu(f, l, heading)
    return out


def formation_errors(positions: dict[int, Vec3], master_id: int, heading: float, members: Iterable[int],
                     spacing_m: float, half_angle_rad: float) -> dict[int, float]:
    """3-D distance of every member follower from its ideal slot around the master's position."""
    ranks = assign_ranks(members, master_id)
    master_pos = positions[master_id]
    return {
        did: dist(positions[did], slot_position(master_pos, heading, rank, spacing_m, half_angle_rad))
        for did, rank in ranks.items() if did in positions
    }


def rms(values: Iterable[float]) -> float:
    vals = list(values)
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else 0.0
