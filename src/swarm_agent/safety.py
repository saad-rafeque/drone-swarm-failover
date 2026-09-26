"""Separation and geofence (pure Python)."""
from __future__ import annotations

import itertools
import math
from collections.abc import Iterable, Mapping

from .geometry import Vec3, add, dist, scale, sub


def min_pairwise_distance(positions: Mapping[int, Vec3]) -> tuple[float, tuple[int, int] | None]:
    """Smallest 3-D distance between any two drones, and which pair."""
    best, pair = math.inf, None
    for (a, pa), (b, pb) in itertools.combinations(positions.items(), 2):
        d = dist(pa, pb)
        if d < best:
            best, pair = d, (a, b)
    return best, pair


def repulsion(my_pos: Vec3, others: Iterable[Vec3], min_sep_m: float, factor: float, gain_mps: float) -> Vec3:
    """Velocity pushing away from every neighbour closer than factor * min_sep_m.

    Linear in penetration: 0 at the activation radius, gain_mps at min_sep_m, and larger
    still inside min_sep_m (docs/SPECIFICATION.md §6: repulsion when closer than 1.5 x min separation).
    """
    r_act = factor * min_sep_m
    span = r_act - min_sep_m
    out: Vec3 = (0.0, 0.0, 0.0)
    for p in others:
        away = sub(my_pos, p)
        d = math.sqrt(away[0] ** 2 + away[1] ** 2 + away[2] ** 2)
        if d >= r_act:
            continue
        strength = gain_mps * (r_act - d) / span
        if d < 1e-6:  # exactly co-located: push up, deterministic
            out = add(out, (0.0, 0.0, strength))
        else:
            out = add(out, scale(away, strength / d))
    return out


def inside_geofence(pos: Vec3, radius_m: float, max_alt_m: float) -> bool:
    return math.hypot(pos[0], pos[1]) <= radius_m and pos[2] <= max_alt_m


def geofence_limit(pos: Vec3, vel: Vec3, radius_m: float, max_alt_m: float, margin_m: float,
                   gain: float = 1.0) -> Vec3:
    """Adjust a velocity command so the drone stays inside the fence (origin-centred cylinder).

    Inside (radius - margin) and below (max_alt - margin) the command is unchanged. Beyond
    that, the outward component is removed and a push back proportional to the penetration
    is added.
    """
    vx, vy, vz = vel
    r = math.hypot(pos[0], pos[1])
    r_lim = radius_m - margin_m
    if r > r_lim and r > 0.0:
        ux, uy = pos[0] / r, pos[1] / r
        outward = vx * ux + vy * uy
        if outward > 0.0:
            vx, vy = vx - outward * ux, vy - outward * uy
        push = gain * (r - r_lim)
        vx, vy = vx - push * ux, vy - push * uy
    z_lim = max_alt_m - margin_m
    if pos[2] > z_lim:
        vz = min(vz, -gain * (pos[2] - z_lim))
    return (vx, vy, vz)
