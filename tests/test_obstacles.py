"""Obstacle map queries and the leader's route planner."""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from swarm_agent.obstacles import ObstacleMap, rectangle
from swarm_agent.planner import PathFollower, plan_path

SQUARE = [(10, -5), (20, -5), (20, 5), (10, 5)]


def random_map(rng: random.Random) -> ObstacleMap:
    polys = [rectangle(rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(4, 20), rng.uniform(4, 20),
                       rng.uniform(0, math.pi)) for _ in range(12)]
    circles = [(rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(1, 4)) for _ in range(15)]
    return ObstacleMap(polys, circles)


def sphere_trace(m: ObstacleMap, x: float, y: float, ang: float, max_range: float) -> float:
    """Independent reference: march along the ray by the clearance until touching a surface."""
    t = 0.0
    while t < max_range:
        c = m.clearance(x + t * math.cos(ang), y + t * math.sin(ang), search_m=max_range)
        if c < 1e-4:
            return t
        t += c
    return max_range


def test_known_geometry():
    m = ObstacleMap([SQUARE], [(0.0, 10.0, 2.0)])
    d = m.raycast(0.0, 0.0, np.array([0.0, math.pi, math.pi / 2]), 30.0)
    assert d == pytest.approx([10.0, 30.0, 8.0])
    assert m.clearance(0.0, 0.0) == pytest.approx(8.0)          # nearest: the tree, 10 - 2
    assert m.clearance(15.0, 0.0) == pytest.approx(-5.0)        # inside the building
    assert m.inside_polygon(15.0, 0.0) and not m.inside_polygon(25.0, 0.0)
    assert m.blocked(0.0, 7.5, radius=1.0) and not m.blocked(-5.0, -5.0, radius=1.0)
    assert m.raycast(0.0, 10.0, np.array([0.0]), 30.0)[0] == 0.0  # inside a tree


def test_raycast_matches_sphere_tracing():
    rng = random.Random(3)
    checked = 0
    for _ in range(6):
        m = random_map(rng)
        for _ in range(15):
            x, y = rng.uniform(-70, 70), rng.uniform(-70, 70)
            if m.clearance(x, y) < 0.5:
                continue
            angles = np.linspace(0.0, 2 * math.pi, 24, endpoint=False)
            got = m.raycast(x, y, angles, 40.0)
            ref = [sphere_trace(m, x, y, a, 40.0) for a in angles]
            assert got == pytest.approx(ref, abs=2e-3)
            checked += 1
    assert checked > 50


def test_occupancy_grid_marks_inside_cells():
    m = ObstacleMap([SQUARE], [(0.0, 10.0, 2.0)])
    occ = m.occupancy(0.0, -10.0, 30, 30, 1.0)       # x 0..30, y -10..20
    assert occ[10, 15] and not occ[10, 25]           # (15.5, 0.5) inside, (25.5, 0.5) outside
    assert occ[19, 0] and not occ[19, 3]             # tree at (0, 10): cell (0.5, 9.5) in, (3.5, 9.5) out


def test_plan_goes_around_a_wall_with_clearance():
    wall = [(-50, 100), (50, 100), (50, 110), (-50, 110)]
    m = ObstacleMap([wall])
    path = plan_path(m, (0, 0), (0, 200), clearance_m=8.0)
    assert path is not None and path[0] == (0.0, 0.0) and path[-1] == (0.0, 200.0)
    pts = np.asarray(path)
    length = float(np.hypot(*np.diff(pts, axis=0).T).sum())
    assert length > 200.0
    for a, b in zip(pts[:-1], pts[1:]):
        for u in np.linspace(0, 1, 50):
            p = a + u * (b - a)
            assert m.clearance(*p) >= 8.0 - 2.0 * math.sqrt(2)  # clearance minus one grid cell diagonal


def test_plan_is_straight_when_nothing_is_in_the_way_and_none_when_enclosed():
    m = ObstacleMap([[(30, -5), (40, -5), (40, 5), (30, 5)]])
    assert plan_path(m, (0, 20), (0, 200)) == [(0.0, 20.0), (0.0, 200.0)]
    box = [[(-30, -30), (30, -30), (30, -26), (-30, -26)], [(-30, 26), (30, 26), (30, 30), (-30, 30)],
           [(-30, -30), (-26, -30), (-26, 30), (-30, 30)], [(26, -30), (30, -30), (30, 30), (26, 30)]]
    assert plan_path(ObstacleMap(box), (0, 0), (0, 200), clearance_m=4.0) is None


def test_path_follower_reaches_the_end():
    f = PathFollower([(0, 0), (0, 100), (80, 100)])
    x, y = 0.0, 0.0
    for _ in range(2000):
        vx, vy, left = f.command(x, y, 5.0)
        x, y = x + vx * 0.1, y + vy * 0.1
    assert math.hypot(x - 80, y - 100) < 0.5 and left == pytest.approx(0.0, abs=1e-6)
    assert f.tangent(50.0) == pytest.approx(math.pi / 2) and f.tangent(150.0) == pytest.approx(0.0)
