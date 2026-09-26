"""Separation (repulsion) and geofence."""
from __future__ import annotations

import math

import pytest

from swarm_agent.safety import geofence_limit, inside_geofence, min_pairwise_distance, repulsion


def test_min_pairwise_distance():
    d, pair = min_pairwise_distance({1: (0, 0, 0), 2: (10, 0, 0), 3: (0, 3, 4)})
    assert d == pytest.approx(5.0) and pair == (1, 3)
    assert min_pairwise_distance({1: (0, 0, 0)}) == (math.inf, None)


def test_repulsion_zero_outside_activation_radius():
    assert repulsion((0, 0, 0), [(7.6, 0, 0)], 5.0, 1.5, 3.0) == (0.0, 0.0, 0.0)


def test_repulsion_strength_and_direction():
    v = repulsion((0, 0, 0), [(5.0, 0, 0)], 5.0, 1.5, 3.0)
    assert v == pytest.approx((-3.0, 0.0, 0.0))           # gain at min separation, pointing away
    closer = repulsion((0, 0, 0), [(2.5, 0, 0)], 5.0, 1.5, 3.0)
    assert closer[0] < -3.0                                # stronger inside min separation
    two = repulsion((0, 0, 0), [(0, 6.0, 0), (0, -6.0, 0)], 5.0, 1.5, 3.0)
    assert two == pytest.approx((0.0, 0.0, 0.0))           # symmetric neighbours cancel
    assert repulsion((1, 1, 1), [(1, 1, 1)], 5.0, 1.5, 3.0)[2] > 0  # co-located: push up


def test_geofence_inside_unchanged():
    v = (3.0, -2.0, 1.0)
    assert geofence_limit((100.0, 100.0, 30.0), v, 1500.0, 50.0, 3.0) == v
    assert inside_geofence((100.0, 100.0, 30.0), 1500.0, 50.0)
    assert not inside_geofence((1600.0, 0.0, 30.0), 1500.0, 50.0)


def test_geofence_blocks_outward_and_pushes_back():
    v = geofence_limit((1499.0, 0.0, 30.0), (5.0, 2.0, 0.0), 1500.0, 50.0, 3.0)
    assert v[0] < 0.0 and v[1] == pytest.approx(2.0)       # outward removed, push inward, tangent kept
    inward = geofence_limit((1499.0, 0.0, 30.0), (-5.0, 0.0, 0.0), 1500.0, 50.0, 3.0)
    assert inward[0] < -5.0                                 # already inward: kept, plus push


def test_geofence_ceiling():
    assert geofence_limit((0.0, 0.0, 48.0), (0.0, 0.0, 2.0), 1500.0, 50.0, 3.0)[2] < 0.0
    assert geofence_limit((0.0, 0.0, 46.0), (0.0, 0.0, 2.0), 1500.0, 50.0, 3.0)[2] == 2.0
