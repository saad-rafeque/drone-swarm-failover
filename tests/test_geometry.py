"""Global -> shared ENU conversion (docs/SPECIFICATION.md §6 'Frames — critical') and vector helpers."""
from __future__ import annotations

import math
import random

import pytest

from swarm_agent.geometry import (WGS84_A, WGS84_E2, EnuFrame, GeoPoint, body_to_enu, clamp_norm,
                                  clamp_xy_z, dist, ecef_to_geodetic, geodetic_to_ecef, heading_of,
                                  norm, norm_xy)

ORIGIN = GeoPoint(47.397742, 8.545594, 488.0)


def test_origin_maps_to_zero():
    e, n, u = EnuFrame(ORIGIN).to_enu(ORIGIN)
    assert abs(e) < 1e-6 and abs(n) < 1e-6 and abs(u) < 1e-6


def test_small_offsets_match_independent_curvature_formula():
    """1 km north / east of the origin, compared with meridian/prime-vertical radii (not ECEF)."""
    frame = EnuFrame(ORIGIN)
    lat = math.radians(ORIGIN.lat_deg)
    s2 = math.sin(lat) ** 2
    m_rad = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * s2) ** 1.5
    n_rad = WGS84_A / math.sqrt(1 - WGS84_E2 * s2)
    dlat = math.degrees(1000.0 / (m_rad + ORIGIN.alt_m))
    e, n, _ = frame.to_enu(GeoPoint(ORIGIN.lat_deg + dlat, ORIGIN.lon_deg, ORIGIN.alt_m))
    assert abs(n - 1000.0) < 0.05 and abs(e) < 1e-3
    dlon = math.degrees(1000.0 / ((n_rad + ORIGIN.alt_m) * math.cos(lat)))
    e, n, _ = frame.to_enu(GeoPoint(ORIGIN.lat_deg, ORIGIN.lon_deg + dlon, ORIGIN.alt_m))
    assert abs(e - 1000.0) < 0.05
    # A parallel of latitude curves poleward of the tangent plane's east axis:
    # north offset ~ d^2 tan(lat) / (2 (N + h)) = 0.085 m here.
    expected_n = 1000.0 ** 2 * math.tan(lat) / (2 * (n_rad + ORIGIN.alt_m))
    assert abs(n - expected_n) < 0.005


def test_up_is_altitude_difference_nearby():
    _, _, u = EnuFrame(ORIGIN).to_enu(GeoPoint(ORIGIN.lat_deg, ORIGIN.lon_deg, ORIGIN.alt_m + 30.0))
    assert abs(u - 30.0) < 1e-6


def test_round_trip_random_points_within_2km():
    frame = EnuFrame(ORIGIN)
    rng = random.Random(7)
    for _ in range(500):
        enu = (rng.uniform(-2000, 2000), rng.uniform(-2000, 2000), rng.uniform(-50, 150))
        back = frame.to_enu(frame.to_geodetic(enu))
        assert dist(back, enu) < 1e-3


def test_ecef_round_trip():
    for p in (ORIGIN, GeoPoint(-33.9, 151.2, 10.0), GeoPoint(0.0, 0.0, 0.0), GeoPoint(60.0, -120.0, 3000.0)):
        q = ecef_to_geodetic(*geodetic_to_ecef(p))
        assert abs(q.lat_deg - p.lat_deg) < 1e-9 and abs(q.lon_deg - p.lon_deg) < 1e-9
        assert abs(q.alt_m - p.alt_m) < 1e-4


def test_shared_frame_preserves_true_distances():
    """ENU is a rigid transform of ECEF, so inter-drone distances are exact."""
    frame = EnuFrame(ORIGIN)
    a = GeoPoint(47.3981, 8.5461, 520.0)
    b = GeoPoint(47.3970, 8.5449, 505.0)
    true = dist(geodetic_to_ecef(a), geodetic_to_ecef(b))
    assert abs(dist(frame.to_enu(a), frame.to_enu(b)) - true) < 1e-6


def test_local_frames_must_not_be_compared_directly():
    """Two drones whose local frames start at their own home give the WRONG separation when their
    local positions are compared; converting both to the shared frame gives the right one."""
    home_a = GeoPoint(47.397742, 8.545594, 488.0)
    home_b = EnuFrame(home_a).to_geodetic((10.0, 0.0, 0.0))  # 10 m east of A
    a_now = EnuFrame(home_a).to_geodetic((0.0, 50.0, 30.0))   # A flew 50 m north, 30 m up
    b_now = EnuFrame(home_b).to_geodetic((0.0, 50.0, 30.0))   # B flew the same local path
    local_a = EnuFrame(home_a).to_enu(a_now)
    local_b = EnuFrame(home_b).to_enu(b_now)
    assert dist(local_a, local_b) < 1e-3                      # wrong: looks co-located
    shared = EnuFrame(ORIGIN)
    assert abs(dist(shared.to_enu(a_now), shared.to_enu(b_now)) - 10.0) < 0.01  # right


def test_vector_helpers():
    assert norm((3.0, 4.0, 12.0)) == pytest.approx(13.0)
    assert norm_xy((3.0, 4.0, 12.0)) == pytest.approx(5.0)
    assert clamp_norm((3.0, 4.0, 0.0), 2.5) == pytest.approx((1.5, 2.0, 0.0))
    assert clamp_norm((0.0, 0.0, 0.0), 1.0) == (0.0, 0.0, 0.0)
    assert clamp_xy_z((6.0, 8.0, 5.0), 5.0, 2.0) == pytest.approx((3.0, 4.0, 2.0))
    assert clamp_xy_z((0.0, 0.0, -5.0), 5.0, 3.0, 1.5) == pytest.approx((0.0, 0.0, -1.5))
    assert clamp_xy_z((1.0, 1.0, -1.0), 5.0, 3.0) == pytest.approx((1.0, 1.0, -1.0))
    assert heading_of(0.0, 1.0) == pytest.approx(math.pi / 2)
    assert heading_of(1.0, 0.0) == pytest.approx(0.0)


@pytest.mark.parametrize("heading,expected", [
    (0.0, (10.0, 5.0)),               # facing east: forward=east, left=north
    (math.pi / 2, (-5.0, 10.0)),      # facing north: forward=north, left=west
])
def test_body_to_enu(heading, expected):
    assert body_to_enu(10.0, 5.0, heading) == pytest.approx(expected)
