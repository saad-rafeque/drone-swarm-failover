"""Global (WGS-84 lat/lon/alt) <-> one shared ENU frame, plus small 3-D vector helpers.

Every drone converts its OWN global position into a single ENU frame anchored at
config.origin. Each drone's MAVROS local frame has its own origin, so local positions of
different drones are never compared directly (docs/SPECIFICATION.md §6).

Vectors are plain tuples (east, north, up) — faster than numpy for 3-element math and
free of any dependency.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]

# WGS-84 ellipsoid
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

ZERO: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True, slots=True)
class GeoPoint:
    lat_deg: float
    lon_deg: float
    alt_m: float


def geodetic_to_ecef(p: GeoPoint) -> Vec3:
    lat = math.radians(p.lat_deg)
    lon = math.radians(p.lon_deg)
    s = math.sin(lat)
    c = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    return (
        (n + p.alt_m) * c * math.cos(lon),
        (n + p.alt_m) * c * math.sin(lon),
        (n * (1.0 - WGS84_E2) + p.alt_m) * s,
    )


def ecef_to_geodetic(x: float, y: float, z: float) -> GeoPoint:
    """Iterative inverse; converges to sub-millimetre in a few iterations away from the poles."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    n = WGS84_A
    for _ in range(6):
        s = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
        lat = math.atan2(z + WGS84_E2 * n * s, p)
    s = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * s * s)
    alt = p * math.cos(lat) + (z + WGS84_E2 * n * s) * s - n
    return GeoPoint(math.degrees(lat), math.degrees(lon), alt)


class EnuFrame:
    """Shared ENU frame anchored at a geodetic origin."""

    def __init__(self, origin: GeoPoint) -> None:
        self.origin = origin
        lat = math.radians(origin.lat_deg)
        lon = math.radians(origin.lon_deg)
        self._sl, self._cl = math.sin(lat), math.cos(lat)
        self._so, self._co = math.sin(lon), math.cos(lon)
        self._x0, self._y0, self._z0 = geodetic_to_ecef(origin)

    def to_enu(self, p: GeoPoint) -> Vec3:
        x, y, z = geodetic_to_ecef(p)
        dx, dy, dz = x - self._x0, y - self._y0, z - self._z0
        sl, cl, so, co = self._sl, self._cl, self._so, self._co
        return (
            -so * dx + co * dy,
            -sl * co * dx - sl * so * dy + cl * dz,
            cl * co * dx + cl * so * dy + sl * dz,
        )

    def to_geodetic(self, enu: Vec3) -> GeoPoint:
        e, n, u = enu
        sl, cl, so, co = self._sl, self._cl, self._so, self._co
        dx = -so * e - sl * co * n + cl * co * u
        dy = co * e - sl * so * n + cl * so * u
        dz = cl * n + sl * u
        return ecef_to_geodetic(self._x0 + dx, self._y0 + dy, self._z0 + dz)


# ---------------------------------------------------------------- vector helpers (ENU tuples)

def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, k: float) -> Vec3:
    return (a[0] * k, a[1] * k, a[2] * k)


def norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def norm_xy(a: Vec3) -> float:
    return math.hypot(a[0], a[1])


def dist(a: Vec3, b: Vec3) -> float:
    return norm(sub(a, b))


def clamp_norm(a: Vec3, max_norm: float) -> Vec3:
    """Scale `a` down so its length is at most max_norm (direction preserved)."""
    n = norm(a)
    if n <= max_norm or n == 0.0:
        return a
    return scale(a, max_norm / n)


def clamp_xy_z(a: Vec3, max_xy: float, max_z: float) -> Vec3:
    """Limit horizontal speed to max_xy (direction preserved) and |vertical| to max_z."""
    h = math.hypot(a[0], a[1])
    k = max_xy / h if h > max_xy else 1.0
    return (a[0] * k, a[1] * k, max(-max_z, min(max_z, a[2])))


def heading_of(dx: float, dy: float) -> float:
    """ENU heading [rad]: angle from East, counter-clockwise (East=0, North=+pi/2)."""
    return math.atan2(dy, dx)


def body_to_enu(forward: float, left: float, heading: float) -> tuple[float, float]:
    """Rotate a (forward, left) body offset into (east, north) for the given ENU heading."""
    c, s = math.cos(heading), math.sin(heading)
    return (forward * c - left * s, forward * s + left * c)
