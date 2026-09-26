"""Leader route around obstacles: A* on an inflated occupancy grid, then line-of-sight shortcuts.

This is the classical, map-based half of obstacle avoidance: only the leader follows the planned
route (with `clearance_m` to every obstacle); followers keep formation around it and dodge
locally (avoidance.py: classical potential field or the learned policy).
"""
from __future__ import annotations

import heapq
import math
from collections.abc import Sequence

import numpy as np

from .obstacles import ObstacleMap

SQRT2 = math.sqrt(2.0)


def _inflate(occ: np.ndarray, cells: float) -> np.ndarray:
    """Cells within `cells` grid units of an occupied cell (Euclidean)."""
    if not occ.any() or cells <= 0:
        return occ.copy()
    try:
        from scipy.ndimage import distance_transform_edt
        return distance_transform_edt(~occ) <= cells
    except ImportError:  # pragma: no cover - small fallback without scipy
        out = occ.copy()
        r = int(math.ceil(cells))
        ys, xs = np.nonzero(occ)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= cells * cells:
                    yy, xx = np.clip(ys + dy, 0, occ.shape[0] - 1), np.clip(xs + dx, 0, occ.shape[1] - 1)
                    out[yy, xx] = True
        return out


class Grid:
    def __init__(self, omap: ObstacleMap, points: Sequence[Sequence[float]], clearance_m: float,
                 res_m: float = 2.0, margin_m: float = 80.0) -> None:
        pts = np.asarray(points, dtype=float)
        self.res = res_m
        self.x0 = float(pts[:, 0].min() - margin_m)
        self.y0 = float(pts[:, 1].min() - margin_m)
        self.nx = int(math.ceil((pts[:, 0].max() + margin_m - self.x0) / res_m))
        self.ny = int(math.ceil((pts[:, 1].max() + margin_m - self.y0) / res_m))
        occ = omap.occupancy(self.x0, self.y0, self.nx, self.ny, res_m)
        self.blocked = _inflate(occ, clearance_m / res_m)

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (min(max(int((x - self.x0) / self.res), 0), self.nx - 1),
                min(max(int((y - self.y0) / self.res), 0), self.ny - 1))

    def centre(self, c: tuple[int, int]) -> tuple[float, float]:
        return (self.x0 + (c[0] + 0.5) * self.res, self.y0 + (c[1] + 0.5) * self.res)

    def free(self, c: tuple[int, int]) -> bool:
        return 0 <= c[0] < self.nx and 0 <= c[1] < self.ny and not self.blocked[c[1], c[0]]

    def line_free(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1) * 2
        xs = np.rint(np.linspace(a[0], b[0], n + 1)).astype(int)
        ys = np.rint(np.linspace(a[1], b[1], n + 1)).astype(int)
        return not self.blocked[ys, xs].any()

    def nearest_free(self, c: tuple[int, int], max_r: int = 30) -> tuple[int, int] | None:
        if self.free(c):
            return c
        for r in range(1, max_r + 1):
            ring = [(c[0] + dx, c[1] + dy) for dx in range(-r, r + 1) for dy in (-r, r)] + \
                   [(c[0] + dx, c[1] + dy) for dy in range(-r + 1, r) for dx in (-r, r)]
            ring = [q for q in ring if self.free(q)]
            if ring:
                return min(ring, key=lambda q: (q[0] - c[0]) ** 2 + (q[1] - c[1]) ** 2)
        return None


def plan_path(omap: ObstacleMap, start: Sequence[float], goal: Sequence[float], clearance_m: float = 8.0,
              res_m: float = 2.0, margin_m: float = 80.0) -> list[tuple[float, float]] | None:
    """Waypoints from start to goal keeping `clearance_m` from every obstacle (None if no route).

    The start and goal themselves may sit inside the inflated zone (a drone parked near a wall);
    the search then begins from the nearest free cell.
    """
    start, goal = (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))
    if omap.empty:
        return [start, goal]
    g = Grid(omap, [start, goal], clearance_m, res_m, margin_m)
    s, t = g.nearest_free(g.cell(*start)), g.nearest_free(g.cell(*goal))
    if s is None or t is None:
        return None
    if g.line_free(s, t):
        return [start, goal]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {s: 0.0}
    h = lambda c: math.hypot(c[0] - t[0], c[1] - t[1])  # noqa: E731
    open_ = [(h(s), 0.0, s)]
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]
    while open_:
        _, gc, c = heapq.heappop(open_)
        if c == t:
            break
        if gc > cost.get(c, math.inf):
            continue
        for dx, dy, w in steps:
            q = (c[0] + dx, c[1] + dy)
            if not g.free(q):
                continue
            if dx and dy and not (g.free((c[0] + dx, c[1])) and g.free((c[0], c[1] + dy))):
                continue  # no corner cutting
            nc = gc + w
            if nc < cost.get(q, math.inf):
                cost[q] = nc
                came[q] = c
                heapq.heappush(open_, (nc + h(q), nc, q))
    if t not in cost:
        return None
    cells = [t]
    while cells[-1] != s:
        cells.append(came[cells[-1]])
    cells.reverse()
    # line-of-sight shortcuts
    keep = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not g.line_free(cells[i], cells[j]):
            j -= 1
        keep.append(cells[j])
        i = j
    pts = [start] + [g.centre(c) for c in keep[1:-1]] + [goal]
    return pts


class PathFollower:
    """Pure pursuit along a polyline: velocity toward a look-ahead point, slowing near the end."""

    def __init__(self, waypoints: Sequence[Sequence[float]], lookahead_m: float = 15.0) -> None:
        self.p = np.asarray(waypoints, dtype=float)
        seg = np.diff(self.p, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.cum[-1])
        self.lookahead = lookahead_m
        self.s = 0.0  # progress, never decreases

    def project(self, x: float, y: float) -> float:
        """Arc length of the closest point on the path (searched forward of the current progress)."""
        best, best_s = math.inf, self.s
        for k in range(len(self.seg_len)):
            if self.cum[k + 1] < self.s - 1e-9:
                continue
            a, l = self.p[k], self.seg_len[k]
            if l < 1e-9:
                continue
            u = np.clip(((x - a[0]) * (self.p[k + 1][0] - a[0]) + (y - a[1]) * (self.p[k + 1][1] - a[1])) / (l * l), 0.0, 1.0)
            q = a + u * (self.p[k + 1] - a)
            d = math.hypot(q[0] - x, q[1] - y)
            s = self.cum[k] + u * l
            if d < best - 1e-9 and s >= self.s - 1e-9:
                best, best_s = d, s
        self.s = max(self.s, float(best_s))
        return self.s

    def point(self, s: float) -> np.ndarray:
        s = min(max(s, 0.0), self.length)
        k = int(np.searchsorted(self.cum, s, side="right") - 1)
        k = min(k, len(self.seg_len) - 1)
        l = self.seg_len[k]
        u = 0.0 if l < 1e-9 else (s - self.cum[k]) / l
        return self.p[k] + u * (self.p[k + 1] - self.p[k])

    def tangent(self, s: float) -> float:
        """ENU heading of the path at arc length s."""
        k = int(np.clip(np.searchsorted(self.cum, s, side="right") - 1, 0, len(self.seg_len) - 1))
        d = self.p[k + 1] - self.p[k]
        return math.atan2(d[1], d[0])

    def command(self, x: float, y: float, speed: float, slow_radius_m: float = 20.0) -> tuple[float, float, float]:
        """(vx, vy, remaining distance) toward the look-ahead point."""
        s = self.project(x, y)
        target = self.point(s + self.lookahead)
        dx, dy = target[0] - x, target[1] - y
        d = math.hypot(dx, dy)
        to_goal = math.hypot(self.p[-1][0] - x, self.p[-1][1] - y)
        v = speed * min(1.0, to_goal / slow_radius_m) if self.length - s < self.lookahead + slow_radius_m else speed
        if d < 1e-6:
            return 0.0, 0.0, self.length - s
        return v * dx / d, v * dy / d, self.length - s
