"""Static obstacles (buildings, trees) in the shared ENU frame: range sensing and clearance.

Obstacles are 2-D at the flight altitude: building footprints as polygons and trees as circles,
treated as taller than the swarm flies (drones go around, not over). The same map serves three
users: the range "sensor" every drone reads (ray casts, like a 2-D lidar), collision checks in
the simulators, and the leader's path planner (planner.py).

Everything is numpy-vectorised; a uniform grid index keeps each query to nearby shapes.
Coordinates are metres, (east, north).
"""
from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

import numpy as np

CELL_M = 25.0  # grid-index cell size


class ObstacleMap:
    def __init__(self, polygons: Iterable[Sequence[Sequence[float]]] = (), circles: Iterable[Sequence[float]] = (),
                 cell_m: float = CELL_M) -> None:
        self.polygons = [np.asarray(p, dtype=float).reshape(-1, 2) for p in polygons]
        self.polygons = [p[:-1] if len(p) > 3 and np.allclose(p[0], p[-1]) else p for p in self.polygons]
        if any(len(p) < 3 for p in self.polygons):
            raise ValueError("a polygon needs at least 3 vertices")
        self.circles = np.asarray(list(circles), dtype=float).reshape(-1, 3)
        segs = [np.hstack([p, np.roll(p, -1, axis=0)]) for p in self.polygons]
        self.segments = np.vstack(segs) if segs else np.zeros((0, 4))
        self.seg_poly = np.concatenate([np.full(len(p), k) for k, p in enumerate(self.polygons)]) \
            if self.polygons else np.zeros(0, dtype=int)
        self.poly_bbox = np.array([[p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()]
                                   for p in self.polygons]).reshape(-1, 4)
        self.cell = float(cell_m)
        self._seg_cells: dict[tuple[int, int], list[int]] = {}
        self._circ_cells: dict[tuple[int, int], list[int]] = {}
        for k, (x1, y1, x2, y2) in enumerate(self.segments):
            self._index(self._seg_cells, k, min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
        for k, (x, y, r) in enumerate(self.circles):
            self._index(self._circ_cells, k, x - r, y - r, x + r, y + r)
        self._seg_arr = {c: np.array(v) for c, v in self._seg_cells.items()}
        self._circ_arr = {c: np.array(v) for c, v in self._circ_cells.items()}

    # ------------------------------------------------------------------ index
    def _index(self, table: dict, k: int, x0: float, y0: float, x1: float, y1: float) -> None:
        c = self.cell
        for i in range(math.floor(x0 / c), math.floor(x1 / c) + 1):
            for j in range(math.floor(y0 / c), math.floor(y1 / c) + 1):
                table.setdefault((i, j), []).append(k)

    def _near(self, x: float, y: float, radius: float) -> tuple[np.ndarray, np.ndarray]:
        """Indices of segments and circles whose cells overlap the square [x +- radius, y +- radius]."""
        c = self.cell
        i0, i1 = math.floor((x - radius) / c), math.floor((x + radius) / c)
        j0, j1 = math.floor((y - radius) / c), math.floor((y + radius) / c)
        segs, circs = [], []
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                if (i, j) in self._seg_arr:
                    segs.append(self._seg_arr[(i, j)])
                if (i, j) in self._circ_arr:
                    circs.append(self._circ_arr[(i, j)])
        s = np.unique(np.concatenate(segs)) if segs else np.zeros(0, dtype=int)
        k = np.unique(np.concatenate(circs)) if circs else np.zeros(0, dtype=int)
        return s, k

    @property
    def empty(self) -> bool:
        return len(self.segments) == 0 and len(self.circles) == 0

    # ------------------------------------------------------------------ queries
    def raycast(self, x: float, y: float, angles: np.ndarray, max_range: float) -> np.ndarray:
        """Distance along each ray (ENU angle, from East counter-clockwise) to the first obstacle,
        capped at max_range. A ray starting inside a tree reads 0."""
        angles = np.asarray(angles, dtype=float)
        out = np.full(angles.shape, float(max_range))
        si, ci = self._near(x, y, max_range)
        if len(si) == 0 and len(ci) == 0:
            return out
        dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
        if len(si):
            s = self.segments[si]
            ax, ay = s[:, 0] - x, s[:, 1] - y
            ex, ey = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]
            den = dx * ey - dy * ex                       # cross(d, e)
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (ax * ey - ay * ex) / den             # cross(a, e) / cross(d, e)
                u = (ax * dy - ay * dx) / den             # cross(a, d) / cross(d, e)
            hit = (np.abs(den) > 1e-12) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
            out = np.minimum(out, np.where(hit, t, np.inf).min(axis=1))
        if len(ci):
            c = self.circles[ci]
            cx, cy, r = c[:, 0] - x, c[:, 1] - y, c[:, 2]
            b = dx * cx + dy * cy
            cc = cx * cx + cy * cy - r * r
            disc = b * b - cc
            with np.errstate(invalid="ignore"):
                t = b - np.sqrt(disc)
            t = np.where(cc <= 0.0, 0.0, t)               # starting inside a tree
            hit = (disc >= 0.0) & (t >= 0.0)
            out = np.minimum(out, np.where(hit, t, np.inf).min(axis=1))
        return out

    def clearance(self, x: float, y: float, search_m: float = 30.0) -> float:
        """Distance to the nearest obstacle surface (0 or less when inside one); search_m if none nearby."""
        best = float(search_m)
        si, ci = self._near(x, y, search_m)
        if len(si):
            s = self.segments[si]
            ex, ey = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]
            l2 = np.maximum(ex * ex + ey * ey, 1e-12)
            t = np.clip(((x - s[:, 0]) * ex + (y - s[:, 1]) * ey) / l2, 0.0, 1.0)
            d = np.hypot(s[:, 0] + t * ex - x, s[:, 1] + t * ey - y)
            best = min(best, float(d.min()))
        if len(ci):
            c = self.circles[ci]
            best = min(best, float((np.hypot(c[:, 0] - x, c[:, 1] - y) - c[:, 2]).min()))
        if self.inside_polygon(x, y):
            return -best
        return best

    def nearest_point(self, x: float, y: float, search_m: float) -> tuple[float, float] | None:
        """Vector from (x, y) to the closest obstacle surface point within search_m (None if none)."""
        best, vec = float(search_m), None
        si, ci = self._near(x, y, search_m)
        if len(si):
            s = self.segments[si]
            ex, ey = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]
            l2 = np.maximum(ex * ex + ey * ey, 1e-12)
            t = np.clip(((x - s[:, 0]) * ex + (y - s[:, 1]) * ey) / l2, 0.0, 1.0)
            qx, qy = s[:, 0] + t * ex - x, s[:, 1] + t * ey - y
            d = np.hypot(qx, qy)
            k = int(d.argmin())
            if d[k] < best:
                best, vec = float(d[k]), (float(qx[k]), float(qy[k]))
        if len(ci):
            c = self.circles[ci]
            cx, cy = c[:, 0] - x, c[:, 1] - y
            dc = np.hypot(cx, cy)
            d = dc - c[:, 2]
            k = int(d.argmin())
            if d[k] < best:
                f = d[k] / max(dc[k], 1e-9)
                best, vec = float(d[k]), (float(cx[k] * f), float(cy[k] * f))
        return vec

    def inside_polygon(self, x: float, y: float) -> bool:
        """Even-odd test against the building footprints whose bounding box holds the point."""
        if not len(self.poly_bbox):
            return False
        b = self.poly_bbox
        cand = np.nonzero((b[:, 0] <= x) & (x <= b[:, 2]) & (b[:, 1] <= y) & (y <= b[:, 3]))[0]
        for k in cand:
            p = self.polygons[k]
            x1, y1 = p[:, 0], p[:, 1]
            x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
            crosses = ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / np.where(y2 != y1, y2 - y1, 1e-12) + x1)
            if np.count_nonzero(crosses) % 2 == 1:
                return True
        return False

    def blocked(self, x: float, y: float, radius: float) -> bool:
        """True when a disc of this radius around the point touches or overlaps an obstacle."""
        return self.clearance(x, y, search_m=radius + 1.0) <= radius

    def occupancy(self, x0: float, y0: float, nx: int, ny: int, res: float) -> np.ndarray:
        """Boolean grid [ny, nx] of cells whose centre lies inside an obstacle (row j = y0 + (j+.5)*res)."""
        xs = x0 + (np.arange(nx) + 0.5) * res
        ys = y0 + (np.arange(ny) + 0.5) * res
        grid = np.zeros((ny, nx), dtype=bool)
        x1_, y1_ = x0 + nx * res, y0 + ny * res
        bb = self.poly_bbox
        hit = np.nonzero((bb[:, 2] >= x0) & (bb[:, 0] <= x1_) & (bb[:, 3] >= y0) & (bb[:, 1] <= y1_))[0] if len(bb) else []
        for k in hit:
            p, (bx0, by0, bx1, by1) = self.polygons[k], bb[k]
            i0, i1 = np.searchsorted(xs, bx0), np.searchsorted(xs, bx1, side="right")
            j0, j1 = np.searchsorted(ys, by0), np.searchsorted(ys, by1, side="right")
            if i0 >= i1 or j0 >= j1:
                continue
            gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
            inside = np.zeros(gx.shape, dtype=bool)
            x1, y1 = p[:, 0], p[:, 1]
            x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
            for a, b, c, d in zip(x1, y1, x2, y2):
                if b == d:
                    continue
                inside ^= ((b > gy) != (d > gy)) & (gx < (c - a) * (gy - b) / (d - b) + a)
            grid[j0:j1, i0:i1] |= inside
        c = self.circles
        near = c[(c[:, 0] + c[:, 2] >= x0) & (c[:, 0] - c[:, 2] <= x1_) & (c[:, 1] + c[:, 2] >= y0)
                 & (c[:, 1] - c[:, 2] <= y1_)] if len(c) else c
        for cx, cy, r in near:
            i0, i1 = np.searchsorted(xs, cx - r), np.searchsorted(xs, cx + r, side="right")
            j0, j1 = np.searchsorted(ys, cy - r), np.searchsorted(ys, cy + r, side="right")
            if i0 >= i1 or j0 >= j1:
                continue
            gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
            grid[j0:j1, i0:i1] |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        return grid


def rectangle(cx: float, cy: float, w: float, h: float, angle: float) -> np.ndarray:
    """Corners of a w x h rectangle centred at (cx, cy), rotated by angle [rad]."""
    c, s = math.cos(angle), math.sin(angle)
    pts = np.array([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]])
    return pts @ np.array([[c, s], [-s, c]]) + np.array([cx, cy])
