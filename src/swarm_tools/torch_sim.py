"""Batched PyTorch version of obstacle_sim.FormationSim: thousands of formation missions at once on a GPU.

It is a line-by-line port of the numpy simulator (same leader route following, V-slot law, drone
repulsion, point-mass dynamics, crash rules, observation and reward) so that a policy trained here
behaves the same in obstacle_sim and in the agent. tests/test_torch_sim.py checks the two step for
step on identical scenarios. Two deliberate differences:
  * range rays are cast only against the K nearest walls and trees of each drone (k_seg, k_circ);
    with K at least the number of shapes the results are identical, and `ray_overflow` counts the
    cases where more than K shapes lay within ray range;
  * obstacle clearance is checked for every drone at every physics step (the numpy version skips
    drones that cannot reach an obstacle within the control period - same outcome, less work).
Scenarios (buildings, trees, the leader's A* route) are generated on the CPU with
obstacle_sim.make_scenario and packed into padded tensors (a "pool") before training.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import torch

from swarm_agent.avoidance import (ERR_SCALE_M, MAX_CORRECTION_MPS, N_NEIGHBORS, N_RAYS, NEIGHBOR_RANGE_M, OBS_DIM,
                                   RAY_ANGLES, RAY_RANGE_M, V_SCALE_MPS)
from swarm_agent.formation import assign_slots, v_slot_body
from swarm_tools.obstacle_sim import Scenario, SimCfg

LOOKAHEAD_M = 15.0      # planner.PathFollower defaults
SLOW_RADIUS_M = 20.0
INF = float("inf")


# ---------------------------------------------------------------------------------------------- pool
def pack_scenarios(scenarios: list[Scenario]) -> dict[str, np.ndarray]:
    """Padded arrays for a list of scenarios (save with np.savez_compressed, load with ScenarioPool.load)."""
    P = len(scenarios)
    M = max(1, max(len(s.omap.segments) for s in scenarios))
    C = max(1, max(len(s.omap.circles) for s in scenarios))
    W = max(len(s.path) for s in scenarios)
    seg = np.zeros((P, M, 4), np.float32)
    seg_ok = np.zeros((P, M), bool)
    circ = np.zeros((P, C, 3), np.float32)
    circ_ok = np.zeros((P, C), bool)
    path = np.zeros((P, W, 2), np.float64)
    n_wp = np.zeros(P, np.int64)
    seeds = np.zeros(P, np.int64)
    for k, s in enumerate(scenarios):
        m, c, w = len(s.omap.segments), len(s.omap.circles), len(s.path)
        seg[k, :m], seg_ok[k, :m] = s.omap.segments, True
        circ[k, :c], circ_ok[k, :c] = s.omap.circles, True
        p = np.asarray(s.path, dtype=np.float64)
        path[k, :w], path[k, w:] = p, p[-1]
        n_wp[k], seeds[k] = w, s.seed
    return {"seg": seg, "seg_ok": seg_ok, "circ": circ, "circ_ok": circ_ok, "path": path, "n_wp": n_wp, "seeds": seeds}


@dataclass
class ScenarioPool:
    seg: torch.Tensor       # [P, M, 4]
    seg_ok: torch.Tensor    # [P, M]
    circ: torch.Tensor      # [P, C, 3]
    circ_ok: torch.Tensor   # [P, C]
    path: torch.Tensor      # [P, W, 2]
    n_wp: torch.Tensor      # [P]
    seeds: torch.Tensor     # [P]

    @classmethod
    def from_arrays(cls, a: dict, device: torch.device | str, dtype: torch.dtype = torch.float32) -> "ScenarioPool":
        t = lambda x, dt=None: torch.as_tensor(np.asarray(x), device=device, dtype=dt)  # noqa: E731
        return cls(t(a["seg"], dtype), t(a["seg_ok"], torch.bool), t(a["circ"], dtype), t(a["circ_ok"], torch.bool),
                   t(a["path"], dtype), t(a["n_wp"], torch.long), t(a["seeds"], torch.long))

    @classmethod
    def load(cls, path: str, device, dtype=torch.float32) -> "ScenarioPool":
        with np.load(path) as z:
            return cls.from_arrays({k: z[k] for k in z.files}, device, dtype)

    @property
    def size(self) -> int:
        return int(self.seg.shape[0])


def _clamp_rows(v: torch.Tensor, limit) -> torch.Tensor:
    n = torch.linalg.vector_norm(v, dim=-1, keepdim=True)
    return v * torch.where(n > limit, limit / n.clamp_min(1e-12), torch.ones_like(n))


def _rot(v: torch.Tensor, c: torch.Tensor, s: torch.Tensor) -> torch.Tensor:
    """Rotate [..., 2] vectors by the angle whose cos/sin broadcast against v[..., 0]."""
    return torch.stack([v[..., 0] * c - v[..., 1] * s, v[..., 0] * s + v[..., 1] * c], dim=-1)


# ---------------------------------------------------------------------------------------------- sim
class BatchedFormationSim:
    """n_envs independent episodes. Drone 0 of each is the leader; drones 1..N-1 are the agents."""

    def __init__(self, cfg: SimCfg, pool: ScenarioPool, n_envs: int, device: torch.device | str = "cpu",
                 seed: int = 0, k_seg: int = 64, k_circ: int = 48, dtype: torch.dtype = torch.float32) -> None:
        self.cfg, self.pool, self.B, self.dev, self.dtype = cfg, pool, n_envs, torch.device(device), dtype
        self.n, self.nf = cfg.n_drones, cfg.n_drones - 1
        ids = list(range(1, self.n + 1))
        slots = assign_slots(ids, 1)
        self.slot_body = torch.tensor([v_slot_body(slots[i], cfg.spacing_m, cfg.half_angle_rad) for i in ids[1:]],
                                      device=self.dev, dtype=dtype)
        self.ray_ang = torch.as_tensor(RAY_ANGLES, device=self.dev, dtype=dtype)
        self.k_seg = min(k_seg, pool.seg.shape[1])
        self.k_circ = min(k_circ, pool.circ.shape[1])
        self.gen = torch.Generator(device=self.dev)
        self.gen.manual_seed(seed)
        B, n, nf, W = n_envs, self.n, self.nf, pool.path.shape[1]
        z = lambda *shape: torch.zeros(*shape, device=self.dev, dtype=dtype)  # noqa: E731
        self.scen = torch.zeros(B, dtype=torch.long, device=self.dev)
        self.seg, self.seg_ok = z(B, pool.seg.shape[1], 4), torch.zeros(B, pool.seg.shape[1], dtype=torch.bool, device=self.dev)
        self.circ, self.circ_ok = z(B, pool.circ.shape[1], 3), torch.zeros(B, pool.circ.shape[1], dtype=torch.bool, device=self.dev)
        self.path, self.n_wp = z(B, W, 2), torch.zeros(B, dtype=torch.long, device=self.dev)
        self.cum, self.seg_len, self.length = z(B, W), z(B, max(W - 1, 1)), z(B)
        self.pos, self.vel, self.drift = z(B, n, 2), z(B, n, 2), z(B, n, 2)
        self.alive = torch.ones(B, n, dtype=torch.bool, device=self.dev)
        self.heading, self.s, self.t = z(B), z(B), z(B)
        self.t_arrive = torch.full((B,), INF, device=self.dev, dtype=dtype)
        self.t_limit, self.err_prev = z(B), z(B, nf)
        self.st = {k: z(B) for k in ("crash_obstacle", "crash_drone", "sep_violation_s", "err_sum", "err_n",
                                     "effort_sum", "effort_n")}
        self.st["min_sep"] = torch.full((B,), INF, device=self.dev, dtype=dtype)
        self.st["min_clearance"] = torch.full((B,), INF, device=self.dev, dtype=dtype)
        self.ray_overflow = 0
        self.near_dist = z(B, nf)

    # ------------------------------------------------------------------ scenarios / reset
    def reset(self, env_mask: torch.Tensor | None = None, scenario_idx: torch.Tensor | None = None) -> torch.Tensor:
        """Reset the masked envs (all if None) onto random pool scenarios (or the given indices); returns obs."""
        m = torch.ones(self.B, dtype=torch.bool, device=self.dev) if env_mask is None else env_mask
        ids = torch.nonzero(m).squeeze(1)
        if len(ids):
            if scenario_idx is None:
                scenario_idx = torch.randint(0, self.pool.size, (len(ids),), generator=self.gen, device=self.dev)
            self._load(ids, scenario_idx)
        return self.observe()

    def _load(self, ids: torch.Tensor, k: torch.Tensor) -> None:
        p, c, dt = self.pool, self.cfg, self.dtype
        self.scen[ids] = k
        self.seg[ids], self.seg_ok[ids] = p.seg[k].to(dt), p.seg_ok[k]
        self.circ[ids], self.circ_ok[ids] = p.circ[k].to(dt), p.circ_ok[k]
        path = p.path[k].to(dt)
        self.path[ids], self.n_wp[ids] = path, p.n_wp[k]
        seg_len = torch.linalg.vector_norm(path[:, 1:] - path[:, :-1], dim=-1)
        self.seg_len[ids] = seg_len
        cum = torch.cat([torch.zeros(len(ids), 1, device=self.dev, dtype=dt), torch.cumsum(seg_len, dim=1)], dim=1)
        self.cum[ids] = cum
        self.length[ids] = cum[:, -1]
        self.s[ids] = 0.0
        self.heading[ids] = self._tangent_at(ids, torch.zeros(len(ids), device=self.dev, dtype=dt))
        start = path[:, 0]
        hc, hs = torch.cos(self.heading[ids])[:, None], torch.sin(self.heading[ids])[:, None]
        pos = torch.zeros(len(ids), self.n, 2, device=self.dev, dtype=dt)
        pos[:, 0] = start
        pos[:, 1:] = start[:, None] + _rot(self.slot_body[None].expand(len(ids), -1, -1), hc, hs)
        self.pos[ids], self.vel[ids], self.drift[ids] = pos, 0.0, 0.0
        self.alive[ids] = True
        self.t[ids] = 0.0
        self.t_arrive[ids] = INF
        self.err_prev[ids] = 0.0
        self.t_limit[ids] = self.length[ids] / c.cruise_mps * 1.6 + 40.0
        for key, v in self.st.items():
            v[ids] = INF if key in ("min_sep", "min_clearance") else 0.0

    # ------------------------------------------------------------------ leader route (planner.PathFollower)
    def _seg_index(self, cum: torch.Tensor, n_wp: torch.Tensor, s: torch.Tensor) -> torch.Tensor:
        k = torch.searchsorted(cum, s[:, None].contiguous(), right=True).squeeze(1) - 1
        return torch.minimum(k.clamp(min=0), n_wp - 2)

    def _tangent_at(self, ids: torch.Tensor, s: torch.Tensor) -> torch.Tensor:
        k = self._seg_index(self.cum[ids], self.n_wp[ids], s)
        path = self.path[ids]
        d = path.gather(1, (k + 1)[:, None, None].expand(-1, 1, 2)).squeeze(1) - \
            path.gather(1, k[:, None, None].expand(-1, 1, 2)).squeeze(1)
        return torch.atan2(d[:, 1], d[:, 0])

    def _point(self, s: torch.Tensor) -> torch.Tensor:
        s = torch.minimum(s.clamp(min=0.0), self.length)
        k = self._seg_index(self.cum, self.n_wp, s)
        l = self.seg_len.gather(1, k[:, None]).squeeze(1)
        c0 = self.cum.gather(1, k[:, None]).squeeze(1)
        u = torch.where(l < 1e-9, torch.zeros_like(s), (s - c0) / l.clamp_min(1e-12))
        pk = self.path.gather(1, k[:, None, None].expand(-1, 1, 2)).squeeze(1)
        pk1 = self.path.gather(1, (k + 1)[:, None, None].expand(-1, 1, 2)).squeeze(1)
        return pk + u[:, None] * (pk1 - pk)

    def _route(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Leader velocity command, remaining route length and the route heading (all [B])."""
        c = self.cfg
        a, b = self.path[:, :-1], self.path[:, 1:]
        ab = b - a
        l = self.seg_len
        x = self.pos[:, 0]
        u = (((x[:, None] - a) * ab).sum(-1) / (l * l).clamp_min(1e-18)).clamp(0.0, 1.0)
        q = a + u[..., None] * ab
        d = torch.linalg.vector_norm(q - x[:, None], dim=-1)
        s_k = self.cum[:, :-1] + u * l
        real = torch.arange(l.shape[1], device=self.dev)[None] < (self.n_wp - 1)[:, None]
        s0 = self.s[:, None] - 1e-9
        valid = real & (self.cum[:, 1:] >= s0) & (l >= 1e-9) & (s_k >= s0)
        d = torch.where(valid, d, torch.full_like(d, INF))
        k = d.argmin(dim=1)
        best = torch.where(valid.any(dim=1), s_k.gather(1, k[:, None]).squeeze(1), self.s)
        self.s = torch.maximum(self.s, best)
        target = self._point(self.s + LOOKAHEAD_M)
        dxy = target - x
        dist = torch.linalg.vector_norm(dxy, dim=-1)
        goal = self.path.gather(1, (self.n_wp - 1)[:, None, None].expand(-1, 1, 2)).squeeze(1)
        to_goal = torch.linalg.vector_norm(goal - x, dim=-1)
        slow = (self.length - self.s) < (LOOKAHEAD_M + SLOW_RADIUS_M)
        v = torch.where(slow, c.cruise_mps * (to_goal / SLOW_RADIUS_M).clamp(max=1.0), torch.full_like(dist, c.cruise_mps))
        cmd = torch.where((dist < 1e-6)[:, None], torch.zeros_like(dxy), v[:, None] * dxy / dist.clamp_min(1e-12)[:, None])
        tangent = self._tangent_at(torch.arange(self.B, device=self.dev), self.s)
        return cmd, self.length - self.s, tangent

    # ------------------------------------------------------------------ geometry helpers
    def slots(self) -> torch.Tensor:
        hc, hs = torch.cos(self.heading)[:, None], torch.sin(self.heading)[:, None]
        return self.pos[:, :1] + _rot(self.slot_body[None], hc, hs)

    def desired(self) -> torch.Tensor:
        c = self.cfg
        corr = _clamp_rows(c.gain * (self.slots() - self.pos[:, 1:]), c.max_corr_mps)
        return _clamp_rows(self.vel[:, :1] + corr, c.max_speed_mps)

    def _shape_distances(self, p: torch.Tensor):
        """Per-shape distances from points p [B, K, 2]: segments [B, K, M], vectors to the nearest points on
        them [B, K, M, 2], circles [B, K, C] and their vectors [B, K, C, 2] (padding = inf)."""
        a = self.seg[:, None, :, 0:2]
        e = self.seg[:, None, :, 2:4] - a
        pp = p[:, :, None]
        l2 = (e * e).sum(-1).clamp_min(1e-12)
        t = (((pp - a) * e).sum(-1) / l2).clamp(0.0, 1.0)
        qv = a + t[..., None] * e - pp
        ds = torch.where(self.seg_ok[:, None], torch.linalg.vector_norm(qv, dim=-1), torch.full_like(t, INF))
        cv = self.circ[:, None, :, 0:2] - pp
        dc0 = torch.linalg.vector_norm(cv, dim=-1)
        r = self.circ[:, None, :, 2]
        dcirc = torch.where(self.circ_ok[:, None], dc0 - r, torch.full_like(dc0, INF))
        cvec = cv * ((dc0 - r) / dc0.clamp_min(1e-9))[..., None]
        return ds, qv, dcirc, cvec

    def _clearance(self, p: torch.Tensor) -> torch.Tensor:
        ds, _, dc, _ = self._shape_distances(p)
        return torch.minimum(ds.min(-1).values, dc.min(-1).values)

    def _sense(self):
        """Rays [B, nf, R], nearest-obstacle vectors [B, nf, 2] and in-range flags [B, nf] for the followers."""
        pf = self.pos[:, 1:]
        B, nf = pf.shape[:2]
        ds, qv, dc, cvec = self._shape_distances(pf)
        # nearest obstacle point within ray range (obstacles.nearest_point: segments first, then circles if closer)
        ms, ks = ds.min(-1)
        mc, kc = dc.min(-1)
        vs = qv.gather(2, ks[..., None, None].expand(-1, -1, 1, 2)).squeeze(2)
        vc = cvec.gather(2, kc[..., None, None].expand(-1, -1, 1, 2)).squeeze(2)
        use_c = mc < ms
        near = torch.where(use_c[..., None], vc, vs)
        dist = torch.where(use_c, mc, ms)
        has = dist < RAY_RANGE_M
        # rays against the K nearest segments and circles
        ang = self.heading[:, None, None] + self.ray_ang[None, None]          # [B, 1, R]
        dx, dy = torch.cos(ang)[..., None], torch.sin(ang)[..., None]         # [B, 1, R, 1]
        out = torch.full((B, nf, N_RAYS), RAY_RANGE_M, device=self.dev, dtype=self.dtype)
        if self.seg_ok.any():
            dsk, idx = torch.topk(ds, self.k_seg, dim=-1, largest=False)
            self.ray_overflow += int(((ds < RAY_RANGE_M).sum(-1) > self.k_seg).sum())
            sg = self.seg[:, None].expand(-1, nf, -1, -1).gather(2, idx[..., None].expand(-1, -1, -1, 4))
            ok = torch.isfinite(dsk)[:, :, None]                                  # [B, nf, 1, K]
            ax = (sg[..., 0] - pf[..., 0:1])[:, :, None]
            ay = (sg[..., 1] - pf[..., 1:2])[:, :, None]
            ex = (sg[..., 2] - sg[..., 0])[:, :, None]
            ey = (sg[..., 3] - sg[..., 1])[:, :, None]
            den = dx * ey - dy * ex                                              # [B, nf, R, K]
            safe = torch.where(den.abs() > 1e-12, den, torch.ones_like(den))
            t = (ax * ey - ay * ex) / safe
            u = (ax * dy - ay * dx) / safe
            hit = ok & (den.abs() > 1e-12) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
            out = torch.minimum(out, torch.where(hit, t, torch.full_like(t, INF)).min(-1).values)
        if self.circ_ok.any():
            dck, idc = torch.topk(dc, self.k_circ, dim=-1, largest=False)
            self.ray_overflow += int(((dc < RAY_RANGE_M).sum(-1) > self.k_circ).sum())
            cg = self.circ[:, None].expand(-1, nf, -1, -1).gather(2, idc[..., None].expand(-1, -1, -1, 3))
            ok = torch.isfinite(dck)[:, :, None]
            cx = (cg[..., 0] - pf[..., 0:1])[:, :, None]
            cy = (cg[..., 1] - pf[..., 1:2])[:, :, None]
            r = cg[..., 2][:, :, None]
            b = dx * cx + dy * cy
            cc = cx * cx + cy * cy - r * r
            disc = b * b - cc
            t = b - torch.sqrt(disc.clamp_min(0.0))
            t = torch.where(cc <= 0.0, torch.zeros_like(t), t)
            hit = ok & (disc >= 0.0) & (t >= 0.0)
            out = torch.minimum(out, torch.where(hit, t, torch.full_like(t, INF)).min(-1).values)
        dead = ~self.alive[:, 1:]
        out = torch.where(dead[..., None], torch.full_like(out, RAY_RANGE_M), out)
        has = has & ~dead
        return out, torch.where(has[..., None], near, torch.zeros_like(near)), has, torch.where(has, dist, torch.full_like(dist, RAY_RANGE_M))

    def observe(self) -> torch.Tensor:
        rays, near, has, ndist = self._sense()
        self.near_dist = ndist
        hc, hs = torch.cos(-self.heading), torch.sin(-self.heading)
        c1, s1 = hc[:, None], hs[:, None]
        B, nf = self.B, self.nf
        obs = torch.zeros(B, nf, OBS_DIM, device=self.dev, dtype=self.dtype)
        obs[..., 0:2] = _rot(self.desired(), c1, s1) / V_SCALE_MPS
        obs[..., 2:4] = _rot(self.vel[:, 1:], c1, s1) / V_SCALE_MPS
        obs[..., 4:6] = (_rot(self.slots() - self.pos[:, 1:], c1, s1) / ERR_SCALE_M).clamp(-1.5, 1.5)
        obs[..., 6:6 + N_RAYS] = (rays / RAY_RANGE_M).clamp(0.0, 1.0)
        obs[..., 6 + N_RAYS:8 + N_RAYS] = _rot(near, c1, s1) / RAY_RANGE_M
        rel = self.pos[:, None, :, :] - self.pos[:, 1:, None, :]                  # [B, nf, n, 2]
        relv = self.vel[:, None, :, :] - self.vel[:, 1:, None, :]
        dist = torch.linalg.vector_norm(rel, dim=-1)
        valid = self.alive[:, None, :] & (dist < NEIGHBOR_RANGE_M)
        eye = torch.zeros(nf, self.n, dtype=torch.bool, device=self.dev)
        eye[torch.arange(nf), torch.arange(1, self.n)] = True
        valid = valid & ~eye[None]
        key = torch.where(valid, rel[..., 0] ** 2 + rel[..., 1] ** 2, torch.full_like(dist, INF))
        order = torch.sort(key, dim=-1, stable=True).indices[..., :N_NEIGHBORS]  # [B, nf, 3]
        base = 8 + N_RAYS
        c2, s2 = hc[:, None], hs[:, None]
        for m in range(min(N_NEIGHBORS, order.shape[-1])):
            j = order[..., m]
            ok = torch.isfinite(key.gather(2, j[..., None]).squeeze(-1))[..., None]
            rp = rel.gather(2, j[..., None, None].expand(-1, -1, 1, 2)).squeeze(2)
            rv = relv.gather(2, j[..., None, None].expand(-1, -1, 1, 2)).squeeze(2)
            obs[..., base + 5 * m: base + 5 * m + 2] = torch.where(ok, _rot(rp, c2, s2) / NEIGHBOR_RANGE_M, torch.zeros_like(rp))
            obs[..., base + 5 * m + 2: base + 5 * m + 4] = torch.where(ok, _rot(rv, c2, s2) / V_SCALE_MPS, torch.zeros_like(rv))
            obs[..., base + 5 * m + 4] = ok[..., 0].to(self.dtype)
        obs = torch.where(self.alive[:, 1:, None], obs, torch.zeros_like(obs))
        return obs

    # ------------------------------------------------------------------ step
    def step(self, actions: torch.Tensor, auto_reset: bool = True):
        """actions [B, nf, 2] in [-1, 1] (heading frame). Returns (obs, reward, terminated, truncated, over,
        terminal_obs, finished) where obs already holds the new episodes of finished envs and
        terminal_obs [B, nf, OBS] holds their last observation; `finished` lists summaries."""
        c = self.cfg
        a = actions.to(self.dtype).clamp(-1.0, 1.0)
        hc, hs = torch.cos(self.heading)[:, None], torch.sin(self.heading)[:, None]
        dv = _rot(a * MAX_CORRECTION_MPS, hc, hs)                                  # ENU corrections [B, nf, 2]
        alive_before = self.alive.clone()
        r_act = c.rep_factor * c.min_sep_m
        eye = torch.eye(self.n, dtype=torch.bool, device=self.dev)[None]
        for _ in range(c.control_every):
            vl, left, tangent = self._route()
            dh = torch.remainder(tangent - self.heading + math.pi, 2 * math.pi) - math.pi
            step = math.radians(c.heading_rate_dps) * c.dt
            self.heading = self.heading + dh.clamp(-step, step)
            cmd = torch.empty_like(self.pos)
            cmd[:, 0] = vl
            cmd[:, 1:] = _clamp_rows(self.desired() + dv, c.max_speed_mps)
            diff = self.pos[:, :, None, :] - self.pos[:, None, :, :]
            d = torch.linalg.vector_norm(diff, dim=-1)
            d = torch.where(eye | ~self.alive[:, None, :], torch.full_like(d, INF), d)
            strength = torch.where(d < r_act, c.rep_gain_mps * (r_act - d) / (r_act - c.min_sep_m), torch.zeros_like(d))
            push = (diff * (strength / d.clamp_min(1e-9))[..., None]).sum(2)
            cmd = _clamp_rows(cmd + push, c.max_speed_mps)
            acc = _clamp_rows((cmd + self.drift - self.vel) / c.tau_s, c.max_acc)
            self.vel = torch.where(self.alive[..., None], self.vel + acc * c.dt, torch.zeros_like(self.vel))
            self.pos = self.pos + self.vel * c.dt
            if c.drift_sigma_mps > 0:
                k = math.exp(-c.dt / c.drift_tau_s)
                noise = torch.randn(self.drift.shape, generator=self.gen, device=self.dev, dtype=self.dtype)
                self.drift = self.drift * k + c.drift_sigma_mps * math.sqrt(1 - k * k) * noise
            self.t = self.t + c.dt
            # crashes into obstacles, then between drones (same order as obstacle_sim)
            clr = self._clearance(self.pos)
            live_clr = torch.where(self.alive, clr, torch.full_like(clr, INF))
            self.st["min_clearance"] = torch.minimum(self.st["min_clearance"], live_clr.min(-1).values)
            hit_obs = self.alive & (clr < c.crash_clearance_m)
            self.st["crash_obstacle"] += hit_obs.sum(-1).to(self.dtype)
            self.alive = self.alive & ~hit_obs
            dd = torch.linalg.vector_norm(self.pos[:, :, None] - self.pos[:, None], dim=-1)
            pair = self.alive[:, :, None] & self.alive[:, None, :] & ~eye
            dd = torch.where(pair, dd, torch.full_like(dd, INF))
            mn = dd.flatten(1).min(-1).values
            self.st["min_sep"] = torch.minimum(self.st["min_sep"], mn)
            self.st["sep_violation_s"] += (mn < c.min_sep_m).to(self.dtype) * c.dt
            hit_dr = self.alive & (dd < c.collide_m).any(-1)
            self.st["crash_drone"] += hit_dr.sum(-1).to(self.dtype)
            self.alive = self.alive & ~hit_dr
            arrive = (left < 1.0) & torch.isinf(self.t_arrive)
            self.t_arrive = torch.where(arrive, self.t, self.t_arrive)
        # rewards
        af, ab = self.alive[:, 1:], alive_before[:, 1:]
        new_crash = ab & ~af
        err = torch.linalg.vector_norm(self.slots() - self.pos[:, 1:], dim=-1)
        obs = self.observe()
        clear = (obs[..., 6:6 + N_RAYS].min(-1).values * RAY_RANGE_M)
        rel = self.pos[:, 1:, None] - self.pos[:, None]
        ddf = torch.linalg.vector_norm(rel, dim=-1)
        eyef = torch.zeros(self.nf, self.n, dtype=torch.bool, device=self.dev)
        eyef[torch.arange(self.nf), torch.arange(1, self.n)] = True
        ddf = torch.where(eyef[None] | ~self.alive[:, None, :], torch.full_like(ddf, INF), ddf)
        nearest = torch.where(af, ddf.min(-1).values, torch.full_like(err, INF))
        a2 = (a * a).sum(-1)
        rew = (0.1 * torch.exp(-err / 10.0) + 0.02 * (self.err_prev - err).clamp(-2.0, 2.0) - 0.01 * a2
               - 0.05 * (1.0 - clear / 4.0).clamp(0.0, 1.0) - 0.2 * (1.0 - nearest / c.min_sep_m).clamp(0.0, 1.0))
        self.err_prev = err
        rew = torch.where(af, rew, torch.zeros_like(rew))
        rew = torch.where(new_crash, torch.full_like(rew, -10.0), rew)
        live = af & ab
        self.st["err_sum"] += torch.where(live, err, torch.zeros_like(err)).sum(-1)
        self.st["err_n"] += live.sum(-1).to(self.dtype)
        self.st["effort_sum"] += torch.where(live, torch.linalg.vector_norm(dv, dim=-1), torch.zeros_like(err)).sum(-1)
        self.st["effort_n"] += live.sum(-1).to(self.dtype)
        settled = torch.isfinite(self.t_arrive) & (self.t - self.t_arrive >= c.settle_s)
        over = settled | (self.t >= self.t_limit) | ~self.alive[:, 0] | ~af.any(-1)
        if c.end_on_crash:
            over = over | new_crash.any(-1)
        terminated = new_crash
        truncated = over[:, None] & ~terminated
        finished = self._summaries(over, settled, err) if bool(over.any()) else []
        terminal_obs = obs.clone()
        if auto_reset and bool(over.any()):
            obs = torch.where(over[:, None, None], self.reset(over), obs)
        return obs, rew, terminated, truncated, over, terminal_obs, finished

    def _summaries(self, over: torch.Tensor, settled: torch.Tensor, err: torch.Tensor) -> list[dict]:
        out = []
        af = self.alive[:, 1:]
        sq = torch.where(af, err * err, torch.zeros_like(err)).sum(-1) / af.sum(-1).clamp_min(1)
        rms = torch.sqrt(sq)
        stuck = (af & (err > 10.0)).sum(-1)
        ids = torch.nonzero(over).squeeze(1).tolist()
        g = {k: v.detach().cpu() for k, v in self.st.items()}
        rms_c, st_c, set_c = rms.cpu(), stuck.cpu(), settled.cpu()
        seeds = self.pool.seeds[self.scen].cpu()
        for i in ids:
            crash_free = g["crash_obstacle"][i] == 0 and g["crash_drone"][i] == 0
            rms_final = float(rms_c[i]) if bool(set_c[i]) and bool(af[i].any()) else None
            out.append({"seed": int(seeds[i]), "crash_free": bool(crash_free),
                        "restored": rms_final is not None and rms_final < 3.0,
                        "success": bool(crash_free) and rms_final is not None and rms_final < 3.0,
                        "crashes": int(g["crash_obstacle"][i] + g["crash_drone"][i]), "stuck": int(st_c[i]),
                        "sep_violation_s": float(g["sep_violation_s"][i]), "min_sep_m": float(g["min_sep"][i]),
                        "mean_slot_err_m": float(g["err_sum"][i] / max(float(g["err_n"][i]), 1.0)),
                        "final_rms_m": rms_final})
        return out
