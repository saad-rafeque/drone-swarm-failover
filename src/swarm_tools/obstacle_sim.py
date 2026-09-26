"""Formation flight through buildings and trees: a fast numpy simulator for RL training and for
comparing avoiders (none / classical potential field / learned policy) on identical scenarios.

The leader flies the A* route (planner.py). Followers use the same V-slot law as the agent
(formation.py: leader velocity feed-forward + saturated P on the slot error), plus an avoider
correction, then the agent's safety layer (drone-to-drone repulsion from safety.py, speed limit).
Point-mass dynamics as in puresim.py: first-order velocity tracking, acceleration limit and a
wind-like Gauss-Markov drift. Tunables of the swarm (spacing, gains, separation) come from
config/swarm.yaml, so this simulator and the agent share one source of truth.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from swarm_agent.avoidance import (ERR_SCALE_M, MAX_CORRECTION_MPS, N_NEIGHBORS, N_RAYS, NEIGHBOR_RANGE_M, OBS_DIM,
                                   RAY_ANGLES, RAY_RANGE_M, V_SCALE_MPS, AvoidInput, action_to_correction)
from swarm_agent.config import Config
from swarm_agent.formation import assign_slots, v_slot_body
from swarm_agent.obstacles import ObstacleMap, rectangle
from swarm_agent.planner import PathFollower, plan_path


@dataclass(frozen=True)
class SimCfg:
    n_drones: int = 10
    spacing_m: float = 10.0
    half_angle_rad: float = math.radians(45.0)
    cruise_mps: float = 5.0
    gain: float = 1.0
    max_corr_mps: float = 3.0
    max_speed_mps: float = 10.0
    min_sep_m: float = 5.0
    rep_factor: float = 1.5
    rep_gain_mps: float = 3.0
    dt: float = 0.05
    control_every: int = 2                  # policy at 10 Hz, physics at 20 Hz
    tau_s: float = 0.35
    max_acc: float = 4.0
    drift_sigma_mps: float = 0.15
    drift_tau_s: float = 3.0
    crash_clearance_m: float = 0.6          # drone touching a wall or a tree
    collide_m: float = 1.0                  # two drones touching
    leader_clearance_m: float = 8.0
    route_m: tuple[float, float] = (300.0, 600.0)
    corridor_half_m: float = 150.0
    keep_clear_m: float = 70.0
    building_density: tuple[float, float] = (0.0, 2.0)   # buildings per hectare
    tree_density: tuple[float, float] = (0.0, 1.0)       # tree clusters per hectare
    trees_per_cluster: tuple[int, int] = (3, 13)
    settle_s: float = 15.0
    heading_rate_dps: float = 20.0
    end_on_crash: bool = True


def sim_cfg_from(config: Config, **overrides) -> SimCfg:
    f, s, m = config.formation, config.safety, config.mission
    base = SimCfg(n_drones=config.swarm.num_drones, spacing_m=f.spacing_m,
                  half_angle_rad=math.radians(f.v_half_angle_deg), cruise_mps=m.cruise_speed_mps, gain=f.pos_gain,
                  max_corr_mps=f.max_correction_mps, max_speed_mps=f.max_speed_mps, min_sep_m=s.min_separation_m,
                  rep_factor=s.repulsion_factor, rep_gain_mps=s.repulsion_gain)
    return replace(base, **overrides)


@dataclass
class Scenario:
    seed: int
    omap: ObstacleMap
    start: tuple[float, float]
    goal: tuple[float, float]
    path: list[tuple[float, float]]
    polygons: list[np.ndarray] = field(default_factory=list)
    circles: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))


def make_scenario(seed: int, cfg: SimCfg) -> Scenario:
    """Random route with buildings (rectangles, street-aligned or not) and tree clusters (circles)."""
    rng = np.random.default_rng(seed)
    start = np.zeros(2)
    for attempt in range(40):
        scale = 1.0 if attempt < 20 else 0.5          # thin out if routes keep getting blocked
        L = rng.uniform(*cfg.route_m)
        th = rng.uniform(-math.pi, math.pi)
        u, n = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
        goal = L * u
        area_ha = L * 2 * cfg.corridor_half_m / 1e4

        def clear_of_ends(c: np.ndarray, r: float) -> bool:
            return (np.hypot(*(c - start)) - r > cfg.keep_clear_m) and (np.hypot(*(c - goal)) - r > cfg.keep_clear_m)

        polys = []
        for _ in range(rng.poisson(scale * rng.uniform(*cfg.building_density) * area_ha)):
            c = rng.uniform(0, L) * u + rng.uniform(-cfg.corridor_half_m, cfg.corridor_half_m) * n
            w, h = rng.uniform(8, 30, 2)
            if not clear_of_ends(c, 0.5 * math.hypot(w, h)):
                continue
            ang = th + (rng.normal(0, 0.05) if rng.random() < 0.5 else rng.uniform(0, math.pi))
            polys.append(rectangle(c[0], c[1], w, h, ang))
        circles = []
        for _ in range(rng.poisson(scale * rng.uniform(*cfg.tree_density) * area_ha)):
            c0 = rng.uniform(0, L) * u + rng.uniform(-cfg.corridor_half_m, cfg.corridor_half_m) * n
            sigma = rng.uniform(4, 15)
            for _ in range(rng.integers(*cfg.trees_per_cluster)):
                c = c0 + rng.normal(0, sigma, 2)
                r = rng.uniform(1.5, 3.5)
                if clear_of_ends(c, r):
                    circles.append((c[0], c[1], r))
        omap = ObstacleMap(polys, circles)
        path = plan_path(omap, start, goal, clearance_m=cfg.leader_clearance_m)
        if path is not None:
            return Scenario(seed, omap, (0.0, 0.0), (float(goal[0]), float(goal[1])), path, omap.polygons,
                            omap.circles)
    raise RuntimeError(f"no route found for scenario seed {seed}")


def _clamp_rows(v: np.ndarray, limit: float) -> np.ndarray:
    n = np.hypot(v[:, 0], v[:, 1])
    k = np.where(n > limit, limit / np.maximum(n, 1e-12), 1.0)
    return v * k[:, None]


class FormationSim:
    """One episode at a time. Drone 0 is the leader (ID 1); drones 1..N-1 are the followers (agents)."""

    def __init__(self, cfg: SimCfg, seed: int = 0) -> None:
        self.cfg = cfg
        self.n = cfg.n_drones
        self.nf = cfg.n_drones - 1
        ids = list(range(1, self.n + 1))
        slots = assign_slots(ids, 1)
        self.slot_body = np.array([v_slot_body(slots[i], cfg.spacing_m, cfg.half_angle_rad) for i in ids[1:]])
        self.rng = np.random.default_rng(seed)
        self.scenario: Scenario | None = None

    # ------------------------------------------------------------------ episode
    def reset(self, scenario: Scenario | int | None = None) -> np.ndarray:
        c = self.cfg
        if scenario is None:
            scenario = int(self.rng.integers(0, 2**31 - 1))
        self.scenario = scenario if isinstance(scenario, Scenario) else make_scenario(scenario, c)
        sc = self.scenario
        self.route = PathFollower(sc.path)
        self.heading = self.route.tangent(0.0)
        self.pos = np.zeros((self.n, 2))
        self.pos[0] = sc.start
        self.pos[1:] = sc.start + self._rotate(self.slot_body, self.heading)
        self.vel = np.zeros((self.n, 2))
        self.drift = np.zeros((self.n, 2))
        self.alive = np.ones(self.n, dtype=bool)
        self.t = 0.0
        self.t_arrive: float | None = None
        self.err_prev = np.zeros(self.nf)
        self._sensed: tuple | None = None
        self.t_limit = self.route.length / c.cruise_mps * 1.6 + 40.0
        self.stats = {"crash_obstacle": [], "crash_drone": [], "min_clearance": math.inf, "min_sep": math.inf,
                      "sep_violation_s": 0.0, "err_sum": 0.0, "err_n": 0, "effort_sum": 0.0, "effort_n": 0,
                      "rms_final": None}
        return self.observe()

    @staticmethod
    def _rotate(v: np.ndarray, a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.stack([v[:, 0] * c - v[:, 1] * s, v[:, 0] * s + v[:, 1] * c], axis=1)

    def slots(self) -> np.ndarray:
        return self.pos[0] + self._rotate(self.slot_body, self.heading)

    def desired(self) -> np.ndarray:
        """Formation-law desired velocity of every follower (formation.follower_velocity, horizontal)."""
        c = self.cfg
        corr = _clamp_rows(c.gain * (self.slots() - self.pos[1:]), c.max_corr_mps)
        return _clamp_rows(self.vel[0] + corr, c.max_speed_mps)

    def _sense(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Range readings [nf, N_RAYS], closest-obstacle vectors [nf, 2] and whether one is in range [nf].
        Cached until the state changes (the next _advance or reset)."""
        if self._sensed is not None and self._sensed[0] == self.t:
            return self._sensed[1]
        omap = self.scenario.omap
        rays = np.full((self.nf, N_RAYS), RAY_RANGE_M)
        near = np.zeros((self.nf, 2))
        has = np.zeros(self.nf, dtype=bool)
        ang = self.heading + RAY_ANGLES
        for k in range(self.nf):
            if not self.alive[k + 1]:
                continue
            x, y = self.pos[k + 1]
            rays[k] = omap.raycast(x, y, ang, RAY_RANGE_M)
            v = omap.nearest_point(x, y, RAY_RANGE_M)
            if v is not None:
                near[k], has[k] = v, True
        self._sensed = (self.t, (rays, near, has))
        return rays, near, has

    def inputs(self) -> list[AvoidInput]:
        """Per-follower avoider inputs (what the onboard agent would assemble)."""
        slots, vdes = self.slots(), self.desired()
        rays, near, has = self._sense()
        out = []
        for k in range(self.nf):
            i = k + 1
            nb = [(*(self.pos[j] - self.pos[i]), *(self.vel[j] - self.vel[i]))
                  for j in range(self.n) if j != i and self.alive[j]]
            out.append(AvoidInput(self.heading, tuple(vdes[k]), tuple(self.vel[i]), tuple(slots[k] - self.pos[i]),
                                  rays[k], tuple(near[k]) if has[k] else None, nb))
        return out

    def observe(self) -> np.ndarray:
        """avoidance.observation() for every follower at once (identical values, vectorised)."""
        rays, near, has = self._sense()
        self.near_dist = np.where(has, np.hypot(near[:, 0], near[:, 1]), RAY_RANGE_M)
        c, s = math.cos(-self.heading), math.sin(-self.heading)

        def rot(v: np.ndarray) -> np.ndarray:
            return np.stack([v[..., 0] * c - v[..., 1] * s, v[..., 0] * s + v[..., 1] * c], axis=-1)

        nf = self.nf
        obs = np.zeros((nf, OBS_DIM))
        obs[:, 0:2] = rot(self.desired()) / V_SCALE_MPS
        obs[:, 2:4] = rot(self.vel[1:]) / V_SCALE_MPS
        obs[:, 4:6] = np.clip(rot(self.slots() - self.pos[1:]) / ERR_SCALE_M, -1.5, 1.5)
        obs[:, 6:6 + N_RAYS] = np.clip(rays / RAY_RANGE_M, 0.0, 1.0)
        obs[:, 6 + N_RAYS:8 + N_RAYS] = np.where(has[:, None], rot(near) / RAY_RANGE_M, 0.0)
        rel = self.pos[None, :, :] - self.pos[1:, None, :]            # [nf, n, 2] other - self
        relv = self.vel[None, :, :] - self.vel[1:, None, :]
        valid = self.alive[None, :] & (np.hypot(rel[..., 0], rel[..., 1]) < NEIGHBOR_RANGE_M)
        valid[np.arange(nf), np.arange(1, self.n)] = False
        key = np.where(valid, rel[..., 0] ** 2 + rel[..., 1] ** 2, np.inf)
        order = np.argsort(key, axis=1, kind="stable")[:, :N_NEIGHBORS]
        rows = np.arange(nf)
        base = 8 + N_RAYS
        for m in range(min(N_NEIGHBORS, order.shape[1])):
            j = order[:, m]
            ok = np.isfinite(key[rows, j])[:, None]
            obs[:, base + 5 * m: base + 5 * m + 2] = np.where(ok, rot(rel[rows, j]) / NEIGHBOR_RANGE_M, 0.0)
            obs[:, base + 5 * m + 2: base + 5 * m + 4] = np.where(ok, rot(relv[rows, j]) / V_SCALE_MPS, 0.0)
            obs[:, base + 5 * m + 4] = ok[:, 0]
        obs[~self.alive[1:]] = 0.0
        return obs.astype(np.float32)

    # ------------------------------------------------------------------ stepping
    def step_with(self, avoider) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
        """Advance one control period using an avoider object (evaluation)."""
        dv = np.array([avoider.correction(inp) if self.alive[k + 1] else (0.0, 0.0)
                       for k, inp in enumerate(self.inputs())])
        return self._advance(dv, actions=None)

    def step(self, actions: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
        """Advance one control period with normalized policy actions [nf, 2] (training)."""
        dv = np.array([action_to_correction(a, self.heading) for a in actions])
        return self._advance(dv, actions=np.asarray(actions, dtype=float))

    def _advance(self, dv: np.ndarray, actions: np.ndarray | None):
        c = self.cfg
        omap = self.scenario.omap
        crashed_before = ~self.alive.copy()
        r_act = c.rep_factor * c.min_sep_m
        # Only drones that could reach an obstacle within this control period need the per-substep check:
        # from the last sensing, travel is at most (max speed + drift margin) * period.
        reach = c.crash_clearance_m + (c.max_speed_mps + 0.5) * c.dt * c.control_every
        watch = self.alive.copy()
        watch[1:] &= self.near_dist <= reach
        if self.alive[1:].any():
            self.stats["min_clearance"] = min(self.stats["min_clearance"], float(self.near_dist[self.alive[1:]].min()))
        for _ in range(c.control_every):
            # leader: route + heading
            vx, vy, left = self.route.command(*self.pos[0], c.cruise_mps)
            target = self.route.tangent(self.route.s)
            dh = math.remainder(target - self.heading, 2 * math.pi)
            step = math.radians(c.heading_rate_dps) * c.dt
            self.heading += max(-step, min(step, dh))
            cmd = np.zeros((self.n, 2))
            cmd[0] = (vx, vy)
            cmd[1:] = _clamp_rows(self.desired() + dv, c.max_speed_mps)
            # safety layer: drone-to-drone repulsion (safety.repulsion), then the speed limit
            diff = self.pos[:, None, :] - self.pos[None, :, :]
            d = np.hypot(diff[..., 0], diff[..., 1])
            np.fill_diagonal(d, np.inf)
            d[:, ~self.alive] = np.inf
            strength = np.where(d < r_act, c.rep_gain_mps * (r_act - d) / (r_act - c.min_sep_m), 0.0)
            push = (diff * (strength / np.maximum(d, 1e-9))[..., None]).sum(axis=1)
            cmd = _clamp_rows(cmd + push, c.max_speed_mps)
            # dynamics
            a = _clamp_rows((cmd + self.drift - self.vel) / c.tau_s, c.max_acc)
            self.vel = np.where(self.alive[:, None], self.vel + a * c.dt, 0.0)
            self.pos = self.pos + self.vel * c.dt
            if c.drift_sigma_mps > 0:
                k = math.exp(-c.dt / c.drift_tau_s)
                self.drift = self.drift * k + c.drift_sigma_mps * math.sqrt(1 - k * k) * self.rng.normal(size=(self.n, 2))
            self.t += c.dt
            # collisions and safety statistics
            for i in np.nonzero(watch & self.alive)[0]:
                cl = omap.clearance(*self.pos[i], search_m=RAY_RANGE_M)
                self.stats["min_clearance"] = min(self.stats["min_clearance"], cl)
                if cl < c.crash_clearance_m:
                    self.alive[i] = False
                    self.stats["crash_obstacle"].append((round(self.t, 2), int(i) + 1))
            alive = np.nonzero(self.alive)[0]
            if len(alive) > 1:
                sub = np.hypot(*(self.pos[alive][:, None, :] - self.pos[alive][None, :, :]).transpose(2, 0, 1))
                np.fill_diagonal(sub, np.inf)
                mn = float(sub.min())
                self.stats["min_sep"] = min(self.stats["min_sep"], mn)
                if mn < c.min_sep_m:
                    self.stats["sep_violation_s"] += c.dt
                hit = np.nonzero((sub < c.collide_m).any(axis=1))[0]
                for i in alive[hit]:
                    self.alive[i] = False
                    self.stats["crash_drone"].append((round(self.t, 2), int(i) + 1))
            if left < 1.0 and self.t_arrive is None:
                self.t_arrive = self.t
        # rewards for the followers
        new_crash = ~self.alive[1:] & ~crashed_before[1:]
        err = np.hypot(*(self.slots() - self.pos[1:]).T)
        obs = self.observe()
        clear = obs[:, 6:6 + N_RAYS].min(axis=1) * RAY_RANGE_M
        dd = np.hypot(*(self.pos[1:, None, :] - self.pos[None, :, :]).transpose(2, 0, 1))   # [nf, n]
        dd[np.arange(self.nf), np.arange(1, self.n)] = np.inf
        dd[:, ~self.alive] = np.inf
        nearest = np.where(self.alive[1:], dd.min(axis=1), np.inf)
        a2 = (actions ** 2).sum(axis=1) if actions is not None else (dv ** 2).sum(axis=1) / MAX_CORRECTION_MPS ** 2
        rew = (0.1 * np.exp(-err / 10.0) + 0.02 * np.clip(self.err_prev - err, -2.0, 2.0) - 0.01 * a2
               - 0.05 * np.clip(1.0 - clear / 4.0, 0.0, 1.0) - 0.2 * np.clip(1.0 - nearest / c.min_sep_m, 0.0, 1.0))
        self.err_prev = err
        rew = np.where(self.alive[1:], rew, 0.0)
        rew = np.where(new_crash, -10.0, rew)
        live = self.alive[1:] & ~crashed_before[1:]
        if live.any():
            self.stats["err_sum"] += float(err[live].sum())
            self.stats["err_n"] += int(live.sum())
            self.stats["effort_sum"] += float(np.hypot(*dv[live].T).sum())
            self.stats["effort_n"] += int(live.sum())
        settled = self.t_arrive is not None and self.t - self.t_arrive >= c.settle_s
        over = settled or self.t >= self.t_limit or not self.alive[0] or \
            (c.end_on_crash and bool(new_crash.any())) or not self.alive[1:].any()
        terminated = new_crash.copy()
        truncated = np.where(over, ~terminated, False)
        if over and settled:
            fl = self.alive[1:]
            self.stats["rms_final"] = float(np.sqrt((err[fl] ** 2).mean())) if fl.any() else None
            self.stats["stuck"] = int((err[fl] > 10.0).sum())
        info = {"over": over, "settled": settled, "t": self.t}
        return obs, rew.astype(np.float32), terminated, truncated, info

    def summary(self) -> dict:
        s = self.stats
        crash_free = not s["crash_obstacle"] and not s["crash_drone"]
        restored = s["rms_final"] is not None and s["rms_final"] < 3.0
        return {
            "seed": self.scenario.seed, "success": crash_free and restored, "crash_free": crash_free,
            "restored": restored, "stuck": s.get("stuck"),
            "crashes_obstacle": len(s["crash_obstacle"]), "crashes_drone": len(s["crash_drone"]),
            "min_clearance_m": round(s["min_clearance"], 2), "min_sep_m": round(s["min_sep"], 2),
            "sep_violation_s": round(s["sep_violation_s"], 2),
            "mean_slot_err_m": round(s["err_sum"] / max(s["err_n"], 1), 2),
            "final_rms_m": None if s["rms_final"] is None else round(s["rms_final"], 2),
            "mean_correction_mps": round(s["effort_sum"] / max(s["effort_n"], 1), 3),
            "arrive_s": None if self.t_arrive is None else round(self.t_arrive, 1),
            "route_m": round(self.route.length, 1), "obstacles": len(self.scenario.polygons) + len(self.scenario.circles),
        }


def run_episode(cfg: SimCfg, scenario: Scenario | int, avoider, seed: int = 0) -> dict:
    sim = FormationSim(replace(cfg, end_on_crash=False), seed=seed)
    sim.reset(scenario)
    while True:
        _, _, _, _, info = sim.step_with(avoider)
        if info["over"]:
            return sim.summary()
