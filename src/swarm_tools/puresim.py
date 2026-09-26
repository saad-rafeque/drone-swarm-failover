"""Point-mass swarm simulator (no PX4, no ROS) that runs the real AgentCore for every drone.

Models: first-order velocity tracking with acceleration limits, ground contact, battery drain,
a heartbeat network with latency / jitter / random loss and blockable directed links
(partitions, master link loss, link drops), and hard kills. Heartbeats pass through the
binary codec, so quantisation is exercised as well.
"""
from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field

from swarm_agent.agent_core import AgentCore, Command, FlightMode, OwnState
from swarm_agent.config import Config
from swarm_agent.formation import assign_slots, initial_layout, rms, slot_position
from swarm_agent.geometry import ZERO, Vec3, dist, heading_of
from swarm_agent.heartbeat import Heartbeat, Phase, Role, decode, encode
from swarm_agent.safety import min_pairwise_distance


@dataclass
class Dynamics:
    tau_s: float = 0.35        # velocity response time constant
    max_acc_xy: float = 4.0    # m/s^2
    max_acc_z: float = 3.0
    dist_sigma_mps: float = 0.0  # horizontal velocity-tracking disturbance (Gauss-Markov) std
    dist_tau_s: float = 3.0      # its correlation time


@dataclass
class SimDrone:
    drone_id: int
    pos: Vec3
    vel: Vec3 = ZERO
    battery_pct: float = 100.0
    alive: bool = True
    landed: bool = True
    dist: tuple[float, float] = (0.0, 0.0)

    def integrate(self, cmd: Command | None, dt: float, dyn: Dynamics, land_speed: float,
                  rng: random.Random | None = None) -> None:
        if cmd is None or (cmd.mode == FlightMode.GROUND and self.landed):
            self.vel = ZERO
            return
        target = (0.0, 0.0, -land_speed) if cmd.mode == FlightMode.LAND else cmd.vel
        if cmd.mode == FlightMode.GROUND:
            target = (0.0, 0.0, -land_speed)
        if dyn.dist_sigma_mps > 0.0 and rng is not None and not self.landed and cmd.mode != FlightMode.LAND:
            a = math.exp(-dt / dyn.dist_tau_s)
            k = dyn.dist_sigma_mps * math.sqrt(1.0 - a * a)
            self.dist = (a * self.dist[0] + k * rng.gauss(0.0, 1.0), a * self.dist[1] + k * rng.gauss(0.0, 1.0))
            target = (target[0] + self.dist[0], target[1] + self.dist[1], target[2])
        ax = (target[0] - self.vel[0]) / dyn.tau_s
        ay = (target[1] - self.vel[1]) / dyn.tau_s
        az = (target[2] - self.vel[2]) / dyn.tau_s
        a_xy = math.hypot(ax, ay)
        if a_xy > dyn.max_acc_xy:
            ax, ay = ax * dyn.max_acc_xy / a_xy, ay * dyn.max_acc_xy / a_xy
        az = max(-dyn.max_acc_z, min(dyn.max_acc_z, az))
        vx, vy, vz = self.vel[0] + ax * dt, self.vel[1] + ay * dt, self.vel[2] + az * dt
        x, y, z = self.pos[0] + vx * dt, self.pos[1] + vy * dt, self.pos[2] + vz * dt
        if z <= 0.0:
            z = 0.0
            if target[2] <= 0.0:          # not commanded to climb: on the ground
                vx = vy = vz = 0.0
                self.landed = True
            else:
                vz = max(vz, 0.0)
                self.landed = False
        else:
            self.landed = False
        self.pos, self.vel = (x, y, z), (vx, vy, vz)


class Network:
    """Heartbeat transport with per-message loss/latency and blockable directed links."""

    def __init__(self, rng: random.Random, latency_s: float = 0.0, jitter_s: float = 0.0, loss: float = 0.0):
        self.rng = rng
        self.latency_s, self.jitter_s, self.loss = latency_s, jitter_s, loss
        self.blocked: set[tuple[int, int]] = set()
        self._queue: list[tuple[float, int, int, Heartbeat]] = []
        self._seq = 0
        self.sent = self.dropped = 0

    def send(self, now: float, src: int, hb: Heartbeat, dsts) -> None:
        for dst in dsts:
            self.sent += 1
            if (src, dst) in self.blocked or (self.loss > 0.0 and self.rng.random() < self.loss):
                self.dropped += 1
                continue
            t = now + self.latency_s + (self.rng.random() * self.jitter_s if self.jitter_s else 0.0)
            heapq.heappush(self._queue, (t, self._seq, dst, hb))
            self._seq += 1

    def due(self, now: float):
        while self._queue and self._queue[0][0] <= now + 1e-9:
            _, _, dst, hb = heapq.heappop(self._queue)
            yield dst, hb

    def partition(self, groups: list[set[int]]) -> None:
        for a in groups:
            for b in groups:
                if a is b:
                    continue
                self.blocked.update((i, j) for i in a for j in b)

    def block_tx(self, src: int, dsts) -> None:
        self.blocked.update((src, d) for d in dsts if d != src)

    def heal(self) -> None:
        self.blocked.clear()


@dataclass
class StepStats:
    t: float
    masters: list[int]
    converged: bool
    min_sep: float
    formation_rms: float | None


class PureSim:
    def __init__(self, cfg: Config, *, seed: int = 0, latency_s: float = 0.0, jitter_s: float = 0.0,
                 loss: float = 0.0, dt: float = 0.05, boot_spread_s: float = 5.0,
                 drain_pct_per_s: float = 0.0, dynamics: Dynamics | None = None) -> None:
        self.cfg = cfg
        self.rng = random.Random(seed)
        self.dt = dt
        self.t = 0.0
        self.dyn = dynamics or Dynamics()
        self.drain = drain_pct_per_s
        self.half_angle = math.radians(cfg.formation.v_half_angle_deg)
        heading = heading_of(*cfg.mission.goal_enu_m)
        layout = initial_layout(cfg.drone_ids, heading, cfg.formation.spacing_m, self.half_angle)
        self.drones = {i: SimDrone(i, (e, n, 0.0)) for i, (e, n) in layout.items()}
        self.boot = {i: self.rng.uniform(0.0, boot_spread_s) for i in cfg.drone_ids}
        self.agents = {i: AgentCore(cfg, i, self.drones[i].pos, self.boot[i]) for i in cfg.drone_ids}
        self.net = Network(self.rng, latency_s, jitter_s, loss)
        self.cmds: dict[int, Command] = {}
        self.min_sep_seen = math.inf
        self.min_sep_pair: tuple[int, int] | None = None

    # ------------------------------------------------------------------ faults
    def alive_ids(self) -> list[int]:
        return [i for i, d in self.drones.items() if d.alive]

    def kill(self, drone_id: int) -> None:
        self.drones[drone_id].alive = False

    def set_battery(self, drone_id: int, pct: float) -> None:
        self.drones[drone_id].battery_pct = pct

    # ------------------------------------------------------------------ queries
    def role(self, i: int) -> Role:
        return self.agents[i].election.role

    def masters(self) -> list[int]:
        return [i for i in self.alive_ids() if self.role(i) == Role.MASTER]

    def converged(self) -> bool:
        """Exactly one master among alive drones, and every alive non-retired drone follows it."""
        ms = self.masters()
        if len(ms) != 1:
            return False
        m = ms[0]
        term = self.agents[m].election.term
        for i in self.alive_ids():
            e = self.agents[i].election
            if e.role == Role.RETIRED:
                continue
            if e.master_id != m or e.term != term:
                return False
        return True

    def airborne_positions(self) -> dict[int, Vec3]:
        return {i: d.pos for i, d in self.drones.items() if d.alive and not d.landed}

    def formation_rms(self) -> float | None:
        """RMS slot error of member followers (true positions) while a single master cruises."""
        ms = self.masters()
        if len(ms) != 1:
            return None
        m = ms[0]
        ag = self.agents[m]
        if ag.phase != Phase.CRUISE:
            return None
        members = ag.election.members(self.t)
        mp = self.drones[m].pos
        errs = [dist(self.drones[i].pos, slot_position(mp, ag.heading, slot, self.cfg.formation.spacing_m,
                                                         self.half_angle))
                for i, slot in assign_slots(members, m).items()
                if self.drones[i].alive and self.role(i) == Role.FOLLOWER]
        return rms(errs) if errs else None

    # ------------------------------------------------------------------ stepping
    def step(self) -> StepStats:
        self.t += self.dt
        t = self.t
        for dst, hb in self.net.due(t):
            if self.drones[dst].alive and t >= self.boot[dst]:
                self.agents[dst].on_heartbeat(hb, t)
        order = [i for i in self.alive_ids() if t >= self.boot[i]]
        self.rng.shuffle(order)
        alive = self.alive_ids()
        for i in order:
            d = self.drones[i]
            own = OwnState(d.pos, d.vel, d.battery_pct, ready=True, landed=d.landed)
            cmd, hb = self.agents[i].step(own, t)
            self.cmds[i] = cmd
            if hb is not None:
                self.net.send(t, i, decode(encode(hb)), [j for j in alive if j != i])
        land_speed = self.cfg.mission.land_speed_mps
        for i in alive:
            d = self.drones[i]
            d.integrate(self.cmds.get(i), self.dt, self.dyn, land_speed, self.rng)
            if not d.landed and self.drain:
                d.battery_pct = max(0.0, d.battery_pct - self.drain * self.dt)
        pos = self.airborne_positions()
        min_sep, pair = min_pairwise_distance(pos) if len(pos) > 1 else (math.inf, None)
        if min_sep < self.min_sep_seen:
            self.min_sep_seen, self.min_sep_pair = min_sep, pair
        return StepStats(t, self.masters(), self.converged(), min_sep, self.formation_rms())

    def run_until(self, t_end: float, on_step=None) -> None:
        while self.t < t_end - 1e-9:
            stats = self.step()
            if on_step is not None:
                on_step(self, stats)
