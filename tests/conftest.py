"""Shared test helpers: config loading and a tiny heartbeat bus for Election-only tests."""
from __future__ import annotations

from dataclasses import dataclass, field

import pytest

from swarm_agent.config import Config, default_config_path, load_config
from swarm_agent.election import Election
from swarm_agent.geometry import ZERO
from swarm_agent.heartbeat import Flag, Heartbeat, Phase, Role, members_to_mask


@pytest.fixture(scope="session")
def cfg() -> Config:
    return load_config(default_config_path())


def hb_from(e: Election, now: float, *, ready: bool = True, pos=(0.0, 0.0, 30.0)) -> Heartbeat:
    """Heartbeat as AgentCore would build it from an Election's state."""
    flags = e.flags(now) | (Flag.READY if ready else 0)
    members = members_to_mask(e.members(now)) if e.role == Role.MASTER else 0
    return Heartbeat(drone_id=e.my_id, role=e.role, term=e.term, phase=Phase.CRUISE,
                     master_id=e.master_id, handover_to=e.handover_to, flags=flags, battery_pct=80.0,
                     pos=pos, vel=ZERO, heading=0.0, members=members, stamp=now)


@dataclass
class Bus:
    """Runs Election instances with a heartbeat every `period` and delivery after `latency`.
    `blocked` holds directed (src, dst) links that drop everything; `dead` drones stop entirely."""
    elections: dict[int, Election]
    period: float = 0.2
    latency: float = 0.0
    dt: float = 0.05
    t: float = 0.0
    blocked: set[tuple[int, int]] = field(default_factory=set)
    dead: set[int] = field(default_factory=set)
    _next_hb: dict[int, float] = field(default_factory=dict)
    _queue: list[tuple[float, int, Heartbeat]] = field(default_factory=list)

    def step(self) -> None:
        self.t = round(self.t + self.dt, 9)
        due = [m for m in self._queue if m[0] <= self.t + 1e-9]
        self._queue = [m for m in self._queue if m[0] > self.t + 1e-9]
        for _, dst, hb in due:
            if dst not in self.dead:
                self.elections[dst].on_heartbeat(hb, self.t)
        for i, e in self.elections.items():
            if i in self.dead:
                continue
            e.update(self.t)
            if e.changed or self.t >= self._next_hb.get(i, 0.0):
                e.changed = False
                self._next_hb[i] = self.t + self.period
                hb = hb_from(e, self.t)
                for j in self.elections:
                    if j != i and (i, j) not in self.blocked:
                        self._queue.append((self.t + self.latency, j, hb))

    def run(self, seconds: float) -> None:
        end = self.t + seconds
        while self.t < end - 1e-9:
            self.step()

    def run_until(self, cond, timeout: float) -> float:
        start = self.t
        while not cond():
            if self.t - start > timeout:
                raise AssertionError(f"condition not met within {timeout}s")
            self.step()
        return self.t - start

    def masters(self) -> list[int]:
        return [i for i, e in self.elections.items() if i not in self.dead and e.role == Role.MASTER]

    def partition(self, a: set[int], b: set[int]) -> None:
        self.blocked |= {(i, j) for i in a for j in b} | {(j, i) for i in a for j in b}


def make_bus(ids, *, master_timeout=1.5, peer_timeout=1.5, handover_timeout=1.0, startup_listen=3.0,
             boot: dict[int, float] | None = None, **kw) -> Bus:
    boot = boot or {}
    return Bus({i: Election(i, master_timeout, peer_timeout, handover_timeout, startup_listen, boot.get(i, 0.0))
                for i in ids}, **kw)
