"""Phase 4 fault injection (central tool; used by scripts/run_mission.py --fault F1..F5).

Each fault fires at a seeded random time during cruise: uniform in
[CRUISE start + FAULT_WINDOW_S[0], CRUISE start + FAULT_WINDOW_S[1]], where CRUISE start is the
first agent_state in which the master reports phase CRUISE.
  F1 master killed        SIGKILL the master's PX4 + MAVROS process groups and its agent process
                          (the whole drone is gone, as a crashed drone would be)
  F2 master link lost     link emulator drops 100 % of the master's heartbeats for the rest of the run
  F3 master low battery   PX4's simulated battery: SIM_BAT_MIN_PCT lowered on the master via the PX4
                          shell client, so the battery_simulator (SIM_BAT_DRAIN) drains it below 30 %
  F4 follower killed      as F1, for a random follower
  F5 partition then heal  front group (lower half of IDs) / back group (upper half), healed after a
                          random 10-20 s. Spatially separated groups, as a distance-driven radio
                          partition would be (see reports/PHASE_2.md)
Every action is appended to <run_dir>/fault_events.jsonl.
"""
from __future__ import annotations

import json
import os
import random
import signal
import subprocess
import time
from pathlib import Path

import psutil

FAULT_WINDOW_S = (30.0, 150.0)
F5_HEAL_S = (10.0, 20.0)
F3_BATTERY_MIN_PCT = 20.0   # above PX4's low (15 %) and critical thresholds; below our 30 % handover


def agent_pid(drone_id: int) -> int | None:
    for p in psutil.process_iter(["cmdline"]):
        cl = p.info["cmdline"] or []
        if "swarm_agent.ros_node" in cl and "--id" in cl and cl[cl.index("--id") + 1] == str(drone_id):
            return p.pid
    return None


class Fault:
    def __init__(self, kind: str, seed: int, cfg, sim, watcher, run_dir: Path) -> None:
        self.kind, self.cfg, self.sim, self.watcher = kind, cfg, sim, watcher
        self.rng = random.Random(seed)
        self.delay = self.rng.uniform(*FAULT_WINDOW_S)
        self.heal_after = self.rng.uniform(*F5_HEAL_S)
        self.events_path = run_dir / "fault_events.jsonl"
        self.cruise_t: float | None = None
        self.fired_t: float | None = None
        self.healed_t: float | None = None
        self.detail: dict = {"kind": kind, "seed": seed, "delay_after_cruise_s": round(self.delay, 2)}
        self.killed: set[int] = set()

    @property
    def finished(self) -> bool:
        return self.fired_t is not None and (self.kind != "F5" or self.healed_t is not None)

    def _event(self, **d) -> None:
        d = {"t": round(time.time(), 3), "kind": self.kind, **d}
        with open(self.events_path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(d) + "\n")

    def _master(self) -> int | None:
        ms = [i for i, s in self.watcher.latest.items() if s.get("role") == "MASTER" and i not in self.killed]
        return ms[0] if len(ms) == 1 else None

    def _kill(self, drone_id: int) -> None:
        self.sim.kill_drone(drone_id)          # PX4 + MAVROS process groups
        pid = agent_pid(drone_id)
        if pid is not None:
            os.kill(pid, signal.SIGKILL)
        self.killed.add(drone_id)

    def poll(self, now: float) -> set[int]:
        if self.cruise_t is None:
            if any(s.get("role") == "MASTER" and s.get("phase") == "CRUISE" for s in self.watcher.latest.values()):
                self.cruise_t = now
                self._event(action="cruise_detected")
            return self.killed
        if self.fired_t is None and now >= self.cruise_t + self.delay:
            master = self._master()
            if master is None:
                return self.killed                 # wait for a single, known master
            self.fired_t = now
            self.detail.update(t_fault=round(now, 3), master_before=master)
            if self.kind == "F1":
                self._kill(master)
                self.detail["target"] = master
            elif self.kind == "F2":
                self.watcher.link_cmd(cmd="block_tx", src=master)
                self.detail["target"] = master
            elif self.kind == "F3":
                inst = master - self.cfg.swarm.first_id
                client = Path(os.path.expanduser(self.cfg.sim.px4_dir)) / "build" / self.cfg.sim.build_target / "bin" / "px4-param"
                out = subprocess.run([str(client), "--instance", str(inst), "set", "SIM_BAT_MIN_PCT",
                                      str(F3_BATTERY_MIN_PCT)], capture_output=True, text=True, timeout=10)
                self.detail.update(target=master, px4_param_rc=out.returncode,
                                   px4_param_out=(out.stdout + out.stderr).strip()[-300:])
            elif self.kind == "F4":
                followers = sorted(i for i, s in self.watcher.latest.items()
                                   if s.get("role") == "FOLLOWER" and i not in self.killed)
                victim = self.rng.choice(followers)
                self._kill(victim)
                self.detail["target"] = victim
            elif self.kind == "F5":
                ids = self.cfg.drone_ids
                front, back = ids[: len(ids) // 2], ids[len(ids) // 2:]
                self.watcher.link_cmd(cmd="partition", groups=[front, back])
                self.detail.update(groups=[front, back], heal_after_s=round(self.heal_after, 2))
            self._event(action="fault", **{k: v for k, v in self.detail.items() if k != "kind"})
        elif self.kind == "F5" and self.fired_t is not None and self.healed_t is None \
                and now >= self.fired_t + self.heal_after:
            self.watcher.link_cmd(cmd="heal")
            self.healed_t = now
            self.detail["t_heal"] = round(now, 3)
            self._event(action="heal")
        return self.killed


def make_fault(spec: str, seed: int, cfg, sim, watcher, run_dir: Path) -> Fault:
    if spec not in ("F1", "F2", "F3", "F4", "F5"):
        raise ValueError(f"unknown fault {spec!r}")
    return Fault(spec, seed, cfg, sim, watcher, run_dir)
