"""Fast-simulator backend for the ground-control app.

Runs src/swarm_tools/puresim.py - the real AgentCore for every drone (election, formation, safety),
with point-mass physics - in a background thread at 1x .. max speed. Home and target are real GPS
coordinates; the shared ENU frame is anchored at home, and every drone's state is reported back as
latitude/longitude. Faults are injected per drone from the web page.
"""
from __future__ import annotations

import dataclasses
import math
import random
import threading
import time
from collections import deque

from swarm_agent.config import Config, OriginCfg, validate
from swarm_agent.geometry import EnuFrame, GeoPoint
from swarm_agent.heartbeat import Phase, Role
from swarm_tools.puresim import Dynamics, PureSim

FAULTS = {
    "crash": "Crash / circuit failure: the drone drops",
    "motor": "Motor failure: the drone drops",
    "battery_low": "Battery low (29 %): hands over if master, flies home",
    "battery_critical": "Battery critical (9 %): lands right where it is",
    "battery_empty": "Battery empty: the drone drops",
    "gps": "GPS failure: emergency landing",
    "radio_cut": "Radio transmitter cut: others stop hearing it",
    "radio_restore": "Radio transmitter restored",
}
PHASE_TEXT = {Phase.TAKEOFF: "Takeoff to cruise altitude", Phase.CRUISE: "Cruise to the target",
              Phase.HOLD: "Target reached: hovering", Phase.LAND: "Landing at the target"}
DEFAULTS = {
    "n": 10, "home": [33.7036, 73.0231], "target": [33.7299, 73.0373],   # example: F-9 Park -> Faisal Mosque
    "cruise_mps": 5.0, "endurance_min": 25.0, "seed": 1, "drift": 0.15,
}
MIN_ROUTE_M = 50.0


def mission_config(base: Config, n: int, home: list[float], target: list[float], cruise_mps: float) -> Config:
    """Base config re-anchored at a real home position, with a real GPS target."""
    frame = EnuFrame(GeoPoint(home[0], home[1], 0.0))
    ge, gn, _ = frame.to_enu(GeoPoint(target[0], target[1], 0.0))
    dist = math.hypot(ge, gn)
    if dist < MIN_ROUTE_M:
        raise ValueError(f"target is only {dist:.0f} m from home; pick one at least {MIN_ROUTE_M:.0f} m away")
    r = dataclasses.replace
    cfg = r(base,
            swarm=r(base.swarm, num_drones=n),
            origin=OriginCfg(home[0], home[1], 0.0),
            mission=r(base.mission, goal_enu_m=(ge, gn), cruise_speed_mps=cruise_mps),
            formation=r(base.formation, max_speed_mps=max(base.formation.max_speed_mps, cruise_mps + 5.0)),
            safety=r(base.safety, geofence_radius_m=max(base.safety.geofence_radius_m, dist * 1.2 + 500.0)))
    validate(cfg)
    return cfg


class FastSimBackend:
    name = "fast"
    label = "Fast simulator (real agent code, point-mass physics)"

    def __init__(self, base: Config) -> None:
        self.base = base
        self.lock = threading.RLock()
        self.params = dict(DEFAULTS)
        self.speed = 4.0
        self.running = False
        self.events: deque[tuple[int, float, str, str]] = deque(maxlen=300)
        self.seq = 0
        self._debt = 0.0
        self._rate = (time.monotonic(), 0.0)
        self.actual_speed = 0.0
        self._start(self.params)
        threading.Thread(target=self._loop, daemon=True).start()

    # ------------------------------------------------------------------ mission lifecycle
    def _start(self, p: dict) -> None:
        cfg = mission_config(self.base, int(p["n"]), p["home"], p["target"], float(p["cruise_mps"]))
        drain = 100.0 / (float(p["endurance_min"]) * 60.0)
        self.cfg = cfg
        self.frame = EnuFrame(GeoPoint(p["home"][0], p["home"][1], 0.0))
        self.sim = PureSim(cfg, seed=int(p["seed"]), drain_pct_per_s=drain,
                           dynamics=Dynamics(dist_sigma_mps=float(p["drift"])))
        self.fault_rng = random.Random(int(p["seed"]) * 7919)
        self.events.clear()
        self._ev_idx = {i: 0 for i in cfg.drone_ids}
        self._alive = {i: True for i in cfg.drone_ids}
        self._last_master: int | None = None
        self._last_phase: Phase | None = None
        self._done = False
        self._stats = None
        self._heading = {i: math.degrees(math.atan2(cfg.mission.goal_enu_m[0], cfg.mission.goal_enu_m[1])) % 360.0
                         for i in cfg.drone_ids}
        self._debt = 0.0
        dist = math.hypot(*cfg.mission.goal_enu_m)
        self._event("info", f"Mission ready: {len(cfg.drone_ids)} drones, route {dist / 1000:.2f} km, "
                            f"battery endurance {p['endurance_min']:.0f} min (seed {p['seed']})")
        if dist / cfg.mission.cruise_speed_mps > 0.8 * float(p["endurance_min"]) * 60.0:
            self._event("fault", "Warning: the route is longer than the batteries allow; drones will turn back "
                                 "at 30 % battery")

    def _event(self, kind: str, text: str) -> None:
        self.seq += 1
        self.events.append((self.seq, round(self.sim.t, 1) if hasattr(self, "sim") else 0.0, kind, text))

    # ------------------------------------------------------------------ simulation thread
    def _loop(self) -> None:
        last = time.monotonic()
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            wall_dt, last = now - last, now
            with self.lock:
                if not self.running:
                    self.actual_speed = 0.0
                    continue
                t_before = self.sim.t
                self._debt = min(self._debt + wall_dt * self.speed, 2.0)
                budget = time.monotonic() + 0.03
                while self._debt >= self.sim.dt and time.monotonic() < budget:
                    self._stats = self.sim.step()
                    self._debt -= self.sim.dt
                    self._after_step()
                    if self._done:
                        self.running = False
                        break
                t0, st = self._rate
                acc = st + (self.sim.t - t_before)
                if now - t0 >= 1.0:
                    self.actual_speed, self._rate = acc / (now - t0), (now, 0.0)
                else:
                    self._rate = (t0, acc)

    def _after_step(self) -> None:
        sim = self.sim
        for i, ag in sim.agents.items():
            evs = ag.election.events
            for ev in evs[self._ev_idx[i]:]:
                if ev.kind == "claim":
                    how = "planned handover" if ev.detail.get("reason") == "handover" else "election"
                    self._event("master", f"Drone {i} became master (term {ev.detail['term']}, {how})")
                elif ev.kind == "step_down":
                    self._event("info", f"Drone {i} stepped down; drone {ev.detail['to']} leads")
                elif ev.kind == "handover_named":
                    self._event("info", f"Master {i} is handing over to drone {ev.detail['successor']}")
                elif ev.kind == "retire":
                    why = "GPS lost" if not sim.drones[i].gps_ok else (
                        "battery critical" if sim.drones[i].battery_pct <= self.cfg.battery.critical_pct else "battery low")
                    plan = "emergency landing" if ag.emergency else "flying home"
                    self._event("fault", f"Drone {i} leaves the formation ({why}): {plan}")
            self._ev_idx[i] = len(evs)
        for i, d in sim.drones.items():
            if self._alive[i] and not d.alive:
                self._event("fault", f"Drone {i} went down" + (" (battery empty)" if d.battery_pct <= 0 else ""))
            self._alive[i] = d.alive
        ms = sim.masters()
        m = ms[0] if len(ms) == 1 else None
        if m is None and self._last_master is not None and not ms:
            self._event("info", "No master heard: drones hover while they elect a new one")
        self._last_master = m
        if m is not None:
            ph = sim.agents[m].phase
            if ph != self._last_phase and ph in PHASE_TEXT:
                self._event("phase", PHASE_TEXT[ph])
            self._last_phase = ph
        alive = [i for i in sim.alive_ids()]
        if alive and self._last_phase in (Phase.LAND, Phase.LANDED) and all(sim.drones[i].landed for i in alive):
            self._done = True
            self._event("phase", "Mission complete: every drone still flying has landed")
        elif alive and all(sim.drones[i].landed and sim.role(i) == Role.RETIRED for i in alive) and sim.t > 60:
            self._done = True
            self._event("fault", "Mission ended: no drone could continue to the target")
        elif not alive:
            self._done = True
            self._event("fault", "Mission ended: every drone is down")

    # ------------------------------------------------------------------ state for the web page
    def snapshot(self) -> dict:
        with self.lock:
            sim, cfg = self.sim, self.cfg
            drones = []
            for i, d in sim.drones.items():
                ag = sim.agents[i]
                g = self.frame.to_geodetic((d.pos[0], d.pos[1], 0.0))
                spd = math.hypot(d.vel[0], d.vel[1])
                if spd > 0.4:
                    self._heading[i] = math.degrees(math.atan2(d.vel[0], d.vel[1])) % 360.0
                if not d.alive:
                    role, status = "DOWN", ("falling" if d.falling else "crashed")
                else:
                    role = ag.election.role.name
                    status = ag.last_cmd.reason if ag.last_cmd else "idle"
                drones.append({
                    "id": i, "lat": round(g.lat_deg, 7), "lon": round(g.lon_deg, 7), "alt": round(d.pos[2], 1),
                    "speed": round(spd, 1), "vz": round(d.vel[2], 1), "hdg": round(self._heading[i]),
                    "battery": round(d.battery_pct, 1), "role": role, "status": status,
                    "master": ag.election.master_id if d.alive else 0, "term": ag.election.term,
                    "gps_ok": d.gps_ok, "radio_ok": not any(l[0] == i for l in sim.net.blocked),
                    "landed": d.landed,
                })
            ms = sim.masters()
            m = ms[0] if len(ms) == 1 else None
            ref = sim.drones[m].pos if m else None
            gx, gy = cfg.mission.goal_enu_m
            left = math.hypot(gx - ref[0], gy - ref[1]) if ref else None
            st = self._stats
            return {
                "backend": self.label, "t": round(sim.t, 1), "running": self.running, "speed": self.speed,
                "actual_speed": round(self.actual_speed, 1), "params": self.params,
                "phase": sim.agents[m].phase.name if m else None, "master": m,
                "term": sim.agents[m].election.term if m else None, "masters": ms,
                "alive": len(sim.alive_ids()), "total": len(sim.drones),
                "rms": None if st is None or st.formation_rms is None else round(st.formation_rms, 2),
                "min_sep": None if st is None or math.isinf(st.min_sep) else round(st.min_sep, 1),
                "dist_total": round(math.hypot(gx, gy)), "dist_left": None if left is None else round(left),
                "eta": None if left is None else round(left / cfg.mission.cruise_speed_mps),
                "cruise_alt": cfg.mission.cruise_alt_m, "fence_alt": cfg.safety.geofence_max_alt_m,
                "drones": drones, "events": list(self.events)[-60:], "faults": FAULTS,
            }

    # ------------------------------------------------------------------ commands from the web page
    def _targets(self, spec) -> list[int]:
        sim = self.sim
        alive = sim.alive_ids()
        if spec == "master":
            return sim.masters()[:1]
        if isinstance(spec, str) and spec.startswith("random:"):
            k = int(spec.split(":")[1])
            return sorted(self.fault_rng.sample(alive, min(k, len(alive))))
        return [int(i) for i in spec if int(i) in sim.drones]

    def command(self, c: dict) -> dict:
        with self.lock:
            kind = c.get("cmd")
            if kind == "start":
                p = dict(self.params)
                for key in ("n", "home", "target", "cruise_mps", "endurance_min", "seed", "drift"):
                    if key in c:
                        p[key] = c[key]
                p["n"] = max(1, min(int(p["n"]), 30))
                p["cruise_mps"] = max(1.0, min(float(p["cruise_mps"]), 12.0))
                p["endurance_min"] = max(1.0, min(float(p["endurance_min"]), 10000.0))
                try:
                    self._start(p)
                except ValueError as exc:
                    return {"ok": False, "msg": str(exc)}
                self.params = p
                self.running = True
                return {"ok": True, "msg": "Mission started"}
            if kind == "reset":
                self._start(self.params)
                self.running = True
                return {"ok": True, "msg": "Same mission restarted (same seed)"}
            if kind in ("pause", "resume"):
                self.running = kind == "resume" and not self._done
                return {"ok": True, "msg": "Paused" if not self.running else "Running"}
            if kind == "speed":
                self.speed = max(0.25, min(float(c.get("value", 1.0)), 500.0))
                return {"ok": True, "msg": f"Speed {self.speed:g}x"}
            if kind == "fault":
                ftype = c.get("type")
                if ftype not in FAULTS:
                    return {"ok": False, "msg": f"unknown fault {ftype!r}"}
                ids = self._targets(c.get("target", []))
                if not ids:
                    return {"ok": False, "msg": "no drone selected (or none alive)"}
                for i in ids:
                    d = self.sim.drones[i]
                    if ftype in ("crash", "motor"):
                        self.sim.kill(i)
                    elif ftype == "battery_low":
                        d.battery_pct = min(d.battery_pct, 29.0)
                    elif ftype == "battery_critical":
                        d.battery_pct = min(d.battery_pct, 9.0)
                    elif ftype == "battery_empty":
                        d.battery_pct = 0.0
                        if not d.landed:
                            self.sim.kill(i)
                    elif ftype == "gps":
                        d.gps_ok = False
                    elif ftype == "radio_cut":
                        self.sim.net.block_tx(i, self.sim.drones)
                    elif ftype == "radio_restore":
                        self.sim.net.unblock_tx(i)
                names = ", ".join(str(i) for i in ids)
                self._event("fault", f"Injected on drone{'s' if len(ids) > 1 else ''} {names}: {FAULTS[ftype]}")
                return {"ok": True, "msg": f"{FAULTS[ftype]} -> {names}"}
            if kind == "partition":
                alive = sorted(self.sim.alive_ids())
                front, back = alive[: len(alive) // 2], alive[len(alive) // 2:]
                self.sim.net.partition([set(front), set(back)])
                self._event("fault", f"Radio partition: {front} cannot hear {back}")
                return {"ok": True, "msg": "Partition applied"}
            if kind == "heal":
                self.sim.net.heal()
                self._event("info", "All radio links restored")
                return {"ok": True, "msg": "Radios healed"}
            return {"ok": False, "msg": f"unknown command {kind!r}"}
