#!/usr/bin/env python3
"""Build the swarm replay page: a logged PX4 mission + a master-failover demo from the fast simulator.

Usage: python3 scripts/make_replay.py <px4_run_dir> <template.html> <out.html> [px4_fault_run_dir]
The PX4 scenario comes from <run_dir>/states.jsonl (+ formation_rms.csv, fault_events.jsonl);
the failover demo runs src/swarm_tools/puresim.py (same AgentCore, point-mass physics) unless a
PX4 fault run is given.
"""
from __future__ import annotations

import bisect
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.formation import assign_slots, slot_position  # noqa: E402
from swarm_agent.heartbeat import Phase, Role  # noqa: E402

DT = 0.25
PHASE_CODE = {"IDLE": "I", "TAKEOFF": "T", "CRUISE": "C", "HOLD": "H", "LAND": "L", "LANDED": "L"}


def dm(x: float) -> int:
    return int(round(x * 10))


def scenario_from_run(run_dir: Path, name: str, subtitle: str) -> dict:
    cfg = load_config(default_config_path())
    half = math.radians(cfg.formation.v_half_angle_deg)
    states = [json.loads(l) for l in open(run_dir / "states.jsonl", encoding="utf-8") if l.strip()]
    by: dict[int, list[dict]] = {}
    for s in states:
        by.setdefault(s["id"], []).append(s)
    for rows in by.values():
        rows.sort(key=lambda s: s["t"])
    times = {i: [s["t"] for s in rows] for i, rows in by.items()}
    ids = sorted(by)
    t0 = min(s["t"] for s in states)
    t1 = max(s["t"] for s in states)
    rms_rows = []
    if (run_dir / "formation_rms.csv").exists():
        rms_rows = [(float(r["t_s"]), float(r["rms_m"])) for r in csv.DictReader(open(run_dir / "formation_rms.csv"))]
    rms_t = [r[0] for r in rms_rows]
    fault_t = []
    if (run_dir / "fault_events.jsonl").exists():
        for l in open(run_dir / "fault_events.jsonl"):
            e = json.loads(l)
            if e.get("action") in ("fault", "heal"):
                fault_t.append(e)

    sc = {"name": name, "subtitle": subtitle, "dt": DT, "ids": ids, "goal": list(cfg.mission.goal_enu_m),
          "cruise_alt": cfg.mission.cruise_alt_m, "fence_alt": cfg.safety.geofence_max_alt_m,
          "e": {i: [] for i in ids}, "n": {i: [] for i in ids}, "u": {i: [] for i in ids},
          "se": {i: [] for i in ids}, "sn": {i: [] for i in ids},
          "role": [], "master": [], "term": [], "phase": [], "rms": [], "events": []}
    last_master, last_phase, last_roles = None, None, {}
    t = t0
    while t <= t1 + 1e-9:
        cur = {}
        for i in ids:
            k = bisect.bisect_right(times[i], t) - 1
            if k >= 0 and t - times[i][k] < 0.5 and "pos" in by[i][k] and by[i][k].get("fcu_ok", True):
                cur[i] = by[i][k]
        masters = [i for i, s in cur.items() if s.get("role") == "MASTER"]
        m = masters[0] if len(masters) == 1 else None
        roles = ""
        for i in ids:
            s = cur.get(i)
            if s is None:
                roles += "X"
                for key in ("e", "n", "u", "se", "sn"):
                    sc[key][i].append(None)
                continue
            dt = t - s.get("pos_t", s["t"])
            p = [s["pos"][k] + s["vel"][k] * dt for k in range(3)]
            sc["e"][i].append(dm(p[0]))
            sc["n"][i].append(dm(p[1]))
            sc["u"][i].append(dm(max(p[2], 0.0)))
            r = s.get("role")
            code = {"MASTER": "M", "RETIRED": "R"}.get(r, "F")
            if code == "F" and s.get("orphan"):
                code = "O"
            elif code == "F" and s.get("transit"):
                code = "T"
            roles += code
            sc["se"][i].append(None)
            sc["sn"][i].append(None)
        if m is not None and "members" in cur[m] and cur[m].get("phase") in ("CRUISE", "HOLD"):
            ms = cur[m]
            dtm = t - ms.get("pos_t", ms["t"])
            mp = tuple(ms["pos"][k] + ms["vel"][k] * dtm for k in range(3))
            for i, slot in assign_slots(ms["members"], m).items():
                sp = slot_position(mp, ms["heading"], slot, cfg.formation.spacing_m, half)
                sc["se"][i][-1], sc["sn"][i][-1] = dm(sp[0]), dm(sp[1])
        sc["role"].append(roles)
        sc["master"].append(m or 0)
        sc["term"].append(cur[m].get("term", 0) if m else 0)
        ph = PHASE_CODE.get(cur[m].get("phase"), "I") if m else (last_phase or "I")
        sc["phase"].append(ph)
        tr = t - t0
        k = bisect.bisect_left(rms_t, tr - 0.13)
        sc["rms"].append(int(round(rms_rows[k][1] * 100)) if k < len(rms_t) and abs(rms_t[k] - tr) < 0.13 and m else -1)
        if m != last_master and m is not None:
            sc["events"].append([round(tr, 2), "master", f"Drone {m} is master (term {cur[m].get('term')})"])
        if ph != last_phase and m is not None:
            label = {"T": "Takeoff to 30 m", "C": "Cruise to goal in V formation", "H": "At goal: hover 10 s",
                     "L": "All drones land"}.get(ph)
            if label:
                sc["events"].append([round(tr, 2), "phase", label])
        for i in ids:
            prev = last_roles.get(i)
            now_r = roles[ids.index(i)]
            if prev and prev != "X" and now_r == "X":
                sc["events"].append([round(tr, 2), "fault", f"Drone {i} lost (killed)"])
            if prev in ("F", "O", "M") and now_r == "T":
                sc["events"].append([round(tr, 2), "info", f"Drone {i} moves on the transit layer (-6 m)"])
            if prev in ("F", "T", "M") and now_r == "O":
                sc["events"].append([round(tr, 2), "info", f"Drone {i} not heard by master: orphan layer (+8 m)"])
            if prev in ("F", "M", "T") and now_r == "R":
                sc["events"].append([round(tr, 2), "info", f"Drone {i} low battery: returns home (+15 m)"])
            last_roles[i] = now_r
        last_master, last_phase = m, ph
        t += DT
    for e in fault_t:
        label = {"F1": "Master killed", "F2": "Master's radio link cut", "F3": "Master battery drain started",
                 "F4": "Follower killed", "F5": "Radio partition: front / back groups"}.get(e["kind"], e["kind"])
        if e["action"] == "heal":
            label = "Partition healed"
        sc["events"].append([round(e["t"] - t0, 2), "fault", label])
    sc["events"].sort(key=lambda x: x[0])
    sc["frames"] = len(sc["role"])
    sc["start_frame"] = next((f for f, p in enumerate(sc["phase"]) if p == "C"), 0)
    return sc


def scenario_puresim(n: int = 10, seed: int = 7) -> dict:
    """Master failover demo in the point-mass simulator: master killed during cruise."""
    from swarm_tools.puresim import Dynamics, PureSim
    cfg = load_config(default_config_path()).with_num_drones(n)
    half = math.radians(cfg.formation.v_half_angle_deg)
    sim = PureSim(cfg, seed=seed, dynamics=Dynamics(dist_sigma_mps=0.15))
    ids = cfg.drone_ids
    sc = {"name": "Failover demo", "subtitle": "Fast simulator (same agent code, point-mass physics): "
          "master drone 1 is killed during cruise", "dt": DT, "ids": ids, "goal": list(cfg.mission.goal_enu_m),
          "cruise_alt": cfg.mission.cruise_alt_m, "fence_alt": cfg.safety.geofence_max_alt_m,
          "e": {i: [] for i in ids}, "n": {i: [] for i in ids}, "u": {i: [] for i in ids},
          "se": {i: [] for i in ids}, "sn": {i: [] for i in ids},
          "role": [], "master": [], "term": [], "phase": [], "rms": [], "events": []}
    state = {"cruise": None, "killed": False, "next": 0.0, "last_m": None, "last_ph": None, "roles": {}}

    def on_step(s: PureSim, st) -> None:
        ms = s.masters()
        m = ms[0] if len(ms) == 1 else None
        if m is not None and s.agents[m].phase == Phase.CRUISE and state["cruise"] is None:
            state["cruise"] = s.t
        if state["cruise"] is not None and not state["killed"] and s.t >= state["cruise"] + 60.0 and m is not None:
            s.kill(m)
            state["killed"] = True
            sc["events"].append([round(s.t, 2), "fault", f"Master (drone {m}) killed"])
        if s.t + 1e-9 < state["next"]:
            return
        state["next"] += DT
        roles = ""
        for i in ids:
            d, ag = s.drones[i], s.agents[i]
            if not d.alive:
                roles += "X"
                for key in ("e", "n", "u", "se", "sn"):
                    sc[key][i].append(None)
                continue
            sc["e"][i].append(dm(d.pos[0]))
            sc["n"][i].append(dm(d.pos[1]))
            sc["u"][i].append(dm(d.pos[2]))
            sc["se"][i].append(None)
            sc["sn"][i].append(None)
            r = ag.election.role
            code = "M" if r == Role.MASTER else "R" if r == Role.RETIRED else "O" if ag.orphan else "T" if ag.transit else "F"
            roles += code
        if m is not None and s.agents[m].phase in (Phase.CRUISE, Phase.HOLD):
            ag = s.agents[m]
            for i, slot in assign_slots(ag.election.members(s.t), m).items():
                if s.drones[i].alive:
                    sp = slot_position(s.drones[m].pos, ag.heading, slot, cfg.formation.spacing_m, half)
                    sc["se"][i][-1], sc["sn"][i][-1] = dm(sp[0]), dm(sp[1])
        ph = PHASE_CODE[s.agents[m].phase.name] if m else (state["last_ph"] or "I")
        sc["role"].append(roles)
        sc["master"].append(m or 0)
        sc["term"].append(s.agents[m].election.term if m else 0)
        sc["phase"].append(ph)
        sc["rms"].append(int(round(st.formation_rms * 100)) if st.formation_rms is not None else -1)
        if m != state["last_m"] and m is not None:
            sc["events"].append([round(s.t, 2), "master", f"Drone {m} is master (term {s.agents[m].election.term})"])
        if m is None and state["last_m"] is not None:
            sc["events"].append([round(s.t, 2), "info", "No master heard: followers hover"])
        if ph != state["last_ph"] and m is not None:
            label = {"T": "Takeoff to 30 m", "C": "Cruise to goal in V formation", "H": "At goal: hover 10 s",
                     "L": "All drones land"}.get(ph)
            if label:
                sc["events"].append([round(s.t, 2), "phase", label])
        for i in ids:
            prev, now_r = state["roles"].get(i), roles[ids.index(i)]
            if prev in ("F", "O", "M") and now_r == "T":
                sc["events"].append([round(s.t, 2), "info", f"Drone {i} moves on the transit layer (-6 m)"])
            state["roles"][i] = now_r
        state["last_m"], state["last_ph"] = m, ph

    sim.run_until(300.0, on_step)
    sc["events"].sort(key=lambda x: x[0])
    sc["frames"] = len(sc["role"])
    kill = next(e[0] for e in sc["events"] if e[1] == "fault")
    sc["start_frame"] = max(0, int((kill - 8.0) / DT))
    return sc


def main() -> None:
    run_dir, template, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    s1 = scenario_from_run(run_dir, "PX4 flight", "Logged run: 10 PX4 SIH quadrotors + MAVROS + ROS 2 agents, "
                           "no faults (" + run_dir.name + ")")
    if len(sys.argv) > 4:
        fr = Path(sys.argv[4])
        s2 = scenario_from_run(fr, "Failover (PX4)", "Logged run with a fault: " + fr.name)
        s2["start_frame"] = max(0, int((next(e[0] for e in s2["events"] if e[1] == "fault") - 8.0) / DT))
    else:
        s2 = scenario_puresim()
    data = json.dumps({"scenarios": [s1, s2]}, separators=(",", ":"))
    html = template.read_text(encoding="utf-8").replace("/*__REPLAY_DATA__*/", data)
    out.write_text(html, encoding="utf-8")
    print(out, f"{len(html) / 1e6:.2f} MB", "frames", s1["frames"], s2["frames"])


if __name__ == "__main__":
    main()
