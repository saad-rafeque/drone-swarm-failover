#!/usr/bin/env python3
"""Run the Phase 4 fault types (F1-F5) in the pure-Python simulator and print key numbers.

Usage: PYTHONPATH=src python3 scripts/puresim_faults.py [--n 10] [--seeds 20] [--latency 0] [--loss 0]
Metrics per trial: handover time (fault -> new master's heartbeat heard by all alive drones,
i.e. convergence), formation RMS recovery (< 2 m) time, min separation, goal reached.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.heartbeat import Phase, Role  # noqa: E402
from swarm_tools.puresim import PureSim  # noqa: E402

RMS_OK_M = 2.0


def run_trial(cfg, fault: str, seed: int, latency: float, loss: float, t_fault: float | None = None,
              jitter: float | None = None) -> dict:
    """latency, jitter in seconds (jitter defaults to 20 % of latency), loss as a fraction."""
    sim = PureSim(cfg, seed=seed, latency_s=latency, jitter_s=latency * 0.2 if jitter is None else jitter, loss=loss)
    rng = sim.rng
    t_fault = t_fault if t_fault is not None else rng.uniform(40.0, 150.0)
    ids = cfg.drone_ids
    res = {"fault": fault, "seed": seed, "t_fault": round(t_fault, 2)}
    state = {"done": False, "master_before": None, "converged_at": None, "rms_ok_at": None,
             "heal_at": None, "master_after": None, "fault_applied": False, "retiree": None}

    def on_step(s: PureSim, st) -> None:
        if not state["fault_applied"] and s.t >= t_fault:
            ms = s.masters()
            m = ms[0] if ms else None
            state["master_before"] = m
            state["fault_applied"] = True
            if fault == "F1":
                s.kill(m)
            elif fault == "F2":
                s.net.block_tx(m, ids)
            elif fault == "F3":
                s.set_battery(m, cfg.battery.handover_pct - 1.0)
                state["retiree"] = m
            elif fault == "F4":
                followers = [i for i in s.alive_ids() if i != m]
                s.kill(rng.choice(followers))
            elif fault == "F5":
                alive = sorted(s.alive_ids())
                k = rng.randint(1, len(alive) - 1)
                s.net.partition([set(alive[:k]), set(alive[k:])])
                state["heal_at"] = s.t + rng.uniform(5.0, 20.0)
            return
        if state["fault_applied"]:
            if fault == "F5" and state["heal_at"] and s.t >= state["heal_at"] and s.net.blocked:
                s.net.heal()
                state["reference"] = s.t
            ref = state.get("reference", t_fault if fault != "F5" else None)
            if ref is None:
                return
            if state["converged_at"] is None and st.converged and (fault != "F4" or True):
                ms = st.masters
                if fault in ("F1", "F2", "F3") and ms and ms[0] == state["master_before"]:
                    return
                state["converged_at"] = s.t
                state["master_after"] = ms[0]
            if state["converged_at"] is not None and state["rms_ok_at"] is None:
                r = st.formation_rms
                if r is not None and r < RMS_OK_M and s.t - state["converged_at"] > 1.0:
                    state["rms_ok_at"] = s.t

    sim.run_until(330.0, on_step)
    ref = state.get("reference", t_fault)
    res["handover_s"] = None if state["converged_at"] is None else round(state["converged_at"] - ref, 3)
    res["rms_recovery_s"] = None if state["rms_ok_at"] is None else round(state["rms_ok_at"] - ref, 2)
    res["master_before"], res["master_after"] = state["master_before"], state["master_after"]
    res["min_sep_m"] = round(sim.min_sep_seen, 2)
    res["min_sep_pair"] = sim.min_sep_pair
    gx, gy = cfg.mission.goal_enu_m
    flyers = [i for i in sim.alive_ids() if sim.role(i) != Role.RETIRED]
    res["goal_reached"] = all(math.hypot(sim.drones[i].pos[0] - gx, sim.drones[i].pos[1] - gy) < 60.0
                              and sim.drones[i].landed for i in flyers)
    # shape-independent: every drone still flying landed, and the leader landed at the goal (a long column or
    # echelon lands its tail more than 60 m behind the goal although the mission ended normally)
    lead = min(flyers) if flyers else None
    res["all_landed_leader_at_goal"] = bool(flyers) and all(sim.drones[i].landed for i in flyers) and \
        math.hypot(sim.drones[lead].pos[0] - gx, sim.drones[lead].pos[1] - gy) < 60.0
    if state["retiree"]:
        d = sim.drones[state["retiree"]]
        home = sim.agents[state["retiree"]].home
        res["retiree_home_dist_m"] = round(math.hypot(d.pos[0] - home[0], d.pos[1] - home[1]), 1)
        res["retiree_landed"] = d.landed
    res["final_masters"] = sim.masters()
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--latency", type=float, default=0.0)
    ap.add_argument("--loss", type=float, default=0.0)
    ap.add_argument("--faults", nargs="+", default=["F1", "F2", "F3", "F4", "F5"])
    ap.add_argument("--jsonl", default="")
    args = ap.parse_args()
    cfg = load_config(default_config_path()).with_num_drones(args.n)
    out = open(args.jsonl, "w", encoding="utf-8") if args.jsonl else None
    for fault in args.faults:
        rows = [run_trial(cfg, fault, seed, args.latency, args.loss) for seed in range(args.seeds)]
        if out:
            for r in rows:
                out.write(json.dumps(r) + "\n")
        ho = [r["handover_s"] for r in rows if r["handover_s"] is not None]
        rec = [r["rms_recovery_s"] for r in rows if r["rms_recovery_s"] is not None]
        seps = [r["min_sep_m"] for r in rows]
        worst = min(rows, key=lambda r: r["min_sep_m"])
        print(f"{fault}: handover median {statistics.median(ho) if ho else None} worst {max(ho) if ho else None} "
              f"({len(ho)}/{len(rows)}) | rms<2m median {statistics.median(rec) if rec else None} "
              f"worst {max(rec) if rec else None} ({len(rec)}/{len(rows)}) | min sep {min(seps)} "
              f"(seed {worst['seed']} pair {worst['min_sep_pair']}) | goal {sum(r['goal_reached'] for r in rows)}/{len(rows)}"
              + (f" | retiree home dist max {max(r.get('retiree_home_dist_m', 0) for r in rows)}" if fault == "F3" else ""))


if __name__ == "__main__":
    main()
