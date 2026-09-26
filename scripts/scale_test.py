#!/usr/bin/env python3
"""Does the swarm logic scale with the number of drones? Fast-simulator test.

For each N the same 1 km mission is flown with the real agent code (AgentCore) on point-mass
physics: V formation, cruise, the master is killed at --t-fault (when N >= 2), the rest must elect
a new master, re-form and land at the goal. Per run it records the failover time, formation RMS,
minimum separation, the landing result, the heartbeat size and the simulator's speed on this laptop.
Optional radio loss/latency and wind-like velocity drift make the conditions harder.

Usage: PYTHONPATH=src python3 scripts/scale_test.py [--ns 1 2 3 5 10 20 50 100] [--seeds 1 2 3]
           [--loss-pct 10 --latency-ms 150 --drift 0.15] [--jsonl reports/logs/scaling/fastsim_scaling.jsonl]
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.heartbeat import Phase, Role, encode  # noqa: E402
from swarm_tools.puresim import Dynamics, PureSim  # noqa: E402

RMS_OK_M = 2.0
T_END_S = 420.0


def run(cfg, seed: int, t_fault: float, loss_pct: float = 0.0, latency_ms: float = 0.0, drift: float = 0.0) -> dict:
    n = cfg.swarm.num_drones
    lat = latency_ms / 1000.0
    sim = PureSim(cfg, seed=seed, latency_s=lat, jitter_s=lat * 0.2, loss=loss_pct / 100.0,
                  dynamics=Dynamics(dist_sigma_mps=drift))
    st = {"old": None, "claim_t": None, "conv_t": None, "rms_ok_t": None, "done_t": None,
          "rms_cruise": [], "hb_bytes": 0, "seq": [], "max_masters": 0}
    gx, gy = cfg.mission.goal_enu_m
    send = sim.net.send

    def measure_send(now, src, hb, dsts) -> None:     # largest heartbeat on the air (sampled 1 Hz)
        if now - st.setdefault("hb_t", -1.0) >= 1.0:
            st["hb_t"], st["hb_bytes"] = now, max(st["hb_bytes"], len(encode(hb)))
        send(now, src, hb, dsts)

    sim.net.send = measure_send
    wall0 = time.perf_counter()
    while sim.t < T_END_S and st["done_t"] is None:
        s = sim.step()
        st["max_masters"] = max(st["max_masters"], len(s.masters))
        if len(s.masters) == 1 and (not st["seq"] or st["seq"][-1] != s.masters[0]):
            st["seq"].append(s.masters[0])
        if n >= 2 and st["old"] is None and sim.t >= t_fault:
            st["old"] = s.masters[0] if s.masters else None
            if st["old"] is not None:
                sim.kill(st["old"])
            continue
        if st["old"] is not None:
            new = [m for m in s.masters if m != st["old"]]
            if st["claim_t"] is None and len(s.masters) == 1 and new:
                st["claim_t"] = sim.t
            if st["conv_t"] is None and s.converged and new:
                st["conv_t"] = sim.t
            if st["conv_t"] is not None and st["rms_ok_t"] is None and s.formation_rms is not None \
                    and s.formation_rms < RMS_OK_M and sim.t - st["conv_t"] > 1.0:
                st["rms_ok_t"] = sim.t
        if s.formation_rms is not None and (st["old"] is None or sim.t > t_fault + 15.0):
            st["rms_cruise"].append(s.formation_rms)
        ms = s.masters
        if len(ms) == 1 and sim.agents[ms[0]].phase in (Phase.LAND, Phase.LANDED) \
                and all(sim.drones[i].landed for i in sim.alive_ids()):
            st["done_t"] = sim.t
    wall = time.perf_counter() - wall0
    ms = sim.masters()
    flyers = [i for i in sim.alive_ids() if sim.role(i) != Role.RETIRED]
    m = ms[0] if len(ms) == 1 else None
    rc = st["rms_cruise"]
    return {
        "n": n, "seed": seed, "loss_pct": loss_pct, "latency_ms": latency_ms, "drift_mps": drift,
        "t_fault": t_fault if n >= 2 else None, "killed": st["old"],
        "new_master": m, "final_masters": ms, "master_sequence": st["seq"], "max_simultaneous_masters": st["max_masters"],
        "claim_s": None if st["claim_t"] is None else round(st["claim_t"] - t_fault, 2),
        "failover_s": None if st["conv_t"] is None else round(st["conv_t"] - t_fault, 2),
        "rms_recovery_s": None if st["rms_ok_t"] is None else round(st["rms_ok_t"] - t_fault, 2),
        "rms_cruise_mean_m": round(statistics.fmean(rc), 3) if rc else None,
        "rms_cruise_max_m": round(max(rc), 3) if rc else None,
        "min_sep_m": None if math.isinf(sim.min_sep_seen) else round(sim.min_sep_seen, 2),
        "mission_done": st["done_t"] is not None, "t_done_s": st["done_t"] and round(st["done_t"], 1),
        "all_landed": all(sim.drones[i].landed for i in flyers),
        "master_goal_err_m": None if m is None else round(math.hypot(sim.drones[m].pos[0] - gx,
                                                                     sim.drones[m].pos[1] - gy), 1),
        "max_dist_from_goal_m": round(max(math.hypot(sim.drones[i].pos[0] - gx, sim.drones[i].pos[1] - gy)
                                          for i in flyers), 1) if flyers else None,
        "hb_bytes": st["hb_bytes"], "channel_kbit_s": round(n * cfg.heartbeat.rate_hz * st["hb_bytes"] * 8 / 1000, 1),
        "sim_s": round(sim.t, 1), "wall_s": round(wall, 1), "speedup": round(sim.t / wall, 1),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", type=int, nargs="+", default=[1, 2, 3, 5, 10, 20, 50, 100])
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--t-fault", type=float, default=90.0)
    ap.add_argument("--loss-pct", type=float, default=0.0)
    ap.add_argument("--latency-ms", type=float, default=0.0)
    ap.add_argument("--drift", type=float, default=0.0, help="wind-like velocity drift std [m/s]")
    ap.add_argument("--jsonl", default="")
    args = ap.parse_args()
    base = load_config(default_config_path())
    out = None
    if args.jsonl:
        Path(args.jsonl).parent.mkdir(parents=True, exist_ok=True)
        out = open(args.jsonl, "a", encoding="utf-8")
    for n in args.ns:
        cfg = base.with_num_drones(n)
        for seed in args.seeds:
            r = run(cfg, seed, args.t_fault, args.loss_pct, args.latency_ms, args.drift)
            print(json.dumps(r), flush=True)
            if out:
                out.write(json.dumps(r) + "\n")
                out.flush()


if __name__ == "__main__":
    main()
