#!/usr/bin/env python3
"""Full-stack comparison on a REAL map: the agent code of every drone (election, formation, safety, avoider)
flies an OpenStreetMap route in the fast simulator, optionally losing its leader halfway.

Default route: F-9 Park -> Faisal Mosque, Islamabad (3.2 km; ~930 buildings and 26 woods/parks from OSM,
low-altitude profile: every building and wood is an obstacle to go around).

Usage: PYTHONPATH=src python3 scripts/rl_eval_route.py --policy models/avoid_policy.npz
          [--seeds 5] [--out reports/logs/rl/route_eval] [--jobs 3]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.agent_core import World  # noqa: E402
from swarm_agent.avoidance import LearnedPolicy, NoAvoidance, PotentialField, Shielded  # noqa: E402
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.geometry import EnuFrame, GeoPoint  # noqa: E402
from swarm_agent.heartbeat import Phase, Role  # noqa: E402
from swarm_agent.planner import plan_path  # noqa: E402
from swarm_tools import osm  # noqa: E402
from swarm_tools.gcs.fastsim_backend import OSM_MARGIN_DEG, mission_config  # noqa: E402
from swarm_tools.puresim import Dynamics, PureSim  # noqa: E402

HOME, TARGET = (33.7036, 73.0231), (33.7299, 73.0373)
T_MAX_S = 1100.0


def build(method: str, policy: str, apf: dict):
    return {"none": lambda: NoAvoidance(), "apf": lambda: PotentialField(**apf),
            "apf-default": lambda: PotentialField(), "rl": lambda: LearnedPolicy(policy),
            "rl+shield": lambda: Shielded(LearnedPolicy(policy))}[method]()


def episode(args: tuple) -> dict:
    method, seed, kill_leader, n, policy, apf = args
    frame = EnuFrame(GeoPoint(HOME[0], HOME[1], 0.0))
    bbox = (min(HOME[0], TARGET[0]) - OSM_MARGIN_DEG, min(HOME[1], TARGET[1]) - OSM_MARGIN_DEG,
            max(HOME[0], TARGET[0]) + OSM_MARGIN_DEG, max(HOME[1], TARGET[1]) + OSM_MARGIN_DEG)
    omap, counts, _ = osm.to_obstacles(osm.fetch(*bbox), frame)
    cfg = mission_config(load_config(default_config_path()), n, list(HOME), list(TARGET), 5.0)
    route = plan_path(omap, (0.0, 0.0), cfg.mission.goal_enu_m, clearance_m=8.0, res_m=4.0, margin_m=150.0)
    route_m = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    sim = PureSim(cfg, seed=seed, dynamics=Dynamics(dist_sigma_mps=0.15), world=World(omap, build(method, policy, apf), route))
    killed, t_kill, new_master_t, rms = None, None, None, []
    while sim.t < T_MAX_S:
        st = sim.step()
        ms = st.masters
        if kill_leader and killed is None and ms and sim.agents[ms[0]].phase == Phase.CRUISE:
            m = ms[0]
            mp = sim.drones[m].pos
            if math.hypot(mp[0], mp[1]) > 0.5 * math.hypot(*cfg.mission.goal_enu_m):
                killed, t_kill = m, sim.t
                sim.kill(m)
        if killed is not None and new_master_t is None and len(ms) == 1 and ms[0] != killed and st.converged:
            new_master_t = sim.t
        if st.formation_rms is not None:
            rms.append(st.formation_rms)
        if len(ms) == 1 and sim.agents[ms[0]].phase in (Phase.LAND, Phase.LANDED) and \
                all(sim.drones[i].landed for i in sim.alive_ids()):
            break
    gx, gy = cfg.mission.goal_enu_m
    ms = sim.masters()
    done = len(ms) == 1 and sim.agents[ms[0]].phase in (Phase.LAND, Phase.LANDED)
    flyers = [i for i in sim.alive_ids() if sim.role(i) != Role.RETIRED]
    return {
        "method": method, "seed": seed, "kill_leader": kill_leader, "drones": n, "route_m": round(route_m),
        "obstacles": counts, "hits": len(sim.obstacle_hits), "hit_list": sim.obstacle_hits,
        "alive_end": len(sim.alive_ids()) - (0 if killed is None else 0), "mission_done": done,
        "t_end_s": round(sim.t, 1), "failover_s": None if new_master_t is None else round(new_master_t - t_kill, 2),
        "min_sep_m": round(sim.min_sep_seen, 2),
        "rms_mean_m": round(float(np.mean(rms)), 2) if rms else None,
        "rms_p95_m": round(float(np.percentile(rms, 95)), 2) if rms else None,
        "far_from_goal": sum(math.hypot(sim.drones[i].pos[0] - gx, sim.drones[i].pos[1] - gy) > 60 for i in flyers),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="models/avoid_policy.npz")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--drones", type=int, default=10)
    ap.add_argument("--out", default="reports/logs/rl/route_eval")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--methods", nargs="+", default=["none", "apf", "apf-default", "rl", "rl+shield"])
    ap.add_argument("--apf-tuning", default="reports/logs/rl/apf_tuning.jsonl")
    args = ap.parse_args()
    rows = [json.loads(l) for l in Path(args.apf_tuning).read_text().splitlines()] if Path(args.apf_tuning).exists() else []
    b = max(rows, key=lambda r: tuple(r["score"])) if rows else {}
    apf = {k: b[k] for k in ("d0_m", "k_rep", "k_tan")} if b else {}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(m, s, kill, args.drones, args.policy, apf) for kill in (False, True) for m in args.methods
             for s in range(1, args.seeds + 1)]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        res = list(ex.map(episode, tasks))
    with open(out / "episodes.jsonl", "w", encoding="utf-8") as fh:
        for r in res:
            fh.write(json.dumps(r) + "\n")
    lines = [f"Route F-9 Park -> Faisal Mosque (OSM), {args.drones} drones, {args.seeds} drift seeds, tuned APF {apf}", "",
             "| Leader killed halfway | Method | Missions completed | Drones hit obstacles (total) | Missions with 0 hits | "
             "Min sep (m) | Formation RMS mean / p95 (m) | New leader (s) |", "|---|---|---|---|---|---|---|---|"]
    for kill in (False, True):
        for m in args.methods:
            g = [r for r in res if r["method"] == m and r["kill_leader"] == kill]
            fo = [r["failover_s"] for r in g if r["failover_s"] is not None]
            lines.append(f"| {'yes' if kill else 'no'} | {m} | {sum(r['mission_done'] for r in g)}/{len(g)} | "
                         f"{sum(r['hits'] for r in g)} | {sum(r['hits'] == 0 for r in g)}/{len(g)} | "
                         f"{min(r['min_sep_m'] for r in g):.2f} | "
                         f"{np.mean([r['rms_mean_m'] for r in g if r['rms_mean_m'] is not None]):.2f} / "
                         f"{np.mean([r['rms_p95_m'] for r in g if r['rms_p95_m'] is not None]):.2f} | "
                         f"{(f'{np.median(fo):.2f}' if fo else '-')} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
