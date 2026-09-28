#!/usr/bin/env python3
"""Replay a ground-control route mission (the same code path and settings as scripts/run_route.py; the fast
simulator is repeatable, so the replay is the same flight) and record every follower between two simulated
times: distance to its slot, speed, height, obstacle clearance, orphan and transit-layer flags.

Used to explain the formation-error episode of the Islamabad -> Lahore run after charging stop 5:
  PYTHONPATH=src python3 scripts/route_window.py --target 31.5204 74.3587 --from 5000 --to 5300 \\
      --out reports/logs/long_route/lahore_full_stop5_detail.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.formation import assign_slots, slot_position  # noqa: E402
from swarm_tools.gcs.fastsim_backend import AVOIDERS, DEFAULTS, FastSimBackend  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--home", type=float, nargs=2, default=DEFAULTS["home"], metavar=("LAT", "LON"))
    ap.add_argument("--target", type=float, nargs=2, required=True, metavar=("LAT", "LON"))
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--obstacles", choices=["none", "osm"], default="osm")
    ap.add_argument("--avoider", choices=sorted(AVOIDERS), default="rl+shield")
    ap.add_argument("--altitude", choices=["auto", "low", "normal"], default="auto")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--from", dest="t_from", type=float, required=True, help="simulated seconds")
    ap.add_argument("--to", dest="t_to", type=float, required=True)
    ap.add_argument("--every", type=float, default=5.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    b = FastSimBackend(load_config(default_config_path()))
    p = dict(DEFAULTS, n=args.n, home=list(args.home), target=list(args.target), obstacles=args.obstacles,
             avoider=args.avoider, altitude=args.altitude, seed=args.seed)
    with b.lock:
        b.running = False
        prep = b._prepare(p)
        b._install(p, prep)
        b.params = p
    sim = b.sim
    rows, next_t = [], args.t_from
    while sim.t < args.t_to:
        with b.lock:
            st = sim.step()
            b._stats = st
            b._after_step()
        if sim.t < next_t:
            continue
        next_t += args.every
        if len(st.masters) != 1:
            continue
        m = st.masters[0]
        ag, mp = sim.agents[m], sim.drones[m].pos
        row = {"t_s": round(sim.t, 1), "leader": m, "leader_phase": ag.phase.name,
               "formation_rms_m": None if st.formation_rms is None else round(st.formation_rms, 2), "followers": {}}
        for i, slot in assign_slots(ag.election.members(sim.t), m).items():
            d = sim.drones[i]
            sp = slot_position(mp, ag.heading, slot, sim.cfg.formation.spacing_m, sim.half_angle,
                               shape=sim.cfg.formation.shape)
            clear = sim.world.omap.clearance(d.pos[0], d.pos[1], search_m=40.0) if sim.world else None
            row["followers"][str(i)] = {
                "slot_error_m": round(math.dist(d.pos[:2], sp[:2]), 1),
                "speed_mps": round(math.hypot(d.vel[0], d.vel[1]), 1), "height_m": round(d.pos[2], 1),
                "obstacle_clearance_m": None if clear is None else round(clear, 1),
                "orphan": sim.agents[i].orphan, "transit_layer": sim.agents[i].transit}
        rows.append(row)
    Path(args.out).write_text(json.dumps({"params": p, "from_s": args.t_from, "to_s": args.t_to, "rows": rows},
                                         indent=1) + "\n")
    worst = max(rows, key=lambda r: r["formation_rms_m"] or 0.0)
    far = max(worst["followers"].items(), key=lambda kv: kv[1]["slot_error_m"])
    print(f"wrote {args.out}: {len(rows)} rows; worst at t={worst['t_s']} s, formation error "
          f"{worst['formation_rms_m']} m, drone {far[0]} {far[1]}")


if __name__ == "__main__":
    main()
