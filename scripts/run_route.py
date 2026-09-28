#!/usr/bin/env python3
"""Fly one ground-control mission without the browser and save what happened as JSON.

Same code path as the app's Start button (swarm_tools.gcs.fastsim_backend): OpenStreetMap download
or cache, route planning, charging stops, the fast simulator with the real agent code. It runs as
fast as the CPU allows and writes the outcome, every event, a progress row every 1,000 simulated
seconds, and the formation error and closest pair while cruising.

Usage (defaults: home F-9 Park Islamabad, 10 drones, OpenStreetMap obstacles, RL + brake, seed 1):
  PYTHONPATH=src python3 scripts/run_route.py --target 33.5973 73.0479 --altitude auto \\
      --out reports/logs/long_route/rawalpindi_auto.json
  Lahore (about 270 km, ~70,000 simulated seconds, roughly 30-60 minutes on this laptop):
  PYTHONPATH=src python3 scripts/run_route.py --target 31.5204 74.3587 --out reports/logs/long_route/lahore.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
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
    ap.add_argument("--max-sim-hours", type=float, default=48.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    b = FastSimBackend(load_config(default_config_path()))
    p = dict(DEFAULTS, n=args.n, home=list(args.home), target=list(args.target), obstacles=args.obstacles,
             avoider=args.avoider, altitude=args.altitude, seed=args.seed)
    wall0 = time.time()
    with b.lock:
        b.running = False                        # the app's own thread stays idle; this script steps the sim
        prep = b._prepare(p)                     # map download (or cache) and route planning
        b._install(p, prep)
        b.params = p
    print(f"prepared in {time.time() - wall0:.0f} s", flush=True)
    sim = b.sim
    events: list[list] = []
    seen = 0
    rms_max, rms_sum, rms_n, sep_min = 0.0, 0.0, 0, math.inf
    above_2m_s, episodes, open_ep = 0.0, [], None      # cruise stretches with formation error above 10 m
    progress, next_row = [], 1000.0
    t_run = time.time()
    while not b._done and sim.t < args.max_sim_hours * 3600.0:
        with b.lock:
            st = sim.step()
            b._stats = st
            b._after_step()
        if b.seq != seen:
            events += [list(e) for e in b.events if e[0] > seen]
            seen = b.seq
        m = st.masters[0] if len(st.masters) == 1 else None
        if m is not None and sim.agents[m].phase.name == "CRUISE":
            if st.formation_rms is not None:
                rms_max, rms_sum, rms_n = max(rms_max, st.formation_rms), rms_sum + st.formation_rms, rms_n + 1
                above_2m_s += sim.dt if st.formation_rms > 2.0 else 0.0
                if open_ep is None and st.formation_rms > 10.0:
                    open_ep = {"start_s": round(sim.t, 1), "next_stop": b.snapshot()["next_stop"], "peak_m": 0.0}
                if open_ep is not None:
                    if st.formation_rms > open_ep["peak_m"]:
                        open_ep["peak_m"], open_ep["peak_at_s"] = round(st.formation_rms, 1), round(sim.t, 1)
                    if st.formation_rms < 5.0:
                        open_ep["end_s"] = round(sim.t, 1)
                        episodes.append(open_ep)
                        open_ep = None
            if math.isfinite(st.min_sep):
                sep_min = min(sep_min, st.min_sep)
        if sim.t >= next_row:
            s = b.snapshot()
            row = {"t_s": s["t"], "stops_done": s["next_stop"], "dist_left_m": s["dist_left"], "alive": s["alive"],
                   "hits": s["hits"], "leader": s["master"], "wall_s": round(time.time() - t_run)}
            progress.append(row)
            print(json.dumps(row), flush=True)
            next_row += 1000.0
    s = b.snapshot()
    landings = sum("Landing at charging stop" in e[3] for e in events)
    out = {
        "params": p, "code": "scripts/run_route.py (swarm_tools.gcs.fastsim_backend)",
        "wall_s": round(time.time() - wall0), "sim_s": s["t"], "speed_x": round(s["t"] / max(time.time() - t_run, 1e-9), 1),
        "outcome": next((e[3] for e in reversed(events) if e[3].startswith("Mission")), "time limit reached"),
        "alive": s["alive"], "total": s["total"], "obstacle_hits": s["hits"],
        "stops_total": len(s["stops"]), "stop_landings": landings, "route_km": round(s["dist_total"] / 1000, 2),
        "cruise_formation_rms_m": {"max": round(rms_max, 2), "mean": round(rms_sum / rms_n, 2) if rms_n else None,
                                   "time_above_2m_s": round(above_2m_s, 1),
                                   "episodes_above_10m": episodes + ([open_ep] if open_ep else [])},
        "cruise_min_separation_m": None if math.isinf(sep_min) else round(sep_min, 2),
        "progress": progress, "events": events,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k not in ("events", "progress", "params")}))


if __name__ == "__main__":
    main()
