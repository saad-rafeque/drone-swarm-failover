#!/usr/bin/env python3
"""Diagnostic: per-instance CPU/RAM of idle (connected, on the ground) PX4 SIH + MAVROS vs N.
Usage: scripts/ros_env.sh python3 scripts/mavros_idle_scaling.py --ns 1 3 5 [--extra-args ...]"""
import argparse, json, time
import psutil
from swarm_agent.config import load_config, default_config_path
from swarm_tools import sim_launch
from swarm_tools.sim_launch import SimLauncher, kill_orphans

ap = argparse.ArgumentParser()
ap.add_argument("--ns", type=int, nargs="+", required=True)
ap.add_argument("--label", default="default")
ap.add_argument("--warmup", type=float, default=15.0)
ap.add_argument("--window", type=float, default=15.0)
ap.add_argument("--pluginlists", default="", help="override plugin list yaml (e.g. MAVROS default)")
args = ap.parse_args()
results = []
for n in args.ns:
    cfg = load_config(default_config_path()).with_num_drones(n)
    kill_orphans()
    sim = SimLauncher(cfg, {i: cfg.origin_geo for i in cfg.drone_ids})
    if args.pluginlists:
        from pathlib import Path
        sim.pluginlists = Path(args.pluginlists)
    sim.start(cfg.drone_ids, stagger_s=0.5)
    try:
        time.sleep(args.warmup)
        procs = []
        for dp in sim.procs.values():
            for key, p in (("px4", dp.px4), ("mavros", dp.mavros)):
                pp = psutil.Process(p.pid); pp.cpu_percent(None); procs.append((key, pp))
        psutil.cpu_percent(None)
        time.sleep(args.window)
        cpu = {"px4": 0.0, "mavros": 0.0}
        for key, pp in procs:
            cpu[key] += pp.cpu_percent(None)
        sys_cpu = psutil.cpu_percent(None)
        rss = sim.rss_mb()
        threads = sum(pp.num_threads() for key, pp in procs if key == "mavros")
        r = {"label": args.label, "n": n, "system_cpu_pct": round(sys_cpu, 1),
             "mavros_cpu_per_instance_pct_core": round(cpu["mavros"] / n, 1),
             "px4_cpu_per_instance_pct_core": round(cpu["px4"] / n, 1),
             "mavros_rss_per_instance_mb": round(rss["mavros"] / n, 1),
             "mavros_threads_per_instance": round(threads / n, 1)}
        print(json.dumps(r), flush=True)
        results.append(r)
    finally:
        sim.stop()
        time.sleep(2)
