#!/usr/bin/env python3
"""Tune the classical potential-field avoider on TRAINING scenarios, so the RL comparison is against
its best settings (held-out evaluation scenarios are never used here).

Usage: PYTHONPATH=src python3 scripts/rl_tune_apf.py [--scenarios 10] [--out reports/logs/rl/apf_tuning.jsonl]
Prints the best parameters; scripts/rl_eval.py reads them from the output file.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.avoidance import PotentialField  # noqa: E402
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.obstacle_sim import make_scenario, run_episode, sim_cfg_from  # noqa: E402

TUNE_SEED0 = 500_000          # inside the training range (below TRAIN_SEED_MAX), never used for evaluation


def score(rows: list[dict]) -> tuple:
    return (sum(r["success"] for r in rows), sum(r["crash_free"] for r in rows),
            -float(np.mean([r["crashes_obstacle"] + r["crashes_drone"] for r in rows])),
            -float(np.mean([r["final_rms_m"] if r["final_rms_m"] is not None else 99.0 for r in rows])))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", type=int, default=10)
    ap.add_argument("--out", default="reports/logs/rl/apf_tuning.jsonl")
    args = ap.parse_args()
    cfg = replace(sim_cfg_from(load_config(default_config_path()).with_num_drones(10)),
                  building_density=(1.0, 1.0), tree_density=(0.5, 0.5))
    scen = [make_scenario(TUNE_SEED0 + k, cfg) for k in range(args.scenarios)]
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    best = None
    with open(args.out, "w", encoding="utf-8") as fh:
        for d0, k_rep, k_tan in itertools.product((6.0, 8.0, 10.0), (5.0, 10.0, 20.0), (0.3, 1.0)):
            av = PotentialField(d0_m=d0, k_rep=k_rep, k_tan=k_tan)
            rows = [run_episode(cfg, s, av) for s in scen]
            sc = score(rows)
            rec = {"d0_m": d0, "k_rep": k_rep, "k_tan": k_tan, "score": sc, "success": sc[0], "crash_free": sc[1],
                   "crashes": -sc[2], "final_rms_m": -sc[3]}
            fh.write(json.dumps(rec) + "\n")
            fh.flush()
            print(rec, flush=True)
            if best is None or sc > best[0]:
                best = (sc, rec)
    print("BEST", json.dumps(best[1]))


if __name__ == "__main__":
    main()
