#!/usr/bin/env python3
"""Build the scenario pools for GPU training (scripts/rl_train_gpu.py).

  train.npz    random obstacle courses from the training seed range (densities drawn from the
               training ranges in SimCfg), packed with swarm_tools.torch_sim.pack_scenarios
  heldout.npz  exactly the 90 held-out courses of scripts/rl_eval.py (30 per density level, same
               seeds), with a "levels" array, so GPU-side scores match the laptop evaluation

Usage: PYTHONPATH=src python3 scripts/build_pools.py [--train 8000] [--workers 4] [--out data/pools]
"""
from __future__ import annotations

import argparse
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.obstacle_sim import make_scenario, sim_cfg_from  # noqa: E402

TRAIN_SEED_MAX = 1_000_000                      # as in swarm_tools.rl_vecenv
EVAL_SEED0 = TRAIN_SEED_MAX + 100               # as in scripts/rl_eval.py
LEVELS = [("low", (0.5, 0.25)), ("medium", (1.0, 0.5)), ("high", (2.0, 1.0))]


def _make(args: tuple):
    seed, dens = args
    cfg = sim_cfg_from(load_config(default_config_path()).with_num_drones(10))
    if dens is not None:
        cfg = replace(cfg, building_density=(dens[0], dens[0]), tree_density=(dens[1], dens[1]))
    return make_scenario(seed, cfg)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", type=int, default=8000)
    ap.add_argument("--heldout-per-level", type=int, default=30)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="data/pools")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    from swarm_tools.torch_sim import pack_scenarios          # needs torch (present on Kaggle and in .venv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    train_seeds = rng.choice(TRAIN_SEED_MAX, size=args.train, replace=False).tolist()
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        train = list(ex.map(_make, [(s, None) for s in train_seeds], chunksize=16))
        held_args = [(EVAL_SEED0 + k, dens) for _, dens in LEVELS for k in range(args.heldout_per_level)]
        held = list(ex.map(_make, held_args, chunksize=4))
    a = pack_scenarios(train)
    np.savez_compressed(out / "train.npz", **a)
    h = pack_scenarios(held)
    h["levels"] = np.repeat(np.arange(len(LEVELS)), args.heldout_per_level)
    np.savez_compressed(out / "heldout.npz", **h)
    print(f"train: {len(train)} scenarios, held-out: {len(held)} ({', '.join(n for n, _ in LEVELS)}), "
          f"{time.time() - t0:.0f} s -> {out}/train.npz, {out}/heldout.npz")


if __name__ == "__main__":
    main()
