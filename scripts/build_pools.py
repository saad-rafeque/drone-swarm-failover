#!/usr/bin/env python3
"""Build the scenario pools for GPU training (scripts/rl_train_gpu.py).

  train.npz    random obstacle courses from the training seed range (densities drawn from the
               training ranges in SimCfg), packed with swarm_tools.torch_sim.pack_scenarios
  val.npz      90 validation courses (30 per density level, seeds TRAIN_SEED_MAX + 0..29) with a
               "levels" array: the trainer picks best_policy.npz with these. They are never trained
               on and never used for the reported results.
  heldout.npz  exactly the 90 test courses of scripts/rl_eval.py (seeds TRAIN_SEED_MAX + 100..129),
               for reporting only - never pass it to the trainer, or the best policy would be chosen
               on the test courses and the comparison would no longer be fair

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
VAL_SEED0 = TRAIN_SEED_MAX                      # validation (model selection): TRAIN_SEED_MAX + 0..99 is reserved for it
EVAL_SEED0 = TRAIN_SEED_MAX + 100               # test, as in scripts/rl_eval.py
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
        k_per = args.heldout_per_level
        assert k_per <= EVAL_SEED0 - VAL_SEED0, "validation seeds would run into the test seeds"
        val = list(ex.map(_make, [(VAL_SEED0 + k, dens) for _, dens in LEVELS for k in range(k_per)], chunksize=4))
        held = list(ex.map(_make, [(EVAL_SEED0 + k, dens) for _, dens in LEVELS for k in range(k_per)], chunksize=4))
    levels = np.repeat(np.arange(len(LEVELS)), args.heldout_per_level)
    np.savez_compressed(out / "train.npz", **pack_scenarios(train))
    np.savez_compressed(out / "val.npz", **pack_scenarios(val), levels=levels)
    np.savez_compressed(out / "heldout.npz", **pack_scenarios(held), levels=levels)
    print(f"train: {len(train)} scenarios, validation: {len(val)}, test (held-out): {len(held)} "
          f"({', '.join(n for n, _ in LEVELS)}), {time.time() - t0:.0f} s -> {out}/train.npz, val.npz, heldout.npz")


if __name__ == "__main__":
    main()
