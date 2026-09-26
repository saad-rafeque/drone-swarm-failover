#!/usr/bin/env python3
"""Train the formation obstacle-avoidance policy with PPO (Stable-Baselines3) on the fast simulator.

Every follower drone is one PPO "environment" and all of them share one policy. During training the
policy is scored on held-out scenarios (seeds the training never draws) and the best one is kept.

Usage:
  PYTHONPATH=src .venv/bin/python scripts/rl_train.py --steps 6000000 --run reports/logs/rl/run1
Writes (in --run): model.zip, policy.npz (numpy weights for swarm_agent.avoidance.LearnedPolicy),
best_policy.npz, progress.csv (SB3 log), eval.jsonl (held-out scores), config.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import VecMonitor, VecNormalize

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.avoidance import LearnedPolicy  # noqa: E402
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.obstacle_sim import make_scenario, run_episode, sim_cfg_from  # noqa: E402
from swarm_tools.rl_vecenv import TRAIN_SEED_MAX, SwarmVecEnv  # noqa: E402

EVAL_DENSITY = {"building_density": (1.0, 1.0), "tree_density": (0.5, 0.5)}   # the "medium" level


def export_policy(model: PPO, path: Path) -> None:
    """Deterministic actor (hidden tanh layers + linear action head) as numpy arrays."""
    linears = [m for m in model.policy.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    out: dict[str, np.ndarray] = {"n_layers": np.array(len(linears))}
    for k, lin in enumerate(linears):
        out[f"W{k}"] = lin.weight.detach().cpu().numpy().astype(np.float64)
        out[f"b{k}"] = lin.bias.detach().cpu().numpy().astype(np.float64)
    out["Wa"] = model.policy.action_net.weight.detach().cpu().numpy().astype(np.float64)
    out["ba"] = model.policy.action_net.bias.detach().cpu().numpy().astype(np.float64)
    np.savez(path, **out)


class HeldOutEval(BaseCallback):
    def __init__(self, run: Path, sim_cfg, every: int, n_scenarios: int) -> None:
        super().__init__()
        self.run, self.every, self.next_at = run, every, every
        self.cfg = replace(sim_cfg, **EVAL_DENSITY)
        self.scenarios = [make_scenario(TRAIN_SEED_MAX + k, self.cfg) for k in range(n_scenarios)]
        self.best: tuple | None = None

    def _on_step(self) -> bool:
        if self.num_timesteps < self.next_at:
            return True
        self.next_at += self.every
        tmp = self.run / "policy.npz"
        export_policy(self.model, tmp)
        pol = LearnedPolicy(tmp)
        t0 = time.perf_counter()
        res = [run_episode(self.cfg, sc, pol) for sc in self.scenarios]
        row = {
            "steps": int(self.num_timesteps), "wall_s": round(time.perf_counter() - t0, 1),
            "success": sum(r["success"] for r in res), "crash_free": sum(r["crash_free"] for r in res),
            "restored": sum(r["restored"] for r in res), "n": len(res),
            "crashes": float(np.mean([r["crashes_obstacle"] + r["crashes_drone"] for r in res])),
            "sep_violation_s": float(np.mean([r["sep_violation_s"] for r in res])),
            "mean_err_m": float(np.mean([r["mean_slot_err_m"] for r in res])),
            "final_rms_m": float(np.mean([r["final_rms_m"] if r["final_rms_m"] is not None else 99.0 for r in res])),
        }
        with open(self.run / "eval.jsonl", "a", encoding="utf-8") as fh:
            fh.write(json.dumps(row) + "\n")
        key = (row["success"], row["crash_free"], -row["crashes"], -row["final_rms_m"])
        if self.best is None or key > self.best:
            self.best = key
            export_policy(self.model, self.run / "best_policy.npz")
            self.model.save(self.run / "best_model.zip")
        print(f"[eval] {row}", flush=True)
        return True


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=6_000_000)
    ap.add_argument("--run", default="reports/logs/rl/run1")
    ap.add_argument("--n-sims", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--eval-every", type=int, default=500_000)
    ap.add_argument("--eval-scenarios", type=int, default=8)
    ap.add_argument("--drones", type=int, default=10)
    args = ap.parse_args()
    run = Path(args.run)
    run.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(2)
    sim_cfg = sim_cfg_from(load_config(default_config_path()).with_num_drones(args.drones))
    hyper = dict(n_steps=512, batch_size=2304, n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                 clip_range=0.2, ent_coef=0.0, max_grad_norm=0.5)
    policy_kwargs = dict(net_arch=dict(pi=[128, 128], vf=[128, 128]), activation_fn=torch.nn.Tanh, log_std_init=-1.0)
    (run / "config.json").write_text(json.dumps({
        "args": vars(args), "sim": {k: v for k, v in asdict(sim_cfg).items()}, "ppo": hyper,
        "policy": {"net_arch": [128, 128], "activation": "tanh", "log_std_init": -1.0},
        "eval_density": EVAL_DENSITY, "torch": torch.__version__}, indent=2, default=str))
    env = VecNormalize(VecMonitor(SwarmVecEnv(sim_cfg, args.n_sims, seed=args.seed)), norm_obs=False, norm_reward=True)
    model = PPO("MlpPolicy", env, seed=args.seed, verbose=1, device="cpu", policy_kwargs=policy_kwargs, **hyper)
    model.set_logger(configure(str(run), ["stdout", "csv"]))
    t0 = time.time()
    model.learn(total_timesteps=args.steps, callback=HeldOutEval(run, sim_cfg, args.eval_every, args.eval_scenarios))
    model.save(run / "model.zip")
    export_policy(model, run / "policy.npz")
    print(f"done in {(time.time() - t0) / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
