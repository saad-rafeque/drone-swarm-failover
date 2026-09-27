#!/usr/bin/env python3
"""PPO on the GPU for the formation obstacle-avoidance policy (batched simulator: swarm_tools/torch_sim.py).

The algorithm follows Stable-Baselines3's PPO defaults used by scripts/rl_train.py (separate tanh actor
and critic, state-independent log-std, GAE, clipped surrogate, advantage normalisation, VecNormalize-style
reward scaling, truncation bootstrapping) but keeps rollouts, rewards and updates on the GPU, the same
structure as CleanRL's ppo_continuous_action.py. The actor is exported in the numpy format read by
swarm_agent.avoidance.LearnedPolicy, so a policy trained here drops straight into the evaluation scripts,
the simulators and the agent.

Built for time-limited sessions (Kaggle: 12 h): it checkpoints every --checkpoint-min minutes, stops
cleanly when --hours is used up, and resumes from <run>/ckpt.pt when started again with the same --run.

Usage:
  PYTHONPATH=src python3 scripts/rl_train_gpu.py --pool data/pools/train.npz --eval-pool data/pools/val.npz \
      --run runs/seed1 --seed 1 --hours 9.5 [--envs 1024] [--steps 3e9]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import deque
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.avoidance import OBS_DIM  # noqa: E402
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.obstacle_sim import sim_cfg_from  # noqa: E402
from swarm_tools.torch_sim import BatchedFormationSim, ScenarioPool  # noqa: E402


def mlp(sizes: list[int], out_gain: float) -> nn.Sequential:
    layers: list[nn.Module] = []
    for a, b in zip(sizes[:-2], sizes[1:-1]):
        lin = nn.Linear(a, b)
        nn.init.orthogonal_(lin.weight, math.sqrt(2))
        nn.init.zeros_(lin.bias)
        layers += [lin, nn.Tanh()]
    out = nn.Linear(sizes[-2], sizes[-1])
    nn.init.orthogonal_(out.weight, out_gain)
    nn.init.zeros_(out.bias)
    return nn.Sequential(*layers, out)


class ActorCritic(nn.Module):
    def __init__(self, hidden: list[int], log_std_init: float) -> None:
        super().__init__()
        self.actor = mlp([OBS_DIM, *hidden, 2], 0.01)
        self.critic = mlp([OBS_DIM, *hidden, 1], 1.0)
        self.log_std = nn.Parameter(torch.full((2,), log_std_init))

    def dist(self, obs: torch.Tensor) -> torch.distributions.Normal:
        return torch.distributions.Normal(self.actor(obs), self.log_std.exp().expand(obs.shape[0], 2))

    def value(self, obs: torch.Tensor) -> torch.Tensor:
        return self.critic(obs).squeeze(-1)


def export_npz(model: ActorCritic, path: Path) -> None:
    lins = [m for m in model.actor if isinstance(m, nn.Linear)]
    out: dict[str, np.ndarray] = {"n_layers": np.array(len(lins) - 1)}
    for k, lin in enumerate(lins[:-1]):
        out[f"W{k}"] = lin.weight.detach().cpu().double().numpy()
        out[f"b{k}"] = lin.bias.detach().cpu().double().numpy()
    out["Wa"] = lins[-1].weight.detach().cpu().double().numpy()
    out["ba"] = lins[-1].bias.detach().cpu().double().numpy()
    tmp = path.with_suffix(".tmp.npz")
    np.savez(tmp, **out)
    os.replace(tmp, path)


class RewardScaler:
    """VecNormalize(norm_obs=False, norm_reward=True): divide by the running std of discounted returns."""

    def __init__(self, n: int, gamma: float, device, clip: float = 10.0, eps: float = 1e-8) -> None:
        self.ret = torch.zeros(n, device=device, dtype=torch.float64)
        self.mean, self.var, self.count = 0.0, 1.0, 1e-4
        self.gamma, self.clip, self.eps = gamma, clip, eps

    def __call__(self, rew: torch.Tensor, done: torch.Tensor) -> torch.Tensor:
        self.ret = self.ret * self.gamma + rew.double()
        x = self.ret
        b_mean, b_var, b_count = float(x.mean()), float(x.var(unbiased=False)), x.numel()
        delta, tot = b_mean - self.mean, self.count + b_count
        self.mean += delta * b_count / tot
        self.var = (self.var * self.count + b_var * b_count + delta ** 2 * self.count * b_count / tot) / tot
        self.count = tot
        self.ret = torch.where(done, torch.zeros_like(self.ret), self.ret)
        return (rew / math.sqrt(self.var + self.eps)).clamp(-self.clip, self.clip)

    def state(self) -> dict:
        return {"mean": self.mean, "var": self.var, "count": self.count}

    def load(self, s: dict) -> None:
        self.mean, self.var, self.count = s["mean"], s["var"], s["count"]


@torch.no_grad()
def evaluate(model: ActorCritic, cfg, pool: ScenarioPool, device, levels: np.ndarray) -> dict:
    """Deterministic policy on every held-out scenario once (episodes continue after a crash, as in rl_eval.py)."""
    sim = BatchedFormationSim(replace(cfg, end_on_crash=False), pool, pool.size, device, seed=12345)
    obs = sim.reset(scenario_idx=torch.arange(pool.size, device=device))
    done = torch.zeros(pool.size, dtype=torch.bool, device=device)
    results: dict[int, dict] = {}
    for _ in range(20000):
        act = model.actor(obs.reshape(-1, OBS_DIM)).reshape(pool.size, -1, 2)
        obs, _, _, _, over, _, finished = sim.step(act, auto_reset=False)
        idx = torch.nonzero(over & ~done).squeeze(1).tolist()
        fin = {f["seed"]: f for f in finished}
        for i in idx:
            results[i] = fin[int(pool.seeds[i])]
        done |= over
        if bool(done.all()):
            break
    out = {}
    for lv in sorted(set(levels.tolist())):
        rows = [results[i] for i in range(pool.size) if levels[i] == lv and i in results]
        n = max(len(rows), 1)
        out[str(lv)] = {"n": len(rows), "success": sum(r["success"] for r in rows) / n,
                        "crash_free": sum(r["crash_free"] for r in rows) / n,
                        "crashes": sum(r["crashes"] for r in rows) / n,
                        "restored": sum(r["restored"] for r in rows) / n,
                        "stuck": sum(r["stuck"] for r in rows) / n}
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pool", required=True)
    ap.add_argument("--eval-pool", default="", help="validation pool that picks best_policy.npz (val.npz from "
                    "build_pools.py; never the held-out test pool)")
    ap.add_argument("--run", required=True)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--envs", type=int, default=1024)
    ap.add_argument("--drones", type=int, default=10)
    ap.add_argument("--rollout", type=int, default=64)
    ap.add_argument("--epochs", type=int, default=4)
    ap.add_argument("--minibatches", type=int, default=8)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--gamma", type=float, default=0.99)
    ap.add_argument("--lam", type=float, default=0.95)
    ap.add_argument("--clip", type=float, default=0.2)
    ap.add_argument("--vf-coef", type=float, default=0.5)
    ap.add_argument("--ent-coef", type=float, default=0.0)
    ap.add_argument("--max-grad", type=float, default=0.5)
    ap.add_argument("--hidden", type=int, nargs="+", default=[128, 128])
    ap.add_argument("--log-std-init", type=float, default=-1.0)
    ap.add_argument("--steps", type=float, default=3e9, help="agent transitions in total (across sessions)")
    ap.add_argument("--hours", type=float, default=9.5, help="stop and checkpoint after this long (this session)")
    ap.add_argument("--checkpoint-min", type=float, default=20.0)
    ap.add_argument("--eval-min", type=float, default=30.0)
    ap.add_argument("--k-seg", type=int, default=64)
    ap.add_argument("--k-circ", type=int, default=48)
    args = ap.parse_args()

    t_start = time.time()
    run = Path(args.run)
    run.mkdir(parents=True, exist_ok=True)
    dev = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else
                       (args.device if args.device != "auto" else "cpu"))
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    cfg = sim_cfg_from(load_config(default_config_path()).with_num_drones(args.drones))
    pool = ScenarioPool.load(args.pool, dev)
    sim = BatchedFormationSim(cfg, pool, args.envs, dev, seed=args.seed, k_seg=args.k_seg, k_circ=args.k_circ)
    eval_pool, eval_levels = None, None
    if args.eval_pool:
        eval_pool = ScenarioPool.load(args.eval_pool, dev)
        with np.load(args.eval_pool) as z:
            eval_levels = z["levels"] if "levels" in z.files else np.zeros(eval_pool.size, dtype=int)
    model = ActorCritic(args.hidden, args.log_std_init).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr, eps=1e-5)
    A = args.envs * (args.drones - 1)
    scaler = RewardScaler(A, args.gamma, dev)
    step, update, best = 0, 0, None
    ck = run / "ckpt.pt"
    if ck.exists():
        s = torch.load(ck, map_location=dev, weights_only=False)
        model.load_state_dict(s["model"])
        opt.load_state_dict(s["opt"])
        scaler.load(s["scaler"])
        step, update, best = s["step"], s["update"], s.get("best")
        print(f"resumed from {ck}: step {step:,}, update {update}", flush=True)
    meta = {"args": vars(args), "sim": {k: str(v) for k, v in asdict(cfg).items()}, "device": str(dev),
            "gpu": torch.cuda.get_device_name(0) if dev.type == "cuda" else None, "torch": torch.__version__,
            "pool": {"scenarios": pool.size, "segments": int(pool.seg.shape[1]), "circles": int(pool.circ.shape[1])}}
    (run / f"session_{int(t_start)}.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta), flush=True)

    T = args.rollout
    buf_obs = torch.zeros(T, A, OBS_DIM, device=dev)
    buf_act = torch.zeros(T, A, 2, device=dev)
    buf_logp = torch.zeros(T, A, device=dev)
    buf_rew = torch.zeros(T, A, device=dev)
    buf_done = torch.zeros(T, A, device=dev)
    buf_val = torch.zeros(T, A, device=dev)
    obs = sim.reset().reshape(A, OBS_DIM)
    ep_ret = torch.zeros(A, device=dev)
    recent_ret: deque[float] = deque(maxlen=2000)
    recent_eps: deque[dict] = deque(maxlen=500)
    log_f = open(run / "progress.csv", "a", newline="")
    writer = csv.writer(log_f)
    if log_f.tell() == 0:
        writer.writerow(["time_s", "step", "update", "fps", "ep_return", "episodes", "success", "crash_free",
                         "crashes", "restored", "pg_loss", "v_loss", "entropy", "approx_kl", "clipfrac",
                         "explained_var", "log_std", "ray_overflow"])
    last_ck = last_eval = time.time()
    budget_s = args.hours * 3600.0

    def checkpoint(tag: str = "ckpt") -> None:
        tmp = run / "ckpt.tmp.pt"
        torch.save({"model": model.state_dict(), "opt": opt.state_dict(), "scaler": scaler.state(), "step": step,
                    "update": update, "best": best, "args": vars(args)}, tmp)
        os.replace(tmp, ck)
        export_npz(model, run / "policy.npz")
        print(f"[{tag}] step {step:,} saved", flush=True)

    while step < args.steps and time.time() - t_start < budget_s:
        t0 = time.time()
        for t in range(T):
            with torch.no_grad():
                d = model.dist(obs)
                act = d.sample()
                buf_logp[t] = d.log_prob(act).sum(-1)
                buf_val[t] = model.value(obs)
            buf_obs[t], buf_act[t] = obs, act
            nobs, rew, term, trunc, over, term_obs, finished = sim.step(act.reshape(args.envs, -1, 2))
            rew, term, trunc = rew.reshape(A).float(), term.reshape(A), trunc.reshape(A)
            done = over[:, None].expand(-1, args.drones - 1).reshape(A)
            ep_ret += rew
            if bool(done.any()):
                recent_ret.extend(ep_ret[done].tolist())
                ep_ret[done] = 0.0
                recent_eps.extend(finished)
            r = scaler(rew, done).float()
            if bool(trunc.any()):
                with torch.no_grad():
                    tv = model.value(term_obs.reshape(A, OBS_DIM).float())
                r = r + args.gamma * torch.where(trunc, tv, torch.zeros_like(tv))
            buf_rew[t], buf_done[t] = r, done.float()
            obs = nobs.reshape(A, OBS_DIM).float()
        with torch.no_grad():
            next_val = model.value(obs)
            adv = torch.zeros_like(buf_rew)
            last = torch.zeros(A, device=dev)
            for t in reversed(range(T)):
                nv = next_val if t == T - 1 else buf_val[t + 1]
                nonterm = 1.0 - buf_done[t]
                delta = buf_rew[t] + args.gamma * nv * nonterm - buf_val[t]
                last = delta + args.gamma * args.lam * nonterm * last
                adv[t] = last
            ret = adv + buf_val
        b_obs, b_act, b_logp = buf_obs.reshape(-1, OBS_DIM), buf_act.reshape(-1, 2), buf_logp.reshape(-1)
        b_adv, b_ret, b_val = adv.reshape(-1), ret.reshape(-1), buf_val.reshape(-1)
        n = b_obs.shape[0]
        mb = n // args.minibatches
        clipfracs, kls = [], []
        for _ in range(args.epochs):
            perm = torch.randperm(n, device=dev)
            for k in range(args.minibatches):
                i = perm[k * mb:(k + 1) * mb]
                d = model.dist(b_obs[i])
                logp = d.log_prob(b_act[i]).sum(-1)
                ratio = (logp - b_logp[i]).exp()
                a_ = b_adv[i]
                a_ = (a_ - a_.mean()) / (a_.std() + 1e-8)
                pg = torch.max(-a_ * ratio, -a_ * ratio.clamp(1 - args.clip, 1 + args.clip)).mean()
                v = model.value(b_obs[i])
                vl = 0.5 * ((v - b_ret[i]) ** 2).mean()
                ent = d.entropy().sum(-1).mean()
                loss = pg + args.vf_coef * vl - args.ent_coef * ent
                opt.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), args.max_grad)
                opt.step()
                with torch.no_grad():
                    kls.append(float(((ratio - 1) - (ratio.log())).mean()))
                    clipfracs.append(float(((ratio - 1).abs() > args.clip).float().mean()))
        step += n
        update += 1
        dt = time.time() - t0
        ev = float(1 - torch.var(b_ret - b_val) / torch.var(b_ret).clamp_min(1e-8))
        eps = list(recent_eps)
        k = max(len(eps), 1)
        row = [round(time.time() - t_start, 1), step, update, round(n / dt), round(float(np.mean(recent_ret)), 3) if recent_ret else "",
               len(eps), round(sum(e["success"] for e in eps) / k, 3), round(sum(e["crash_free"] for e in eps) / k, 3),
               round(sum(e["crashes"] for e in eps) / k, 3), round(sum(e["restored"] for e in eps) / k, 3),
               round(pg.item(), 4), round(vl.item(), 4), round(ent.item(), 3), round(float(np.mean(kls)), 5),
               round(float(np.mean(clipfracs)), 3), round(ev, 3), [round(x, 3) for x in model.log_std.tolist()],
               sim.ray_overflow]
        writer.writerow(row)
        log_f.flush()
        if update % 5 == 1:
            print(f"step {step:,} fps {row[3]:,} ret {row[4]} success {row[6]} crash_free {row[7]} crashes {row[8]} "
                  f"kl {row[13]} ev {row[15]}", flush=True)
        if eval_pool is not None and time.time() - last_eval > args.eval_min * 60:
            last_eval = time.time()
            res = evaluate(model, cfg, eval_pool, dev, eval_levels)
            rec = {"step": step, "time_s": round(time.time() - t_start), "levels": res}
            with open(run / "eval.jsonl", "a") as fh:
                fh.write(json.dumps(rec) + "\n")
            score = float(np.mean([v["success"] for v in res.values()]))
            if best is None or score > best:
                best = score
                export_npz(model, run / "best_policy.npz")
            print(f"[eval] {json.dumps(rec)}", flush=True)
        if time.time() - last_ck > args.checkpoint_min * 60:
            last_ck = time.time()
            checkpoint()
    checkpoint("final" if step >= args.steps else "budget")
    (run / "status.json").write_text(json.dumps({"step": step, "update": update, "done": step >= args.steps,
                                                 "session_hours": round((time.time() - t_start) / 3600, 2)}))
    log_f.close()


if __name__ == "__main__":
    main()
