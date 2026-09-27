#!/usr/bin/env python3
"""Fair comparison of obstacle avoiders on held-out scenarios (never used in training or model selection).

Methods: none (formation law only), none+shield (stopping-distance brake only), apf (classical potential
field + brake, parameters tuned on training scenarios), rl (learned policy), rl+shield (policy + brake).
Every method flies exactly the same scenarios (same obstacles, same route, same wind drift seed).

Usage: PYTHONPATH=src python3 scripts/rl_eval.py --policy reports/logs/rl/run1/best_policy.npz
          [--episodes 30] [--drones 10] [--out reports/logs/rl/eval] [--jobs 3]
Writes episodes.jsonl, summary.json, summary.md and comparison.png in --out.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.avoidance import LearnedPolicy, NoAvoidance, PotentialField, Shielded  # noqa: E402
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.obstacle_sim import make_scenario, run_episode, sim_cfg_from  # noqa: E402

TRAIN_SEED_MAX = 1_000_000          # same constant as swarm_tools.rl_vecenv (kept here to avoid importing SB3)
EVAL_SEED0 = TRAIN_SEED_MAX + 100   # model selection uses TRAIN_SEED_MAX + 0..99 (laptop 0..7, GPU val.npz 0..29)
LEVELS = {"low": (0.5, 0.25), "medium": (1.0, 0.5), "high": (2.0, 1.0)}   # buildings / tree clusters per hectare
METHODS = ["none", "none+shield", "apf", "rl", "rl+shield"]


def best_apf(path: Path) -> dict:
    rows = [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []
    if not rows:
        return {}
    b = max(rows, key=lambda r: tuple(r["score"]))
    return {"d0_m": b["d0_m"], "k_rep": b["k_rep"], "k_tan": b["k_tan"]}


def build(method: str, policy: str, apf: dict):
    if method == "none":
        return NoAvoidance()
    if method == "none+shield":
        return Shielded(NoAvoidance())
    if method == "apf":
        return PotentialField(**apf)
    if method == "rl":
        return LearnedPolicy(policy)
    if method == "rl+shield":
        return Shielded(LearnedPolicy(policy))
    raise ValueError(method)


def job(args: tuple) -> list[dict]:
    level, method, seeds, drones, policy, apf = args
    b, t = LEVELS[level]
    cfg = replace(sim_cfg_from(load_config(default_config_path()).with_num_drones(drones)),
                  building_density=(b, b), tree_density=(t, t))
    av = build(method, policy, apf)
    out = []
    for s in seeds:
        r = run_episode(cfg, make_scenario(s, cfg), av, seed=s)
        r.update(level=level, method=method, drones=drones)
        out.append(r)
    return out


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def summarize(rows: list[dict]) -> dict:
    n = len(rows)
    f = lambda k: [r[k] for r in rows if r[k] is not None]  # noqa: E731
    succ = sum(r["success"] for r in rows)
    return {
        "episodes": n, "success": succ, "success_rate": succ / n, "success_ci95": wilson(succ, n),
        "crash_free_rate": sum(r["crash_free"] for r in rows) / n, "restored_rate": sum(r["restored"] for r in rows) / n,
        "crashes_per_episode": float(np.mean([r["crashes_obstacle"] + r["crashes_drone"] for r in rows])),
        "stuck_per_episode": float(np.mean([r["stuck"] or 0 for r in rows])),
        "sep_violation_s": float(np.mean(f("sep_violation_s"))), "min_sep_m": float(min(f("min_sep_m"))),
        "min_clearance_m": float(min(f("min_clearance_m"))), "mean_slot_err_m": float(np.mean(f("mean_slot_err_m"))),
        "final_rms_median_m": float(np.median(f("final_rms_m"))) if f("final_rms_m") else None,
        "mean_correction_mps": float(np.mean(f("mean_correction_mps"))),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", required=True)
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--drones", type=int, default=10)
    ap.add_argument("--out", default="reports/logs/rl/eval")
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--apf-tuning", default="reports/logs/rl/apf_tuning.jsonl")
    ap.add_argument("--methods", nargs="+", default=METHODS)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    apf = best_apf(Path(args.apf_tuning))
    seeds = [EVAL_SEED0 + k for k in range(args.episodes)]
    chunks = [seeds[i::3] for i in range(3)]
    tasks = [(lv, m, ch, args.drones, args.policy, apf) for lv in LEVELS for m in args.methods for ch in chunks]
    rows: list[dict] = []
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        for res in ex.map(job, tasks):
            rows.extend(res)
    with open(out / "episodes.jsonl", "w", encoding="utf-8") as fh:
        for r in sorted(rows, key=lambda r: (r["level"], r["method"], r["seed"])):
            fh.write(json.dumps(r) + "\n")
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        groups[(r["level"], r["method"])].append(r)
    summary = {f"{lv}/{m}": summarize(groups[(lv, m)]) for lv in LEVELS for m in args.methods}
    # paired comparison on identical scenarios: learned policy vs classical
    paired = {}
    for lv in LEVELS:
        a = {r["seed"]: r["success"] for r in groups[(lv, "rl")]} if "rl" in args.methods else {}
        b = {r["seed"]: r["success"] for r in groups[(lv, "apf")]} if "apf" in args.methods else {}
        common = sorted(set(a) & set(b))
        paired[lv] = {"rl_only": sum(a[s] and not b[s] for s in common), "apf_only": sum(b[s] and not a[s] for s in common),
                      "both": sum(a[s] and b[s] for s in common), "neither": sum(not a[s] and not b[s] for s in common)}
    meta = {"policy": args.policy, "apf_params": apf, "drones": args.drones, "episodes": args.episodes,
            "seeds": [seeds[0], seeds[-1]], "levels": LEVELS}
    (out / "summary.json").write_text(json.dumps({"meta": meta, "summary": summary, "paired": paired}, indent=2))
    lines = [f"Held-out scenarios {seeds[0]}..{seeds[-1]}, {args.drones} drones, APF {apf}", "",
             "| Level | Method | Success (95 % CI) | Crash-free | Formation restored | Crashes / ep | Stuck / ep | "
             "< 5 m time (s) | Min sep (m) | Mean slot err (m) | Final RMS (m) |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for lv in LEVELS:
        for m in args.methods:
            s = summary[f"{lv}/{m}"]
            lo, hi = s["success_ci95"]
            lines.append(f"| {lv} | {m} | {s['success']}/{s['episodes']} ({lo:.0%}–{hi:.0%}) | {s['crash_free_rate']:.0%} | "
                         f"{s['restored_rate']:.0%} | {s['crashes_per_episode']:.2f} | {s['stuck_per_episode']:.2f} | "
                         f"{s['sep_violation_s']:.1f} | {s['min_sep_m']:.2f} | {s['mean_slot_err_m']:.2f} | "
                         f"{s['final_rms_median_m'] if s['final_rms_median_m'] is None else round(s['final_rms_median_m'], 2)} |")
    lines += ["", "Paired on identical scenarios (success): " + "; ".join(
        f"{lv}: RL only {p['rl_only']}, APF only {p['apf_only']}, both {p['both']}, neither {p['neither']}"
        for lv, p in paired.items())]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
        x = np.arange(len(LEVELS))
        w = 0.8 / len(args.methods)
        for k, m in enumerate(args.methods):
            for ax, key, scale in ((axes[0], "success_rate", 100), (axes[1], "crashes_per_episode", 1),
                                   (axes[2], "stuck_per_episode", 1)):
                ax.bar(x + k * w, [summary[f"{lv}/{m}"][key] * scale for lv in LEVELS], w, label=m)
        for ax, title in zip(axes, ("Success rate (%)", "Crashes per mission", "Drones left behind per mission")):
            ax.set_xticks(x + w * (len(args.methods) - 1) / 2, list(LEVELS))
            ax.set_title(title)
            ax.grid(axis="y", alpha=0.3)
        axes[0].legend(fontsize=8)
        fig.suptitle(f"Obstacle avoidance in formation, {args.drones} drones, {args.episodes} held-out routes per level")
        fig.tight_layout()
        fig.savefig(out / "comparison.png", dpi=120)
    except Exception as exc:  # pragma: no cover
        print("plot skipped:", exc)


if __name__ == "__main__":
    main()
