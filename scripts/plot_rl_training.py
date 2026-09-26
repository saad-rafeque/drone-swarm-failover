#!/usr/bin/env python3
"""Learning curve of an RL run: training return (SB3 progress.csv) and held-out scores (eval.jsonl).

Usage: python3 scripts/plot_rl_training.py reports/logs/rl/run1 [--out reports/rl_training.png]
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", default="reports/rl_training.png")
    args = ap.parse_args()
    run = Path(args.run)
    steps, ret = [], []
    with open(run / "progress.csv", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if row.get("rollout/ep_rew_mean") and row.get("time/total_timesteps"):
                steps.append(int(float(row["time/total_timesteps"])) / 1e6)
                ret.append(float(row["rollout/ep_rew_mean"]))
    ev = [json.loads(l) for l in (run / "eval.jsonl").read_text().splitlines() if l.strip()]
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
    axes[0].plot(steps, ret, color="#2767b3")
    axes[0].set_title("Training return per follower episode")
    axes[0].set_xlabel("million steps")
    axes[0].grid(alpha=0.3)
    es = [e["steps"] / 1e6 for e in ev]
    axes[1].plot(es, [e["crash_free"] for e in ev], "o-", color="#17804f", label="missions without a crash")
    axes[1].plot(es, [e["success"] for e in ev], "s-", color="#2767b3", label="crash-free and formation restored")
    ax2 = axes[1].twinx()
    ax2.plot(es, [e["crashes"] for e in ev], "^--", color="#b9372f", label="crashes per mission")
    ax2.set_ylabel("crashes per mission", color="#b9372f")
    axes[1].set_ylim(0, max(e["n"] for e in ev))
    axes[1].set_ylabel(f"of {ev[0]['n']} held-out missions")
    axes[1].set_xlabel("million steps")
    axes[1].set_title("Held-out scenarios during training (medium density)")
    axes[1].grid(alpha=0.3)
    h1, l1 = axes[1].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    axes[1].legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
