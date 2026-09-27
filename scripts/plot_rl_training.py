#!/usr/bin/env python3
"""Learning curve of an RL run: training return (progress.csv) and the scores on the validation courses (eval.jsonl).

Reads both run formats: the laptop trainer (scripts/rl_train.py, Stable-Baselines3 progress.csv, one eval row per
check on 8 medium-density courses) and the GPU trainer (scripts/rl_train_gpu.py, e.g. a Kaggle run, one eval row
per check with scores for the low, medium and high density validation courses).

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

LEVEL_NAMES = {"0": "few obstacles", "1": "medium", "2": "dense"}
LEVEL_COLORS = {"0": "#17804f", "1": "#2767b3", "2": "#b9372f"}


def read_progress(path: Path) -> tuple[list[float], list[float], str]:
    steps, ret = [], []
    with open(path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    gpu = bool(rows) and "ep_return" in rows[0]
    for row in rows:
        if gpu:
            if row.get("ep_return"):
                steps.append(int(float(row["step"])) / 1e6)
                ret.append(float(row["ep_return"]))
        elif row.get("rollout/ep_rew_mean") and row.get("time/total_timesteps"):
            steps.append(int(float(row["time/total_timesteps"])) / 1e6)
            ret.append(float(row["rollout/ep_rew_mean"]))
    return steps, ret, "gpu" if gpu else "laptop"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", default="reports/rl_training.png")
    args = ap.parse_args()
    run = Path(args.run)
    steps, ret, kind = read_progress(run / "progress.csv")
    ev = [json.loads(l) for l in (run / "eval.jsonl").read_text().splitlines() if l.strip()]
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
    axes[0].plot(steps, ret, color="#2767b3")
    axes[0].set_title("Training return per follower episode")
    axes[0].set_xlabel("million steps")
    axes[0].grid(alpha=0.3)
    if ev and "levels" in ev[0]:                      # GPU run: fractions per density level
        es = [e["step"] / 1e6 for e in ev]
        for lv in sorted(ev[0]["levels"]):
            axes[1].plot(es, [e["levels"][lv]["success"] for e in ev], "o-", ms=3, color=LEVEL_COLORS.get(lv, "#555"),
                         label=f"{LEVEL_NAMES.get(lv, lv)}: crash-free and formation restored")
        axes[1].set_ylim(0, 1)
        n = sum(v["n"] for v in ev[0]["levels"].values())
        axes[1].set_ylabel("share of validation missions")
        axes[1].set_title(f"Validation courses during training ({n} courses)")
        axes[1].legend(fontsize=8, loc="upper left")
    else:                                             # laptop run: counts on the medium-density courses
        es = [e["steps"] / 1e6 for e in ev]
        axes[1].plot(es, [e["crash_free"] for e in ev], "o-", color="#17804f", label="missions without a crash")
        axes[1].plot(es, [e["success"] for e in ev], "s-", color="#2767b3", label="crash-free and formation restored")
        ax2 = axes[1].twinx()
        ax2.plot(es, [e["crashes"] for e in ev], "^--", color="#b9372f", label="crashes per mission")
        ax2.set_ylabel("crashes per mission", color="#b9372f")
        axes[1].set_ylim(0, max(e["n"] for e in ev))
        axes[1].set_ylabel(f"of {ev[0]['n']} held-out missions")
        axes[1].set_title("Held-out scenarios during training (medium density)")
        h1, l1 = axes[1].get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        axes[1].legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")
    axes[1].set_xlabel("million steps")
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"wrote {args.out} ({kind} run, {len(ev)} evaluations)")


if __name__ == "__main__":
    main()
