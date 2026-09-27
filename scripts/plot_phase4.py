#!/usr/bin/env python3
"""Phase 4 figure: takeover times per fault against the acceptance limits, and the formation error around
every fault (from scripts/phase4_metrics.py output and each trial's formation_rms.csv).

Usage: python3 scripts/plot_phase4.py reports/logs/phase_4/phase4_table.json [--out reports/phase4_faults.png]
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

NAMES = {"F1": "F1 leader killed", "F2": "F2 leader's radio lost", "F3": "F3 leader low battery",
         "F4": "F4 follower killed", "F5": "F5 radio split, healed"}
SHORT = {"F1": "F1\nleader killed", "F2": "F2\nradio lost", "F3": "F3\nlow battery", "F5": "F5\nsplit, healed"}
COLORS = {"F1": "#b9372f", "F2": "#7a4fb3", "F3": "#c98a14", "F4": "#2767b3", "F5": "#17804f"}
LIMITS = {"F1": (3.0, 4.0), "F2": (3.0, 4.0), "F3": (1.0, 1.0), "F5": (3.0, 3.0)}   # (median, worst) in s


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("table")
    ap.add_argument("--out", default="reports/phase4_faults.png")
    args = ap.parse_args()
    table = json.loads(Path(args.table).read_text())
    trials = [r for r in table["trials"] if "error" not in r]
    base = Path(args.table).parent
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={"width_ratios": [1, 1.5]})

    faults = [f for f in ("F1", "F2", "F3", "F5") if any(r["fault"] == f for r in trials)]
    for k, f in enumerate(faults):
        vals = [r["handover_s"] for r in trials if r["fault"] == f and r.get("handover_s") is not None]
        ax1.scatter([k + 0.08 * ((j % 5) - 2) for j in range(len(vals))], vals, s=22, color=COLORS[f], zorder=3)
        if vals:
            med = statistics.median(vals)
            ax1.plot([k - 0.3, k + 0.3], [med, med], color=COLORS[f], lw=2.5)
        lim_med, lim_worst = LIMITS[f]
        ax1.plot([k - 0.38, k + 0.38], [lim_med, lim_med], color="#333", lw=1, ls="--")
        if lim_worst != lim_med:
            ax1.plot([k - 0.38, k + 0.38], [lim_worst, lim_worst], color="#333", lw=1, ls=":")
    ax1.set_xticks(range(len(faults)), [SHORT[f] for f in faults], fontsize=8)
    ax1.set_ylabel("seconds")
    ax1.set_ylim(bottom=0)
    ax1.set_title("New leader after the fault (F5: after the heal)\nbar: median, dashed: limit for the median, "
                  "dotted: limit for the worst", fontsize=9)
    ax1.grid(axis="y", alpha=0.3)

    for r in trials:
        f = r["fault"]
        ref = r.get("t_heal_s") if f == "F5" else r.get("t_trigger_s") if f == "F3" else r.get("t_fault_s")
        path = Path(r["dir"]) / "formation_rms.csv" if "dir" in r else base / r["trial"] / "formation_rms.csv"
        if ref is None or not path.exists():
            continue
        rows = [(float(x["t_s"]) - ref, float(x["rms_m"])) for x in csv.DictReader(open(path, encoding="utf-8"))]
        rows = [(t, v) for t, v in rows if -10.0 <= t <= 40.0]
        if rows:
            ax2.plot(*zip(*rows), color=COLORS[f], lw=0.9, alpha=0.75)
    for f in ("F1", "F2", "F3", "F4", "F5"):
        if any(r["fault"] == f for r in trials):
            ax2.plot([], [], color=COLORS[f], label=NAMES[f])
    ax2.axhline(2.0, color="#333", lw=1, ls="--")
    ax2.axvline(0.0, color="#333", lw=0.8)
    ax2.axvline(15.0, color="#333", lw=0.8, ls=":")
    ax2.set_xlabel("seconds after the fault (F3: after the battery trigger, F5: after the heal)")
    ax2.set_ylabel("formation RMS error (m)")
    ax2.set_title("Formation error around the fault (dashed: 2 m limit, dotted: 15 s)", fontsize=10)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
