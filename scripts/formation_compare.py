#!/usr/bin/env python3
"""Compare the formation shapes (V, line, column, echelon) in the fast simulator with the real agent code.

For every shape and seed (10 drones, clean radio): a mission without a fault (formation error while
cruising, closest pair, goal), F1 (leader killed: new leader, formation back under 2 m), F4 (a follower
killed: formation back under 2 m) and F5 (radio split, then healed: one leader again, formation back).
Writes <out>/runs.jsonl, <out>/summary.json, <out>/summary.md and a figure with the four shapes seen from
above and the key numbers.

Usage: PYTHONPATH=src python3 scripts/formation_compare.py --seeds 10 --jobs 2 \\
           --out reports/logs/formations --png reports/formation_shapes.png
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import statistics
import sys
import time
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from puresim_faults import run_trial  # noqa: E402
from radio_sweep import run_none  # noqa: E402
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.formation import SHAPES, initial_layout  # noqa: E402

KINDS = ("none", "F1", "F4", "F5")
NAMES = {"V": "V", "line": "Line abreast", "column": "Column", "echelon": "Echelon (right)"}


def cfg_for(shape: str):
    cfg = load_config(default_config_path()).with_num_drones(10)
    return dataclasses.replace(cfg, formation=dataclasses.replace(cfg.formation, shape=shape))


def task(args: tuple) -> dict:
    shape, kind, seed = args
    cfg = cfg_for(shape)
    if kind == "none":
        res = run_none(cfg, seed, 0.0, 0.0)
        res.pop("leaders")
    else:
        r = run_trial(cfg, kind, seed, 0.0, 0.0, jitter=0.0)
        res = {k: r[k] for k in ("handover_s", "rms_recovery_s", "min_sep_m", "goal_reached",
                                 "all_landed_leader_at_goal")}
    return {"shape": shape, "kind": kind, "seed": seed, **res}


def med(vals):
    vals = [v for v in vals if v is not None]
    return round(statistics.median(vals), 2) if vals else None


def worst(vals):
    vals = [v for v in vals if v is not None]
    return round(max(vals), 2) if vals else None


def summarize(rows: list[dict]) -> list[dict]:
    out = []
    for shape in SHAPES:
        rs = [r for r in rows if r["shape"] == shape]
        by = {k: [r for r in rs if r["kind"] == k] for k in KINDS}
        out.append({
            "shape": shape, "runs": len(rs),
            "cruise_rms_mean_m": med([r["cruise_rms_mean_m"] for r in by["none"]]),
            "cruise_rms_max_m": worst([r["cruise_rms_max_m"] for r in by["none"]]),
            "f1_new_leader_median_s": med([r["handover_s"] for r in by["F1"]]),
            "f1_recovery_median_s": med([r["rms_recovery_s"] for r in by["F1"]]),
            "f1_recovery_worst_s": worst([r["rms_recovery_s"] for r in by["F1"]]),
            "f4_recovery_median_s": med([r["rms_recovery_s"] for r in by["F4"]]),
            "f5_one_leader_median_s": med([r["handover_s"] for r in by["F5"]]),
            "f5_recovery_median_s": med([r["rms_recovery_s"] for r in by["F5"]]),
            "f5_recovery_worst_s": worst([r["rms_recovery_s"] for r in by["F5"]]),
            "min_sep_m": min(r["min_sep_m"] for r in rs),
            "runs_under_5m": {k: sum(1 for r in by[k] if r["min_sep_m"] < 5.0) for k in KINDS},
            "runs_under_5m_total": sum(1 for r in rs if r["kind"] != "none" and r["min_sep_m"] < 5.0),
            "goal_reached": sum(1 for r in rs if r["all_landed_leader_at_goal"]),
        })
    return out


def fmt(v, digits=2) -> str:
    return "-" if v is None else f"{v:.{digits}f}"


def markdown(summary: list[dict], seeds: int) -> str:
    lines = [f"Fast simulator, 10 drones, clean radio, {seeds} seeds per shape and kind (no fault, F1 leader killed, "
             "F4 follower killed, F5 radio split then healed). Formation error = RMS distance of the followers "
             "from their slots. Goal = every drone still flying landed and the leader landed within 60 m of the goal "
             "(a long column or echelon lands its tail further back).", "",
             "| Shape | Cruise error mean / worst (m) | F1 new leader (s) | F1 formation back median / worst (s) | "
             "F4 formation back (s) | F5 one leader after heal (s) | F5 formation back median / worst (s) | "
             "Closest pair (m) | Runs under 5 m (F1 / F4 / F5) | Goal |", "|---|---|---|---|---|---|---|---|---|---|"]
    for s in summary:
        lines.append(f"| {NAMES[s['shape']]} | {fmt(s['cruise_rms_mean_m'])} / {fmt(s['cruise_rms_max_m'])} | "
                     f"{fmt(s['f1_new_leader_median_s'])} | {fmt(s['f1_recovery_median_s'])} / "
                     f"{fmt(s['f1_recovery_worst_s'])} | {fmt(s['f4_recovery_median_s'])} | "
                     f"{fmt(s['f5_one_leader_median_s'])} | {fmt(s['f5_recovery_median_s'])} / "
                     f"{fmt(s['f5_recovery_worst_s'])} | {fmt(s['min_sep_m'])} | "
                     f"{s['runs_under_5m']['F1']} / {s['runs_under_5m']['F4']} / {s['runs_under_5m']['F5']} | "
                     f"{s['goal_reached']}/{s['runs']} |")
    return "\n".join(lines) + "\n"


def figure(summary: list[dict], path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ink, muted, grid, bar, leader = "#0b0b0b", "#52514e", "#e4e3df", "#2a78d6", "#eb6834"
    cfg = cfg_for("V")
    half = math.radians(cfg.formation.v_half_angle_deg)
    fig = plt.figure(figsize=(13, 6.6))
    gs = fig.add_gridspec(2, 4, height_ratios=[1.15, 1.0], hspace=0.45, wspace=0.3)
    for k, shape in enumerate(SHAPES):                       # top row: each shape seen from above, flying up
        ax = fig.add_subplot(gs[0, k])
        lay = initial_layout(range(1, 11), math.pi / 2, cfg.formation.spacing_m, half, shape)
        for i, (e, n) in lay.items():
            ax.scatter([e], [n], s=110 if i == 1 else 60, color=leader if i == 1 else bar, zorder=3,
                       edgecolor="#ffffff", linewidth=1.5)
            side = shape in ("column", "echelon")          # label beside the dot where drones line up vertically
            ax.annotate(str(i), (e, n), textcoords="offset points", xytext=(7, -3) if side else (0, 7),
                        ha="left" if side else "center", fontsize=7, color=muted)
        ax.annotate("", xy=(0, 16), xytext=(0, 6), arrowprops={"arrowstyle": "->", "color": muted, "lw": 1.2})
        ax.set_title(NAMES[shape], fontsize=10, color=ink, loc="left")
        ax.set_aspect("equal")
        ax.set_xlim(-75, 80)
        ax.set_ylim(-105, 25)
        ax.set_xticks([])
        ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color(grid)
    ax.text(1.0, -0.08, "leader in orange; arrow: flight direction; 10 m between neighbours", transform=ax.transAxes,
            ha="right", va="top", fontsize=7.5, color=muted)
    labels = [NAMES[s["shape"]] for s in summary]
    panels = [("Formation back under 2 m after the\nleader is lost (F1), median (s)", "f1_recovery_median_s", None),
              ("Formation back under 2 m after a\nradio split heals (F5), worst (s)", "f5_recovery_worst_s", None),
              ("Fault runs closer than 5 m\n(of 30: F1, F4, F5)", "runs_under_5m_total", None),
              ("Closest pair over all runs (m)\n(limit 5 m)", "min_sep_m", 5.0)]
    for k, (title, key, limit) in enumerate(panels):
        ax = fig.add_subplot(gs[1, k])
        vals = [s[key] if s[key] is not None else 0.0 for s in summary]
        ax.bar(range(len(vals)), vals, width=0.55, color=bar, zorder=2)
        for x, v in enumerate(vals):
            label = f"{v:.0f}" if key == "runs_under_5m_total" else f"{v:.2f}" if v < 10 else f"{v:.1f}"
            ax.text(x, v, label, ha="center", va="bottom", fontsize=8, color=ink)
        if limit is not None:
            ax.axhline(limit, color=muted, lw=1, ls="--", zorder=1)
        ax.set_xticks(range(len(labels)), [lb.replace(" (right)", "").replace(" abreast", "") for lb in labels],
                      fontsize=8, color=muted)
        ax.set_title(title, fontsize=9, color=ink, loc="left")
        ax.grid(axis="y", color=grid, lw=0.8, zorder=0)
        ax.tick_params(colors=muted, length=0)
        ax.set_ylim(0, max(vals + [limit or 0]) * 1.2 or 1)
        for sp in ("top", "right", "left"):
            ax.spines[sp].set_visible(False)
        ax.spines["bottom"].set_color(grid)
    fig.suptitle("Formation shapes in the fast simulator (10 drones, same agent code)", fontsize=11, color=ink,
                 x=0.01, ha="left")
    fig.savefig(path, dpi=130, facecolor="#ffffff", bbox_inches="tight")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default="reports/logs/formations")
    ap.add_argument("--png", default="reports/formation_shapes.png")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(shape, kind, seed) for shape in SHAPES for kind in KINDS for seed in range(args.seeds)]
    t0 = time.time()
    rows = []
    with Pool(args.jobs) as pool, open(out / "runs.jsonl", "w", encoding="utf-8") as fh:
        for r in pool.imap_unordered(task, tasks):
            rows.append(r)
            fh.write(json.dumps(r) + "\n")
    summary = summarize(rows)
    (out / "summary.json").write_text(json.dumps({"seeds": args.seeds, "wall_s": round(time.time() - t0),
                                                  "shapes": summary}, indent=1) + "\n")
    (out / "summary.md").write_text(markdown(summary, args.seeds))
    figure(summary, args.png)
    print(markdown(summary, args.seeds))
    print(f"wrote {out}/runs.jsonl, summary.json, summary.md and {args.png} in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
