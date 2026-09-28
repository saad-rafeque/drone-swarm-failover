#!/usr/bin/env python3
"""Phase 5 in the fast simulator: radio realism sweep of the heartbeat link, delay x loss, 10 drones.

Conditions: delay 50 / 150 / 300 ms x loss 0 / 10 / 30 % (no jitter), the same grid as the PX4 sweep in
scripts/phase5_runs.sh. For every condition and seed:
  none  a mission without a fault: false leader changes (a new leader after take-off although nothing
        failed), time with more than one leader, formation error while cruising, closest pair, goal reached;
  F1    the leader killed during cruise, and
  F2    the leader's radio cut: new leader agreed (s), formation back under 2 m (s), closest pair, goal.
Each condition is judged with the Phase 4 limits (new leader median 3.0 s / worst 4.0 s, formation back
within 15 s, closest pair >= 5 m, goal in >= 90 % of runs) and the Phase 5 rule (no false leader change at
<= 10 % loss). The fast simulator runs the real agent code (swarm_tools.puresim).

Usage: PYTHONPATH=src python3 scripts/radio_sweep.py --seeds 20 --jobs 3 \\
           --out reports/logs/phase_5_fastsim --png reports/phase5_radio_sweep.png
"""
from __future__ import annotations

import argparse
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
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.heartbeat import Phase, Role  # noqa: E402
from swarm_tools.puresim import PureSim  # noqa: E402

DELAYS_MS = (50, 150, 300)
LOSSES_PCT = (0, 10, 30)
MISSION_S = 330.0
LIMITS = {"handover_median_s": 3.0, "handover_worst_s": 4.0, "recovery_worst_s": 15.0, "min_sep_m": 5.0,
          "goal_share": 0.9}


def run_none(cfg, seed: int, delay_s: float, loss: float) -> dict:
    """A mission without a fault; every leader after the one that took off is a false leader change."""
    sim = PureSim(cfg, seed=seed, latency_s=delay_s, jitter_s=0.0, loss=loss)
    st = {"leaders": [], "multi_s": 0.0, "rms_max": 0.0, "rms_sum": 0.0, "rms_n": 0}

    def on_step(s: PureSim, stats) -> None:
        ms = stats.masters
        if not st["leaders"]:
            if len(ms) == 1 and s.agents[ms[0]].phase != Phase.IDLE:      # the swarm is taking off
                st["leaders"].append((ms[0], s.agents[ms[0]].election.term))
            return
        if len(ms) > 1:
            st["multi_s"] += s.dt
        for m in ms:
            key = (m, s.agents[m].election.term)
            if key not in st["leaders"]:
                st["leaders"].append(key)
        if stats.formation_rms is not None:
            st["rms_max"] = max(st["rms_max"], stats.formation_rms)
            st["rms_sum"] += stats.formation_rms
            st["rms_n"] += 1

    sim.run_until(MISSION_S, on_step)
    gx, gy = cfg.mission.goal_enu_m
    flyers = [i for i in sim.alive_ids() if sim.role(i) != Role.RETIRED]
    lead = min(flyers) if flyers else None
    return {"false_changes": max(0, len(st["leaders"]) - 1), "leaders": st["leaders"],
            "all_landed_leader_at_goal": bool(flyers) and all(sim.drones[i].landed for i in flyers)
            and math.hypot(sim.drones[lead].pos[0] - gx, sim.drones[lead].pos[1] - gy) < 60.0,
            "multi_leader_s": round(st["multi_s"], 2), "cruise_rms_max_m": round(st["rms_max"], 2),
            "cruise_rms_mean_m": round(st["rms_sum"] / st["rms_n"], 2) if st["rms_n"] else None,
            "min_sep_m": round(sim.min_sep_seen, 2),
            "goal_reached": all(math.hypot(sim.drones[i].pos[0] - gx, sim.drones[i].pos[1] - gy) < 60.0
                                and sim.drones[i].landed for i in flyers)}


def task(args: tuple) -> dict:
    kind, delay_ms, loss_pct, seed = args
    cfg = load_config(default_config_path()).with_num_drones(10)
    delay_s, loss = delay_ms / 1000.0, loss_pct / 100.0
    if kind == "none":
        res = run_none(cfg, seed, delay_s, loss)
    else:
        r = run_trial(cfg, kind, seed, delay_s, loss, jitter=0.0)
        res = {k: r[k] for k in ("handover_s", "rms_recovery_s", "min_sep_m", "goal_reached", "master_before",
                                 "master_after", "t_fault")}
    return {"kind": kind, "delay_ms": delay_ms, "loss_pct": loss_pct, "seed": seed, **res}


def summarize(rows: list[dict]) -> list[dict]:
    out = []
    for d in DELAYS_MS:
        for p in LOSSES_PCT:
            cond = [r for r in rows if r["delay_ms"] == d and r["loss_pct"] == p]
            none = [r for r in cond if r["kind"] == "none"]
            faults = [r for r in cond if r["kind"] in ("F1", "F2")]
            ho = [r["handover_s"] for r in faults if r["handover_s"] is not None]
            rec = [r["rms_recovery_s"] for r in faults if r["rms_recovery_s"] is not None]
            s = {"delay_ms": d, "loss_pct": p, "missions": len(none), "fault_trials": len(faults),
                 "false_changes_total": sum(r["false_changes"] for r in none),
                 "missions_with_false_change": sum(1 for r in none if r["false_changes"]),
                 "false_changes_per_mission": round(sum(r["false_changes"] for r in none) / len(none), 2),
                 "multi_leader_s_max": max(r["multi_leader_s"] for r in none),
                 "cruise_rms_max_m": max(r["cruise_rms_max_m"] for r in none),
                 "handover_measured": len(ho), "handover_median_s": round(statistics.median(ho), 2) if ho else None,
                 "handover_worst_s": round(max(ho), 2) if ho else None,
                 "recovery_measured": len(rec), "recovery_median_s": round(statistics.median(rec), 2) if rec else None,
                 "recovery_worst_s": round(max(rec), 2) if rec else None,
                 "min_sep_m": min(r["min_sep_m"] for r in cond),
                 "goal_reached": sum(1 for r in cond if r["goal_reached"]), "runs": len(cond)}
            s["checks"] = {
                "no_false_change": s["false_changes_total"] == 0,
                "handover": len(ho) == len(faults) and s["handover_median_s"] <= LIMITS["handover_median_s"]
                and s["handover_worst_s"] <= LIMITS["handover_worst_s"],
                "recovery": len(rec) == len(faults) and s["recovery_worst_s"] <= LIMITS["recovery_worst_s"],
                "separation": s["min_sep_m"] >= LIMITS["min_sep_m"],
                "goal": s["goal_reached"] >= LIMITS["goal_share"] * s["runs"]}
            s["phase4_criteria_hold"] = all(v for k, v in s["checks"].items() if k != "no_false_change")
            out.append(s)
    return out


def fmt(v, digits=2) -> str:
    return "-" if v is None else f"{v:.{digits}f}"


def markdown(summary: list[dict], seeds: int) -> str:
    lines = [f"Fast simulator, 10 drones, {seeds} seeds per condition and kind (missions without a fault, F1, F2); "
             "no jitter. Limits: new leader median 3.0 s / worst 4.0 s, formation back within 15 s, closest pair "
             ">= 5 m, goal in >= 90 % of runs; no false leader change at <= 10 % loss.", "",
             "| Delay (ms) | Loss (%) | False leader changes (missions affected) | New leader median / worst (s) | "
             "Formation < 2 m median / worst (s) | Closest pair (m) | Goal | Phase 4 limits | No false change |",
             "|---|---|---|---|---|---|---|---|---|"]
    for s in summary:
        lines.append(f"| {s['delay_ms']} | {s['loss_pct']} | {s['false_changes_total']} ({s['missions_with_false_change']}"
                     f"/{s['missions']}) | {fmt(s['handover_median_s'])} / {fmt(s['handover_worst_s'])} | "
                     f"{fmt(s['recovery_median_s'])} / {fmt(s['recovery_worst_s'])} | {fmt(s['min_sep_m'])} | "
                     f"{s['goal_reached']}/{s['runs']} | {'hold' if s['phase4_criteria_hold'] else 'FAIL'} | "
                     f"{'yes' if s['checks']['no_false_change'] else 'NO'} |")
    return "\n".join(lines) + "\n"


def figure(summary: list[dict], path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Rectangle

    ink, muted, critical = "#0b0b0b", "#52514e", "#d03b3b"
    ramps = {   # one hue per panel, light -> dark (reference palette: blue ramp; orange and aqua slots)
        "blue": ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"],
        "orange": ["#fde4d8", "#f6ab8a", "#eb6834", "#b8491d", "#7a2d0f"],
        "aqua": ["#d3f2e6", "#8fdcbd", "#1baf7a", "#137f58", "#0b5139"],
    }
    panels = [   # title, colour value, colour-scale top (the limit), over-limit test, cell text
        ("False leader changes per mission\n(no fault injected; target 0 at <= 10 % loss)", "blue",
         lambda s: s["false_changes_per_mission"], 1.0,
         lambda s: not s["checks"]["no_false_change"] and s["loss_pct"] <= 10,
         lambda s: (f"{s['false_changes_per_mission']:.2f}", f"{s['missions_with_false_change']}/{s['missions']} missions")),
        ("New leader after F1 / F2: median, and worst below (s)\n(limits: median 3.0 s, worst 4.0 s)", "orange",
         lambda s: s["handover_median_s"], LIMITS["handover_median_s"], lambda s: not s["checks"]["handover"],
         lambda s: (fmt(s["handover_median_s"]), f"worst {fmt(s['handover_worst_s'])}")),
        ("Formation back under 2 m after F1 / F2: worst (s)\n(limit 15 s)", "aqua",
         lambda s: s["recovery_worst_s"], LIMITS["recovery_worst_s"], lambda s: not s["checks"]["recovery"],
         lambda s: (fmt(s["recovery_worst_s"], 1), f"median {fmt(s['recovery_median_s'], 1)}")),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.8))
    for ax, (title, hue, value, vmax, over, text) in zip(axes, panels):
        grid = [[next(s for s in summary if s["delay_ms"] == d and s["loss_pct"] == p)
                 for p in LOSSES_PCT] for d in DELAYS_MS]
        vals = [[(value(c) if value(c) is not None else float("nan")) for c in row] for row in grid]
        cmap = LinearSegmentedColormap.from_list(hue, ramps[hue])
        ax.imshow(vals, cmap=cmap, vmin=0.0, vmax=vmax, aspect="auto")
        for r, row in enumerate(grid):
            for c, cell in enumerate(row):
                v = value(cell)
                rgb = cmap(min(1.0, (v or 0.0) / vmax))[:3]
                lum = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]
                txt_color = "#ffffff" if lum < 0.5 else ink
                main, sub = text(cell)
                flagged = over(cell)
                if flagged:
                    ax.add_patch(Rectangle((c - 0.46, r - 0.46), 0.92, 0.92, fill=False, lw=2.5, ec=critical))
                    ax.text(c, r + 0.31, "\u2715 over limit", ha="center", va="center", fontsize=8,
                            color=txt_color, fontweight="bold")
                ax.text(c, r - (0.2 if flagged else 0.1), main, ha="center", va="center", fontsize=12,
                        color=txt_color, fontweight="bold")
                ax.text(c, r + (0.08 if flagged else 0.22), sub, ha="center", va="center", fontsize=8, color=txt_color)
        ax.set_xticks(range(len(LOSSES_PCT)), [f"{p} %" for p in LOSSES_PCT])
        ax.set_yticks(range(len(DELAYS_MS)), [f"{d} ms" for d in DELAYS_MS])
        ax.set_xlabel("heartbeat loss", color=muted)
        ax.set_ylabel("heartbeat delay", color=muted)
        ax.tick_params(colors=muted, length=0)
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.set_title(title, fontsize=9.5, color=ink, loc="left")
    fig.suptitle("Radio sweep in the fast simulator (10 drones). Colour runs from 0 to each limit; numbers are the measured values", fontsize=11,
                 color=ink, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor="#ffffff")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--out", default="reports/logs/phase_5_fastsim")
    ap.add_argument("--png", default="reports/phase5_radio_sweep.png")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    tasks = [(k, d, p, s) for d in DELAYS_MS for p in LOSSES_PCT for k in ("none", "F1", "F2") for s in range(args.seeds)]
    t0 = time.time()
    rows = []
    with Pool(args.jobs) as pool, open(out / "radio_sweep.jsonl", "w", encoding="utf-8") as fh:
        for i, r in enumerate(pool.imap_unordered(task, tasks), 1):
            rows.append(r)
            fh.write(json.dumps(r) + "\n")
            if i % 45 == 0:
                print(f"{i}/{len(tasks)} runs, {time.time() - t0:.0f} s", flush=True)
    summary = summarize(rows)
    (out / "summary.json").write_text(json.dumps({"seeds": args.seeds, "limits": LIMITS, "wall_s": round(time.time() - t0),
                                                  "conditions": summary}, indent=1) + "\n")
    (out / "summary.md").write_text(markdown(summary, args.seeds))
    figure(summary, args.png)
    print(markdown(summary, args.seeds))
    print(f"wrote {out}/radio_sweep.jsonl, summary.json, summary.md and {args.png} in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
