#!/usr/bin/env python3
"""Plot one mission run: tracks (with V snapshots), formation in the master's frame, RMS over time.

Usage: python3 scripts/plot_mission.py <run_dir> <out.png> [title]
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.formation import assign_slots, v_slot_body  # noqa: E402

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
MASTER, FOLLOWER, SLOT, LIMIT = "#2a78d6", "#a3a29c", "#eb6834", "#52514e"


def style(ax, title: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=10, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8)


def main() -> None:
    run_dir, out = Path(sys.argv[1]), Path(sys.argv[2])
    title = sys.argv[3] if len(sys.argv) > 3 else run_dir.name
    cfg = load_config(default_config_path())
    half = math.radians(cfg.formation.v_half_angle_deg)
    states = [json.loads(l) for l in open(run_dir / "states.jsonl", encoding="utf-8") if l.strip()]
    t0 = min(s["t"] for s in states)
    by: dict[int, list[dict]] = {}
    for s in states:
        if "pos" in s:
            by.setdefault(s["id"], []).append(s)
    masters = [s for s in states if s.get("role") == "MASTER" and s.get("phase") == "CRUISE"]
    main_master = max(set(s["id"] for s in masters), key=lambda i: sum(1 for s in masters if s["id"] == i))

    fig = plt.figure(figsize=(13, 6.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 1.25], height_ratios=[1.3, 1.0])
    ax_tr, ax_fm, ax_rm = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])

    # (a) tracks
    for i, rows in sorted(by.items()):
        color = MASTER if i == main_master else FOLLOWER
        ax_tr.plot([r["pos"][0] for r in rows], [r["pos"][1] for r in rows], color=color,
                   linewidth=1.6 if i == main_master else 1.0, zorder=3 if i == main_master else 2)
        last = rows[-1]["pos"]
        ax_tr.annotate(str(i), (last[0], last[1]), xytext=(0, 5), textcoords="offset points", ha="center",
                       fontsize=7, color=INK2)
    cruise_t = [s["t"] for s in masters if s["id"] == main_master]
    for frac in (0.15, 0.5, 0.85):
        ts = cruise_t[0] + frac * (cruise_t[-1] - cruise_t[0])
        snap = {}
        for i, rows in by.items():
            r = min(rows, key=lambda r: abs(r["t"] - ts))
            if abs(r["t"] - ts) < 0.3 and not r.get("landed"):
                snap[i] = r["pos"]
        ax_tr.scatter([p[0] for p in snap.values()], [p[1] for p in snap.values()], s=10, color=INK, zorder=4)
        ax_tr.annotate(f"t={ts - t0:.0f} s", (max(p[0] for p in snap.values()) + 4, max(p[1] for p in snap.values())),
                       fontsize=7, color=INK2, va="top")
    gx, gy = cfg.mission.goal_enu_m
    ax_tr.scatter([gx], [gy], marker="x", s=40, color=INK, zorder=5)
    ax_tr.annotate("goal", (gx, gy), xytext=(6, 0), textcoords="offset points", fontsize=8, color=INK2, va="center")
    style(ax_tr, "Tracks (shared ENU), V snapshots in black")
    ax_tr.set_xlabel("East [m]", color=INK2, fontsize=9)
    ax_tr.set_ylabel("North [m]", color=INK2, fontsize=9)
    ax_tr.set_xlim(-90, 90)

    # (b) formation in the master's frame during cruise
    mrows = {round(s["t"], 1): s for s in masters if s["id"] == main_master}
    slots = assign_slots(next(s for s in masters if s["id"] == main_master)["members"], main_master)
    for i, rows in by.items():
        if i == main_master:
            continue
        fw, lf = [], []
        for r in rows:
            ms = mrows.get(round(r["t"], 1))
            if ms is None or r.get("phase") != "CRUISE":
                continue
            dt = r.get("pos_t", r["t"]) - ms.get("pos_t", ms["t"])
            mp = [ms["pos"][k] + ms["vel"][k] * dt for k in range(2)]
            de, dn = r["pos"][0] - mp[0], r["pos"][1] - mp[1]
            h = ms["heading"]
            fw.append(de * math.cos(h) + dn * math.sin(h))
            lf.append(-de * math.sin(h) + dn * math.cos(h))
        ax_fm.scatter(lf, fw, s=1.5, color=FOLLOWER, alpha=0.35, linewidths=0)
    for i, slot in slots.items():
        f, l = v_slot_body(slot, cfg.formation.spacing_m, half)
        ax_fm.scatter([l], [f], s=36, facecolors="none", edgecolors=SLOT, linewidths=1.3, zorder=3)
        ax_fm.annotate(str(i), (l, f), xytext=(6, -2), textcoords="offset points", fontsize=7, color=INK2)
    ax_fm.scatter([0], [0], s=36, color=MASTER, zorder=3)
    ax_fm.annotate(f"master {main_master}", (0, 0), xytext=(6, 2), textcoords="offset points", fontsize=7, color=INK2)
    style(ax_fm, "Followers vs ideal slots (rings), master frame, cruise")
    ax_fm.set_xlabel("Left of master [m]", color=INK2, fontsize=9)
    ax_fm.set_ylabel("Ahead of master [m]", color=INK2, fontsize=9)
    ax_fm.set_aspect("equal")
    ax_fm.invert_xaxis()

    # (c) RMS over time
    rows = list(csv.DictReader(open(run_dir / "formation_rms.csv", encoding="utf-8")))
    ax_rm.plot([float(r["t_s"]) for r in rows], [float(r["rms_m"]) for r in rows], color=MASTER, linewidth=1.2)
    ax_rm.axhline(2.0, color=LIMIT, linewidth=1, linestyle=(0, (4, 3)))
    ax_rm.annotate("2 m limit", (float(rows[0]["t_s"]), 2.0), xytext=(2, 3), textcoords="offset points",
                   fontsize=7, color=INK2)
    style(ax_rm, "Formation RMS error during cruise")
    ax_rm.set_xlabel("Time since first log sample [s]", color=INK2, fontsize=9)
    ax_rm.set_ylabel("RMS [m]", color=INK2, fontsize=9)
    ax_rm.set_ylim(0, max(2.5, max(float(r["rms_m"]) for r in rows) * 1.1))

    fig.suptitle(title, color=INK, fontsize=11, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main()
