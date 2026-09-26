#!/usr/bin/env python3
"""Plot Phase 0 altitude-over-time evidence (run 1 + cross-check run) -> reports/phase0_altitude.png."""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "reports" / "logs" / "phase_0"
RUNS = [("run2_crosscheck", "Cross-check run", "#2a78d6"), ("run1", "Run 1", "#eb6834")]
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"


def load(run: str) -> tuple[list[float], list[float], dict[str, float]]:
    rows = list(csv.DictReader(open(LOGS / run / "flight_altitude.csv", encoding="utf-8")))
    t_arm = next(float(r["t_s"]) for r in rows if r["armed"] == "1")
    marks = {
        "hover": next(float(r["t_s"]) for r in rows if r["phase"] == "hover") - t_arm,
        "land": next(float(r["t_s"]) for r in rows if r["phase"] == "land") - t_arm,
    }
    pts = [(float(r["t_s"]) - t_arm, float(r["z_local_m"])) for r in rows if r["z_local_m"] != "nan"]
    return [p[0] for p in pts], [p[1] for p in pts], marks


def main() -> None:
    fig, ax = plt.subplots(figsize=(9, 4.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    marks = {}
    for run, label, color in RUNS:
        t, z, m = load(run)
        marks = marks or m
        ax.plot(t, z, color=color, linewidth=2, label=label, solid_capstyle="round")
        i = min(range(len(t)), key=lambda k: abs(t[k] - 30.0))
        ax.annotate(label, (t[i], z[i]), xytext=(0, 12 if run == "run1" else -18), textcoords="offset points",
                    color=INK2, fontsize=9, ha="center")
    ax.axhline(10.0, color=INK2, linewidth=1, linestyle=(0, (4, 3)))
    ax.text(-4.5, 10.35, "target 10 m", color=INK2, fontsize=9)
    for key, text in (("hover", "hover 20 s starts"), ("land", "AUTO.LAND")):
        ax.axvline(marks[key], color=GRID, linewidth=1.2, zorder=0)
        ax.text(marks[key] + 0.6, 1.0, text, color=INK2, fontsize=9, rotation=90, va="bottom")
    ax.set_xlabel("Time since arming [s]", color=INK2)
    ax.set_ylabel("Altitude, local z [m]", color=INK2)
    ax.set_title("Phase 0 — one SIH drone via MAVROS: take off to 10 m, hover 20 s, land", color=INK,
                 fontsize=11, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2)
    ax.set_ylim(-1, 12)
    ax.set_xlim(-5, None)
    leg = ax.legend(loc="upper right", frameon=False, fontsize=9)
    for txt in leg.get_texts():
        txt.set_color(INK)
    fig.tight_layout()
    out = ROOT / "reports" / "phase0_altitude.png"
    fig.savefig(out, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main()
