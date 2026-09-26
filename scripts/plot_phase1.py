#!/usr/bin/env python3
"""Phase 1 resource plot: default vs lean MAVROS plugin set -> reports/phase1_resources.png."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "reports" / "logs" / "phase_1"
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
LEAN, DEFAULT = ("Lean MAVROS (6 plugins)", "#2a78d6"), ("Default MAVROS (px4_pluginlists)", "#eb6834")


def jsonl(name: str) -> list[dict]:
    return [json.loads(l) for l in (LOGS / name).read_text().splitlines() if l.strip()]


def summary(folder: str) -> dict:
    return json.loads((LOGS / folder / "summary.json").read_text())


def style(ax, title: str, ylabel: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=10, loc="left")
    ax.set_xlabel("Drones (N)", color=INK2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=8)
    ax.set_xticks([1, 3, 5, 10])
    ax.set_xlim(0, 11)
    ax.set_ylim(bottom=0)


def series(ax, xs, ys, spec, fmt: str) -> None:
    label, color = spec
    ax.plot(xs, ys, color=color, linewidth=2, marker="o", markersize=6, label=label,
            markeredgecolor=SURFACE, markeredgewidth=1.5)
    ax.annotate(fmt.format(ys[-1]), (xs[-1], ys[-1]), xytext=(6, 0), textcoords="offset points",
                color=INK2, fontsize=8, va="center")


def main() -> None:
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2), dpi=150)
    fig.patch.set_facecolor(SURFACE)

    d, l = jsonl("idle_scaling_default.jsonl"), jsonl("idle_scaling_lean.jsonl")
    ax = axes[0]
    series(ax, [r["n"] for r in d], [r["mavros_cpu_per_instance_pct_core"] for r in d], DEFAULT, "{:.0f}%")
    series(ax, [r["n"] for r in l], [r["mavros_cpu_per_instance_pct_core"] for r in l], LEAN, "{:.0f}%")
    style(ax, "MAVROS CPU per instance, idle on ground", "% of one core")

    ns_d, ns_l = [3, 5], [3, 5, 10]
    sd = [summary(f"n{n}_default_mavros") for n in ns_d]
    sl = [summary(f"n{n}") for n in ns_l]
    ax = axes[1]
    series(ax, ns_d, [s["cpu_hover"]["cpu_system_pct"]["mean"] for s in sd], DEFAULT, "{:.0f}%")
    series(ax, ns_l, [s["cpu_hover"]["cpu_system_pct"]["mean"] for s in sl], LEAN, "{:.0f}%")
    base = 14.6  # reports/logs/phase_1/idle_baseline.txt (no sim running)
    ax.axhline(base, color=INK2, linewidth=1, linestyle=(0, (4, 3)))
    ax.text(0.3, base + 2, "laptop idle, no sim", color=INK2, fontsize=8)
    ax.set_ylim(0, 105)
    style(ax, "System CPU during 30 s hover (flight test)", "% of all 4 logical CPUs")
    ax.set_ylim(0, 105)

    ax = axes[2]
    series(ax, ns_d, [s["peak_rss_vmhwm_mb"]["per_drone"] for s in sd], DEFAULT, "{:.0f} MB")
    series(ax, ns_l, [s["peak_rss_vmhwm_mb"]["per_drone"] for s in sl], LEAN, "{:.0f} MB")
    style(ax, "Peak RAM per drone (PX4 + MAVROS, VmHWM)", "MB")

    handles, labels = axes[0].get_legend_handles_labels()
    leg = fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, fontsize=9,
                     bbox_to_anchor=(0.5, 1.0))
    for txt in leg.get_texts():
        txt.set_color(INK)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = ROOT / "reports" / "phase1_resources.png"
    fig.savefig(out, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main()
