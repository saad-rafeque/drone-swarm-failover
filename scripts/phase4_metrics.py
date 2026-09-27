#!/usr/bin/env python3
"""Phase 4 per-trial and per-fault metrics from mission run folders.

Per trial (a run_dir containing fault_events.jsonl, states.jsonl, metrics.json):
  handover_s    F1/F2: fault -> first heartbeat of the new master (its claim_t in agent_state;
                        the agent sends a heartbeat in the same control step it claims)
                F3:    battery trigger (old master's retire_t) -> successor's claim_t
                F5:    heal -> every live drone follows one single master (convergence)
                F4:    none; master_changed must be False
  recovery_s    reference (fault; battery trigger for F3; heal for F5) -> formation RMS < 2 m and staying
                < 2 m for 5 s, counted from the worst RMS within 30 s after the reference (0 if the RMS
                never reached 2 m)
  min_sep_m, goal_reached (metrics.json), retiree_home (F3: old master landed within 5 m of home)
Usage: python3 scripts/phase4_metrics.py --out table.json [--md table.md] <trial_dir> [<trial_dir> ...]
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

RMS_LIMIT_M = 2.0
STAY_S = 5.0


def load_states(d: Path) -> list[dict]:
    from metrics import open_states          # reads states.jsonl or states.jsonl.gz
    with open_states(d) as fh:
        return [json.loads(l) for l in fh if l.strip()]


def trial(d: Path) -> dict:
    ev = [json.loads(l) for l in open(d / "fault_events.jsonl", encoding="utf-8") if l.strip()]
    fault = next(e for e in ev if e.get("action") == "fault")
    heal = next((e for e in ev if e.get("action") == "heal"), None)
    # the moment the fault started (before the kill commands ran); "t" is when the event line was written after them
    kind, t_fault, m0 = fault["kind"], fault.get("t_fault", fault["t"]), fault["master_before"]
    states = load_states(d)
    t0 = min(s["t"] for s in states)
    metrics = json.loads((d / "metrics.json").read_text())
    out = {"trial": d.name, "dir": str(d), "fault": kind, "t_fault_s": round(t_fault - t0, 2), "master_before": m0,
           "target": fault.get("target"), "min_sep_m": metrics.get("min_separation_m"),
           "goal_reached": metrics.get("goal_reached_all")}
    killed = {fault["target"]} if kind in ("F1", "F4") else set()

    def claims_after(t: float) -> list[tuple[float, int]]:
        seen = {}
        for s in states:
            c = s.get("claim_t")
            if c is not None and c >= t - 1e-6 and s["id"] != m0:
                seen.setdefault(s["id"], c)
        return sorted((c, i) for i, c in seen.items())

    if kind in ("F1", "F2"):
        cl = claims_after(t_fault)
        if cl:
            out["handover_s"], out["new_master"] = round(cl[0][0] - t_fault, 3), cl[0][1]
    elif kind == "F3":
        rt = next((s["retire_t"] for s in states if s["id"] == m0 and s.get("retire_t") is not None), None)
        if rt is not None:
            cl = claims_after(rt)
            out["t_trigger_s"] = round(rt - t0, 2)
            if cl:
                out["handover_s"], out["new_master"] = round(cl[0][0] - rt, 3), cl[0][1]
        fin = metrics.get("final", {}).get(str(m0), {})
        out["retiree_home"] = bool(fin.get("retired") and fin.get("ok"))
        out["retiree_dist_home_m"] = fin.get("dist_to_target_m")
    elif kind == "F4":
        masters = {(s["id"], s.get("term")) for s in states if s.get("role") == "MASTER" and s["t"] >= t_fault}
        out["master_changed"] = masters != {(m0, next(s["term"] for s in states if s["id"] == m0 and s.get("role") == "MASTER"))}
    if kind == "F5" and heal is not None:
        t_heal = heal["t"]
        out["t_heal_s"] = round(t_heal - t0, 2)
        by: dict[int, list[dict]] = {}
        for s in states:
            if s["t"] >= t_heal:
                by.setdefault(s["id"], []).append(s)
        t = t_heal
        conv = None
        while t < t_heal + 30.0:
            cur = {}
            for i, rows in by.items():
                prev = [r for r in rows if r["t"] <= t]
                if prev and t - prev[-1]["t"] < 0.3:
                    cur[i] = prev[-1]
            ms = [i for i, s in cur.items() if s.get("role") == "MASTER"]
            if len(ms) == 1 and all(s.get("master") == ms[0] for i, s in cur.items() if s.get("role") == "FOLLOWER"):
                conv = t
                break
            t += 0.05
        if conv is not None:
            out["handover_s"] = round(conv - t_heal, 3)
    ref = t_fault
    if kind == "F5" and heal is not None:
        ref = heal["t"]
    elif kind == "F3" and out.get("t_trigger_s") is not None:
        ref = out["t_trigger_s"] + t0              # the formation changes at the battery trigger, not at the injection
    rows = [(float(r["t_s"]) + t0, float(r["rms_m"])) for r in csv.DictReader(open(d / "formation_rms.csv", encoding="utf-8"))]
    after = [(t, v) for t, v in rows if t >= ref]
    window = [(t, v) for t, v in after if t <= ref + 30.0]
    start = max(window, key=lambda x: x[1])[0] if window and max(v for _, v in window) >= RMS_LIMIT_M else ref
    rec = None
    for k, (t, v) in enumerate(after):
        if t < start:
            continue
        if v < RMS_LIMIT_M and all(w < RMS_LIMIT_M for tt, w in after[k:] if tt <= t + STAY_S) and \
                after[-1][0] >= t + STAY_S:
            rec = t
            break
    if rec is not None:
        out["recovery_s"] = round(rec - ref, 2)
    out["max_rms_after_m"] = round(max((v for _, v in after), default=float("nan")), 3)
    return out


def summarize(rows: list[dict]) -> dict:
    by: dict[str, list[dict]] = {}
    for r in rows:
        by.setdefault(r["fault"], []).append(r)
    table = {}
    for f, rs in sorted(by.items()):
        ho = [r["handover_s"] for r in rs if r.get("handover_s") is not None]
        rec = [r["recovery_s"] for r in rs if r.get("recovery_s") is not None]
        table[f] = {
            "trials": len(rs),
            "handover_median_s": round(statistics.median(ho), 3) if ho else None,
            "handover_worst_s": round(max(ho), 3) if ho else None,
            "handover_measured": len(ho),
            "recovery_median_s": round(statistics.median(rec), 2) if rec else None,
            "recovery_worst_s": round(max(rec), 2) if rec else None,
            "recovered_within_15s": sum(1 for r in rs if r.get("recovery_s") is not None and r["recovery_s"] <= 15.0),
            "min_sep_m": min(r["min_sep_m"] for r in rs if r.get("min_sep_m") is not None),
            "goal_reached": sum(1 for r in rs if r.get("goal_reached")),
        }
        if f == "F3":
            table[f]["retiree_home"] = sum(1 for r in rs if r.get("retiree_home"))
        if f == "F4":
            table[f]["master_changed"] = sum(1 for r in rs if r.get("master_changed"))
    return table


LIMITS = {"F1": "3.0 / 4.0", "F2": "3.0 / 4.0", "F3": "1.0", "F4": "no change", "F5": "3.0 (after heal)"}


def fmt(v, digits: int = 2) -> str:
    return "-" if v is None else f"{v:.{digits}f}"


def markdown(table: dict, rows: list[dict]) -> str:
    """Per-fault and per-trial tables for reports/PHASE_4.md (numbers straight from the logs)."""
    out = ["| Fault | Trials | New leader median / worst (s) | Limit (s) | Formation < 2 m median / worst (s) | "
           "Recovered within 15 s | Closest pair (m) | Goal reached | Other |", "|---|---|---|---|---|---|---|---|---|"]
    for f, t in table.items():
        other = ""
        if f == "F3":
            other = f"old leader landed at home: {t['retiree_home']}/{t['trials']}"
        if f == "F4":
            other = f"leader changed: {t['master_changed']}/{t['trials']}"
        ho = "no change" if f == "F4" else f"{fmt(t['handover_median_s'])} / {fmt(t['handover_worst_s'])}"
        out.append(f"| {f} | {t['trials']} | {ho} | {LIMITS.get(f, '')} | {fmt(t['recovery_median_s'])} / "
                   f"{fmt(t['recovery_worst_s'])} | {t['recovered_within_15s']}/{t['trials']} | {fmt(t['min_sep_m'])} | "
                   f"{t['goal_reached']}/{t['trials']} | {other} |")
    out += ["", "| Trial | Leader before | Target | Fault at (s) | New leader (s) | New leader id | Formation < 2 m (s) | "
            "Max RMS after (m) | Closest pair (m) | Goal |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(rows, key=lambda r: (r["fault"], int(r["trial"].split("_t")[-1]) if "_t" in r["trial"] else 0)):
        if "error" in r:
            out.append(f"| {r['trial']} | error: {r['error'][:60]} | | | | | | | | |")
            continue
        out.append(f"| {r['trial']} | {r['master_before']} | {r.get('target') if r.get('target') is not None else '-'} | "
                   f"{fmt(r['t_fault_s'], 1)} | {fmt(r.get('handover_s'))} | {r.get('new_master', '-')} | "
                   f"{fmt(r.get('recovery_s'))} | {fmt(r.get('max_rms_after_m'))} | {fmt(r['min_sep_m'])} | "
                   f"{'yes' if r['goal_reached'] else 'no'} |")
    return "\n".join(out) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--md", help="also write the tables as Markdown")
    ap.add_argument("dirs", nargs="+")
    args = ap.parse_args()
    rows = []
    for d in args.dirs:
        if not Path(d).is_dir():
            continue                              # e.g. the F1_t1.out console files next to the trial folders
        try:
            rows.append(trial(Path(d)))
        except (FileNotFoundError, StopIteration, KeyError, ValueError) as exc:
            rows.append({"trial": Path(d).name, "fault": Path(d).name.split("_")[0], "error": repr(exc)})
    table = summarize([r for r in rows if "error" not in r])
    Path(args.out).write_text(json.dumps({"per_fault": table, "trials": rows}, indent=2) + "\n")
    if args.md:
        Path(args.md).write_text(markdown(table, rows))
    print(json.dumps(table, indent=1))


if __name__ == "__main__":
    main()
