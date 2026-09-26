#!/usr/bin/env python3
"""Phase 4 per-trial and per-fault metrics from mission run folders.

Per trial (a run_dir containing fault_events.jsonl, states.jsonl, metrics.json):
  handover_s    F1/F2: fault -> first heartbeat of the new master (its claim_t in agent_state;
                        the agent sends a heartbeat in the same control step it claims)
                F3:    battery trigger (old master's retire_t) -> successor's claim_t
                F5:    heal -> every live drone follows one single master (convergence)
                F4:    none; master_changed must be False
  recovery_s    reference (fault; heal for F5) -> formation RMS < 2 m and staying < 2 m for 5 s
  min_sep_m, goal_reached (metrics.json), retiree_home (F3: old master landed within 5 m of home)
Usage: python3 scripts/phase4_metrics.py --out table.json <trial_dir> [<trial_dir> ...]
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path

RMS_LIMIT_M = 2.0
STAY_S = 5.0


def load_states(d: Path) -> list[dict]:
    return [json.loads(l) for l in open(d / "states.jsonl", encoding="utf-8") if l.strip()]


def trial(d: Path) -> dict:
    ev = [json.loads(l) for l in open(d / "fault_events.jsonl", encoding="utf-8") if l.strip()]
    fault = next(e for e in ev if e.get("action") == "fault")
    heal = next((e for e in ev if e.get("action") == "heal"), None)
    kind, t_fault, m0 = fault["kind"], fault["t"], fault["master_before"]
    states = load_states(d)
    t0 = min(s["t"] for s in states)
    metrics = json.loads((d / "metrics.json").read_text())
    out = {"trial": d.name, "fault": kind, "t_fault_s": round(t_fault - t0, 2), "master_before": m0,
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
    ref = heal["t"] if kind == "F5" and heal is not None else t_fault
    rows = [(float(r["t_s"]) + t0, float(r["rms_m"])) for r in csv.DictReader(open(d / "formation_rms.csv", encoding="utf-8"))]
    after = [(t, v) for t, v in rows if t >= ref]
    rec = None
    for k, (t, v) in enumerate(after):
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("dirs", nargs="+")
    args = ap.parse_args()
    rows = []
    for d in args.dirs:
        try:
            rows.append(trial(Path(d)))
        except (FileNotFoundError, StopIteration, KeyError, ValueError) as exc:
            rows.append({"trial": Path(d).name, "fault": Path(d).name.split("_")[0], "error": repr(exc)})
    table = summarize([r for r in rows if "error" not in r])
    Path(args.out).write_text(json.dumps({"per_fault": table, "trials": rows}, indent=2) + "\n")
    print(json.dumps(table, indent=1))


if __name__ == "__main__":
    main()
