#!/usr/bin/env python3
"""Metrics for one mission run (Phases 3-6): <run_dir>/states.jsonl -> <run_dir>/metrics.json.

Every drone's agent_state samples (10 Hz) are aligned on a 0.1 s grid; each drone's position at a
grid time is its latest sample (<= 0.3 s old) extrapolated with its velocity. Positions are in the
shared ENU frame (east/north from the global fix, height from the local pose).
  time_to_goal_s      master's first CRUISE -> first HOLD (goal radius reached)
  formation RMS       grid times with exactly one master, in CRUISE: RMS over the master's member
                      followers of the 3-D distance to their slot (formation.formation_errors)
  min separation      smallest 3-D distance between any two airborne drones on the grid
  goal                every drone still reporting at the end, not retired: landed within 60 m of
                      the goal (its own slot there); retired drones: landed within 5 m of home
Usage: python3 scripts/metrics.py <run_dir>
"""
from __future__ import annotations

import bisect
import itertools
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.formation import formation_errors, rms  # noqa: E402

GRID_S = 0.1
MAX_AGE_S = 0.3
GOAL_SLOT_TOL_M = 60.0
HOME_TOL_M = 5.0


def open_states(run_dir: Path):
    """states.jsonl, or states.jsonl.gz once the batch has compressed it (Phase 4 onwards)."""
    plain = run_dir / "states.jsonl"
    if plain.exists():
        return open(plain, encoding="utf-8")
    import gzip
    return gzip.open(run_dir / "states.jsonl.gz", "rt", encoding="utf-8")


def load(run_dir: Path) -> dict[int, list[dict]]:
    by_id: dict[int, list[dict]] = {}
    with open_states(run_dir) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            by_id.setdefault(d["id"], []).append(d)
    for rows in by_id.values():
        rows.sort(key=lambda d: d["t"])
    return by_id


def sample_at(rows: list[dict], times: list[float], t: float) -> dict | None:
    k = bisect.bisect_right(times, t) - 1
    if k < 0 or t - times[k] > MAX_AGE_S:
        return None
    return rows[k]


def position_at(s: dict, t: float) -> tuple[float, float, float] | None:
    if "pos" not in s:
        return None
    dt = t - s.get("pos_t", s["t"])
    p, v = s["pos"], s.get("vel", [0.0, 0.0, 0.0])
    return (p[0] + v[0] * dt, p[1] + v[1] * dt, p[2] + v[2] * dt)


def pct(vals: list[float], q: float) -> float:
    s = sorted(vals)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


def compute(run_dir: Path) -> dict:
    cfg = load_config(default_config_path())
    half = math.radians(cfg.formation.v_half_angle_deg)
    by_id = load(run_dir)
    times = {i: [d["t"] for d in rows] for i, rows in by_id.items()}
    t0 = min(r[0]["t"] for r in by_id.values())
    t1 = max(r[-1]["t"] for r in by_id.values())

    first = {"TAKEOFF": None, "CRUISE": None, "HOLD": None, "LAND": None}
    rms_series: list[tuple[float, float]] = []
    sep_series: list[tuple[float, float, tuple[int, int]]] = []
    master_track: list[tuple[float, int, int]] = []   # (t, master, term) on change
    n_masters_max = 0
    t = t0
    while t <= t1:
        cur = {i: sample_at(rows, times[i], t) for i, rows in by_id.items()}
        cur = {i: s for i, s in cur.items() if s is not None and s.get("fcu_ok", True)}
        masters = [i for i, s in cur.items() if s.get("role") == "MASTER"]
        n_masters_max = max(n_masters_max, len(masters))
        if len(masters) == 1:
            m = masters[0]
            ms = cur[m]
            ph = ms.get("phase")
            if ph in first and first[ph] is None:
                first[ph] = t
            if not master_track or master_track[-1][1:] != (m, ms.get("term")):
                master_track.append((round(t - t0, 2), m, ms.get("term")))
            if ph == "CRUISE" and "members" in ms:
                pos = {i: position_at(s, t) for i, s in cur.items() if i in ms["members"]}
                pos = {i: p for i, p in pos.items() if p is not None}
                if m in pos:
                    errs = formation_errors(pos, m, ms["heading"], ms["members"], cfg.formation.spacing_m, half,
                                            cfg.formation.shape)
                    if errs:
                        rms_series.append((t, rms(errs.values())))
        air = {i: position_at(s, t) for i, s in cur.items() if not s.get("landed", True)}
        air = {i: p for i, p in air.items() if p is not None}
        if len(air) >= 2:
            d, pair = min(((math.dist(pa, pb), (a, b)) for (a, pa), (b, pb) in itertools.combinations(air.items(), 2)))
            sep_series.append((t, d, pair))
        t += GRID_S

    out: dict = {"run_dir": str(run_dir), "drones": sorted(by_id), "samples": sum(len(r) for r in by_id.values()),
                 "duration_s": round(t1 - t0, 1),
                 "phase_start_s": {k: (None if v is None else round(v - t0, 2)) for k, v in first.items()},
                 "master_changes": master_track, "max_simultaneous_masters": n_masters_max}
    if first["CRUISE"] is not None and first["HOLD"] is not None:
        out["time_to_goal_s"] = round(first["HOLD"] - first["CRUISE"], 2)
    if rms_series:
        vals = [v for _, v in rms_series]
        out["formation_rms_cruise_m"] = {"mean": round(sum(vals) / len(vals), 3), "p95": round(pct(vals, 0.95), 3),
                                         "max": round(max(vals), 3), "samples": len(vals)}
        worst_t = max(rms_series, key=lambda x: x[1])[0]
        out["formation_rms_cruise_m"]["t_of_max_s"] = round(worst_t - t0, 2)
        out["_rms_series"] = [(round(tt - t0, 2), round(v, 3)) for tt, v in rms_series]
    if sep_series:
        tmin, dmin, pair = min(sep_series, key=lambda x: x[1])
        out["min_separation_m"] = round(dmin, 3)
        out["min_separation_pair"] = pair
        out["min_separation_t_s"] = round(tmin - t0, 2)
    gx, gy = cfg.mission.goal_enu_m
    final: dict = {}
    for i, rows in by_id.items():
        last, home = rows[-1], next((r["pos"] for r in rows if "pos" in r), None)
        if "pos" not in last:
            continue
        retired = last.get("role") == "RETIRED"
        ref = home if retired else (gx, gy)
        dist = math.hypot(last["pos"][0] - ref[0], last["pos"][1] - ref[1])
        ok = bool(last.get("landed")) and dist <= (HOME_TOL_M if retired else GOAL_SLOT_TOL_M)
        final[i] = {"landed": last.get("landed"), "retired": retired, "dist_to_target_m": round(dist, 1),
                    "ok": ok, "last_t_s": round(last["t"] - t0, 1), "reporting_at_end": t1 - last["t"] < 2.0}
    out["final"] = final
    live = [i for i, f in final.items() if f["reporting_at_end"]]
    out["goal_reached_all"] = bool(live) and all(final[i]["ok"] for i in live)
    return out


def main() -> None:
    run_dir = Path(sys.argv[1]).resolve()
    m = compute(run_dir)
    series = m.pop("_rms_series", [])
    (run_dir / "metrics.json").write_text(json.dumps(m, indent=2) + "\n")
    with open(run_dir / "formation_rms.csv", "w", encoding="utf-8") as fh:
        fh.write("t_s,rms_m\n")
        fh.writelines(f"{t},{v}\n" for t, v in series)
    print(json.dumps({k: m.get(k) for k in ("time_to_goal_s", "formation_rms_cruise_m", "min_separation_m",
                                            "min_separation_pair", "goal_reached_all", "max_simultaneous_masters",
                                            "master_changes")}, indent=1))


if __name__ == "__main__":
    main()
