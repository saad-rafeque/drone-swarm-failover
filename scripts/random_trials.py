#!/usr/bin/env python3
"""Phase 2 acceptance: randomized pure-Python runs (random kills, link drops, partitions).

Each run: N in [3, 10] drones, random link latency (0-300 ms, 20 % jitter) and loss (0-30 %)
during the fault window, 1-5 random faults at random times (on the ground, during takeoff or
in cruise):
  kill_master, kill_random, master_link_lost (all of the master's outgoing heartbeats dropped),
  link_drops (random 10-50 % of directed links), partition (random two-way split),
  low_battery_master (planned handover; at most once)
Faults with a duration last 2-15 s. At least 2 non-retired drones always stay alive.
After the last fault clears (t_clear) every link is healed and random loss stops (latency stays).
Success: from some time t_conv >= t_clear until the end of the run (t_clear + 20 s) there is
exactly one master among alive drones and every alive non-retired drone follows it with the
same term. Convergence time = t_conv - t_clear.

Usage: PYTHONPATH=src python3 scripts/random_trials.py --runs 1000 --workers 3
"""
from __future__ import annotations

import argparse
import json
import random
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.heartbeat import Role  # noqa: E402
from swarm_tools.puresim import PureSim  # noqa: E402

SETTLE_S = 20.0
KINDS = ["kill_master", "kill_random", "master_link_lost", "link_drops", "partition", "low_battery_master"]


def plan(rng: random.Random, n: int) -> list[dict]:
    events, t = [], rng.uniform(5.0, 20.0)
    for _ in range(rng.randint(1, 5)):
        kind = rng.choice(KINDS)
        dur = rng.uniform(2.0, 15.0) if kind in ("master_link_lost", "link_drops", "partition") else 0.0
        events.append({"kind": kind, "t": round(t, 2), "dur": round(dur, 2)})
        t += rng.uniform(1.0, 25.0)
    return events


def run_one(seed: int) -> dict:
    cfg0 = load_config(default_config_path())
    rng = random.Random(seed)
    n = rng.randint(3, 10)
    latency = rng.choice([0.0, 0.05, 0.15, 0.3])
    loss = rng.uniform(0.0, 0.3)
    cfg = cfg0.with_num_drones(n)
    sim = PureSim(cfg, seed=seed, latency_s=latency, jitter_s=0.2 * latency, loss=loss)
    events = plan(rng, n)
    t_clear = max(e["t"] + e["dur"] for e in events)
    applied: list[dict] = []
    battery_used = False
    pending = sorted(events, key=lambda e: e["t"])
    blocks: list[tuple[float, set]] = []  # (end time, links) for timed link faults
    max_masters = 0
    conv_since: float | None = None
    healed = False

    def live_flyers():
        return [i for i in sim.alive_ids() if sim.role(i) != Role.RETIRED]

    while sim.t < t_clear + SETTLE_S:
        while pending and sim.t >= pending[0]["t"]:
            ev = pending.pop(0)
            kind, ms = ev["kind"], sim.masters()
            flyers = live_flyers()
            rec = dict(ev)
            if kind in ("kill_master", "kill_random") and len(flyers) > 2:
                victim = ms[0] if kind == "kill_master" and ms else rng.choice(flyers)
                sim.kill(victim)
                rec["target"] = victim
            elif kind == "master_link_lost" and ms:
                links = {(ms[0], j) for j in sim.drones if j != ms[0]}
                sim.net.blocked |= links
                blocks.append((sim.t + ev["dur"], links))
                rec["target"] = ms[0]
            elif kind == "link_drops":
                all_links = [(i, j) for i in sim.drones for j in sim.drones if i != j]
                links = set(rng.sample(all_links, max(1, int(len(all_links) * rng.uniform(0.1, 0.5)))))
                sim.net.blocked |= links
                blocks.append((sim.t + ev["dur"], links))
                rec["links"] = len(links)
            elif kind == "partition":
                ids = list(sim.drones)
                rng.shuffle(ids)
                k = rng.randint(1, len(ids) - 1)
                a, b = set(ids[:k]), set(ids[k:])
                links = {(i, j) for i in a for j in b} | {(j, i) for i in a for j in b}
                sim.net.blocked |= links
                blocks.append((sim.t + ev["dur"], links))
                rec["groups"] = [sorted(a), sorted(b)]
            elif kind == "low_battery_master" and ms and not battery_used and len(flyers) > 2:
                sim.set_battery(ms[0], cfg.battery.handover_pct - 1.0)
                battery_used = True
                rec["target"] = ms[0]
            else:
                rec["skipped"] = True
            applied.append(rec)
        for end, links in list(blocks):
            if sim.t >= end:
                sim.net.blocked -= links
                blocks.remove((end, links))
        if not healed and sim.t >= t_clear:
            sim.net.heal()
            sim.net.loss = 0.0
            healed = True
        st = sim.step()
        max_masters = max(max_masters, len(st.masters))
        if sim.t >= t_clear:
            if st.converged:
                if conv_since is None:
                    conv_since = sim.t
            else:
                conv_since = None

    ms = sim.masters()
    return {
        "seed": seed, "n": n, "latency_s": latency, "loss": round(loss, 3), "events": applied,
        "t_clear": round(t_clear, 2), "success": conv_since is not None and len(ms) == 1,
        "convergence_s": None if conv_since is None else round(conv_since - t_clear, 3),
        "final_masters": ms, "max_simultaneous_masters": max_masters,
        "alive": len(sim.alive_ids()), "retired": sum(1 for i in sim.alive_ids() if sim.role(i) == Role.RETIRED),
        "min_sep_m": round(sim.min_sep_seen, 2),
        "last_event": applied[-1]["kind"] if applied else None,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=1000)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--first-seed", type=int, default=0)
    ap.add_argument("--out", default=str(ROOT / "reports" / "logs" / "phase_2"))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    seeds = range(args.first_seed, args.first_seed + args.runs)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows = list(pool.map(run_one, seeds, chunksize=10))
    wall = time.time() - t0
    with open(out / "random_trials.jsonl", "w", encoding="utf-8") as fh:
        for r in rows:
            fh.write(json.dumps(r) + "\n")
    ok = [r for r in rows if r["success"]]
    conv = sorted(r["convergence_s"] for r in ok)
    by_last: dict[str, list[float]] = {}
    for r in ok:
        by_last.setdefault(r["last_event"], []).append(r["convergence_s"])
    summary = {
        "runs": len(rows), "success": len(ok), "success_pct": round(100.0 * len(ok) / len(rows), 2),
        "failures": [r["seed"] for r in rows if not r["success"]],
        "convergence_s": {"median": statistics.median(conv), "p95": conv[int(0.95 * (len(conv) - 1))],
                          "max": conv[-1], "min": conv[0]} if conv else None,
        "convergence_by_last_event_median_s": {k: round(statistics.median(v), 3) for k, v in sorted(by_last.items())},
        "runs_with_split_brain_during_faults": sum(1 for r in rows if r["max_simultaneous_masters"] > 1),
        "n_distribution": {n: sum(1 for r in rows if r["n"] == n) for n in range(3, 11)},
        "event_counts": {k: sum(1 for r in rows for e in r["events"] if e["kind"] == k and not e.get("skipped"))
                         for k in KINDS},
        "min_sep_m_min": min(r["min_sep_m"] for r in rows),
        "wall_s": round(wall, 1), "workers": args.workers, "seeds": [args.first_seed, args.first_seed + args.runs - 1],
    }
    (out / "random_trials_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
