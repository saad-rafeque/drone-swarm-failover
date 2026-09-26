"""Flat CSV form of agent_state records (docs/SPECIFICATION.md Phase 3: logger CSV with positions, roles, terms,
timestamps). The logger writes both states.jsonl (complete) and states.csv (these columns)."""
from __future__ import annotations

import csv
import json
from pathlib import Path

COLUMNS = ["t", "id", "east_m", "north_m", "up_m", "v_east", "v_north", "v_up", "role", "term", "master",
           "phase", "reason", "px4_mode", "armed", "landed", "battery_pct", "orphan", "transit", "retire",
           "cmd_east", "cmd_north", "cmd_up", "fcu_ok"]


def flatten(d: dict) -> dict:
    pos, vel, cmd = d.get("pos") or [None] * 3, d.get("vel") or [None] * 3, d.get("cmd") or [None] * 3
    return {
        "t": d.get("t"), "id": d.get("id"), "east_m": pos[0], "north_m": pos[1], "up_m": pos[2],
        "v_east": vel[0], "v_north": vel[1], "v_up": vel[2], "role": d.get("role"), "term": d.get("term"),
        "master": d.get("master"), "phase": d.get("phase"), "reason": d.get("reason"), "px4_mode": d.get("mode"),
        "armed": d.get("armed"), "landed": d.get("landed"), "battery_pct": d.get("battery"),
        "orphan": d.get("orphan"), "transit": d.get("transit"), "retire": d.get("retire"),
        "cmd_east": cmd[0], "cmd_north": cmd[1], "cmd_up": cmd[2], "fcu_ok": d.get("fcu_ok"),
    }


def jsonl_to_csv(src: Path, dst: Path) -> int:
    n = 0
    with open(src, encoding="utf-8") as fin, open(dst, "w", newline="", encoding="utf-8") as fout:
        w = csv.DictWriter(fout, fieldnames=COLUMNS)
        w.writeheader()
        for line in fin:
            if line.strip():
                w.writerow(flatten(json.loads(line)))
                n += 1
    return n
