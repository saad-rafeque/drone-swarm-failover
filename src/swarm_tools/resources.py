"""1 Hz CPU / memory sampler for simulation runs (central tool)."""
from __future__ import annotations

import csv
import os
import threading
import time
from collections.abc import Callable
from pathlib import Path

import psutil

GROUPS = {
    "px4": lambda p: p.info["name"] == "px4",
    "mavros": lambda p: p.info["name"] == "mavros_node",
    "agents": lambda p: "swarm_agent.ros_node" in " ".join(p.info["cmdline"] or []),
    "tools": lambda p: any(m in " ".join(p.info["cmdline"] or []) for m in
                           ("swarm_tools.link_emulator", "swarm_tools.logger_node", "run_mission.py")),
}


class ResourceMonitor(threading.Thread):
    """Samples CPU (% of one core, summed per process group), RSS and system CPU/memory."""

    def __init__(self, label: Callable[[], str] = lambda: "", period_s: float = 1.0) -> None:
        super().__init__(daemon=True)
        self.label, self.period = label, period_s
        self.rows: list[dict] = []
        self._stop_evt = threading.Event()
        self._procs: dict[int, tuple[str, psutil.Process]] = {}

    def _refresh(self) -> None:
        for p in psutil.process_iter(["name", "cmdline"]):
            if p.pid in self._procs:
                continue
            for group, match in GROUPS.items():
                try:
                    if match(p):
                        p.cpu_percent(None)
                        self._procs[p.pid] = (group, p)
                        break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    break

    def run(self) -> None:
        psutil.cpu_percent(None)
        t0 = time.time()
        while not self._stop_evt.wait(self.period):
            self._refresh()
            cpu = dict.fromkeys(GROUPS, 0.0)
            rss = dict.fromkeys(GROUPS, 0.0)
            for pid, (group, p) in list(self._procs.items()):
                try:
                    cpu[group] += p.cpu_percent(None)
                    rss[group] += p.memory_info().rss / 1e6
                except psutil.NoSuchProcess:
                    del self._procs[pid]
            row = {"t_s": round(time.time() - t0, 1), "label": self.label(),
                   "cpu_system_pct": psutil.cpu_percent(None),
                   "mem_available_mb": round(psutil.virtual_memory().available / 1e6),
                   "load1": round(os.getloadavg()[0], 2)}
            for g in GROUPS:
                row[f"cpu_{g}_pct_core"] = round(cpu[g], 1)
                row[f"rss_{g}_mb"] = round(rss[g], 1)
            self.rows.append(row)

    def stop(self) -> None:
        self._stop_evt.set()
        self.join(timeout=3)

    def write_csv(self, path: Path) -> None:
        if not self.rows:
            return
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(self.rows[0]))
            w.writeheader()
            w.writerows(self.rows)

    def stats(self, label: str | None = None) -> dict:
        rows = [r for r in self.rows if label is None or r["label"] == label]
        out: dict = {}
        for key in ("cpu_system_pct", "cpu_px4_pct_core", "cpu_mavros_pct_core", "cpu_agents_pct_core",
                    "cpu_tools_pct_core", "rss_agents_mb", "mem_available_mb", "load1"):
            vals = [r[key] for r in rows]
            if vals:
                out[key] = {"mean": round(sum(vals) / len(vals), 1), "max": round(max(vals), 1),
                            "min": round(min(vals), 1)}
        return out
