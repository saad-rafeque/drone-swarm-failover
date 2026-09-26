#!/usr/bin/env python3
"""Phase 1 acceptance: N SIH drones + N MAVROS instances arm, take off to 10 m, hover 30 s, land.

Also verifies each drone's spawn position in the shared ENU frame, samples CPU and memory at
1 Hz (per process group and system-wide), reads kernel peak RSS (VmHWM) per process, and
checks that no PX4 / MAVROS process died. Evidence goes to reports/logs/phase_1/n<N>[_tag]/.

Run: scripts/ros_env.sh python3 scripts/phase1_scale_test.py --n 3
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import shutil
import sys
import threading
import time

import psutil
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from swarm_agent.config import default_config_path, load_config
from swarm_agent.formation import initial_layout
from swarm_agent.geometry import EnuFrame, GeoPoint, heading_of
from swarm_tools.mavros_link import MavrosLink
from swarm_tools.sim_launch import REPO_ROOT, SimLauncher, kill_orphans, namespace_of

TARGET_ALT_M = 10.0
HOVER_S = 30.0
ALT_TOL_M = 0.3
CLIMB_MAX_MPS = 1.5
MIN_AVAILABLE_MB = 800  # rule 6


class Harness(Node):
    def __init__(self, ids: list[int], setpoint_hz: float) -> None:
        super().__init__("phase1_scale_test")
        self.links = {i: MavrosLink(self, i, namespace_of(i)) for i in ids}
        self.create_timer(1.0 / setpoint_hz, self._stream)

    def _stream(self) -> None:
        for link in self.links.values():
            link.publish_setpoint()


class ResourceMonitor(threading.Thread):
    """1 Hz CPU / memory sampler for the px4 and mavros process groups and the whole system."""

    def __init__(self, sim: SimLauncher, phase_ref: dict) -> None:
        super().__init__(daemon=True)
        self.sim, self.phase_ref = sim, phase_ref
        self.rows: list[dict] = []
        self.stop_evt = threading.Event()
        self.me = psutil.Process()

    def run(self) -> None:
        procs: dict[int, tuple[str, psutil.Process]] = {}
        psutil.cpu_percent(None)
        self.me.cpu_percent(None)
        t0 = time.time()
        while not self.stop_evt.wait(1.0):
            for dp in self.sim.procs.values():
                for key, p in (("px4", dp.px4), ("mavros", dp.mavros)):
                    if p is not None and p.pid not in procs and p.poll() is None:
                        try:
                            pp = psutil.Process(p.pid)
                            pp.cpu_percent(None)
                            procs[p.pid] = (key, pp)
                        except psutil.NoSuchProcess:
                            pass
            cpu = {"px4": 0.0, "mavros": 0.0}
            for key, pp in procs.values():
                try:
                    cpu[key] += pp.cpu_percent(None)
                except psutil.NoSuchProcess:
                    pass
            rss = self.sim.rss_mb()
            vm = psutil.virtual_memory()
            self.rows.append({
                "t_s": round(time.time() - t0, 1), "phase": self.phase_ref["phase"],
                "cpu_system_pct": psutil.cpu_percent(None),          # % of all 4 logical CPUs
                "cpu_px4_pct_core": round(cpu["px4"], 1),            # % of one core (sum)
                "cpu_mavros_pct_core": round(cpu["mavros"], 1),
                "cpu_harness_pct_core": self.me.cpu_percent(None),
                "rss_px4_mb": round(rss["px4"], 1), "rss_mavros_mb": round(rss["mavros"], 1),
                "mem_available_mb": round(vm.available / 1e6), "load1": round(os.getloadavg()[0], 2),
            })


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--tag", default="", help="suffix for the output folder, e.g. crosscheck")
    args = ap.parse_args()
    cfg = load_config(default_config_path()).with_num_drones(args.n)
    ids = cfg.drone_ids
    out = REPO_ROOT / "reports" / "logs" / "phase_1" / (f"n{args.n}" + (f"_{args.tag}" if args.tag else ""))
    out.mkdir(parents=True, exist_ok=True)

    avail_mb = psutil.virtual_memory().available / 1e6
    print(f"N={args.n}: available memory before launch {avail_mb:.0f} MB", flush=True)
    if avail_mb < MIN_AVAILABLE_MB:
        print("ABORT: available memory below 800 MB (rule 6) - reduce drone count")
        return 2

    frame = EnuFrame(cfg.origin_geo)
    heading = heading_of(*cfg.mission.goal_enu_m)
    layout = initial_layout(ids, heading, cfg.formation.spacing_m, math.radians(cfg.formation.v_half_angle_deg))
    homes = {i: frame.to_geodetic((e, n, 0.0)) for i, (e, n) in layout.items()}
    homes = {i: GeoPoint(g.lat_deg, g.lon_deg, cfg.origin.alt_m) for i, g in homes.items()}

    kill_orphans()
    sim = SimLauncher(cfg, homes)
    phase = {"phase": "boot"}
    summary: dict = {"n": args.n, "ids": ids, "available_mb_before": round(avail_mb),
                     "px4_work_dir": str(sim.work_dir), "criteria": {}}
    alt_rows: list[list] = []
    monitor = ResourceMonitor(sim, phase)

    rclpy.init()
    node = Harness(ids, cfg.setpoints.rate_hz)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    links = node.links
    t_start = time.time()
    last_log = 0.0

    def log() -> None:
        nonlocal last_log
        if time.time() - last_log < 0.2:
            return
        last_log = time.time()
        for i, lk in links.items():
            z = lk.local[2] if lk.local else float("nan")
            alt_rows.append([round(last_log - t_start, 2), phase["phase"], i, lk.state.mode,
                             int(lk.state.armed), lk.landed_state, round(z, 3)])

    def wait(cond, timeout_s: float, what: str) -> float:
        t0 = time.time()
        while not cond():
            if time.time() - t0 > timeout_s:
                raise TimeoutError(f"timeout after {timeout_s:.0f}s waiting for {what}")
            log()
            time.sleep(0.05)
        return time.time() - t0

    try:
        sim.start(ids, stagger_s=1.0)
        monitor.start()
        spin.start()
        summary["t_all_connected_s"] = round(wait(
            lambda: all(lk.state.connected and lk.local and lk.fix for lk in links.values()),
            120, "all MAVROS connected with position"), 1)

        # Spawn verification in the shared ENU frame (horizontal; altitude is local z, see report).
        time.sleep(1.0)
        spawn = {}
        for i, lk in links.items():
            e, n, _ = frame.to_enu(GeoPoint(lk.fix.latitude, lk.fix.longitude, cfg.origin.alt_m))
            ie, inn = layout[i]
            spawn[i] = {"intended_en": [round(ie, 2), round(inn, 2)], "measured_en": [round(e, 2), round(n, 2)],
                        "error_m": round(math.hypot(e - ie, n - inn), 2)}
        pair_d = [math.dist(spawn[a]["measured_en"], spawn[b]["measured_en"])
                  for a, b in itertools.combinations(ids, 2)]
        summary["spawn"] = spawn
        summary["spawn_max_error_m"] = max(s["error_m"] for s in spawn.values())
        summary["spawn_min_pair_distance_m"] = round(min(pair_d), 2) if pair_d else None

        phase["phase"] = "prestream"
        time.sleep(2.0)  # > 2 Hz setpoint stream before OFFBOARD (docs.px4.io offboard)
        starts = {i: lk.local[:2] for i, lk in links.items()}
        reached: set[int] = set()
        armed_at: dict[int, float] = {}

        def control() -> None:
            for i, lk in links.items():
                if lk.offboard_armed:
                    armed_at.setdefault(i, time.time() - t_start)
                    x, y, z = lk.local
                    x0, y0 = starts[i]
                    vz = max(-CLIMB_MAX_MPS, min(CLIMB_MAX_MPS, 1.0 * (TARGET_ALT_M - z)))
                    lk.cmd = (0.5 * (x0 - x), 0.5 * (y0 - y), vz)
                    if abs(z - TARGET_ALT_M) < ALT_TOL_M:
                        reached.add(i)
                elif lk.state.mode != "OFFBOARD":
                    lk.request_mode("OFFBOARD")
                else:
                    lk.request_arm()

        phase["phase"] = "arm_climb"
        t_arm = time.time()

        def all_reached() -> bool:
            control()
            return len(reached) == len(ids)

        wait(all_reached, 120, "all drones at 10 m")
        summary["t_arm_to_all_at_alt_s"] = round(time.time() - t_arm, 1)
        summary["armed_at_s"] = {i: round(t, 1) for i, t in sorted(armed_at.items())}

        phase["phase"] = "hover"
        t_hover = time.time()
        hover_z = {i: [] for i in ids}
        hover_min_pair = float("inf")

        def hovered() -> bool:
            nonlocal hover_min_pair
            control()
            for i, lk in links.items():
                hover_z[i].append(lk.local[2])
                if not lk.offboard_armed:
                    raise RuntimeError(f"drone {i} left OFFBOARD/armed during hover ({lk.state.mode})")
            pos = {i: frame.to_enu(GeoPoint(lk.fix.latitude, lk.fix.longitude, cfg.origin.alt_m))[:2]
                   + (lk.local[2],) for i, lk in links.items()}
            for a, b in itertools.combinations(ids, 2):
                hover_min_pair = min(hover_min_pair, math.dist(pos[a], pos[b]))
            return time.time() - t_hover >= HOVER_S

        wait(hovered, HOVER_S + 15, "hover")
        summary["hover_actual_s"] = round(time.time() - t_hover, 1)
        summary["hover_alt_range_m"] = {i: [round(min(z), 3), round(max(z), 3)] for i, z in hover_z.items()}
        summary["hover_min_pair_distance_m"] = round(hover_min_pair, 2) if len(ids) > 1 else None

        phase["phase"] = "land"
        t_land = time.time()
        for lk in links.values():
            lk.cmd = (0.0, 0.0, 0.0)

        def all_disarmed() -> bool:
            for lk in links.values():
                if lk.state.armed:
                    lk.request_mode("AUTO.LAND")
            return all(not lk.state.armed for lk in links.values())

        wait(all_disarmed, 120, "all drones landed and disarmed")
        summary["t_land_to_all_disarmed_s"] = round(time.time() - t_land, 1)
        summary["landed_state_final"] = {i: lk.landed_state for i, lk in links.items()}
        phase["phase"] = "done"
        time.sleep(1.0)
        log()
    except Exception as exc:
        summary["error"] = repr(exc)
    finally:
        dead = [f"{key}_{i}" for i, dp in sim.procs.items()
                for key, p in (("px4", dp.px4), ("mavros", dp.mavros)) if p is None or p.poll() is not None]
        vmhwm = sim.peak_rss_mb()
        monitor.stop_evt.set()
        monitor.join(timeout=3)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sim.stop()

        rows = monitor.rows
        hover_rows = [r for r in rows if r["phase"] == "hover"]
        summary["processes_dead_before_stop"] = dead
        summary["peak_rss_vmhwm_mb"] = {"px4": round(vmhwm["px4"], 1), "mavros": round(vmhwm["mavros"], 1),
                                        "total": round(vmhwm["px4"] + vmhwm["mavros"], 1),
                                        "per_drone": round((vmhwm["px4"] + vmhwm["mavros"]) / args.n, 1)}
        summary["min_available_mb"] = min((r["mem_available_mb"] for r in rows), default=None)

        def stats(key: str, rs: list[dict]) -> dict:
            vals = [r[key] for r in rs]
            return {"mean": round(sum(vals) / len(vals), 1), "max": round(max(vals), 1)} if vals else {}

        summary["cpu_hover"] = {k: stats(k, hover_rows) for k in
                                ("cpu_system_pct", "cpu_px4_pct_core", "cpu_mavros_pct_core",
                                 "cpu_harness_pct_core", "load1")}
        summary["cpu_whole_run"] = {k: stats(k, rows) for k in ("cpu_system_pct", "load1")}
        crit = summary["criteria"]
        crit["all_armed_took_off_to_10m"] = "t_arm_to_all_at_alt_s" in summary
        crit["hover_30s"] = summary.get("hover_actual_s", 0) >= HOVER_S
        crit["all_landed_disarmed"] = "t_land_to_all_disarmed_s" in summary and all(
            v == 1 for v in summary.get("landed_state_final", {}).values())
        crit["no_process_crash"] = not dead
        crit["distinct_spawn_positions"] = (summary.get("spawn_min_pair_distance_m") or 99) >= cfg.safety.min_separation_m
        summary["result"] = "PASS" if all(crit.values()) and "error" not in summary else "FAIL"

        with open(out / "resources.csv", "w", newline="", encoding="utf-8") as fh:
            if rows:
                w = csv.DictWriter(fh, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
        with open(out / "altitude.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "phase", "drone_id", "mode", "armed", "landed_state", "z_local_m"])
            w.writerows(alt_rows)
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        for i in ids:
            for name in (f"px4_{i}.log", f"mavros_{i}.log"):
                if (sim.log_dir / name).exists():
                    shutil.copy(sim.log_dir / name, out / name)
    print(json.dumps({k: summary[k] for k in summary if k not in ("spawn", "hover_alt_range_m", "armed_at_s")},
                     indent=1), flush=True)
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
