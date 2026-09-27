#!/usr/bin/env python3
"""Run one full swarm mission in simulation (Phases 3-6).

 1. check free memory (rule 6), kill orphan px4/mavros processes
 2. start N PX4 SIH + N MAVROS (homes = the V formation, src/swarm_tools/sim_launch.py)
 3. wait until every MAVROS is connected and has a position
 4. `ros2 launch launch/swarm.launch.py` (link emulator + logger + one agent process per drone)
 5. watch /uav*/agent_state until every live drone has landed after the LAND phase (or timeout),
    injecting a fault on the way if requested (Phase 4)
 6. stop everything, copy process logs, compute metrics (scripts/metrics.py) -> run_dir/metrics.json

Usage: scripts/ros_env.sh python3 scripts/run_mission.py --n 10 --run-dir reports/logs/phase_3/run1
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_agent.formation import initial_layout  # noqa: E402
from swarm_agent.geometry import EnuFrame, GeoPoint, heading_of  # noqa: E402
from swarm_tools.mavros_link import MavrosLink  # noqa: E402
from swarm_tools.resources import ResourceMonitor, power_state  # noqa: E402
from swarm_tools.sim_launch import SimLauncher, kill_orphans, namespace_of  # noqa: E402

MIN_AVAILABLE_MB = 800


class Watcher(Node):
    """Tracks every agent's latest agent_state; can send link-emulator commands."""

    def __init__(self, ids: list[int]) -> None:
        super().__init__("mission_watcher")
        self.latest: dict[int, dict] = {}
        self.first_seen: dict[int, float] = {}
        self.control = self.create_publisher(String, "/link_emulator/control", 10)
        for i in ids:
            self.create_subscription(String, f"/uav{i}/agent_state", self._on_state, 20)

    def _on_state(self, msg: String) -> None:
        d = json.loads(msg.data)
        d["_rx"] = time.time()
        self.latest[d["id"]] = d
        self.first_seen.setdefault(d["id"], d["_rx"])

    def link_cmd(self, **cmd) -> None:
        self.control.publish(String(data=json.dumps(cmd)))


def agent_pid(drone_id: int) -> int | None:
    needle = ["swarm_agent.ros_node", "--id", str(drone_id)]
    for p in psutil.process_iter(["cmdline"]):
        cl = p.info["cmdline"] or []
        if all(tok in cl for tok in needle) and cl[cl.index("--id") + 1] == str(drone_id):
            return p.pid
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--timeout", type=float, default=600.0, help="mission timeout after launch [s]")
    ap.add_argument("--loss-pct", type=float, default=None)
    ap.add_argument("--latency-ms", type=float, default=None)
    ap.add_argument("--jitter-ms", type=float, default=None)
    ap.add_argument("--fault", default="", help="Phase 4 fault spec (see scripts/faults.py)")
    ap.add_argument("--fault-seed", type=int, default=0)
    ap.add_argument("--allow-battery", action="store_true",
                    help="run even when the laptop is not on its charger (results then are not comparable)")
    args = ap.parse_args()

    cfg = load_config(default_config_path()).with_num_drones(args.n)
    ids = cfg.drone_ids
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    for stale in ("states.jsonl", "link_events.jsonl", "fault_events.jsonl"):
        (run_dir / stale).unlink(missing_ok=True)  # inside the repo's reports/ only
    summary: dict = {"n": args.n, "ids": ids, "run_dir": str(run_dir), "fault": args.fault,
                     "loss_pct": args.loss_pct, "latency_ms": args.latency_ms, "jitter_ms": args.jitter_ms}

    avail = psutil.virtual_memory().available / 1e6
    summary["available_mb_before"] = round(avail)
    if avail < MIN_AVAILABLE_MB:
        print("ABORT: available memory below 800 MB (rule 6)")
        return 2
    summary["power_before"] = power_state()
    if summary["power_before"]["ac_online"] is False and not args.allow_battery:
        print("ABORT: the laptop is on battery; its power-saving profile saturates the CPU with 10 drones "
              "(use --allow-battery to run anyway)")
        return 3
    kill_orphans()

    frame = EnuFrame(cfg.origin_geo)
    layout = initial_layout(ids, heading_of(*cfg.mission.goal_enu_m), cfg.formation.spacing_m,
                            math.radians(cfg.formation.v_half_angle_deg))
    homes = {}
    for i, (e, n) in layout.items():
        g = frame.to_geodetic((e, n, 0.0))
        homes[i] = GeoPoint(g.lat_deg, g.lon_deg, cfg.origin.alt_m)
    sim = SimLauncher(cfg, homes, log_dir=run_dir / "proc_logs")
    label = {"v": "boot"}
    monitor = ResourceMonitor(lambda: label["v"])

    rclpy.init()
    node = Watcher(ids)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    # MAVROS subscriptions only until the swarm is up. Spun from this thread on its own executor and
    # destroyed only after that executor stops: destroying a node that another thread is spinning
    # killed the watcher thread in two Phase 3 runs (InvalidHandle), so they timed out.
    ready_node = Node("mission_readiness")
    links = {i: MavrosLink(ready_node, i, namespace_of(i)) for i in ids}
    ready_ex = SingleThreadedExecutor()
    ready_ex.add_node(ready_node)
    launch: subprocess.Popen | None = None
    fault = None
    t0 = time.time()
    try:
        sim.start(ids, stagger_s=1.0)
        monitor.start()
        deadline = time.time() + 120
        while not all(l.state.connected and l.fix is not None and l.local is not None for l in links.values()):
            if time.time() > deadline:
                raise TimeoutError("MAVROS instances not all connected with position after 120 s")
            ready_ex.spin_once(timeout_sec=0.2)
        summary["t_sim_ready_s"] = round(time.time() - t0, 1)
        ready_ex.shutdown()
        ready_node.destroy_node()

        cmd = ["ros2", "launch", str(ROOT / "launch" / "swarm.launch.py"), f"num_drones:={args.n}",
               f"run_dir:={run_dir}"]
        for opt in ("loss_pct", "latency_ms", "jitter_ms"):
            v = getattr(args, opt)
            if v is not None:
                cmd.append(f"{opt}:={v}")
        launch_log = open(run_dir / "proc_logs" / "ros2_launch.log", "w", encoding="utf-8")
        launch = subprocess.Popen(cmd, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        t_launch = time.time()
        label["v"] = "mission"

        if args.fault:
            from faults import make_fault  # Phase 4
            fault = make_fault(args.fault, args.fault_seed, cfg, sim, node, run_dir)

        killed: set[int] = set()
        result = "TIMEOUT"
        while time.time() - t_launch < args.timeout:
            time.sleep(0.2)
            if node.latest and time.time() - max(s["_rx"] for s in node.latest.values()) > 10.0:
                result = "WATCHER_STALLED"   # no agent_state for 10 s: tooling problem, not a flight result
                break
            if launch.poll() is not None:
                result = "LAUNCH_EXITED"
                break
            if fault is not None:
                killed |= fault.poll(time.time())
            st = node.latest
            live = [i for i in ids if i not in killed]
            if any(s.get("phase") == "TAKEOFF" for s in st.values()):
                label["v"] = "flight"
            done = st and all(i in st for i in live) and all(
                st[i].get("landed") and not st[i].get("armed") and (
                    st[i].get("phase") in ("LAND", "LANDED") or st[i].get("role") == "RETIRED")
                for i in live)
            if done and (fault is None or fault.finished):
                result = "COMPLETED"
                break
        summary["result"] = result
        summary["mission_wall_s"] = round(time.time() - t_launch, 1)
        summary["killed"] = sorted(killed)
        time.sleep(1.0)
    except Exception as exc:
        summary["result"] = "ERROR"
        summary["error"] = repr(exc)
    finally:
        label["v"] = "shutdown"
        summary["peak_rss_vmhwm_mb"] = {k: round(v, 1) for k, v in sim.peak_rss_mb().items()}
        summary["power_after"] = power_state()
        if launch is not None and launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
        monitor.stop()
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sim.stop()
        monitor.write_csv(run_dir / "resources.csv")
        summary["resources_flight"] = monitor.stats("flight")
        summary["resources_mission"] = monitor.stats("mission")
        if fault is not None:
            summary["fault_detail"] = fault.detail
        (run_dir / "run_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    rc = subprocess.run([sys.executable, str(ROOT / "scripts" / "metrics.py"), str(run_dir)],
                        capture_output=True, text=True)
    print(rc.stdout[-3000:], rc.stderr[-2000:])
    print(json.dumps({k: summary[k] for k in ("result", "mission_wall_s", "t_sim_ready_s") if k in summary}))
    return 0 if summary.get("result") == "COMPLETED" else 1


if __name__ == "__main__":
    sys.exit(main())
