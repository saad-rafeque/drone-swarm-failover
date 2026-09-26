#!/usr/bin/env python3
"""Phase 0 acceptance test: one SIH drone + one MAVROS instance.

Arms, takes off to 10 m with OFFBOARD velocity setpoints, hovers 20 s, lands (AUTO.LAND).
Evidence written to reports/logs/phase_0/:
  flight_altitude.csv   altitude over time (10 Hz)
  flight_summary.json   key numbers incl. peak RAM of PX4 + MAVROS
  px4_1.log, mavros_1.log  process console output

Run: scripts/ros_env.sh python3 scripts/phase0_flight_test.py
Interfaces verified on the running system (ros2 topic/service list, mavros_msgs/*.msg) and
docs.px4.io/main/en/flight_modes/offboard (setpoints must stream >= 2 Hz before OFFBOARD).
"""
from __future__ import annotations

import argparse
import csv
import json
import shutil
import sys
import threading
import time
from pathlib import Path

import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from swarm_agent.config import default_config_path, load_config
from swarm_tools.sim_launch import REPO_ROOT, SimLauncher, kill_orphans, namespace_of

TARGET_ALT_M = 10.0
HOVER_S = 20.0
ALT_TOL_M = 0.3
CLIMB_MAX_MPS = 1.5
MIN_AVAILABLE_MB = 800  # rule 6
OUT = REPO_ROOT / "reports" / "logs" / "phase_0"


class FlightTest(Node):
    def __init__(self, ns: str, setpoint_hz: float) -> None:
        super().__init__("phase0_flight_test", namespace=ns)
        self.state = State()
        self.landed_state = ExtendedState.LANDED_STATE_UNDEFINED
        self.pos: tuple[float, float, float] | None = None
        self.rel_alt = float("nan")
        self.cmd = (0.0, 0.0, 0.0)
        self.create_subscription(State, "mavros/state", self._on_state, qos_profile_sensor_data)
        self.create_subscription(ExtendedState, "mavros/extended_state", self._on_ext, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "mavros/local_position/pose", self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Float64, "mavros/global_position/rel_alt", self._on_rel_alt, qos_profile_sensor_data)
        self.sp_pub = self.create_publisher(TwistStamped, "mavros/setpoint_velocity/cmd_vel", 10)
        self.arm_cli = self.create_client(CommandBool, "mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, "mavros/set_mode")
        self.create_timer(1.0 / setpoint_hz, self._publish_setpoint)

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_ext(self, msg: ExtendedState) -> None:
        self.landed_state = msg.landed_state

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.pos = (p.x, p.y, p.z)

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = msg.data

    def _publish_setpoint(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.cmd
        self.sp_pub.publish(msg)

    def call(self, client, request, timeout_s: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        fut = client.call_async(request)
        end = time.time() + timeout_s
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        res = self.call(self.mode_cli, req)
        return bool(res and res.mode_sent)

    def arm(self, value: bool) -> bool:
        req = CommandBool.Request()
        req.value = value
        res = self.call(self.arm_cli, req)
        return bool(res and res.success)


def wait_until(cond, timeout_s: float, what: str, tick=None) -> float:
    t0 = time.time()
    while not cond():
        if time.time() - t0 > timeout_s:
            raise TimeoutError(f"timeout after {timeout_s:.0f}s waiting for {what}")
        if tick:
            tick()
        time.sleep(0.05)
    return time.time() - t0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-subdir", default="", help="subfolder of reports/logs/phase_0 for this run")
    out = OUT / ap.parse_args().out_subdir
    cfg = load_config(default_config_path()).with_num_drones(1)
    drone_id = cfg.drone_ids[0]
    out.mkdir(parents=True, exist_ok=True)

    avail_mb = psutil.virtual_memory().available / 1e6
    print(f"available memory before launch: {avail_mb:.0f} MB")
    if avail_mb < MIN_AVAILABLE_MB:
        print("ABORT: available memory below 800 MB (rule 6)")
        return 2

    kill_orphans()
    sim = SimLauncher(cfg, {drone_id: cfg.origin_geo})
    summary: dict = {"drone_id": drone_id, "target_alt_m": TARGET_ALT_M, "hover_s": HOVER_S,
                     "available_mb_before": round(avail_mb), "px4_work_dir": str(sim.work_dir)}
    peak = {"px4": 0.0, "mavros": 0.0, "total": 0.0}
    rows: list[list] = []
    t_start = time.time()
    phase = "boot"

    rclpy.init()
    node = FlightTest(namespace_of(drone_id), cfg.setpoints.rate_hz)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)

    last_log = 0.0

    def tick() -> None:
        nonlocal last_log
        now = time.time()
        if now - last_log >= 0.1:
            last_log = now
            rss = sim.rss_mb()
            total = rss["px4"] + rss["mavros"]
            peak["px4"] = max(peak["px4"], rss["px4"])
            peak["mavros"] = max(peak["mavros"], rss["mavros"])
            peak["total"] = max(peak["total"], total)
            z = node.pos[2] if node.pos else float("nan")
            rows.append([round(now - t_start, 2), phase, node.state.mode, int(node.state.armed),
                         node.landed_state, round(z, 3), round(node.rel_alt, 3), round(node.cmd[2], 3),
                         round(total, 1)])

    try:
        sim.start([drone_id])
        spin.start()
        summary["t_connected_s"] = round(wait_until(lambda: node.state.connected, 60, "MAVROS connected", tick), 1)
        wait_until(lambda: node.pos is not None, 60, "local position", tick)
        x0, y0, _ = node.pos

        # Stream zero-velocity setpoints for 2 s (> 2 Hz proof of life) before OFFBOARD + arm.
        phase = "prearm"
        t_ready = time.time()
        wait_until(lambda: time.time() - t_ready > 2.0, 5, "setpoint pre-stream", tick)

        def offboard_and_armed() -> bool:
            if node.state.mode != "OFFBOARD":
                node.set_mode("OFFBOARD")
            elif not node.state.armed:
                node.arm(True)
            return node.state.mode == "OFFBOARD" and node.state.armed

        summary["t_to_offboard_armed_s"] = round(wait_until(offboard_and_armed, 90, "OFFBOARD + armed", tick), 1)

        def hold(target_z: float) -> None:
            x, y, z = node.pos
            vz = max(-CLIMB_MAX_MPS, min(CLIMB_MAX_MPS, 1.0 * (target_z - z)))
            node.cmd = (0.5 * (x0 - x), 0.5 * (y0 - y), vz)

        phase = "climb"
        t_climb = time.time()

        def reached() -> bool:
            hold(TARGET_ALT_M)
            tick()
            return abs(node.pos[2] - TARGET_ALT_M) < ALT_TOL_M

        wait_until(reached, 60, f"altitude {TARGET_ALT_M} m")
        summary["t_climb_s"] = round(time.time() - t_climb, 1)

        phase = "hover"
        t_hover = time.time()
        hover_z: list[float] = []

        def hovered() -> bool:
            hold(TARGET_ALT_M)
            tick()
            hover_z.append(node.pos[2])
            if node.state.mode != "OFFBOARD":
                raise RuntimeError(f"left OFFBOARD during hover (mode {node.state.mode})")
            return time.time() - t_hover >= HOVER_S

        wait_until(hovered, HOVER_S + 10, "hover")
        summary["hover_actual_s"] = round(time.time() - t_hover, 1)
        summary["hover_alt_mean_m"] = round(sum(hover_z) / len(hover_z), 3)
        summary["hover_alt_min_m"] = round(min(hover_z), 3)
        summary["hover_alt_max_m"] = round(max(hover_z), 3)

        phase = "land"
        node.cmd = (0.0, 0.0, 0.0)
        t_land = time.time()

        def landing_mode() -> bool:
            if node.state.mode != "AUTO.LAND":
                node.set_mode("AUTO.LAND")
            return node.state.mode == "AUTO.LAND"

        wait_until(landing_mode, 15, "AUTO.LAND", tick)
        wait_until(lambda: not node.state.armed, 90, "landed + disarmed", tick)
        summary["t_land_to_disarm_s"] = round(time.time() - t_land, 1)
        summary["landed_state_final"] = node.landed_state
        phase = "done"
        t_tail = time.time()
        wait_until(lambda: time.time() - t_tail > 1.0, 5, "log tail", tick)
        summary["result"] = "PASS"
    except Exception as exc:  # evidence even on failure
        summary["result"] = "FAIL"
        summary["error"] = repr(exc)
    finally:
        summary["max_alt_m"] = round(max((r[5] for r in rows if r[5] == r[5]), default=float("nan")), 3)
        summary["peak_rss_sampled_mb"] = {k: round(v, 1) for k, v in peak.items()}
        vmhwm = sim.peak_rss_mb()
        vmhwm["total"] = vmhwm["px4"] + vmhwm["mavros"]
        summary["peak_rss_vmhwm_mb"] = {k: round(v, 1) for k, v in vmhwm.items()}
        summary["duration_s"] = round(time.time() - t_start, 1)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sim.stop()
        with open(out / "flight_altitude.csv", "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "phase", "mode", "armed", "landed_state", "z_local_m", "rel_alt_m",
                        "cmd_vz_mps", "rss_px4_mavros_mb"])
            w.writerows(rows)
        (out / "flight_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        for name in (f"px4_{drone_id}.log", f"mavros_{drone_id}.log"):
            src = sim.log_dir / name
            if src.exists():
                shutil.copy(src, out / name)
    print(json.dumps(summary, indent=2))
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
