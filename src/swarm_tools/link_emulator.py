"""Heartbeat link emulator (central tool, docs/SPECIFICATION.md §6): the simulated radio between drones.

Forwards every /uav<i>/hb_out message to every other /uav<j>/hb_in, with per-message random loss,
latency (+ uniform jitter), and blockable directed links. Fault injection and radio-condition
changes arrive as JSON commands on /link_emulator/control (std_msgs/String):
  {"cmd": "set", "loss_pct": 10, "latency_ms": 150, "jitter_ms": 30}
  {"cmd": "block_tx", "src": 1}                       # F2: all of drone 1's heartbeats dropped
  {"cmd": "partition", "groups": [[1,2,3],[4,5,6]]}   # F5
  {"cmd": "block", "links": [[1,2],[2,1]]}, {"cmd": "unblock", "links": [[1,2]]}
  {"cmd": "heal"}                                     # clear every block (loss/latency unchanged)
Every command and a per-link counter summary are appended to <run_dir>/link_events.jsonl.

Run: scripts/ros_env.sh python3 -m swarm_tools.link_emulator --num-drones 10 --run-dir <dir>
"""
from __future__ import annotations

import argparse
import heapq
import json
import random
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8MultiArray

from swarm_agent.config import default_config_path, load_config

DELIVERY_TICK_S = 0.01   # delay resolution when latency/jitter are emulated


class LinkEmulator(Node):
    def __init__(self, ids: list[int], loss_pct: float, latency_ms: float, jitter_ms: float, seed: int,
                 run_dir: Path) -> None:
        super().__init__("link_emulator")
        self.ids = ids
        self.loss = loss_pct / 100.0
        self.latency = latency_ms / 1000.0
        self.jitter = jitter_ms / 1000.0
        self.rng = random.Random(seed)
        self.blocked: set[tuple[int, int]] = set()
        self.forwarded = {(i, j): 0 for i in ids for j in ids if i != j}
        self.dropped = dict.fromkeys(self.forwarded, 0)
        self._queue: list[tuple[float, int, int, bytes]] = []
        self._seq = 0
        self.events_path = run_dir / "link_events.jsonl"
        run_dir.mkdir(parents=True, exist_ok=True)
        q = qos_profile_sensor_data
        self.pubs = {j: self.create_publisher(UInt8MultiArray, f"/uav{j}/hb_in", q) for j in ids}
        for i in ids:
            self.create_subscription(UInt8MultiArray, f"/uav{i}/hb_out", lambda m, i=i: self._on_hb(i, m), q)
        self.create_subscription(String, "/link_emulator/control", self._on_control, 10)
        self._timer = None
        self._ensure_timer()
        self._event({"cmd": "start", "loss_pct": loss_pct, "latency_ms": latency_ms, "jitter_ms": jitter_ms,
                     "seed": seed, "ids": ids})

    def _event(self, d: dict) -> None:
        d = {"t": round(time.time(), 3), **d}
        with open(self.events_path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(d) + "\n")

    def _ensure_timer(self) -> None:
        """The delivery timer only runs while delays are emulated (saves CPU at zero latency)."""
        if self._timer is None and (self.latency > 0.0 or self.jitter > 0.0):
            self._timer = self.create_timer(DELIVERY_TICK_S, self._deliver)

    def _on_hb(self, src: int, msg: UInt8MultiArray) -> None:
        data = bytes(msg.data)
        now = time.monotonic()
        for dst in self.ids:
            if dst == src:
                continue
            link = (src, dst)
            if link in self.blocked or (self.loss > 0.0 and self.rng.random() < self.loss):
                self.dropped[link] += 1
                continue
            delay = self.latency + (self.rng.random() * self.jitter if self.jitter > 0.0 else 0.0)
            if delay <= 0.0:
                self._publish(dst, data, link)
            else:
                heapq.heappush(self._queue, (now + delay, self._seq, dst, data))
                self._seq += 1
                self.forwarded[link] += 1

    def _publish(self, dst: int, data: bytes, link: tuple[int, int] | None = None) -> None:
        self.pubs[dst].publish(UInt8MultiArray(data=data))
        if link is not None:
            self.forwarded[link] += 1

    def _deliver(self) -> None:
        now = time.monotonic()
        while self._queue and self._queue[0][0] <= now:
            _, _, dst, data = heapq.heappop(self._queue)
            self._publish(dst, data)

    def _on_control(self, msg: String) -> None:
        try:
            c = json.loads(msg.data)
            cmd = c["cmd"]
            if cmd == "set":
                self.loss = c.get("loss_pct", self.loss * 100.0) / 100.0
                self.latency = c.get("latency_ms", self.latency * 1000.0) / 1000.0
                self.jitter = c.get("jitter_ms", self.jitter * 1000.0) / 1000.0
                self._ensure_timer()
            elif cmd == "block_tx":
                self.blocked |= {(c["src"], j) for j in self.ids if j != c["src"]}
            elif cmd == "partition":
                groups = [set(g) for g in c["groups"]]
                for a in groups:
                    for b in groups:
                        if a is not b:
                            self.blocked |= {(i, j) for i in a for j in b}
            elif cmd == "block":
                self.blocked |= {tuple(l) for l in c["links"]}
            elif cmd == "unblock":
                self.blocked -= {tuple(l) for l in c["links"]}
            elif cmd == "heal":
                self.blocked.clear()
            else:
                raise ValueError(f"unknown cmd {cmd!r}")
            self._event({**c, "blocked_links": len(self.blocked)})
        except (ValueError, KeyError, TypeError) as exc:
            self._event({"error": repr(exc), "raw": msg.data})

    def write_summary(self) -> None:
        fwd, drp = sum(self.forwarded.values()), sum(self.dropped.values())
        self._event({"cmd": "summary", "forwarded": fwd, "dropped": drp,
                     "drop_pct": round(100.0 * drp / (fwd + drp), 3) if fwd + drp else 0.0})


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="heartbeat link emulator")
    ap.add_argument("--num-drones", type=int, required=True)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--config", default=str(default_config_path()))
    ap.add_argument("--loss-pct", type=float, default=None)
    ap.add_argument("--latency-ms", type=float, default=None)
    ap.add_argument("--jitter-ms", type=float, default=None)
    args = ap.parse_args(argv)
    cfg = load_config(args.config).with_num_drones(args.num_drones)
    le = cfg.link_emulator
    rclpy.init()
    node = LinkEmulator(cfg.drone_ids,
                        le.loss_pct if args.loss_pct is None else args.loss_pct,
                        le.latency_ms if args.latency_ms is None else args.latency_ms,
                        le.jitter_ms if args.jitter_ms is None else args.jitter_ms,
                        le.seed, Path(args.run_dir))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
