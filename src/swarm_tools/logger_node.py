"""Logger (central tool): writes every drone's /uav<i>/agent_state to <run_dir>/states.jsonl.

One JSON object per line, as published by the agent at logging.rate_hz: time, id, position and
velocity in the shared ENU frame, role, term, master, phase, command, PX4 mode, armed, landed,
battery, and (for the master) its member list. scripts/metrics.py turns this into metrics.

Run: scripts/ros_env.sh python3 -m swarm_tools.logger_node --num-drones 10 --run-dir <dir>
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from swarm_agent.config import default_config_path, load_config


class StateLogger(Node):
    def __init__(self, ids: list[int], run_dir: Path) -> None:
        super().__init__("swarm_logger")
        run_dir.mkdir(parents=True, exist_ok=True)
        self.fh = open(run_dir / "states.jsonl", "a", encoding="utf-8", buffering=1 << 16)
        self.count = 0
        for i in ids:
            self.create_subscription(String, f"/uav{i}/agent_state", self._on_state, 50)
        self.create_timer(2.0, self.fh.flush)

    def _on_state(self, msg: String) -> None:
        self.fh.write(msg.data + "\n")
        self.count += 1

    def close(self) -> None:
        self.fh.flush()
        self.fh.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="swarm state logger")
    ap.add_argument("--num-drones", type=int, required=True)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--config", default=str(default_config_path()))
    args = ap.parse_args(argv)
    cfg = load_config(args.config).with_num_drones(args.num_drones)
    rclpy.init()
    node = StateLogger(cfg.drone_ids, Path(args.run_dir))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
