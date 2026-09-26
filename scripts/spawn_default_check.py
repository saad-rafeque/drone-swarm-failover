#!/usr/bin/env python3
"""Phase 1 check: where do 2 SIH instances spawn when PX4_HOME_* is NOT set?"""
import json, time, threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from swarm_agent.config import load_config, default_config_path
from swarm_tools.mavros_link import MavrosLink
from swarm_tools.sim_launch import SimLauncher, kill_orphans, namespace_of

cfg = load_config(default_config_path()).with_num_drones(2)
kill_orphans()
sim = SimLauncher(cfg, {i: None for i in cfg.drone_ids})
rclpy.init()
node = Node("spawn_default_check")
links = {i: MavrosLink(node, i, namespace_of(i)) for i in cfg.drone_ids}
ex = SingleThreadedExecutor(); ex.add_node(node)
threading.Thread(target=ex.spin, daemon=True).start()
try:
    sim.start(cfg.drone_ids, stagger_s=0.5)
    t0 = time.time()
    while not all(l.fix for l in links.values()) and time.time() - t0 < 60:
        time.sleep(0.2)
    time.sleep(1.0)
    out = {i: {"lat": l.fix.latitude, "lon": l.fix.longitude} for i, l in links.items()}
    import math
    a, b = out[1], out[2]
    dn = (a["lat"] - b["lat"]) * 111_320.0
    de = (a["lon"] - b["lon"]) * 111_320.0 * math.cos(math.radians(a["lat"]))
    out["horizontal_separation_m_approx"] = round(math.hypot(dn, de), 2)
    print(json.dumps(out, indent=1))
finally:
    ex.shutdown(); node.destroy_node(); rclpy.shutdown(); sim.stop()
