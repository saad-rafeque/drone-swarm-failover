"""Ground-control fast-sim backend: commands, faults and snapshots (no browser, no ROS)."""
from __future__ import annotations

import math

from swarm_tools.gcs.fastsim_backend import FastSimBackend, mission_config

HOME = [33.7036, 73.0231]          # Islamabad, F-9 Park (example)


def run(b: FastSimBackend, seconds: float) -> None:
    """Step the simulation directly (deterministic; the background thread stays paused)."""
    with b.lock:
        b.running = False
        for _ in range(int(seconds / b.sim.dt)):
            b._stats = b.sim.step()
            b._after_step()


def test_mission_on_real_gps_with_faults(cfg):
    b = FastSimBackend(cfg)
    assert b.command({"cmd": "start", "n": 5, "home": HOME, "target": [33.7100, 73.0300]})["ok"]
    run(b, 60.0)
    s = b.snapshot()
    assert s["master"] == 1 and s["phase"] == "CRUISE" and len(s["drones"]) == 5
    assert all(abs(d["lat"] - HOME[0]) < 0.02 and abs(d["lon"] - HOME[1]) < 0.02 for d in s["drones"])
    assert b.command({"cmd": "fault", "type": "crash", "target": "master"})["ok"]
    run(b, 4.0)
    s = b.snapshot()
    assert s["master"] == 2 and s["alive"] == 4
    assert any("became master (term 2" in e[3] for e in s["events"])
    assert b.command({"cmd": "fault", "type": "gps", "target": [3]})["ok"]
    run(b, 1.0)
    d3 = next(d for d in b.snapshot()["drones"] if d["id"] == 3)
    assert d3["role"] == "RETIRED" and not d3["gps_ok"] and d3["status"].startswith("emergency")
    assert not b.command({"cmd": "fault", "type": "nonsense", "target": [3]})["ok"]
    assert not b.command({"cmd": "start", "target": HOME})["ok"]       # target on top of home is refused


def test_random_kills_are_repeatable_with_the_same_seed(cfg):
    picks = []
    for _ in range(2):
        b = FastSimBackend(cfg)
        b.command({"cmd": "start", "n": 8, "home": HOME, "target": [33.7100, 73.0300], "seed": 5})
        run(b, 40.0)
        b.command({"cmd": "fault", "type": "crash", "target": "random:3"})
        picks.append(sorted(d["id"] for d in b.snapshot()["drones"] if d["role"] == "DOWN"))
    assert picks[0] == picks[1] and len(picks[0]) == 3


def test_long_route_islamabad_to_lahore_converts(cfg):
    c = mission_config(cfg, 10, HOME, [31.5204, 74.3587], 5.0)
    dist = math.hypot(*c.mission.goal_enu_m)
    assert 250_000 < dist < 300_000
    assert c.safety.geofence_radius_m > dist
