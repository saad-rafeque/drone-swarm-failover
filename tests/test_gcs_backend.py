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
    claim = next(e[3] for e in s["events"] if "became master (term 2" in e[3])
    gap = float(claim.split(", ")[-1].split(" s after the fault on drone 1")[0])   # takeover time after the kill
    assert 1.4 <= gap <= 2.5
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


def test_osm_elements_become_obstacles_without_network():
    import pytest
    from swarm_agent.geometry import EnuFrame, GeoPoint
    from swarm_tools.osm import to_obstacles
    frame = EnuFrame(GeoPoint(33.70, 73.02, 0.0))
    sq = [{"lat": 33.7001 + a, "lon": 73.0201 + b} for a, b in ((0, 0), (0, 1e-4), (1e-4, 1e-4), (1e-4, 0), (0, 0))]
    data = {"elements": [{"type": "way", "tags": {"building": "yes"}, "geometry": sq},
                         {"type": "way", "tags": {"natural": "wood"}, "geometry": [dict(p) for p in sq]},
                         {"type": "way", "tags": {"building": "yes"}, "geometry": sq[:3]},       # not closed: skipped
                         {"type": "node", "tags": {"natural": "tree"}, "lat": 33.7, "lon": 73.02}]}
    omap, counts, kinds = to_obstacles(data, frame)
    assert counts == {"buildings": 1, "woods": 1, "trees": 1, "below": 0} and kinds == ["building", "wood"]
    tall = {"elements": [dict(data["elements"][0], tags={"building": "yes", "building:levels": "10"}),
                         dict(data["elements"][0], tags={"building": "yes"}),                  # ~9 m: below 25 m
                         dict(data["elements"][1]), data["elements"][3]]}
    _, c25, k25 = to_obstacles(tall, frame, min_height_m=25.0)
    assert c25 == {"buildings": 1, "woods": 0, "trees": 0, "below": 3} and k25 == ["building"]
    assert omap.clearance(0.0, 0.0) < 0.0                      # inside the tree at the frame origin
    assert omap.clearance(0.0, -10.0, search_m=50) == pytest.approx(7.0)   # 10 m to the tree centre, radius 3


def test_obstacle_mission_uses_the_cached_osm_map(cfg):
    from swarm_tools import osm
    b = FastSimBackend(cfg)
    if not osm.cache_path(*b._bbox(b.params)).exists():
        import pytest
        pytest.skip("OSM data for the default route is not cached")
    assert b.command({"cmd": "start", "n": 6, "obstacles": "osm", "avoider": "apf"})["ok"]
    s = b.snapshot()
    assert s["route"] and len(s["route"]) >= 2 and s["obstacles_version"] == b.obstacles_payload()["version"]
    assert len(b.obstacles_payload()["polygons"]) > 100
    run(b, 90.0)
    s = b.snapshot()
    assert s["phase"] == "CRUISE" and s["hits"] == 0
    assert not b.command({"cmd": "start", "obstacles": "osm", "avoider": "nonsense"})["ok"]
    far = b.command({"cmd": "start", "obstacles": "osm", "avoider": "apf", "target": [24.8607, 67.0011]})  # Karachi
    assert not far["ok"] and "up to" in far["msg"] and b.loading is None                     # refused before any download


def test_city_to_city_open_sky_gets_charging_stops(cfg):
    b = FastSimBackend(cfg)
    r = b.command({"cmd": "start", "n": 5, "obstacles": "none", "target": [31.5204, 74.3587],   # Lahore
                   "endurance_min": 25, "cruise_mps": 5})
    assert r["ok"] and b.loading is None
    s = b.snapshot()
    assert 55 <= len(s["stops"]) <= 75                          # ~270 km in legs of ~4 km
    assert abs(s["route"][-1][0] - 31.5204) < 1e-5 and abs(s["route"][-1][1] - 74.3587) < 1e-5
    assert any("charging stops" in e[3] for e in s["events"])
    run(b, 60.0)
    s = b.snapshot()
    assert s["phase"] == "CRUISE" and s["next_stop"] == 0 and s["alive"] == 5


def test_docs_urls_open_repository_markdown_but_never_other_files():
    from swarm_tools.gcs.server import REPO, shared_file
    assert shared_file("/docs/README.md") == REPO / "README.md"
    assert shared_file("/docs/RUNBOOK.md") == (REPO / "docs" / "RUNBOOK.md").resolve()
    assert shared_file("/docs/CHANGELOG.md") == (REPO / "CHANGELOG.md").resolve()    # linked from the README
    assert shared_file("/reports/PHASE_0.md") == (REPO / "reports" / "PHASE_0.md").resolve()
    for url in ("/docs/../config/map_keys.local.yaml", "/docs/config/map_keys.example.yaml", "/docs/../../etc/passwd",
                "/docs/../README.md.local.md", "/docs/.git/config", "/docs/.pytest_cache/README.md", "/docs/src/swarm_tools/gcs/server.py"):
        assert shared_file(url) is None, url
