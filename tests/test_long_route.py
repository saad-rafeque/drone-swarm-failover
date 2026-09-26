"""Long routes: ground-point conversion far from the origin, chunked planning, charging stops."""
from __future__ import annotations

import dataclasses
import math

import pytest

from swarm_agent.agent_core import World
from swarm_agent.geometry import EnuFrame, GeoPoint
from swarm_agent.heartbeat import Heartbeat, Phase, Role, decode, encode
from swarm_agent.obstacles import ObstacleMap, rectangle
from swarm_agent.planner import PathFollower, place_stops, plan_long_route
from swarm_tools.puresim import PureSim


def test_ground_points_round_trip_far_from_the_origin():
    frame = EnuFrame(GeoPoint(33.7036, 73.0231, 0.0))          # Islamabad
    lahore = GeoPoint(31.5204, 74.3587, 0.0)
    e, n, u = frame.to_enu(lahore)
    assert u < -5000.0                                          # the ground is km below the tangent plane there
    back = frame.surface_point(e, n)
    err = math.dist(frame.to_enu(back), frame.to_enu(lahore))
    assert err < 0.5
    naive = frame.to_geodetic((e, n, 0.0))                      # dropping u would be hundreds of metres off
    assert math.dist(frame.to_enu(GeoPoint(naive.lat_deg, naive.lon_deg, 0.0))[:2], (e, n)) > 100.0


def test_charging_phase_travels_in_the_heartbeat():
    hb = Heartbeat(drone_id=3, role=Role.MASTER, term=2, phase=Phase.CHARGE, master_id=3, handover_to=0, flags=0,
                   battery_pct=55.0, pos=(1.0, 2.0, 0.0), vel=(0.0, 0.0, 0.0), heading=0.0, members=0, stamp=1.0)
    assert decode(encode(hb)).phase == Phase.CHARGE


def test_long_route_goes_around_walls_across_the_line():
    walls = [rectangle(0.0, y, 400.0, 20.0, 0.0) for y in (1500.0, 4200.0, 7100.0)]   # 400 m wide walls
    omap = ObstacleMap(walls)
    route = plan_long_route(omap, (0.0, 0.0), (0.0, 9000.0), clearance_m=8.0, chunk_m=2000.0)
    assert route[0] == (0.0, 0.0) and route[-1] == (0.0, 9000.0)
    for a, b in zip(route[:-1], route[1:]):
        for u in (0.0, 0.25, 0.5, 0.75, 1.0):
            x, y = a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
            assert omap.clearance(x, y) > 8.0 - 2.0 * 4.0 * math.sqrt(2)   # clearance minus one coarse grid cell
    stops = place_stops(omap, route, every_m=2500.0, need_clear_m=45.0)
    assert 2 <= len(stops) <= 4
    assert all(omap.clearance(x, y, search_m=50.0) >= 45.0 for _, x, y in stops)
    assert all(b[0] - a[0] > 2000.0 for a, b in zip(stops, stops[1:]))
    assert PathFollower(route).length > 9000.0


def test_swarm_lands_at_a_stop_recharges_and_finishes(cfg):
    c = dataclasses.replace(cfg.with_num_drones(4), mission=dataclasses.replace(cfg.mission, goal_enu_m=(0.0, 1200.0)))
    route = [(0.0, 0.0), (0.0, 1200.0)]
    stops = place_stops(ObstacleMap(), route, every_m=600.0, end_margin_m=200.0)
    assert len(stops) == 1
    sim = PureSim(c, seed=4, world=World(ObstacleMap(), None, route, stops), drain_pct_per_s=100.0 / 400.0,
                  charge_pct_per_s=100.0 / 20.0)
    charged_at = None
    while sim.t < 900.0:
        sim.step()
        if charged_at is None and all(sim.agents[i].phase == Phase.CHARGE for i in sim.alive_ids()):
            charged_at = sim.t
            assert all(abs(sim.drones[i].pos[1] - 600.0) < 45.0 and sim.drones[i].landed for i in sim.alive_ids())
        if any(sim.agents[i].mission_complete for i in sim.alive_ids()) and all(sim.drones[i].landed for i in sim.alive_ids()):
            break
    assert charged_at is not None, "the swarm never charged"
    assert len(sim.alive_ids()) == 4 and sim.masters() == [1]
    m = sim.drones[1]
    assert math.hypot(m.pos[0], m.pos[1] - 1200.0) < 5.0 and m.landed
    assert all(sim.drones[i].battery_pct > 40.0 for i in sim.alive_ids())   # 1.2 km would drain 60 % without the stop
    assert sim.agents[1].next_stop == 1


def test_low_battery_on_a_long_route_lands_in_place(cfg):
    c = dataclasses.replace(cfg.with_num_drones(3), mission=dataclasses.replace(cfg.mission, goal_enu_m=(0.0, 1500.0)))
    route = [(0.0, 0.0), (0.0, 1500.0)]
    stops = place_stops(ObstacleMap(), route, every_m=700.0, end_margin_m=200.0)
    sim = PureSim(c, seed=5, world=World(ObstacleMap(), None, route, stops))
    sim.run_until(60.0)
    sim.set_battery(3, c.battery.handover_pct - 1.0)
    y_at = sim.drones[3].pos[1]
    sim.run_until(120.0)
    d = sim.drones[3]
    assert sim.role(3) == Role.RETIRED and d.landed and d.pos[1] > y_at - 20.0     # did not fly back home


@pytest.mark.parametrize("dist_km", [5, 60])
def test_stops_scale_with_route_length(dist_km):
    route = [(0.0, 0.0), (0.0, dist_km * 1000.0)]
    stops = place_stops(ObstacleMap(), route, every_m=4000.0)
    assert len(stops) == max(0, math.ceil((dist_km * 1000.0 - 500.0) / 4000.0) - 1)


def test_new_leader_after_a_stop_continues_to_the_next_stop(cfg):
    c = dataclasses.replace(cfg.with_num_drones(4), mission=dataclasses.replace(cfg.mission, goal_enu_m=(0.0, 1800.0)),
                            safety=dataclasses.replace(cfg.safety, geofence_radius_m=3000.0))
    route = [(0.0, 0.0), (0.0, 1800.0)]
    stops = place_stops(ObstacleMap(), route, every_m=600.0, end_margin_m=200.0)
    assert [round(st[0]) for st in stops] == [600, 1200]
    sim = PureSim(c, seed=6, world=World(ObstacleMap(), None, route, stops), drain_pct_per_s=100.0 / 500.0,
                  charge_pct_per_s=100.0 / 20.0)
    killed = False
    while sim.t < 1200.0:
        sim.step()
        a1 = sim.agents[1]
        if not killed and a1.next_stop == 1 and a1.phase == Phase.CRUISE and sim.drones[1].pos[1] > 750.0:
            sim.kill(1)
            killed = True
        if killed and any(sim.agents[i].mission_complete for i in sim.alive_ids()) \
                and all(sim.drones[i].landed for i in sim.alive_ids()):
            break
    assert killed and sim.masters() == [2]
    new = sim.agents[2]
    assert new.next_stop == 2                                     # charged at the second stop, not back at the first
    d = sim.drones[2]
    assert math.hypot(d.pos[0], d.pos[1] - 1800.0) < 5.0 and d.landed
    assert len(sim.alive_ids()) == 3
