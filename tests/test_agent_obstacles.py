"""The agent code with an obstacle World: route around a wall, avoidance, and going home around it."""
from __future__ import annotations

import dataclasses
import math

from swarm_agent.agent_core import World
from swarm_agent.avoidance import NoAvoidance, PotentialField
from swarm_agent.heartbeat import Phase, Role
from swarm_agent.obstacles import ObstacleMap
from swarm_agent.planner import plan_path
from swarm_tools.puresim import PureSim

WALL = [(-200.0, 150.0), (20.0, 150.0), (20.0, 160.0), (-200.0, 160.0)]   # blocks the straight line home -> goal


def fly(cfg, avoider, low_battery_at: float | None = None, t_end: float = 400.0):
    cfg = dataclasses.replace(cfg.with_num_drones(5), mission=dataclasses.replace(cfg.mission, goal_enu_m=(0.0, 300.0)))
    omap = ObstacleMap([WALL])
    route = plan_path(omap, (0.0, 0.0), (0.0, 300.0), clearance_m=8.0)
    sim = PureSim(cfg, seed=3, world=World(omap, avoider, route))
    retiree = None
    while sim.t < t_end:
        sim.step()
        if low_battery_at is not None and retiree is None and sim.t >= low_battery_at:
            retiree = 3
            sim.set_battery(retiree, cfg.battery.handover_pct - 1.0)
        ms = sim.masters()
        if ms and sim.agents[ms[0]].phase == Phase.LAND and all(sim.drones[i].landed for i in sim.alive_ids()) \
                and (retiree is None or sim.drones[retiree].landed):
            break
    return sim, route, retiree


def test_route_goes_around_the_wall_and_the_master_follows_it(cfg):
    sim, route, _ = fly(cfg, PotentialField())
    assert len(route) > 2 and max(x for x, _ in route) > 20.0          # around the east end of the wall
    assert not sim.obstacle_hits and len(sim.alive_ids()) == 5
    assert all(sim.drones[i].landed for i in sim.alive_ids())
    m = sim.masters()[0]
    assert math.hypot(sim.drones[m].pos[0], sim.drones[m].pos[1] - 300.0) < 5.0
    assert sim.min_sep_seen > 4.0


def test_without_avoidance_followers_fly_into_the_wall(cfg):
    sim, _, _ = fly(cfg, NoAvoidance())
    assert sim.obstacle_hits                                              # the left arm crosses the wall's end


def test_a_drone_going_home_plans_around_the_wall(cfg):
    sim, _, retiree = fly(cfg, PotentialField(), low_battery_at=90.0, t_end=600.0)
    assert sim.role(retiree) == Role.RETIRED and sim.drones[retiree].landed
    home = sim.agents[retiree].home
    assert math.hypot(sim.drones[retiree].pos[0] - home[0], sim.drones[retiree].pos[1] - home[1]) < 5.0
    assert not [h for h in sim.obstacle_hits if h[1] == retiree]
