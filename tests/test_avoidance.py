"""Avoiders (none / potential field / learned policy) and the obstacle simulator's agreement with the agent code."""
from __future__ import annotations

import math
import random
from dataclasses import replace

import numpy as np
import pytest

from swarm_agent.avoidance import (MAX_CORRECTION_MPS, N_RAYS, OBS_DIM, RAY_ANGLES, RAY_RANGE_M, AvoidInput,
                                   LearnedPolicy, NoAvoidance, PotentialField, Shielded, brake, make_avoider,
                                   observation)
from swarm_agent.formation import follower_velocity
from swarm_agent.obstacles import ObstacleMap
from swarm_agent.safety import repulsion
from swarm_tools.obstacle_sim import FormationSim, make_scenario, run_episode, sim_cfg_from


def inp_at(m: ObstacleMap, x: float, y: float, heading: float, v_des=(5.0, 0.0), neighbors=()) -> AvoidInput:
    return AvoidInput(heading, v_des, (0.0, 0.0), (0.0, 0.0), m.raycast(x, y, heading + RAY_ANGLES, RAY_RANGE_M),
                      m.nearest_point(x, y, RAY_RANGE_M), list(neighbors))


def test_observation_is_the_same_whichever_way_the_route_points():
    wall = ObstacleMap([[(10, -20), (12, -20), (12, 20), (10, 20)]])
    a = inp_at(wall, 0.0, 0.0, 0.0, v_des=(5.0, 1.0), neighbors=[(-7.0, 7.0, 0.5, 0.0)])
    th = 1.1                                    # rotate the whole scene by th around the origin
    c, s = math.cos(th), math.sin(th)
    rotated = ObstacleMap([[(x * c - y * s, x * s + y * c) for x, y in [(10, -20), (12, -20), (12, 20), (10, 20)]]])
    rv = lambda x, y: (x * c - y * s, x * s + y * c)  # noqa: E731
    b = inp_at(rotated, 0.0, 0.0, th, v_des=rv(5.0, 1.0), neighbors=[(*rv(-7.0, 7.0), *rv(0.5, 0.0))])
    oa, ob = observation(a), observation(b)
    assert oa.shape == (OBS_DIM,) and np.allclose(oa, ob, atol=1e-5)
    assert oa[6] == pytest.approx(10.0 / RAY_RANGE_M)       # ray straight ahead hits the wall at 10 m


def test_potential_field_brakes_and_pushes_away():
    wall = ObstacleMap([[(4, -20), (6, -20), (6, 20), (4, 20)]])
    inp = inp_at(wall, 0.0, 0.0, 0.0, v_des=(8.0, 0.0))
    dvx, dvy = PotentialField().correction(inp)
    vx = 8.0 + dvx
    assert vx <= math.sqrt(2 * 3.0 * (4.0 - 1.5)) + 1e-9     # stopping-distance limit toward the wall
    assert PotentialField().correction(inp_at(ObstacleMap(), 0, 0, 0.0)) == (0.0, 0.0)
    assert NoAvoidance().correction(inp) == (0.0, 0.0)


def test_brake_only_limits_motion_toward_close_readings():
    wall = ObstacleMap([[(3, -20), (5, -20), (5, 20), (3, 20)]])
    inp = inp_at(wall, 0.0, 0.0, 0.0)
    v = brake(np.array([6.0, 2.0]), inp)
    assert v[0] <= math.sqrt(2 * 3.0 * (3.0 - 1.5)) + 1e-9 and v[1] == pytest.approx(2.0, abs=0.6)
    away = brake(np.array([-6.0, 0.0]), inp)
    assert away == pytest.approx([-6.0, 0.0])
    sh = Shielded(NoAvoidance())
    assert sh.name == "none+shield" and 5.0 + sh.correction(inp_at(wall, 0, 0, 0.0))[0] < 5.0


def test_learned_policy_math_and_input_check(tmp_path):
    rng = np.random.default_rng(0)
    w = {"n_layers": np.array(2), "W0": rng.normal(size=(8, OBS_DIM)), "b0": rng.normal(size=8),
         "W1": rng.normal(size=(8, 8)), "b1": rng.normal(size=8), "Wa": rng.normal(size=(2, 8)), "ba": rng.normal(size=2)}
    np.savez(tmp_path / "p.npz", **w)
    pol = LearnedPolicy(tmp_path / "p.npz")
    o = rng.uniform(-1, 1, OBS_DIM)
    ref = w["Wa"] @ np.tanh(w["W1"] @ np.tanh(w["W0"] @ o + w["b0"]) + w["b1"]) + w["ba"]
    assert pol.act(o) == pytest.approx(ref)
    dv = pol.correction(inp_at(ObstacleMap(), 0, 0, 0.0))
    assert max(abs(dv[0]), abs(dv[1])) <= MAX_CORRECTION_MPS + 1e-9
    w["W0"] = rng.normal(size=(8, OBS_DIM + 1))
    np.savez(tmp_path / "bad.npz", **w)
    with pytest.raises(ValueError):
        LearnedPolicy(tmp_path / "bad.npz")
    assert make_avoider("rl+shield", tmp_path / "p.npz").name == "rl+shield"
    with pytest.raises(ValueError):
        make_avoider("rl")


def test_sim_follower_law_and_repulsion_match_the_agent_code(cfg):
    sc = sim_cfg_from(cfg.with_num_drones(10))
    sim = FormationSim(replace(sc, building_density=(0, 0), tree_density=(0, 0)), seed=1)
    sim.reset(7)
    rng = random.Random(2)
    sim.pos += np.array([[rng.uniform(-6, 6), rng.uniform(-6, 6)] for _ in range(sim.n)])
    sim.vel = np.array([[rng.uniform(-4, 4), rng.uniform(-4, 4)] for _ in range(sim.n)])
    slots, vdes = sim.slots(), sim.desired()
    f = cfg.formation
    for k in range(sim.nf):
        ref = follower_velocity((*sim.pos[k + 1], 0.0), (*slots[k], 0.0), (*sim.vel[0], 0.0), f.pos_gain,
                                f.max_correction_mps, f.max_speed_mps, 2.0)
        assert vdes[k] == pytest.approx(ref[:2])
    s = cfg.safety
    pts = [(rng.uniform(-8, 8), rng.uniform(-8, 8)) for _ in range(6)]
    for i, p in enumerate(pts):
        ref = repulsion((*p, 0.0), [(*q, 0.0) for j, q in enumerate(pts) if j != i], s.min_separation_m,
                        s.repulsion_factor, s.repulsion_gain)
        r_act = s.repulsion_factor * s.min_separation_m
        mine = np.zeros(2)
        for j, q in enumerate(pts):
            d = math.dist(p, q)
            if j != i and d < r_act:
                mine += (np.array(p) - q) / d * s.repulsion_gain * (r_act - d) / (r_act - s.min_separation_m)
        assert mine == pytest.approx(ref[:2])


def test_vectorised_observations_equal_the_reference_observation(cfg):
    sc = replace(sim_cfg_from(cfg.with_num_drones(10)), building_density=(2.0, 2.0), tree_density=(1.0, 1.0))
    sim = FormationSim(sc, seed=3)
    sim.reset(make_scenario(21, sc))
    rng = np.random.default_rng(4)
    for trial in range(6):
        sim.pos = sim.pos + rng.uniform(-15, 15, sim.pos.shape)
        sim.vel = rng.uniform(-6, 6, sim.vel.shape)
        sim.heading += rng.uniform(-1, 1)
        sim.alive[rng.integers(1, sim.n)] = trial % 2 == 0    # sometimes a dead follower
        sim.t += 0.1                                          # new state: invalidate the sensing cache
        ref = np.stack([observation(inp) if sim.alive[k + 1] else np.zeros(OBS_DIM, np.float32)
                        for k, inp in enumerate(sim.inputs())])
        assert np.allclose(sim.observe(), ref, atol=1e-6)


def test_open_sky_episode_keeps_formation_and_scenarios_are_reproducible(cfg):
    sc = replace(sim_cfg_from(cfg.with_num_drones(6)), building_density=(0, 0), tree_density=(0, 0),
                 route_m=(200.0, 200.0))
    r = run_episode(sc, 3, NoAvoidance())
    assert r["success"] and r["crashes_obstacle"] == 0 and r["final_rms_m"] < 1.0 and r["min_sep_m"] > 5.0
    dense = sim_cfg_from(cfg.with_num_drones(6))
    a, b = make_scenario(11, dense), make_scenario(11, dense)
    assert a.path == b.path and len(a.polygons) == len(b.polygons)
    for end in (a.start, a.goal):
        assert a.omap.clearance(*end, search_m=100.0) > dense.keep_clear_m - 1.0


def test_vector_env_runs_one_episode_with_random_actions(cfg):
    pytest.importorskip("stable_baselines3")
    from swarm_tools.rl_vecenv import SwarmVecEnv
    sc = replace(sim_cfg_from(cfg.with_num_drones(4)), route_m=(150.0, 150.0))
    env = SwarmVecEnv(sc, n_sims=2, seed=0)
    obs = env.reset()
    assert obs.shape == (6, OBS_DIM)
    ends = 0
    for _ in range(600):
        obs, rew, done, infos = env.step(np.random.default_rng(0).uniform(-1, 1, (6, 2)))
        if done.any():
            assert all("terminal_observation" in i for i, d in zip(infos, done) if d)
            ends += 1
    assert ends > 0 and obs.shape == (6, OBS_DIM)
