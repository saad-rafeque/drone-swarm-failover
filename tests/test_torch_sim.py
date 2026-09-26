"""The batched PyTorch simulator must step exactly like the numpy reference (obstacle_sim.FormationSim)."""
from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from swarm_agent.avoidance import OBS_DIM  # noqa: E402
from swarm_tools.obstacle_sim import FormationSim, make_scenario, sim_cfg_from  # noqa: E402
from swarm_tools.torch_sim import BatchedFormationSim, ScenarioPool, pack_scenarios  # noqa: E402


def reference_pool(cfg, n_drones=6, seeds=(31, 32, 33)):
    sc = replace(sim_cfg_from(cfg.with_num_drones(n_drones)), drift_sigma_mps=0.0, route_m=(200.0, 300.0),
                 building_density=(1.5, 1.5), tree_density=(0.8, 0.8))
    scenarios = [make_scenario(s, sc) for s in seeds]
    pool = ScenarioPool.from_arrays(pack_scenarios(scenarios), "cpu", torch.float64)
    return sc, scenarios, pool


def test_batched_sim_matches_the_numpy_reference_step_for_step(cfg):
    sc, scenarios, pool = reference_pool(cfg)
    B = len(scenarios)
    ts = BatchedFormationSim(sc, pool, B, "cpu", k_seg=10_000, k_circ=10_000, dtype=torch.float64)
    obs_t = ts.reset(scenario_idx=torch.arange(B))
    ref = [FormationSim(sc) for _ in range(B)]
    obs_n = [r.reset(s) for r, s in zip(ref, scenarios)]
    for b in range(B):
        assert np.allclose(obs_t[b].numpy(), obs_n[b], atol=1e-6)
    rng = np.random.default_rng(0)
    done = [False] * B
    steps = 0
    while not all(done) and steps < 700:
        act = np.clip(rng.normal(0.0, 0.4, (B, sc.n_drones - 1, 2)), -1, 1)
        o_t, r_t, term_t, trunc_t, over_t, _, _ = ts.step(torch.as_tensor(act), auto_reset=False)
        for b in range(B):
            if done[b]:
                continue
            o, r, term, trunc, info = ref[b].step(act[b])
            assert np.allclose(ts.pos[b].numpy(), ref[b].pos, atol=1e-6), (b, steps)
            assert np.allclose(r_t[b].numpy(), r, atol=1e-5), (b, steps)
            assert np.array_equal(term_t[b].numpy(), term) and np.array_equal(trunc_t[b].numpy(), trunc)
            assert np.allclose(o_t[b].numpy(), o, atol=1e-5), (b, steps)
            assert bool(over_t[b]) == info["over"]
            done[b] = info["over"]
        steps += 1
    assert steps > 50


def test_auto_reset_returns_terminal_observation_and_summaries(cfg):
    sc = replace(sim_cfg_from(cfg.with_num_drones(4)), route_m=(120.0, 120.0), building_density=(0.0, 0.0),
                 tree_density=(0.0, 0.0))
    pool = ScenarioPool.from_arrays(pack_scenarios([make_scenario(s, sc) for s in (41, 42)]), "cpu")
    ts = BatchedFormationSim(sc, pool, 4, "cpu", seed=1)
    ts.reset()
    ends = []
    for _ in range(600):
        obs, rew, term, trunc, over, term_obs, finished = ts.step(torch.zeros(4, sc.n_drones - 1, 2))
        if bool(over.any()):
            ends.extend(finished)
            assert torch.all(ts.t[over] == 0.0)            # finished envs start a new episode
            assert torch.all(trunc[over]) and not bool(term.any())
    assert obs.shape == (4, sc.n_drones - 1, OBS_DIM)
    assert len(ends) >= 4 and all(e["success"] and e["final_rms_m"] < 1.0 for e in ends)
