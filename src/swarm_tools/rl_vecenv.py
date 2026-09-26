"""Stable-Baselines3 vector environment over obstacle_sim: every follower drone is one "env".

All followers share one policy (parameter sharing). A shared episode ends when the leader has
arrived and the formation has had time to settle, when time runs out, or at the first crash.
Crashed drones get a terminal transition; the others are marked as truncated so PPO bootstraps
their value instead of treating the stop as their own failure.

Training draws scenario seeds below TRAIN_SEED_MAX; evaluation uses seeds at or above it, so
the comparison runs on routes the policy has never seen.
"""
from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from swarm_agent.avoidance import OBS_DIM
from swarm_tools.obstacle_sim import FormationSim, SimCfg

TRAIN_SEED_MAX = 1_000_000


class SwarmVecEnv(VecEnv):
    def __init__(self, cfg: SimCfg, n_sims: int, seed: int = 0) -> None:
        self.sims = [FormationSim(cfg, seed=seed * 7919 + k) for k in range(n_sims)]
        self.na = cfg.n_drones - 1
        self._rng = np.random.default_rng(seed)
        self._actions: np.ndarray | None = None
        super().__init__(n_sims * self.na,
                         gym.spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32),
                         gym.spaces.Box(-1.0, 1.0, (2,), np.float32))

    def _scenario(self) -> int:
        return int(self._rng.integers(0, TRAIN_SEED_MAX))

    def reset(self) -> np.ndarray:
        return np.concatenate([sim.reset(self._scenario()) for sim in self.sims])

    def step_async(self, actions: np.ndarray) -> None:
        self._actions = np.asarray(actions, dtype=float)

    def step_wait(self):
        na = self.na
        obs_out, rew_out, done_out, infos = [], [], [], []
        for k, sim in enumerate(self.sims):
            obs, rew, term, trunc, info = sim.step(self._actions[k * na:(k + 1) * na])
            inf: list[dict[str, Any]] = [{} for _ in range(na)]
            if info["over"]:
                for i in range(na):
                    inf[i]["terminal_observation"] = obs[i]
                    inf[i]["TimeLimit.truncated"] = bool(trunc[i])
                    inf[i]["episode_summary"] = sim.summary() if i == 0 else None
                obs = sim.reset(self._scenario())
                done = np.ones(na, dtype=bool)
            else:
                done = np.zeros(na, dtype=bool)
            obs_out.append(obs)
            rew_out.append(rew)
            done_out.append(done)
            infos.extend(inf)
        return np.concatenate(obs_out), np.concatenate(rew_out), np.concatenate(done_out), infos

    def close(self) -> None:
        pass

    def get_attr(self, attr_name: str, indices=None) -> list[Any]:
        return [getattr(self, attr_name)] * len(self._indices(indices))

    def set_attr(self, attr_name: str, value: Any, indices=None) -> None:
        setattr(self, attr_name, value)

    def env_method(self, method_name: str, *method_args, indices=None, **method_kwargs) -> list[Any]:
        return [None] * len(self._indices(indices))

    def env_is_wrapped(self, wrapper_class, indices=None) -> list[bool]:
        return [False] * len(self._indices(indices))

    def _indices(self, indices) -> list[int]:
        if indices is None:
            return list(range(self.num_envs))
        return [indices] if isinstance(indices, int) else list(indices)
