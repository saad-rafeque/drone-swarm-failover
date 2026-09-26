"""Local obstacle avoidance for a drone in formation: classical potential field or a learned policy.

Both avoiders see exactly the same input and produce the same kind of output, so they can be
compared fairly and swapped in the agent:

  input   desired velocity from the formation law (or the leader's route), own velocity, error to
          the formation slot, 24 range readings around the drone (2-D lidar, 25 m), the closest
          obstacle point, and the three nearest drones (from their heartbeats)
  output  a horizontal velocity correction [m/s] added to the desired velocity

The drone-to-drone repulsion and the geofence (safety.py, AgentCore._safe) stay on top of either
avoider as a hard safety layer. Everything is expressed in the formation-heading frame
(x forward, y left), so the policy does not depend on which way the route points.

The learned policy runs with numpy only (weights exported from training to an .npz file), so the
onboard agent needs no machine-learning library.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

N_RAYS = 24
RAY_RANGE_M = 25.0
N_NEIGHBORS = 3
NEIGHBOR_RANGE_M = 25.0
V_SCALE_MPS = 10.0
ERR_SCALE_M = 20.0
MAX_CORRECTION_MPS = 8.0          # per axis, in the heading frame (enough to cancel the formation pull)
OBS_DIM = 2 + 2 + 2 + N_RAYS + 2 + 5 * N_NEIGHBORS
RAY_ANGLES = np.linspace(0.0, 2.0 * math.pi, N_RAYS, endpoint=False)   # relative to the heading


def rot(x: float, y: float, a: float) -> tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


@dataclass
class AvoidInput:
    """What one drone knows at one instant (ENU vectors; heading is the formation heading)."""
    heading: float
    v_des: tuple[float, float]
    vel: tuple[float, float]
    slot_err: tuple[float, float]
    rays: np.ndarray                                   # N_RAYS distances at heading + RAY_ANGLES
    nearest: tuple[float, float] | None = None         # vector to the closest obstacle point (None: none in range)
    neighbors: list[tuple[float, float, float, float]] = field(default_factory=list)  # rel x, y, vx, vy


def observation(inp: AvoidInput) -> np.ndarray:
    h = -inp.heading
    o = np.zeros(OBS_DIM, dtype=np.float32)
    o[0:2] = np.array(rot(*inp.v_des, h)) / V_SCALE_MPS
    o[2:4] = np.array(rot(*inp.vel, h)) / V_SCALE_MPS
    o[4:6] = np.clip(np.array(rot(*inp.slot_err, h)) / ERR_SCALE_M, -1.5, 1.5)
    o[6:6 + N_RAYS] = np.clip(np.asarray(inp.rays, dtype=float) / RAY_RANGE_M, 0.0, 1.0)
    if inp.nearest is not None:
        o[6 + N_RAYS:8 + N_RAYS] = np.array(rot(*inp.nearest, h)) / RAY_RANGE_M
    near = sorted((n for n in inp.neighbors if math.hypot(n[0], n[1]) < NEIGHBOR_RANGE_M),
                  key=lambda n: n[0] * n[0] + n[1] * n[1])[:N_NEIGHBORS]
    base = 8 + N_RAYS
    for k, (rx, ry, rvx, rvy) in enumerate(near):
        o[base + 5 * k: base + 5 * k + 2] = np.array(rot(rx, ry, h)) / NEIGHBOR_RANGE_M
        o[base + 5 * k + 2: base + 5 * k + 4] = np.array(rot(rvx, rvy, h)) / V_SCALE_MPS
        o[base + 5 * k + 4] = 1.0
    return o


def action_to_correction(action: np.ndarray, heading: float) -> tuple[float, float]:
    """Policy action in [-1, 1]^2 (heading frame) -> ENU velocity correction [m/s]."""
    a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0) * MAX_CORRECTION_MPS
    return rot(float(a[0]), float(a[1]), heading)


class NoAvoidance:
    name = "none"

    def correction(self, inp: AvoidInput) -> tuple[float, float]:
        return 0.0, 0.0


def brake(v: np.ndarray, inp: AvoidInput, a_brake: float = 3.0, d_safe_m: float = 1.5) -> np.ndarray:
    """Limit the velocity toward every range reading to what can still stop d_safe_m short of it:
    v . u_i <= sqrt(2 a (d_i - d_safe)). Classical stopping-distance filter."""
    v = np.array(v, dtype=float)
    ang = inp.heading + RAY_ANGLES
    d = np.asarray(inp.rays, dtype=float)
    allowed = np.sqrt(np.maximum(0.0, 2.0 * a_brake * (d - d_safe_m)))
    for i in np.argsort(d):
        if d[i] >= RAY_RANGE_M:
            break
        u = np.array([math.cos(ang[i]), math.sin(ang[i])])
        comp = float(v @ u)
        if comp > allowed[i]:
            v -= (comp - allowed[i]) * u
    return v


class PotentialField:
    """Classical baseline: repulsion from the closest obstacle point, a sliding term that removes the
    part of the desired velocity heading into the obstacle, a tangential (vortex) push that breaks
    symmetric dead-ends, and the stopping-distance brake on every range reading. Parameters are tuned
    on training scenarios (scripts/rl_tune_apf.py)."""
    name = "apf"

    def __init__(self, d0_m: float = 8.0, k_rep: float = 10.0, k_tan: float = 0.5, k_slide: float = 1.0,
                 a_brake: float = 3.0, d_safe_m: float = 1.5) -> None:
        self.d0, self.k_rep, self.k_tan, self.k_slide = d0_m, k_rep, k_tan, k_slide
        self.a_brake, self.d_safe = a_brake, d_safe_m

    def correction(self, inp: AvoidInput) -> tuple[float, float]:
        dv = self._field(inp)
        vd = np.asarray(inp.v_des, dtype=float)
        v = brake(vd + dv, inp, self.a_brake, self.d_safe)
        return float(v[0] - vd[0]), float(v[1] - vd[1])

    def _field(self, inp: AvoidInput) -> np.ndarray:
        if inp.nearest is None:
            return np.zeros(2)
        d = max(math.hypot(*inp.nearest), 0.2)
        if d >= self.d0:
            return np.zeros(2)
        u = np.array([-inp.nearest[0], -inp.nearest[1]]) / d        # unit vector away from the obstacle
        mag = self.k_rep * (1.0 / d - 1.0 / self.d0)
        dv = mag * u
        vd = np.asarray(inp.v_des, dtype=float)
        into = -float(vd @ u)                                       # desired speed toward the obstacle
        if into > 0.0:
            dv += self.k_slide * into * min(1.0, 2.0 * (self.d0 - d) / self.d0) * u
            t = np.array([-u[1], u[0]])
            if float(vd @ t) < 0.0:
                t = -t
            dv += self.k_tan * mag * t
        lim = MAX_CORRECTION_MPS * math.sqrt(2.0)
        nn = float(np.hypot(*dv))
        if nn > lim:
            dv *= lim / nn
        return dv


class Shielded:
    """Any avoider with the stopping-distance brake applied to its output (a safety shield)."""

    def __init__(self, inner, a_brake: float = 3.0, d_safe_m: float = 1.5) -> None:
        self.inner, self.a_brake, self.d_safe = inner, a_brake, d_safe_m
        self.name = f"{inner.name}+shield"

    def correction(self, inp: AvoidInput) -> tuple[float, float]:
        vd = np.asarray(inp.v_des, dtype=float)
        v = brake(vd + np.asarray(self.inner.correction(inp)), inp, self.a_brake, self.d_safe)
        return float(v[0] - vd[0]), float(v[1] - vd[1])


class LearnedPolicy:
    """Deterministic MLP policy (tanh hidden layers) exported from training; numpy inference only."""
    name = "rl"

    def __init__(self, path: str | Path) -> None:
        w = np.load(path)
        n = int(w["n_layers"])
        self.layers = [(w[f"W{k}"], w[f"b{k}"]) for k in range(n)]
        self.out = (w["Wa"], w["ba"])
        if self.layers[0][0].shape[1] != OBS_DIM:
            raise ValueError(f"policy expects {self.layers[0][0].shape[1]} inputs, this code builds {OBS_DIM}")

    def act(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float64)
        for W, b in self.layers:
            x = np.tanh(x @ W.T + b)
        return x @ self.out[0].T + self.out[1]

    def correction(self, inp: AvoidInput) -> tuple[float, float]:
        return action_to_correction(self.act(observation(inp)), inp.heading)


def make_avoider(kind: str, policy_path: str | Path | None = None):
    """none | apf | rl | rl+shield | none+shield"""
    base, _, shield = kind.partition("+")
    if base == "none":
        av = NoAvoidance()
    elif base == "apf":
        av = PotentialField()
    elif base == "rl":
        if policy_path is None:
            raise ValueError("the learned avoider needs a policy file")
        av = LearnedPolicy(policy_path)
    else:
        raise ValueError(f"unknown avoider {kind!r}")
    if shield:
        if shield != "shield":
            raise ValueError(f"unknown avoider {kind!r}")
        av = Shielded(av)
    return av
