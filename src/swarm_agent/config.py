"""Typed loader for config/swarm.yaml. Unknown or missing keys are errors, not silent defaults."""
from __future__ import annotations

import dataclasses
import math
import typing
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .geometry import GeoPoint

MAX_DRONE_ID = 63  # heartbeat member bitmask is 64 bits (bit 0 unused)


@dataclass(frozen=True)
class SwarmCfg:
    num_drones: int
    first_id: int


@dataclass(frozen=True)
class OriginCfg:
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass(frozen=True)
class MissionCfg:
    cruise_alt_m: float
    cruise_speed_mps: float
    goal_enu_m: tuple[float, float]
    goal_radius_m: float
    hover_at_goal_s: float
    takeoff_alt_tolerance_m: float
    takeoff_timeout_s: float
    startup_listen_s: float
    startup_timeout_s: float
    climb_rate_mps: float
    descent_rate_mps: float
    land_speed_mps: float


@dataclass(frozen=True)
class FormationCfg:
    shape: str
    spacing_m: float
    v_half_angle_deg: float
    pos_gain: float
    max_correction_mps: float
    max_speed_mps: float
    orphan_alt_offset_m: float
    transit_threshold_m: float
    transit_exit_m: float
    transit_alt_offset_m: float
    transit_max_correction_mps: float


@dataclass(frozen=True)
class HeartbeatCfg:
    rate_hz: float
    master_timeout_s: float
    peer_timeout_s: float
    handover_timeout_s: float


@dataclass(frozen=True)
class BatteryCfg:
    handover_pct: float
    retire_alt_offset_m: float


@dataclass(frozen=True)
class SafetyCfg:
    min_separation_m: float
    repulsion_factor: float
    repulsion_gain: float
    ghost_timeout_s: float
    geofence_radius_m: float
    geofence_max_alt_m: float
    geofence_margin_m: float


@dataclass(frozen=True)
class SetpointCfg:
    rate_hz: float


@dataclass(frozen=True)
class LinkCfg:
    latency_ms: float
    loss_pct: float


@dataclass(frozen=True)
class SimCfg:
    px4_dir: str
    px4_version: str
    build_target: str
    model: str
    work_dir: str


@dataclass(frozen=True)
class Config:
    swarm: SwarmCfg
    origin: OriginCfg
    mission: MissionCfg
    formation: FormationCfg
    heartbeat: HeartbeatCfg
    battery: BatteryCfg
    safety: SafetyCfg
    setpoints: SetpointCfg
    link_emulator: LinkCfg
    sim: SimCfg

    @property
    def drone_ids(self) -> list[int]:
        return list(range(self.swarm.first_id, self.swarm.first_id + self.swarm.num_drones))

    @property
    def origin_geo(self) -> GeoPoint:
        return GeoPoint(self.origin.lat_deg, self.origin.lon_deg, self.origin.alt_m)

    def with_num_drones(self, n: int) -> "Config":
        cfg = dataclasses.replace(self, swarm=dataclasses.replace(self.swarm, num_drones=n))
        validate(cfg)
        return cfg


class ConfigError(ValueError):
    pass


def _build(cls: type, data: Any, where: str) -> Any:
    if not isinstance(data, dict):
        raise ConfigError(f"{where}: expected a mapping, got {type(data).__name__}")
    hints = typing.get_type_hints(cls)
    names = [f.name for f in dataclasses.fields(cls)]
    missing = [n for n in names if n not in data]
    unknown = [k for k in data if k not in names]
    if missing or unknown:
        raise ConfigError(f"{where}: missing keys {missing}, unknown keys {unknown}")
    kwargs = {}
    for name in names:
        hint = hints[name]
        value = data[name]
        if dataclasses.is_dataclass(hint):
            kwargs[name] = _build(hint, value, f"{where}.{name}")
        elif typing.get_origin(hint) is tuple:
            args = typing.get_args(hint)
            if not isinstance(value, (list, tuple)) or len(value) != len(args):
                raise ConfigError(f"{where}.{name}: expected a list of {len(args)} numbers")
            kwargs[name] = tuple(float(v) for v in value)
        elif hint is float:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigError(f"{where}.{name}: expected a number, got {value!r}")
            kwargs[name] = float(value)
        elif hint is int:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ConfigError(f"{where}.{name}: expected an integer, got {value!r}")
            kwargs[name] = value
        else:
            kwargs[name] = str(value)
    return cls(**kwargs)


def validate(cfg: Config) -> None:
    ids = cfg.drone_ids
    if cfg.swarm.num_drones < 1 or ids[0] < 1 or ids[-1] > MAX_DRONE_ID:
        raise ConfigError(f"drone IDs must be within 1..{MAX_DRONE_ID}, got {ids[0]}..{ids[-1]}")
    s, m, f = cfg.safety, cfg.mission, cfg.formation
    ceiling = s.geofence_max_alt_m - s.geofence_margin_m
    for label, alt in (
        ("cruise altitude", m.cruise_alt_m),
        ("orphan layer", m.cruise_alt_m + f.orphan_alt_offset_m),
        ("return layer", m.cruise_alt_m + cfg.battery.retire_alt_offset_m),
    ):
        if alt > ceiling:
            raise ConfigError(f"{label} {alt} m is above geofence ceiling minus margin ({ceiling} m)")
    layers = sorted([0.0, f.orphan_alt_offset_m, f.transit_alt_offset_m, cfg.battery.retire_alt_offset_m])
    if any(b - a < s.min_separation_m for a, b in zip(layers, layers[1:])):
        raise ConfigError(f"altitude layers {layers} (relative to cruise) must be >= min separation apart")
    if m.cruise_alt_m + f.transit_alt_offset_m < 2.0 * s.min_separation_m:
        raise ConfigError("transit layer too close to the ground")
    if not f.transit_exit_m < f.transit_threshold_m:
        raise ConfigError("transit_exit_m must be below transit_threshold_m")
    if math.hypot(*m.goal_enu_m) > s.geofence_radius_m - s.geofence_margin_m:
        raise ConfigError("goal lies outside the geofence")
    if f.spacing_m <= s.min_separation_m:
        raise ConfigError("formation spacing must exceed the minimum separation")
    if f.shape != "V":
        raise ConfigError(f"unsupported formation shape {f.shape!r}")
    hb = cfg.heartbeat
    if hb.master_timeout_s < 2.0 / hb.rate_hz:
        raise ConfigError("master_timeout_s must cover at least two heartbeat periods")
    if not 0.0 <= cfg.link_emulator.loss_pct <= 100.0:
        raise ConfigError("link_emulator.loss_pct must be within 0..100")


def load_config(path: str | Path) -> Config:
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    cfg = _build(Config, raw, "config")
    validate(cfg)
    return cfg


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "swarm.yaml"
