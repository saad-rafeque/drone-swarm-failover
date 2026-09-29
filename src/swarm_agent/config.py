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
from .heartbeat import MAX_ID as MAX_DRONE_ID  # PX4 MAV_SYS_ID range is 1..250

# MAVLink message IDs of the streams the agent may set (mavros/set_message_interval), from MAVLink
# common.xml (PX4 v1.18.0-rc1: src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml)
MAVLINK_MSG_IDS = {"SYS_STATUS": 1, "GPS_RAW_INT": 24, "ATTITUDE": 30, "ATTITUDE_QUATERNION": 31,
                   "LOCAL_POSITION_NED": 32, "GLOBAL_POSITION_INT": 33, "VFR_HUD": 74, "HIGHRES_IMU": 105,
                   "ALTITUDE": 141, "HOME_POSITION": 242, "EXTENDED_SYS_STATE": 245, "STATUSTEXT": 253,
                   "ODOMETRY": 331}
# PX4 `mavlink start -m` modes a radio link may use ("normal" = PX4's default, started without -m)
PX4_LINK_MODES = ("normal", "minimal", "low_bandwidth", "onboard_low_bandwidth")
# MAVLink v2 frame sizes (payload + 12 bytes header and checksum, common.xml) of what MAVROS sends
SETPOINT_FRAME_B = 65      # SET_POSITION_TARGET_LOCAL_NED (payload 53 B)
TIMESYNC_FRAME_B = 30      # TIMESYNC (payload 18 B), sent by MAVROS and answered by PX4
SYSTEM_TIME_FRAME_B = 24   # SYSTEM_TIME (payload 12 B), 1 Hz from MAVROS
HEARTBEAT_FRAME_B = 21     # HEARTBEAT (payload 9 B), 1 Hz from MAVROS
RADIO_MAX_LOAD = 0.8       # design choice: planned traffic may use at most 80 % of the radio's usable rate


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
    cruise_accel_mps2: float
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
    fcu_timeout_s: float


@dataclass(frozen=True)
class BatteryCfg:
    handover_pct: float
    critical_pct: float
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
    jitter_ms: float
    loss_pct: float
    seed: int


@dataclass(frozen=True)
class RadioStandinCfg:
    air_rate_bps: float
    ecc: int
    max_window_ms: float
    loss_pct: float
    max_queue_s: float
    mavros_port_base: int
    relay_port_base: int
    seed: int
    # how a drone on a telemetry radio is set up (the stand-in, and later a real drone on a radio)
    px4_link_mode: str                        # PX4 MAVLink mode of that link (MAV_0_MODE on TELEM1)
    px4_link_rate_bytes_s: int                # PX4's cap on that link (MAV_0_RATE)
    timesync_rate_hz: float                   # MAVROS time sync over the radio (MAVROS default 10 Hz)
    fcu_timeout_s: float                      # replaces heartbeat.fcu_timeout_s for that drone
    autopilot_streams_hz: dict[str, float]    # replaces autopilot_streams_hz for that drone

    @property
    def usable_rate_bps(self) -> float:
        """Both directions share this rate (time-division turns); error correction halves it."""
        return self.air_rate_bps / (2.0 if self.ecc else 1.0)

    def planned_load_bytes_s(self, setpoint_rate_hz: float) -> float:
        """Worst-case traffic of one radio drone, both directions: PX4's capped telemetry, MAVROS's
        setpoints, time sync (request and answer), system time and heartbeat."""
        up = (setpoint_rate_hz * SETPOINT_FRAME_B + self.timesync_rate_hz * TIMESYNC_FRAME_B
              + SYSTEM_TIME_FRAME_B + HEARTBEAT_FRAME_B)
        down = self.px4_link_rate_bytes_s + self.timesync_rate_hz * TIMESYNC_FRAME_B
        return up + down


@dataclass(frozen=True)
class LoggingCfg:
    rate_hz: float


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
    autopilot_streams_hz: dict[str, float]
    link_emulator: LinkCfg
    radio_standin: RadioStandinCfg
    logging: LoggingCfg
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
        elif typing.get_origin(hint) is dict:
            if not isinstance(value, dict):
                raise ConfigError(f"{where}.{name}: expected a mapping")
            kwargs[name] = {str(k): float(v) for k, v in value.items()}
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
    if not 0.0 <= cfg.battery.critical_pct < cfg.battery.handover_pct:
        raise ConfigError("battery.critical_pct must be below battery.handover_pct")
    if not f.transit_exit_m < f.transit_threshold_m:
        raise ConfigError("transit_exit_m must be below transit_threshold_m")
    if math.hypot(*m.goal_enu_m) > s.geofence_radius_m - s.geofence_margin_m:
        raise ConfigError("goal lies outside the geofence")
    if f.spacing_m <= s.min_separation_m:
        raise ConfigError("formation spacing must exceed the minimum separation")
    if f.shape not in ("V", "line", "column", "echelon"):     # swarm_agent.formation.SHAPES
        raise ConfigError(f"unsupported formation shape {f.shape!r}")
    hb = cfg.heartbeat
    if hb.master_timeout_s < 2.0 / hb.rate_hz:
        raise ConfigError("master_timeout_s must cover at least two heartbeat periods")
    if not 0.0 <= cfg.link_emulator.loss_pct <= 100.0:
        raise ConfigError("link_emulator.loss_pct must be within 0..100")
    r = cfg.radio_standin
    if r.air_rate_bps <= 0 or r.ecc not in (0, 1) or r.max_window_ms < 0 or r.max_queue_s <= 0:
        raise ConfigError("radio_standin: air_rate_bps > 0, ecc 0 or 1, max_window_ms >= 0, max_queue_s > 0")
    if not 0.0 <= r.loss_pct <= 100.0:
        raise ConfigError("radio_standin.loss_pct must be within 0..100")
    if r.px4_link_mode not in PX4_LINK_MODES or not 10 <= r.px4_link_rate_bytes_s <= 10_000_000:
        raise ConfigError(f"radio_standin: px4_link_mode one of {PX4_LINK_MODES}, px4_link_rate_bytes_s 10..10000000")
    if r.timesync_rate_hz < 0 or r.fcu_timeout_s <= 0:
        raise ConfigError("radio_standin: timesync_rate_hz >= 0 and fcu_timeout_s > 0")
    load = r.planned_load_bytes_s(cfg.setpoints.rate_hz)
    if load > RADIO_MAX_LOAD * r.usable_rate_bps / 8.0:
        raise ConfigError(f"radio_standin: planned traffic {load:.0f} B/s exceeds {RADIO_MAX_LOAD:.0%} of the radio's "
                          f"usable {r.usable_rate_bps / 8.0:.0f} B/s")
    for label, streams in (("autopilot_streams_hz", cfg.autopilot_streams_hz),
                           ("radio_standin.autopilot_streams_hz", r.autopilot_streams_hz)):
        unknown = sorted(set(streams) - set(MAVLINK_MSG_IDS))
        if unknown or any(v <= 0 for v in streams.values()):
            raise ConfigError(f"{label}: unknown streams {unknown} or a rate <= 0 (known: {sorted(MAVLINK_MSG_IDS)})")
    # port blocks of 40 (+ PX4 instance), like PX4's own offboard plan (14540 / 14580), must not overlap
    blocks = [r.mavros_port_base, r.relay_port_base, 14540, 14580]
    if any(abs(p - q) < 40 for i, p in enumerate(blocks) for q in blocks[i + 1:]):
        raise ConfigError("radio_standin port bases must be 40 or more apart and clear of PX4's 14540 / 14580 blocks")


def load_config(path: str | Path) -> Config:
    with open(path, encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)
    cfg = _build(Config, raw, "config")
    validate(cfg)
    return cfg


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "swarm.yaml"
