"""config/swarm.yaml loader and validation."""
from __future__ import annotations

import copy

import pytest
import yaml

from swarm_agent.config import ConfigError, _build, Config, default_config_path, load_config, validate


def raw() -> dict:
    return yaml.safe_load(default_config_path().read_text())


def build(d: dict) -> Config:
    cfg = _build(Config, d, "config")
    validate(cfg)
    return cfg


def test_default_config_loads_with_section5_values():
    cfg = load_config(default_config_path())
    assert cfg.formation.spacing_m == 10.0
    assert cfg.mission.cruise_alt_m == 30.0 and cfg.mission.cruise_speed_mps == 5.0
    assert cfg.heartbeat.rate_hz == 5.0 and cfg.heartbeat.master_timeout_s == 1.5
    assert cfg.battery.handover_pct == 30.0
    assert cfg.safety.min_separation_m == 5.0
    assert cfg.safety.geofence_radius_m == 1500.0 and cfg.safety.geofence_max_alt_m == 50.0
    assert cfg.setpoints.rate_hz == 20.0
    assert cfg.drone_ids == list(range(1, cfg.swarm.num_drones + 1))
    assert cfg.with_num_drones(10).drone_ids == list(range(1, 11))


@pytest.mark.parametrize("mutate,msg", [
    (lambda d: d["swarm"].update(extra=1), "unknown keys"),
    (lambda d: d["mission"].pop("cruise_alt_m"), "missing keys"),
    (lambda d: d["mission"].update(cruise_alt_m="high"), "expected a number"),
    (lambda d: d["swarm"].update(num_drones=2.5), "expected an integer"),
    (lambda d: d["mission"].update(goal_enu_m=[1.0]), "list of 2"),
    (lambda d: d.update(swarm=[1, 2]), "expected a mapping"),
    (lambda d: d["swarm"].update(num_drones=64), "drone IDs"),
    (lambda d: d["battery"].update(retire_alt_offset_m=25.0), "above geofence"),
    (lambda d: d["formation"].update(orphan_alt_offset_m=3.0), "altitude layers"),
    (lambda d: d["formation"].update(transit_alt_offset_m=-22.0), "too close to the ground"),
    (lambda d: d["formation"].update(transit_exit_m=20.0), "transit_exit_m"),
    (lambda d: d["mission"].update(goal_enu_m=[0.0, 1600.0]), "outside the geofence"),
    (lambda d: d["formation"].update(spacing_m=4.0), "spacing"),
    (lambda d: d["formation"].update(shape="line"), "unsupported formation"),
    (lambda d: d["heartbeat"].update(master_timeout_s=0.3), "two heartbeat periods"),
    (lambda d: d["link_emulator"].update(loss_pct=120.0), "loss_pct"),
])
def test_invalid_configs_rejected(mutate, msg):
    d = copy.deepcopy(raw())
    mutate(d)
    with pytest.raises(ConfigError, match=msg):
        build(d)
