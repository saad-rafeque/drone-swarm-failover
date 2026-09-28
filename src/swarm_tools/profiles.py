"""Drone profiles (config/profiles/*.yaml): for every drone, how its MAVROS reaches its autopilot.

Moving a drone from simulation to hardware must only change its connection URL (docs/SPECIFICATION.md §1),
so a profile is exactly that list:
  sim      PX4 SIH on this computer, MAVROS over UDP (PX4's port plan, swarm_tools.sim_launch)
  standin  simulated, but its MAVROS <-> PX4 link runs through the telemetry-radio stand-in
           (swarm_tools.radio_proxy, Phase 6)
  real     a real flight controller behind a MAVROS serial URL. Checked as text only: nothing in this
           repository opens a serial port (rule 7), and the simulation tools refuse to start such a profile.

URL formats: MAVROS docs/connection_urls.md, "serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]"
and "serial-hwfc://..." (hardware flow control).
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from swarm_agent.config import Config
from swarm_tools.radio_proxy import standin_fcu_url
from swarm_tools.sim_launch import instance_of, sim_fcu_url, sysid_of

LINKS = ("sim", "standin", "real")
SERIAL_URL = re.compile(r"^serial(-hwfc)?://(/dev/[A-Za-z0-9._/-]+):(\d+)(\?ids=(\d+),(\d+))?$")
BAUDS = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}
PROFILE_DIR = Path(__file__).resolve().parents[2] / "config" / "profiles"


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class DroneLink:
    drone_id: int
    link: str
    fcu_url: str
    note: str = ""


@dataclass(frozen=True)
class Profile:
    name: str
    description: str
    drones: tuple[DroneLink, ...]

    def of(self, drone_id: int) -> DroneLink:
        return next(d for d in self.drones if d.drone_id == drone_id)

    @property
    def real(self) -> list[int]:
        return [d.drone_id for d in self.drones if d.link == "real"]

    @property
    def standin(self) -> list[int]:
        return [d.drone_id for d in self.drones if d.link == "standin"]


def _check_real(drone_id: int, entry: dict, where: str) -> str:
    url = str(entry.get("fcu_url", ""))
    m = SERIAL_URL.match(url)
    if not m:
        raise ProfileError(f"{where}: fcu_url {url!r} is not a MAVROS serial URL (serial:///dev/<device>:<baud>)")
    if int(m.group(3)) not in BAUDS:
        raise ProfileError(f"{where}: baud rate {m.group(3)} is not one of {sorted(BAUDS)}")
    if entry.get("mav_sys_id") != drone_id:
        raise ProfileError(f"{where}: mav_sys_id must be set and equal the swarm ID {drone_id} "
                           "(the real drone's PX4 MAV_SYS_ID)")
    if m.group(5) is not None and int(m.group(5)) != drone_id:
        raise ProfileError(f"{where}: ?ids= system ID must equal the swarm ID {drone_id}")
    return url


def load_profile(path: str | Path, cfg: Config) -> Profile:
    """Validate a profile against the configured swarm (every drone ID of cfg gets a link)."""
    path = Path(path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or set(raw) != {"name", "description", "default", "drones"}:
        raise ProfileError(f"{path.name}: expected exactly the keys name, description, default, drones")
    default = raw["default"] or {}
    if default.get("link") not in ("sim", "standin") or set(default) != {"link"}:
        raise ProfileError(f"{path.name}: default must be {{link: sim}} or {{link: standin}}")
    entries = raw["drones"] or {}
    unknown = sorted(set(entries) - set(cfg.drone_ids))
    if unknown:
        raise ProfileError(f"{path.name}: drones {unknown} are not in this swarm (IDs {cfg.drone_ids[0]}.."
                           f"{cfg.drone_ids[-1]})")
    drones = []
    for did in cfg.drone_ids:
        entry = entries.get(did, default)
        where = f"{path.name}, drone {did}"
        link = entry.get("link")
        if link not in LINKS:
            raise ProfileError(f"{where}: link must be one of {LINKS}")
        allowed = {"link", "note"} | ({"fcu_url", "mav_sys_id"} if link == "real" else set())
        if set(entry) - allowed:
            raise ProfileError(f"{where}: unexpected keys {sorted(set(entry) - allowed)}")
        if link == "real":
            url = _check_real(did, entry, where)
        elif link == "standin":
            url = standin_fcu_url(cfg.radio_standin, instance_of(cfg, did))
        else:
            url = sim_fcu_url(cfg, did)
        if link != "real" and sysid_of(cfg, did) != did:
            raise ProfileError(f"{where}: PX4 SIH gives this drone MAV_SYS_ID {sysid_of(cfg, did)}, not {did}")
        drones.append(DroneLink(did, link, url, str(entry.get("note", ""))))
    return Profile(str(raw["name"]), str(raw["description"]), tuple(drones))
