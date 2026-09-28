"""Drone profiles (config/profiles/*.yaml): every drone gets a link, real drones are checked as text only, and
the simulation tools refuse to start a profile that has a real drone (docs/SPECIFICATION.md, rule 7)."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from swarm_tools.profiles import PROFILE_DIR, ProfileError, load_profile
from swarm_tools.sim_launch import SimLauncher

REPO = Path(__file__).resolve().parents[1]


def write(tmp_path: Path, body: str) -> Path:
    p = tmp_path / "profile.yaml"
    p.write_text("name: t\ndescription: test\ndefault: {link: sim}\n" + body, encoding="utf-8")
    return p


def test_the_three_profiles(cfg):
    sim = load_profile(PROFILE_DIR / "sim.yaml", cfg)
    standin = load_profile(PROFILE_DIR / "standin.yaml", cfg)
    mixed = load_profile(PROFILE_DIR / "mixed.yaml", cfg)
    assert [d.drone_id for d in sim.drones] == cfg.drone_ids and not sim.real and not sim.standin
    assert sim.of(1).fcu_url == "udp://:14540@127.0.0.1:14580"
    assert standin.standin == [1] and standin.of(1).fcu_url == "udp://:15540@127.0.0.1:15580"
    assert standin.of(2).fcu_url == sim.of(2).fcu_url
    assert mixed.real == [1] and mixed.of(1).fcu_url == "serial:///dev/ttyUSB0:57600"


def test_simulation_tools_refuse_a_real_drone_and_use_the_profile_urls(cfg):
    with pytest.raises(RuntimeError, match="never connect to a real flight controller"):
        SimLauncher(cfg, {}, profile=load_profile(PROFILE_DIR / "mixed.yaml", cfg))
    sim = SimLauncher(cfg, {}, profile=load_profile(PROFILE_DIR / "standin.yaml", cfg))
    assert sim.fcu_url(1) == "udp://:15540@127.0.0.1:15580" and sim.fcu_url(3) == "udp://:14542@127.0.0.1:14582"


@pytest.mark.parametrize("entry, message", [
    ('1: {link: real, fcu_url: "/dev/ttyUSB0:57600", mav_sys_id: 1}', "not a MAVROS serial URL"),
    ('1: {link: real, fcu_url: "serial:///dev/ttyUSB0:12345", mav_sys_id: 1}', "baud rate"),
    ('1: {link: real, fcu_url: "serial:///dev/ttyUSB0:57600"}', "mav_sys_id"),
    ('2: {link: real, fcu_url: "serial:///dev/ttyUSB0:57600?ids=1,1", mav_sys_id: 2}', "ids= system ID"),
    ("12: {link: sim}", "not in this swarm"),
    ('3: {link: sim, fcu_url: "udp://:1@127.0.0.1:2"}', "unexpected keys"),
    ("2: {link: radio}", "link must be one of"),
])
def test_invalid_profiles_are_rejected(cfg, tmp_path, entry, message):
    with pytest.raises(ProfileError, match=message):
        load_profile(write(tmp_path, "drones:\n  " + entry + "\n"), cfg)


def test_a_real_drone_cannot_be_the_default(cfg, tmp_path):
    p = tmp_path / "p.yaml"
    p.write_text("name: t\ndescription: t\ndefault: {link: real}\ndrones: {}\n", encoding="utf-8")
    with pytest.raises(ProfileError, match="default must be"):
        load_profile(p, cfg)


def test_check_profile_script(tmp_path):
    ok = subprocess.run([sys.executable, "scripts/check_profile.py", "config/profiles/mixed.yaml"], cwd=REPO,
                        capture_output=True, text=True)
    assert ok.returncode == 0 and "never opened" in ok.stdout and "refuse to start" in ok.stdout
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: b\ndescription: b\ndefault: {link: sim}\ndrones:\n  1: {link: real}\n", encoding="utf-8")
    res = subprocess.run([sys.executable, "scripts/check_profile.py", str(bad)], cwd=REPO, capture_output=True,
                         text=True)
    assert res.returncode == 1 and res.stdout.startswith("INVALID")


def test_no_code_opens_serial_ports():
    """Rule 7: no module imports a serial library or opens a /dev/tty* device."""
    for f in list((REPO / "src").rglob("*.py")) + list((REPO / "scripts").glob("*.py")):
        text = f.read_text(encoding="utf-8")
        assert "import serial" not in text and "open(\"/dev/tty" not in text and "open('/dev/tty" not in text, f
