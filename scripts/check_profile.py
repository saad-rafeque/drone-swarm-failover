#!/usr/bin/env python3
"""Check a drone profile (config/profiles/*.yaml) against config/swarm.yaml and print every drone's link.

Real drones are checked as text only (URL format, baud rate, system ID); this script never opens a serial
port, and the simulation tools refuse to start a profile that has a real drone.

Usage: PYTHONPATH=src python3 scripts/check_profile.py config/profiles/mixed.yaml [--n 10]
Exit code 0 when the profile is valid, 1 otherwise.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.profiles import ProfileError, load_profile  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("profile")
    ap.add_argument("--n", type=int, help="number of drones (default: config/swarm.yaml)")
    args = ap.parse_args()
    cfg = load_config(default_config_path())
    if args.n:
        cfg = cfg.with_num_drones(args.n)
    try:
        prof = load_profile(args.profile, cfg)
    except ProfileError as exc:
        print(f"INVALID: {exc}")
        return 1
    print(f"profile '{prof.name}': {prof.description}")
    for d in prof.drones:
        extra = "  (checked as text only; never opened)" if d.link == "real" else ""
        print(f"  drone {d.drone_id:>2}  {d.link:<8} {d.fcu_url}{extra}{'  - ' + d.note if d.note else ''}")
    if prof.real:
        print(f"VALID. Real drones {prof.real}: the simulation tools refuse to start this profile (simulation only).")
    else:
        print("VALID. The simulation tools can start this profile"
              + (f"; drones {prof.standin} run behind the telemetry-radio stand-in." if prof.standin else "."))
    return 0


if __name__ == "__main__":
    sys.exit(main())
