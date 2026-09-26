#!/usr/bin/env python3
"""Start the swarm ground-control web app.

Usage: python3 scripts/gcs.py [--port 8080]
Then open http://localhost:8080 in a browser (the map needs internet for its tiles).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_agent.config import default_config_path, load_config  # noqa: E402
from swarm_tools.gcs.fastsim_backend import FastSimBackend  # noqa: E402
from swarm_tools.gcs.server import serve  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    serve(FastSimBackend(load_config(default_config_path())), args.port)


if __name__ == "__main__":
    main()
