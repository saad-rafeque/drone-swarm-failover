#!/usr/bin/env bash
# Quick start in one command: installs the Python packages into .venv, checks the install with the test suite,
# then starts the ground-control app (fast simulator, maps, 3-D view, results, documents) and opens it in the
# browser. No sudo, no ROS 2, no PX4: those are needed only for the PX4 flights (docs/RUNBOOK.md, section 1).
# Usage: ./setup.sh [--no-tests] [--no-start]      Safe to run again. Stop the app with scripts/stop_swarm.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

RUN_TESTS=1
START=1
for arg in "$@"; do
  case "$arg" in
    --no-tests) RUN_TESTS=0 ;;
    --no-start) START=0 ;;
    -h|--help) sed -n '2,5p' "$0" | cut -c3-; exit 0 ;;
    *) echo "unknown option: $arg (see ./setup.sh --help)" >&2; exit 2 ;;
  esac
done

step() { printf '\n==> %s\n' "$*"; }
fail() { printf '\nSetup stopped: %s\n' "$*" >&2; exit 1; }

step "1/5 Python"
PY="${PYTHON:-python3}"
command -v "$PY" >/dev/null 2>&1 || fail "python3 was not found. Install Python 3.12 or newer (Ubuntu: sudo apt install python3 python3-venv)."
"$PY" -c 'import sys; sys.exit(sys.version_info < (3, 12))' || fail "Python 3.12 or newer is needed; found $("$PY" --version 2>&1)."
"$PY" --version

step "2/5 Python packages (into .venv in this folder; nothing is installed system-wide)"
if [ ! -x .venv/bin/python ]; then
  "$PY" -m venv .venv || fail "could not create .venv. On Ubuntu or Debian: sudo apt install python3-venv"
fi
.venv/bin/python -m pip install --quiet --disable-pip-version-check -r requirements.txt \
  || fail "installing the packages failed (see the messages above; an internet connection is needed)."
.venv/bin/python -c 'import numpy, scipy, matplotlib, yaml, psutil, pytest; print("numpy", numpy.__version__, "| scipy", scipy.__version__, "| matplotlib", matplotlib.__version__)'

step "3/5 Tests"
if [ "$RUN_TESTS" = 1 ]; then
  # tests that need ROS 2 or PyTorch are skipped automatically when those are not installed
  .venv/bin/python -m pytest -q -x || fail "a test failed (see above). Please report it with the output."
else
  echo "skipped (--no-tests)"
fi

step "4/5 Map keys (optional)"
if [ ! -f config/map_keys.local.yaml ]; then
  cp config/map_keys.example.yaml config/map_keys.local.yaml
  echo "Created config/map_keys.local.yaml with empty keys. The 2-D map works without them (OpenStreetMap)."
  echo "For satellite imagery and the 3-D view, paste your own free Mapbox and Cesium ion tokens into that file."
else
  echo "config/map_keys.local.yaml is already there; left unchanged."
fi

step "5/5 Ground-control app"
if [ "$START" = 1 ]; then
  bash scripts/start_swarm.sh
  echo "Running at http://localhost:8080 (it opens in your browser). Stop it with: bash scripts/stop_swarm.sh"
else
  echo "not started (--no-start). Start it with: bash scripts/start_swarm.sh"
fi
printf '\nDone. PX4 flights need the full installation: docs/RUNBOOK.md, section 1.\n'
