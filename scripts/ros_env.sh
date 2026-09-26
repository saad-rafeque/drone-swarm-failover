#!/usr/bin/env bash
# Run a command with ROS 2 Jazzy sourced and the repo's src/ on PYTHONPATH.
# Usage: scripts/ros_env.sh python3 scripts/phase0_flight_test.py
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$@"
