#!/usr/bin/env bash
# Phase 3 acceptance: 3 missions at N=10, then (rule 3a) kill all sim processes and repeat the
# 3 missions from a clean shell. Output: reports/logs/phase_3/run{1,2,3} and crosscheck{1,2,3}.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
N="${N:-10}"
for i in 1 2 3; do
  echo "=== run$i start $(date --iso-8601=seconds)"
  scripts/ros_env.sh python3 scripts/run_mission.py --n "$N" --run-dir "reports/logs/phase_3/run$i" > "reports/logs/phase_3/run$i.out" 2>&1
  echo "=== run$i exit $?"
done
pkill -x px4; pkill -x mavros_node
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin N="$N" bash --noprofile --norc -c '
cd "'"$ROOT"'" || exit 1
for i in 1 2 3; do
  echo "=== crosscheck$i start $(date --iso-8601=seconds)"
  scripts/ros_env.sh python3 scripts/run_mission.py --n "$N" --run-dir "reports/logs/phase_3/crosscheck$i" > "reports/logs/phase_3/crosscheck$i.out" 2>&1
  echo "=== crosscheck$i exit $?"
done'
echo "=== all done $(date --iso-8601=seconds)"
