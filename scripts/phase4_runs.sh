#!/usr/bin/env bash
# Phase 4: 10 trials per fault (F1..F5) at N=10, interleaved by round so partial results cover
# every fault. Trial folders: reports/logs/phase_4/<fault>_t<round>. Existing completed trials
# (run_summary.json with "COMPLETED") are skipped, so the batch can be resumed.
# Usage: scripts/phase4_runs.sh [first_round] [last_round] [out_subdir]   (DRY_RUN=1: print the commands only)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
FIRST="${1:-1}"; LAST="${2:-10}"; SUB="${3:-phase_4}"; N="${N:-10}"
mkdir -p "reports/logs/$SUB"
ac_online() {
  local ps   # local: the trial loop below uses $f for the fault name
  for ps in /sys/class/power_supply/*/online; do
    [ "$(cat "$(dirname "$ps")/type" 2>/dev/null)" = "Mains" ] && [ "$(cat "$ps")" = "1" ] && return 0
  done
  return 1
}
power_ok() { ac_online && [ "$(powerprofilesctl get 2>/dev/null)" != "power-saver" ]; }
wait_for_ac() {   # on battery or in power-saver mode the CPU is throttled and saturates with 10 drones
  power_ok && return
  echo "=== waiting for the charger and a Balanced/Performance power mode $(date --iso-8601=seconds)"
  until power_ok; do sleep 60; done
  echo "=== power ok $(date --iso-8601=seconds)"
  sleep 30
}
for r in $(seq "$FIRST" "$LAST"); do
  for f in F1 F2 F3 F4 F5; do
    d="reports/logs/$SUB/${f}_t${r}"
    if grep -q '"result": "COMPLETED"' "$d/run_summary.json" 2>/dev/null; then
      echo "=== $f round $r already done"; continue
    fi
    wait_for_ac
    echo "=== $f round $r start $(date --iso-8601=seconds) (power profile: $(powerprofilesctl get 2>/dev/null || echo unknown))"
    if [ -n "${DRY_RUN:-}" ]; then
      echo "DRY_RUN: run_mission.py --n $N --run-dir $d --fault $f --fault-seed $r"; continue
    fi
    scripts/ros_env.sh python3 scripts/run_mission.py --n "$N" --run-dir "$d" --fault "$f" --fault-seed "$r" \
      > "$d.out" 2>&1
    rc=$?
    [ -f "$d/states.jsonl" ] && gzip -f "$d/states.jsonl"   # ~16 MB -> ~1.5 MB; the metric scripts read .gz
    echo "=== $f round $r exit $rc $(date --iso-8601=seconds)"
  done
done
echo "=== all done $(date --iso-8601=seconds)"
