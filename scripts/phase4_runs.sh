#!/usr/bin/env bash
# Phase 4: 10 trials per fault (F1..F5) at N=10, interleaved by round so partial results cover
# every fault. Trial folders: reports/logs/phase_4/<fault>_t<round>. Existing completed trials
# (run_summary.json with "COMPLETED") are skipped, so the batch can be resumed.
# Usage: scripts/phase4_runs.sh [first_round] [last_round] [out_subdir]   (DRY_RUN=1: print the commands only)
# Options (environment): N (drones, default 10), FAULTS (default "F1 F2 F3 F4 F5"), PROFILE (a drone profile,
# for example config/profiles/standin.yaml for the Phase 6 radio stand-in; see scripts/phase6_runs.sh).
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
FIRST="${1:-1}"; LAST="${2:-10}"; SUB="${3:-phase_4}"; N="${N:-10}"
FAULTS="${FAULTS:-F1 F2 F3 F4 F5}"; PROFILE="${PROFILE:-}"
mkdir -p "reports/logs/$SUB"
source "$ROOT/scripts/batch_power.sh"   # ac_online, power_ok, wait_for_ac
for r in $(seq "$FIRST" "$LAST"); do
  for f in $FAULTS; do
    d="reports/logs/$SUB/${f}_t${r}"
    if grep -q '"result": "COMPLETED"' "$d/run_summary.json" 2>/dev/null; then
      echo "=== $f round $r already done"; continue
    fi
    wait_for_ac
    echo "=== $f round $r start $(date --iso-8601=seconds) (power profile: $(powerprofilesctl get 2>/dev/null || echo unknown))"
    if [ -n "${DRY_RUN:-}" ]; then
      echo "DRY_RUN: run_mission.py --n $N --run-dir $d --fault $f --fault-seed $r ${PROFILE:+--profile $PROFILE}"; continue
    fi
    scripts/ros_env.sh python3 scripts/run_mission.py --n "$N" --run-dir "$d" --fault "$f" --fault-seed "$r" \
      ${PROFILE:+--profile "$PROFILE"} > "$d.out" 2>&1
    rc=$?
    [ -f "$d/states.jsonl" ] && gzip -f "$d/states.jsonl"   # ~16 MB -> ~1.5 MB; the metric scripts read .gz
    echo "=== $f round $r exit $rc $(date --iso-8601=seconds)"
  done
done
echo "=== all done $(date --iso-8601=seconds)"
