#!/usr/bin/env bash
# Phase 5 on PX4: radio realism sweep of the link emulator (the heartbeats between the agents), N=10.
# Nine conditions: delay 50 / 150 / 300 ms x loss 0 / 10 / 30 %. For each condition, one mission without a
# fault (false leader changes, formation error, separation) and one with the leader killed (F1: takeover
# time, formation recovery). Folders: reports/logs/phase_5/d<delay>_l<loss>_<none|F1>. Missions already
# completed are skipped, so the sweep can run in parts; it waits for the charger like the Phase 4 batch.
# Usage: bash scripts/phase5_runs.sh [first_condition] [last_condition]
#        (conditions 1-9 in the order below; one condition = two missions, about 16 minutes; DRY_RUN=1 prints only)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
source "$ROOT/scripts/batch_power.sh"   # ac_online, power_ok, wait_for_ac
FIRST="${1:-1}"; LAST="${2:-9}"; N="${N:-10}"
CONDITIONS=("50 0" "50 10" "50 30" "150 0" "150 10" "150 30" "300 0" "300 10" "300 30")
mkdir -p reports/logs/phase_5
for k in $(seq "$FIRST" "$LAST"); do
  read -r delay loss <<< "${CONDITIONS[$((k - 1))]}"
  for kind in none F1; do
    d="reports/logs/phase_5/d${delay}_l${loss}_${kind}"
    if grep -q '"result": "COMPLETED"' "$d/run_summary.json" 2>/dev/null; then
      echo "=== $d already done"; continue
    fi
    fault=(); [ "$kind" = F1 ] && fault=(--fault F1 --fault-seed "$k")
    wait_for_ac
    echo "=== $d start $(date --iso-8601=seconds) (power profile: $(powerprofilesctl get 2>/dev/null || echo unknown))"
    if [ -n "${DRY_RUN:-}" ]; then
      echo "DRY_RUN: run_mission.py --n $N --run-dir $d --latency-ms $delay --loss-pct $loss ${fault[*]}"; continue
    fi
    scripts/ros_env.sh python3 scripts/run_mission.py --n "$N" --run-dir "$d" --latency-ms "$delay" \
      --loss-pct "$loss" "${fault[@]}" > "$d.out" 2>&1
    rc=$?
    [ -f "$d/states.jsonl" ] && gzip -f "$d/states.jsonl"
    echo "=== $d exit $rc $(date --iso-8601=seconds)"
  done
done
echo "=== all done $(date --iso-8601=seconds)"
