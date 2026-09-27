#!/usr/bin/env bash
# Phase 4 acceptance batch: rounds FIRST..10 of the five faults (resumes; finished trials are skipped),
# then (rule 3a) kill every sim process and run one more round (fault seed 11) from a clean shell as the
# cross-check. Output: reports/logs/phase_4/F<k>_t<round> and reports/logs/phase_4_crosscheck/F<k>_t11.
# Usage: bash scripts/phase4_all.sh [first_round]
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
FIRST="${1:-1}"
bash scripts/phase4_runs.sh "$FIRST" 10 phase_4
pkill -x px4; pkill -x mavros_node
sleep 3
echo "=== cross-check from a clean shell $(date --iso-8601=seconds)"
env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin bash --noprofile --norc -c '
cd "'"$ROOT"'" || exit 1
bash scripts/phase4_runs.sh 11 11 phase_4_crosscheck'
echo "=== phase 4 batch finished $(date --iso-8601=seconds)"
