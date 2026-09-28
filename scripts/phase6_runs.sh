#!/usr/bin/env bash
# Phase 6 hardware stand-in test on PX4: faults F1 and F2 with drone 1 (the leader) behind the telemetry-radio
# stand-in (config/profiles/standin.yaml), 10 trials each, at N=10. Trial folders: reports/logs/phase_6/F<k>_t<n>.
# Resumable and charger-aware like the Phase 4 batch it runs (scripts/phase4_runs.sh).
# Usage: bash scripts/phase6_runs.sh [first_round] [last_round]      (one round = F1 + F2, about 16 minutes)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAULTS="F1 F2" PROFILE=config/profiles/standin.yaml exec bash "$ROOT/scripts/phase4_runs.sh" "${1:-1}" "${2:-10}" phase_6
