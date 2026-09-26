#!/usr/bin/env bash
# Build PX4 SITL (SIH) with at most 2 parallel jobs (docs/SPECIFICATION.md rule 6).
# PX4 dir and build target come from config/swarm.yaml (sim.px4_dir, sim.build_target).
set -eu
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
read -r PX4_DIR TARGET < <(python3 -c '
import os, sys, yaml
sim = yaml.safe_load(open(sys.argv[1]))["sim"]
print(os.path.expanduser(sim["px4_dir"]), sim["build_target"])
' "$ROOT/config/swarm.yaml")

cd "$PX4_DIR"
# PX4's Makefile reads "-jN" from `ps T`, which finds nothing without a terminal, and then
# ninja falls back to nproc+2 jobs. Passing j=2 sets the Makefile variable explicitly.
make "$TARGET" j=2
