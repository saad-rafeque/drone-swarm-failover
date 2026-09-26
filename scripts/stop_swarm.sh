#!/usr/bin/env bash
# Stop the ground-control app started by start_swarm.sh.
PIDFILE="${XDG_RUNTIME_DIR:-/tmp}/swarm_gcs.pid"
if [ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null; then echo "stopped"; rm -f "$PIDFILE"; else echo "not running"; fi
