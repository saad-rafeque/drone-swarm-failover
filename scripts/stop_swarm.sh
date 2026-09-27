#!/usr/bin/env bash
# Stop the ground-control app started by start_swarm.sh (the Swarm Control launchers).
# Uses the saved process ID when it still belongs to scripts/gcs.py; otherwise looks for Python processes running
# scripts/gcs.py. (Only processes named python* are considered, so a shell whose command line merely mentions the
# script is never hit.)
PIDFILE="${XDG_RUNTIME_DIR:-/tmp}/swarm_gcs.pid"
is_app() { [ "$(cut -c1-6 "/proc/$1/comm" 2>/dev/null)" = "python" ] && tr '\0' ' ' <"/proc/$1/cmdline" 2>/dev/null | grep -q "scripts/gcs.py"; }
pids=""
if [ -f "$PIDFILE" ] && is_app "$(cat "$PIDFILE")"; then pids="$(cat "$PIDFILE")"; fi
if [ -z "$pids" ]; then
  for p in $(pgrep '^python'); do is_app "$p" && pids="$pids $p"; done
fi
rm -f "$PIDFILE"
if [ -z "$pids" ]; then echo "not running"; exit 0; fi
kill $pids 2>/dev/null
for _ in $(seq 1 50); do
  alive=""
  for p in $pids; do kill -0 "$p" 2>/dev/null && alive=1; done
  if [ -z "$alive" ]; then echo "stopped"; exit 0; fi
  sleep 0.1
done
echo "still running (process$pids); stop it with: kill -9$pids"
exit 1
