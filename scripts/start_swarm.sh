#!/usr/bin/env bash
# One click: start the swarm ground-control app (fast simulator, maps, 3-D view, results, docs) and open it.
# Used by the "Swarm Control" desktop icon (scripts/install_launcher.sh). Safe to run twice.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URL="http://localhost:8080"
LOG="${XDG_RUNTIME_DIR:-/tmp}/swarm_gcs.log"
PIDFILE="${XDG_RUNTIME_DIR:-/tmp}/swarm_gcs.pid"
if ! curl -s --max-time 2 "$URL/api/state" >/dev/null 2>&1; then
  PY=python3
  [ -x "$ROOT/.venv/bin/python" ] && PY="$ROOT/.venv/bin/python"
  cd "$ROOT" && nohup env PYTHONPATH="$ROOT/src" "$PY" "$ROOT/scripts/gcs.py" --port 8080 >"$LOG" 2>&1 &
  echo $! >"$PIDFILE"
  for _ in $(seq 1 40); do curl -s --max-time 1 "$URL/api/state" >/dev/null 2>&1 && break; sleep 0.5; done
fi
[ -n "${SWARM_NO_BROWSER:-}" ] || xdg-open "$URL" >/dev/null 2>&1 &
