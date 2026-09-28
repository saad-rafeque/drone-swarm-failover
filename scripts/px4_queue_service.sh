#!/usr/bin/env bash
# The PX4 test queue (scripts/px4_queue.py) as a systemd user service: it starts when the owner logs in,
# continues with the first unfinished trial after a shutdown or restart, and restarts if it crashes. It runs
# as the logged-in user (no sudo) and only on the charger. Pause and resume on the "PX4 tests" page of the
# ground-control app, or with `python3 scripts/px4_queue.py pause|resume`.
# Usage: bash scripts/px4_queue_service.sh install | remove | status | log
set -eu
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME=swarm-px4-queue.service
UNIT="$HOME/.config/systemd/user/$NAME"
case "${1:-status}" in
  install)
    mkdir -p "$(dirname "$UNIT")"
    cat > "$UNIT" <<EOF
[Unit]
Description=Swarm Failover PX4 test queue (pause and resume on the PX4 tests page of Swarm Control)

[Service]
Type=simple
WorkingDirectory=$ROOT
ExecStart=/usr/bin/python3 "$ROOT/scripts/px4_queue.py" run
Restart=on-failure
RestartSec=60

[Install]
WantedBy=default.target
EOF
    systemctl --user daemon-reload
    systemctl --user enable --now "$NAME"
    echo "installed and started: $UNIT"
    ;;
  remove)
    systemctl --user disable --now "$NAME" 2>/dev/null || true
    rm -f "$UNIT"
    systemctl --user daemon-reload
    echo "removed $UNIT (finished trials stay in reports/logs/)"
    ;;
  status) systemctl --user status "$NAME" --no-pager || true ;;
  log) tail -n 30 "$ROOT/reports/logs/px4_queue/runner.log" ;;
  *) echo "usage: $0 install | remove | status | log"; exit 2 ;;
esac
