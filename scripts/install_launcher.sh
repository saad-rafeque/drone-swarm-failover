#!/usr/bin/env bash
# Build the "Swarm Control" program at the top of the project folder: double-click it in the Files app
# (user files only, no sudo). No desktop or app-menu icon; icons left by older versions of this script are removed.
set -eu
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v gcc >/dev/null 2>&1; then
  echo "gcc not found: install it (sudo apt install gcc) or start the app with ./start_swarm_control.sh" >&2
  exit 1
fi
gcc -O2 -o "$ROOT/Swarm Control" "$ROOT/scripts/launcher.c"
gio set "$ROOT/Swarm Control" metadata::custom-icon "file://$ROOT/src/swarm_tools/gcs/static/icon.svg" 2>/dev/null || true
chmod +x "$ROOT/scripts/start_swarm.sh" "$ROOT/scripts/stop_swarm.sh"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
for old in "$HOME/.local/share/applications/swarm-control.desktop" "$DESK/Swarm Control.desktop"; do
  if [ -f "$old" ] && grep -q "start_swarm.sh" "$old"; then
    rm "$old"
    echo "Removed old icon: $old"
  fi
done
echo "Installed: $ROOT/Swarm Control"
