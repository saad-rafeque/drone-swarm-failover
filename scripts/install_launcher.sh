#!/usr/bin/env bash
# Put a "Swarm Control" icon on the desktop and in the application menu (user files only, no sudo).
set -eu
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENTRY="[Desktop Entry]
Type=Application
Name=Swarm Control
Comment=Drone swarm ground control: live map, 3-D view, results and handover docs
Exec=bash \"$ROOT/scripts/start_swarm.sh\"
Icon=$ROOT/src/swarm_tools/gcs/static/icon.svg
Terminal=false
Categories=Education;"
APPS="$HOME/.local/share/applications"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
mkdir -p "$APPS"
printf '%s\n' "$ENTRY" > "$APPS/swarm-control.desktop"
chmod +x "$APPS/swarm-control.desktop" "$ROOT/scripts/start_swarm.sh" "$ROOT/scripts/stop_swarm.sh"
if [ -d "$DESK" ]; then
  cp "$APPS/swarm-control.desktop" "$DESK/Swarm Control.desktop"
  chmod +x "$DESK/Swarm Control.desktop"
  gio set "$DESK/Swarm Control.desktop" metadata::trusted true 2>/dev/null || true
fi
echo "Installed: $APPS/swarm-control.desktop${DESK:+ and $DESK/Swarm Control.desktop}"
