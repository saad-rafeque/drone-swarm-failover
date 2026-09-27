#!/usr/bin/env bash
# Double-click (or right-click -> "Run as a Program") to start the whole Swarm Control app:
# live mission map, 3-D view, results, PX4 flight replays and the handover documents.
# It starts the app in the background (if it is not already running) and opens it in the browser.
# Stop it with scripts/stop_swarm.sh. Details: README.md, section "Getting started".
exec bash "$(dirname "$(readlink -f "$0")")/scripts/start_swarm.sh"
