#!/usr/bin/env bash
# Send a push notification to your phone via ntfy.sh
# Usage:
#   scripts/notify.sh "Title" "Message" [priority]    # priority: min|low|default|high|urgent
#   scripts/notify.sh --hook                           # used by editor notification hooks (reads JSON on stdin)
# Topic is read from $NTFY_TOPIC or ~/.config/swarm-ntfy/topic (kept OUT of the repo).
set -u

CONF="${HOME}/.config/swarm-ntfy/topic"
TOPIC="${NTFY_TOPIC:-}"
if [ -z "$TOPIC" ] && [ -f "$CONF" ]; then TOPIC="$(tr -d '[:space:]' < "$CONF")"; fi
if [ -z "$TOPIC" ]; then
  echo "notify: no topic set (create $CONF)" >&2
  exit 0   # never block the caller
fi

if [ "${1:-}" = "--hook" ]; then
  PAYLOAD="$(cat)"
  MSG="$(printf '%s' "$PAYLOAD" | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
    print(d.get("message") or d.get("notification_type") or "Action needed")
except Exception:
    print("Action needed")
')"
  TITLE="Swarm project - action needed"
  PRIO="high"
else
  TITLE="${1:-Swarm project}"
  MSG="${2:-(no message)}"
  PRIO="${3:-default}"
fi

curl -s -m 10 \
  -H "Title: ${TITLE}" \
  -H "Priority: ${PRIO}" \
  -d "${MSG}" \
  "https://ntfy.sh/${TOPIC}" > /dev/null || echo "notify: send failed" >&2
exit 0
