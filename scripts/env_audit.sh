#!/usr/bin/env bash
# Phase 0 environment audit. Prints raw command output (evidence for reports/PHASE_0.md).
# Usage: scripts/env_audit.sh > reports/logs/phase_0/env_audit.txt
set -u

section() { printf '\n===== %s =====\n' "$1"; }

section "date"
date --iso-8601=seconds

section "OS"
grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release
uname -srm

section "ROS 2"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u   # ROS setup scripts reference unset variables
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
ls /opt/ros 2>&1

section "Python"
python3 --version
for m in numpy yaml pytest pytest_cov coverage matplotlib scipy pymavlink; do
  python3 -c "import $m; print('$m', getattr($m, '__version__', 'ok'))" 2>/dev/null || echo "$m MISSING"
done

section "RAM (free -m)"
free -m

section "CPU"
echo "nproc=$(nproc)"
lscpu | grep -E '^Model name|^CPU\(s\)|^Thread|^Core'

section "Disk (need >= 15 GB free)"
df -h "$HOME"

section "Build tools"
for t in git cmake make ninja gcc g++ ccache colcon rosdep; do
  printf '%-8s %s\n' "$t" "$(command -v "$t" || echo MISSING)"
done

section "MAVROS / GeographicLib"
dpkg -l 2>/dev/null | awk '/ros-jazzy-mavros/{print $2, $3}' | grep . || echo "ros-jazzy-mavros NOT installed"
ls /usr/share/GeographicLib 2>/dev/null || echo "GeographicLib datasets NOT found in /usr/share/GeographicLib"

section "PX4"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [ -d "$PX4_DIR/.git" ]; then
  git -C "$PX4_DIR" describe --tags --always
  ls "$PX4_DIR/build" 2>/dev/null || echo "no build/ yet"
else
  echo "PX4 not found at $PX4_DIR"
fi

section "Orphan px4/mavros processes"
pgrep -a -x px4 || echo "no px4"
pgrep -af 'mavros_node' || echo "no mavros"
