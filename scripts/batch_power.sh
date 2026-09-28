# Shared by the PX4 batch scripts (sourced): wait for the charger and a power profile other than power saver.
# On battery or in power-saver mode the CPU is throttled and saturates with 10 PX4 drones (reports/PHASE_4.md).
ac_online() {
  local ps   # local: the batch loops use $f for the fault name
  for ps in /sys/class/power_supply/*/online; do
    [ "$(cat "$(dirname "$ps")/type" 2>/dev/null)" = "Mains" ] && [ "$(cat "$ps")" = "1" ] && return 0
  done
  return 1
}
power_ok() { ac_online && [ "$(powerprofilesctl get 2>/dev/null)" != "power-saver" ]; }
wait_for_ac() {   # on battery or in power-saver mode the CPU is throttled and saturates with 10 drones
  power_ok && return
  echo "=== waiting for the charger and a Balanced/Performance power mode $(date --iso-8601=seconds)"
  until power_ok; do sleep 60; done
  echo "=== power ok $(date --iso-8601=seconds)"
  sleep 30
}
