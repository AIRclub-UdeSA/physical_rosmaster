#!/usr/bin/env bash
# Wait until the platform has run 10+ minutes, then take three settled runs.
set -uo pipefail
start_us=$(systemctl show -p ExecMainStartTimestampMonotonic --value rosmaster-platform.service)
until awk -v u="$(cut -d" " -f1 /proc/uptime)" -v t="$start_us" "BEGIN { exit !(u - t / 1e6 >= 615) }"; do
  timeout 1 tail -f /dev/null
done
for run in 1 2 3; do
  echo "$(cut -d" " -f1 /proc/uptime) s uptime: settled_$run starting"
  ~/probe-pr44/capture.sh settled_$run --group camera > /dev/null 2>&1
  echo "$(cut -d" " -f1 /proc/uptime) s uptime: settled_$run exited $?"
done
