#!/usr/bin/env bash
# Run one #43 measurement condition on x3-c, recording host-side conditions
# the in-container probe cannot see. Usage:
#   capture.sh <condition> <probe topic args...>
set -uo pipefail
condition="$1"; shift
container=rosmaster_humble
repo=/root/yahboomcar_ws/src/physical_rosmaster

mono() { systemctl show -p "$2" --value "$1"; }
uptime_s=$(cut -d' ' -f1 /proc/uptime)
platform_start_us=$(mono rosmaster-platform.service ExecMainStartTimestampMonotonic)
ready_exit_us=$(mono rosmaster-platform-ready.service ExecMainExitTimestampMonotonic)
age() { awk -v u="$uptime_s" -v t="$1" 'BEGIN { if (t > 0) printf "%.1f", u - t / 1e6; else print "none" }'; }
depth_path=""; for d in /sys/bus/usb/devices/*; do
  [ "$(cat "$d/idVendor" 2>/dev/null):$(cat "$d/idProduct" 2>/dev/null)" = "2bc5:060f" ] && depth_path=$(basename "$d")
done

notes=(
  "condition=$condition"
  "commit=$(docker exec $container git -C $repo rev-parse --short HEAD)"
  "worktree_dirty_files=$(docker exec $container git -C $repo status --porcelain | wc -l)"
  "platform_active=$(systemctl is-active rosmaster-platform.service)"
  "platform_age_s_at_start=$(age "$platform_start_us")"
  "ready_result=$(mono rosmaster-platform-ready.service Result)"
  "ready_exited_s_before_start=$(age "$ready_exit_us")"
  "ntp_synchronized=$(timedatectl show -p NTPSynchronized --value)"
  "astra_depth_usb_path=${depth_path:-absent}"
  "usb_disconnects_this_boot=$(journalctl -k -b 0 --no-pager | grep -ciE 'usb disconnect|clear tt')"
  "throttled=$(vcgencmd get_throttled | cut -d= -f2)"
  "soc_temp_c=$(vcgencmd measure_temp | grep -oE '[0-9.]+')"
  "stationary=yes"
)
note_args=""; for n in "${notes[@]}"; do printf -v quoted '%q' "$n"; note_args+=" --note $quoted"; done

docker exec $container bash -lc "source /opt/ros/humble/setup.bash && source /root/yahboomcar_ws/install/setup.bash && cd /root/probe-pr44 && python3 sensor_capability_probe.py $* --topic /voltage --duration 40 --per-message $note_args --output out/camera_${condition}.json"
