#!/usr/bin/env bash
# Start the #43 boot capture as soon as the boot-ready gate's own contract
# probe has finished (so the capture probe is the cloud's only subscriber)
# and NTP has stepped the clock (so latency is not measured across a step).
set -uo pipefail
here=$(dirname "$0")
waited=0
echo "$(cut -d' ' -f1 /proc/uptime) s uptime: waiting for rosmaster-platform-ready to finish"
while [ "$(systemctl show -p ExecMainExitTimestampMonotonic --value rosmaster-platform-ready.service)" = "0" ]; do
  if [ "$(systemctl is-failed rosmaster-platform.service)" = "failed" ]; then
    echo "rosmaster-platform FAILED this boot; not capturing"; systemctl status rosmaster-platform.service --no-pager | tail -5; exit 1
  fi
  [ "$waited" -ge 300 ] && { echo "ready gate did not finish within 150 s"; exit 1; }
  waited=$((waited + 1)); timeout 0.5 tail -f /dev/null
done
echo "$(cut -d' ' -f1 /proc/uptime) s uptime: ready gate finished, result $(systemctl show -p Result --value rosmaster-platform-ready.service)"
waited=0
until [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ] || [ "$waited" -ge 120 ]; do
  waited=$((waited + 1)); timeout 0.5 tail -f /dev/null
done
echo "$(cut -d' ' -f1 /proc/uptime) s uptime: ntp synchronized=$(timedatectl show -p NTPSynchronized --value); starting capture"
"$here/capture.sh" boot --group camera
echo "$(cut -d' ' -f1 /proc/uptime) s uptime: capture exited $?"
