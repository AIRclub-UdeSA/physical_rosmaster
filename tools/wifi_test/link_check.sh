#!/usr/bin/env bash
# Link, throughput, multicast and ROS checks, then the no-load baseline (step A).
# Usage: bash link_check.sh [first_step]   (~3 min; first_step resumes at 1..8)
source "$(dirname "$0")/common.sh"
LOG="$RESULTS/link_check_$(date +%H%M).txt"
exec > >(tee -a "$LOG") 2>&1
FIRST="${1:-1}"
runs() { [ "$1" -ge "$FIRST" ]; }

if runs 1; then
step "1/8 Workstation network"
nmcli -t -f ACTIVE,SSID,CHAN,SIGNAL dev wifi | grep '^yes'
expect "the router's SSID, on a 5 GHz channel (36-48 or 149-161)"
fi

if runs 2; then
step "2/8 Robot link, power and platform"
rssh 'export PATH=$PATH:/usr/sbin
hostname -I
iw dev wlan0 link | grep -E "SSID|freq|signal|bitrate"
iw dev wlan0 get power_save
vcgencmd get_throttled; vcgencmd measure_temp
df -h / | tail -1
echo "platform: $(systemctl is-active rosmaster-platform)"
lsusb -d 2bc5:060f || echo "ASTRA DEPTH FUNCTION MISSING"' || echo ">>> NO SSH TO THE ROBOT"
expect "freq 5xxx, signal better than -65 dBm, Power save: off, throttled=0x0, platform: active"
fi

if runs 3; then
step "3/8 Robot clock"
# Stepping the clock under a running platform makes base_node log a false
# "Wheel encoders timed out" (it recovers), so only correct a real offset.
offset=$(( $(rssh date +%s) - $(date +%s) ))
echo "robot - workstation: ${offset} s"
if [ "${offset#-}" -gt 2 ]; then
  rssh "sudo date -s @$(date +%s)" >/dev/null
  echo "corrected: robot $(rssh date +%T)  workstation $(date +%T)"
fi
expect "0 or +-1 s (a larger offset is corrected)"
fi

if runs 4; then
step "4/8 Ping (50 packets, 10 s)"
ping -c 50 -i 0.2 "$ROBOT" | tail -2
expect "0% packet loss, avg < 10 ms, max < 50 ms"
fi

if runs 5; then
step "5/8 iperf3 (3 x 10 s)"
for args in "" "-R" "-R -u -b 60M"; do
  echo "-- iperf3 $args"
  rssh 'iperf3 -s -1 -D'; sleep 1
  # shellcheck disable=SC2086
  iperf3 -c "$ROBOT" -t 10 $args | tail -4
done
expect "-R (robot to workstation, the sensor direction) well above 55 Mbit/s; UDP loss < 1%"
fi

if runs 6; then
step "6/8 Multicast"
# Wait on the listener's PID only: a bare `wait` also waits for the tee above.
echo "-- robot -> workstation"
(timeout 15 ros2 multicast receive || echo ">>> NOT RECEIVED") &
pid=$!
sleep 2
cexec 'source /opt/ros/humble/setup.bash; ros2 multicast send'
wait $pid
echo "-- workstation -> robot"
(cexec 'source /opt/ros/humble/setup.bash; timeout 15 ros2 multicast receive || echo ">>> NOT RECEIVED"') &
pid=$!
sleep 4
ros2 multicast send
wait $pid
expect "'Received from ...: Hello World!' in both directions"
fi

if runs 7; then
step "7/8 ROS from the workstation"
ros2 daemon stop >/dev/null 2>&1
echo "nodes: $(timeout 20 ros2 node list 2>/dev/null | wc -l)"
expect "nodes: 7 (0 right after the robot changed network: restart the platform)"
timeout 15 ros2 topic echo /voltage --once || echo ">>> NO DATA"
expect "data: battery voltage; charge below ~10.4 V before measuring"
for t in /odom /scan /cam_1/color/image_raw /cam_1/depth/color/points; do
  echo "-- hz $t"
  timeout -s INT 10 ros2 topic hz "$t" 2>&1 | grep -E "average" | tail -1
done
expect "odom ~10, scan ~7.2, image ~30, cloud 7-20"
echo "-- bw image"
timeout -s INT 10 ros2 topic bw /cam_1/color/image_raw 2>&1 | grep -E "B/s" | tail -1
expect "~7 MB/s"
fi

if runs 8; then
step "8/8 Step A: on-robot latency with no remote subscriber (30 s)"
robot_probe A
expect "image ~30 Hz at ~5 ms; cloud at ~50 ms"
fi

echo; echo "Done. Log: $LOG"
