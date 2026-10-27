#!/usr/bin/env bash
# Measures one load step: on-robot camera latency (30 s) with a ping alongside.
# Usage: bash load_step.sh <label>   with RViz already showing that step's displays.
source "$(dirname "$0")/common.sh"
LABEL="${1:?usage: bash load_step.sh <label>}"
LOG="$RESULTS/step_${LABEL}_$(date +%H%M).txt"
exec > >(tee -a "$LOG") 2>&1

step "Step $LABEL (30 s, leave everything as is)"
ping -c 150 -i 0.2 "$ROBOT" > "$RESULTS/ping_$LABEL.txt" &
pid=$!
rssh 'export PATH=$PATH:/usr/sbin; iw dev wlan0 link | grep -E "signal|tx bitrate"'
robot_probe "$LABEL"
wait $pid
tail -2 "$RESULTS/ping_$LABEL.txt"
rssh 'vcgencmd get_throttled; systemctl is-active rosmaster-platform'
expect "cloud latency < 150 ms, no ping loss, throttled=0x0, active"
