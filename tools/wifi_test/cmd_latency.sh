#!/usr/bin/env bash
# /cmd_vel latency from the workstation, logged to the results directory.
# Moves the wheels: only with the robot lifted, and say so explicitly.
# Usage: WHEELS_LIFTED=yes bash cmd_latency.sh <label>   (for example: WHEELS_LIFTED=yes bash cmd_latency.sh D)
source "$(dirname "$0")/common.sh"
LABEL="${1:?usage: WHEELS_LIFTED=yes bash cmd_latency.sh <label>}"
if [ "${WHEELS_LIFTED:-}" != yes ]; then
  echo ">>> This moves the wheels. Lift the robot, then rerun with WHEELS_LIFTED=yes." >&2
  exit 1
fi
exec > >(tee -a "$RESULTS/cmd_${LABEL}_$(date +%H%M).txt") 2>&1
step "cmd_vel under load $LABEL (5 pulses of 1.5 s, ~20 s)"
echo "   robot: $ROBOT   ROS_DOMAIN_ID: $ROS_DOMAIN_ID (the robot on this domain moves)"
python3 "$WIFI_TEST_DIR/cmd_latency.py" --confirm-wheels-lifted
status=$?
expect "all 5 pulses counted, never DID NOT STOP or /odom STALE; compare against the same robot with no remote load"
exit "$status"
