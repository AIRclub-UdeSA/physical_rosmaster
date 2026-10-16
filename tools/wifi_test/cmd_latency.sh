#!/usr/bin/env bash
# /cmd_vel latency from the workstation, logged to the results directory.
# WHEELS LIFTED. Usage: bash cmd_latency.sh <label>   (for example: bash cmd_latency.sh D)
source "$(dirname "$0")/common.sh"
LABEL="${1:?usage: bash cmd_latency.sh <label>}"
exec > >(tee -a "$RESULTS/cmd_${LABEL}_$(date +%H%M).txt") 2>&1
step "cmd_vel under load $LABEL (5 pulses of 1.5 s, ~20 s)"
python3 "$WIFI_TEST_DIR/cmd_latency.py" --confirm-wheels-lifted
expect "never DID NOT STOP; compare against the same robot with no remote load"
