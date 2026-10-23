#!/usr/bin/env bash
# Moves a robot to another saved NetworkManager connection and restarts the
# platform: nodes started on the old network stay undiscoverable on the new one
# (see docs/troubleshooting/known_issues/network-change-leaves-nodes-undiscoverable.md).
# Usage: bash switch_network.sh <robot> <connection-name>
#   e.g. bash switch_network.sh raphael.local router-test
ROBOT="${1:?usage: bash switch_network.sh <robot> <connection-name>}"
CONNECTION="${2:?usage: bash switch_network.sh <robot> <connection-name>}"
# Detached on the robot: SSH drops as soon as the robot leaves this network.
ssh -o ConnectTimeout=8 pi@"$ROBOT" \
  "sudo nohup sh -c 'nmcli connection up \"$CONNECTION\"; sleep 5; systemctl restart rosmaster-platform.service' >/dev/null 2>&1 &" \
  && echo "$ROBOT is moving to '$CONNECTION' and restarting the platform (~1 min).
Move the workstation to the same network, then: ping $ROBOT"
