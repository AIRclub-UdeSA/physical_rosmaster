#!/usr/bin/env bash
# ExecStart for rosmaster-platform.service. A real script instead of an
# inline unit-file command so this `exec docker exec ...` replaces itself,
# and the container-side `exec ros2 launch ...` replaces that shell in turn:
# systemd ends up tracking the ros2 launch process directly, with no nohup
# or backgrounded parent to lose track of its children. See the 2026-09-02
# finding in docs/robot_side_next_moves.md about signals not reaching a
# detached ros2 launch — this avoids that shape entirely.
# Install as /usr/local/sbin/rosmaster-platform-launch (mode 0755, root-owned).
set -euo pipefail

CONTAINER_NAME="${ROSMASTER_CONTAINER_NAME:-rosmaster_humble}"
WORKSPACE="${ROSMASTER_WORKSPACE:-/root/yahboomcar_ws}"

exec docker exec \
  -e ROSMASTER_MOTOR_PORT \
  -e ROSMASTER_LIDAR_PORT \
  -e ROSMASTER_ASTRA_SERIAL \
  "$CONTAINER_NAME" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && source '$WORKSPACE/install/setup.bash' && exec ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py"
