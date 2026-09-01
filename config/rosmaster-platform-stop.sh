#!/usr/bin/env bash
# ExecStop cleanup for rosmaster-platform.service. A `docker exec` client
# exiting does not guarantee the process tree it started inside the
# container dies, and on 2026-09-02 a launched ros2 graph proved
# unresponsive to SIGINT/SIGTERM sent to only its top process once it had
# been backgrounded (see docs/robot_side_next_moves.md). Sweep every
# platform node by name instead of trusting one signal to cascade.
# Install as /usr/local/sbin/rosmaster-platform-stop (mode 0755, root-owned).
#
# No `set -e`: pkill exits nonzero when a pattern matches nothing, which is
# the normal, expected case here, not a failure to abort on.
set -uo pipefail

CONTAINER_NAME="${ROSMASTER_CONTAINER_NAME:-rosmaster_humble}"
GRACE_SECONDS="${PLATFORM_STOP_GRACE:-5}"

# Top-level launch first, then every node it can start. Most specific
# pattern first so a short name cannot match an unrelated process.
PATTERNS=(
  "yahboomcar_bringup_X3_launch"
  "Mcnamu_driver_X3"
  "base_node_X3"
  "imu_filter_madgwick_node"
  "sllidar_node"
  "astra_sensor_adapter"
  "astra_camera_node"
  "robot_state_publisher"
)

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" >/dev/null 2>&1; then
  logger -t rosmaster-platform-stop "Container ${CONTAINER_NAME} not running; nothing to stop"
  exit 0
fi

for pattern in "${PATTERNS[@]}"; do
  docker exec "$CONTAINER_NAME" pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep "$GRACE_SECONDS"

killed_any=0
for pattern in "${PATTERNS[@]}"; do
  if docker exec "$CONTAINER_NAME" pkill -KILL -f "$pattern" 2>/dev/null; then
    killed_any=1
  fi
done

if [ "$killed_any" -eq 1 ]; then
  logger -t rosmaster-platform-stop \
    "Some platform processes needed SIGKILL after ${GRACE_SECONDS}s grace"
else
  logger -t rosmaster-platform-stop "Clean stop: all platform processes exited on SIGTERM"
fi

exit 0
