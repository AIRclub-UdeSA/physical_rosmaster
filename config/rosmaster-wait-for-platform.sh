#!/usr/bin/env bash
# Waits, bounded, for the Docker container and the motor/LiDAR/Astra devices
# an autostart launch needs, instead of racing a container mid-restart or
# udev that has not settled yet. Reads the same ROSMASTER_* identity
# variables as the launch itself, so it waits for the exact paths that will
# actually be used, not a hardcoded guess.
# Install as /usr/local/sbin/rosmaster-wait-for-platform (mode 0755, root-owned).
set -euo pipefail

CONTAINER_NAME="${ROSMASTER_CONTAINER_NAME:-rosmaster_humble}"
MOTOR_PORT="${ROSMASTER_MOTOR_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
LIDAR_PORT="${ROSMASTER_LIDAR_PORT:-/dev/robot/lidar}"
WAIT_TIMEOUT="${PLATFORM_WAIT_TIMEOUT:-60}"
WAIT_INTERVAL="${PLATFORM_WAIT_INTERVAL:-2}"

deadline=$(( $(date +%s) + WAIT_TIMEOUT ))

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" = "true" ]
}

# The X3 Astra exposes OpenNI depth as 2bc5:060f and RGB as a separate UVC
# function at 2bc5:050f (see astra_platform.launch.py); both are required.
astra_present() {
  lsusb -d 2bc5:060f >/dev/null 2>&1 && lsusb -d 2bc5:050f >/dev/null 2>&1
}

while true; do
  missing=()
  container_running || missing+=("container:${CONTAINER_NAME}")
  [ -e "$MOTOR_PORT" ] || missing+=("motor:${MOTOR_PORT}")
  [ -e "$LIDAR_PORT" ] || missing+=("lidar:${LIDAR_PORT}")
  astra_present || missing+=("astra:usb-2bc5")

  if [ "${#missing[@]}" -eq 0 ]; then
    logger -t rosmaster-wait-for-platform "OK: container and all devices present"
    exit 0
  fi

  if [ "$(date +%s)" -ge "$deadline" ]; then
    logger -t rosmaster-wait-for-platform \
      "Timed out after ${WAIT_TIMEOUT}s waiting for: ${missing[*]}"
    exit 1
  fi

  sleep "$WAIT_INTERVAL"
done
