#!/usr/bin/env bash
# ExecStart for rosmaster-shutdown-listener.service (issue #18): powers the
# host off when a confirmed request arrives on /system/shutdown, without SSH.
# ROS 2 runs only inside the container, so the container side
# (tools/ros_shutdown_listener.py) only listens and reports through its exit
# status; this host side does the stopping. No port, no new container option,
# and it runs independently of the platform launch, so it still works after
# the platform has crashed.
#
# Order matters: powering off the Pi does NOT stop the motors. The motor
# controller is battery-powered and has no link-loss watchdog (see
# docs/troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md),
# so the platform is stopped first, letting the driver send its best-effort
# zero commands, and only then is the host powered off. Main power still has
# to be switched off by hand afterwards.
# Install as /usr/local/sbin/rosmaster-shutdown-listen (mode 0755, root-owned).
#
# No `set -e`: the listener's nonzero exit status is the request itself, and
# a failed platform stop must still be followed by the poweroff.
set -uo pipefail

CONTAINER_NAME="${ROSMASTER_CONTAINER_NAME:-rosmaster_humble}"
WORKSPACE="${ROSMASTER_WORKSPACE:-/root/yahboomcar_ws}"
PLATFORM_UNIT="rosmaster-platform.service"
# Must match EXIT_SHUTDOWN_REQUESTED in tools/ros_shutdown_listener.py.
EXIT_SHUTDOWN_REQUESTED=42

# The value a request must carry: this robot's hostname, unless platform.env
# sets a less guessable one. Exported and passed to docker exec by name only,
# so it never appears in a command line `ps` can read.
ROSMASTER_SHUTDOWN_CONFIRM="${ROSMASTER_SHUTDOWN_CONFIRM:-$(hostname -s)}"
export ROSMASTER_SHUTDOWN_CONFIRM

# A docker exec client dying (unit stop, restart) does not kill what it
# started inside the container; clear any listener a previous run left behind.
docker exec "$CONTAINER_NAME" pkill -TERM -f ros_shutdown_listener.py 2>/dev/null || true

status=0
docker exec \
  -e ROSMASTER_SHUTDOWN_CONFIRM \
  "$CONTAINER_NAME" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && exec python3 '$WORKSPACE/src/physical_rosmaster/tools/ros_shutdown_listener.py'" \
  || status=$?

if [ "$status" -ne "$EXIT_SHUTDOWN_REQUESTED" ]; then
  logger -t rosmaster-shutdown \
    "Listener exited with status ${status} and no shutdown request; systemd restarts it"
  exit 1
fi

logger -t rosmaster-shutdown \
  "Shutdown requested over ROS (/system/shutdown); stopping ${PLATFORM_UNIT} so the driver sends its zero commands"

started=$(date +%s)
if systemctl stop "$PLATFORM_UNIT"; then
  logger -t rosmaster-shutdown \
    "${PLATFORM_UNIT} stopped after $(( $(date +%s) - started ))s"
else
  stop_status=$?
  # Power off anyway: the request was explicit, and the shutdown runs the
  # same ExecStop sweep again. Refusing would leave a half-stopped robot.
  logger -p user.err -t rosmaster-shutdown \
    "systemctl stop ${PLATFORM_UNIT} failed with status ${stop_status} after $(( $(date +%s) - started ))s; powering off anyway"
fi

logger -t rosmaster-shutdown \
  "Powering off the host. This halts only the Pi: switch main power off once it has halted"
systemctl poweroff
