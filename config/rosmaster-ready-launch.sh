#!/usr/bin/env bash
# ExecStart for rosmaster-platform-ready.service. Two steps, in order:
#   1) run the same physical-contract probe used for every manual
#      acceptance gate this project has passed, from inside the container;
#   2) only if that passes, fire the buzzer + RGBLight boot-ready signal.
# `set -e` makes a probe failure or timeout abort before step 2 runs, so no
# signal is ever sent for an unverified graph.
# Install as /usr/local/sbin/rosmaster-ready-launch (mode 0755, root-owned).
set -euo pipefail

CONTAINER_NAME="${ROSMASTER_CONTAINER_NAME:-rosmaster_humble}"
WORKSPACE="${ROSMASTER_WORKSPACE:-/root/yahboomcar_ws}"

docker exec "$CONTAINER_NAME" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && source '$WORKSPACE/install/setup.bash' && python3 '$WORKSPACE/src/physical_rosmaster/tools/physical_contract_probe.py'"

docker exec \
  -e READY_RGB_EFFECT \
  "$CONTAINER_NAME" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && source '$WORKSPACE/install/setup.bash' && bash '$WORKSPACE/src/physical_rosmaster/config/rosmaster-ready-signal.sh'"
