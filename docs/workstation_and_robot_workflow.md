# Workstation and robot workflow

Use the workstation for source changes, package inventory, dependency review,
unit tests, Xacro expansion, launch construction, and builds. Use a robot for
USB identity, udev, `Rosmaster_Lib`, camera calibration, real message quality,
motion, and final contract acceptance. Before importing sources, ensure that
`vcs` is installed; Ubuntu provides it in `python3-vcstool`.

## Workstation

Create a normal ROS workspace and import the pinned camera driver:

```bash
(
set -eo pipefail

mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git
# Current pre-merge x3-c rollout: select the reviewed platform branch using
# physical_rosmaster/docs/robot_side_next_moves.md before importing dependencies.

cd ~/rosmaster_physical_ws
vcs import src < src/physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C src/ros2_astra_camera status --porcelain)"
test "$(git -C src/ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list --base-paths src
colcon build --symlink-install
)
```

This default clone is the canonical workflow after the architecture reaches
`main`. For the current pre-merge `x3-c` rollout, select one reviewed,
published `platform/simulator-parity` head at an exact reviewed full SHA that
contains both `a08b097` and `bc965a6` before the `vcs import`; use
[robot_side_next_moves.md](robot_side_next_moves.md).

Expected local package inventory:

```text
laserscan_to_point_pulisher
sllidar_ros2
yahboomcar_astra
yahboomcar_base_node
yahboomcar_bringup
yahboomcar_ctrl
yahboomcar_description
yahboomcar_visual
```

The complete `colcon list --base-paths src` inventory must contain exactly
these eight packages plus `astra_camera` and `astra_camera_msgs`.

Useful focused checks:

```bash
(
set -eo pipefail

cd ~/rosmaster_physical_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/physical_rosmaster-ros-logs
mkdir -p "$ROS_LOG_DIR"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select \
  laserscan_to_point_pulisher sllidar_ros2 yahboomcar_astra \
  yahboomcar_base_node yahboomcar_bringup yahboomcar_ctrl \
  yahboomcar_description yahboomcar_visual
colcon test-result --verbose
python3 -m compileall -q src/physical_rosmaster

cd src/physical_rosmaster
export PYTHONPATH="$PWD/yahboomcar_bringup:$PWD/tools:${PYTHONPATH:-}"
python3 -m pytest -q \
  tools/test_rosmaster_lib_probe.py \
  tools/test_motor_live_loss_probe.py \
  tools/test_motor_live_loss_ros_smoke.py \
  tools/test_physical_contract_probe.py \
  tools/test_safe_cmd_vel_pulse.py
)
```

The package gate already runs the motor transport, driver, and launch tests;
the direct pytest command contains only standalone source-tree tools. The
current workstation has a globally installed pytest plugin that is incompatible
with its pytest version, so these isolated checks disable plugin autoload
explicitly. This is not a runtime setting. Keep `ROS_LOG_DIR` on a writable
path for ROS launch and node tests.
The live-loss ROS smoke test must run on Linux with permission to enumerate
local network interfaces and open localhost DDS sockets.

If the exact reviewed vendor source is available on the workstation, run the
real-pyserial pseudo-terminal compatibility gate separately:

```bash
export ROSMASTER_V339_SOURCE=/absolute/path/to/Rosmaster_Lib.py
python3 -m pytest -q tools/test_rosmaster_v339_pty.py
```

A complete build requires dependencies declared by the pinned Orbbec driver, including `camera_info_manager`, `image_transport`, `image_geometry`, `cv_bridge`, OpenCV, and its USB/OpenNI dependencies. Let `rosdep` resolve them for the target ROS distribution.

Do not install `Rosmaster_Lib` merely to make workstation imports pass. It is a robot hardware dependency; the driver process is not expected to run on a workstation.

## Robot/container

Expected paths:

- host: Yahboom installation, Docker lifecycle, USB passthrough, systemd;
- container: `/root/yahboomcar_ws`;
- repository: `/root/yahboomcar_ws/src/physical_rosmaster`;
- external camera driver: `/root/yahboomcar_ws/src/ros2_astra_camera`.

Before changing an existing workspace, put source backups outside `/root/yahboomcar_ws`. A backup containing packages anywhere under the workspace causes duplicate package discovery.

For the current pre-merge `x3-c` rollout, replace the reviewed-SHA marker and
verify the exact published revision before importing or building:

```bash
(
set -eo pipefail

cd /root/yahboomcar_ws/src/physical_rosmaster
git fetch origin
git checkout platform/simulator-parity
git pull --ff-only
test -z "$(git status --porcelain)"
test "$(git rev-parse HEAD)" = \
  "$(git rev-parse origin/platform/simulator-parity)"
git merge-base --is-ancestor \
  a08b097b22781ca500fd61c01164a4e7167b3873 HEAD
git merge-base --is-ancestor \
  bc965a6f5ccdafb01efd3a1a6a230e9d3bbd8e80 HEAD
test "$(git rev-parse HEAD)" = "REPLACE_WITH_REVIEWED_FULL_SHA"
)
```

Before rebuilding an existing workspace, preserve generated state outside the
workspace instead of deleting it in place:

```bash
(
set -eo pipefail

cd /root/yahboomcar_ws
archive_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
generated_archive="/root/rosmaster-workspace-archives/${archive_stamp}-pre-deploy"
test ! -e "$generated_archive"
mkdir -p /root/rosmaster-workspace-archives
mkdir "$generated_archive"
for generated_tree in build install log; do
  if [ -e "$generated_tree" ]; then
    mv -- "$generated_tree" "$generated_archive/"
  fi
done
test ! -e build
test ! -e install
test ! -e log
)
```

Retain the timestamped archive until the clean build/test evidence is reviewed.
Removing it is a later, explicit maintenance action.

Inside the container:

```bash
(
set -eo pipefail

cd /root/yahboomcar_ws/src
vcs import . < physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C ros2_astra_camera status --porcelain)"
test "$(git -C ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list --base-paths src
colcon build --symlink-install
source install/setup.bash
)
```

Before any driver construction, run the fail-closed installed-source preflight
with the same Python used by ROS 2:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

The command must exit `0`; a nonzero result blocks launch. It reports the
installed path and digest, and commit `bc965a6` makes a source mismatch
fail closed rather than merely printing it. The runtime independently checks
the reviewed source hash after import and before constructing the transport.
The hash confirms the expected version and private hook shape. Runtime
`a08b097` expects
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c` for
public V3.3.9. Because import happens first, this is not supply-chain
attestation.

## Stable devices

Discover identities while no competing process owns the devices:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
lsusb
ros2 run astra_camera list_devices_node
```

Copy and edit [../config/99-rosmaster-x3.rules.example](../config/99-rosmaster-x3.rules.example) on the robot host. Never deploy placeholder serial values. Reload rules, reconnect hardware, and verify aliases before starting ROS.

Use a unique device serial when available. If the CH340 motor controller has no
serial, bind its alias to a dedicated physical USB port with the template's
`KERNELS` fallback and verify that port after reboot.

Configure the container:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL="REPLACE_WITH_ASTRA_SERIAL"
```

The launch arguments can override these environment variables for a single run.

## Motor runtime boundary

Runtime commit `a08b097` treats only checksum-valid parsed arrivals as
controller liveness: `0x0A` for speed and battery, `0x0D` for encoders, and
either `0x0B` or `0x0E` for raw IMU. All three required channels must stay
fresh. Cached `Rosmaster_Lib` getters never establish or refresh liveness.

A terminal report-freshness, receive-thread, or observed serial-write failure
suppresses controller-derived topic publication, nonzero motion commands, and
all RGB/buzzer requests. The driver emits a structured `ERROR`, exits, and the
strict launch shuts down the remaining graph.

Serial-write completion only establishes that a full frame reached the host
serial layer. It is not a controller acknowledgement. Likewise, redundant zero
attempts are not physical-stop proof. Active-motion validation must compare
reported motion/encoders and external observation; do not infer acceptance from
a successful write call or stop log.

## Manual validation order

Use [robot_side_verification_todo.md](robot_side_verification_todo.md) as the
authoritative, evidence-backed checklist for the first robot acceptance run.
Runtime commit `a08b097` has not yet passed this robot-side checklist. Deploy
and record one clean branch head at an exact reviewed full SHA containing both
that runtime commit and validation-tooling commit `bc965a6`, rather than
treating older `x3-c` evidence as acceptance of the new transport behavior.

Start the platform manually:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Do not start joystick, keyboard, calibration, or an external project yet. In a second shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

If strict bringup exits, fix the missing required process or hardware. Do not weaken the launch to continue with a partial graph.

After the first positive probe passes, stop bringup and launch once with the
motor controller absent. Require the driver never to become healthy and the
strict graph to drain. Restore the controller, start a clean strict launch, and
pass the full physical contract again before arming live loss.

The dedicated live-loss check is still a no-motion test. Keep every wheel
secured, ensure that no `/cmd_vel` publisher exists, export the same
`ROSMASTER_MOTOR_PORT` value in the probe shell, and run:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
mkdir -p /root/rosmaster-recovery-evidence
python3 tools/motor_live_loss_probe.py \
  --device "$ROSMASTER_MOTOR_PORT" \
  --confirm-wheels-secured \
  --output /root/rosmaster-recovery-evidence/motor-live-loss.json
```

Wait for `ARMED` before disconnecting only the motor-controller device. The
probe observes but never publishes commands. Restore the device afterward,
restart strict bringup, and require the full physical contract to pass again.
This validates graph/data failure behavior, not stopping from active motion.

After the non-motion gate passes, follow the detailed robot checklist for a
separate securely lifted, very-low-speed active-link-loss test with the main
power switch immediately reachable. Tested on `x3-c` on 2026-09-02: it did not
demonstrate bounded physical stop, root-caused to an absent command-loss
watchdog in the motor controller protocol. The project owner accepted this as
a known limitation rather than a floor-use blocker — see
[the known-issue writeup](troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md).
Main power remains the only proven stop mechanism for this failure mode.

Then complete the remaining lifted gates:

1. lift and secure the robot;
2. record `/cmd_vel`, `/joint_states`, `/odom`, `/tf`, voltage, and diagnostics;
3. use the bounded pulse tool for forward, left strafe, and CCW rotation;
4. verify all wheel signs and `odom -> base_footprint` direction;
5. verify driver watchdog stop and joystick deadman/release/timeout;
6. repeat conservative trials on a clear floor with external distance/heading observations.

## Operator processes

Run only one `/cmd_vel` source at a time.

```bash
ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py device_id:=0
ros2 run yahboomcar_ctrl yahboom_keyboard
```

The bounded pulse tool refuses to start if another `/cmd_vel` publisher is visible:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder
```

Raw sensor inspection needs no behavior package:

```bash
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/mag
ros2 topic echo /diagnostics
ros2 topic echo /scan --once
ros2 topic echo /cam_1/color/camera_info --once
ros2 topic echo /cam_1/depth/camera_info --once
```

Use `camera_calibration` only if the device-reported intrinsics fail the contract or a calibrated replacement is intentionally being produced. Preserve the device serial, resolution, calibration date, and generated YAML in the robot's deployment record; do not commit one robot's calibration as a fleet-wide default.

## Autostart boundary

The old autostart instructions targeted separate camera/LiDAR processes, unstable device names, and the former EKF graph. They are obsolete.

Do not point host systemd or `/root/auto_start.sh` at this branch. Autostart
stays blocked until one X3 passes the complete contract, no-motion live-loss
and restoration checks, lifted motion, floor motion, and simulator/physical
consumer acceptance. Bounded active-link-loss stop behavior is a documented,
owner-accepted exception rather than a required pass — see
[the known-issue writeup](troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md).
Preparing versioned autostart files is a later project, not part of the current
manual-validation architecture.
