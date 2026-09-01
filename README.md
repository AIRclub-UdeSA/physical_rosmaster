# Physical ROSMASTER X3 Platform

This repository is the robot-side hardware platform for AIRclub UdeSA's Yahboom ROSMASTER X3 fleet. It deliberately provides no autonomous behavior, localization, mapping, navigation, perception application, or EKF. A project repository should run on top of it and own those choices.

The default stack provides:

- motor-controller access and a watchdog-protected, feedback-gated `/cmd_vel`
  input;
- four-wheel encoder state and mecanum wheel odometry;
- robot description and TF;
- raw and Madgwick-filtered IMU data;
- angle-compensated, cable-masked A1 LiDAR data;
- calibrated Astra RGB-D data normalized to the simulator contract;
- standard `/diagnostics` health for controller telemetry and wheel odometry;
- voltage, firmware, magnetometer, buzzer, and RGB hardware extensions.

Default bringup never publishes `/cmd_vel`. Joystick, keyboard, pulse tests, and calibration are separate, explicit operator actions.

## Public contract

The machine-readable contract is
[config/robot_contract.yaml](config/robot_contract.yaml). Its simulator-facing
interfaces match simulator commit `772ba25`; `/clock` and ground truth are
excluded, and the hardware-only extensions below are additional.

| Interface | Physical implementation |
|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` input; no default publisher |
| `/joint_states` | Position and velocity for four simulator-named wheel joints |
| `/odom` | Encoder-only mecanum odometry, `odom` → `base_footprint` |
| `/tf` | Wheel odometry owns `odom` → `base_footprint` |
| `/imu/data` | Madgwick-filtered IMU; `use_mag=false`; no IMU TF |
| `/scan` | A1 scan in `laser_link`, after physical cable/self-return masking |
| `/cam_1/color/*` | Calibrated RGB8 color image and camera info |
| `/cam_1/depth/*` | Metric 32FC1 depth, camera info, and XYZRGB cloud |

The cloud is transformed into x-forward `cam_1_depth_frame`; it is not merely relabeled. Hardware-only topics such as `/diagnostics`, `/imu/data_raw`, `/imu/mag`, `/vel_raw`, `/voltage`, `/edition`, `/Buzzer`, `/RGBLight`, and `/scan_filtered` remain available.

## Motor transport and failure boundary

Runtime commit `a08b097` derives motor-controller liveness only from
checksum-valid report arrivals parsed from the serial receive stream:

- `0x0A` supplies speed and battery feedback;
- `0x0D` supplies the four encoder counters;
- either `0x0B` or `0x0E` supplies raw IMU feedback.

All three channels must arrive and remain fresh. Reading a cached
`Rosmaster_Lib` getter never creates or refreshes liveness evidence.

A terminal feedback, receive-thread, or observed serial-write failure is
latched. The driver then suppresses controller-derived ROS output, nonzero
motion commands, and all RGB/buzzer requests; publishes a structured
motor-controller `ERROR` diagnostic; and exits. The strict platform launch
treats that driver exit as fatal and shuts down the rest of the graph.

This is fail-closed process and data behavior, not proof that the controller
executed a stop. A completed host serial write is not a controller
acknowledgement, and redundant zero-command attempts are not physical-stop
proof. Lifted and floor acceptance must confirm motion and stopping from
available controller feedback and external observation.

The allowlisted public V3.3.9 `Rosmaster_Lib` source hash is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
It is a compatibility/version gate for the exact private hooks used by this
runtime. Python imports the module before the source is hashed, so the check is
not supply-chain attestation.

## Retained packages

`colcon list --base-paths .` discovers exactly these eight local packages:

- `yahboomcar_bringup`: strict X3 driver and complete platform launch;
- `yahboomcar_base_node`: encoder-only mecanum `/odom` and TF;
- `yahboomcar_description`: canonical X3 description;
- `yahboomcar_ctrl`: opt-in joystick and keyboard operator tools;
- `yahboomcar_astra`: Astra normalization and strict sensor watchdog;
- `sllidar_ros2`: A1 driver and platform scan preprocessing;
- `yahboomcar_visual`: generic scan/image inspection conversions;
- `laserscan_to_point_pulisher`: generic `LaserScan` → `PointCloud2` utility. The historical package spelling is retained.

Removed behavior packages are recoverable at tag `pre-platform-contract-cleanup` or through Git history.

Default bringup composes the required platform packages into one strict graph
for manual validation. Operator-control and generic inspection/conversion
utilities remain separate and opt in. There is no accepted autostart routine
for this architecture yet.

## Workstation setup

The target runtime remains ROS 2 Humble inside the robot container. A workstation can review, test, and build with a compatible ROS 2 environment.

```bash
mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git
# Current pre-merge x3-c recovery: select the reviewed platform branch using
# physical_rosmaster/docs/robot_side_next_moves.md before importing dependencies.

cd ~/rosmaster_physical_ws
vcs import src < src/physical_rosmaster/physical_rosmaster.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The `.repos` file pins Orbbec's `ros2_astra_camera` to `f7e71d9ce806e788cb48d8580aac2c778fba4214`. Do not replace the pin without repeating camera contract validation.

The clone command above describes the eventual canonical setup from `main`.
Until the simulator-parity work is merged, the current `x3-c` recovery must
instead deploy one reviewed, published `platform/simulator-parity` head that
contains runtime commit `a08b097`; follow
[the ordered recovery runbook](docs/robot_side_next_moves.md).

Focused hardware-free checks:

```bash
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

cd src/physical_rosmaster
export PYTHONPATH="$PWD/yahboomcar_bringup:$PWD/tools:$PYTHONPATH"
python3 -m pytest -q \
  yahboomcar_bringup/test/test_rosmaster_transport.py \
  yahboomcar_bringup/test/test_x3_driver_utils.py \
  yahboomcar_bringup/test/test_x3_driver_feedback.py \
  yahboomcar_bringup/test/test_x3_launch.py \
  tools/test_motor_live_loss_probe.py \
  tools/test_physical_contract_probe.py \
  tools/test_safe_cmd_vel_pulse.py
```

The explicit plugin setting isolates these checks from an incompatible global
pytest plugin installed on the current workstation; it is not a robot runtime
setting. ROS tests also need a writable `ROS_LOG_DIR`.

## Robot setup and manual bringup

The expected clone path is `/root/yahboomcar_ws/src/physical_rosmaster` in the
`rosmaster_humble` container. `Rosmaster_Lib` remains a robot-provided
dependency and is not vendored here. Its source hash verifies the reviewed
runtime shape after import; it does not establish package provenance or trust.

Discover and configure stable hardware identities before launch:

```bash
ls -l /dev/serial/by-id
lsusb
ros2 run astra_camera list_devices_node
```

Set per-robot identities in the container environment:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL="REPLACE_WITH_ASTRA_SERIAL"
```

Before launch, stop every competing legacy/controller process, verify that no
command source or `/cmd_vel` publisher remains, and secure or lift all four
wheels. Then launch manually:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Normal bringup shuts down when a required process exits. The motor driver
fails terminally on missing/stale required report channels, receiver failure,
or an observed serial-write failure. The camera adapter fails if all valid
RGB-D streams do not appear within its startup deadline. The physical probe
requires current healthy controller and encoder `/diagnostics` before passing.

In a second shell, run the contract gate:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

After the positive contract, test motor absence at startup. Stop bringup,
disconnect only the motor controller, relaunch, and require the driver never to
become healthy and the strict graph to drain. Reconnect it, start a clean
launch, and pass the complete physical contract again before the subsequent
supervised no-motion live-loss gate.

For live loss, export the same motor path in the probe shell, create the
evidence directory, and pass both explicitly:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
mkdir -p /root/rosmaster-recovery-evidence
python3 tools/motor_live_loss_probe.py \
  --device "$ROSMASTER_MOTOR_PORT" \
  --confirm-wheels-secured \
  --output /root/rosmaster-recovery-evidence/motor-live-loss.json
```

Follow [the workstation/robot workflow](docs/workstation_and_robot_workflow.md)
for the `ARMED`, second restoration, and final positive-contract sequence.
This probe is not an active-motion stop test.

## Operator tools

These are never part of default bringup.

```bash
# Joystick: held deadman, configurable mapping, timeout, release stop
ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py device_id:=0

# Keyboard: must run in an interactive terminal
ros2 run yahboomcar_ctrl yahboom_keyboard

# Bounded supervised pulse
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder

# Raw IMU and magnetometer inspection
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/mag
```

Manual control is capped at `0.20 m/s` linear and `1.0 rad/s` angular, with lower gears available. Calibration nodes start inert and require an explicit `start_test=true` parameter after their bounded settings are reviewed.

## Rollout status

Autostart remains blocked. Runtime commit `a08b097` has hardware-free
workstation coverage, but its clean deployment and physical validation on a
robot are pending. Older robot runs do not validate this runtime.

The gate is:

1. deploy and record one clean reviewed branch head containing runtime commit
   `a08b097`, with verified stable device identities;
2. pass the positive non-motion contract, motor-absent-at-startup gate, restored
   positive contract, observer-only live motor-loss gate, and another restored
   positive contract, in that order;
3. prove bounded physical stop in a separate securely lifted,
   very-low-speed active-link-loss test, or validate a controller-side or
   independent hardware watchdog, then pass the other lifted command,
   encoder, odometry, watchdog, and deadman checks;
4. repeat bounded forward, lateral, and rotation floor trials;
5. run one minimal consumer against simulator and hardware without remaps.

Only after one X3 passes all five should a new autostart routine be designed.

## Documentation

- [docs/setup_guide_ros2_humble_autostart.md](docs/setup_guide_ros2_humble_autostart.md): manual robot setup and the explicit autostart gate;
- [docs/workstation_and_robot_workflow.md](docs/workstation_and_robot_workflow.md): workstation/robot responsibilities;
- [docs/robot_side_verification_todo.md](docs/robot_side_verification_todo.md): mandatory first-robot verification checklist and evidence record;
- [docs/robot_side_next_moves.md](docs/robot_side_next_moves.md): ordered `x3-c` remediation and acceptance runbook;
- [docs/odometry_validation.md](docs/odometry_validation.md): encoder-only odometry validation;
- [agents/README.md](agents/README.md): index of historical pre-cleanup audit and validation evidence;
- [docs/troubleshooting/README.md](docs/troubleshooting/README.md): incident history and known issues.

## Provenance and licensing

This tree includes Yahboom-derived code and the BSD-licensed Slamtec driver. Package-level metadata has been improved for maintained AIRclub packages, but that does not replace a complete repository-wide provenance review.
