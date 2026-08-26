# Physical ROSMASTER X3 Platform

This repository is the robot-side hardware platform for AIRclub UdeSA's Yahboom ROSMASTER X3 fleet. It deliberately provides no autonomous behavior, localization, mapping, navigation, perception application, or EKF. A project repository should run on top of it and own those choices.

The default stack provides:

- motor-controller access and a watchdog-protected `/cmd_vel` input;
- four-wheel encoder state and mecanum wheel odometry;
- robot description and TF;
- raw and Madgwick-filtered IMU data;
- angle-compensated, cable-masked A1 LiDAR data;
- calibrated Astra RGB-D data normalized to the simulator contract;
- standard `/diagnostics` health for controller telemetry and wheel odometry;
- voltage, firmware, magnetometer, buzzer, and RGB hardware extensions.

Default bringup never publishes `/cmd_vel`. Joystick, keyboard, pulse tests, and calibration are separate, explicit operator actions.

## Public contract

The machine-readable contract is [config/robot_contract.yaml](config/robot_contract.yaml). It matches simulator commit `772ba25` except for `/clock` and ground truth.

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

## Workstation setup

The target runtime remains ROS 2 Humble inside the robot container. A workstation can review, test, and build with a compatible ROS 2 environment.

```bash
mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd ~/rosmaster_physical_ws
vcs import src < src/physical_rosmaster/physical_rosmaster.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

The `.repos` file pins Orbbec's `ros2_astra_camera` to `f7e71d9ce806e788cb48d8580aac2c778fba4214`. Do not replace the pin without repeating camera contract validation.

Focused hardware-free checks:

```bash
colcon test --packages-select yahboomcar_base_node
colcon test-result --verbose
python3 -m pytest -q \
  src/physical_rosmaster/yahboomcar_bringup/test/test_x3_driver_utils.py \
  src/physical_rosmaster/yahboomcar_ctrl/test/test_teleop_safety.py \
  src/physical_rosmaster/yahboomcar_astra/test/test_sensor_adapter.py \
  src/physical_rosmaster/laserscan_to_point_pulisher/test/test_scan_conversion.py
```

## Robot setup and manual bringup

The expected clone path is `/root/yahboomcar_ws/src/physical_rosmaster` in the `rosmaster_humble` container. `Rosmaster_Lib` remains a robot-provided dependency and is not vendored here.

Discover and configure stable hardware identities before launch:

```bash
ls -l /dev/serial/by-id
lsusb
ros2 run astra_camera list_devices_node
```

Set per-robot identities in the container environment:

```bash
export ROSMASTER_MOTOR_PORT=/dev/serial/by-id/<motor-controller-id>
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL=<astra-serial>
```

Then launch manually:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Normal bringup shuts down when a required process exits. The motor driver also fails after sustained encoder/telemetry loss, and the camera adapter fails if all valid RGB-D streams do not appear within its startup deadline. The physical probe requires healthy controller and encoder `/diagnostics` before passing.

In a second shell, run the contract gate:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

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

Autostart is intentionally deferred. The feature is ready for robot-side platform validation, not fleet startup.

The gate is:

1. identify the actual Astra model and serial and install its udev rules;
2. pass non-motion contract checks;
3. pass lifted command, encoder, odometry, watchdog, and deadman checks;
4. repeat bounded forward, lateral, and rotation floor trials;
5. run one minimal consumer against simulator and hardware without remaps.

Only after one X3 passes all five should a new autostart routine be designed.

## Documentation

- [docs/setup_guide_ros2_humble_autostart.md](docs/setup_guide_ros2_humble_autostart.md): manual robot setup and the explicit autostart gate;
- [docs/workstation_and_robot_workflow.md](docs/workstation_and_robot_workflow.md): workstation/robot responsibilities;
- [docs/robot_side_verification_todo.md](docs/robot_side_verification_todo.md): mandatory first-robot verification checklist and evidence record;
- [docs/robot_side_next_moves.md](docs/robot_side_next_moves.md): ordered runbook for closing the remaining PR #3 robot gates;
- [docs/odometry_validation.md](docs/odometry_validation.md): encoder-only odometry validation;
- [agents/README.md](agents/README.md): status of pre-cleanup audit and validation evidence;
- [docs/troubleshooting/README.md](docs/troubleshooting/README.md): incident history and known issues.

## Provenance and licensing

This tree includes Yahboom-derived code and the BSD-licensed Slamtec driver. Package-level metadata has been improved for maintained AIRclub packages, but that does not replace a complete repository-wide provenance review.
