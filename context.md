# Physical ROSMASTER Agent Context

Date: 2026-08-11

This file is for coding agents or engineers working inside the robot/container. Read this before changing code on the robot.

## Objective

This repository contains the physical ROS 2 Humble workspace source for AIRclub UdeSA Yahboom ROSMASTER X3 mecanum robots. The simulator is intentionally separate at `AIRclub-UdeSA/yahboom_rosmaster`.

The active engineering goal is to bring the physical robot closer to the simulator contract, especially odometry:

- keep `/cmd_vel` as the command input
- publish real wheel state through `/joint_states` if encoder counters are usable
- compute mecanum odometry from encoder deltas when hardware confirms this is possible
- keep EKF ownership clear: raw odometry on `/odom_raw`, filtered odometry on `/odom`

## Current Repo State

- Public repo: `https://github.com/AIRclub-UdeSA/physical_rosmaster`
- Expected robot workspace: `/root/yahboomcar_ws`
- Expected clone path inside container: `/root/yahboomcar_ws/src/physical_rosmaster`
- Expected Docker container: `rosmaster_humble`
- Expected ROS distro: Humble
- Large optional artifacts are restored with `tools/fetch_large_artifacts.sh`
- `Rosmaster_Lib` is not vendored here; it is expected to be installed on the robot/container.
- On `x3-c`, the installed `Rosmaster_Lib` has been verified to match public V3.3.9 exactly.

## Read These Docs First

- `README.md`: repo overview and package inventory.
- `docs/setup_guide_ros2_humble_autostart.md`: sanitized setup guide consolidated from the original robot notes.
- `docs/workstation_and_robot_workflow.md`: how to work outside vs inside the robot.
- `docs/odometry_validation.md`: hardware validation plan for encoder odometry.
- `docs/large_artifacts.md`: optional SLAM/PCD artifact restore.
- `docs/troubleshooting/README.md`: incident-driven troubleshooting index.
- `agents/rosmaster_lib_public_v3_3_9.md`: public Yahboom library findings.
- `agents/physical_rosmaster_todo.md`: current task list.

## Safety Rules On The Robot

- Lift the robot or keep it in a safe open area before publishing nonzero `/cmd_vel`.
- Always send a zero `/cmd_vel` after motion tests.
- Do not run destructive workspace commands until the current `src` tree is backed up.
- Do not overwrite robot-side local edits. Check `git status --short` before pulling or editing.
- Do not commit `build/`, `install/`, `log/`, bags, caches, `ORBvoc.txt`, or `.pcd` files.
- Do not vendor `Rosmaster_Lib` into this repo until licensing is decided.

Emergency stop command:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Robot Environment Setup

Inside the container:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Verify this repo:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git status --short
git pull --ff-only
```

Build:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Odometry Facts So Far

Physical driver path:

- `yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py` subscribes to `/cmd_vel`.
- It calls `Rosmaster_Lib.Rosmaster.set_car_motion(...)`.
- It publishes `/vel_raw` from `Rosmaster_Lib.Rosmaster.get_motion_data()`.
- `yahboomcar_base_node/src/base_node_X3.cpp` integrates `/vel_raw` into `/odom_raw`.
- `robot_localization` fuses `/odom_raw` with IMU and publishes `/odom`.

Public Yahboom `Rosmaster_Lib` V3.3.9 findings:

- `get_motion_data()` returns cached controller speed feedback from serial packet `FUNC_REPORT_SPEED = 0x0A`; it is not direct Python echo of ROS `/cmd_vel`.
- `get_motor_encoder()` returns four cached signed 32-bit motor encoder counters from serial packet `FUNC_REPORT_ENCODER = 0x0D`.
- On `x3-c`, a stationary probe showed `motion_vx`, `motion_vy`, and `motion_vz` staying at `0.000000`, encoder counters staying at `-3, 2, 1, 90`, battery around `10.5` to `10.6` V, and no serial errors.
- The remaining question is whether those counters are sufficient and correctly signed for motion odometry during actual wheel movement.

## Immediate Robot-Side Work

1. Confirm installed `Rosmaster_Lib` if you need to re-check the robot container:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

2. Sample stationary motion/encoder data:

```bash
python3 tools/rosmaster_lib_probe.py --samples 100 --period 0.1
```

3. Follow `docs/odometry_validation.md` to record `/cmd_vel`, `/vel_raw`, `/odom_raw`, `/odom`, IMU, and `/tf`.

4. Report exact outputs and robot conditions:

- robot lifted or on floor
- command used
- wheel that moved or was resisted
- encoder values before/after
- `/vel_raw` values
- `/odom_raw` behavior

## Likely Code Path If Encoders Work

- Update `Mcnamu_driver_X3.py` to call `get_motor_encoder()`.
- Publish real X3 wheel names and positions/velocities in `/joint_states`.
- Add a new encoder-delta mecanum odometry implementation or refactor `base_node_X3.cpp` to consume wheel state.
- Use simulator math as the reference contract.
- Keep existing velocity odometry as fallback until the encoder path is validated on the robot.

## Verification Commands

Hardware-free focused test:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select yahboomcar_base_node --ctest-args -R test_x3_odometry
```

Robot-side launch smoke test:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Topic smoke checks:

```bash
ros2 topic list
ros2 topic echo /vel_raw --once
ros2 topic echo /odom_raw --once
ros2 topic echo /imu/data_raw --once
```
