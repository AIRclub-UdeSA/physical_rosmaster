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
 - It clamps motion commands, stops persistent commands after 0.5 seconds without an update, and sends repeated zero commands on startup/shutdown.
 - It polls `get_motor_encoder()` and publishes four wheel positions and angular velocities on `/joint_states`.
 - It publishes `/vel_raw` from `Rosmaster_Lib.Rosmaster.get_motion_data()`.
 - `yahboomcar_base_node/src/base_node_X3.cpp` computes 4-wheel mecanum kinematics from `/joint_states` and integrates velocities into `/odom_raw` (falls back to `/vel_raw` if joint states are absent).
 - `robot_localization` fuses `/odom_raw` with IMU and publishes `/odom`.
 
 Hardware validation findings:
 
 - `get_motor_encoder()` returns four 32-bit signed counters.
- Floor pulse tests verified that encoder counters increment and `/odom_raw` integrates during motion, but they did not satisfy the lifted validation gate or provide ground-truth calibration.
 - The direction-controlled 2026-08-20 hand test validated raw packet-field order `[FL, FR, BL, BR] = [m1, m3, m2, m4]` with signs `[+, +, +, +]`.
 - The operator verified the powered-off cabling against Yahboom's diagram. `Rosmaster_Lib`'s `m1..m4` names are packet positions, not proven PCB motor-port labels, so no cable swap is required.
- With the rebuilt mapping, floor observations matched the forward, strafe-left, and CCW wheel-sign patterns. Wheel magnitudes remain strongly imbalanced, and floor yaw commands `0.12` and `0.30` did not move the encoders; `0.50` did. A true lifted repetition remains outstanding.
 - Earlier floor testing revealed a motor deadband at low speeds (0.10 m/s).
 - Standard recovery checklist is in `agents/x3-c_validation_checklist.md`.
 
 ## Immediate Robot-Side Work
 
1. Follow `agents/x3-c_validation_checklist.md` for hardware recovery and validation.
2. Repeat the forward, strafe, and rotation sign checks with the robot securely lifted.
3. Measure exact encoder CPR with marked wheel rotations and investigate the per-wheel magnitude imbalance.
4. Ensure battery is charged (> 12.0 V) before any further floor motion or ground-truth calibration.
5. Run repeatable floor trials only after those gates pass.

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
