# Odometry Validation Plan

Date: 2026-08-11

Goal: decide whether the physical robot can use encoder-delta mecanum odometry, matching the simulator contract, instead of relying only on firmware velocity integration.

## Current Sources

- `/cmd_vel`: commanded chassis velocity.
- `/vel_raw`: chassis velocity returned by `Rosmaster_Lib.get_motion_data()` and published by `Mcnamu_driver_X3.py`.
- `/odom_raw`: velocity integration from `yahboomcar_base_node`.
- `/odom`: EKF output after fusing `/odom_raw` and IMU.
- `Rosmaster_Lib.get_motor_encoder()`: four motor encoder counters exposed by the public V3.3.9 library, not yet published by the X3 ROS driver.

## Stage 1: Confirm Installed Library

Inside `rosmaster_humble`:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

Record:

- source path
- SHA256
- file size
- version comment
- whether it matches the public V3.3.9 hash in `agents/rosmaster_lib_public_v3_3_9.md`

## Stage 2: Sample Motion And Encoders Without ROS Motion Commands

With the robot stationary:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 50 --period 0.1
```

Expected:

- motion values should stay near zero
- encoder counters should stay stable when wheels are not moving
- battery should report a plausible voltage if the library exposes it

Lift the robot and rotate each wheel by hand if safe. Encoder counters should change for the corresponding wheel channels.

## Stage 3: Compare Command, Firmware Velocity, And Encoders

Terminal A:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Terminal B:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 bag record -o /tmp/x3_odom_probe \
  /cmd_vel /vel_raw /odom_raw /odom /imu/data_raw /imu/data /tf
```

Terminal C:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 300 --period 0.1
```

Motion commands, with wheels lifted first:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.20, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.50}}"
sleep 2
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Repeat on the floor at low speed only after lifted tests behave correctly.

## Stage 4: Interpret Results

Good encoder signal:

- counters change with wheel rotation
- counters stop changing when wheels stop
- count signs are consistent for forward, strafe, and rotate commands
- one resisted wheel produces a visibly different delta for that wheel

Good firmware velocity signal:

- `/vel_raw` goes near zero after stop commands
- `/vel_raw.linear.y` is nonzero for strafe commands
- `/vel_raw.angular.z` is nonzero for rotate commands
- `/vel_raw` changes when the robot is physically resisted if firmware speed is encoder-derived

If encoders are stable, implement physical odometry from four wheel deltas and publish meaningful `/joint_states`.

If encoders are unavailable or unreliable, keep firmware velocity odometry, document it as such, tune covariance honestly, and rely on IMU/LiDAR fusion for correction.

## Stage 5: Data To Capture For Implementation

Record these before coding encoder odometry:

- wheel order: encoder motor 1, 2, 3, 4 to physical wheel name
- sign convention for each wheel
- ticks per wheel revolution
- wheel radius
- wheel separation in X and Y, or equivalent mecanum geometry constants
- whether encoder counts are absolute since boot or periodic deltas
- whether counts reset when motor controller resets
