# Physical ROSMASTER

Physical ROS 2 Humble workspace for the Yahboom ROSMASTER X3 mecanum robots used by AIRclub UdeSA.

This repository is intentionally separate from the simulator repository, `AIRclub-UdeSA/yahboom_rosmaster`. Simulator users should not need physical robot drivers, Docker setup, serial-device access, or Yahboom hardware libraries.

## Current Scope

This repo is a curated boundary around the physical robot source tree that was previously used directly as `/root/yahboomcar_ws/src` inside the `rosmaster_humble` Docker container.

The setup guide currently lives outside this repo at:

`/home/juan/Downloads/Guia ros2 humble y autostart.md`

Important deployment assumptions from that guide:

- ROS 2 Humble runs inside Docker container `rosmaster_humble`.
- Image used during setup: `yahboomtechnology/ros-humble:4.1.2`.
- The container runs with host networking, privileged mode, `/dev` mounted, and a robot-specific `ROS_DOMAIN_ID`.
- `Rosmaster_Lib` is copied into the container from the robot host and made importable for Python.
- The physical bringup launches the motor driver, base odometry node, IMU filter, EKF, joystick node, camera, lidar, and robot description.

## Package Inventory

Buildable packages currently discovered by `colcon`:

- `laserscan_to_point_pulisher`
- `robot_pose_publisher`
- `sllidar_ros2`
- `yahboom_app_save_map`
- `yahboom_web_savmap_interfaces`
- `yahboomcar_astra`
- `yahboomcar_base_node`
- `yahboomcar_bringup`
- `yahboomcar_ctrl`
- `yahboomcar_description`
- `yahboomcar_description_x1`
- `yahboomcar_laser`
- `yahboomcar_linefollow`
- `yahboomcar_mediapipe`
- `yahboomcar_msgs`
- `yahboomcar_nav`
- `yahboomcar_slam`
- `yahboomcar_visual`
- `yahboomcar_voice_ctrl`

Packages currently present but ignored by `COLCON_IGNORE`:

- `yahboomcar_KCFTracker`
- `robot_pose_publisher_ros2`
- `yahboomcar_point`

## Build

From the ROS 2 workspace root, with this repository cloned under `src/physical_rosmaster`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To verify package discovery from this repository boundary:

```bash
colcon list --base-paths physical_rosmaster
```

## Odometry Status

The current physical X3 odometry path needs correction and validation.

Current flow:

- `yahboomcar_bringup/Mcnamu_driver_X3.py` subscribes to `cmd_vel`.
- It sends commands to the robot through `Rosmaster_Lib.Rosmaster.set_car_motion(...)`.
- It publishes `vel_raw` using values returned by `Rosmaster_Lib.Rosmaster.get_motion_data()`.
- `yahboomcar_base_node/src/base_node_X3.cpp` integrates `vel_raw` into `/odom_raw`.
- `robot_localization` fuses `/odom_raw` and `/imu/data`, remapping `/odometry/filtered` to `/odom`.

The local source does not prove that `/odom_raw` is computed from four wheel encoder position deltas. The next step is to inspect the exact `Rosmaster_Lib` used on the robot and determine what `get_motion_data()` returns.

Known immediate issue:

- `base_node_X3.cpp` assigns `odom.twist.twist.linear.y` and then overwrites it with `0.0`, suppressing lateral mecanum velocity in the published odometry twist.

## Development Notes

Working notes and task tracking live in `agents/`:

- `agents/rosmaster_physical_audit.md`
- `agents/physical_rosmaster_todo.md`

## Publishing Caveat

Most package manifests still contain `TODO: Package description` and `TODO: License declaration`. Treat this as a private or internal repo until package provenance and licensing are cleaned up.
