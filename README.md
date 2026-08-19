# Physical ROSMASTER

ROS 2 Humble source workspace for the physical Yahboom ROSMASTER X3 mecanum robots used by AIRclub UdeSA.

This repository contains the robot-side packages, launch files, descriptions, hardware drivers, navigation/SLAM configuration, and setup notes needed to run the real robot. It is meant to be cloned into a ROS 2 workspace, usually inside the `rosmaster_humble` Docker container on the robot.

The goal is repeatable robot preparation: whether a ROSMASTER is freshly built or already in service, the repo should give a clear path to clone, build, validate, and start it from the docs.

## Repository Status

- Target robot: Yahboom ROSMASTER X3 mecanum base
- Target ROS distro: ROS 2 Humble
- Main robot workspace: `/root/yahboomcar_ws`
- Expected robot clone path: `/root/yahboomcar_ws/src/physical_rosmaster`
- Docker image used in the current setup: `yahboomtechnology/ros-humble:4.1.2`
- Docker container name used in the current setup: `rosmaster_humble`

`Rosmaster_Lib` is not vendored in this repository. On the robot, it is copied into the container from the Yahboom host installation and exposed to Python.

## Relation To The Simulator

The simulator lives in a separate repository:

https://github.com/AIRclub-UdeSA/yahboom_rosmaster

Keep the repositories separate. Simulator users should not need physical robot dependencies, Docker setup, serial devices, camera/LiDAR hardware access, or Yahboom hardware libraries. The long-term goal is to make both repos share the same robot contract: topic names, frame names, wheel geometry, and odometry behavior.

## Quick Start: Workstation

Use a workstation for source review, Git work, documentation, and hardware-free tests.

```bash
mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd ~/rosmaster_physical_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install --packages-select yahboomcar_base_node
colcon test --packages-select yahboomcar_base_node --ctest-args -R test_x3_odometry
```

Full workspace builds may require the same ROS dependencies used on the robot. The focused `yahboomcar_base_node` test is the current hardware-free odometry regression check.

## Quick Start: Robot

Inside the robot's `rosmaster_humble` container:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash

cd /root/yahboomcar_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd /root/yahboomcar_ws
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install
source install/setup.bash
```

Before replacing an existing robot workspace, back up the old `src` tree. The full clone/build/autostart procedure is documented in `docs/setup_guide_ros2_humble_autostart.md`.

## Large Optional Artifacts

These files are intentionally excluded from Git:

- `yahboomcar_slam/params/ORBvoc.txt`
- `yahboomcar_slam/pcl/*.pcd`

Normal robot bringup, teleoperation, camera, LiDAR, IMU, EKF, and base odometry should not require them. Restore them only for ORB-SLAM-related workflows or point-cloud examples:

```bash
tools/fetch_large_artifacts.sh
```

Checksums and manual download instructions are in `docs/large_artifacts.md`.

The `yahboomcar_slam` package now skips the optional `pcl` install path when the directory is absent, so a clean clone can build the core workspace without restoring the large bundle first.

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

Packages present but ignored by `COLCON_IGNORE`:

- `yahboomcar_KCFTracker`
- `robot_pose_publisher_ros2`
- `yahboomcar_point`

## Odometry Status

The physical X3 odometry path calculates mecanum body velocities from wheel encoder feedback published on `/joint_states`, falling back to firmware velocity integration if joint states are unavailable.

Current flow:

- `yahboomcar_bringup/Mcnamu_driver_X3.py` subscribes to `/cmd_vel` and sends commands via `Rosmaster_Lib.Rosmaster.set_car_motion(...)`.
- `Mcnamu_driver_X3.py` polls `Rosmaster_Lib.Rosmaster.get_motor_encoder()` and publishes four-wheel positions and angular velocities on `/joint_states`.
- It also publishes `/vel_raw` from `Rosmaster_Lib.Rosmaster.get_motion_data()` for firmware speed telemetry.
- `yahboomcar_base_node/src/base_node_X3.cpp` consumes `/joint_states`, evaluates 4-wheel mecanum kinematics, and integrates body velocities into `/odom_raw`.
- `robot_localization` (EKF) fuses `/odom_raw` and `/imu/data`, publishing `/odom` and broadcasting `odom -> base_footprint`.

Validation status & checklist:

- Lifted tests verified encoder feedback and raw odometry calculation.
- Sign calibration and floor deadband validation are documented in `agents/x3-c_validation_checklist.md`.

Validation probe tools:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
python3 tools/rosmaster_lib_probe.py --samples 100 --period 0.1
```

See `agents/x3-c_validation_checklist.md` and `docs/odometry_validation.md` for the full test procedure.

## Troubleshooting

- `docs/troubleshooting/README.md`: incident-driven troubleshooting index and robot failure notes.

## Documentation

- `docs/setup_guide_ros2_humble_autostart.md`: ROS 2 Humble, Docker, clone/build, hardware tests, and autostart setup.
- `docs/workstation_and_robot_workflow.md`: how to work outside the robot versus inside the robot/container.
- `docs/odometry_validation.md`: plan for validating encoder counters, `/vel_raw`, `/odom_raw`, and EKF output.
- `docs/large_artifacts.md`: optional artifact bundle and checksums.
- `docs/troubleshooting/README.md`: incident-driven troubleshooting index.
- `context.md`: concise context for coding agents or engineers working inside the robot.
- `agents/physical_rosmaster_todo.md`: working task list.
- `agents/rosmaster_physical_audit.md`: initial repository audit.
- `agents/rosmaster_lib_public_v3_3_9.md`: notes from inspecting public Yahboom `Rosmaster_Lib`.

## License Caveat

This repository includes Yahboom-derived source and a vendored copy of `sllidar_ros2`. Many package manifests still contain `TODO: Package description` and `TODO: License declaration`. Public repository visibility should not be interpreted as a clean repository-wide open-source license until package provenance and licensing are cleaned up.