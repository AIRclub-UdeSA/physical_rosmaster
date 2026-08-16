# Rosmaster Physical Workspace Audit

Date: 2026-08-11
Workspace: `/home/juan/Documents/rosmaster_physical/src/physical_rosmaster`
Setup guide: `/home/juan/Downloads/Guia ros2 humble y autostart.md`
Simulator repo compared: `AIRclub-UdeSA/yahboom_rosmaster`

## Guide Summary

- Robots are Yahboom ROSMASTER X3 mecanum units running ROS 2 Humble inside a Docker container named `rosmaster_humble`.
- Container image used in the guide: `yahboomtechnology/ros-humble:4.1.2`, with host networking, privileged mode, `/dev` mounted, and `ROS_DOMAIN_ID` set per robot.
- `Rosmaster_Lib` is copied from the host into `/root/Rosmaster_Lib` in the container and made importable through a `.pth` file. `pyserial` is installed with `pip3`.
- The guide replaces the container workspace source with this custom `src`, then builds `/root/yahboomcar_ws` with `colcon build --symlink-install`.
- The guide says this source was modified to add `sllidar_ros2`, filter/remove lidar returns from robot structure, and make odom TF work.
- Packages intentionally ignored with `COLCON_IGNORE`: `yahboomcar_KCFTracker`, `robot_pose_publisher_ros2`, `yahboomcar_point`.
- Autostart is implemented as host systemd one-shot service `rosmaster-autostart.service` -> `/usr/local/bin/start_rosmaster.sh` -> `docker exec -d ... /root/auto_start.sh`.
- Current autostart guide uses unstable device names: `/dev/video0`, `/dev/ttyUSB1`, fallback `/dev/ttyUSB0`. This should become udev-backed names before treating the robot as production.

## Physical Workspace Shape

- This repo boundary is now `/home/juan/Documents/rosmaster_physical/src/physical_rosmaster`.
- Before this boundary was created, packages lived directly under `/home/juan/Documents/rosmaster_physical/src`.
- `colcon list` sees 19 buildable packages:
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
- The repo is roughly 314 MB. Largest directories observed:
  - `yahboomcar_slam` around 177 MB
  - `yahboomcar_description` around 104 MB
  - `yahboomcar_description_x1` around 15 MB
  - `yahboomcar_visual` around 14 MB
- Publishing readiness issue: nearly every local package has `TODO: Package description` and `TODO: License declaration`; only vendored `sllidar_ros2` has a real license file and BSD license metadata.
- `colcon list` created a generated `log/` directory in this `src`; ignore this in Git.

## Simulator Repo Shape

- Cloned for comparison into `/tmp/yahboom_rosmaster_airclub.Gqmqip` during this pass.
- Simulator repo has 9 curated packages:
  - `yahboom_rosmaster` metapackage
  - `yahboom_rosmaster_bringup`
  - `yahboom_rosmaster_description`
  - `yahboom_rosmaster_docking`
  - `yahboom_rosmaster_gazebo`
  - `yahboom_rosmaster_localization`
  - `yahboom_rosmaster_msgs`
  - `yahboom_rosmaster_navigation`
  - `yahboom_rosmaster_system_tests`
- Simulator package metadata is substantially cleaner: named packages, BSD-3-Clause license, descriptions, root README, `.gitignore`, package-local LICENSE files.
- Its README states `/odom` is integrated from `/joint_states` wheel joint positions by `yahboom_rosmaster_gazebo/scripts/wheel_state_odometry.py`, and `odom -> base_footprint` is published by that same node.

## Odometry Finding

Physical X3 launch path:

- `yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py` launches:
  - `Mcnamu_driver_X3`
  - `base_node_X3`
  - `imu_filter_madgwick_node`
  - `ekf_x1_x3_launch.py`
  - `yahboom_joy_X3`
- `Mcnamu_driver_X3.py` subscribes to `cmd_vel`, sends those values to hardware with `self.car.set_car_motion(vx, vy, angular)`, then publishes `vel_raw` from `self.car.get_motion_data()`.
- `base_node_X3.cpp` subscribes to `vel_raw`, integrates that Twist directly into `x_pos_`, `y_pos_`, and `heading_`, then publishes `/odom_raw`.
- `ekf_x1_x3.yaml` fuses `/odom_raw` and `/imu/data`, remapping `/odometry/filtered` to `/odom`.

Conclusion:

- The colleague is directionally right: physical `/odom_raw` is not computed from four wheel encoder position deltas in this codebase. It is velocity integration from `vel_raw`.
- Whether `vel_raw` is merely echoing commanded velocity or a firmware-estimated velocity depends on `Rosmaster_Lib.get_motion_data()`, which is outside this repository. The local source does not show wheel tick/position/encoder delta odometry.
- Fixed locally: the X3 odometry message path now preserves lateral `linear.y`, initializes `last_vel_time_`, uses the node clock, and uses the declared `odom_frame` / `base_footprint_frame` parameters for odometry and TF.
- Still open: there is no timeout/safe stop in `base_node_X3.cpp` if `vel_raw` stops arriving.
- Public Yahboom docs list both `get_motion_data()` and `get_motor_encoder()`, and on `x3-c` the deployed `Rosmaster_Lib` has now been verified to match public V3.3.9. The remaining question is whether the exposed encoder counters are stable and useful enough for odometry.

Simulator contrast:

- `wheel_state_odometry.py` subscribes to `/joint_states`, extracts four wheel joint positions, computes deltas, rejects discontinuities, applies mecanum kinematics, integrates midpoint pose, publishes `/odom`, and publishes `odom -> base_footprint`.
- That structure is the better reference for real/sim parity, if the real board can expose wheel encoder positions or wheel angular velocities.

## GitHub Recommendation

Recommended path: one curated monorepo first, not many separate repos now.

Rationale:

- This source is a vendor-derived robot workspace with many under-declared package dependencies and TODO metadata.
- Splitting into multiple repos before cleanup would multiply packaging, licensing, CI, and release overhead.
- The simulator repo already uses the more maintainable package naming and grouping pattern. The physical stack should either join that repo under a clear physical/hardware package family or become one sibling repo with similarly curated package names.

Practical options:

1. Best long-term: extend `AIRclub-UdeSA/yahboom_rosmaster` with real-robot packages such as `yahboom_rosmaster_hardware`, `yahboom_rosmaster_physical_bringup`, and shared `description/navigation/localization` where compatible.
2. Safer first upload: create a new private repo for this exact physical `src` snapshot, with root README, `.gitignore`, clear provenance, package metadata cleanup, and issues tracking the odometry/udev/licensing work.
3. Avoid for now: splitting individual Yahboom packages into many GitHub repos.

## Next Recommended Work

1. Add root `.gitignore` and README/provenance notes.
2. Decide whether to preserve vendor package names initially or migrate toward `yahboom_rosmaster_*`.
3. Validate the fixed X3 odometry publication on the physical robot during forward, strafe, and rotate commands.
4. Determine from the verified deployed `Rosmaster_Lib` whether encoder positions are available and stable enough for odometry. If yes, implement real wheel-state odometry matching simulator semantics. If no, label current odom as firmware/command-velocity odom with higher covariance and do not treat it as encoder odometry.
5. Replace `/dev/ttyUSB*` and `/dev/video0` in guide/autostart with udev symlinks.
6. Add minimal build/launch tests for bringup and odometry math before pushing public code.
