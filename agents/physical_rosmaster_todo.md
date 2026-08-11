# Physical ROSMASTER TODO

Date: 2026-08-11
Decision: keep the physical robot code separate from the simulator repo. The physical workspace should become one repo named `physical_rosmaster`.

## Done

- [x] Read the setup guide and capture the current deployment model.
- [x] Do a high-level package pass over the physical `src`.
- [x] Compare the physical odometry path with the simulator repo.
- [x] Record the first audit in `agents/rosmaster_physical_audit.md`.
- [x] Decide not to split every package into separate repos.
- [x] Decide to keep simulator and physical code separate.
- [x] Add a focused X3 odometry regression test and fix the local lateral twist publication bug.
- [x] Download and inspect public Yahboom `Rosmaster_Lib` V3.3.9 reference without vendoring it.
- [x] Add workstation-vs-robot workflow docs for cloning, building, and validating on the robot.
- [x] Add a robot-side `Rosmaster_Lib` probe script for hash and encoder/motion sampling.
- [x] Add a GitHub Release based restore workflow for optional large SLAM/PCD artifacts.

## Phase 1: Create The Physical Repo Boundary

- [x] Move the current package directories into one top-level folder named `physical_rosmaster`.
- [x] Keep `agents/` available for working notes, either at repo root or copied into `physical_rosmaster/agents`.
- [x] Verify `colcon list --base-paths physical_rosmaster` still discovers all expected packages.
- [x] Add a root `.gitignore` for ROS 2 generated state: `build/`, `install/`, `log/`, `.colcon/`, Python caches, editor files, bags, and local temp outputs.
- [x] Add a root `README.md` explaining that this is the physical ROSMASTER X3 workspace, separate from `AIRclub-UdeSA/yahboom_rosmaster`.
- [x] Add provenance notes: Yahboom-derived packages, local modifications, added `sllidar_ros2`, ignored packages, Docker/Humble target.
- [x] User approved making `AIRclub-UdeSA/physical_rosmaster` public; keep README provenance/license caveat visible because upstream Yahboom repository has no GitHub-detected license.
- [ ] Clean up package license policy. Most local packages still say `TODO: License declaration`, so public visibility should not be interpreted as a clean repository-wide open-source license.
- [x] Initialize Git in `physical_rosmaster`.
- [x] Create the GitHub repo as private: `AIRclub-UdeSA/physical_rosmaster`.
- [x] Push an initial snapshot branch after `.gitignore`, README, and provenance notes exist.

## Phase 2: Inspect `Rosmaster_Lib`

- [ ] Locate the exact `Rosmaster_Lib` used on the robot host and copied into the container; compare its SHA256 against the public V3.3.9 reference.
- [x] Confirm `Rosmaster_Lib` is not vendored in this repo and is not importable on this workstation.
- [x] Check public Yahboom driver docs for motion/encoder APIs; docs list `get_motion_data()` and `get_motor_encoder()`.
- [x] Inspect public V3.3.9 `Rosmaster_Lib.Rosmaster.get_motion_data()`.
- [x] Determine that public V3.3.9 `get_motion_data()` returns cached firmware/controller speed feedback from `FUNC_REPORT_SPEED`, not a direct Python echo of `cmd_vel`.
- [x] Search public V3.3.9 `Rosmaster_Lib` for encoder/tick APIs; `get_motor_encoder()` returns four cached signed 32-bit motor encoder counters.
- [x] Record the public V3.3.9 serial packet fields used for motion and encoder feedback in `agents/rosmaster_lib_public_v3_3_9.md`.
- [ ] Determine whether the firmware speed packet itself is encoder-derived, command-derived, IMU-assisted, or another controller estimate.
- [ ] Run a hardware check comparing `/cmd_vel` and `vel_raw` while:
  - robot is lifted,
  - robot is on the floor,
  - one wheel is resisted or slipping,
  - `/cmd_vel` is stopped.
- [ ] Decide whether real encoder-position odometry is possible from exposed data.

## Phase 3: Fix Immediate Physical Odometry Bugs

- [x] Add a focused test or small replay harness for `base_node_X3` odometry behavior.
- [x] Fix `yahboomcar_base_node/src/base_node_X3.cpp` so `odom.twist.twist.linear.y` is not overwritten to `0.0`.
- [x] Initialize `last_vel_time_` correctly so the first `dt` is not invalid.
- [x] Use node clock consistently instead of constructing a fresh `rclcpp::Clock`.
- [x] Respect the declared `odom_frame` and `base_footprint_frame` parameters instead of hardcoding frame strings.
- [ ] Add a velocity timeout/safe behavior if `vel_raw` stops.
- [ ] Review covariance values. If odom is velocity-integrated and not encoder-position-derived, use covariance that reflects that uncertainty.
- [x] Rebuild `yahboomcar_base_node` and run the focused X3 odometry regression locally.
- [ ] Verify `/odom_raw` twist and pose on the robot during forward, strafe, and rotate commands.

## Phase 4: Implement Correct Real Odometry If Hardware Allows

- [ ] If wheel encoder positions are available, implement a real wheel-state odometry node for the physical robot.
- [ ] Reuse the simulator math as the reference contract: four wheel deltas -> mecanum chassis delta -> `/odom`.
- [ ] Publish or consume real `JointState` with four physical wheel joints and meaningful positions/velocities.
- [ ] Decide whether the encoder odometry node should replace `base_node_X3` or live beside it as a new physical package.
- [ ] Keep EKF ownership clear: either raw odom publishes `/odom_raw` and EKF publishes `/odom`, or a single odom source owns `odom -> base_footprint`.
- [ ] If only chassis velocity feedback is available, rename/document it as firmware velocity odometry, not wheel encoder odometry.

## Phase 5: Bring Physical And Sim Closer Together

- [ ] Define a shared topic/frame contract for both repos:
  - `/cmd_vel`
  - `/joint_states`
  - `/odom`
  - `/imu/data`
  - `/scan`
  - `/tf`
  - `odom -> base_footprint -> base_link -> laser_link/imu_link/camera_link`
- [ ] Compare physical URDF dimensions against the simulator description.
- [ ] Align wheel names and mecanum geometry parameters where possible.
- [ ] Align lidar frame naming. The physical guide alternates between `radar_Link`, `laser`, and `laser_link`; choose one.
- [ ] Align IMU topic flow: physical driver publishes `/imu/data_raw`, Madgwick publishes `/imu/data`, EKF consumes `/imu/data`.
- [ ] Add a small contract test checklist that can be run on either sim or real robot.
- [ ] Keep docs cross-linked, but keep repos separate: simulator users should not need physical robot dependencies.

## Phase 6: Physical Bringup Hardening

- [ ] Replace `/dev/ttyUSB0`, `/dev/ttyUSB1`, and `/dev/video0` with udev symlinks such as `/dev/robot/lidar`, `/dev/robot/camera_front`, and motor/IMU names where applicable.
- [ ] Put udev rules and installation instructions in the physical repo.
- [ ] Move autostart scripts from the guide into versioned files.
- [ ] Decide whether to keep Docker-only deployment or add a cleaner host/systemd launch path.
- [ ] Add startup checks for required devices before launching lidar/camera/driver nodes.
- [ ] Add graceful shutdown behavior that sends zero velocity before stopping the driver stack.
- [ ] Add log rotation notes for long-running robot use.

## Phase 7: Cleanup And Publishability

- [ ] Replace package `TODO` descriptions with useful package descriptions.
- [ ] Replace package `TODO` licenses after provenance/licensing is decided.
- [ ] Declare missing package dependencies in `package.xml` files, especially Python runtime deps.
- [ ] Remove notebook checkpoints, unused scratch files, and generated files from the tracked repo.
- [ ] Decide what to do with ignored packages:
  - keep ignored and document,
  - remove from repo,
  - or repair later.
- [x] Document how to restore ignored large artifacts from GitHub Releases.
- [ ] Add a minimal CI build job if feasible for ROS 2 Humble.
- [ ] Tag the first known-working physical snapshot after robot validation.

## Open Questions

- Does the motor controller expose actual per-wheel encoder positions?
- Is `Rosmaster_Lib.get_motion_data()` based on encoder feedback or just the requested chassis command?
- Do we want `physical_rosmaster` package names to stay as Yahboom names initially, or migrate gradually to `physical_rosmaster_*` names?
- Should the initial GitHub repo be private until licensing is cleaned up?
