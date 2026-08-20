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
- [x] Verify the installed `Rosmaster_Lib` on `x3-c` matches public V3.3.9 exactly.
- [x] Capture a stationary `Rosmaster_Lib` probe on `x3-c` with stable zero motion and stable encoder counters.
- [x] Add a GitHub Release based restore workflow for optional large SLAM/PCD artifacts.
- [x] Add root `context.md` for coding agents working inside the robot/container.
- [x] Consolidate the original ROS 2 Humble/autostart setup guide into repo docs.

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

- [x] Locate the exact `Rosmaster_Lib` used on the robot host and copied into the container; compare its SHA256 against the public V3.3.9 reference.
- [x] Confirm `Rosmaster_Lib` is not vendored in this repo and is not importable on this workstation.
- [x] Check public Yahboom driver docs for motion/encoder APIs; docs list `get_motion_data()` and `get_motor_encoder()`.
- [x] Inspect public V3.3.9 `Rosmaster_Lib.Rosmaster.get_motion_data()`.
- [x] Determine that public V3.3.9 `get_motion_data()` returns cached firmware/controller speed feedback from `FUNC_REPORT_SPEED`, not a direct Python echo of `cmd_vel`.
- [x] Search public V3.3.9 `Rosmaster_Lib` for encoder/tick APIs; `get_motor_encoder()` returns four cached signed 32-bit motor encoder counters.
- [x] Record the public V3.3.9 serial packet fields used for motion and encoder feedback in `agents/rosmaster_lib_public_v3_3_9.md`.
- [ ] Determine whether the firmware speed packet itself is encoder-derived, command-derived, IMU-assisted, or another controller estimate.
- [ ] Run a lifted hardware check showing the encoder counters and motion packet respond to `/cmd_vel` while:
  - robot is lifted,
  - `/cmd_vel` is stopped.
  - The 2026-08-20 powered checks showed encoder and motion-packet response on the floor; they do not complete this lifted check.
- [ ] Run the remaining hardware check comparing `/cmd_vel` and `vel_raw` while:
  - robot is lifted,
  - robot is on the floor,
  - one wheel is resisted or slipping,
  - `/cmd_vel` is stopped.
- [x] Decide whether real encoder-position odometry is possible from exposed data; the four counters are now published as wheel joint positions and consumed by `base_node_X3`.

## Phase 3: Fix Immediate Physical Odometry Bugs

- [x] Add a focused test or small replay harness for `base_node_X3` odometry behavior.
- [x] Fix `yahboomcar_base_node/src/base_node_X3.cpp` so `odom.twist.twist.linear.y` is not overwritten to `0.0`.
- [x] Initialize `last_vel_time_` correctly so the first `dt` is not invalid.
- [x] Use node clock consistently instead of constructing a fresh `rclcpp::Clock`.
- [x] Respect the declared `odom_frame` and `base_footprint_frame` parameters instead of hardcoding frame strings.
- [x] Add joint-state and firmware-velocity timeouts; publish zero twist without integrating when both sources become stale.
- [x] Make odometry covariance configurable for both pose and twist.
- [ ] Tune covariance values from repeated ground-truth floor runs.
- [x] Add a motor-command watchdog, command limits, repeated zero commands on shutdown, and a bounded `/cmd_vel` pulse tool.
- [x] Reject encoder counter wrap/reset discontinuities and make wheel channel order/signs configurable.
- [x] Validate the `x3-c` packet-field mapping with direction-controlled hand tests: `[FL, FR, BL, BR] = [m1, m3, m2, m4]`, with signs `[+, +, +, +]`.
- [x] Visually verify the powered-off motor wiring against Yahboom's diagram; no cable move was required.
- [x] Withdraw the proposed cable swaps: `Rosmaster_Lib`'s `m1..m4` names are packet positions, not evidence of PCB `M1..M4` port identity.
- [x] Rebuild and deploy corrected `encoder_order: [0, 2, 1, 3]`; subsequent floor observations matched all three expected wheel-sign patterns.
- [ ] Repeat forward, strafe, and rotate validation with the robot securely lifted.
- [x] Rebuild `yahboomcar_base_node` and run the focused X3 odometry regression locally.
- [x] Observe `/odom_raw` twist and pose during floor forward, strafe, and rotate commands; these observations are not ground-truth calibration.
- [ ] Verify `/odom_raw` twist and pose during a true lifted repetition.
- [ ] Resolve the large per-wheel magnitude imbalance and confirm `encoder_cpr` with exact marked rotations before ground-truth calibration.

## Phase 4: Implement Correct Real Odometry If Hardware Allows

- [x] Implement wheel-state odometry for the physical robot in `base_node_X3`.
- [x] Use four wheel deltas, mecanum chassis velocity, and midpoint pose integration as the physical raw-odometry path.
- [x] Publish and consume `JointState` with four physical wheel joints and meaningful positions/velocities.
- [x] Keep encoder odometry in `base_node_X3`, with `/vel_raw` retained only as a timed fallback.
- [x] Keep EKF ownership clear: raw odom publishes `/odom_raw`; EKF publishes `/odom` and owns `odom -> base_footprint` in normal bringup.
- [x] Continue publishing `/vel_raw` as explicitly documented firmware/controller velocity telemetry.

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
- [x] Add graceful driver shutdown behavior that sends repeated zero velocity commands before stopping.
- [ ] Add log rotation notes for long-running robot use.

## Incident Notes

- The 2026-08-16 `x3-c` setup incident surfaced a host-side `runc` corruption, a duplicate-package backup-tree mistake, a stale `install/` tree after a failed package build, and a missing-optional-artifact build assumption in `yahboomcar_slam`.
- The optional `pcl` install guard has been added so a clean clone can build without restoring the large artifact bundle first.

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

- Is `Rosmaster_Lib.get_motion_data()` based on encoder feedback or just the requested chassis command?
- What are the calibrated CPR, wheel radius, scale factors, and covariance values on the floor?
- Do we want `physical_rosmaster` package names to stay as Yahboom names initially, or migrate gradually to `physical_rosmaster_*` names?
