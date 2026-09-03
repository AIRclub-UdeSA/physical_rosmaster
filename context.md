# Current engineering context

This is the physical ROSMASTER X3 hardware platform. Its design target is simulator parity at the robot-facing boundary, not a complete robot application.

## Non-negotiable boundary

Default bringup may provide drivers, sensors, preprocessing, wheel odometry, TF, and hardware health. It must not provide autonomous behavior, localization, mapping, navigation, an EKF, tracking, following, avoidance, or any `/cmd_vel` publisher.

External projects own all behavior and run on top of the same public topics and frames used by simulator commit `772ba25`.

## Current Git state

- validated pre-cleanup baseline: `aafed44`;
- recovery tag: `pre-platform-contract-cleanup`;
- implementation branch: `platform/simulator-parity`;
- autostart: blocked until robot-side acceptance.

## Runtime graph

- `Mcnamu_driver_X3.py` owns motor I/O, `/cmd_vel` subscription, watchdog, `/joint_states`, raw IMU/magnetometer, `/vel_raw`, voltage, firmware telemetry, and controller health on `/diagnostics`.
- `base_node_X3.cpp` consumes only canonical wheel encoders and publishes `/odom`, `odom -> base_footprint`, and encoder health on `/diagnostics`. There is no `/vel_raw` fallback.
- `robot_state_publisher` expands the one X3 Xacro. It owns platform transforms through `cam_1_link`.
- `imu_filter_madgwick` converts `/imu/data_raw` to `/imu/data`, with `use_mag=false` and `publish_tf=false`.
- `sllidar_ros2` publishes cable-masked `/scan` in `laser_link`; rejected physical returns remain visible on `/scan_filtered`.
- the pinned Orbbec driver runs below `/_hardware/astra`; `yahboomcar_astra` publishes normalized public `/cam_1/...` topics.

Camera-internal calibrated transforms are owned by the Orbbec driver. The public depth image is `32FC1` metres, color is `rgb8`, and the XYZRGB cloud is transformed into `cam_1_depth_frame`.

## Safety invariants

- Default bringup has zero `/cmd_vel` publishers.
- Stale/discontinuous encoders stop integration, publish a zero twist state, and raise `/diagnostics`; they never select firmware velocity as another estimator.
- Sustained motor/IMU read failure terminates the driver.
- Missing or invalid RGB-D streams terminate strict camera bringup.
- Joystick motion requires a held deadman and stops on release, malformed input, timeout, and shutdown.
- Keyboard motion stops on input timeout and shutdown.
- Manual tools are capped at `0.20 m/s` and `1.0 rad/s`.
- Calibration starts inert, is bounded by target and time, and stops on TF failure.

## Local packages

Exactly eight packages remain: `yahboomcar_bringup`, `yahboomcar_base_node`, `yahboomcar_description`, `yahboomcar_ctrl`, `yahboomcar_astra`, `sllidar_ros2`, `yahboomcar_visual`, and `laserscan_to_point_pulisher`.

## Validation entry points

```bash
colcon list --base-paths . --names-only
colcon build --symlink-install
colcon test --packages-select yahboomcar_base_node
python3 tools/physical_contract_probe.py
```

The physical probe requires all public sensor topics, unique sensor/odometry publishers, healthy controller and encoder diagnostics, finite and increasing data, valid calibration, metric depth, XYZRGB fields, correct frames, odometry TF ownership, and time-resolvable `odom` → sensor transforms. It also verifies that default bringup has no `/cmd_vel` publisher.

## Hardware gate still outstanding

Workstation validation cannot establish the camera model/serial, USB/udev state, encoder signs on another unit, sensor calibration validity, or floor behavior. On one X3:

1. import `physical_rosmaster.repos` and install dependencies with `rosdep`;
2. identify motor, A1, and Astra stable identities;
3. run manual strict bringup and the non-motion probe;
4. run lifted and bounded floor tests from the current docs;
5. compare one consumer against the simulator without remaps.

Do not prepare or enable a new autostart routine before that gate passes.

## Historical evidence

Files under `agents/` describe the pre-cleanup architecture where `/odom_raw` fed an EKF. They are retained as validation evidence only. Do not copy their old launch graph or topic ownership into current work; see `agents/README.md`.
