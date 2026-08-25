# Robot-side verification TODO

Use this checklist on one ROSMASTER X3 before starting any autostart work. Every
gate is mandatory unless it is marked optional. Record commands, logs, bag paths,
and measured results in the deployment record for the tested robot.

## Test record

- [ ] Robot identifier: `________________`
- [ ] Tester and observer: `________________`
- [ ] Date, surface, payload, and battery state: `________________`
- [ ] Repository commit from `git rev-parse HEAD`: `________________`
- [ ] `Rosmaster_Lib` path and SHA256: `________________`
- [ ] Astra model and serial: `________________`
- [ ] Motor and LiDAR stable identities: `________________`
- [ ] ROS bag and log directory: `________________`
- [ ] External consumer repository and commit: `________________`

## Stop conditions

Stop the test and leave autostart blocked if any of these occurs:

- required motor, LiDAR, or Astra hardware is absent or cannot be selected by a
  stable identity;
- strict bringup continues with a missing required sensor;
- default bringup has a `/cmd_vel` publisher;
- encoder signs, motion direction, watchdog behavior, or stop behavior is wrong;
- `/odom`, TF, sensor frames, camera calibration, metric depth, or diagnostics
  fail the physical contract;
- any command source remains active unexpectedly or the operator loses a safe
  stop path.

## 1. Make the robot safe

- [ ] Disable the old host/container autostart routine.
- [ ] Confirm no old ROS graph or process owns the serial, USB, camera, or
  `/cmd_vel` interfaces.
- [ ] Put source backups outside `/root/yahboomcar_ws` so `colcon` cannot discover
  duplicate packages.
- [ ] Lift and secure all four wheels for non-motion and initial motion tests.
- [ ] Establish a tested stop path and keep an observer beside the robot.
- [ ] Charge the battery and inspect the wheels, rollers, cables, and loose
  payloads before commanding motion.

## 2. Install and verify the exact source

Inside the ROS 2 Humble container:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git fetch origin
git checkout platform/simulator-parity
git pull --ff-only
git rev-parse HEAD

cd /root/yahboomcar_ws/src
vcs import . < physical_rosmaster/physical_rosmaster.repos

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install
colcon test --packages-select \
  laserscan_to_point_pulisher sllidar_ros2 yahboomcar_astra \
  yahboomcar_base_node yahboomcar_bringup yahboomcar_ctrl \
  yahboomcar_description yahboomcar_visual
colcon test-result --verbose
source install/setup.bash
```

- [ ] The physical repository reports exactly the eight documented local
  packages.
- [ ] The external workspace contains `astra_camera` and `astra_camera_msgs` at
  the commit pinned in `physical_rosmaster.repos`.
- [ ] The complete Humble workspace builds without removed navigation, SLAM,
  localization, or EKF dependencies.
- [ ] Tests have no failures. Record any intentional skips.
- [ ] `python3 -c "from Rosmaster_Lib import Rosmaster; print(Rosmaster)"` passes.
- [ ] Hash the installed `Rosmaster_Lib` and compare it with the validated robot
  deployment record before using the encoder mapping.

```bash
python3 - <<'PY'
import hashlib
import inspect
from Rosmaster_Lib import Rosmaster

path = inspect.getsourcefile(Rosmaster)
with open(path, "rb") as source:
    digest = hashlib.sha256(source.read()).hexdigest()
print(path)
print(digest)
PY
```

The previously validated public V3.3.9 hash is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
Treat a mismatch as a review item, not as permission to assume the encoder API
and mapping are unchanged.

## 3. Identify and stabilize every device

Run discovery while competing processes are stopped:

```bash
lsusb
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
ros2 run astra_camera list_devices_node
```

- [ ] Record the motor-controller vendor/product IDs and unique serial identity.
- [ ] Record the Slamtec A1 adapter vendor/product IDs and unique serial identity.
- [ ] Confirm the camera is an Astra-family Orbbec device and record its exact
  model and serial.
- [ ] If no Orbbec device is detected, stop: simulator parity has failed.
- [ ] Replace every placeholder in
  `config/99-rosmaster-x3.rules.example` with observed values.
- [ ] Install and reload the motor/LiDAR rules and the pinned Orbbec driver's USB
  permission rules on the host.
- [ ] Reconnect the devices and verify their stable names after a host reboot.
- [ ] Verify all three devices and permissions inside the container.
- [ ] Set and record `ROSMASTER_MOTOR_PORT`, `ROSMASTER_LIDAR_PORT`, and
  `ROSMASTER_ASTRA_SERIAL`.

## 4. Pass the strict non-motion gate

Keep the wheels secured and start only the platform:

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

In a second shell:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

- [ ] Strict bringup stays running with all required hardware present.
- [ ] The physical contract probe passes without exceptions or weakened checks.
- [ ] Default bringup has no `/cmd_vel` publisher.
- [ ] Every required sensor, `/joint_states`, and `/odom` has exactly one
  publisher with the expected type and compatible QoS.
- [ ] Controller and wheel-odometry `/diagnostics` are healthy.
- [ ] `/joint_states` contains finite position and velocity for the four canonical
  wheel joints.
- [ ] `/odom` is finite, uses `odom` and `base_footprint`, and is the only source
  of `odom -> base_footprint`.
- [ ] `/imu/data` is finite in `imu_link`; Madgwick owns no TF; raw IMU and
  magnetometer extensions remain available.
- [ ] `/scan` is finite where valid, uses `laser_link`, and preserves the cable
  mask; `/scan_filtered` exposes rejected cable/self-return points.
- [ ] Color is calibrated `rgb8`; depth is calibrated metric `32FC1`; the cloud
  contains XYZRGB in `cam_1_depth_frame`.
- [ ] `odom` resolves to every sensor frame at the timestamp of sampled messages.
- [ ] With no motion commands and stationary wheels, encoder positions and
  `/odom` remain stable.

Fail-closed checks, performed one device at a time with no command publisher:

- [ ] Launch without the camera and confirm strict bringup exits after the camera
  startup deadline.
- [ ] Launch without the LiDAR and confirm strict bringup exits clearly.
- [ ] Launch without motor-controller feedback and confirm the driver/bringup
  exits after sustained read failure.
- [ ] Restore all hardware and repeat the contract probe successfully.

## 5. Pass lifted motion and operator-safety gates

Keep all wheels securely lifted. Start a bag before publishing motion:

```bash
ros2 bag record \
  /cmd_vel /joint_states /odom /tf /diagnostics /vel_raw /voltage /rosout
```

Run one bounded axis at a time from the repository root:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --y 0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --yaw 0.30 --duration 1.0 --require-recorder
```

With the wheels still lifted, exercise the driver watchdog by interrupting a
low-speed stream without sending a final zero:

```bash
timeout 1s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}}"
```

The `timeout` command normally returns nonzero because it deliberately terminates
the publisher. Observe `/cmd_vel`, `/diagnostics`, and wheel motion long enough to
confirm the driver stops after its configured command timeout.

- [ ] Forward `+X`: FL, FR, BL, BR encoder deltas are `+ + + +`; `/odom` moves
  in `+X`.
- [ ] Left strafe `+Y`: encoder deltas are `- + + -`; `/odom` moves in `+Y`.
- [ ] CCW `+Z`: encoder deltas are `- + - +`; `/odom` yaw increases.
- [ ] Position stays continuous after stopping and odometry stops when encoders
  stop.
- [ ] No normal trial reports a stale encoder or discontinuity diagnostic.
- [ ] Interrupt a low-speed command stream while lifted and verify the motor
  watchdog commands zero within its configured timeout.
- [ ] Joystick publishes motion only while the configured deadman is held.
- [ ] Joystick release, malformed input, input loss, and shutdown each produce a
  stop.
- [ ] Keyboard input timeout and shutdown each produce a stop.
- [ ] Calibration processes publish no motion while `start_test=false`.
- [ ] Only one `/cmd_vel` publisher was present during every individual test.

## 6. Pass bounded floor trials

Move to a clear, level test area. Use a spotter, conservative speeds, one command
source, and external distance/heading measurements.

- [ ] Record battery voltage, surface, payload, command, duration, measured
  displacement/heading, encoder deltas, and `/odom` delta for every trial.
- [ ] Repeat longitudinal `+X` and `-X` trials at least three times each.
- [ ] Repeat left and right lateral trials at least three times.
- [ ] Repeat CW and CCW rotation trials at least three times.
- [ ] Direction and approximate scale are qualitatively correct in every axis.
- [ ] Watchdog and operator stops remain reliable under floor load.
- [ ] No unexpected reset, non-finite value, encoder discontinuity, TF conflict,
  or sustained diagnostic error occurs.
- [ ] Do not change geometry, CPR, scale, or covariance from a single trial;
  document repeated systematic error separately from mecanum slip.

## 7. Prove simulator/physical consumer parity

- [ ] Select one minimal external consumer or project repository and record its
  commit.
- [ ] Run it against simulator commit `772ba25` using the public contract.
- [ ] Run the same code against the physical X3.
- [ ] Use no topic remaps, frame substitutions, hardware-only branches, or source
  changes between the two runs.
- [ ] Confirm the consumer receives `/joint_states`, `/odom`, TF, filtered IMU,
  scan, calibrated RGB-D images, and XYZRGB points as expected.
- [ ] Record any simulator-only `/clock` or ground-truth use and confirm it is not
  required by the portable consumer.

## 8. Acceptance and handoff

- [ ] Archive terminal logs, contract-probe output, ROS bags, device identities,
  udev rules, environment configuration, measurements, and failure observations.
- [ ] File issues for every unresolved failure; do not hide exceptions in local
  launch edits or uncommitted robot state.
- [ ] A second reviewer confirms all required boxes and evidence.
- [ ] Mark this X3 as the first accepted platform only after sections 1–7 pass.

Autostart remains blocked until this checklist passes on one X3. The next task
after acceptance is to design a versioned, failure-propagating autostart routine
that invokes the single strict bringup and never starts operator tools or project
behavior by default.
