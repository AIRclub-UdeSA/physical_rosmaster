# Robot-side verification TODO

Use this checklist on one ROSMASTER X3 before starting any autostart work. Every
gate is mandatory unless it is marked optional. Record commands, logs, bag paths,
and measured results in the deployment record for the tested robot.

This checklist targets one exact reviewed branch head containing runtime
commit `a08b097`. Its robot-side validation is pending, and autostart remains
blocked.

## Test record

- [ ] Robot identifier: `________________`
- [ ] Tester and observer: `________________`
- [ ] Date, surface, payload, and battery state: `________________`
- [ ] Reviewed deployment SHA: `________________`
- [ ] Repository commit from `git rev-parse HEAD` (must contain `a08b097`): `________________`
- [ ] `Rosmaster_Lib` path and SHA256: `________________`
- [ ] Astra model and serial: `________________`
- [ ] Astra physical topology and cold/warm boot evidence: `________________`
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
- lifted active-link loss does not demonstrate a bounded physical stop and no
  validated controller/firmware or independent hardware watchdog is present;
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

Do not begin this deployment until the reviewed branch containing `a08b097`
has been published. Both cleanliness checks and all three revision checks below
must succeed after replacing the reviewed-SHA marker.

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
test -z "$(git status --porcelain)"
git fetch origin
git checkout platform/simulator-parity
git pull --ff-only
test -z "$(git status --porcelain)"
git rev-parse HEAD
git rev-parse origin/platform/simulator-parity
test "$(git rev-parse HEAD)" = \
  "$(git rev-parse origin/platform/simulator-parity)"
git merge-base --is-ancestor a08b097 HEAD
test "$(git rev-parse HEAD)" = "REPLACE_WITH_REVIEWED_FULL_SHA"

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
- [ ] Hash the installed `Rosmaster_Lib.py` and require the exact supported
  digest before starting the driver.

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

The only supported public V3.3.9 `Rosmaster_Lib.py` SHA256 is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
Commit `a08b097` checks this digest after import and refuses a mismatch. This is
a compatibility/version gate for the reviewed implementation, not supply-chain
attestation: it does not prove the file's provenance, trusted installation, or
the integrity of the surrounding host.

After the V3.3.9 constructor returns, the runtime configures and validates a
`0.05 s` pyserial `write_timeout` and observes exceptions and short writes that
the vendor methods otherwise swallow. The constructor's UART-servo
torque-enable write occurs before this wrapper exists. A completed host write
is not an MCU acknowledgement and does not prove physical stop.

## 3. Identify and stabilize every device

Run discovery while competing processes are stopped:

```bash
lsusb
lsusb -t
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
ros2 run astra_camera list_devices_node
```

- [ ] Record the motor-controller vendor/product IDs and stable identity. Prefer
  a unique serial; if the controller exposes none, dedicate and record its
  physical USB port.
- [ ] Record the Slamtec A1 adapter vendor/product IDs and unique serial identity.
- [ ] Confirm the camera is an Astra-family Orbbec device and record its exact
  model and serial.
- [ ] On `x3-c`, keep the Astra on downstream port 4 of the powered Yahboom hub.
  Confirm both functions, `2bc5:060f` depth and `2bc5:050f` UVC/RGB, appear on
  one cold boot and three consecutive warm boots without touching any cable.
- [ ] Treat the port-4 result as `x3-c`-specific evidence, not a universal port
  rule; independently record and boot-validate the topology of every other X3.
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
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
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

### Required-device absence at startup

Perform each startup test separately, with the wheels secured and no command
publisher:

- [ ] Launch without the camera and confirm strict bringup exits after the camera
  startup deadline.
- [ ] Launch without the LiDAR and confirm strict bringup exits clearly.
- [ ] Launch with the motor controller absent from startup. Confirm the driver
  never becomes healthy, exits after its startup-feedback deadline, and strict
  bringup drains.
- [ ] After each startup-absence test, restore all hardware, start a clean
  strict launch, and pass the full physical contract before continuing.

Every new X3 must run all three startup-absence checks. For the current
motor-only `x3-c` remediation, the previously accepted camera and LiDAR checks
need not be repeated unless deployment or positive-contract evidence regresses;
motor absence, both restorations, and live motor loss remain mandatory.

### Observer-only live motor-controller loss

This is a distinct no-command test, not a continuation of motor-absent startup.
Keep all wheels secured and ensure joystick, keyboard, calibration, and project
behavior are stopped. After a full healthy stationary baseline, run:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
mkdir -p /root/rosmaster-recovery-evidence
python3 tools/motor_live_loss_probe.py \
  --device "$ROSMASTER_MOTOR_PORT" \
  --confirm-wheels-secured \
  --output /root/rosmaster-recovery-evidence/motor-live-loss.json
```

- [ ] The probe records a complete, fresh, stationary baseline for all required
  controller-derived topics.
- [ ] No `/cmd_vel` publisher or `/cmd_vel` message appears in any phase.
- [ ] After the controller device disappears, controller-derived topics become
  quiet within their deadlines.
- [ ] Motor diagnostics publish `ERROR` with structured failed freshness
  evidence.
- [ ] `driver_node` exits, strict bringup drains, and the strict graph remains
  stably drained for the probe's observation window.
- [ ] Archive `motor-live-loss.json` with the test record.
- [ ] Restore the controller, start a clean strict launch, and pass the full
  physical contract again.

Passing this observer-only gate proves ROS data, diagnostics, process exit, and
graph drain. It does **not** prove physical stop when link loss occurs during
active motion.

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

Only after the observer-only live-loss gate passes, run a separate active safety
gate. Secure all four wheels, use one very-low-speed command source, keep an
observer beside the robot, and keep the main power switch immediately reachable.
Predeclare the maximum acceptable physical stop time and record wheel motion,
commands, diagnostics, and any power intervention.

First interrupt the command stream with the controller link healthy and measure
the watchdog stop. Then repeat at very low speed while deliberately removing
the controller link. Cut the main switch immediately if behavior is unexpected
or the declared bound is exceeded. A successful host serial write only means
the host accepted the frame; it does not establish controller execution.

- [ ] Forward `+X`: FL, FR, BL, BR encoder deltas are `+ + + +`; `/odom` moves
  in `+X`.
- [ ] Left strafe `+Y`: encoder deltas are `- + + -`; `/odom` moves in `+Y`.
- [ ] CCW `+Z`: encoder deltas are `- + - +`; `/odom` yaw increases.
- [ ] Position stays continuous after stopping and odometry stops when encoders
  stop.
- [ ] No normal trial reports a stale encoder or discontinuity diagnostic.
- [ ] Interrupt a low-speed command stream while lifted and verify the motor
  watchdog commands zero and the wheels physically stop within the declared
  bound.
- [ ] During securely lifted very-low-speed motion, remove the controller link
  and verify bounded physical stop with the main switch continuously reachable.
- [ ] If active-link loss does not demonstrate bounded physical stop, stop the
  gate and require a validated controller/firmware or independent hardware
  watchdog before any floor use.
- [ ] Joystick publishes motion only while the configured deadman is held.
- [ ] Joystick release, malformed input, input loss, and shutdown each produce a
  stop.
- [ ] Keyboard input timeout and shutdown each produce a stop.
- [ ] Calibration processes publish no motion while `start_test=false`.
- [ ] Only one `/cmd_vel` publisher was present during every individual test.

## 6. Pass bounded floor trials

Move to a clear, level test area. Use a spotter, conservative speeds, one command
source, and external distance/heading measurements.

- [ ] The lifted active-link-loss gate demonstrated bounded physical stop, or a
  controller/firmware or independent hardware watchdog has been installed and
  validated. Otherwise floor use remains prohibited.

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

Robot validation of `a08b097` is pending. Autostart remains blocked until this
checklist passes on one X3. The next task after acceptance is to design a
versioned, failure-propagating autostart routine that invokes the single strict
bringup and never starts operator tools or project behavior by default.
