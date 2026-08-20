# X3-C Odometry & Motion Recovery Checklist

Date: 2026-08-20
Target: Physical Yahboom ROSMASTER X3 (`x3-c`)
Container: `rosmaster_humble` (`ROS_DOMAIN_ID=11`)
Workspace: `/root/yahboomcar_ws/src/physical_rosmaster`

## Emergency Stop

Keep a second terminal ready with this command during every motion test:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
```

The driver also stops the controller when `/cmd_vel` is stale for 0.5 seconds. The watchdog is a last line of defense, not a replacement for supervision or physical access to motor power.

## Phase 0: Physical Preflight

- [ ] Battery is charged above 12.0 V before floor testing. Do not repeat the previous 10.1 V test.
- [ ] Battery remains above the battery maker's safe lower limit under load.
- [ ] Motor power switch is confirmed and reachable.
- [x] Robot was confirmed securely lifted before the 2026-08-20 Phase 3 pulses.
- [ ] A person supervises the robot and can cut motor power.
- [x] Autostart and unrelated navigation, joystick, line-follow, laser, voice, and color-tracking nodes are stopped for the current validation session.
- [ ] Optional host fix: `/etc/hosts` contains `127.0.1.1 x3-c`.

Do not continue to a floor test until all lifted checks pass.

## Phase 1: Source, Build, and Hardware-Free Tests

Inside `rosmaster_humble`:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git status --short --branch
git log -1 --oneline
```

If the clone is clean and an update is required:

```bash
git fetch origin
git pull --ff-only origin main
```

Never overwrite robot-local changes to update the checkout.

Build and test:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select yahboomcar_base_node --ctest-args -R test_x3_odometry
colcon test-result --verbose

cd /root/yahboomcar_ws/src/physical_rosmaster
python3 -m pytest -q yahboomcar_bringup/test/test_x3_driver_utils.py
python3 tools/rosmaster_lib_probe.py --hash-only
```

Pass criteria:

- [x] All 19 normal packages build.
- [x] Focused C++ odometry tests pass.
- [x] X3 driver safety/encoder helper tests pass.
- [x] Installed `Rosmaster_Lib` matches public V3.3.9.
- [x] `x3_driver.yaml` contains a positive `cmd_vel_timeout`.
- [x] `use_joy` remains false by default.

## Phase 2: Per-Wheel Encoder Mapping (Lifted, No ROS Nodes)

The probe and ROS driver must never own the motor-controller serial device
simultaneously. On `x3-c`, use the stable path
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`; its `/dev/ttyUSB*` number
can change after a USB reset.

```bash
ros2 node list
pgrep -af 'Mcnamu_driver_X3|driver_node' || true
lsof /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 2>/dev/null || true
```

With no driver process active:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 500 --period 0.1
```

Pre-rewire observations from 2026-08-20:

| Physical wheel | Raw channel | Delta sign | Delta ticks | Ticks/revolution |
| --- | --- | ---: | ---: | ---: |
| Front-left | `m1` | `+` | `+1371` | Not measured |
| Front-right | `m3` | `+` | `+1424` | Not measured |
| Back-left | `m2` | `+` | `+866` | Not measured |
| Back-right | `m4` | `-` | `-1886` | Not measured |

These observations describe the cable fault; they are not the desired software
mapping. Yahboom's official X3 wiring diagram specifies:

| Factory controller port | Physical wheel |
| --- | --- |
| `M4` | Front-left |
| `M2` | Front-right |
| `M3` | Back-left |
| `M1` | Back-right |

With all robot and motor power disconnected and controller LEDs off, move the
complete keyed motor/encoder cables without reversing or repinning a plug:

- [ ] Swap `M1 <-> M4`.
- [ ] Swap `M2 <-> M3`.
- [ ] Restore power with the robot lifted and repeat the four hand rotations.

The expected post-rewire order is `[FL, FR, BL, BR] = [m4, m2, m3, m1]`, or
zero-based `encoder_order: [3, 1, 2, 0]`. The signs must be measured again;
`[1.0, 1.0, 1.0, 1.0]` is provisional until then.

Update only the versioned parameters in `yahboomcar_bringup/param/x3_driver.yaml`:

- `encoder_order`: zero-based raw channels in `[FL, FR, BL, BR]` order.
- `encoder_signs`: `+1.0` or `-1.0` so physically forward rotation is positive for every wheel.
- `encoder_cpr`: use measured evidence; do not assume `1040.0` is correct without checking.

Then rebuild `yahboomcar_bringup` and repeat the hardware-free tests.

The synchronized captures established the current cable identities. The
back-left capture also had an incidental `m1` delta of about `-175`, but `m2`
was the clearly dominant channel. The turns were approximate, so they are not
valid CPR evidence; `encoder_cpr: 1040.0` remains provisional until a marked,
exact-turn test.

## Phase 3: Lifted Graph and Kinematic Validation

Launch only the clean core stack:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

From another sourced terminal, inspect the graph:

```bash
ros2 node list
ros2 topic info -v /cmd_vel
ros2 topic info -v /joint_states
ros2 topic info -v /odom_raw
ros2 topic info -v /odom
timeout 12s ros2 topic hz /joint_states
timeout 12s ros2 topic hz /odom_raw
```

Pass criteria before motion:

- [x] One each: `driver_node`, `base_node`, `ekf_filter_node`, Madgwick filter, and `robot_state_publisher`.
- [x] Zero `/cmd_vel` publishers while idle and exactly one actuator subscriber from `driver_node`; a passive rosbag subscriber is allowed during recording.
- [x] One `/joint_states` publisher from `driver_node`.
- [x] One `/odom_raw` publisher from `base_node`.
- [x] One `/odom` publisher and one observed `odom -> base_footprint` transform with raw odom TF disabled.
- [x] `/joint_states` and `/odom_raw` are approximately 10 Hz.
- [x] Stationary wheel positions and raw pose remain stable for at least 30 seconds.

Stationary evidence from 2026-08-20 is recorded in
[`x3-c_odom_validation_2026-08-20.md`](x3-c_odom_validation_2026-08-20.md).
Motion remains blocked until the battery is above 12.0 V and the robot is
securely lifted for the per-wheel test.

Start a new evidence bag; do not reuse the August 16 pre-encoder bag:

```bash
ros2 bag record -o /tmp/x3_odom_validation_2026-08-20 \
  /cmd_vel /joint_states /vel_raw /odom_raw /odom \
  /imu/data_raw /imu/data /voltage /edition /tf /diagnostics
```

With the robot confirmed lifted, run bounded pulses from the repository root:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/safe_cmd_vel_pulse.py --x 0.20 --duration 1.5
python3 tools/safe_cmd_vel_pulse.py --y 0.20 --duration 1.5
python3 tools/safe_cmd_vel_pulse.py --yaw 0.50 --duration 1.5
```

The pulse tool refuses to run if another `/cmd_vel` publisher exists or if
there is not exactly one actuator subscriber from `driver_node`. A passive
`rosbag2_recorder` subscriber is allowed during evidence capture.

Pass criteria:

- [ ] Forward: all normalized wheel deltas positive; odom `delta x > +0.10 m`; lateral/yaw leakage recorded.
- [ ] Strafe left: `FL- FR+ BL+ BR-`; odom `delta y > +0.10 m`; forward/yaw leakage recorded.
- [ ] Rotate CCW: `FL- FR+ BL- BR+`; odom yaw delta positive; translation leakage recorded.
- [x] `/vel_raw` and wheel velocities return near zero after each completed pulse.
- [x] A zero command is observed after every completed pulse.
- [x] No discontinuity, stale-input, duplicate-publisher, or TF-authority warning appears.

Any incorrect sign stops the session. Correct order/sign parameters and repeat lifted validation before floor testing.

The first 2026-08-20 forward and positive-Y pulses used the old provisional
mapping and showed incorrect normalized signs. The subsequent hand test and
factory diagram exposed incorrect motor-port assignments. Complete the cable
swaps, repeat the hand test, then rebuild and repeat all three lifted pulses
before accepting this phase.

## Phase 4: Floor Breakaway and Repeatability

Use a smooth level surface with a clear two-meter perimeter. Start a new floor bag and retain battery voltage in the recording.

Run one-second bounded pulses, stopping if the robot moves unexpectedly:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.15 --duration 1.0
python3 tools/safe_cmd_vel_pulse.py --x 0.20 --duration 1.0
python3 tools/safe_cmd_vel_pulse.py --x 0.25 --duration 1.0
```

- [ ] Record the lowest repeatable breakaway speed; do not label one failed speed as deadband without repetitions.
- [ ] Record voltage before, during, and after motion.
- [ ] Verify physical motion, `/joint_states`, `/vel_raw`, and `/odom_raw` agree.
- [ ] Repeat forward, reverse, left, right, CW, and CCW trials to expose directional bias.

## Phase 5: Ground-Truth Calibration

Record start/end samples with valid Humble syntax:

```bash
ros2 topic echo /odom_raw --field pose.pose.position --once
```

For distance tests, use measured marks and bounded timed pulses. Run at least three trials in both directions. Calculate actual distance from the marks and odometry displacement from the bag.

Calibration order:

1. Confirm encoder order/signs.
2. Confirm `encoder_cpr` from wheel turns.
3. Measure wheel radius and X/Y wheel separation.
4. Adjust `linear_scale_x`, `linear_scale_y`, and `angular_scale` only for remaining systematic error. These scales apply to both wheel and firmware-fallback odometry.
5. Tune pose/twist covariances from repeatability and residual error.

For rotation, calculate continuous unwrapped yaw from the bag. A final quaternion near zero after 360 degrees is not proof that the integrated angle reached `2*pi`.

Acceptance target after calibration:

- [ ] At least three 1.0 m forward/reverse trials with mean absolute error below 5%.
- [ ] At least three measured left/right strafe trials with error and slip documented.
- [ ] At least three CW/CCW 360-degree trials with mean absolute error below 5%.
- [ ] No unexplained encoder reset, timeout, or TF jump.

## Phase 6: Sign-Off

- [ ] Save bag metadata, Git revision, parameters, battery state, floor type, and measurements in `agents/x3-c_odom_validation_<DATE>.md`.
- [ ] Store large bags outside Git; do not treat `robot_artifacts/x3_lifted_probe` as current evidence.
- [ ] Review covariance and EKF behavior using the completed trials.
- [ ] Validate LiDAR/camera device names and headless autostart only after core motion and odometry pass.
- [ ] Tag a known-working physical snapshot only after the full sign-off.
