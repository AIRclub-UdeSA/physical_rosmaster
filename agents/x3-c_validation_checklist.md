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

- [x] Battery is confirmed fully charged for the installed pack before floor testing. On 2026-08-21 the operator measured `11.7 V` at the pack with a multimeter while ROS/controller telemetry reported approximately `11.3 V`; do not apply the superseded generic `> 12.0 V` threshold.
- [ ] Battery remains above the battery maker's safe lower limit under load.
- [ ] Motor power switch is confirmed and reachable.
- [x] Robot is confirmed securely lifted before any Phase 3 pulse. The operator confirmed the physical state before the 2026-08-21 pulses; every 2026-08-20 powered pulse remains classified as a floor test.
- [ ] A person supervises the robot and can cut motor power.
- [x] Autostart actuator/core duplicates and competing `/cmd_vel` publishers are stopped for the current validation session. Camera and lidar processes may remain because they do not command motion.
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
git switch main
git pull --ff-only origin main
git log -1 --oneline
```

These commands assume the validation branch has been merged to `main`. If a
specific unmerged revision is required, check out that branch explicitly.
Never switch branches or overwrite robot-local changes to update the checkout.

Build and test:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select yahboomcar_base_node --ctest-args -R test_x3_odometry
colcon test-result --verbose

cd /root/yahboomcar_ws/src/physical_rosmaster
python3 -m pytest -q \
  yahboomcar_bringup/test/test_x3_driver_utils.py \
  tools/test_safe_cmd_vel_pulse.py
python3 tools/rosmaster_lib_probe.py --hash-only
```

Pass criteria:

- [x] All 19 normal packages build.
- [x] Focused C++ odometry tests pass.
- [x] X3 driver safety/encoder helper tests pass.
- [x] Bounded-pulse recorder gating tests pass.
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

Direction-controlled observations from 2026-08-20:

| Physical wheel | Raw channel | Delta sign | Delta ticks | Ticks/revolution |
| --- | --- | ---: | ---: | ---: |
| Front-left | `m1` | `+` | `+6305` | Not measured |
| Front-right | `m3` | `+` | `+5932` | Not measured |
| Back-left | `m2` | `+` | `+6733` | Not measured |
| Back-right | `m4` | `+` | `+5984` | Not measured |

For this repeat, the operator and camera faced the same direction. Forward was
defined as moving the top of the whole wheel toward the camera/front, and only
one wheel was intentionally moved in each synchronized capture. This resolves
the first pass's ambiguous back-right direction.

The validated software mapping is `[FL, FR, BL, BR] = [m1, m3, m2, m4]`, or
zero-based `encoder_order: [0, 2, 1, 3]`, with
`encoder_signs: [1.0, 1.0, 1.0, 1.0]`.

The operator confirmed that the powered-off wiring follows Yahboom's physical
port layout. Keeping PCB ports distinct from report-packet field names gives:

| Physical wheel | PCB port | Raw encoder packet field | ROS joint |
| --- | --- | --- | --- |
| Front-left | `M4` | `m1` | `front_left_joint` |
| Front-right | `M2` | `m3` | `front_right_joint` |
| Back-left | `M3` | `m2` | `back_left_joint` |
| Back-right | `M1` | `m4` | `back_right_joint` |

Do not infer PCB motor-port wiring from these field names. The installed
`Rosmaster_Lib` unpacks four consecutive encoder report fields and returns them
as `m1..m4`; it does not map those names to the controller's printed `M1..M4`
labels. The operator inspected the powered-off wiring against Yahboom's diagram
and found it correct. The earlier proposed cable swaps are withdrawn.

Update only the versioned parameters in `yahboomcar_bringup/param/x3_driver.yaml`:

- `encoder_order`: zero-based raw channels in `[FL, FR, BL, BR]` order.
- `encoder_signs`: `+1.0` or `-1.0` so physically forward rotation is positive for every wheel.
- `encoder_cpr`: use measured evidence; do not assume `1040.0` is correct without checking.

Then rebuild `yahboomcar_bringup` and repeat the hardware-free tests.

Small incidental changes appeared while handling the chassis: about `-36` on
`m1` during the back-left test, and `+109` on `m1` plus `+83` on `m3` during the
back-right test. Each was under 2% of the dominant wheel change and does not
affect channel identification. The turns were approximate, so they are not
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
The completed true lifted repetition is recorded in
[`x3-c_lifted_odom_validation_2026-08-21.md`](x3-c_lifted_odom_validation_2026-08-21.md).
The powered 2026-08-20 tests documented below were initially recorded as
lifted, but the operator later clarified that all were on the floor. The battery
had not been verified fully charged for those trials, so they remain unsuitable
for calibration and do not complete Phase 3. The earlier generic `> 12.0 V`
threshold was superseded on 2026-08-21 by paired full-charge multimeter and
controller readings plus under-load sag monitoring.

Start a new evidence bag; do not reuse the August 16 pre-encoder bag:

```bash
ros2 bag record \
  --qos-profile-overrides-path \
  /root/yahboomcar_ws/src/physical_rosmaster/tools/x3_validation_qos.yaml \
  --regex '^/(cmd_vel|joint_states|vel_raw|odom_raw|odom|rosout|imu/data_raw|imu/data|voltage|edition|tf|diagnostics)$' \
  -o /tmp/x3_odom_validation_YYYY-MM-DD_lifted
```

Replace `YYYY-MM-DD` and add a run suffix when needed; never overwrite or append
new evidence to an earlier bag.

Regex recording keeps discovery open for the short-lived pulse publisher. Add
`--require-recorder` to each `safe_cmd_vel_pulse.py` command so motion cannot
start until both the driver and `rosbag2_recorder` are QoS-compatible matched
subscriptions on `/cmd_vel`. The analyzer normally
delimits trials from recorded `/cmd_vel` samples. If those samples are missing
but the timestamped `safe_cmd_vel_pulse` start record exists on `/rosout`, it
uses that record as a fallback.

With the robot confirmed lifted, run bounded pulses from the repository root:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/safe_cmd_vel_pulse.py --x 0.20 --duration 1.5 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --y 0.20 --duration 1.5 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --yaw 0.50 --duration 1.5 --require-recorder
```

The pulse tool refuses to run if another `/cmd_vel` publisher exists or if
there is not exactly one actuator subscriber from `driver_node`. A passive
`rosbag2_recorder` subscriber is allowed during evidence capture.

Pass criteria:

- [x] Forward sign gate: all normalized wheel deltas positive.
- [x] Forward integration gate: calculated odom `delta x` is positive; lateral/yaw leakage recorded. This is not a ground-distance measurement while lifted.
- [x] Strafe-left sign gate: `FL- FR+ BL+ BR-`.
- [x] Strafe integration gate: calculated odom `delta y` is positive; forward/yaw leakage recorded. This is not a ground-distance measurement while lifted.
- [x] Rotate CCW: `FL- FR+ BL- BR+`; odom yaw delta positive; translation leakage recorded.
- [x] `/vel_raw` and wheel velocities return near zero after each completed lifted pulse.
- [x] A zero command is observed after every completed lifted pulse.
- [x] No discontinuity, stale-input, duplicate-publisher, or TF-authority warning appears during lifted pulses.

Any incorrect sign stops the session. Correct order/sign parameters and repeat lifted validation before floor testing.

True lifted results from 2026-08-21, using recorded nonzero command windows:

| Command | Duration | Wheel delta `[FL, FR, BL, BR]` rad | Raw odom `[x, y, yaw]` | Result |
| --- | ---: | --- | --- | --- |
| `x=+0.20` | `1.464 s` | `[+7.3405, +9.4006, +7.9506, +7.9869]` | `[+0.2667, +0.0485, +0.1048]` | Sign and positive-X integration pass |
| `y=+0.20` | `1.466 s` | `[-7.6123, +9.4369, +7.9627, -7.8056]` | `[-0.0340, +0.2689, +0.0640]` | Sign and positive-Y integration pass |
| `yaw=+0.50` | `1.463 s` | `[-2.6281, +4.7426, -2.3864, +2.8335]` | `[+0.0178, +0.0228, +0.6295]` | Sign and positive-yaw integration pass |

The evidence bag is `/tmp/x3_odom_validation_2026-08-21_lifted_r1`.
Voltage was `11.3 V` idle and reached `10.9-11.0 V` during the lifted pulses.
All velocities settled to zero, `/cmd_vel` had zero publishers after each
pulse, the core shut down cleanly, and the controller serial port was released.
One EKF update-rate overrun (`0.204 s`) occurred; no discontinuity, stale-input,
duplicate-publisher, or TF-authority warning occurred. CPR, ground-distance
scale, and per-wheel response remain uncalibrated.

Floor observations from 2026-08-20 are retained below as useful sign evidence,
but they are not a Phase 3 lifted pass. Each used a recorded 0.757-second
nonzero command window:

| Command | Wheel delta `[FL, FR, BL, BR]` rad | Raw odom `[x, y, yaw]` | Result |
| --- | --- | --- | --- |
| `x=+0.12` | `[+0.6223, +2.0904, +0.6525, +1.9031]` | `[+0.0384, +0.0206, +0.1359]` | Sign pass; large magnitude/yaw bias |
| `y=+0.12` | `[-0.5800, +0.8337, +0.2356, -0.1269]` | `[-0.0047, +0.0141, +0.0526]` | Sign pass; large magnitude/yaw bias |
| `yaw=+0.50` | `[-0.1752, +0.4833, -0.0242, +0.2175]` | `[+0.0017, +0.0051, +0.0450]` | Sign and yaw pass; weak BL response |

Recorded floor `yaw=+0.12` and `yaw=+0.30` trials did not move the wheel
encoders; `+0.50` was the first tested yaw command to break through. The floor
patterns match all three expected signs; the lifted gates were later completed
on 2026-08-21.
CPR, per-wheel magnitude imbalance, and low-voltage behavior remain unresolved.
Do not use these floor odom deltas as distance calibration because no external
ground truth was measured.

## Phase 4: Floor Breakaway and Repeatability

Use a smooth level surface with a clear two-meter perimeter. Start a new floor
bag using the Phase 3 recorder procedure, with a fresh output name, and retain
battery voltage in the recording.

Run one-second bounded pulses, stopping if the robot moves unexpectedly:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.15 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --x 0.20 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --x 0.25 --duration 1.0 --require-recorder
```

Charged-pack observations from 2026-08-21:

| Command | Duration | Wheel delta `[FL, FR, BL, BR]` rad | Raw odom `[x, y, yaw]` | Voltage | Physical observation |
| --- | ---: | --- | --- | --- | --- |
| `x=+0.15` | `0.962 s` | `[+2.2535, +3.2866, +2.3381, +3.0208]` | `[+0.0895, +0.0093, +0.0858]` | `11.0-11.2 V` | Not seen clearly by operator |
| `x=+0.15` | `2.978 s` | `[+13.8653, +14.9709, +14.1492, +14.5238]` | `[+0.4677, +0.0806, +0.0740]` | `11.2 V` | Visually smooth; actual distance not measured precisely |

- [ ] Record the lowest repeatable breakaway speed; `0.15 m/s` moved repeatably, but lower speeds were not tested with this fully charged pack.
- [x] Record voltage before, during, and after motion.
- [x] Verify physical motion, `/joint_states`, `/vel_raw`, and `/odom_raw` agree qualitatively; precise scale remains unverified.
- [ ] Repeat forward, reverse, left, right, CW, and CCW trials to expose directional bias.

The same bag contains 29 later standard-keyboard command windows. The installed
teleop started at its hardcoded defaults (`0.5 m/s`, `1.0 rad/s`) rather than the
intended reduced values. It exercised forward, reverse, lateral, diagonal, and
CCW commands, and the driver watchdog stopped sparse inputs 11 times. Preserve
that data as qualitative stress/directional evidence only: maneuver annotations,
precise physical distances/headings, a CW rotation, and continuous-yaw analysis
are insufficient for calibration.

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

- [x] Save bag metadata, Git revision, parameters, battery state, floor type, and available measurements in dated validation reports. The exact floor type and precise external measurements remain unrecorded and are explicitly called out.
- [x] Store large bags outside Git; current evidence paths and SHA-256 hashes are recorded in the 2026-08-21 reports. Do not treat `robot_artifacts/x3_lifted_probe` as current evidence.
- [ ] Review covariance and EKF behavior using the completed trials.
- [ ] Validate LiDAR/camera device names and headless autostart only after core motion and odometry pass.
- [ ] Tag a known-working physical snapshot only after the full sign-off.
# Historical validation checklist

> This checklist was completed against the pre-cleanup `/odom_raw` + EKF platform. It is evidence, not the current rollout procedure. Use `tools/physical_contract_probe.py` and current `docs/` for new validation.
