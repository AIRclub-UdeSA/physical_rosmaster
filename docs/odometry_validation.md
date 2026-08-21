# Odometry Validation Plan

Date: 2026-08-11

Goal: validate and calibrate the encoder-delta mecanum odometry now implemented on the physical X3, while retaining firmware velocity only as a timed fallback.

## Current Sources

- `/cmd_vel`: commanded chassis velocity.
- `/vel_raw`: chassis velocity returned by `Rosmaster_Lib.get_motion_data()` and published by `Mcnamu_driver_X3.py`.
- `/joint_states`: four wheel positions/velocities derived from `get_motor_encoder()` using configurable channel order, signs, and CPR.
- `/odom_raw`: mecanum wheel-delta integration from `yahboomcar_base_node`, with midpoint heading integration.
- `/odom`: EKF output after fusing `/odom_raw` and IMU.
- `Rosmaster_Lib.get_motor_encoder()`: the four signed 32-bit counters underlying `/joint_states`.

The driver has a `cmd_vel_timeout` watchdog and the base node rejects stale or discontinuous wheel input. Motion validation must use `tools/safe_cmd_vel_pulse.py`; do not use an unbounded publisher or rely on a later shell command to stop the robot.

## Stage 1: Confirm Installed Library

Inside `rosmaster_humble`:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

Record:

- source path
- SHA256
- file size
- version comment
- whether it matches the public V3.3.9 hash in `agents/rosmaster_lib_public_v3_3_9.md`

## Stage 2: Sample Motion And Encoders Without ROS Motion Commands

With the robot stationary:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 100 --period 0.1
```

Expected:

- motion values should stay near zero
- encoder counters should stay stable when wheels are not moving
- battery should report a plausible voltage if the library exposes it

Lift the robot and rotate each wheel by hand if safe. Encoder counters should change for the corresponding wheel channels.

### x3-c Stationary Probe Result

Observed on `x3-c` with the robot stationary:

- `motion_vx`, `motion_vy`, and `motion_vz` stayed at `0.000000` for all 100 samples
- encoder counters stayed constant at `-3, 2, 1, 90`
- reported battery voltage stayed around `10.5` to `10.6` V
- no serial errors were reported during the probe

This confirms the controller is returning a stable stationary baseline, but it does not yet prove whether the encoder counters are sufficient for motion odometry.

### x3-c Floor Motion Probe Result

The operator later clarified that the powered probe was performed with the
robot on the floor, despite initially being recorded as lifted. Observed while
commanded through `/cmd_vel`:

- forward command produced increasing encoder counts and nonzero `motion_vx`, `motion_vy`, and `motion_vz`
- strafe-left command produced clear encoder deltas on the four wheels and increasing `motion_vy`
- rotate-ccw command produced strong, asymmetric encoder deltas across the four wheels and increasing `motion_vz`
- battery stayed around `10.5` V during the short motion pulses

This confirms that the reported encoder counters move with commanded floor motion, but it did not satisfy the lifted validation gate or provide ground-truth calibration. The lifted gate was subsequently completed on 2026-08-21 as described below.

### x3-c 2026-08-20 Encoder Mapping

A direction-controlled repeat test defined forward as moving the top of each
whole wheel toward the camera/front of the robot. Isolated hand rotations found
`[FL, FR, BL, BR] = [m1, m3, m2, m4]`, with signs `[+, +, +, +]`. The source
configuration therefore uses `encoder_order: [0, 2, 1, 3]` and all-positive
`encoder_signs`.

The operator separately confirmed Yahboom's physical PCB port layout as
`[FL, FR, BL, BR] = [M4, M2, M3, M1]`. Combining the physical inspection with
the isolated hand test gives the empirical relationship
`[m1, m2, m3, m4] = [M4, M3, M2, M1]`. The lowercase names are report-packet
fields; the uppercase names are printed controller ports.

The earlier proposed `M1 <-> M4` and `M2 <-> M3` cable swaps were withdrawn.
The operator verified the powered-off wiring against Yahboom's diagram, and
the installed `Rosmaster_Lib` only names four consecutive encoder packet fields
`m1..m4`; it does not establish that those names equal the controller's printed
motor-port labels. CPR remains provisional because the hand rotations were not
exact marked turns.

After rebuilding this mapping, bounded floor tests matched the expected wheel
sign patterns for forward, strafe-left, and CCW rotation. Wheel magnitudes were
strongly unequal, `yaw=+0.12` and `+0.30 rad/s` did not move the encoders on the
floor, and `yaw=+0.50 rad/s` did.

### x3-c 2026-08-21 Lifted Validation

With the robot explicitly confirmed lifted, recorded 1.5-second bounded pulses
passed the expected wheel patterns and positive raw-odom integration:

- forward: wheels `[+7.3405, +9.4006, +7.9506, +7.9869] rad`, odom
  `[+0.2667, +0.0485, +0.1048]`
- strafe left: wheels `[-7.6123, +9.4369, +7.9627, -7.8056] rad`, odom
  `[-0.0340, +0.2689, +0.0640]`
- rotate CCW: wheels `[-2.6281, +4.7426, -2.3864, +2.8335] rad`, odom
  `[+0.0178, +0.0228, +0.6295]`

All reported velocities returned to zero after every pulse. Treat ordering,
signs, and the wheel-state integration path as validated, but do not treat CPR,
distance scale, leakage, or per-wheel response as calibrated without marked
wheel rotations and externally measured floor trials.

### x3-c 2026-08-21 Charged-Pack Floor Response

The operator measured the fully charged pack at `11.7 V` with a multimeter;
controller telemetry read `11.2-11.3 V` at idle. This supersedes the earlier
generic `> 12.0 V` floor gate. Battery preflight now records paired readings and
under-load sag.

Two recorded `x=+0.15 m/s` trials lasted `0.962 s` and `2.978 s`. The longer
trial produced wheel deltas `[+13.8653, +14.9709, +14.1492, +14.5238] rad` and
raw-odom delta `[+0.4677, +0.0806, +0.0740]`; controller voltage stayed at
`11.2 V`. The operator described the movement as visually smooth, but no precise
external distance or heading measurement was accepted. This validates
repeatable floor response, not odometry scale.

The same bag contains a later unmeasured standard-keyboard exploration at the
package's hardcoded `0.5 m/s` and `1.0 rad/s` defaults. Treat those windows as
qualitative stress/directional evidence only. See
`agents/x3-c_floor_odom_validation_2026-08-21.md` for the evidence boundary,
bag metadata, checksums, and final safety state.

## Stage 3: Compare Command, Firmware Velocity, And Encoders

Terminal A:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Terminal B:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 bag record \
  --qos-profile-overrides-path \
  /root/yahboomcar_ws/src/physical_rosmaster/tools/x3_validation_qos.yaml \
  --regex '^/(cmd_vel|joint_states|vel_raw|odom_raw|odom|rosout|imu/data_raw|imu/data|voltage|edition|tf|diagnostics)$' \
  -o /tmp/x3_odom_probe
```

Do not run `rosmaster_lib_probe.py` while `driver_node` is active. Both
processes would attempt to own the motor-controller serial device. On `x3-c`,
use `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` rather than an unstable
`/dev/ttyUSB*` number. Use the probe only during the no-ROS per-wheel hand
test.

Motion commands, with wheels lifted first:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.20 --duration 1.5 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --y 0.20 --duration 1.5 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --yaw 0.50 --duration 1.5 --require-recorder
```

Repeat on the floor at low speed only after lifted tests behave correctly.

## Stage 4: Interpret Results

Good encoder signal:

- counters change with wheel rotation
- counters stop changing when wheels stop
- count signs are consistent for forward, strafe, and rotate commands
- one resisted wheel produces a visibly different delta for that wheel

Good firmware velocity signal:

- `/vel_raw` goes near zero after stop commands
- `/vel_raw.linear.y` is nonzero for strafe commands
- `/vel_raw.angular.z` is nonzero for rotate commands
- `/vel_raw` changes when the robot is physically resisted if firmware speed is encoder-derived

If encoders are stable, retain wheel-state odometry as the primary path and calibrate its parameters from repeated ground-truth runs.

If encoders become unavailable, verify that `/vel_raw` takes over only after the configured joint-state timeout. If both sources stop, `/odom_raw` must publish a zero twist once without integrating additional pose.

## Stage 5: Data To Capture For Calibration

Record these before accepting encoder odometry:

- wheel order: encoder motor 1, 2, 3, 4 to physical wheel name
- sign convention for each wheel
- ticks per wheel revolution
- wheel radius
- wheel separation in X and Y, or equivalent mecanum geometry constants
- whether encoder counts are absolute since boot or periodic deltas
- whether counts reset when motor controller resets
