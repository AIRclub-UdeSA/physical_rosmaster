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

### x3-c Lifted Motion Probe Result

Observed while the robot was lifted and commanded through `/cmd_vel`:

- forward command produced increasing encoder counts and nonzero `motion_vx`, `motion_vy`, and `motion_vz`
- strafe-left command produced clear encoder deltas on the four wheels and increasing `motion_vy`
- rotate-ccw command produced strong, asymmetric encoder deltas across the four wheels and increasing `motion_vz`
- battery stayed around `10.5` V during the short motion pulses

This is enough to confirm that the reported encoder counters move with commanded motion while lifted. The next step is to capture the exact sign convention and compare the ROS `/vel_raw` and `/odom_raw` topics against these same motions.

### x3-c 2026-08-20 Wiring Diagnosis

Synchronized forward hand rotations found the pre-rewire cable identities as
`[FL, FR, BL, BR] = [m1, m3, m2, m4]`, with observed signs
`[+, +, +, -]`. That does not match Yahboom's factory X3 layout
`[m4, m2, m3, m1]`. The planned powered-off correction is to swap the complete
`M1 <-> M4` cables and the complete `M2 <-> M3` cables. Do not reverse a keyed
plug or move individual pins.

The post-rewire order and signs must be confirmed by another no-command hand
test before powered validation. The source configuration carries the expected
factory order `[3, 1, 2, 0]` and provisional all-positive signs. CPR also
remains provisional because the hand rotations were not exact marked turns.

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
ros2 bag record -o /tmp/x3_odom_probe \
  /cmd_vel /joint_states /vel_raw /odom_raw /odom \
  /imu/data_raw /imu/data /voltage /edition /tf /diagnostics
```

Do not run `rosmaster_lib_probe.py` while `driver_node` is active. Both
processes would attempt to own the motor-controller serial device. On `x3-c`,
use `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` rather than an unstable
`/dev/ttyUSB*` number. Use the probe only during the no-ROS per-wheel hand
test.

Motion commands, with wheels lifted first:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.20 --duration 1.5
python3 tools/safe_cmd_vel_pulse.py --y 0.20 --duration 1.5
python3 tools/safe_cmd_vel_pulse.py --yaw 0.50 --duration 1.5
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
