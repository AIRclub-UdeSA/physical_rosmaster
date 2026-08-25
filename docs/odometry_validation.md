# Encoder-only X3 odometry validation

The canonical physical odometry source is `/odom`. `yahboomcar_base_node` calculates four-wheel mecanum motion from encoder position deltas and publishes `odom -> base_footprint`. There is no EKF and no firmware-velocity fallback.

Historical reports under `agents/` refer to `/odom_raw` and EKF `/odom`. Those topic names describe the pre-cleanup graph only.

## Invariants

- Wheel names are `front_left_wheel_joint`, `front_right_wheel_joint`, `back_left_wheel_joint`, and `back_right_wheel_joint`.
- Joint position and velocity values are finite and ordered FL, FR, BL, BR.
- Encoder arrival time and message stamps are checked independently.
- Non-positive or discontinuous time rebases input without integrating.
- Implausible wheel deltas are rejected.
- Stale encoders publish one zero-twist odometry state and stop integration.
- Encoder stream health and stale/discontinuity failures appear on `/diagnostics`.
- `/vel_raw` remains telemetry only.
- Midpoint heading is used for combined translation/rotation.
- `/odom` contains lateral `twist.linear.y` and configured covariance.
- The odometry node is the only authority for `odom -> base_footprint`.

## Workstation regression

```bash
colcon build --symlink-install --packages-select yahboomcar_base_node
colcon test --packages-select yahboomcar_base_node
colcon test-result --verbose
```

The C++ suite covers mecanum forward, strafe, and rotation equations; lateral twist preservation; midpoint integration; frame/covariance output; source freshness; scale factors; and zero-`dt` handling.

## Stationary robot gate

With default bringup and no `/cmd_vel` publisher:

```bash
ros2 topic info -v /joint_states
ros2 topic info -v /odom
ros2 topic hz /joint_states
ros2 topic hz /odom
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 topic echo /diagnostics
ros2 run tf2_ros tf2_echo odom base_footprint
```

Require one publisher for `/joint_states` and one for `/odom`, canonical joint names, approximately 10 Hz input/output, finite values, and a single odometry TF authority. Stationary drift should be negligible because integration is encoder driven.

Disconnecting or stopping encoder input must produce an odometry error and stop integration. `/vel_raw` continuing to publish must not move `/odom`.

## Lifted motion gate

Secure the X3 so all wheels rotate freely. Keep an emergency stop path and battery monitoring. Record evidence before the pulse:

```bash
ros2 bag record \
  /cmd_vel /joint_states /odom /tf /vel_raw /voltage /rosout
```

In another shell, run one axis at a time:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --y 0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --yaw 0.30 --duration 1.0 --require-recorder
```

Expected encoder signs with the validated FL, FR, BL, BR mapping:

| Command | FL | FR | BL | BR | `/odom` response |
|---|---:|---:|---:|---:|---|
| forward +X | + | + | + | + | +X |
| strafe +Y | − | + | + | − | +Y |
| rotate +Z | − | + | − | + | +yaw |

Also require:

- motion starts only during the bounded command window;
- the driver stops after command timeout;
- encoder position remains continuous after stop;
- odometry pose remains fixed once wheel input is stationary;
- no encoder discontinuity or stale-input error occurs in a normal trial.

Analyze a recorded bag with the current canonical topic:

```bash
python3 tools/analyze_x3_odom_bag.py <bag-directory>
```

## Floor gate

Use a charged battery, clear floor, conservative limits, and external measurements. Repeat forward, lateral, and rotation in both directions. Record commanded duration, measured displacement/heading, encoder deltas, `/odom` delta, voltage, surface, payload, and trial count.

Do not change provisional geometry, CPR, scale, or covariance from a single visual trial. Require repeated external measurements and separate systematic scale error from wheel slip or mecanum surface effects.

## Calibration nodes

Calibration nodes are opt-in and start inert:

```bash
ros2 run yahboomcar_bringup calibrate_linear_X3 --ros-args \
  -p axis:=x -p direction:=1.0 -p test_distance:=0.50 -p speed:=0.10

ros2 param set /calibrate_linear start_test true
```

```bash
ros2 run yahboomcar_bringup calibrate_angular_X3 --ros-args \
  -p direction:=1.0 -p test_angle_degrees:=90.0 -p speed:=0.30

ros2 param set /calibrate_angular start_test true
```

Review all parameters before activation. Each node has maximum target, speed, and duration limits; it stops on completion, cancellation, TF loss, invalid data, timeout, and shutdown.

## Acceptance

Odometry is accepted for platform rollout when lifted and repeated floor trials confirm signs and qualitative response, stale encoders never select another estimator, watchdog stop works, and the full physical contract probe resolves every sensor frame through `odom` at message time.
