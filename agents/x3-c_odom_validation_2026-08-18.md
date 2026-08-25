# X3-C Wheel-State Odometry Validation Report - 2026-08-18

## Scope

Validate the wheel-state odometry merged in `a166f60` on the physical X3-C
robot, including the driver encoder stream, `/odom_raw`, EKF `/odom`, and TF
ownership.

## Baseline

- Repository: `physical_rosmaster`
- Branch: `main`
- Revision: `a166f60` (`Merge pull request #1 from AIRclub-UdeSA/feat/wheel-state-odometry`)
- ROS distribution: ROS 2 Humble
- ROS domain: `11`
- Workspace: `/root/yahboomcar_ws`
- Robot: `x3-c`
- Existing untracked `robot_artifacts/` was preserved and was not part of this validation commit.

## Software validation

The merged packages were rebuilt with:

```bash
source /opt/ros/humble/setup.bash
cd /root/yahboomcar_ws
colcon build --symlink-install --packages-select yahboomcar_base_node yahboomcar_bringup
source install/setup.bash
```

The focused odometry test passed:

```text
Summary: 7 tests, 0 errors, 0 failures, 0 skipped
```

The rebuilt `base_node` correctly subscribed to `/joint_states`. The initial
running stack had stale binaries, orphaned processes, duplicate publishers,
and no EKF output. The stale bringup and display processes were stopped, and
the graph was rebuilt as a single clean stack.

The EKF initially failed with:

```text
error while loading shared libraries: libdiagnostic_updater.so
```

The container package was reinstalled with `apt-get install --reinstall
ros-humble-diagnostic-updater`, after which the EKF launched successfully.

## Clean graph result

The final graph contained:

- One `driver_node`
- One `base_node`
- One `ekf_filter_node`
- One `imu_filter_madgwick`
- One `robot_state_publisher`
- One `/joint_states` publisher from `driver_node`
- One `/odom_raw` publisher from `base_node`
- One `/odom` publisher from `ekf_filter_node`
- One `odom -> base_footprint` TF authority from EKF

The rebuilt base node subscribed to both `/joint_states` and `/vel_raw`.
Stationary encoder positions, velocities, and raw odometry remained stable.
`/odom_raw` published at approximately 10 Hz. The TF graph showed the expected
wheel links below `base_link` and `base_footprint` below `odom`.

The joystick node was stopped during deterministic command tests because it
was continuously publishing competing zero commands on `/cmd_vel`.

## Lifted hardware tests

The robot was confirmed lifted with all wheels off the floor and supervised.
The following short commands were tested, each followed by an explicit zero
velocity command:

- Forward: `linear.x = 0.12` for approximately 1.5 seconds
- Strafe left: `linear.y = 0.12` for approximately 1.5 seconds
- Counter-clockwise rotation: corrected test with `angular.z = 0.12` for approximately 1.5 seconds

One initial rotation command incorrectly placed `0.12` in `angular.y`; that
attempt was invalid and was not counted. The corrected `angular.z` test was
then run.

The capture was recorded at `/tmp/x3_odom_probe` and reindexed after stopping
the recorder. It contained:

- Duration: 74.8 seconds
- `/cmd_vel`: 20 messages
- `/joint_states`: 747 messages
- `/vel_raw`: 747 messages
- `/odom_raw`: 749 messages
- `/odom`: 759 messages
- `/tf`: 1510 messages

After the lifted tests, the robot was stopped. The final sampled raw odometry
was approximately:

```text
x = 0.0657 m
y = -0.2240 m
heading = 0.169 rad
```

The encoder positions were nonzero and wheel velocities returned to zero.

## Floor test result

The robot was placed on the floor and a baseline was captured. A first
approximately 0.8 m forward test was affected by the joystick publisher and
produced no encoder change. The joystick node was then stopped and `/cmd_vel`
was verified to have zero publishers before retrying.

A deterministic second test sent `linear.x = 0.10` for approximately 3 seconds,
followed by zero velocity. The publisher sent 18 forward messages, but:

- Wheel encoder positions did not change.
- Wheel velocities remained zero.
- `/vel_raw` returned zero.
- `/odom_raw` pose did not change.

The driver still reported firmware edition `3.5` and battery voltage around
`10.1 V`, so the ROS driver was alive, but the motor command path did not
produce physical wheel motion in this floor test.

## Result

Software and graph validation passed. The lifted test produced encoder and
odometry data. Floor acceptance of the new odometry calculation is blocked by
the unresolved motor-command or hardware-motion issue. The odometry should
not yet be accepted as physically calibrated.

The next diagnostic should isolate the command path without more blind motion
tests: confirm whether the motor controller receives `set_car_motion`, inspect
serial/controller status during a supervised command, and verify motor enable,
battery/load behavior, and physical wheel response. Once a controlled floor
motion produces encoder deltas, repeat forward, strafe, rotation, and measured
course tests against `/odom_raw` and `/odom`.

## Safety state at end of session

- A zero `/cmd_vel` command was sent after motion tests.
- `/cmd_vel` had zero active publishers at the final check.
- `/vel_raw` was zero.
- Joint velocities were zero.
- No repository source files were modified during validation.
# Historical validation evidence

> This report describes the former `/odom_raw` + EKF graph and is retained only as dated robot evidence.
