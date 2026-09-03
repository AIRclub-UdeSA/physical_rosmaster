# X3-C Lifted Odometry Validation Report - 2026-08-21

## Scope

Complete the true lifted Phase 3 wheel-sign and raw-odometry integration gates
that remained outstanding after the 2026-08-20 floor tests.

## Baseline and physical state

- Host: `x3-c`, inside the `rosmaster_humble` container
- Repository branch: `main`
- Source revision: `079ee0b` (`docs: align X3 validation with confirmed wiring`)
- ROS distribution/domain: Humble, `ROS_DOMAIN_ID=11`
- The operator explicitly confirmed that the robot was lifted before motion.
- Direct no-command probe: stable zero firmware motion, encoder packet fields
  `[0, 0, 0, 0]`, and battery `11.3 V`
- Installed `Rosmaster_Lib`: public V3.3.9 SHA-256 match
- Focused regression: 9 Python safety/mapping tests and 12 C++ odometry tests
  passed

Autostart initially provided duplicate `robot_state_publisher` nodes. The exact
core and display launch trees were stopped, their orphaned children were
removed, and the controller serial port was verified free. Camera and LiDAR
were left running because they do not command the drivetrain.

## Clean pre-motion graph

One clean core was launched with joystick input disabled and raw-odom TF
disabled. Before motion:

- one each of `driver_node`, `base_node`, `ekf_filter_node`,
  `imu_filter_madgwick`, and `robot_state_publisher`
- zero `/cmd_vel` publishers and exactly one actuator subscriber from
  `driver_node`
- one publisher each for `/joint_states`, `/odom_raw`, and `/odom`
- `/joint_states` and `/odom_raw` at approximately `10.0 Hz`
- loaded `encoder_order: [0, 2, 1, 3]`
- loaded `encoder_signs: [1.0, 1.0, 1.0, 1.0]`
- loaded `cmd_vel_timeout: 0.5 s`
- wheel positions/velocities and raw-odom pose/twist remained zero through the
  recorded stationary baseline

## Recorded procedure

Evidence bag:
`/tmp/x3_odom_validation_2026-08-21_lifted_r1`

- Size: `8.0 MiB`
- Duration: `237.592 s`
- Messages: `24127`
- Recorded `/cmd_vel` messages: `114`
- Command windows were delimited directly from `/cmd_vel`, not the log fallback.
- Database SHA-256: `1f3cbc09f0b22599b0483cb710ff69482a44c439833ef10adb9b76d213b40e05`
- Metadata SHA-256: `fe35cc573908783ee073f66e13400f2135e2faafc53a886e5ea0addd222e832a`

Every command used `safe_cmd_vel_pulse.py --require-recorder`, which required
one driver actuator subscriber, a compatible rosbag subscription, and no other
velocity publisher. The tool sent redundant zeros before and after each pulse.

## Results

| Command | Recorded duration | Wheel delta `[FL, FR, BL, BR]` rad | Raw odom delta `[x, y, yaw]` | Peak firmware `[vx, vy, wz]` | Voltage |
| --- | ---: | --- | --- | --- | --- |
| `x=+0.20` | `1.464 s` | `[+7.3405, +9.4006, +7.9506, +7.9869]` | `[+0.2667, +0.0485, +0.1048]` | `[0.1970, 0.0340, 0.2580]` | `10.9-11.3 V` |
| `y=+0.20` | `1.466 s` | `[-7.6123, +9.4369, +7.9627, -7.8056]` | `[-0.0340, +0.2689, +0.0640]` | `[0.0230, 0.1930, 0.2820]` | `10.9-11.3 V` |
| `yaw=+0.50` | `1.463 s` | `[-2.6281, +4.7426, -2.3864, +2.8335]` | `[+0.0178, +0.0228, +0.6295]` | `[0.0270, 0.0270, 0.5170]` | `11.0-11.3 V` |

The forward pattern was `FL+ FR+ BL+ BR+`, strafe-left was
`FL- FR+ BL+ BR-`, and CCW rotation was `FL- FR+ BL- BR+`. Raw odometry
integrated positively on the commanded axis in all three trials. All Phase 3
sign and integration gates passed.

The deltas also show measurable per-wheel magnitude and cross-axis leakage.
Because the chassis was lifted and there was no external displacement reference,
these trials do not calibrate CPR, distance, yaw scale, covariance, or floor
behavior.

## Warnings and final safety state

- One EKF update-rate overrun was observed: `0.204 s`.
- No encoder discontinuity, stale-input, duplicate-publisher, or TF-authority
  warning was observed.
- Wheel, firmware, and raw-odom velocities returned to zero after every pulse.
- `/cmd_vel` had zero publishers after every pulse.
- The rosbag recorder closed normally and the full bag analyzed successfully.
- The clean core received `SIGINT`; every launched process, including the
  driver, exited cleanly.
- The controller serial port was released and the actuator graph was absent at
  the end of the session.

## Result

The corrected encoder mapping, polarity, mecanum sign conventions, and
wheel-state-to-`/odom_raw` integration path pass true lifted validation.
Ground-truth floor calibration still requires a pack confirmed fully charged
with paired multimeter/controller readings, exact marked CPR measurement, and
externally measured repeatable floor trials. The earlier generic `> 12.0 V`
floor threshold was superseded after this report's lifted run.
# Historical validation evidence

> This lifted report was recorded against the former `/odom_raw` + EKF graph. Encoder order/sign results remain evidence; current canonical wheel odometry publishes `/odom` directly.
