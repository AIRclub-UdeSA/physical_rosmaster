# X3-C Floor Odometry Validation Report - 2026-08-21

## Scope

Record the first fully charged-pack floor response after the true lifted sign
gates passed, verify that physical motion and wheel/raw-odometry feedback agree,
and preserve the evidence needed for later workstation analysis.

## Battery gate correction

The operator charged the installed pack fully and measured `11.7 V` directly
with a multimeter. Controller/ROS telemetry read `11.2-11.3 V` at idle. This
pack-specific evidence supersedes the earlier generic `> 12.0 V` checklist
threshold. Future preflight records paired multimeter/controller readings and
under-load sag instead of applying that incorrect absolute gate.

## Pre-motion state

- Host/container: `x3-c`, `rosmaster_humble`
- Source revision before report edits: `079ee0b` on `main`
- The operator confirmed a level, clear floor setup and supervised the motion.
- Autostart duplicates were absent; camera and LiDAR remained as non-actuating
  nodes.
- One clean core used `use_joy:=false` and `pub_odom_tf:=false`.
- `/cmd_vel` had zero publishers and exactly one driver actuator subscriber
  before the recorder joined.
- Loaded mapping: `encoder_order: [0, 2, 1, 3]`, all-positive signs
- Stationary wheel positions/velocities and raw-odom pose/twist were zero.
- Controller voltage immediately before motion was `11.2 V`.

## Evidence bag

Path: `/tmp/x3_odom_validation_2026-08-21_floor_r1`

- Size reported by rosbag: `18.3 MiB`
- Duration: `535.601 s`
- Messages: `55148`
- `/cmd_vel` messages: `1044`
- Analyzed command windows: `31`
- Database SHA-256: `7445d6a972f63d37f876c3b567ffc548a38f737a93940e227d4e57cbe27ee096`
- Metadata SHA-256: `0e4ff591973310820d3ba2f79a42e5316420d699225b343b61b0da7a978cc1fc`

The bag remains outside Git. Its summary and checksums are committed so a later
copy can be verified.

## Accepted bounded trials

Both trials used `safe_cmd_vel_pulse.py --require-recorder`, with redundant
zeros before and after motion:

| Command | Recorded duration | Wheel delta `[FL, FR, BL, BR]` rad | Raw odom delta `[x, y, yaw]` | Peak firmware `[vx, vy, wz]` | Voltage |
| --- | ---: | --- | --- | --- | --- |
| `x=+0.15` | `0.962 s` | `[+2.2535, +3.2866, +2.3381, +3.0208]` | `[+0.0895, +0.0093, +0.0858]` | `[0.1230, 0.0150, 0.1640]` | `11.0-11.2 V` |
| `x=+0.15` | `2.978 s` | `[+13.8653, +14.9709, +14.1492, +14.5238]` | `[+0.4677, +0.0806, +0.0740]` | `[0.1660, 0.0150, 0.1880]` | `11.2 V` |

The operator did not clearly see the first short movement. The three-second
trial was clearly visible and described as visually smooth. Raw odometry
represented approximately `0.475 m` total planar displacement. The operator's
physical measurement was not precise enough to record an accepted ground-truth
distance, lateral offset, or heading change. These trials therefore validate
qualitative response and repeatable breakaway at `0.15 m/s`, not scale accuracy.

The wheel magnitudes were substantially more balanced in the three-second run
than in earlier short floor pulses. Raw odometry still reported lateral and yaw
leakage that was not obvious to the operator.

## Keyboard exploration

After the bounded trials, the standard `teleop_twist_keyboard` package produced
29 additional nonzero command windows. Its installed Humble script hardcodes
initial values of `0.5 m/s` and `1.0 rad/s` and ignores ROS speed parameters.
The recorded commands used those defaults rather than the intended reduced
values.

The keyboard windows exercised forward, reverse, positive/negative lateral,
diagonal, and CCW rotation commands. They are retained as qualitative
stress/directional evidence only because:

- exact physical start/end measurements and maneuver annotations were not
  captured;
- no clearly isolated CW rotation window was recorded;
- several single-key windows have a recorded nonzero duration of `0.000 s`;
- the current analyzer subtracts wrapped quaternion yaw and therefore reports
  misleading near-`2*pi` jumps for some rotation-adjacent windows;
- settled-state windows can overlap closely spaced keyboard commands.

The driver watchdog issued 11 expected timeout stops after sparse keyboard
input, between `0.503 s` and `0.548 s`. This confirms the watchdog intervened;
it also means keyboard windows should not be interpreted like the bounded pulse
trials. A configurable, initially bounded teleop should replace this procedure.

## Final state and result

- Controller voltage stayed between `11.0 V` and `11.2 V` in the analyzed
  floor windows.
- Wheel, firmware, and raw-odom velocities were zero before shutdown.
- No teleop or other `/cmd_vel` publisher remained.
- The recorder closed normally and the bag analyzed successfully.
- The clean core received `SIGINT`; all launched processes exited cleanly and
  the controller serial port was released.

The floor run confirms that the robot breaks away repeatably at `0.15 m/s` and
that physical forward motion, four-wheel feedback, firmware velocity, and raw
odometry respond together. It does not calibrate encoder CPR, linear/yaw scale,
or covariance. Those require marked wheel turns and repeated externally measured
straight, strafe, CW, and CCW trials.
# Historical validation evidence

> This floor report was recorded against the former `/odom_raw` + EKF graph. Motion and encoder observations remain evidence; current canonical wheel odometry publishes `/odom` directly.
