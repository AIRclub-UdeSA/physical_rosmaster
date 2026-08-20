# X3-C Safety and Floor Odometry Validation Report - 2026-08-20

## Scope

Deploy the command-safety and odometry-hardening changes to the physical
X3-C, validate the clean ROS graph without motion, and continue to bounded
motion only if the physical preflight gates pass.

## Baseline

- Repository: `physical_rosmaster`
- Branch: `main`
- Revision before the uncommitted validation changes: `e160aa5`
- ROS distribution: ROS 2 Humble
- ROS domain: `11`
- Workspace: `/root/yahboomcar_ws`
- Robot: `x3-c`
- Physical state reported by the operator: on the floor in a clear safe space
- Floor type and measured clear perimeter: not recorded
- Existing untracked `robot_artifacts/` was preserved.

## Initial state and deployment

Autostart had launched a stale installed build with a competing joystick
publisher, two `robot_state_publisher` nodes, camera, and LiDAR. The live
driver did not have the new watchdog or encoder mapping parameters. A zero
`/cmd_vel` command was sent, the exact autostart processes were stopped, and
the graph and `/dev/ttyUSB0` ownership were verified clear.

The validated source was installed with:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select yahboomcar_base_node yahboomcar_bringup
```

Both packages built successfully. The installed launch was then started with
`use_joy:=false` and `pub_odom_tf:=false`.

## Controller and battery preflight

The installed `Rosmaster_Lib` matched public V3.3.9 at SHA-256
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.

A five-second, 50-sample direct serial probe on `/dev/ttyUSB0` sent no motor
commands. It reported:

- Firmware motion feedback: zero on all axes
- Raw encoder counts: stable at `[0, 0, 0, 1]`
- Battery voltage: `10.6-10.7 V`
- Controller edition after clean launch: `3.5`

The battery failed the checklist's `> 12.0 V` floor-test gate. No motion
command was issued at this point. Powered floor pulses were performed later
after the physical state was incorrectly recorded as lifted; the operator's
later clarification supersedes that record.

## Clean stationary graph

The clean graph contained one each of:

- `driver_node`
- `base_node`
- `ekf_filter_node`
- `imu_filter_madgwick`
- `robot_state_publisher`

Observed endpoint ownership:

- `/cmd_vel`: zero publishers, one subscriber from `driver_node`
- `/joint_states`: one publisher from `driver_node`
- `/odom_raw`: one publisher from `base_node`
- `/odom`: one publisher from `ekf_filter_node`
- `odom -> base_footprint`: available from the EKF while raw odom TF was disabled

The live driver loaded `cmd_vel_timeout: 0.5`, encoder order `[1, 0, 3, 2]`,
signs `[1.0, 1.0, 1.0, 1.0]`, and CPR `1040.0`. The order, signs, and CPR
remain provisional pending a lifted per-wheel test.

Measured rates were approximately:

- `/joint_states`: `10.000 Hz`
- `/vel_raw`: `10.000 Hz`
- `/odom_raw`: `10.001 Hz`

Across approximately 30 seconds, all four normalized joint positions and
velocities stayed at zero and `/odom_raw` stayed at `(x=0, y=0, yaw=0)`.
The EKF transform showed zero translation and less than approximately
`0.01 rad` stationary yaw variation.

## Evidence

A temporary stationary bag was saved outside Git at
`/tmp/x3_stationary_2026-08-20`:

- Duration: `52.664 s`
- Size: `1.8 MiB`
- Messages: `5322`
- `/joint_states`, `/vel_raw`, `/odom_raw`, `/odom`, `/imu/data_raw`,
  `/imu/data`, `/voltage`, and `/edition`: `527` messages each
- `/tf`: `1054` messages
- `/diagnostics`: `52` messages

## Floor motion follow-up (corrected classification)

At the time, the session record said that all four wheels were lifted and that
the low battery was accepted for lifted testing. The operator later clarified
that the robot was on the floor for all powered tests in this report. The clean
core was relaunched, its idle graph was revalidated, and a new bag was recorded
at `/tmp/x3_lifted_validation_2026-08-20`. The path retains its original,
misleading name so the evidence remains locatable.

The first two commands used the bounded pulse tool at 20 Hz, with redundant
zeros before and after each command:

| Trial | Command | Nonzero duration | Wheel delta under old provisional mapping `[FL, FR, BL, BR]` rad | Raw odom delta `[x, y, yaw]` |
| --- | --- | ---: | --- | --- |
| Forward | `x=+0.12` | `0.958 s` | `[+1.3533, +1.1479, +1.7641, +4.6580]` | `[+0.0751, -0.0172, +0.1344]` |
| Strafe | `y=+0.12` | `0.959 s` | `[+1.3775, -1.5527, -1.7218, +4.6459]` | `[+0.0422, -0.0691, +0.1719]` |

Forward wheel signs passed, but the wheel magnitudes were highly uneven and
the odometry accumulated significant yaw. The strafe signs were the exact
inverse of the checklist expectation `FL- FR+ BL+ BR-`, and raw odometry moved
in negative Y. Firmware feedback nevertheless reported positive Y during the
positive-Y command. This leaves two possibilities that must not be guessed:

1. The provisional left/right encoder channel assignments are reversed.
2. The X3 wheel/firmware lateral convention requires the opposite sign in the
   wheel-state kinematic equation.

The sequence stopped at the sign gate, so no rotation command was issued.
Wheel and firmware velocities returned to zero, a final explicit zero was
published, and the core exited cleanly.

The floor-test bag contains `17934` messages over `176.902 s`. During both command
windows the battery sagged to `10.3 V`. The reusable analysis command is:

```bash
source /opt/ros/humble/setup.bash
python3 tools/analyze_x3_odom_bag.py \
  /tmp/x3_lifted_validation_2026-08-20
```

While recording, the pulse tool initially encountered a validation-rule
conflict because rosbag is an additional passive `/cmd_vel` subscriber. The
tool was corrected to require exactly one actuator subscriber named
`driver_node`, allow `rosbag2_recorder`, and continue rejecting any competing
publisher or other actuator subscriber.

After two initial windows in which the operator had not rotated during the
active capture, a first synchronized no-command pass identified the dominant
channel for every physical wheel. That pass reported a negative back-right
delta, but the verbal direction did not have an unambiguous spatial reference.
It was enough to identify channels, not to diagnose encoder polarity or wiring.

After a powered-off inspection found that the motor cabling matched Yahboom's
diagram, the test was repeated with the operator, camera, and robot all facing
the same direction. Forward rolling was explicitly defined as moving the top
of the whole wheel toward the camera/front. The repeat produced:

| Physical wheel, rolled forward | Raw packet field | Dominant raw delta |
| --- | --- | ---: |
| Front-left | `m1` | `+6305` |
| Front-right | `m3` | `+5932` |
| Back-left | `m2` | `+6733` |
| Back-right | `m4` | `+5984` |

The back-left test had an incidental `m1` change of about `-36`. The back-right
test had incidental changes of about `m1 +109` and `m3 +83`. These were each
under 2% of the dominant change and do not affect channel identification. The
validated configuration is:

```yaml
encoder_order: [0, 2, 1, 3]
encoder_signs: [1.0, 1.0, 1.0, 1.0]
```

The rotations were approximate rather than exact marked turns, so they do not
establish CPR. `encoder_cpr: 1040.0` remains provisional.

During the front-right capture, the USB serial device reset and changed from
`/dev/ttyUSB0` to `/dev/ttyUSB2`. The stable symlink remained available at
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`; the driver configuration
and probe now use that path.

Reinterpreting the earlier floor pulses with the validated order and signs
gives forward wheel deltas of approximately
`[+1.1479, +4.6580, +1.3533, +1.7641] rad` and positive-Y deltas of
`[-1.5527, +4.6459, +1.3775, -1.7218] rad`. Their signs match the expected
forward pattern `FL+ FR+ BL+ BR+` and positive-Y pattern
`FL- FR+ BL+ BR-`. Their unequal magnitudes still require investigation and a
repeat under the corrected live configuration, documented below.

## Packet fields are not PCB port labels

The installed V3.3.9 `Rosmaster_Lib` decodes four consecutive signed encoder
fields from `FUNC_REPORT_ENCODER` and returns them as `(m1, m2, m3, m4)`. The
Python code contains no mapping from those packet positions to the controller's
printed motor-port labels. Comparing the packet-field names directly with the
official
[ROSMASTER X3 Wiring Introduction](https://github.com/YahboomTechnology/ROSMASTERX3/blob/main/01.About%20ROSMASTER%20X3/1.%20Wiring%20Introduction/1.%20Wiring%20Introduction.pdf)
therefore cannot diagnose a cable permutation.

The operator inspected the powered-off wiring against that diagram and found
no discrepancy. No cables were moved. The previously proposed `M1 <-> M4` and
`M2 <-> M3` swaps are withdrawn. The source configuration is corrected to the
measured packet-field mapping `[0, 2, 1, 3]` with all-positive signs.

The repeat direct captures sent no motion commands. Firmware motion remained
zero and battery voltage stayed between `10.3 V` and `10.4 V`. The probe then
exited and released the controller serial port.

## Corrected deployment and floor sign evidence

The measured mapping was installed as:

```yaml
encoder_order: [0, 2, 1, 3]
encoder_signs: [1.0, 1.0, 1.0, 1.0]
```

All 19 normal packages rebuilt successfully. The focused X3 regression suites
passed 12 C++ odometry tests, 7 Python driver-helper tests, and 2 pulse-recorder
gating tests. Targeted flake8, pydocstyle, Python compilation, and
`git diff --check` also passed. A broad
package-level `colcon test` reported 246 pre-existing lint failures in legacy
R2, calibration, patrol, and other untouched vendor files; the functional X3
tests themselves passed.

The clean core loaded the corrected values, showed zero idle `/cmd_vel`
publishers, exactly one driver subscriber, and one publisher each for
`/joint_states`, `/odom_raw`, and `/odom`. Stationary wheel, firmware, and raw
odom velocities were all zero.

Recorded corrected floor trials were below. Their evidence paths retain the
original `lifted` labels, which are misleading after the operator's correction:

| Command | Nonzero duration | Wheel delta `[FL, FR, BL, BR]` rad | Raw odom `[x, y, yaw]` | Evidence bag |
| --- | ---: | --- | --- | --- |
| `x=+0.12` | `0.757 s` | `[+0.6223, +2.0904, +0.6525, +1.9031]` | `[+0.0384, +0.0206, +0.1359]` | `/tmp/x3_lifted_corrected_qos_2026-08-20` |
| `y=+0.12` | `0.757 s` | `[-0.5800, +0.8337, +0.2356, -0.1269]` | `[-0.0047, +0.0141, +0.0526]` | `/tmp/x3_lifted_corrected_strafe_rotate_2026-08-20` |
| `yaw=+0.12` | `0.758 s` | `[0, 0, 0, 0]` | `[0, 0, 0]` | `/tmp/x3_lifted_corrected_rotate_verified_2026-08-20` |
| `yaw=+0.30` | `0.757 s` | `[0, 0, 0, 0]` | `[0, 0, 0]` | `/tmp/x3_lifted_corrected_rotate_030_2026-08-20` |
| `yaw=+0.50` | `0.757 s` | `[-0.1752, +0.4833, -0.0242, +0.2175]` | `[+0.0017, +0.0051, +0.0450]` | `/tmp/x3_lifted_corrected_rotate_050_2026-08-20` |

The floor observations matched the forward pattern `FL+ FR+ BL+ BR+`,
strafe-left pattern `FL- FR+ BL+ BR-`, and CCW pattern
`FL- FR+ BL- BR+`. They do not complete the lifted sign gates. The `+0.12` and
`+0.30 rad/s` yaw commands did not move the encoders on the floor; `+0.50 rad/s`
was the first tested yaw command to break through the loaded drivetrain. Wheel magnitudes were strongly
unequal in every moving trial, with a particularly weak back-left response in
the rotation trial. This leaves CPR, controller/motor response, low-voltage
behavior, and per-wheel mechanical variation as calibration work; it does not
invalidate the now-repeated channel-order and sign evidence.

The first two forward recording attempts were not accepted as evidence. One
bag missed `/cmd_vel` discovery and another captured only an incompatible-QoS
zero publisher. A later rotation bag likewise missed the short-lived command
publisher. The recorder procedure now uses a volatile `/cmd_vel` QoS override,
regex discovery, and `safe_cmd_vel_pulse.py --require-recorder`; the analyzer
also supports the safety tool's timestamped `/rosout` record as a fallback when
that record is available.

## Result and blockers

Deployment and stationary graph checks passed. The direction-controlled hand
test validates packet order and forward polarity; the floor trials independently
match all three expected wheel-sign patterns, and the wiring-fault diagnosis is
withdrawn. The lifted wheel-sign gates remain outstanding. Ground-truth odometry
calibration is not accepted because CPR is provisional, wheel magnitudes are
strongly imbalanced, there was no external motion measurement, and the battery
was only `10.3-10.4 V` before motion. Because every powered test was on the
floor, the checklist's floor-voltage gate was bypassed. No further floor motion
should occur until the battery is above `12.0 V` and preflight is repeated.

## Safety state at end of session

- Every nonzero command used the bounded safety tool and lasted no more than
  approximately `1.0 s`; all were performed with the robot on the floor.
- Recorder setup required three forward pulses, one strafe pulse, two
  `yaw=+0.12` pulses, one `yaw=+0.30` pulse, and one `yaw=+0.50` pulse in the
  corrected session; incomplete bags are explicitly excluded above.
- The safety tool sent redundant zeros after every pulse. Final wheel,
  firmware, and raw-odom velocities were zero, with no `/cmd_vel` publisher.
- The clean core exited normally and the driver shutdown path ran.
- The motor driver remained stopped during every repeat hand capture.
- The direct probe exited and released the motor-controller serial port.
- The stable controller path was released; after the latest power cycle it
  resolves to `/dev/ttyUSB0`.
- The actuator core is stopped. Unrelated camera and lidar autostart processes
  were outside this validation and may still be running.
- The latest direct-probe battery readings were `10.3-10.4 V`. Robot power
  remains under operator control and was not independently switched off.
- Contrary to the earlier session record, no true lifted powered validation was
  performed. That validation remains a required future step.
