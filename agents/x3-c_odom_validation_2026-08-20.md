# X3-C Safety and Lifted Odometry Validation Report - 2026-08-20

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
command was issued.

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

## Lifted motion follow-up

The operator subsequently confirmed that all four wheels were lifted and
explicitly accepted proceeding despite the low battery. The clean core was
relaunched, its idle graph was revalidated, and a new bag was recorded at
`/tmp/x3_lifted_validation_2026-08-20`.

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

The lifted bag contains `17934` messages over `176.902 s`. During both command
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
active capture, synchronized no-command hand tests identified all four raw
channels and directions:

| Physical wheel, rotated forward | Raw channel | Dominant raw delta |
| --- | --- | ---: |
| Front-left | `m1` | `+1371` |
| Front-right | `m3` | `+1424` |
| Back-left | `m2` | `+866` |
| Back-right | `m4` | `-1886` |

The back-left capture included approximately `-175` incidental ticks on `m1`,
but the `m2` response was dominant. The back-right capture included only small
incidental changes (`m1 +28`, `m3 +9`). The observations establish the
pre-rewire cable state:

```yaml
encoder_order: [0, 2, 1, 3]
encoder_signs: [1.0, 1.0, 1.0, -1.0]
```

The operator rotated each wheel approximately, not by an exact marked turn,
so these observations do not establish CPR. `encoder_cpr: 1040.0` remains
provisional.

During the front-right capture, the USB serial device reset and changed from
`/dev/ttyUSB0` to `/dev/ttyUSB2`. The stable symlink remained available at
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`; the driver configuration
and probe now use that path.

Reinterpreting the earlier powered pulses with the confirmed order and signs
gives forward wheel deltas of approximately
`[+1.1479, +4.6580, +1.3533, -1.7641] rad` and positive-Y deltas of
`[-1.5527, +4.6459, +1.3775, +1.7218] rad`. In both cases the normalized
back-right direction is inconsistent with the requested chassis motion. This
must be rechecked under the corrected live configuration while lifted; the
robot must not proceed to floor motion until all four physical directions are
correct.

## Wiring diagnosis and planned correction

The operator raised the possibility that motor cables had been connected to
the wrong controller ports. Yahboom's official
[ROSMASTER X3 Wiring Introduction](https://github.com/YahboomTechnology/ROSMASTERX3/blob/main/01.About%20ROSMASTER%20X3/1.%20Wiring%20Introduction/1.%20Wiring%20Introduction.pdf)
specifies the factory physical layout as:

| Factory controller port | Physical wheel |
| --- | --- |
| `M4` | Front-left |
| `M2` | Front-right |
| `M3` | Back-left |
| `M1` | Back-right |

This differs from every measured pre-rewire cable assignment. Restoring the
factory layout requires two complete-cable swaps with robot and motor power
fully disconnected:

- `M1 <-> M4`, moving front-left to `M4` and back-right to `M1`.
- `M2 <-> M3`, moving front-right to `M2` and back-left to `M3`.

The connectors must remain keyed as built; no plug reversal or individual-pin
change is intended. The source now carries the expected post-rewire order
`[FL, FR, BL, BR] = [m4, m2, m3, m1]`, encoded as
`encoder_order: [3, 1, 2, 0]`, with provisional all-positive signs. These
parameters have not been rebuilt or deployed and must be verified by repeating
the no-command hand test after the cable move.

A final 10-sample direct probe sent no motion commands. Firmware motion stayed
zero, encoders stayed at `[2126, 1318, 2970, -1877]`, and battery voltage was
`10.5 V` in every sample. The serial port was then released so the robot could
be powered down and workstation-only work could continue.

## Result and blockers

Deployment and stationary Phase 3 graph validation passed. The hand test and
official factory diagram diagnosed a complete motor-port permutation. Lifted
kinematic validation is not accepted until the powered-off cable swaps are
completed, the hand mapping is repeated, and the corrected configuration is
rebuilt for bounded forward, strafe, and rotation tests. The low battery
remains a recorded limitation; the operator explicitly accepted it for lifted
testing, while the checklist floor-test gate remains unmet.

## Safety state at end of session

- The only nonzero commands were the two bounded lifted pulses documented above.
- A final explicit zero `/cmd_vel` command was issued.
- The clean core exited normally; the driver shutdown path ran.
- The final ROS graph had no `/cmd_vel` topic.
- The stable controller path was released; it currently resolves to `/dev/ttyUSB2`.
- Autostart processes remained stopped for this container session.
- The final battery reading was a stable `10.5 V`; robot power-off was left to
  the operator and was not independently confirmed.
