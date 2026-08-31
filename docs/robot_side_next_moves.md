# Next robot session: close PR #3 hardware gates

This runbook continues the `x3-c` verification recorded on 2026-08-26. Its goal
is to close the remaining physical-platform gates for PR #3. It does not
authorize autostart work.

Use [robot_side_verification_todo.md](robot_side_verification_todo.md) as the
acceptance checklist and
[the first robot report](../robot_artifacts/pr3_robot_verification_2026-08-26.md)
as historical evidence. Record continuation results in
[the 2026-08-30 recovery session](../robot_artifacts/pr3_recovery_session_2026-08-30.md).
Record the exact current PR head used for the next run; do not report a parent
commit plus uncommitted fixes as the accepted source.

The target configuration is still the canonical new-robot setup in the root
README and [setup guide](setup_guide_ros2_humble_autostart.md): one clean Humble
workspace, exact robot library, stable hardware identities, manual strict
bringup, and no default command publisher. The recovery steps in this runbook
add evidence preservation, host/storage health checks, and removal of competing
factory launch paths; they do not authorize restoring the old host wholesale or
creating a second deployment architecture.

## Required order

1. preserve the previous evidence;
2. establish a safe, charged, persistent host configuration;
3. deploy and build a clean current PR head;
4. repeat the positive sensor contract and the three fail-closed checks;
5. resolve the uneven lifted-wheel response;
6. verify operator tools;
7. perform repeated measured floor trials;
8. prove simulator/physical consumer parity and finish the handoff.

Do not move to a later gate because an earlier failure appears unrelated.

## 1. Preserve the previous evidence

Before cleaning the robot workspace or `/tmp`:

- [x] Copy the three current PR #3 rosbag `.db3` files, all attempt-1-through-6
  launch logs, contract-probe output, and isolated build/test logs to durable
  project storage.
- [x] Generate SHA256 hashes for every archived payload.
- [x] Record durable artifact URIs and hashes in the
  [2026-08-30 recovery manifest](../robot_artifacts/pr3_recovered_evidence_2026-08-30.md).
- [x] Confirm the stored bags open with `ros2 bag info`; committed metadata alone
  is not replayable evidence.

The read-only recovery also preserved the older legacy probe bag. All four
SQLite databases pass `PRAGMA integrity_check`, and all four open successfully
with `ros2 bag info` in the recovered robot's ROS 2 Humble container.

If an ephemeral artifact is already gone, mark it lost explicitly. Do not infer
its contents from metadata or silently count it as archived acceptance evidence.

## 2. Establish the host and safety baseline

- [x] Fully charge the robot battery and record rested voltage before launch.
  The charged 2026-08-31 repeat measured `11.400` V at idle; voltage under
  motion load remains pending.
- [ ] Record the observer, stop path, surface, payload, and how the lifted robot
  is restrained.
- [x] Confirm `/root/auto_start.sh` is absent, the external container starts only
  `bash`, and no host service invokes a legacy ROS launch.
- [x] Stop and persistently disable the factory graphical
  `/home/pi/.config/autostart/rosmaster.desktop` entry. The 2026-08-30 preflight
  found that its `rosmaster_main.py` process owns the motor serial device,
  listens on LAN ports 6000/6500, and contains motor/servo command handlers.
- [x] Reboot and verify that `rosmaster_main.py`, ports 6000/6500, and all legacy
  ROS graphs remain absent before allowing the container to open the controller.
- [x] Reboot the host and confirm the legacy ROS graph does not return on its old
  domain or publish `/cmd_vel`.
- [x] Install the reviewed
  `robot_artifacts/99-rosmaster-x3.x3-c.rules` file and the pinned Orbbec USB
  permission rules on the host.
- [x] Disable the conflicting factory `usb.rules`, reload udev, reconnect the
  devices, reboot, and verify `/dev/robot/motor` and `/dev/robot/lidar`.
- [x] Verify the motor/LiDAR aliases and camera permissions are visible inside
  the container.
- [x] Confirm Astra serial `ACRC64300ET` through the pinned camera driver's
  device-discovery path before strict bringup.
- [x] Reboot with all devices connected and confirm that both Astra functions
  (`2bc5:060f` depth and `2bc5:050f` UVC/RGB) enumerate without hotplug. At the
  charged baseline, moving only the camera from a direct Pi USB 3 port to
  downstream port 4 of the powered Yahboom hub produced one successful cold
  boot and three consecutive successful warm reboots. Both functions remained
  under parent `1-1.4` as `1-1.4.1` and `1-1.4.2`; the pinned driver found the
  single expected Astra serial `ACRC64300ET`. The earlier direct-Pi-port failure
  remains a rejected topology, not an accepted hotplug workaround.

The motor alias depends on physical USB topology because its CH340 exposes no
serial. Stop if `KERNELS=="1-1.2"` does not identify the dedicated controller
port after reboot.

Keep the camera on Yahboom hub downstream port 4. Moving it back to a direct Pi
port or changing the motor-controller port reopens the stable-identity gate.

Do not resume motion if the charged system returns to the previous `10.2` to
`10.3` V condition. Inspect the battery, charger, controller supply, and voltage
drop first.

For a pre-launch voltage check, use the passive auto-report path in
`tools/rosmaster_lib_probe.py`. Do not instantiate public Rosmaster_Lib 3.3.9
merely to read voltage: its constructor transmits a UART-servo torque-enable
command. The probe must report the expected library hash and obtain telemetry
without calling `Serial.write()`.

## 3. Deploy one exact, clean revision

Inside the ROS 2 Humble container:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git fetch origin
git checkout platform/simulator-parity
git pull --ff-only
git status --porcelain
git rev-parse HEAD
git rev-parse origin/platform/simulator-parity
```

- [x] `git status --porcelain` is empty.
- [x] The robot `HEAD` equals its cached `origin/platform/simulator-parity` at
  `55caf7a2a572aae0ad2682e265147c46e525921c`, with an empty worktree. The
  workstation and server branch now resolve to
  `3a99fb5f6665f50cf811262fb2c2dc1893895aed`; the intervening commits change
  documentation and evidence only, so the runtime source under test remains
  exact `55caf7a` content.
- [x] Record source-under-test hash
  `55caf7a2a572aae0ad2682e265147c46e525921c`.
- [x] Confirm `physical_rosmaster.repos` still resolves the pinned Astra commit
  `f7e71d9ce806e788cb48d8580aac2c778fba4214`.
- [x] Run `rosdep install`, then perform a clean isolated ten-package build and
  the required eight-package test selection.
- [x] Require 0 build failures, 0 test errors, and 0 test failures; after adding
  the Astra launch regression test, the result was 66 tests, 0 errors, 0
  failures, and 3 intentional skips.
- [x] Reconfirm the installed `Rosmaster_Lib` SHA256 is
  `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.

Do not edit code on the robot during acceptance. If a fix is required, stop the
gate, implement and commit it on the branch, then restart this section from a
clean checkout of the new PR head.

## 4. Repeat the strict sensor gate and fail-closed tests

With the wheels lifted and no command publisher, export the verified identities
and start only the strict platform bringup:

```bash
export ROSMASTER_MOTOR_PORT=/dev/robot/motor
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL=ACRC64300ET
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Run `python3 tools/physical_contract_probe.py` from a second sourced shell.

- [ ] The complete positive contract passes twice: once before motion and once
  after the lifted tests. The pre-motion half is complete at clean runtime head
  `55caf7a`: both the default five-sample and extended 30-sample probes passed
  every required topic, with `/tf_static=2` and zero `/cmd_vel` publishers. The
  required post-motion repeat remains.
- [x] CPU use remains bounded with aligned 320x240 RGB-D and the colored cloud
  arrives consistently. The latest 30-second cloud check ended at `16.127` Hz
  after 27 rolling averages from `8.616` to `23.435` Hz; total container CPU was
  `80.72--87.34%`, adapter CPU `46.3--46.5%`, memory `340.3--346.0` MiB,
  temperature `48.3` C, and throttling `0x0`.
- [x] No default `/cmd_vel` publisher appears.
- [x] Camera absent: the adapter exited code 1 after 20.23 seconds with all five
  required streams missing; the enclosing launch stopped every other node.
- [x] LiDAR absent: `sllidar_node` reported error `80008004`, exited code 255,
  and the enclosing launch immediately stopped every other node.
- [x] Motor feedback absent at startup: with the physical controller and its
  alias absent before launch, the driver exited with code 1 and the enclosing
  strict bringup stopped every other node. The shutdown also reproduced the
  partially initialized driver destructor `AttributeError`, Astra exit `-6`,
  and LiDAR teardown findings; track those separately from the successful
  startup fail-closed result.
- [ ] Live motor-controller loss fails closed. Runtime head `55caf7a` fails this
  gate: after the physical controller disappeared, the Rosmaster receive thread
  raised `SerialException`, but the driver stayed alive, diagnostics remained
  level `OK`/healthy with zero failure counters, and cached `/joint_states`,
  `/voltage`, and `/vel_raw` continued to look fresh. No command publisher was
  present, but stale healthy feedback is not acceptable.
- [ ] After each absence test, restore the device and pass the full contract
  before continuing. Camera, LiDAR, and startup-motor restoration have passed
  the full positive contract. The motor identity and passive zero telemetry
  returned after the live-loss shutdown, but its post-fix full-contract repeat
  remains, so this aggregate item stays unchecked.

Perform device-absence checks one at a time with power controlled safely. Never
disconnect a motor-controller interface while a command source exists.

The current motor driver does not detect sustained live controller loss, and
the contract probe can continue receiving falsely fresh controller-derived
topics.
Stop here on runtime `55caf7a`. Do not start any motion, joystick, keyboard,
calibration, watchdog, or floor gate until a versioned freshness/device-loss fix
has passed clean build/test, deployment, startup absence, live disconnect, and
restoration tests.

## 5. Resolve the weak and uneven lifted-wheel response

This entire section is blocked by the live motor-controller-loss failure on
runtime `55caf7a`.

During the post-hub strict no-command bringup, two contract probes, and sensor
soak, the operator observed no wheel movement. This establishes quiet startup
only; it does not replace the lifted command-response tests below.

The previous lifted run established the expected signs but did not establish
usable motion quality. At `10.2` to `10.3` V, wheel response was strongly uneven
and a one-second `+0.30 rad/s` yaw command produced only `+0.0124 rad` of odometry
yaw. Low voltage is a test condition, not a proven root cause.

Before motion:

- [ ] Record fully charged rested voltage and voltage under command load.
- [ ] Inspect wheel freedom, mecanum roller orientation, motor connectors,
  controller channels, cable strain, and mechanical interference.
- [ ] Start a new bag containing `/cmd_vel`, `/joint_states`, `/odom`, `/tf`,
  `/diagnostics`, `/vel_raw`, `/voltage`, and `/rosout`.

With the robot securely lifted, repeat each bounded pulse at least three times:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --x -0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --y 0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --y -0.10 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --yaw 0.30 --duration 1.0 --require-recorder
python3 tools/safe_cmd_vel_pulse.py --yaw -0.30 --duration 1.0 --require-recorder
```

Analyze each bag with `tools/analyze_x3_odom_bag.py` and compare individual wheel
deltas across repetitions.

Proceed to the floor only if:

- [ ] every expected wheel responds consistently with the correct sign;
- [ ] no wheel repeatedly remains near zero while its peers move;
- [ ] forward and lateral yaw drift is no longer dominated by a large repeated
  wheel imbalance;
- [ ] both CW and CCW commands produce repeatable usable rotation;
- [ ] voltage, diagnostics, encoders, odometry, watchdog, and shutdown remain
  healthy.

If imbalance remains after charging, stop the floor gate and diagnose the
mechanical, motor, controller-channel, wiring, or power path. Do not tune geometry,
CPR, scale, covariance, or encoder mapping to hide unequal actuator response.

Track the Astra parameter-undeclare shutdown messages and SLLidar SIGTERM
escalation separately. They block shutdown-quality acceptance if they prevent a
predictable clean stop, but they do not explain uneven wheel motion.

## 6. Complete operator-tool safety

Joystick, keyboard, calibration, and watchdog testing are blocked by the live
motor-controller-loss failure until the fixed driver passes the Section 4
disconnect and restoration gates.

Run one command source at a time with the wheels lifted:

- [ ] Joystick moves only while the configured deadman is held.
- [ ] Deadman release, input timeout, disconnect, malformed input, and process
  shutdown each command zero.
- [ ] Keyboard timeout and shutdown each command zero.
- [ ] Linear and angular calibration publish no motion with `start_test=false`.
- [ ] An activated bounded calibration stops on success, TF loss, error, timeout,
  and shutdown.
- [ ] The motor watchdog still stops an interrupted stream within its configured
  timeout.

## 7. Perform measured floor acceptance

Floor testing is blocked by the live motor-controller-loss failure and by all
remaining lifted-motion and operator-tool gates.

Use a clear level area, a human observer, the tested stop path, one command
publisher, conservative bounds, and an external distance/heading measurement.

- [ ] Repeat `+X` and `-X` at least three times each.
- [ ] Repeat left and right strafe at least three times each.
- [ ] Repeat CW and CCW rotation at least three times each.
- [ ] Record command, duration, battery voltage, wheel deltas, odometry delta,
  measured displacement/heading, surface, and payload for every trial.
- [ ] Confirm qualitative direction and scale without non-finite data, resets,
  TF conflicts, discontinuities, or sustained diagnostic errors.
- [ ] Repeat watchdog and operator-stop checks under floor load.

Separate repeated systematic odometry error from normal mecanum slip. Propose
geometry or covariance changes only from repeated measured evidence and review
them as a new code change.

## 8. Prove parity and finish PR #3

- [ ] Record one external consumer repository and commit.
- [ ] Run it against simulator commit `772ba25`.
- [ ] Run the same commit against this X3 with no topic remaps, frame
  substitutions, source edits, or hardware-only branches.
- [ ] Archive the new bags and logs in durable storage and commit a small manifest
  containing artifact URIs, SHA256 hashes, commands, results, and the tested PR
  head.
- [ ] Update the robot report and PR #3 description to distinguish passed,
  failed, and pending gates.
- [ ] Obtain a second review of the evidence and shutdown-quality observations.

PR #3 may leave draft only after every required physical gate passes. The new
autostart routine remains a separate follow-up change after the platform is
accepted; it must not be used to complete or bypass this runbook.

That follow-up must branch from the exact accepted simulator-parity platform,
not from the pre-cleanup tag, factory image scripts, or an old autostart commit.
It should add versioned host/service and per-robot identity configuration, run
the single strict platform launch in a failure-propagating foreground path,
perform host/device/storage preflight checks, preserve logs, and stop
gracefully. It must not start any command publisher or application behavior by
default, and it requires its own reboot, failure-injection, and shutdown
validation.
