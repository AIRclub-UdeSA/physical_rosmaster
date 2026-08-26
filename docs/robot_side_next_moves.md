# Next robot session: close PR #3 hardware gates

This runbook continues the `x3-c` verification recorded on 2026-08-26. Its goal
is to close the remaining physical-platform gates for PR #3. It does not
authorize autostart work.

Use [robot_side_verification_todo.md](robot_side_verification_todo.md) as the
acceptance checklist and
[the first robot report](../robot_artifacts/pr3_robot_verification_2026-08-26.md)
as historical evidence. Record the exact current PR head used for the next run;
do not report a parent commit plus uncommitted fixes as the accepted source.

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

- [ ] Copy the three rosbag `.db3` files, all attempt-1-through-6 launch logs,
  contract-probe output, and isolated build/test logs to durable project storage.
- [ ] Generate SHA256 hashes for every archived payload.
- [ ] Record durable artifact URIs and hashes in the robot verification report.
- [ ] Confirm the stored bags open with `ros2 bag info`; committed metadata alone
  is not replayable evidence.

If an ephemeral artifact is already gone, mark it lost explicitly. Do not infer
its contents from metadata or silently count it as archived acceptance evidence.

## 2. Establish the host and safety baseline

- [ ] Fully charge the robot battery and record rested voltage before launch.
- [ ] Record the observer, stop path, surface, payload, and how the lifted robot
  is restrained.
- [ ] Persistently disable the legacy `/root/auto_start.sh` path in the external
  container launcher and any host service that can invoke it.
- [ ] Reboot the host and confirm the legacy ROS graph does not return on its old
  domain or publish `/cmd_vel`.
- [ ] Install the reviewed
  `robot_artifacts/99-rosmaster-x3.x3-c.rules` file and the pinned Orbbec USB
  permission rules on the host.
- [ ] Reload udev, reconnect the devices, reboot, and verify
  `/dev/robot/motor`, `/dev/robot/lidar`, and Astra serial `ACRC64300ET`.
- [ ] Verify the aliases and camera permissions are visible inside the container.

The motor alias depends on physical USB topology because its CH340 exposes no
serial. Stop if `KERNELS=="1-1.2"` does not identify the dedicated controller
port after reboot.

Do not resume motion if the charged system returns to the previous `10.2` to
`10.3` V condition. Inspect the battery, charger, controller supply, and voltage
drop first.

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

- [ ] `git status --porcelain` is empty.
- [ ] Local `HEAD` equals `origin/platform/simulator-parity`.
- [ ] Record that hash as the source under test.
- [ ] Confirm `physical_rosmaster.repos` still resolves the pinned Astra commit
  `f7e71d9ce806e788cb48d8580aac2c778fba4214`.
- [ ] Run `rosdep install`, then perform a clean isolated ten-package build and
  the required eight-package test selection.
- [ ] Require 0 build failures, 0 test errors, and 0 test failures; record any
  intentional skips.
- [ ] Reconfirm the installed `Rosmaster_Lib` SHA256 is
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
  after the lifted tests.
- [ ] CPU use remains bounded with aligned 320x240 RGB-D and the colored cloud
  arrives consistently.
- [ ] No default `/cmd_vel` publisher appears.
- [ ] Camera absent: strict bringup exits after the startup deadline.
- [ ] LiDAR absent: strict bringup exits clearly.
- [ ] Motor feedback absent: driver/bringup exits after sustained failure.
- [ ] After each absence test, restore the device and pass the full contract
  before continuing.

Perform device-absence checks one at a time with power controlled safely. Never
disconnect a motor-controller interface while a command source exists.

## 5. Resolve the weak and uneven lifted-wheel response

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
