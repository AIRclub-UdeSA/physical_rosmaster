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

## Workstation remediation checkpoint

Runtime commit `a08b097` replaces the failed `55caf7a` motor path with
checksum-valid report freshness, observable and bounded post-construction
serial writes, terminal failure diagnostics, controller-topic suppression, and
strict-launch exit. It also adds an observer-only no-motion live-loss probe.
Gate-hardening commit `bc965a6` adds the deterministic recovery-driver smoke
test, exact-source pseudo-terminal coverage, and stricter standalone probe
tests. This is workstation-tested code, not robot acceptance evidence: an exact
reviewed head containing both `a08b097` and `bc965a6` has not yet been deployed
to or validated on `x3-c`.

The fresh workstation build of the eight local packages plus both pinned Astra
packages completed successfully. The local package test result is `125 tests,
0 errors, 0 failures, 3 skipped`; the documented
standalone source-tree command adds 80 passing tests. The separate exact-source
V3.3.9 pseudo-terminal gate adds 8 passing tests. That exact recovered/public
source is compatible with the wrapper and has SHA256
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
That digest is an installed-source compatibility/version gate evaluated after
import, not a supply-chain attestation.

The remediation lineage is `origin/platform/simulator-parity` at `7113f07`
plus `origin/main` at `1d4b94f`, merged as `32feb1b`, followed by runtime commit
`a08b097`, acceptance-sequence documentation commit `079b71a`, and workstation
gate-hardening commit `bc965a6`. The robot has not received this lineage;
deploy only a reviewed, published branch head containing both `a08b097` and
`bc965a6`, and record its exact full SHA.

The target configuration is still the canonical new-robot setup in the root
README and [setup guide](setup_guide_ros2_humble_autostart.md): one clean Humble
workspace, exact robot library, stable hardware identities, manual strict
bringup, and no default command publisher. The recovery steps in this runbook
add evidence preservation, host/storage health checks, and removal of competing
factory launch paths; they do not authorize restoring the old host wholesale or
creating a second deployment architecture.

## Required order

1. archive the latest live-loss evidence that still exists only in the robot
   container;
2. establish a safe, charged, persistent host configuration;
3. deploy and build one clean, exact reviewed full-SHA PR head containing both
   runtime `a08b097` and gate-hardening `bc965a6`;
4. pass the positive contract, motor startup absence, a restored positive
   contract, observer-only live no-command loss, and a second restored positive
   contract, in that order — **done on 2026-09-02, see Section 4**;
5. ~~prove bounded physical stop in a separate securely lifted active-loss
   gate~~ — tested on 2026-09-02: **did not** demonstrate bounded stop. Root
   cause and the owner's decision to accept this as a known limitation rather
   than a blocking gate are recorded in
   [the known-issue writeup](../docs/troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md).
   No longer blocks the items below;
6. resolve the uneven lifted-wheel response;
7. verify operator tools;
8. perform repeated measured floor trials;
9. prove simulator/physical consumer parity and finish the handoff.

Do not move to a later gate because an earlier failure appears unrelated,
except where noted above as an explicit owner-accepted exception.

## 1. Preserve the previous evidence

Before cleaning the robot workspace or `/tmp`:

- [ ] Copy the 2026-09-01 post-hub, motor-startup-absence, restoration, and
  live-motor-loss directories from
  `/root/rosmaster-recovery-evidence/` in the `rosmaster_humble` container to
  durable project storage. These latest records are still described by
  container-local paths and must be archived before a clean deployment.

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

For the remediation run, do not execute the fetch/pull sequence until the
reviewed branch containing both `a08b097` and `bc965a6` is published. Both
cleanliness checks and all four revision checks must succeed after replacing
the reviewed-SHA marker with the exact reviewed full SHA.

```bash
(
set -eo pipefail

cd /root/yahboomcar_ws/src/physical_rosmaster
test -z "$(git status --porcelain)"
git fetch origin
git checkout platform/simulator-parity
git pull --ff-only
test -z "$(git status --porcelain)"
git rev-parse HEAD
git rev-parse origin/platform/simulator-parity
test "$(git rev-parse HEAD)" = \
  "$(git rev-parse origin/platform/simulator-parity)"
git merge-base --is-ancestor \
  a08b097b22781ca500fd61c01164a4e7167b3873 HEAD
git merge-base --is-ancestor \
  bc965a6f5ccdafb01efd3a1a6a230e9d3bbd8e80 HEAD
test "$(git rev-parse HEAD)" = "REPLACE_WITH_REVIEWED_FULL_SHA"
)
```

Run the library compatibility preflight from that exact checkout. It is
fail-closed: require exit status 0, and stop the deployment on any other status.
The `--hash-only` path must not open the serial device or construct the vendor
controller:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only || exit 1
```

Before the clean build, move generated state to a recoverable archive outside
the colcon workspace. Do not use `rm`, and do not move or alter the historical
`/root/rosmaster-recovery-evidence` records preserved in Section 1:

```bash
(
set -eo pipefail

cd /root/yahboomcar_ws
archive_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
generated_archive="/root/rosmaster-workspace-archives/${archive_stamp}-pre-deploy"
test ! -e "$generated_archive"
mkdir -p /root/rosmaster-workspace-archives
mkdir "$generated_archive"
for generated_dir in build install log; do
  if test -e "$generated_dir"; then
    mv "$generated_dir" "$generated_archive"/
  fi
done
test ! -e build
test ! -e install
test ! -e log

cd /root/yahboomcar_ws/src
vcs import . < physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C ros2_astra_camera status --porcelain)"
test "$(git -C ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list --base-paths src
colcon build --symlink-install
colcon test --packages-select \
  laserscan_to_point_pulisher sllidar_ros2 yahboomcar_astra \
  yahboomcar_base_node yahboomcar_bringup yahboomcar_ctrl \
  yahboomcar_description yahboomcar_visual
colcon test-result --verbose
source install/setup.bash
)
```

Require `colcon list --base-paths src` to report exactly the eight repository
packages plus the two pinned Astra packages before accepting the clean build.
Keep the generated-state archive until the new deployment record and its logs
have been copied to durable project storage.

- [x] `git status --porcelain` is empty.
- [x] The robot `HEAD` equals its cached `origin/platform/simulator-parity` at
  `55caf7a2a572aae0ad2682e265147c46e525921c`, with an empty worktree. The
  workstation and server branch at that historical checkpoint resolved to
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

Those checks describe the historical `55caf7a` run. For the remediation run:

- [ ] Archive the container-local evidence listed in Section 1 before cleaning
  generated state.
- [ ] Fetch only after the reviewed branch containing both `a08b097` and
  `bc965a6` is published; record the exact reviewed full SHA, require it as both
  local and remote head, and require an empty worktree.
- [ ] Run `python3 tools/rosmaster_lib_probe.py --hash-only` from that checkout
  and require exit status 0 before constructing the runtime driver or
  continuing the deployment.
- [ ] Move the existing `build`, `install`, and `log` directories to the
  recoverable, timestamped archive outside `/root/yahboomcar_ws`; do not delete
  them or alter the preserved historical `55caf7a` evidence.
- [ ] Import the pinned Astra revision from clean generated state; require its
  worktree to be clean and its `HEAD` to equal
  `f7e71d9ce806e788cb48d8580aac2c778fba4214`; then confirm exactly ten
  workspace packages, build all ten, and run the required eight local-package
  test selection.
- [ ] Require zero build failures, zero test errors, and zero test failures;
  retain the logs rather than borrowing the workstation result as robot
  evidence.

Do not edit code on the robot during acceptance. If a fix is required, stop the
gate, implement and commit it on the branch, then restart this section from a
clean checkout of the new PR head.

## 4. Repeat the strict sensor gate and fail-closed tests

With the wheels lifted and no command publisher, export the verified identities
and start only the strict platform bringup:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROSMASTER_MOTOR_PORT=/dev/robot/motor
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL=ACRC64300ET
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

From a second shell, run:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

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
- [ ] On the exact remediated head, repeat motor absence at startup, reconnect
  the controller, start a clean strict launch, and pass the full contract before
  arming the live-loss probe.
- [ ] Live motor-controller loss fails closed. Runtime head `55caf7a` fails this
  gate: after the physical controller disappeared, the Rosmaster receive thread
  raised `SerialException`, but the driver stayed alive, diagnostics remained
  level `OK`/healthy with zero failure counters, and cached `/joint_states`,
  `/voltage`, and `/vel_raw` continued to look fresh. No command publisher was
  present, but stale healthy feedback is not acceptable. Runtime commit
  `a08b097` is the motor remediation, and `bc965a6` hardens its workstation
  recovery gates; the exact reviewed full-SHA head containing both remains
  unvalidated on the robot. With the wheels secured and the full strict graph
  healthy, run:

  ```bash
  export ROSMASTER_MOTOR_PORT=/dev/robot/motor
  mkdir -p /root/rosmaster-recovery-evidence
  python3 tools/motor_live_loss_probe.py \
    --device "$ROSMASTER_MOTOR_PORT" \
    --confirm-wheels-secured \
    --output /root/rosmaster-recovery-evidence/motor-live-loss.json
  ```

  Wait for `ARMED`, disconnect only `/dev/robot/motor`, and do not reconnect it
  until the probe finishes. A pass requires a stable full-platform baseline,
  no `/cmd_vel` publisher or message, all controller-derived topics quiet by
  the deadline, a structured feedback-loss `ERROR`, driver exit, and stable
  teardown of every strict-platform publisher. This observer sends no command
  and does not prove physical stop from an active command.
- [ ] After each absence test, restore the device and pass the full contract
  before continuing. Camera, LiDAR, and startup-motor restoration passed on the
  historical runtime. The remediated head still requires one full restoration
  after repeated motor-startup absence and a second after live loss, so this
  aggregate item stays unchecked.

Perform device-absence checks one at a time with power controlled safely. The
camera-absence and LiDAR-absence tests need not be repeated for the motor-only
runtime change in `a08b097` unless deployment, launch, or positive-contract
evidence shows a regression. The deployed candidate must still be the exact
reviewed full SHA containing both `a08b097` and `bc965a6`. Motor startup absence,
live no-command loss, and restoration are mandatory.

Stop here until the exact deployed reviewed full SHA containing both `a08b097`
and `bc965a6` passes clean robot build/test, the positive contract, startup motor
absence, a restored positive contract, observer-only live loss, and a second
restored positive contract. Do not treat the no-motion probe as permission for
general motion.

### 2026-09-02 result: full non-motion sequence passed

Deployed and validated exact head `e34f8a35a75fb824add197d18fa330d3934eb89b` on
`x3-c`. `git status --porcelain` empty, `HEAD` matched
`origin/platform/simulator-parity` exactly, both `a08b097` and `bc965a6`
confirmed ancestors. `rosmaster_lib_probe.py --hash-only` exited 0 with the
matching V3.3.9 digest. Clean build of all ten workspace packages (10
finished, 2min31s). `colcon test` on the eight local packages: **125 tests, 0
errors, 0 failures, 3 skipped** — matches the workstation baseline exactly.

All five required non-motion steps then passed in order:

1. Positive contract (pre-motion): passed, all required topics healthy,
   `/tf_static=2`, zero `/cmd_vel` publishers.
2. Motor absence at startup: driver raised `SerialException` on the missing
   device, exited code 1; `ros2 launch`'s own `OnProcessExit` handling
   correctly cascaded shutdown to every other node with no manual
   intervention, and the whole strict graph drained on its own.
3. Restored positive contract: passed.
4. Observer-only live no-command loss (`motor_live_loss_probe.py`): **PASS**.
   Baseline healthy; on disconnect, the receive thread raised
   `SerialException` within ~40ms; three zero-command write attempts were made
   and all three failed (`Errno 5: Input/output error`, correctly recorded as
   "delivery is not proven"); every controller-derived topic had zero messages
   after loss; driver exited and the full strict graph drained in about 1.0s
   with no manual intervention. Full JSON archived on the robot at
   `/root/rosmaster-recovery-evidence/motor-live-loss-2026-09-02.json`.
5. Second restored positive contract: passed.

Container-local evidence from the prior (2026-09-01, runtime `55caf7a`)
session — the `pr3-2026-09-01-55caf7a-{post-hub-positive,fail-motor,
fail-motor-live,restore-after-motor}` directories — was archived and hashed to
durable workstation storage at `/home/juan/rosmaster-evidence/2026-09-01/`
before this deployment, per the required order above. That evidence
independently confirms the `55caf7a` live-loss failure this runtime fixes:
diagnostics stayed `healthy` and `/joint_states` kept publishing at 10 Hz after
the controller was disconnected.

One operational finding unrelated to the safety result: `ros2 launch` did not
respond to `SIGINT` sent externally to a healthy, already-running graph (twice,
~30s), and `SIGTERM` to the launch parent killed only the parent, orphaning all
six child nodes, which had to be terminated individually. This is distinct from
the required-node-exit cascade above, which worked correctly both times it was
exercised. Relevant for autostart design later (a systemd unit's `KillMode`
needs to target the whole cgroup, not just the main PID).

## 5. Prove bounded active-motion stop — tested 2026-09-02, did not pass

Secured the robot fully lifted, with a human at the main power switch and a
predeclared 2-second stop bound. Sub-test 1 (command-timeout watchdog, link
healthy: a 1-second `timeout`-wrapped `+0.05 m/s` stream with no final zero)
passed cleanly — `/joint_states` returned to zero and diagnostics stayed
healthy with zero write failures, matching the driver's `cmd_vel_timeout`
design.

Sub-test 2 (active link loss during motion) did **not** demonstrate bounded
stop. An uninterrupted `+0.05 m/s` stream was started; once the operator
visually confirmed wheel motion (onset took roughly 2-3 seconds at this speed
and voltage — see Section 6), the motor controller was physically
disconnected. `/joint_states` velocity was logged throughout: all four wheels
continued rotating in a sustained, rhythmic oscillation between roughly `1.3`
and `3.7 rad/s` with no decay for the entire 28-second recording window, well
past the 2-second bound. The operator cut main power at the switch per the
predeclared procedure; robot and operator were unharmed, no damage.

Root cause: the exact hash-verified installed `Rosmaster_Lib.py` was pulled
from the robot and its complete protocol was audited (all 30 `FUNC_*` command
codes, every public method). It has no command-timeout, watchdog, heartbeat,
or auto-stop capability at all — not an unused feature, an absent one. A
Pi-side software fix is therefore not feasible for this specific failure mode:
the driver's own `cmd_vel_timeout` watchdog only works by sending an explicit
zero that the still-connected controller obeys, which by definition cannot
happen once the link itself is gone.

The project owner reviewed this evidence on 2026-09-02 and decided to accept
the residual risk rather than block on a hardware fix with no committed
timeline; see
[the known-issue writeup](../docs/troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md)
for the full record. **This gate no longer blocks Sections 6-9 below.** Main
power remains the only proven stop mechanism for a command-link loss during
motion, lifted or floor. Do not combine any future repeat of this trial with
`motor_live_loss_probe.py`, whose pre-arm contract correctly rejects every
command publisher and message.

## 6. Resolve the weak and uneven lifted-wheel response

The exact reviewed full-SHA head containing both `a08b097` and `bc965a6`
passed the no-motion sequence and restoration contract on the robot on
2026-09-02; see Section 4. The active-motion physical-stop gate above did not
pass and is tracked as an accepted, documented exception rather than a
blocker — see
[the known-issue writeup](../docs/troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md).

During the post-hub strict no-command bringup, two contract probes, and sensor
soak, the operator observed no wheel movement. This establishes quiet startup
only; it does not replace the lifted command-response tests below.

The previous lifted run established the expected signs but did not establish
usable motion quality. At `10.2` to `10.3` V, wheel response was strongly uneven
and a one-second `+0.30 rad/s` yaw command produced only `+0.0124 rad` of odometry
yaw. Low voltage is a test condition, not a proven root cause.

A second, independently instrumented data point from 2026-09-02 (battery
`10.6` V) is consistent with that finding and adds detail: logged
`/joint_states` velocity through a 3-second `+0.05 m/s` pulse showed near-zero
position for roughly the first 2 seconds (a visible delay the operator also
observed directly, confirming it is real and not a logging artifact), then a
sudden release into substantial but wildly uneven rotation — front-right moved
`3.67 rad` (~210°) while back-left moved only `0.85 rad` (~48°) over the same
window, with peak per-wheel velocities of `2.5-3.3 rad/s`, faster than the
`~1.5 rad/s` a `0.05 m/s` command should nominally produce. The operator
independently confirmed some wheels visibly started turning before others.
This delayed-then-uneven-then-overshooting shape looks like stiction
(static friction) that releases unevenly per wheel, worsened by low voltage,
rather than a steady proportional response — still to be diagnosed against
the checks below, not assumed.

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

## 7. Complete operator-tool safety

Joystick, keyboard, calibration, and routine watchdog testing remain blocked
until the remediated driver passes the Section 4 no-motion and restoration gates
and the Section 5 active physical-stop gate.

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

## 8. Perform measured floor acceptance

Floor testing is blocked by the pending robot validation of the exact reviewed
full-SHA head containing both `a08b097` and `bc965a6`, bounded active
physical-stop proof, and all remaining lifted-motion and operator-tool gates.

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

## 9. Prove parity and finish PR #3

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

Evidence archival, clean deployment, and the full non-motion sequence are done
as of 2026-09-02 (Section 4). The active-motion physical-stop gate was tested
and is now a documented, owner-accepted exception rather than a blocker
(Section 5). The immediate next action is diagnosing the uneven lifted-wheel
response (Section 6), then operator-tool safety, floor trials, and consumer
parity, in that order; it is still not autostart.
