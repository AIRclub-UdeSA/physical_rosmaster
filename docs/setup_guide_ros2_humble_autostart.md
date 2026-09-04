# ROS 2 Humble X3 setup and autostart gate

Despite the historical filename, this document does not install or enable
autostart. The old routine targeted the former EKF/application tree and
unstable `/dev/ttyUSB*`/`/dev/video*` names. It must not be reused.

This guide prepares one robot for manual validation. Autostart remains blocked
until the final gate passes.

## 1. Preconditions

- Yahboom ROS 2 Humble container is operational.
- The robot is an ROSMASTER X3; X1 and R2 are unsupported.
- The host exposes motor controller, A1 LiDAR, and Astra USB devices to the
  container.
- Existing source and generated-state backups are outside
  `/root/yahboomcar_ws`.
- Host autostart is disabled while validation is in progress.
- The `vcs` command is available in the container (`python3-vcstool` on
  Ubuntu).

If an old service is active, stop it using the robot's existing administration
procedure before opening serial devices manually.

Inspect storage before cloning, building, or changing Docker state:

```bash
df -h /
docker system df
docker ps -a --size
```

Do not use aggressive pruning as the first response to low disk space. An
active container or image can consume most of the disk while Docker reports no
reclaimable space, and `docker system prune -a` does not shrink an active
container's writable layer. Diagnose the actual consumer first; see
[Root filesystem full causing LightDM login loop and Docker growth](troubleshooting/known_issues/root-filesystem-full-login-loop.md).
Preserve bags, calibration, and acceptance evidence, and never delete
`/var/lib/docker` manually.

## 2. Source and dependencies

Inside the container:

```bash
(
set -eo pipefail

mkdir -p /root/yahboomcar_ws/src
cd /root/yahboomcar_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd physical_rosmaster
git fetch origin
git checkout main
git pull --ff-only
test -z "$(git status --porcelain)"
test "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)"
git merge-base --is-ancestor \
  a08b097b22781ca500fd61c01164a4e7167b3873 HEAD
git merge-base --is-ancestor \
  bc965a6f5ccdafb01efd3a1a6a230e9d3bbd8e80 HEAD

cd ..
vcs import . < physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C ros2_astra_camera status --porcelain)"
test "$(git -C ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
)
```

The manifest pins `ros2_astra_camera`. Do not build an arbitrary current
camera-driver branch.

The default clone above is the canonical new-robot path: this architecture
merged to `main` on 2026-09-03, and `main` contains both runtime commit
`a08b097` and workstation-validation commit `bc965a6`. The
`platform/simulator-parity`-branch, SHA-pinned procedure in
[the ordered recovery runbook](robot_side_next_moves.md) was how `x3-c` got
through that merge; it is a historical record, not a path to repeat for
another robot.

Verify the robot-provided motor library with the fail-closed preflight:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

The command must exit `0` and report the supported V3.3.9 digest. If it returns
nonzero or cannot import the library, stop and restore the Yahboom
host/container integration before building. Do not vendor an unknown
`Rosmaster_Lib` copy into this repository or accept a mismatch by visually
comparing printed output.

The only supported `Rosmaster_Lib.py` SHA256 is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
Runtime commit `a08b097` checks this digest after importing the library and
refuses to construct the controller transport when it differs. This is a
compatibility/version gate for the exact implementation whose private receive
and parse hooks were reviewed. It is not supply-chain attestation: matching
bytes do not establish who delivered the file, whether its installation path
is trusted, or whether the surrounding system is uncompromised. Commit
`bc965a6` makes the standalone preflight return nonzero for an unsupported
source.

After the vendor constructor returns, `a08b097` applies and verifies a `0.05 s`
pyserial `write_timeout`, then wraps runtime writes so exceptions and short
writes become terminal driver failures. The V3.3.9 constructor's UART-servo
torque-enable write occurs before that wrapper is installed. A completed
runtime host write also proves only that the serial layer accepted the frame;
it is not an acknowledgement that the motor controller executed it or that the
robot physically stopped.

## 3. Build from clean generated state

```bash
(
set -eo pipefail

cd /root/yahboomcar_ws

# Moving these trees outside the workspace is recoverable and prevents stale
# generated state reuse.
archive_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
generated_archive="/root/rosmaster-workspace-archives/${archive_stamp}-pre-deploy"
test ! -e "$generated_archive"
mkdir -p /root/rosmaster-workspace-archives
mkdir "$generated_archive"
for generated_tree in build install log; do
  if [ -e "$generated_tree" ]; then
    mv -- "$generated_tree" "$generated_archive/"
  fi
done
test ! -e build
test ! -e install
test ! -e log

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

Keep the archived generated state until the clean build and test evidence has
been reviewed; remove it later only as a separate, explicit maintenance action.

The workspace inventory must show exactly eight local packages plus
`astra_camera` and `astra_camera_msgs`. The external camera-driver worktree must
be clean and remain at the commit pinned in `physical_rosmaster.repos`.

## 4. Identify required hardware

On the host:

```bash
lsusb
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 run astra_camera list_devices_node
```

Record exact model, vendor/product IDs, and stable identity for:

- motor controller;
- Slamtec A1 serial adapter;
- Astra-family RGB-D camera.

For `x3-c`, the accepted camera topology is the powered Yahboom hub,
downstream port 4. In that topology, one cold boot followed by three consecutive
warm boots enumerated both Astra USB functions, `2bc5:060f` for depth and
`2bc5:050f` for UVC/RGB, without touching or reconnecting any cable. Preserve
that topology for `x3-c`. This is evidence for that robot, hub, power path, and
cable; it is not a universal instruction that every X3 must use port 4. Record
and boot-validate the stable topology of each additional robot independently.

If no Orbbec/Astra device appears, strict simulator parity fails. Stop here; do
not prepare autostart.

Use [../config/99-rosmaster-x3.rules.example](../config/99-rosmaster-x3.rules.example)
as a template. Replace placeholders with observed values, install it on the
host, reload udev rules, reconnect the devices, and verify the final aliases. A
literal placeholder rule is intentionally nonfunctional.

Prefer a unique device serial. Some CH340 motor controllers expose no serial;
for those units, bind `/dev/robot/motor` to a dedicated physical USB port using
the documented `KERNELS` fallback and verify that identity after reboot.

## 5. Configure identities

In the container shell used for manual launch:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL="REPLACE_WITH_ASTRA_SERIAL"
```

The camera serial may be temporarily omitted only during discovery with exactly
one device attached. It must be recorded and selected before acceptance.

## 6. Manual non-motion gate

Lift the drive wheels or otherwise prevent unintended movement. Start only
default bringup:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Do not start any command publisher. In another shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

Acceptance requires:

- exactly one publisher for every sensor topic, `/joint_states`, and `/odom`;
- no `/cmd_vel` publisher;
- increasing timestamps and finite data;
- `rgb8` color and calibrated intrinsics;
- `32FC1` metric depth with plausible samples;
- XYZRGB cloud in `cam_1_depth_frame`;
- scan in `laser_link` and IMU in `imu_link`;
- canonical wheel joints;
- observed `odom -> base_footprint` authority from wheel odometry;
- healthy controller/encoder status on `/diagnostics`;
- `odom` to every sensor frame resolvable at message time.

Required driver exit or camera startup failure must stop bringup. Fix
hardware/dependencies rather than making sensors optional.

### Required devices absent at startup

On every new X3, test the camera, LiDAR, and motor controller separately. Keep
the wheels secured and start strict bringup with exactly one required device
absent. Camera and LiDAR failure must stop strict bringup; with the motor absent,
the driver must never become healthy, must exit after the startup-feedback
deadline, and the strict graph must drain. After each test, restore the device,
restart from a clean launch, and pass the full physical contract before
continuing.

For the current motor-only `x3-c` remediation, the previously accepted camera
and LiDAR startup-absence results may be carried forward unless deployment or
positive-contract evidence regresses. Motor startup absence and its restored
full contract must be repeated before the live-loss test.

### Observer-only live motor-controller loss

The exact reviewed head containing `a08b097` and `bc965a6` is still pending
this robot-side validation. Keep the wheels secured and do not start joystick,
keyboard, calibration, or any other command source. After strict bringup passes
its full baseline, run from the repository root:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
mkdir -p /root/rosmaster-recovery-evidence
python3 tools/motor_live_loss_probe.py \
  --device "$ROSMASTER_MOTOR_PORT" \
  --confirm-wheels-secured \
  --output /root/rosmaster-recovery-evidence/motor-live-loss.json
```

Acceptance requires all of the following in the probe result:

- a complete, fresh, stationary baseline for every controller-derived topic;
- no `/cmd_vel` publisher and no `/cmd_vel` message before or after loss;
- controller-derived topics become quiet after the motor device disappears;
- motor diagnostics reach `ERROR` with structured failed freshness evidence;
- `driver_node` exits;
- the strict ROS graph drains and remains stably drained for the probe's
  observation window; and
- after restoring the controller, a clean strict launch passes the full
  physical contract again.

This observer-only, no-command test proves ROS data, diagnostic, process-exit,
and graph-drain behavior. It does **not** prove that a robot already moving will
physically stop when feedback or the serial link is lost.

## 7. Lifted and floor motion gates

Follow [odometry_validation.md](odometry_validation.md). Use only the bounded
pulse, joystick, or keyboard tool under direct supervision, one publisher at a
time.

Verify:

- forward, left-strafe, and CCW encoder signs;
- `/odom` direction and TF;
- command watchdog stop;
- joystick held deadman, release stop, malformed-input protection, and timeout;
- keyboard timeout;
- calibration remains inert unless explicitly activated.

After the observer-only live-loss gate passes, perform a separate active safety
gate with all four wheels securely lifted, the main power switch immediately
reachable, an observer present, and only one very-low-speed command source.
First prove the software command watchdog produces a bounded physical stop when
the command stream ends while the link remains healthy. Then, under the same
restrained conditions, test active controller-link loss against a
predeclared stop-time bound and record command, wheel motion, diagnostics, and
power intervention. Cut the main switch immediately if behavior is unexpected
or the bound is exceeded.

Host write completion is not controller execution acknowledgement. Tested on
`x3-c` on 2026-09-02: the lifted active-link-loss test did not demonstrate a
bounded physical stop (wheels kept turning, actively driven, for at least 28
seconds until main power was cut). This was root-caused to a protocol-level
absence of any command-loss watchdog in the motor controller — see
[the known-issue writeup](troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md).
The project owner accepted this as a known platform limitation rather than a
blocking requirement, so it no longer prevents floor use. Main power remains
the only proven stop mechanism for a command-link loss during motion. Do not
treat repeated host zero writes as a substitute.

Then repeat conservative floor trials with external observations. Do not tune
geometry or covariance from visual impression alone.

## 8. Simulator/physical acceptance

Run the same minimal consumer or contract-facing project against:

1. simulator commit `772ba25`;
2. this physical platform.

No topic remaps or frame-name substitutions are allowed. Simulation-only clock
and ground truth are excluded.

## 9. Autostart decision

Autostart work may begin only when one X3 has passed:

- full non-motion probe;
- camera-, LiDAR-, and motor-absent startup gates with a restored full contract
  after each (subject only to the documented current `x3-c` exception);
- observer-only motor live-loss and its restored full contract;
- lifted motion and safety gates (the active-link-loss physical-stop trial is
  a documented, owner-accepted exception rather than a required pass — see
  [the known-issue writeup](troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md));
- bounded floor gates;
- simulator/physical consumer acceptance;
- stable motor, LiDAR, and camera identity configuration.

The future routine should be versioned in this repository, invoke the single
strict platform launch, load per-robot identity configuration, propagate
failures to the host service, and never start joystick, keyboard, calibration,
or project behavior by default. It should also:

- refuse startup when the root filesystem is at least 95% used or has less than
  2 GiB free;
- cap persistent journald storage and Docker container logs;
- keep cleanup in a separate maintenance timer rather than ROS bringup;
- preserve bags, calibration, and acceptance evidence; and
- keep bringup in the foreground so failures propagate to the service manager.

Test a future disk guard by temporarily raising its threshold, never by filling
the robot's storage.

The exact reviewed head containing `a08b097` and `bc965a6` remains pending these
robot-side gates. Until they all pass, leave the fleet autostart disabled for
this new stack.
