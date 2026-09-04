# Physical ROSMASTER X3 Platform

This repository is the robot-side hardware platform for AIRclub UdeSA's Yahboom ROSMASTER X3 fleet. It deliberately provides no autonomous behavior, localization, mapping, navigation, perception application, or EKF. A project repository should run on top of it and own those choices.

The default stack provides:

- motor-controller access and a watchdog-protected, feedback-gated `/cmd_vel`
  input;
- four-wheel encoder state and mecanum wheel odometry;
- robot description and TF;
- raw and Madgwick-filtered IMU data;
- angle-compensated, cable-masked A1 LiDAR data;
- calibrated Astra RGB-D data normalized to the simulator contract;
- standard `/diagnostics` health for controller telemetry and wheel odometry;
- voltage, firmware, magnetometer, buzzer, and RGB hardware extensions.

Default bringup never publishes `/cmd_vel`. Joystick, keyboard, pulse tests, and calibration are separate, explicit operator actions.

## Public contract

The machine-readable contract is
[config/robot_contract.yaml](config/robot_contract.yaml). Its simulator-facing
interfaces match simulator commit `772ba25`; `/clock` and ground truth are
excluded, and the hardware-only extensions below are additional.

| Interface | Physical implementation |
|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` input; no default publisher |
| `/joint_states` | Position and velocity for four simulator-named wheel joints |
| `/odom` | Encoder-only mecanum odometry, `odom` → `base_footprint` |
| `/tf` | Wheel odometry owns `odom` → `base_footprint` |
| `/imu/data` | Madgwick-filtered IMU; `use_mag=false`; no IMU TF |
| `/scan` | A1 scan in `laser_link`, after physical cable/self-return masking |
| `/cam_1/color/*` | Calibrated RGB8 color image and camera info |
| `/cam_1/depth/*` | Metric 32FC1 depth, camera info, and XYZRGB cloud |

The cloud is transformed into x-forward `cam_1_depth_frame`; it is not merely relabeled. Hardware-only topics such as `/diagnostics`, `/imu/data_raw`, `/imu/mag`, `/vel_raw`, `/voltage`, `/edition`, `/Buzzer`, `/RGBLight`, and `/scan_filtered` remain available.

## Motor transport and failure boundary

Runtime commit `a08b097` derives motor-controller liveness only from
checksum-valid report arrivals parsed from the serial receive stream:

- `0x0A` supplies speed and battery feedback;
- `0x0D` supplies the four encoder counters;
- either `0x0B` or `0x0E` supplies raw IMU feedback.

All three channels must arrive and remain fresh. Reading a cached
`Rosmaster_Lib` getter never creates or refreshes liveness evidence.

A terminal feedback, receive-thread, or observed serial-write failure is
latched. The driver then suppresses controller-derived ROS output, nonzero
motion commands, and all RGB/buzzer requests; publishes a structured
motor-controller `ERROR` diagnostic; and exits. The strict platform launch
treats that driver exit as fatal and shuts down the rest of the graph.

This is fail-closed process and data behavior, not proof that the controller
executed a stop. A completed host serial write is not a controller
acknowledgement, and redundant zero-command attempts are not physical-stop
proof.

Confirmed on `x3-c` on 2026-09-02: the vendor motor-controller protocol has no
command-timeout/watchdog capability at all (full `FUNC_*` command-set audit
against the hash-verified installed library), and an active command-link loss
during motion was observed to leave the controller driving the last commanded
velocity for at least 28 seconds with no decay, stopped only by cutting main
power. The project owner has accepted this as a known platform limitation
rather than a blocking gate; see
[the known-issue writeup](docs/troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md)
for the full evidence and decision record. Until an independent hardware
watchdog is added, main power is the only proven stop mechanism for this
failure mode, for any motion, lifted or floor.

The allowlisted public V3.3.9 `Rosmaster_Lib` source hash is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
It is a compatibility/version gate for the exact private hooks used by this
runtime. Python imports the module before the source is hashed, so the check is
not supply-chain attestation.

## Retained packages

`colcon list --base-paths .` discovers exactly these eight local packages:

- `yahboomcar_bringup`: strict X3 driver and complete platform launch;
- `yahboomcar_base_node`: encoder-only mecanum `/odom` and TF;
- `yahboomcar_description`: canonical X3 description;
- `yahboomcar_ctrl`: opt-in joystick and keyboard operator tools;
- `yahboomcar_astra`: Astra normalization and strict sensor watchdog;
- `sllidar_ros2`: A1 driver and platform scan preprocessing;
- `yahboomcar_visual`: generic scan/image inspection conversions;
- `laserscan_to_point_pulisher`: generic `LaserScan` → `PointCloud2` utility. The historical package spelling is retained.

Removed behavior packages are recoverable at tag `pre-platform-contract-cleanup` or through Git history.

Default bringup composes the required platform packages into one strict graph
for manual validation. Operator-control and generic inspection/conversion
utilities remain separate and opt in. There is no accepted autostart routine
for this architecture yet.

## Workstation setup

The target runtime remains ROS 2 Humble inside the robot container. A
workstation can review, test, and build with a compatible ROS 2 environment.
The `vcs` command must already be available; Ubuntu provides it in
`python3-vcstool`.

```bash
(
set -eo pipefail

mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd ~/rosmaster_physical_ws
vcs import src < src/physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C src/ros2_astra_camera status --porcelain)"
test "$(git -C src/ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list --base-paths src
colcon build --symlink-install
source install/setup.bash
)
```

The `.repos` file pins Orbbec's `ros2_astra_camera` to
`f7e71d9ce806e788cb48d8580aac2c778fba4214`. The complete workspace inventory
must contain exactly the eight repository packages plus `astra_camera` and
`astra_camera_msgs`. Do not replace the pin without repeating camera contract
validation.

The clone command above is the canonical setup from `main`, which merged the
simulator-parity work (including runtime commit `a08b097` and
workstation-validation commit `bc965a6`) on 2026-09-03. It supersedes the
`platform/simulator-parity`-branch, SHA-pinned recovery procedure that
[the ordered recovery runbook](docs/robot_side_next_moves.md) used to get
`x3-c` through that merge; that runbook is now a historical record, not a
setup path to repeat for another robot.

Focused hardware-free checks:

```bash
(
set -eo pipefail

cd ~/rosmaster_physical_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/physical_rosmaster-ros-logs
mkdir -p "$ROS_LOG_DIR"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select \
  laserscan_to_point_pulisher sllidar_ros2 yahboomcar_astra \
  yahboomcar_base_node yahboomcar_bringup yahboomcar_ctrl \
  yahboomcar_description yahboomcar_visual
colcon test-result --verbose

cd src/physical_rosmaster
export PYTHONPATH="$PWD/yahboomcar_bringup:$PWD/tools:${PYTHONPATH:-}"
python3 -m pytest -q \
  tools/test_rosmaster_lib_probe.py \
  tools/test_motor_live_loss_probe.py \
  tools/test_motor_live_loss_ros_smoke.py \
  tools/test_physical_contract_probe.py \
  tools/test_safe_cmd_vel_pulse.py
)
```

`colcon test` already runs every test in the selected repository packages,
including the motor transport, driver, and strict-launch suites. The direct
pytest command therefore runs only the standalone source-tree tools. The
explicit plugin setting isolates these checks from an incompatible global
pytest plugin installed on the current workstation; it is not a robot runtime
setting. ROS tests also need a writable
`ROS_LOG_DIR`. The live-loss ROS smoke test must run on Linux with permission
to enumerate local network interfaces and open localhost DDS sockets.

When the exact reviewed vendor source is available on the workstation, exercise
it through real pyserial and a pseudo-terminal as a separate compatibility
gate. The file remains external because `Rosmaster_Lib` is not vendored here:

```bash
export ROSMASTER_V339_SOURCE=/absolute/path/to/Rosmaster_Lib.py
python3 -m pytest -q tools/test_rosmaster_v339_pty.py
```

## Robot setup and manual bringup

The expected clone path is `/root/yahboomcar_ws/src/physical_rosmaster` in the
`rosmaster_humble` container. `Rosmaster_Lib` remains a robot-provided
dependency and is not vendored here. Its source hash verifies the reviewed
runtime shape after import; it does not establish package provenance or trust.

For bringing up an X3 that isn't `x3-c` — clone/build, identify this unit's
hardware, and one smoke check that it's wired right — follow
[docs/setup_guide_ros2_humble_autostart.md](docs/setup_guide_ros2_humble_autostart.md).
It's deliberately fast: the platform code itself is already validated, so it
doesn't re-run the one-time protocol/safety proofs `x3-c` went through.

In short, once cloned and built:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL="REPLACE_WITH_ASTRA_SERIAL"

cd /root/yahboomcar_ws
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Then, in a second shell, confirm it came up healthy:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

Normal bringup shuts down when a required process exits — the motor driver on
missing/stale report channels, receiver failure, or a serial-write failure;
the camera adapter if RGB-D streams don't appear within its startup deadline.
The physical probe requires healthy controller and encoder `/diagnostics`.

## Operator tools

These are never part of default bringup.

```bash
# Joystick: held deadman, configurable mapping, timeout, release stop
ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py device_id:=0

# Keyboard: must run in an interactive terminal
ros2 run yahboomcar_ctrl yahboom_keyboard

# Bounded supervised pulse
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder

# Raw IMU and magnetometer inspection
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/mag
```

Manual control is capped at `0.20 m/s` linear and `1.0 rad/s` angular, with lower gears available. Calibration nodes start inert and require an explicit `start_test=true` parameter after their bounded settings are reviewed.

## Rollout status

Runtime commit `a08b097` and the robot-validation
tooling hardened by `bc965a6` passed full robot deployment and validation on
`x3-c` on 2026-09-02, at exact reviewed head `e34f8a3`.

The gate is:

1. deploy and record one clean branch head at an exact reviewed full SHA that
   contains both `a08b097` and `bc965a6`, with verified stable device
   identities — **done**, `x3-c` at `e34f8a35a75fb824add197d18fa330d3934eb89b`;
2. pass the positive non-motion contract, motor-absent-at-startup gate, restored
   positive contract, observer-only live motor-loss gate, and another restored
   positive contract, in that order — **done**, all five passed cleanly;
3. ~~prove bounded physical stop in a separate securely lifted,
   very-low-speed active-link-loss test, or validate a controller-side or
   independent hardware watchdog~~ — tested on `x3-c` on 2026-09-02 and did
   **not** demonstrate bounded stop; root-caused to a protocol-level absence of
   any command-loss watchdog. The project owner has accepted this as a known
   limitation rather than a blocking gate — see
   [the known-issue writeup](docs/troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md).
   Main power is the only proven stop mechanism for this failure mode;
4. pass the other lifted command, encoder, odometry, watchdog, and deadman
   checks, then repeat bounded forward, lateral, and rotation floor trials —
   a qualitative single-pass floor check (one trial per axis, visual
   confirmation) passed on 2026-09-02; the full 3-rep measured protocol,
   precise external measurement, and operator-tool (joystick/keyboard/
   calibration) checks remain open;
5. run one minimal consumer against simulator and hardware without remaps.

Item 4's full measured protocol, operator tools, and item 5 remain open.
Autostart itself is no longer gated on them, though: it has already been
designed, installed, enabled, and reboot-validated on `x3-c` — see
[docs/autostart_setup.md](docs/autostart_setup.md). Lifted-motion diagnosis
also found a repeatable ~2-4x front-right-vs-back-left actuation imbalance,
most pronounced below roughly `0.10 m/s`/`0.60 rad/s` per-wheel-equivalent;
the owner is tracking this as an accepted known platform characteristic (see
[docs/robot_side_next_moves.md](docs/robot_side_next_moves.md), Section 6)
rather than a blocker.

## Documentation

- [docs/setup_guide_ros2_humble_autostart.md](docs/setup_guide_ros2_humble_autostart.md): fast setup for an additional fleet robot;
- [docs/workstation_and_robot_workflow.md](docs/workstation_and_robot_workflow.md): workstation/robot responsibilities;
- [docs/robot_side_verification_todo.md](docs/robot_side_verification_todo.md): mandatory first-robot verification checklist and evidence record;
- [docs/robot_side_next_moves.md](docs/robot_side_next_moves.md): ordered `x3-c` remediation and acceptance runbook;
- [docs/odometry_validation.md](docs/odometry_validation.md): encoder-only odometry validation;
- [agents/README.md](agents/README.md): index of historical pre-cleanup audit and validation evidence;
- [docs/troubleshooting/README.md](docs/troubleshooting/README.md): incident history and known issues.

## Provenance and licensing

Two licenses apply in this tree, both declared in each package's own `package.xml` and matched by a [root `LICENSE`](LICENSE) file:

- **Apache-2.0** for the seven AIRclub-maintained packages — `yahboomcar_bringup`, `yahboomcar_base_node`, `yahboomcar_description`, `yahboomcar_ctrl`, `yahboomcar_astra`, `yahboomcar_visual`, `laserscan_to_point_pulisher`. All are Yahboom-derived in origin (the platform started from Yahboom's ROSMASTER X3 stack) but have been substantially rewritten under the safety and contract boundaries in [`context.md`](context.md); none still carry Yahboom's original placeholder metadata.
- **BSD** for the vendored `sllidar_ros2` (Slamtec's own driver, imported with its upstream license intact — see [`sllidar_ros2/LICENSE`](sllidar_ros2/LICENSE)). This package is carried close to upstream, not rewritten like the seven above.

### Retired Yahboom packages

The original Yahboom source tree shipped 22 packages; 8 are retained (above). The other 14 were dropped during the platform-contract cleanup and are recoverable only from Git history at tag `pre-platform-contract-cleanup`, never from a hidden local copy. Every one of them still carried Yahboom's unmodified `TODO: License declaration` / `TODO: Package description` placeholders at the point of removal — none were adapted by AIRclub, so removing them removes any AIRclub licensing exposure for that code. The decision for all fourteen is the same: **retired**, not maintained, not shipped, recoverable only for historical reference.

| Package | Why it was out of scope |
|---|---|
| `yahboomcar_nav`, `yahboomcar_slam` | Navigation/SLAM — excluded by the non-negotiable boundary in `context.md` |
| `robot_pose_publisher`, `robot_pose_publisher_ros2` | Pose estimation for a navigation/localization stack this platform doesn't provide |
| `yahboomcar_KCFTracker` | Visual object tracking — autonomous behavior, not a platform driver |
| `yahboomcar_linefollow`, `yahboomcar_point` | Autonomous line-following / point-to-point driving — publishes `/cmd_vel`, which default bringup must never do |
| `yahboomcar_voice_ctrl`, `yahboomcar_mediapipe` | Voice/gesture command interfaces — application behavior, not platform I/O |
| `yahboom_app_save_map`, `yahboom_web_savmap_interfaces` | Map-saving web UI integration — depends on the mapping stack this platform doesn't provide |
| `yahboomcar_laser` | Superseded here by `sllidar_ros2` + `laserscan_to_point_pulisher` |
| `yahboomcar_msgs` | Custom messages consumed only by the removed packages above |
| `yahboomcar_description_x1` | URDF for the X1 robot model — this repository targets X3 only |

This is a repository-wide provenance review, not just improved per-package metadata: every current and historical package's license, origin, and disposition is accounted for above.
