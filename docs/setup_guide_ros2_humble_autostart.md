# ROS 2 Humble X3 setup

Despite the historical filename, this document does not install or enable
autostart — see [autostart_setup.md](autostart_setup.md) for that, once this
guide's checks pass. The old routine targeted the former EKF/application tree
and unstable `/dev/ttyUSB*`/`/dev/video*` names. It must not be reused.

This is the fast path for bringing up an *additional* X3 running the
already-accepted `main` platform code. It covers only what's genuinely
per-robot: cloning and building, identifying this unit's hardware, and one
smoke check that it's wired correctly. It deliberately does **not** repeat the
one-time platform validation that `x3-c` already went through — the motor
live-loss protocol proof, the active-link-loss characteristic, and the
simulator/physical consumer-parity proof all test the *code*, not the unit,
and don't need re-running per robot. See
[robot_side_verification_todo.md](robot_side_verification_todo.md) and
[robot_side_next_moves.md](robot_side_next_moves.md) if you want that history;
neither is required reading to bring up a new robot.

## 1. Preconditions

- Yahboom ROS 2 Humble container is operational.
- The robot is a ROSMASTER X3; X1 and R2 are unsupported.
- The host exposes motor controller, A1 LiDAR, and Astra USB devices to the
  container.
- Existing source and generated-state backups are outside
  `/root/yahboomcar_ws`.
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

## 2. Clone and build

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

cd ..
vcs import . < physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C ros2_astra_camera status --porcelain)"
test "$(git -C ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
)
```

The manifest pins `ros2_astra_camera`; don't build an arbitrary current
camera-driver branch. The workspace inventory should show exactly eight local
packages plus `astra_camera` and `astra_camera_msgs`.

Verify the robot-provided motor library with the fail-closed preflight:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

The command must exit `0` and report the supported V3.3.9 digest. If it
returns nonzero or can't import the library, stop and restore the Yahboom
host/container integration before continuing — don't vendor an unknown
`Rosmaster_Lib` copy into this repository, and don't accept a mismatch by
eyeballing printed output. The only supported SHA256 is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`; the driver
checks this itself at runtime too, so this is an early, cheap version of the
same check.

## 3. Identify this robot's hardware

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

Record exact model, vendor/product IDs, and stable identity for the motor
controller, the Slamtec A1 serial adapter, and the Astra-family RGB-D camera.
This is unavoidably per-robot: USB topology, cable routing, and which hub port
things land on vary unit to unit. If no Orbbec/Astra device appears, stop here
and fix the physical connection before continuing.

Use [../config/99-rosmaster-x3.rules.example](../config/99-rosmaster-x3.rules.example)
as a template. Replace placeholders with this robot's observed values, install
it on the host, reload udev rules, reconnect the devices, and verify the final
aliases. Some CH340 motor controllers expose no serial; for those, bind
`/dev/robot/motor` to a dedicated physical USB port using the documented
`KERNELS` fallback and verify that identity survives a reboot.

## 4. Configure identities

In the container shell used for launch:

```bash
export ROSMASTER_MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_MOTOR_CONTROLLER_ID"
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL="REPLACE_WITH_ASTRA_SERIAL"
```

## 5. Smoke-check the launch

This is the one check worth running before calling a new robot done: confirm
the platform actually comes up healthy on *this* unit's wiring. It's not a
re-proof of the driver — that's already established — just a fast sanity
check that nothing's plugged in wrong. Lift the drive wheels, then:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

In another shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

If it doesn't pass on the first try, the probe's output says exactly what's
missing or wrong — usually a device path, a loose connector, or a camera
serial that doesn't match what's plugged in, not a code problem.

Optionally, with wheels still lifted, use the bounded pulse tool
(`tools/safe_cmd_vel_pulse.py`) to confirm forward/strafe/rotation move the
wheels in the expected direction and `/odom` agrees — this checks *this
unit's* encoder and motor wiring, not the driver, and takes under a minute.
See [odometry_validation.md](odometry_validation.md) if anything looks
backwards.

## 6. Install autostart

Autostart is already validated for this stack — follow
[autostart_setup.md](autostart_setup.md) directly, no separate decision gate
needed.
