# Workstation and robot workflow

Use the workstation for source changes, package inventory, dependency review, unit tests, Xacro expansion, launch construction, and builds. Use a robot for USB identity, udev, `Rosmaster_Lib`, camera calibration, real message quality, motion, and final contract acceptance.

## Workstation

Create a normal ROS workspace and import the pinned camera driver:

```bash
mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd ~/rosmaster_physical_ws
vcs import src < src/physical_rosmaster/physical_rosmaster.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install
```

Expected local package inventory:

```text
laserscan_to_point_pulisher
sllidar_ros2
yahboomcar_astra
yahboomcar_base_node
yahboomcar_bringup
yahboomcar_ctrl
yahboomcar_description
yahboomcar_visual
```

Useful focused checks:

```bash
colcon test --packages-select yahboomcar_base_node
colcon test-result --verbose
python3 -m compileall -q src/physical_rosmaster
```

A complete build requires dependencies declared by the pinned Orbbec driver, including `camera_info_manager`, `image_transport`, `image_geometry`, `cv_bridge`, OpenCV, and its USB/OpenNI dependencies. Let `rosdep` resolve them for the target ROS distribution.

Do not install `Rosmaster_Lib` merely to make workstation imports pass. It is a robot hardware dependency; the driver process is not expected to run on a workstation.

## Robot/container

Expected paths:

- host: Yahboom installation, Docker lifecycle, USB passthrough, systemd;
- container: `/root/yahboomcar_ws`;
- repository: `/root/yahboomcar_ws/src/physical_rosmaster`;
- external camera driver: `/root/yahboomcar_ws/src/ros2_astra_camera`.

Before changing an existing workspace, put source backups outside `/root/yahboomcar_ws`. A backup containing packages anywhere under the workspace causes duplicate package discovery.

Inside the container:

```bash
cd /root/yahboomcar_ws/src
vcs import . < physical_rosmaster/physical_rosmaster.repos

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`Rosmaster_Lib` must be importable by the same Python used by ROS 2:

```bash
python3 -c "from Rosmaster_Lib import Rosmaster; print(Rosmaster)"
```

## Stable devices

Discover identities while no competing process owns the devices:

```bash
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
lsusb
ros2 run astra_camera list_devices_node
```

Copy and edit [../config/99-rosmaster-x3.rules.example](../config/99-rosmaster-x3.rules.example) on the robot host. Never deploy placeholder serial values. Reload rules, reconnect hardware, and verify aliases before starting ROS.

Configure the container:

```bash
export ROSMASTER_MOTOR_PORT=/dev/serial/by-id/<motor-id>
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL=<camera-serial>
```

The launch arguments can override these environment variables for a single run.

## Manual validation order

Use [robot_side_verification_todo.md](robot_side_verification_todo.md) as the
authoritative, evidence-backed checklist for the first robot acceptance run.

Start the platform manually:

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Do not start joystick, keyboard, calibration, or an external project yet. In a second shell:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

If strict bringup exits, fix the missing required process or hardware. Do not weaken the launch to continue with a partial graph.

After the non-motion gate passes:

1. lift and secure the robot;
2. record `/cmd_vel`, `/joint_states`, `/odom`, `/tf`, voltage, and diagnostics;
3. use the bounded pulse tool for forward, left strafe, and CCW rotation;
4. verify all wheel signs and `odom -> base_footprint` direction;
5. verify driver watchdog stop and joystick deadman/release/timeout;
6. repeat conservative trials on a clear floor with external distance/heading observations.

## Operator processes

Run only one `/cmd_vel` source at a time.

```bash
ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py device_id:=0
ros2 run yahboomcar_ctrl yahboom_keyboard
```

The bounded pulse tool refuses to start if another `/cmd_vel` publisher is visible:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0 --require-recorder
```

Raw sensor inspection needs no behavior package:

```bash
ros2 topic echo /imu/data_raw
ros2 topic echo /imu/mag
ros2 topic echo /diagnostics
ros2 topic echo /scan --once
ros2 topic echo /cam_1/color/camera_info --once
ros2 topic echo /cam_1/depth/camera_info --once
```

Use `camera_calibration` only if the device-reported intrinsics fail the contract or a calibrated replacement is intentionally being produced. Preserve the device serial, resolution, calibration date, and generated YAML in the robot's deployment record; do not commit one robot's calibration as a fleet-wide default.

## Autostart boundary

The old autostart instructions targeted separate camera/LiDAR processes, unstable device names, and the former EKF graph. They are obsolete.

Do not point host systemd or `/root/auto_start.sh` at this branch until one X3 passes the complete contract, lifted motion, floor motion, and simulator/physical consumer acceptance. Preparing versioned autostart files is the next project only after that gate.
