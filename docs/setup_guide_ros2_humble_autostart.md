# ROS 2 Humble X3 setup and autostart gate

Despite the historical filename, this document does not install or enable autostart. The old routine targeted the former EKF/application tree and unstable `/dev/ttyUSB*`/`/dev/video*` names. It must not be reused.

This guide prepares one robot for manual validation. Autostart remains blocked until the final gate passes.

## 1. Preconditions

- Yahboom ROS 2 Humble container is operational.
- The robot is an ROSMASTER X3; X1 and R2 are unsupported.
- The host exposes motor controller, A1 LiDAR, and Astra USB devices to the container.
- Existing source and generated-state backups are outside `/root/yahboomcar_ws`.
- Host autostart is disabled while validation is in progress.

If an old service is active, stop it using the robot's existing administration procedure before opening serial devices manually.

## 2. Source and dependencies

Inside the container:

```bash
mkdir -p /root/yahboomcar_ws/src
cd /root/yahboomcar_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git
vcs import . < physical_rosmaster/physical_rosmaster.repos

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

The manifest pins `ros2_astra_camera`. Do not build an arbitrary current camera-driver branch.

Verify the robot-provided motor library:

```bash
python3 -c "from Rosmaster_Lib import Rosmaster; print(Rosmaster)"
```

If that import fails, restore the Yahboom host/container library integration before building. Do not vendor an unknown `Rosmaster_Lib` copy into this repository.

## 3. Build from clean generated state

```bash
cd /root/yahboomcar_ws
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install
source install/setup.bash
```

The physical repository must show eight local packages. The external camera repository adds `astra_camera` and `astra_camera_msgs` to the workspace.

## 4. Identify required hardware

On the host and, where appropriate, inside the container:

```bash
lsusb
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
ros2 run astra_camera list_devices_node
```

Record exact model, vendor/product IDs, and serial for:

- motor controller;
- Slamtec A1 serial adapter;
- Astra-family RGB-D camera.

If no Orbbec/Astra device appears, strict simulator parity fails. Stop here; do not prepare autostart.

Use [../config/99-rosmaster-x3.rules.example](../config/99-rosmaster-x3.rules.example) as a template. Replace placeholders with observed values, install it on the host, reload udev rules, reconnect the devices, and verify the final aliases. A literal placeholder rule is intentionally nonfunctional.

## 5. Configure identities

In the container shell used for manual launch:

```bash
export ROSMASTER_MOTOR_PORT=/dev/serial/by-id/<motor-controller-id>
export ROSMASTER_LIDAR_PORT=/dev/robot/lidar
export ROSMASTER_ASTRA_SERIAL=<astra-serial>
```

The camera serial may be temporarily omitted only during discovery with exactly one device attached. It must be recorded and selected before acceptance.

## 6. Manual non-motion gate

Lift the drive wheels or otherwise prevent unintended movement. Start only default bringup:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Do not start any command publisher. In another shell:

```bash
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
- `odom` → every sensor frame resolvable at message time.

Required driver exit or camera startup failure must stop bringup. Fix hardware/dependencies rather than making sensors optional.

## 7. Lifted and floor motion gates

Follow [odometry_validation.md](odometry_validation.md). Use only the bounded pulse, joystick, or keyboard tool under direct supervision, one publisher at a time.

Verify:

- forward, left-strafe, and CCW encoder signs;
- `/odom` direction and TF;
- command watchdog stop;
- joystick held deadman, release stop, malformed-input protection, and timeout;
- keyboard timeout;
- calibration remains inert unless explicitly activated.

Then repeat conservative floor trials with external observations. Do not tune geometry or covariance from visual impression alone.

## 8. Simulator/physical acceptance

Run the same minimal consumer or contract-facing project against:

1. simulator commit `772ba25`;
2. this physical platform.

No topic remaps or frame-name substitutions are allowed. Simulation-only clock and ground truth are excluded.

## 9. Autostart decision

Autostart work may begin only when one X3 has passed:

- full non-motion probe;
- lifted motion and safety gates;
- bounded floor gates;
- simulator/physical consumer acceptance;
- stable motor, LiDAR, and camera identity configuration.

The future routine should be versioned in this repository, invoke the single strict platform launch, load per-robot identity configuration, propagate failures to the host service, and never start joystick, keyboard, calibration, or project behavior by default.

Until then, leave the fleet autostart disabled for this new stack.
