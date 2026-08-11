# Workstation And Robot Workflow

Date: 2026-08-11

This repo is the source boundary for the physical ROSMASTER X3 workspace. Keep the simulator in `AIRclub-UdeSA/yahboom_rosmaster`; keep the physical robot drivers and hardware setup here.

## Roles

Use the workstation for source review, Git work, tests that do not need hardware, odometry math, and documentation.

Use the robot/container for hardware checks: `Rosmaster_Lib`, serial devices, camera, LiDAR, `/cmd_vel`, `/vel_raw`, `/joint_states`, `/odom_raw`, `/odom`, and autostart validation.

If a coding agent is running on the robot, start with `context.md`.

## Outside The Robot

Clone and work from a normal ROS 2 workspace:

```bash
mkdir -p ~/rosmaster_physical_ws/src
cd ~/rosmaster_physical_ws/src
git clone git@github.com:AIRclub-UdeSA/physical_rosmaster.git
cd ~/rosmaster_physical_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src/physical_rosmaster
```

Build focused packages when possible:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select yahboomcar_base_node
colcon test --packages-select yahboomcar_base_node --ctest-args -R test_x3_odometry
```

Expected workstation limitation: `Rosmaster_Lib` is usually not installed outside the robot. That is fine for C++ odometry tests and code review. Do not vendor the downloaded Yahboom driver library into this repo until licensing is decided.

Before pushing:

```bash
git status --short
git diff --check
git diff
```

Keep generated state out of Git: `build/`, `install/`, `log/`, ROS bags, caches, local environment files, `ORBvoc.txt`, and `.pcd` point-cloud artifacts.

## Inside The Robot Container

The current deployed model uses Docker container `rosmaster_humble` with workspace `/root/yahboomcar_ws`.

Keep responsibilities clear:

- Robot host: WiFi, hostname, Docker lifecycle, systemd autostart, USB device visibility, backups.
- Docker container: ROS 2 commands, `colcon build`, `Rosmaster_Lib` import, launch files, topic checks.

Enter the container:

```bash
docker start rosmaster_humble
docker exec -it rosmaster_humble bash
```

Install basics if the container does not have them:

```bash
apt-get update
apt-get install -y git python3-pip python3-serial \
  ros-humble-robot-localization \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-usb-cam
```

Back up the current workspace before replacing the old `src` tree:

```bash
cd /root/yahboomcar_ws
tar czf /root/yahboomcar_ws_src_backup_$(date +%F_%H%M%S).tgz src
mv src src.before_physical_rosmaster_$(date +%F_%H%M%S)
mkdir -p src
```

Clone this repo under `src/physical_rosmaster`:

```bash
cd /root/yahboomcar_ws/src
git clone git@github.com:AIRclub-UdeSA/physical_rosmaster.git
```

If SSH keys are not configured on the robot, use a read-only deploy key or a short-lived GitHub token. Avoid putting personal tokens into shell history or shared notes.

Build the workspace:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install
source install/setup.bash
```

To update an existing robot clone later:

```bash
docker exec -it rosmaster_humble bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git pull --ff-only
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If `git pull --ff-only` refuses because the robot has local edits, stop and inspect with `git status --short`; do not overwrite robot-side changes blindly.

## Rosmaster_Lib In The Container

The robot guide copied `Rosmaster_Lib` from the host into `/root/Rosmaster_Lib` and exposed it through a `.pth` file. Keep that model until we replace it with a cleaner package/dependency plan.

Check that Python can import the library:

```bash
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK Rosmaster_Lib')"
```

Probe the exact installed source and hash:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

If import fails, restore the library path:

```bash
echo "/root/Rosmaster_Lib" > /usr/lib/python3.10/dist-packages/rosmaster_lib.pth
pip3 install pyserial
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK Rosmaster_Lib')"
```

## Large Local Artifacts

A fresh Git clone will not contain:

- `yahboomcar_slam/params/ORBvoc.txt`
- `yahboomcar_slam/pcl/*.pcd`

Normal X3 driver/base bringup should not need those files. ORB-SLAM or point-cloud demos may need them. If the team wants those features reproducible from a clean clone, publish the artifacts separately through Git LFS, a GitHub release asset, or an internal Drive link, then document the restore command.

## First Robot Validation After Clone

Use wheels lifted for first motion tests.

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

In another container shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 topic list
ros2 topic echo /vel_raw --once
ros2 topic echo /odom_raw --once
ros2 topic echo /imu/data_raw --once
```

Stop motion explicitly after any command:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Autostart

Do not point autostart at a newly cloned repo until manual bringup passes. Once validated, the existing host service can keep starting `/root/auto_start.sh`, and that script should source:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
```

Then start the physical launch:

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Future cleanup: move `/root/auto_start.sh`, `/usr/local/bin/start_rosmaster.sh`, systemd units, and udev rules into versioned files in this repo.

The current setup/autostart procedure is documented in `docs/setup_guide_ros2_humble_autostart.md`.
