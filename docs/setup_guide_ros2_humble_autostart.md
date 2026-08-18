# ROS 2 Humble And Autostart Setup Guide

Date: 2026-08-11

This is the repository copy of the original ROSMASTER X3 setup/autostart guide. It has been sanitized for a public repo and updated to use the public `AIRclub-UdeSA/physical_rosmaster` Git workflow instead of a manually transferred `src.zip`.

Use placeholders such as `<ROBOT_IP>`, `<ROBOT_HOSTNAME>`, `<ROS_DOMAIN_ID>`, `<CAMERA_DEVICE>`, and `<LIDAR_SERIAL_PORT>` for robot-specific values.

## Scope

The current deployment model is:

- robot host runs Docker
- ROS 2 Humble runs inside Docker container `rosmaster_humble`
- container image: `yahboomtechnology/ros-humble:4.1.2`
- ROS workspace inside container: `/root/yahboomcar_ws`
- physical source repo cloned at `/root/yahboomcar_ws/src/physical_rosmaster`
- `Rosmaster_Lib` is copied from the robot host into the container and exposed to Python
- autostart is currently host systemd -> Docker exec -> `/root/auto_start.sh`

## Host Network And Identity

Set the robot hostname:

```bash
hostname
sudo hostnamectl set-hostname <ROBOT_HOSTNAME>
sudo nano /etc/hosts
sudo reboot
```

Update `/etc/hosts` so the `127.0.1.1` line uses the new hostname:

```text
127.0.1.1 <ROBOT_HOSTNAME>
```

Configure the robot WiFi from the desktop network manager or your site-specific network tooling. Do not commit WiFi passwords or private network credentials to this repo.

### Set The Default WiFi Network

Before doing anything over SSH, set the network the robot should join automatically on boot. This is done from the robot's own desktop, using the NetworkManager applet.

1. Click the Network icon and open **Edit Connections**.
2. Find the robot's default/factory WiFi network (e.g. an access point the robot itself broadcasts or ships pre-configured with) and select it.
3. Go to the **General** tab and uncheck **"Connect automatically with priority"**. Save the changes.
4. Click the Network icon again and connect to the target WiFi network (`<TARGET_WIFI_SSID>`).
5. If the network does not appear in the list, go to **Advanced Options → Connect to Hidden Wi-Fi Network** and create a hidden connection for `<TARGET_WIFI_SSID>`. Once created, it should also show up under the regular Network icon.
6. Enter the WiFi password when prompted. Do not store this password in this repo.
7. Go to **Edit Connections → `<TARGET_WIFI_SSID>` → General** and check **"Connect automatically with priority"**.
8. Reboot the robot. It should now connect automatically to `<TARGET_WIFI_SSID>` on startup.

Only once this is confirmed should you proceed to SSH-based setup below.

SSH into the robot host:

```bash
ssh pi@<ROBOT_IP>
```

Use the configured robot password or SSH key. Do not store credentials in this repo.

## Docker Cleanup And Humble Image

Inspect existing Docker state:

```bash
docker images
docker ps -a
docker system df
df -h
```

Remove old stopped containers or unused images only after confirming they are not needed. The old Foxy images can consume significant disk space.

For a deep clean (stopped containers, dangling and unused images, unused networks, and build cache), use:

```bash
docker system prune -a
```

This removes anything not associated with a running container, so double-check `docker ps -a` first and confirm before running it on a shared robot.

Pull the Humble image:

```bash
docker pull yahboomtechnology/ros-humble:4.1.2
docker images
```

## Create The Humble Container

Stop/remove an old Humble container only after backing up anything needed from it:

```bash
docker rm -f rosmaster_humble
```

Create the container:

```bash
docker run -it --privileged \
  --network host \
  --name rosmaster_humble \
  -v /dev:/dev \
  -e ROS_DOMAIN_ID=<ROS_DOMAIN_ID> \
  yahboomtechnology/ros-humble:4.1.2 bash
```

Inside the container, make the domain persistent:

```bash
sed -i 's/ROS_DOMAIN_ID=20/ROS_DOMAIN_ID=<ROS_DOMAIN_ID>/g' ~/.bashrc
cat ~/.bashrc | grep ROS_DOMAIN_ID
source ~/.bashrc
echo "$ROS_DOMAIN_ID"
```

Source ROS and any installed workspaces:

```bash
source /opt/ros/humble/setup.bash
source ~/imu_ws/install/setup.bash 2>/dev/null || true
source ~/gmapping_ws/install/setup.bash 2>/dev/null || true
source ~/yahboomcar_ws/install/setup.bash 2>/dev/null || true
```

From the robot host, configure Docker restart:

```bash
docker update --restart unless-stopped rosmaster_humble
docker inspect rosmaster_humble | grep RestartPolicy -A 3
```

## Configure Rosmaster_Lib

On the robot host, locate the installed Yahboom Python library:

```bash
sudo find / -type d -name "Rosmaster_Lib" 2>/dev/null
```

Common host path:

```text
/home/pi/software/py_install/Rosmaster_Lib
```

Copy it into the container:

```bash
docker cp /home/pi/software/py_install/Rosmaster_Lib rosmaster_humble:/root/
```

Enter the container:

```bash
docker start rosmaster_humble
docker exec -it rosmaster_humble bash
```

Expose the library to Python:

```bash
echo "/root/Rosmaster_Lib" > /usr/lib/python3.10/dist-packages/rosmaster_lib.pth
pip3 install pyserial
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK Rosmaster_Lib')"
```

After this repo is cloned, inspect the exact installed copy:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

## Clone This Repository Into The Container

Inside the container:

```bash
apt-get update
apt-get install -y git curl unzip python3-pip python3-serial \
  ros-humble-robot-localization \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-usb-cam
```

Back up any existing source tree before replacing it:

```bash
cd /root/yahboomcar_ws
tar czf /root/yahboomcar_ws_src_backup_$(date +%F_%H%M%S).tgz src
mv src src.before_physical_rosmaster_$(date +%F_%H%M%S)
mkdir -p src
```

Clone and build:

```bash
cd /root/yahboomcar_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src/physical_rosmaster
colcon build --symlink-install
source install/setup.bash
```

Optional SLAM/PCD artifacts:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
tools/fetch_large_artifacts.sh
```

Normal X3 driver bringup does not require those large artifacts.

The `yahboomcar_slam` package now guards the optional `pcl` install path, so a clean clone should not fail just because the large bundle was not restored.

## Manual Hardware Tests

Use wheels lifted for first motion tests.

Motor driver only:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3
```

In another container shell:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Camera:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p device_id:=0 \
  -p frame_id:=default_cam \
  -p image_width:=320 \
  -p image_height:=240 \
  -p framerate:=10.0
```

LiDAR:

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py \
  serial_port:=<LIDAR_SERIAL_PORT> \
  frame_id:=laser_link \
  serial_baudrate:=115200
```

Full physical bringup:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

Topic checks:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /vel_raw --once
ros2 topic echo /odom_raw --once
ros2 topic echo /imu/data_raw --once
```

## Odometry Validation

The next odometry step requires robot hardware.

Run:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 100 --period 0.1
```

Then follow `docs/odometry_validation.md`.

## Autostart Script Inside The Container

Create `/root/auto_start.sh` inside the container:

```bash
nano /root/auto_start.sh
```

Recommended current content:

```bash
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash

echo "[$(date)] Starting ROSMASTER X3 bringup..." >> /tmp/ros_autostart.log

nohup ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py >> /tmp/bringup.log 2>&1 &
sleep 5

nohup ros2 launch yahboomcar_description display_X3.launch.py >> /tmp/tfs_description.log 2>&1 &
sleep 5

echo "[$(date)] Waiting for USB devices..." >> /tmp/ros_autostart.log
sleep 10

if [ ! -e /dev/video0 ]; then
  echo "[$(date)] WARNING: /dev/video0 not found" >> /tmp/ros_autostart.log
fi

if [ ! -e /dev/ttyUSB1 ]; then
  echo "[$(date)] WARNING: /dev/ttyUSB1 not found" >> /tmp/ros_autostart.log
fi

echo "[$(date)] Starting usb_cam..." >> /tmp/ros_autostart.log
nohup ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p device_id:=0 \
  -p frame_id:=default_cam \
  -p image_width:=320 \
  -p image_height:=240 \
  -p framerate:=10.0 \
  -p pixel_format:=mjpeg2rgb >> /tmp/usb_cam.log 2>&1 &

sleep 3

echo "[$(date)] Starting sllidar..." >> /tmp/ros_autostart.log
nohup ros2 launch sllidar_ros2 sllidar_a1_launch.py \
  serial_port:=/dev/ttyUSB1 \
  frame_id:=laser_link \
  serial_baudrate:=115200 >> /tmp/sllidar.log 2>&1 &

sleep 5

if pgrep -f sllidar_node >/dev/null; then
  echo "[$(date)] sllidar_node is running" >> /tmp/ros_autostart.log
  ros2 topic pub -1 /RGBLight std_msgs/msg/Int32 "data: 3" >/dev/null 2>&1 || true
else
  echo "[$(date)] ERROR: sllidar_node did not start. Retrying /dev/ttyUSB0..." >> /tmp/ros_autostart.log
  nohup ros2 launch sllidar_ros2 sllidar_a1_launch.py \
    serial_port:=/dev/ttyUSB0 \
    frame_id:=laser_link \
    serial_baudrate:=115200 >> /tmp/sllidar.log 2>&1 &
fi

echo "[$(date)] ROS nodes started" >> /tmp/ros_autostart.log

ros2 topic pub -1 /RGBLight std_msgs/msg/Int32 "data: 0" >/dev/null 2>&1 || true

tail -f /dev/null
```

Make it executable:

```bash
chmod +x /root/auto_start.sh
```

## Host Autostart Script

On the robot host, create `/usr/local/bin/start_rosmaster.sh`:

```bash
sudo nano /usr/local/bin/start_rosmaster.sh
```

Content:

```bash
#!/bin/bash
set -e

sleep 15

pkill -f rosmaster_main.py || true
sleep 2

if [ -n "$(docker ps -q -f name=rosmaster_humble)" ]; then
  echo "Container rosmaster_humble is already running"
else
  docker start rosmaster_humble
  sleep 10
fi

docker exec -d rosmaster_humble bash -c "source /opt/ros/humble/setup.bash && /root/auto_start.sh"

echo "ROS 2 nodes started in rosmaster_humble"
```

Make it executable:

```bash
sudo chmod +x /usr/local/bin/start_rosmaster.sh
```

## systemd Service

On the robot host, create `/etc/systemd/system/rosmaster-autostart.service`:

```ini
[Unit]
Description=ROSMASTER Auto Start Service
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/start_rosmaster.sh
Restart=on-failure
RestartSec=10s

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable rosmaster-autostart.service
sudo systemctl start rosmaster-autostart.service
```

Verify after about 30 seconds:

```bash
docker exec -it rosmaster_humble bash -c "source /opt/ros/humble/setup.bash && source /root/yahboomcar_ws/install/setup.bash && ros2 node list"
```

Expected core nodes include the driver node, LiDAR node, camera node, and robot description/TF nodes depending on launch state.

## Future Hardening

This guide preserves the currently working deployment style. Later cleanup should move autostart scripts, systemd units, udev rules, and device checks into versioned files under this repo.
