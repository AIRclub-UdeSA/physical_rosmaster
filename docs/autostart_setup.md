# Autostart install and validation

Installs the versioned systemd autostart. Autostart is already validated for
this platform code — `x3-c` passed the
[robot-side verification checklist](robot_side_verification_todo.md) sections
1-6 before this was first installed there, but that checklist is
`x3-c`-specific history, not a precondition for another robot. For any other
X3, do this once
[setup_guide_ros2_humble_autostart.md](setup_guide_ros2_humble_autostart.md)'s
smoke check passes. Run every command on the robot's host, not inside the
container, unless shown otherwise.

## 1. File layout

All files live in [`config/`](../config/) and are versioned in this
repository:

| File | Installed to | Purpose |
| --- | --- | --- |
| `rosmaster-disk-guard.sh` | `/usr/local/sbin/rosmaster-disk-guard` | `ExecStartPre`: refuses startup if root is nearly full |
| `rosmaster-wait-for-platform.sh` | `/usr/local/sbin/rosmaster-wait-for-platform` | `ExecStartPre`: waits, bounded, for the container and motor/LiDAR/Astra devices |
| `rosmaster-platform-launch.sh` | `/usr/local/sbin/rosmaster-platform-launch` | `ExecStart`: execs into the container and runs the strict launch in the foreground |
| `rosmaster-platform-stop.sh` | `/usr/local/sbin/rosmaster-platform-stop` | `ExecStop`: name-based sweep of every platform node, as a safety net |
| `rosmaster-ready-launch.sh` | `/usr/local/sbin/rosmaster-ready-launch` | `ExecStart` of the ready service: runs the contract probe, then the boot signal |
| `rosmaster-ready-signal.sh` | `/usr/local/sbin/rosmaster-ready-signal` **inside the container** | the buzzer + `RGBLight` boot-ready sequence |
| `rosmaster-platform.env.example` | `/etc/rosmaster/platform.env` (filled in, never committed) | per-robot device identities and RGB effect choice |
| `rosmaster-platform.service.example` | `/etc/systemd/system/rosmaster-platform.service` | the main strict-launch unit |
| `rosmaster-platform-ready.service.example` | `/etc/systemd/system/rosmaster-platform-ready.service` | the oneshot verify-then-signal unit |

The `.example` suffix marks a template; strip it on install. `platform.env`
holds per-robot values and must never be committed, matching how the udev
rules template works.

## 2. Discover per-robot values

Follow [setup_guide_ros2_humble_autostart.md section 3](setup_guide_ros2_humble_autostart.md#3-identify-this-robots-hardware)
to record the motor, LiDAR, and Astra identities for this robot. You need:

- the motor serial path (prefer `/dev/serial/by-id/...`; fall back to the
  udev-aliased `/dev/robot/motor` only for controllers with no distinguishable
  serial);
- the LiDAR path, typically the udev-aliased `/dev/robot/lidar`;
- the Astra serial number, from `ros2 run astra_camera list_devices_node`
  inside the container.

## 3. Install

From a workstation with the repository checked out, or from a clone on the
robot's host — paths below assume the files were copied to
`/tmp/rosmaster-autostart-install/` first (e.g. with `scp`):

```bash
sudo mkdir -p /etc/rosmaster

sudo install -m 0755 -o root -g root rosmaster-disk-guard.sh /usr/local/sbin/rosmaster-disk-guard
sudo install -m 0755 -o root -g root rosmaster-wait-for-platform.sh /usr/local/sbin/rosmaster-wait-for-platform
sudo install -m 0755 -o root -g root rosmaster-platform-launch.sh /usr/local/sbin/rosmaster-platform-launch
sudo install -m 0755 -o root -g root rosmaster-platform-stop.sh /usr/local/sbin/rosmaster-platform-stop
sudo install -m 0755 -o root -g root rosmaster-ready-launch.sh /usr/local/sbin/rosmaster-ready-launch

sudo install -m 0644 -o root -g root rosmaster-platform.service.example /etc/systemd/system/rosmaster-platform.service
sudo install -m 0644 -o root -g root rosmaster-platform-ready.service.example /etc/systemd/system/rosmaster-platform-ready.service
```

Create the real environment file (replace every value with what you recorded
in step 2; `ROSMASTER_CONTAINER_NAME` and `ROSMASTER_WORKSPACE` only need to
change if this robot's container name or workspace path differs from the
documented defaults):

```bash
sudo tee /etc/rosmaster/platform.env > /dev/null << 'EOF'
ROSMASTER_CONTAINER_NAME=rosmaster_humble
ROSMASTER_WORKSPACE=/root/yahboomcar_ws
ROSMASTER_MOTOR_PORT=REPLACE_WITH_MOTOR_PATH
ROSMASTER_LIDAR_PORT=/dev/robot/lidar
ROSMASTER_ASTRA_SERIAL=REPLACE_WITH_ASTRA_SERIAL
READY_RGB_TRANSIENT_EFFECT=2
READY_RGB_TRANSIENT_SECONDS=3
READY_RGB_EFFECT=6
EOF
sudo chown root:root /etc/rosmaster/platform.env
sudo chmod 0640 /etc/rosmaster/platform.env
```

`rosmaster-ready-signal.sh` is deployed into the running container itself,
not read from the tracked workspace checkout, so it does not need a git
push/pull cycle to update:

```bash
docker cp rosmaster-ready-signal.sh rosmaster_humble:/usr/local/sbin/rosmaster-ready-signal
docker exec rosmaster_humble chown root:root /usr/local/sbin/rosmaster-ready-signal
docker exec rosmaster_humble chmod 0755 /usr/local/sbin/rosmaster-ready-signal
```

Reload systemd and sanity-check the units before starting anything:

```bash
sudo systemctl daemon-reload
sudo systemd-analyze verify rosmaster-platform.service rosmaster-platform-ready.service
```

## 4. Validate before enabling

Start the main service by hand first and confirm the full strict-launch graph
comes up:

```bash
sudo systemctl start rosmaster-platform.service
sudo systemctl status rosmaster-platform.service --no-pager
docker exec rosmaster_humble bash -lc \
  'source /opt/ros/humble/setup.bash && source /root/yahboomcar_ws/install/setup.bash && ros2 node list'
```

Expect all seven nodes: `driver_node`, `sllidar_node`, `base_node`,
`imu_filter_madgwick`, `robot_state_publisher`, `astra_sensor_adapter`,
`_hardware/astra/camera`. Then run the ready service by hand and confirm it
passes the contract probe and sends the boot signal:

```bash
sudo systemctl start rosmaster-platform-ready.service
sudo systemctl status rosmaster-platform-ready.service --no-pager
sudo journalctl -u rosmaster-platform-ready.service --no-pager -l | tail -10
```

Acceptance for this step:

- `rosmaster-platform.service` is `active (running)`, both `ExecStartPre`
  checks report `SUCCESS`.
- `rosmaster-platform-ready.service` exits `0/SUCCESS`; its journal shows
  `Physical contract PASSED` followed by `Sent boot-ready buzzer + RGBLight`.
- The buzzer pattern is audible and the RGB strip visibly plays the
  transient effect, then settles on the steady one and stays there.

## 5. Enable and do a real reboot test

Manual starts only prove the units work when triggered by hand. Enable both,
then reboot and confirm the whole sequence runs unattended:

```bash
sudo systemctl enable rosmaster-platform.service rosmaster-platform-ready.service
sudo reboot
```

After the robot comes back (allow a couple of minutes for the Pi, the
container restart, and the full ROS bringup):

```bash
sudo systemctl status rosmaster-platform.service rosmaster-platform-ready.service --no-pager
sudo journalctl -u rosmaster-platform-ready.service --no-pager -l -b 0
```

Confirm the ready-service journal for the current boot (`-b 0`) shows a
single complete run — probe pass, signal sent, `Deactivated successfully` —
and that you personally saw and heard the boot signal fire with no manual
intervention. This is the actual proof the robot is ready to be powered on
and used, not the manual start in step 4.

## 6. Disable or roll back

```bash
sudo systemctl disable --now rosmaster-platform.service rosmaster-platform-ready.service
sudo rm /etc/systemd/system/rosmaster-platform.service /etc/systemd/system/rosmaster-platform-ready.service
sudo systemctl daemon-reload
```

The scripts under `/usr/local/sbin/` and `/etc/rosmaster/platform.env` are
harmless to leave in place; remove them only if you want a clean uninstall.

## Related documentation

- [setup_guide_ros2_humble_autostart.md](setup_guide_ros2_humble_autostart.md)
- [robot_side_verification_todo.md](robot_side_verification_todo.md)
- [robot_side_next_moves.md](robot_side_next_moves.md)
