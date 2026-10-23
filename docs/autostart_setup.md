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
| `rosmaster-shutdown-listen.sh` | `/usr/local/sbin/rosmaster-shutdown-listen` | optional, [section 7](#7-power-off-over-ros-2-systemshutdown): waits for a `/system/shutdown` request, then stops the platform and powers off |
| `rosmaster-shutdown-listener.service.example` | `/etc/systemd/system/rosmaster-shutdown-listener.service` | optional, section 7: the always-restarting unit that runs it |

The `.example` suffix marks a template; strip it on install. `platform.env`
holds per-robot values and must never be committed, matching how the udev
rules template works.

## 2. Discover per-robot values

Follow [setup_guide_ros2_humble_autostart.md section 8](setup_guide_ros2_humble_autostart.md#8-identify-this-robots-hardware)
to record the motor, LiDAR, and Astra identities for this robot. You need:

- the motor path (prefer `/dev/serial/by-id/...` when the controller has a real
  unique serial; use the udev-aliased `/dev/robot/motor` for controllers with
  none — a serial-less CH340 only shows the generic, non-unique by-id name
  `usb-1a86_USB_Serial-if00-port0`, which is the case on both `x3-c` and
  `mikey`);
- the LiDAR path, typically the udev-aliased `/dev/robot/lidar`;
- the Astra serial number, from `ros2 run astra_camera list_devices_node`
  inside the container.

## 3. Install

The files live in this repository's `config/` directory, and the repository
is normally cloned only inside the container, not on the robot's host. From a
workstation with the repository checked out, copy them to the robot's
host and work from there:

```bash
# On the workstation, from the repository root:
ssh pi@<robot-ip> 'mkdir -p /tmp/rosmaster-autostart-install'
scp config/rosmaster-*.sh config/rosmaster-*.service.example \
  pi@<robot-ip>:/tmp/rosmaster-autostart-install/
ssh pi@<robot-ip>
```

Then, on the robot's host:

```bash
cd /tmp/rosmaster-autostart-install
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
in step 2, with `ROSMASTER_MOTOR_PORT=/dev/robot/motor` for a serial-less CH340;
`ROSMASTER_CONTAINER_NAME` and `ROSMASTER_WORKSPACE` only need to
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

## 7. Power off over ROS 2 (`/system/shutdown`)

Optional, and installed separately from the platform units (issue #18). With
it, you can power the robot off without SSH by publishing one message from
any machine on the robot's `ROS_DOMAIN_ID`.

> **`poweroff` halts only the Raspberry Pi. It does not stop the motors.**
> The motor controller is battery-powered and has no link-loss watchdog
> ([known issue](troubleshooting/known_issues/motor-controller-no-link-loss-watchdog.md)).
> This listener stops `rosmaster-platform.service` first, so the driver sends
> its best-effort zero commands, and only then powers off. If the driver
> had already died, nothing sent those zeros. **Switch main power off once
> the Pi has halted, every time.**

How it works:

- `rosmaster-shutdown-listener.service` (host) runs
  `rosmaster-shutdown-listen`, which runs
  `tools/ros_shutdown_listener.py` inside the container with `docker exec`,
  from the workspace checkout, like the contract probe.
- The listener subscribes to `/system/shutdown` (`std_msgs/msg/String`,
  reliable, volatile) and exits with status 42 only when a message's `data`
  equals this robot's confirmation value. That value is its hostname, or
  `ROSMASTER_SHUTDOWN_CONFIRM` from `platform.env` when set. Anything else is
  logged and ignored.
- On status 42 the host script logs the request, runs `systemctl stop
  rosmaster-platform.service` (the same `ExecStop` sweep as any stop), logs
  the result, and runs `systemctl poweroff`. If the stop fails or times out,
  it logs an error and **powers off anyway**.
- Any other exit (container restarted, listener killed) restarts the unit
  after 5 s, indefinitely. The unit isn't bound to the platform unit, so it
  still works after the platform has crashed.
- A latched request can't fire at boot: the subscription is volatile, so it
  never receives a message published before it started.

The confirmation value guards against accidents (a stray message, a laptop
on the wrong domain), not deliberate misuse. Anything on the robot's domain
that knows the value can power it off. When other people's code shares the
domain (the competition), set a non-obvious `ROSMASTER_SHUTDOWN_CONFIRM` in
`platform.env` and restart the unit, or disable the unit for the event.

### Install

The listener runs from the container's checkout, so update that checkout to
a revision that includes `tools/ros_shutdown_listener.py` first (no
`colcon build` needed: `tools/` isn't installed). Copy the two files to the
host as in [section 3](#3-install), then on the robot's host:

```bash
sudo install -m 0755 -o root -g root rosmaster-shutdown-listen.sh /usr/local/sbin/rosmaster-shutdown-listen
sudo install -m 0644 -o root -g root rosmaster-shutdown-listener.service.example /etc/systemd/system/rosmaster-shutdown-listener.service
sudo systemctl daemon-reload
sudo systemd-analyze verify rosmaster-shutdown-listener.service
sudo systemctl enable --now rosmaster-shutdown-listener.service
sudo journalctl -u rosmaster-shutdown-listener.service --no-pager -l -n 5
```

The journal should show `Listening on /system/shutdown for this robot's
confirmation value`.

### Trigger it from a laptop

On the robot's domain (`raphael` 10, `donatello` 11, `leo` 12, `mikey` 13,
`splinter` 14), with the robot's hostname as `data`:

```bash
ROS_DOMAIN_ID=13 ros2 topic pub -1 /system/shutdown std_msgs/msg/String "{data: mikey}"
```

Always pass `-1`. It waits for the listener, sends once, and exits. Without
it, `ros2 topic pub` republishes every second until stopped, and the robot
would power off again on every boot while that laptop process is still
running.

Expected journal on the robot (`journalctl -b -1` after the next boot, since
the Pi halts):

```text
[system_shutdown_listener]: Confirmed shutdown request on /system/shutdown ('mikey'); handing over to the host to stop the platform and power off
rosmaster-shutdown: Shutdown requested over ROS (/system/shutdown); stopping rosmaster-platform.service so the driver sends its zero commands
rosmaster-platform-stop: Clean stop: all platform processes exited on SIGTERM
rosmaster-shutdown: rosmaster-platform.service stopped after <N>s
rosmaster-shutdown: Powering off the host. This halts only the Pi: switch main power off once it has halted
```

A message with any other `data` logs `Ignored /system/shutdown request
'<value>': it does not match this robot's confirmation value` and changes
nothing.

### Roll back

```bash
sudo systemctl disable --now rosmaster-shutdown-listener.service
sudo rm /etc/systemd/system/rosmaster-shutdown-listener.service
sudo systemctl daemon-reload
docker exec rosmaster_humble pkill -TERM -f ros_shutdown_listener.py
```

The last line clears the in-container listener. Stopping a `docker exec`
client doesn't kill what it started; the unit itself clears leftovers each
time it starts.

## Related documentation

- [setup_guide_ros2_humble_autostart.md](setup_guide_ros2_humble_autostart.md)
- [robot_side_verification_todo.md](robot_side_verification_todo.md)
- [robot_side_next_moves.md](robot_side_next_moves.md)
