# `leo` deployment: 2026-10-13

Fourth fleet robot, set up from a fresh Yahboom factory Raspberry Pi 5
card by following [the per-robot setup guide](../docs/setup_guide_ros2_humble_autostart.md)
and [autostart setup](../docs/autostart_setup.md), the same day as
[`raphael`](raphael_deployment_2026-10-13.md). The setup was done on a home
network; the robot's default network is the faculty's `Privada`, like
`raphael` (see [Network](#network)).

Per the runbook, this did not repeat the one-time platform validation done on
`x3-c`: no motor live-loss probe, no active-link-loss test, no consumer-parity
run. It is a per-unit deployment and smoke check only.

## Result

Pass, including a cold boot.

- With the wheels lifted, `tools/physical_contract_probe.py` failed once on the
  point cloud rate floor (`/cam_1/depth/color/points: measured 2.31 Hz`) and
  passed on the one allowed rerun, with all seven nodes up.
- Started by hand, `rosmaster-platform-ready.service` failed once on the same
  floor (`2.50 Hz`), then passed on the rerun. The operator saw and heard the
  signal.
- After both units were enabled, one unattended warm reboot and one cold boot
  (clean `poweroff`, power switch off about 10 s, back on) each produced a
  single complete run of the ready service: `Physical contract PASSED`, then
  `Sent boot-ready buzzer + RGBLight`, then `Deactivated successfully`. The
  operator saw and heard the boot signal both times.

## Deployed revision

| Item | Value |
| --- | --- |
| `physical_rosmaster` | `main` at `aebad554e1fb95ce6dec0d9c79bdd155768777cb`, clean, equal to `origin/main` |
| `ros2_astra_camera` | `f7e71d9ce806e788cb48d8580aac2c778fba4214`, clean |
| Workspace | exactly the eight local packages plus `astra_camera` and `astra_camera_msgs`; `colcon build --symlink-install` finished in 4 min 3 s |
| `Rosmaster_Lib` | V3.3.9, SHA256 `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`; `tools/rosmaster_lib_probe.py --hash-only` exits 0 |

## Host and container

| Item | Value |
| --- | --- |
| Hostname | `leo` (`leo.local`) |
| `ROS_DOMAIN_ID` | 12 (`raphael` uses 10, `x3-c` 11, `mikey` 13) |
| Timezone | `America/Argentina/Buenos_Aires` |
| Hardware / OS | Raspberry Pi 5 Model B Rev 1.0, Debian 12, kernel `6.6.62+rpt-rpi-2712` |
| Image | `yahboomtechnology/ros-humble:4.1.2`, digest `sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3`, image ID `dedc515c326a` |
| Disk | root 36 GB at 96% before expansion; 57 GB after; 58% used at the end |

Container command:

```bash
docker run -dit --name rosmaster_humble --net=host --privileged -v /dev:/dev \
  --restart unless-stopped -e ROS_DOMAIN_ID=12 -e DISPLAY=:0 \
  yahboomtechnology/ros-humble:4.1.2 bash
```

The factory card had the same SSH host keys and `machine-id` as `raphael`'s
factory card, which confirms the shared identity the guide warns about; both
were regenerated. Like `raphael`, it shipped with only the three `ros-foxy`
images and no Humble image or container. The factory
`~/.config/autostart/rosmaster.desktop` app **was running** when the survey
started: `rosmaster_main.py` held the motor CH340 and listened on TCP 6000 and
6500 on the home network. It was stopped with SIGINT and its `.desktop` file
moved to `~/.config/autostart-disabled/`.

`Rosmaster_Lib`, the apt key replacement (`ros2-apt-source` 1.2.0), the
factory workspace move and the container `/root/.bashrc` fix (domain 20 and
factory overlays disabled; interactive shells report domain 12) were done as
for `raphael`.

## Device identities

| Device | IDs | Identity |
| --- | --- | --- |
| Motor controller | CH340 `1a86:7523` | no serial; bound by physical port `KERNELS=="3-1.2"` (VIA Labs hub `2109:2817` on the Yahboom board, port 2, on Pi root port `3-1`) |
| LiDAR | CP2102 `10c4:ea60` | USB serial `0001`, topology `1-2` |
| Astra depth | `2bc5:060f` | OpenNI serial `ACRC64300FR` |
| Astra UVC/RGB | `2bc5:050f` | |

The motor topology differs from every other robot (`x3-c` and `mikey` use
`1-1.2`, `raphael` `1-2.2`): it depends on which Pi port the Yahboom board's
USB cable is in. The Astra's internal hub (`05e3:0608`) is on Pi root port
`3-2`, not on the Yahboom hub, and its depth function enumerated on the warm
reboots and on the cold boot.

The installed host rules are in
[99-rosmaster-x3.leo.rules](99-rosmaster-x3.leo.rules), plus the pinned
`ros2_astra_camera` Orbbec rule (SHA256
`fd5bcd51b7b6bb7115868d210c68c0ef87d1706580d6d8bb2a777fbdb0590bcf`). The
factory `usb.rules` and `myserial.rules` were moved to
`/etc/udev/rules.disabled/`; the original directory is backed up under
`/var/backups/rosmaster-recovery/2026-10-13-udev-preinstall/`. After a reboot
the aliases and permissions matched the guide on the host and in the
container, and the legacy aliases were gone.

`/etc/rosmaster/platform.env` (not committed) sets
`ROSMASTER_MOTOR_PORT=/dev/robot/motor`, `ROSMASTER_LIDAR_PORT=/dev/robot/lidar`,
and `ROSMASTER_ASTRA_SERIAL=ACRC64300FR`, with the template defaults for the
container name, workspace and boot-ready RGB effects.

## Smoke-check pulses

Wheels lifted, battery about 11.0 V, one-second `tools/safe_cmd_vel_pulse.py`
pulses, each recorded continuously from `/joint_states` and `/odom`. Values are
the change in each wheel's position (rad) and in `/odom`:

| Command | FL | FR | BL | BR | `/odom` | Signs vs. [odometry_validation.md](../docs/odometry_validation.md) |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| `x=0.10` | +1.91 | +1.06 | +0.97 | +1.06 | x +0.040 | match |
| `y=0.10` | -1.91 | +1.55 | +1.42 | -1.06 | y +0.048 | match |
| `yaw=0.60` | -2.04 | +1.72 | -1.20 | +1.34 | yaw +0.31 rad | match |

All four wheels started within 0.1-0.3 s of each other in every pulse; no
wheel lagged the way `raphael`'s back-right wheel did.

## Battery

The controller reported 11.0 V at idle before the smoke check, 10.9 V after
the pulses and through the warm reboot, and 10.8 V after the cold boot.

## Cold boot

Monotonic boot times for the cold boot:

```text
[17.75] rosmaster-wait-for-platform: OK: container and all devices present
[22.55] physical_contract_probe: Waiting up to 35.0s for the physical platform contract
[25.50] astra_camera_node: Device connected
[27.52] Physical contract PASSED
[34.86] systemd-timesyncd: Initial clock synchronization
[45.89] Sent boot-ready buzzer + RGBLight (transient=2 for 3s, then steady=6)
```

The NTP step came after the probe and no false encoder timeout was logged.
Compared with `raphael`'s failed cold boot, the camera delivered depth frames
at full rate from the start; `raphael`'s probe judged the first depth frames
six seconds after the camera connected, while they were still arriving about
1.4 s apart. One pass here does not show how often that happens.

## Point cloud rate floor

The first probe in the lifted smoke check and the first manual ready-service
run both failed only on `/cam_1/depth/color/points` (2.31 Hz and 2.50 Hz), and
`raphael`'s first manual run failed the same way (2.31 Hz). Measured with
`ros2 topic hz`, the cloud averaged about 8.2 Hz. The probe judges a topic on
the five messages after it subscribes, so occasional gaps in the cloud stream
can fall inside that window. Both reruns passed, and both automatic boots
passed on the first try. No gate was changed.

## Network

`NetworkManager` on this card already had a `Privada` profile (WPA-PSK,
password stored, `autoconnect-priority` 1), along with the factory `ROSMASTER`
access-point profile (autoconnect off) and the factory `Yahboom2` profile
(autoconnect off). The home network profile used during setup had priority 0,
so the robot prefers `Privada` whenever it is in range. As the last step of
the session, after checking that `Privada` has its password stored, the home
profile was scheduled for deletion so its password does not stay on the robot
(`nmcli connection delete`, logged to `/var/log/leo-home-wifi-delete.log`).
The robot dropped off the home network seconds later, as expected. On the
first boot on `Privada` (2026-10-14) the profile was already gone: `nmcli`
does not list it, there is no file for it in
`/etc/NetworkManager/system-connections/`, and the deletion log reports it
`successfully deleted`. Nothing had to be deleted that day. The only other
file in that directory is an empty `Wi-Fi connection 1.nmconnection` from the
factory image (December 2023), which `NetworkManager` does not load.

## First boot on the faculty network (2026-10-14)

Pass on the one allowed cold restart. Both boots connected to `Privada`
(`Privada:wlan0` active) without touching any cable, and
`rosmaster-platform.service` was active both times. The shutdown before them,
on 2026-10-13, logged `Clean stop: all platform processes exited on SIGTERM`
and `Journal stopped`.

- First boot (operator switched it on at about 14:20): the operator did not
  hear the signal. The ready service ran once and failed on the point cloud
  rate floor, the same known failure as in
  [Point cloud rate floor](#point-cloud-rate-floor):
  `Physical contract FAILED: /cam_1/depth/color/points: measured 2.73 Hz outside 3.0..40.0 Hz`.
  All four USB devices were present. The log was kept on the robot as
  `~/leo-falla-1.log`, and the robot was restarted cold (clean `poweroff`,
  power switch off about 10 s, back on).
- Second boot (switched on at about 14:26): the operator heard the beeps and
  saw the lights. One complete run of the ready service:

  ```text
  [27.88] Physical contract PASSED
  [43.22] Sent boot-ready buzzer + RGBLight (transient=2 for 3s, then steady=6)
  [43.23] rosmaster-platform-ready.service: Deactivated successfully.
  ```

  `rosmaster-wait-for-platform` reported all devices at 16.84 s and the NTP
  step came at 34.52 s, after the probe. All four USB devices
  (`1a86:7523`, `10c4:ea60`, `2bc5:060f`, `2bc5:050f`) were present,
  `/dev/robot/motor` and `/dev/robot/lidar` existed, `ros2 node list` showed
  all seven nodes on the first try, and the container reported
  `ROS_DOMAIN_ID` 12. The battery read 10.7 V, `vcgencmd get_throttled`
  returned `0x0`, and the root filesystem was 58% used.

## Factory items left in place

- `/etc/hosts` pins `raw.githubusercontent.com` to `185.199.110.133`.
- The kernel command line sets `cfg80211.ieee80211_regdom=CN`.
- `jupyter-lab` listens on `0.0.0.0:8888` and `wayvnc` on 5900; both are
  active.
- SSH accepts password login and `PermitRootLogin without-password`; the
  factory `pi` password was not changed.
- `motion.service` is enabled but inactive; `ModemManager` is active.
- In the Humble image, `/etc/systemd/system/my_ros_service.service`,
  `/root/run_handle.sh` (mode 0777) and the Supervisor configs `chassis.conf`,
  `chassis.config` and `ip.conf` (`autostart=true`, running `run_handle.sh`
  and `run_ip.sh`) are present and inactive, because the container's PID 1 is
  `bash`. `/root/auto_start.sh` does not exist. `rosdep install` removed the
  image's `systemctl` replacement package while installing `systemd`.

## Not tested

- The stop path on its own; it ran as part of both shutdowns and logged a
  clean stop each time.
- Floor trials, and the operator tools (joystick, keyboard, calibration).
