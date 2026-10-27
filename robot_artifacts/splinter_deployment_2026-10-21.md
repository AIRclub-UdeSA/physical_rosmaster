# `splinter` deployment: 2026-10-21

Fifth fleet robot, set up by following
[the per-robot setup guide](../docs/setup_guide_ros2_humble_autostart.md)
and [autostart setup](../docs/autostart_setup.md). Unlike the other four,
this card was not fresh: it carried an older Yahboom factory image with
leftover Melodic state. It was surveyed, wiped back to a factory-like state
(see [Prior state and wipe](#prior-state-and-wipe)) and then set up as if it
were fresh. The work started on 2026-10-16 and was paused during the
workspace build when the battery ran low (a clean `poweroff`); it resumed and
finished on 2026-10-21. The robot runs on the faculty's `Privada` network,
like the rest of the fleet (see [Network](#network)).

Per the runbook, this did not repeat the one-time platform validation done on
`x3-c`: no motor live-loss probe, no active-link-loss test, no consumer-parity
run. It is a per-unit deployment and smoke check only.

## Result

Pass, including a cold boot, with one owner-accepted wheel characteristic.

- With the wheels lifted, `tools/physical_contract_probe.py` passed on the
  first try, with all seven nodes up. The point cloud rate floor did not fail
  on this robot.
- The front-right wheel did not respond to any of the runbook's one-second
  pulses but turns normally in longer ones: it starts up to about 1.2 s
  after the other three. The owner accepted this as a characteristic of this
  unit; see [Front-right start-up lag](#front-right-start-up-lag).
- Started by hand, `rosmaster-platform-ready.service` passed on the first try.
  The operator saw and heard the signal.
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
| Workspace | exactly the eight local packages plus `astra_camera` and `astra_camera_msgs`; `colcon build --symlink-install` finished in 2 min 39 s |
| `Rosmaster_Lib` | V3.3.9, SHA256 `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`; `tools/rosmaster_lib_probe.py --hash-only` exits 0 |

## Prior state and wipe

The card's `~/Version.txt` reads `Card NO.52, Version:4.0.1, Released
20240822`. It is an older factory image than the other four cards: kernel
`6.6.45-v8-16k+` instead of `6.6.62+rpt-rpi-2712`, and Melodic instead of
Foxy images. Before anything was changed, the survey found:

- Docker: two `yahboomtechnology/ros-melodic` images (`4.0.1` and `4.0.0`)
  and six stopped or never-started Melodic containers from 2024 (random
  names, no restart policy, not privileged, no `ROS_DOMAIN_ID`). There was no
  Foxy or Humble image and no `rosmaster_humble` container.
- Host: a Melodic catkin workspace in `~/yahboomcar_ws`, and the scripts that
  started the Melodic containers (`~/run_docker.sh`, `~/test.sh`).
- The root partition was already expanded (54 GB) and 71% used.
- The factory `~/.config/autostart/rosmaster.desktop` was present, but
  `rosmaster_main.py` was not running and nothing listened on TCP 6000 or
  6500.
- udev: no `usb.rules`. Instead, `rplidar.rules` (mode `0777`) created
  `/dev/rplidar` and `/dev/myserial`; `myserial.rules` was all comments.
- No `rosmaster-*` units, scripts or `/etc/rosmaster`, no custom
  `99-rosmaster-x3.rules`, and no crontab or `rc.local` lines that start ROS.
- Host health was good: no failed units, an empty `dpkg --audit`, no
  filesystem or mmc errors, `get_throttled` `0x0`, and the expected
  `Rosmaster_Lib` hash.

With the owner's approval, the following was removed: all six containers,
both Melodic images, `~/yahboomcar_ws`, `~/run_docker.sh`, `~/test.sh`,
`~/.bash_history`, and the empty `~/temp` and `~/yahboomba` directories. The
root filesystem went from 36 GB used (71%) to 17 GB (32%). The factory demos
in the home directory (`KNN`, `TensorFlow_demo`, `yolov4-tiny-tf2`,
`yolov5-master`, `Bookshelf`, `Downloads`, `Pictures`, `~/.local`, `~/.cache`),
the factory app in `~/Rosmaster`, `~/software` and the host's `Rosmaster_Lib`
egg were kept.

## Host and container

| Item | Value |
| --- | --- |
| Hostname | `splinter` (`splinter.local`) |
| `ROS_DOMAIN_ID` | 14 (`raphael` uses 10, `x3-c` 11, `leo` 12, `mikey` 13) |
| Timezone | `America/Argentina/Buenos_Aires` |
| Hardware / OS | Raspberry Pi 5 Model B Rev 1.0, Debian 12, kernel `6.6.45-v8-16k+` |
| Image | `yahboomtechnology/ros-humble:4.1.2`, digest `sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3`, image ID `dedc515c326a`, pulled by digest |
| Disk | root 54 GB (already expanded); 32% used after the wipe; 64% at the end |

Container command:

```bash
docker run -dit --name rosmaster_humble --net=host --privileged -v /dev:/dev \
  --restart unless-stopped -e ROS_DOMAIN_ID=14 -e DISPLAY=:0 \
  yahboomtechnology/ros-humble:4.1.2 bash
```

The card had the shared factory SSH host keys; the host keys and
`machine-id` were regenerated. `Rosmaster_Lib`, the apt key replacement
(`ros2-apt-source` 1.2.0), the factory workspace move and the container
`/root/.bashrc` fix (domain 20 and factory overlays disabled; interactive
shells report domain 14) were done as for `leo`. The factory
`rosmaster.desktop` was moved to `~/.config/autostart-disabled/`.

The battery-driven pause interrupted `rosdep install` while apt was unpacking
packages inside the container. Six `-dev` packages were left unpacked but not
configured and `libgdcm-dev` needed a reinstall. `dpkg --configure -a` and
`apt-get install --reinstall libgdcm-dev` fixed it (empty `dpkg --audit`),
and the remaining section 7 steps then ran on the already-verified clone.

## Device identities

| Device | IDs | Identity |
| --- | --- | --- |
| Motor controller | CH340 `1a86:7523` | no serial; bound by physical port `KERNELS=="1-1.1"` (VIA Labs hub `2109:2817` on the Yahboom board, port 1, on Pi root port `1-1`) |
| LiDAR | CP2102 `10c4:ea60` | USB serial `0001`, topology `1-1.4` |
| Astra depth | `2bc5:060f` | OpenNI serial `ACRC64300FA` |
| Astra UVC/RGB | `2bc5:050f` | |

The motor topology differs from every other robot (`x3-c` and `mikey` use
`1-1.2`, `raphael` `1-2.2`, `leo` `3-1.2`). The cable from the Pi to the
Yahboom board is in the Pi's bottom blue USB 3 port and must stay there. The
Astra's internal hub (`05e3:0608`) is on the Yahboom hub too (`1-1.2`),
not on a direct Pi port as on `leo`. Its depth function enumerated on every
boot in this session, including the cold boot.

The installed host rules are in
[99-rosmaster-x3.splinter.rules](99-rosmaster-x3.splinter.rules)
(byte-identical to the installed file), plus the pinned
`ros2_astra_camera` Orbbec rule (SHA256
`fd5bcd51b7b6bb7115868d210c68c0ef87d1706580d6d8bb2a777fbdb0590bcf`). The
factory `myserial.rules` and `rplidar.rules` were moved to
`/etc/udev/rules.disabled/`; the original directory is backed up under
`/var/backups/rosmaster-recovery/udev-preinstall/`. After a reboot the
aliases and permissions matched the guide on the host and in the container,
and `/dev/myserial` and `/dev/rplidar` were gone.

`/etc/rosmaster/platform.env` (not committed) sets
`ROSMASTER_MOTOR_PORT=/dev/robot/motor`, `ROSMASTER_LIDAR_PORT=/dev/robot/lidar`,
and `ROSMASTER_ASTRA_SERIAL=ACRC64300FA`, with the template defaults for the
container name, workspace and boot-ready RGB effects.

## Smoke-check pulses

Wheels lifted, battery 10.7 V, `tools/safe_cmd_vel_pulse.py` pulses, each
recorded continuously from `/joint_states` and `/odom`. Values are the change
in each wheel's position (rad) and in `/odom`:

| Command | FL | FR | BL | BR | `/odom` | Signs vs. [odometry_validation.md](../docs/odometry_validation.md) |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| `x=0.10`, 1 s | +1.95 | 0.00 | +1.67 | +1.38 | x +0.040 | FR missing; others match |
| `y=0.10`, 1 s | -2.08 | 0.00 | +1.86 | -1.55 | y +0.046 | FR missing; others match |
| `yaw=0.60`, 1 s | -2.00 | 0.00 | -1.86 | +1.46 | yaw +0.26 rad | FR missing; others match |
| `x=0.10`, 3 s | +9.93 | +7.66 | +9.58 | +8.89 | x +0.292 | match |
| `x=0.10`, 15 s | +57.19 | +55.08 | +56.57 | +56.05 | x +1.794 | match |

The one-second pulses were run twice with the same result, plus one more
`x=0.10` pulse while the operator watched the front-right wheel; it did not
turn.

## Front-right start-up lag

The front-right wheel (controller channel M3, per the driver's
`encoder_order` `[0, 2, 1, 3]`) does not start within a one-second pulse.
Before it had ever been driven, its raw encoder read exactly `0` while the
other three had small nonzero counts. In a 3 s pulse it started about 1.1 to
1.2 s after the other three; in the following 15 s pulse, about 0.6 s after.
Once it turns, it has the right sign and keeps pace with the others. Its
15 s total is about 1.5 to 2 rad short, which matches the late start. The
wheel and its encoder therefore both work. The delay is in starting to
move, and it shrank with use. That points to something mechanical, such as
stiction in the gearbox or bearings, rather than wiring. This was not
confirmed by inspection.

Throughout, `/diagnostics` reported `Wheel encoder odometry healthy`. The
owner accepted this as a characteristic of this unit, like `mikey`'s
low-speed stiction and `raphael`'s lagging back-right wheel. No gate, tool
limit or speed was changed. Because the wheel lags, `/odom` shows sideways
drift and a small yaw (`-0.11` to `-0.15` rad) during forward pulses. Floor
odometry on this unit should be read with that in mind.

## Battery

The controller reported 10.7 V at idle before the smoke check and 10.6 V
after the pulses and the manual start. `/voltage` read 10.5 V after the cold
boot. These readings are lower than the rest of the fleet's (`leo` 11.0 V,
`x3-c` about 11.3 V), even after charging between the two days. The
battery's state of health was not measured.

## Warm reboot and cold boot

Monotonic boot times:

```text
warm reboot:
[33.33] rosmaster-wait-for-platform: OK: container and all devices present
[35.98] systemd-timesyncd: Initial clock synchronization
[47.49] astra_camera_node: Device connected: Astra serial number: ACRC64300FA
[49.75] Physical contract PASSED
[69.25] Sent boot-ready buzzer + RGBLight (transient=2 for 3s, then steady=6)

cold boot:
[32.78] rosmaster-wait-for-platform: OK: container and all devices present
[37.31] systemd-timesyncd: Initial clock synchronization
[58.07] astra_camera_node: Device connected: Astra serial number: ACRC64300FA
[60.58] Physical contract PASSED
[80.49] Sent boot-ready buzzer + RGBLight (transient=2 for 3s, then steady=6)
```

On both boots the NTP step came before the probe, and no encoder-timeout
message was logged. The shutdown before the cold boot logged `Clean stop:
all platform processes exited on SIGTERM` and `Journal stopped`.
`vcgencmd get_throttled` returned `0x0` after each boot.

## Network

`NetworkManager` has a `Privada` profile (WPA-PSK, password stored,
`autoconnect-priority` 0), which was used for the whole setup. The other
profiles were the factory `ROSMASTER` access point (autoconnect turned off by
the operator; kept), the factory `Yahboom2` profile (autoconnect turned off by
the operator; deleted at the owner's request) and `Wired connection 1`
(priority -999). The empty factory `Wi-Fi connection 1.nmconnection` in
`/etc/NetworkManager/system-connections/` is still there; `NetworkManager`
does not load it.

## Factory items left in place

- `/etc/hosts` pins `raw.githubusercontent.com` to `185.199.110.133`.
- The kernel command line sets `cfg80211.ieee80211_regdom=CN`.
- `jupyter-lab` listens on `0.0.0.0:8888` and `wayvnc` on 5900.
- SSH accepts password login and `PermitRootLogin without-password`; the
  factory `pi` password was not changed.
- `motion.service` is enabled but inactive; `ModemManager` is active.
- `/etc/udev/rules.d/video.rules` (mode `0777` `video0`/`camera_usb`/
  `camera_depth` aliases) is still installed; the platform selects the Astra
  by serial, not by those names.
- The home directory's factory demos and `~/Rosmaster` (see
  [Prior state and wipe](#prior-state-and-wipe)).
- In the Humble image, `/etc/systemd/system/my_ros_service.service`,
  `/root/run_handle.sh` (mode 0777) and the Supervisor configs `chassis.conf`,
  `chassis.config` and `ip.conf` (`autostart=true`) are present and inactive,
  because the container's PID 1 is `bash`. `/root/auto_start.sh` does not
  exist.

## Not tested

- The stop path on its own; it ran as part of the shutdowns and logged a
  clean stop.
- Floor trials, and the operator tools (joystick, keyboard, calibration).
- The cause of the front-right start-up lag.
