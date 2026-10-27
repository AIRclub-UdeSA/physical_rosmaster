# `raphael` deployment: 2026-10-13

Third fleet robot, set up from a fresh Yahboom factory Raspberry Pi 5 card by
following [the per-robot setup guide](../docs/setup_guide_ros2_humble_autostart.md)
and [autostart setup](../docs/autostart_setup.md). The guide was rewritten for
factory cards during this session (PR #50); the workspace was built from that
revision.

Per the runbook, this did not repeat the one-time platform validation done on
`x3-c`: no motor live-loss probe, no active-link-loss test, no consumer-parity
run. It is a per-unit deployment and smoke check only.

## Result

Pass on a warm reboot; one cold boot did not produce the boot-ready signal.

- With the wheels lifted, `tools/physical_contract_probe.py` passed on the
  first try with the default five samples (all seven nodes up).
- Started by hand, `rosmaster-platform-ready.service` failed once on the point
  cloud rate floor (`/cam_1/depth/color/points: measured 2.31 Hz`), then
  passed on the one allowed rerun. The operator saw and heard the signal.
- After both units were enabled, one unattended warm reboot produced a single
  complete run of the ready service: `Physical contract PASSED`, then
  `Sent boot-ready buzzer + RGBLight`, then `Deactivated successfully`. The
  operator saw and heard the boot signal.
- One cold boot (clean `poweroff`, power switch off, back on) brought the
  platform up, but the ready service failed the contract and sent no signal.
  It was not retried because the battery was low. See
  [Cold boot](#cold-boot).

## Deployed revision

| Item | Value |
| --- | --- |
| `physical_rosmaster` | `main` at `aebad554e1fb95ce6dec0d9c79bdd155768777cb`, clean, equal to `origin/main` |
| `ros2_astra_camera` | `f7e71d9ce806e788cb48d8580aac2c778fba4214`, clean |
| Workspace | exactly the eight local packages plus `astra_camera` and `astra_camera_msgs`; `colcon build --symlink-install` finished in 3 min 1 s |
| `Rosmaster_Lib` | V3.3.9, SHA256 `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`; `tools/rosmaster_lib_probe.py --hash-only` exits 0 |

## Host and container

| Item | Value |
| --- | --- |
| Hostname | `raphael` (`raphael.local`) |
| `ROS_DOMAIN_ID` | 10 (`x3-c` uses 11, `mikey` uses 13) |
| Timezone | `America/Argentina/Buenos_Aires` |
| Hardware / OS | Raspberry Pi 5 Model B Rev 1.0, Debian 12, kernel `6.6.62+rpt-rpi-2712` |
| Image | `yahboomtechnology/ros-humble:4.1.2`, digest `sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3`, image ID `dedc515c326a` |
| Disk | root 36 GB at 96% before expansion; 57 GB after; 59% used with the Humble image and workspace |

Container command:

```bash
docker run -dit --name rosmaster_humble --net=host --privileged -v /dev:/dev \
  --restart unless-stopped -e ROS_DOMAIN_ID=10 -e DISPLAY=:0 \
  yahboomtechnology/ros-humble:4.1.2 bash
```

`Rosmaster_Lib.py` was copied from the host's
`/home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py` to
`/root/Rosmaster_Lib/Rosmaster_Lib.py` in the container and exposed through
`/usr/local/lib/python3.10/dist-packages/rosmaster_lib.pth`. `python3-serial`
and `python3-vcstool` were installed after the expired ROS apt key was replaced
with `ros2-apt-source` 1.2.0, as the guide describes.

Unlike `mikey`, this card shipped with **no** Humble image and no container,
only the three `ros-foxy` images; the full pull was needed.

### Container `.bashrc`

The image's `/root/.bashrc` sets `export ROS_DOMAIN_ID=20` and sources the
factory `~/imu_ws`, `~/gmapping_ws` and `~/yahboomcar_ws` overlays.
Non-interactive `bash -lc` (what the autostart scripts use) returns before
those lines and keeps the container's domain, but an interactive
`docker exec -it rosmaster_humble bash` (or a Dev Containers attach) ended up
on domain 20 with the factory overlays. With the owner's approval the original
was saved to `/root/rosmaster-recovery-backups/bashrc.factory-2026-10-13` and
those lines were commented out; the workspace is now sourced after
`/opt/ros/humble`, only if it exists. Interactive shells now report domain 10.

The exact factory lines were not captured in this record, so the setup guide's
`sed` matches by pattern (any `export ROS_DOMAIN_ID=20` line, and any line
sourcing those three overlays via `~` or `/root`) instead of quoting them. The
original file is still on the robot in `/root/rosmaster-recovery-backups/`.
After the edit the check prints `AMENT_PREFIX_PATH` as well as the domain, to
show the `imu_ws` and `gmapping_ws` overlays are gone.

## Device identities

| Device | IDs | Identity |
| --- | --- | --- |
| Motor controller | CH340 `1a86:7523` | no serial; bound by physical port `KERNELS=="1-2.2"` (VIA Labs hub `2109:2817` on the Yahboom board, port 2, on Pi root port `1-2`) |
| LiDAR | CP2102 `10c4:ea60` | USB serial `0001`, topology `3-2` |
| Astra depth | `2bc5:060f` | OpenNI serial `ACRC64300CP` |
| Astra UVC/RGB | `2bc5:050f` | UVC serial `SN0001` |

The motor's topology is `1-2.2`, not the `1-1.2` recorded on `x3-c` and
`mikey`: the Yahboom board's USB cable is in the other host port. The Astra's
internal hub (`05e3:0608`) is on Pi root port `3-1`, not on the Yahboom hub.
The operator chose to keep this layout. The Astra came up on both warm reboots
and on the one cold boot.

The installed host rules are in
[99-rosmaster-x3.raphael.rules](99-rosmaster-x3.raphael.rules), plus the pinned
`ros2_astra_camera` Orbbec rule (SHA256
`fd5bcd51b7b6bb7115868d210c68c0ef87d1706580d6d8bb2a777fbdb0590bcf`), which
replaced the factory one (`d853072e…`). The factory `usb.rules` and
`myserial.rules` were moved to `/etc/udev/rules.disabled/`; the original
directory is backed up under
`/var/backups/rosmaster-recovery/2026-10-13-udev-preinstall/`.

After a reboot: `/dev/robot/motor` and `/dev/robot/lidar` are `0660 root:dialout`;
`/dev/astradepth` and `/dev/astrauvc` are `0666 root:video`; the legacy aliases
are gone; the same aliases are visible inside the container. The motor alias
depends on the board's USB cable staying in the same Pi port.

`/etc/rosmaster/platform.env` (not committed) sets
`ROSMASTER_MOTOR_PORT=/dev/robot/motor`, `ROSMASTER_LIDAR_PORT=/dev/robot/lidar`,
and `ROSMASTER_ASTRA_SERIAL=ACRC64300CP`, with the template defaults for the
container name, workspace and boot-ready RGB effects.

## Smoke-check pulses

Wheels lifted, battery about 10.5 V, one-second `tools/safe_cmd_vel_pulse.py`
pulses. Values are the change in each wheel's `/joint_states` position (rad)
and the change in `/odom`:

| Command | FL | FR | BL | BR | `/odom` | Signs vs. [odometry_validation.md](../docs/odometry_validation.md) |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| `x=0.10` | +0.90 | +1.24 | +1.07 | +0.62 | x +0.031 | match |
| `y=0.10` | -1.52 | +1.04 | +1.15 | -0.006 | y +0.030, yaw +0.063 | BR did not register |
| `yaw=0.60` | -1.49 | +1.35 | -1.23 | +0.68 | yaw +0.23 | match |
| `y=0.10` (repeat, traced) | -1.56 | +1.26 | +1.06 | -0.63 | | match |

The operator saw the back-right wheel turn during the first strafe pulse, but
its encoder moved only 0.006 rad over the one-second window. The repeat was
recorded continuously from `/joint_states`: every wheel starts at a different
time and they all stop together. FL started at 3.38 s, FR at 3.88 s, BL at
3.98 s and BR at 4.08 s, about 0.7 s after FL, and all stopped at about
4.4-4.5 s. BR is the slowest wheel to start and, in the pulses where it did
turn, moved about half as far as the others (0.62 and 0.68 rad). In the first
strafe it also had to reverse from the previous forward pulse. This is the
low-speed behavior already recorded for `x3-c` and `mikey`, here concentrated
on one wheel. It was not diagnosed further (stiction, motor or drag, at about
10.5 V); it is not a wiring fault, since BR's sign is correct in every pulse
where it moved. No tool, gate or driver setting was changed, and
the stronger `y=0.20` pulse was not needed.

## Battery

The controller reported 10.6 V at idle before the smoke check, 10.5 V after
the pulses and through the reboots, and 10.4 V at the end of the session. No
charging was possible during the session, because the robot cannot be
charged while it is on.

## Cold boot

The robot was shut down with `systemctl poweroff` (clean stop: `Clean stop:
all platform processes exited on SIGTERM`, then `Journal stopped`), the power
switch was turned off for about two minutes, and the robot was turned back on
without touching any cable.

- All four USB devices enumerated, including both Astra functions;
  `rosmaster-wait-for-platform` passed and the platform came up with all seven
  nodes.
- `rosmaster-platform-ready.service` failed and sent no signal (monotonic
  boot time):

  ```text
  [24.80] physical_contract_probe: Waiting up to 35.0s for the physical platform contract
  [27.42] astra_camera_node: Device connected: Astra serial number: ACRC64300CP
  [33.40] Physical contract FAILED: /cam_1/depth/image_raw: measured 0.71 Hz outside 3.0..40.0 Hz
  ```

  The probe judges a topic on the first five messages after it subscribes, and
  it measures the rate from their header stamps. It judged the depth stream
  six seconds after the camera connected, while the first depth frames were
  arriving about 1.4 s apart. Afterwards the platform was healthy: depth at
  30 Hz, `/joint_states` and `/odom` at 10 Hz, `/scan` at 7.4 Hz. On the warm
  reboot the camera connected at 25.6 s and the probe passed at 27.8 s.
  `Restart=no` means a slow start of a few frames costs the boot-ready signal
  even though the graph recovers by itself.
- This is not the known
  [cold-boot enumeration failure](../docs/troubleshooting/known_issues/astra-depth-cold-boot-enumeration-failure.md):
  the depth function enumerated and the platform started.
- Separately, the Pi 5 has no RTC battery. The clock started from the
  `fake-hwclock` time saved at shutdown, and `systemd-timesyncd`'s first sync at
  34.72 s stepped it forward by about 2 min 16 s, the time the robot was off.
  Right after the step, `base_node` logged `Wheel encoders timed out;
  integration stopped (no vel_raw fallback)`. It is a false timeout caused by
  the step, and it recovered by itself: the next encoder message rebases the
  input, and the diagnostic reported `Wheel encoder odometry healthy`
  afterwards. The step came 15 ms after the probe had already failed, so it is
  not the cause of that failure. A step during a probe's sampling window could
  still affect the header-stamp rates; that was not observed.

The journal for this boot is saved on the robot at
`/home/pi/rosmaster-evidence/2026-10-13-coldboot1/`. No gate, timeout, unit or
launch setting was changed. Whether this repeats on every cold boot is
untested.

## Factory items left in place

- `/etc/hosts` pins `raw.githubusercontent.com` to `185.199.110.133`.
- The kernel command line sets `cfg80211.ieee80211_regdom=CN`.
- `jupyter-lab` listens on `0.0.0.0:8888` and `wayvnc` on 5900; both are
  enabled.
- SSH accepts password login and `PermitRootLogin without-password`; the
  factory `pi` password was not changed.
- `motion.service` (webcam daemon, stream port 8081 not restricted to
  localhost) is enabled but was inactive; if it started it could claim the
  Astra's UVC device.
- `ModemManager` is active; it is known to probe USB serial adapters.
- The Humble image's latent autostarts (`my_ros_service.service`, Supervisor
  configs, `/root/run_handle.sh`) were not inspected; they stay inactive
  because the container's PID 1 is `bash`. `rosdep install` removed the image's
  `systemctl` replacement package while installing `systemd`.

## Stop path

`sudo systemctl stop rosmaster-platform.service` on the running platform
returned in about 6 s. `rosmaster-platform-stop` logged `Clean stop: all
platform processes exited on SIGTERM`; afterwards no platform process was left
in the container, `ros2 node list` was empty, nothing held `/dev/robot/motor`
or `/dev/robot/lidar`, and the container kept running. The same clean stop was
logged on both shutdowns.

The unit then reports `failed`, not `inactive`: its main process
(`docker exec`) exits with status 143 on SIGTERM, which systemd counts as a
failure because the unit does not list it in `SuccessExitStatus`. The stop
itself is clean; only the reported state is misleading.

## Not tested

- A second cold boot, to tell whether the ready-service failure repeats.
- Floor trials, and the operator tools (joystick, keyboard, calibration).
