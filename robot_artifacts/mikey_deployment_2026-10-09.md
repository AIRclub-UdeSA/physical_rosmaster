# `mikey` deployment: 2026-10-09

Second fleet robot, set up from a fresh Yahboom factory Raspberry Pi 5 card by
following [the per-robot setup guide](../docs/setup_guide_ros2_humble_autostart.md)
and [autostart setup](../docs/autostart_setup.md). This closes the last open
acceptance item of issue #24 (an additional fleet robot deployed using the
runbook).

Per the runbook, this did not repeat the one-time platform validation done on
`x3-c`: no motor live-loss probe, no active-link-loss test, no consumer-parity
run. It is a per-unit deployment and smoke check only.

## Result

Pass. With the wheels lifted:

- `tools/physical_contract_probe.py` passed on the first try with the default
  five samples (all seven nodes up);
- after `rosmaster-platform.service` and `rosmaster-platform-ready.service`
  were enabled, one unattended warm reboot produced a single complete run of
  the ready service: `Physical contract PASSED`, then `Sent boot-ready buzzer +
  RGBLight`, then `Deactivated successfully`. The operator saw and heard the
  boot signal.

## Deployed revision

| Item | Value |
| --- | --- |
| `physical_rosmaster` | `main` at `52bcb0a503295588857ef1652db39b96d9200af0`, clean, equal to `origin/main` |
| `ros2_astra_camera` | `f7e71d9ce806e788cb48d8580aac2c778fba4214`, clean |
| Workspace | exactly the eight local packages plus `astra_camera` and `astra_camera_msgs`; `colcon build --symlink-install` finished in 2 min 53 s |
| `Rosmaster_Lib` | V3.3.9, SHA256 `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`; `tools/rosmaster_lib_probe.py --hash-only` exits 0 |

## Host and container

| Item | Value |
| --- | --- |
| Hostname | `mikey` (`mikey.local`) |
| `ROS_DOMAIN_ID` | 13 (`x3-c` uses 11) |
| Timezone | `America/Argentina/Buenos_Aires` |
| Hardware / OS | Raspberry Pi 5 Model B Rev 1.0, Debian 12, kernel `6.6.62+rpt-rpi-2712` |
| Image | `yahboomtechnology/ros-humble:4.1.2`, digest `sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3` |

Container command:

```bash
docker run -dit --name rosmaster_humble --net=host --privileged -v /dev:/dev \
  --restart unless-stopped -e ROS_DOMAIN_ID=13 -e DISPLAY=:0 \
  yahboomtechnology/ros-humble:4.1.2 bash
```

`Rosmaster_Lib.py` was copied from the host's
`/home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py` to
`/root/Rosmaster_Lib/Rosmaster_Lib.py` in the container and exposed to the
container's Python 3.10 through `/usr/local/lib/python3.10/dist-packages/rosmaster_lib.pth`
(contents: `/root/Rosmaster_Lib`). `python3-serial` and `python3-vcstool` were
installed.

The factory card also needed, before the runbook's steps could succeed:
unique SSH host keys and `machine-id`, root-partition expansion (36 GB of a
58 GB card, 96% full), removal of the unused `ros-foxy` images, moving the
image's factory `/root/yahboomcar_ws` aside, replacing the expired ROS apt
signing key with `ros2-apt-source` 1.2.0, and disabling the factory
`~/.config/autostart/rosmaster.desktop` (which starts a motion-capable
`rosmaster_main.py` that owns the motor controller).

## Device identities

| Device | IDs | Identity |
| --- | --- | --- |
| Motor controller | CH340 `1a86:7523` | no serial; bound by physical port `KERNELS=="1-1.2"` (the same topology string as `x3-c`) |
| LiDAR | CP2102 `10c4:ea60` | USB serial `0001`, topology `3-2` |
| Astra depth | `2bc5:060f` | OpenNI serial `ACRC64300GM` |
| Astra UVC/RGB | `2bc5:050f` | |

The Astra's internal hub (`05e3:0608`) is on Pi root port `1-2`, not on the
Yahboom hub.

The installed host rules are in
[99-rosmaster-x3.mikey.rules](99-rosmaster-x3.mikey.rules), plus the pinned
`ros2_astra_camera` Orbbec rule (SHA256
`fd5bcd51b7b6bb7115868d210c68c0ef87d1706580d6d8bb2a777fbdb0590bcf`). The
factory `56-orbbec-usb.rules` that it replaced differs by one line
(`0557` → `dabai_dc1_rgb`). The factory `usb.rules` and `myserial.rules`
(`MODE:="0777"`, legacy `/dev/myserial` and `/dev/rplidar`) were moved to
`/etc/udev/rules.disabled/`.

After a reboot: `/dev/robot/motor` and `/dev/robot/lidar` are `0660 root:dialout`;
`/dev/astradepth` and `/dev/astrauvc` are `0666 root:video`; the legacy aliases
are gone; the same aliases are visible inside the container. The motor alias
depends on the controller staying in the same physical port.

`/etc/rosmaster/platform.env` (not committed) sets
`ROSMASTER_MOTOR_PORT=/dev/robot/motor`, `ROSMASTER_LIDAR_PORT=/dev/robot/lidar`,
and `ROSMASTER_ASTRA_SERIAL=ACRC64300GM`, with the template defaults for the
container name, workspace and boot-ready RGB effects.

## Smoke-check pulses

Wheels lifted, battery about 10.6 V, one-second `tools/safe_cmd_vel_pulse.py`
pulses. Values are the change in each wheel's `/joint_states` position (rad)
and the change in `/odom`:

| Command | FL | FR | BL | BR | `/odom` | Signs vs. [odometry_validation.md](../docs/odometry_validation.md) |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| `x=0.10` | +1.60 | +1.03 | +2.16 | +1.64 | x +0.052 | match |
| `y=0.10` | -1.69 | +0.92 | +2.31 | -1.81 | y +0.055 | match |
| `yaw=0.30` | 0 | 0 | -0.59 | 0 | yaw +0.029 | only BL responded |
| `yaw=0.60` | -1.77 | +1.23 | -2.30 | +1.77 | yaw +0.35 | match |
| `yaw=0.30` (repeat) | 0 | +0.006 | -0.012 | 0 | about 0 | no response |

Forward and strafe wheel signs and `/odom` directions match. The 0.30 rad/s
rotation does not drive the wheels reliably; 0.60 rad/s does, with all four
signs correct. This is the low-speed stiction threshold already recorded for
`x3-c` (see Section 6 of [robot_side_next_moves.md](../docs/robot_side_next_moves.md),
which uses `0.60 rad/s`), not a new fault. At 0.30 rad/s here the back-left
wheel moved once while the others did not, where `x3-c` showed the front-right
wheel alone responding. No bag was recorded.

## Battery

The controller reported 10.7 V at idle at the start and 10.5 V at the end of
the session. `x3-c`'s almost-full idle baseline was about 11.3 V
([pr3_recovery_session_2026-08-30.md](pr3_recovery_session_2026-08-30.md)); the smoke
check and pulses were run at the lower voltage with the operator's approval.

## Not tested

- A cold power-cycle boot. Only warm reboots were run, and the Astra depth
  sensor's cold-boot enumeration failure is a known issue
  ([the known-issue page](../docs/troubleshooting/known_issues/astra-depth-cold-boot-enumeration-failure.md)).
- The stop path (`systemctl stop` and `rosmaster-platform-stop`).
- Whether the journal survives physical power removal.
- Floor trials, and the operator tools (joystick, keyboard, calibration).
