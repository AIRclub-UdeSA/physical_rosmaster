# `raphael` on the competition router: 2026-10-16

Whether the router the club will take to the competition carries normal use of
a robot from a workstation: RViz with the camera, bags recorded on the
workstation, and teleoperation. Compared with the faculty network (`Privada`),
where a workstation could not record a bag from the robot.

Run with the scripts now in [tools/wifi_test](../tools/wifi_test/README.md)
(an earlier Spanish draft of the same commands). Raw evidence:
[raphael_router_wifi_2026-10-16/](raphael_router_wifi_2026-10-16/).
Only the probe results and the bag message counts can be checked against those
files. The `iperf3`, ping, multicast and workstation-rate results in
[Link](#link), the signal and bitrate in [Setup](#setup), the bag sizes in
[Bags](#bags), the ping column in [Load steps](#load-steps) and the
[Commands](#commands) table were transcribed by hand from the scripts' console
output, which was not kept.

## Result

The router carries the camera image, bags with the image, and teleop. The
point cloud does not: with the image and the cloud open in a remote RViz, the
robot's WiFi saturates and slows every consumer, including those on the robot.

- Robot to workstation: **91 Mbit/s** on the router, 32.6 on `Privada`.
- Color image on the workstation at 29.9 Hz; both 60 s bags, including the
  one with the image, arrived at 99-100% of what the robot recorded.
- Image plus cloud in RViz: the camera on the robot dropped from 30 to 17.8 Hz
  with ~150 ms latency, ping peaked at 1.5 s, and `/cmd_vel` took ~0.5 s longer
  to act. Nothing crashed.
- A network change under a running platform left 0 discoverable nodes until
  the platform was restarted:
  [known issue](../docs/troubleshooting/known_issues/network-change-leaves-nodes-undiscoverable.md).

## Setup

| Item | Value |
| --- | --- |
| Robot | `raphael`, `ROS_DOMAIN_ID` 10, `physical_rosmaster` `main` at `aebad55`, platform as in its [deployment record](raphael_deployment_2026-10-13.md) |
| Router | 5 GHz, channel 36, WPA2, no internet, client isolation off |
| Robot link | -54 to -62 dBm, tx bitrate 390-433 Mbit/s, WiFi power save off, 2-5 m from the router, stationary |
| `Privada` link (before) | 2.4 GHz channel 1, -50 dBm, 72.2 Mbit/s, power save **on** |
| Workstation | Ubuntu 22.04, ROS 2 Humble, on the same router over WiFi |
| Battery | 10.7 V before, 10.5 V during, **9.8 V** at the end |

## Link

| Check | Result |
| --- | --- |
| `iperf3` robot → workstation (`-R`, TCP, 10 s) | 91.0 Mbit/s |
| `iperf3` workstation → robot (TCP, 10 s) | 19.8 Mbit/s |
| `iperf3 -R -u -b 60M` | 60.0 Mbit/s, 0 of 51,772 datagrams lost |
| Ping, 50 packets | avg 9.2 ms, max 64 ms, 1 lost |
| `ros2 multicast` | received in both directions |
| From the workstation | 7 nodes; `/odom` 9.75 Hz, `/scan` 7.34 Hz, `/cam_1/color/image_raw` 29.87 Hz at 6.97 MB/s, `/cam_1/depth/color/points` 20.4 Hz |

The workstation-to-robot direction was a fifth of the other. It only carries
commands here, so it was not investigated.

## Load steps

Each step added RViz displays on the workstation, then measured the camera on
the robot for 30 s (`sensor_capability_probe.py`, so the latency is on the
robot's own clock) with a ping running alongside. Rate in Hz, latency median /
p95 in ms:

| Step | RViz on the workstation | Color image | Depth image | Cloud | Ping avg / max |
| --- | --- | --- | --- | --- | --- |
| A | none | 30.0, 4 / 13 | 30.0, 5 / 5 | 12.2, 51 / 53 | (link check) |
| C | RobotModel, TF, LaserScan | 30.0, 6 / 7 | 29.7, 3 / 13 | 8.1, 43 / 48 | 6 / 26 |
| D | + Image | 29.2, 8 / 17 | 30.1, 5 / 6 | 7.0, 49 / 59 | 12 / 58 |
| E | + PointCloud2 | **17.8, 162 / 182** | **17.8, 151 / 168** | 16.4, **266 / 349** | **132 / 1537** |

No packet was lost in C, D or E. In E the cloud rate rose while every other
stream fell; the cause was not isolated. The LaserScan display received nothing
until its reliability was set to Best Effort.

In 2026-09 a remote RViz on the cloud raised its on-robot latency from ~50 to
343 ms on `x3-c` on the faculty network
([sensor capabilities](../docs/sensor_capabilities.md#a-remote-rviz-delays-every-subscriber)).
The router shows the same effect, somewhat smaller.

## Bags

60 s, recorded at the same time on the workstation (over WiFi, mcap) and in the
robot's container (local), RViz as in step D:

| Bag | Topics | Workstation / robot |
| --- | --- | --- |
| Light | `/tf`, `/tf_static`, `/odom`, `/scan`, `/imu/data`, `/joint_states`, `/voltage` | 99-101% on every topic; 5 MB |
| Heavy | light + `/cam_1/color/image_raw`, `/cam_1/color/camera_info` | 99-100% on every topic; image 1,741 of 1,748; 389 MB |

## Commands

`cmd_latency.py`, wheels lifted: five 1.5 s pulses at 0.10 m/s, timed on the
workstation from the command to `/odom` showing motion (start), and from the
zero command to `/odom` at rest (stop). Includes WiFi both ways, the 10 Hz
`/odom` period, and wheel stiction (`raphael`'s back-right wheel is the slowest
to start, per its deployment record).

| RViz as in | Start median / max | Stop median / max |
| --- | --- | --- |
| D | 716 / 768 ms | 321 / 332 ms |
| E | 1186 / 1237 ms | 692 / 900 ms |

These came from `cmd_latency.py` before it checked that `/odom` was fresh, and
step E had a 1.5 s ping spike, so rerun the tool before quoting them.

The operator then drove the lifted robot with `teleop_twist_keyboard`
(0.1 m/s): responsive with the image only, noticeably delayed with the cloud.

## Other findings

- **Astra depth missing at the first boot of the day.** `rosmaster-wait-for-platform`
  timed out on `astra:usb-2bc5` and the platform did not start. Reseating the
  Astra USB fixed it, as in the
  [cold-boot known issue](../docs/troubleshooting/known_issues/astra-depth-cold-boot-enumeration-failure.md).
  `rosmaster-platform-ready.service` runs once per boot, so the later manual
  starts gave no boot-ready buzzer or battery light.
- **Clock step.** Setting the robot clock from the workstation (`date -s`, a
  sub-second step) under the running platform produced a false
  `Wheel encoders timed out` in `base_node`, which recovered on its own. The
  link check now only corrects offsets over 2 s.
- **Two `Privada` profiles.** NetworkManager had `Privada`, saved without any
  security (`requires no security`, then `association took too long`), and
  `Privada 1`, with the password. Only `Privada 1` connects. `Privada` now has
  autoconnect off and was left in place.
- **Bag recorder hang.** `ros2 bag record` under `timeout` from an interactive
  terminal stopped before creating the bag: its keyboard controls read the
  terminal from a background process group. `bag_compare.sh` gives it
  `/dev/null` as stdin.

## Left on the robot

- Connection `router-test` for the router: autoconnect, priority 5, WiFi power
  save off. `Privada 1` has priority 10, so the robot prefers `Privada` where
  both exist and joins the router on its own where only the router exists.
- `iperf3` installed on the host, daemon disabled.
- Shut down with `systemctl poweroff` at 9.8 V.

## Not tested

- Cutting the workstation's WiFi while driving (the driver's 0.5 s
  `cmd_vel_timeout` should stop the wheels).
- Router reboot or brief signal loss with the robot keeping its address.
- Distance beyond ~5 m, walls, and a moving robot.
- Several robots and workstations on the router at once.
- A smaller cloud (`ROSMASTER_CLOUD_DECIMATION=2`) over WiFi.
