# WiFi test

Checks whether a network can carry what teams actually do with a robot: RViz
with the camera, a bag recorded on the workstation, and teleoperation. Written
for the router the club takes to competitions, which has no internet. First
run: [raphael, 2026-10-16](../../robot_artifacts/raphael_router_wifi_2026-10-16.md).

Every script runs on the workstation (ROS 2 Humble) and reaches the robot over
SSH with a key (`ssh-copy-id pi@<robot>` once). Results go to
`~/wifi_test_results` (override with `RESULTS=`). `ROBOT` and `ROS_DOMAIN_ID`
have no default; export both for the robot under test (for example
`export ROBOT=raphael.local ROS_DOMAIN_ID=10`). `cmd_latency.sh` moves whatever
robot is on that domain, whatever `ROBOT` says.

## What the sensors need

| Topic | Size x rate | Approx. |
|---|---|---|
| `/cam_1/color/image_raw` (320x240 `rgb8`) | 230 KB x 30 Hz | 55 Mbit/s |
| `/cam_1/depth/image_raw` (320x240 `32FC1`) | 307 KB x 30 Hz | 74 Mbit/s |
| `/cam_1/depth/color/points` | varies, 7-20 Hz | tens of Mbit/s |
| `/scan`, `/odom`, `/tf`, `/imu/data` | small | < 1 Mbit/s |

The Raspberry Pi 5 has a 1x1 802.11ac radio: on 5 GHz expect 90-250 Mbit/s of
real throughput, on 2.4 GHz 30-70. A router's "1 Gbit" label does not change
that; the robot's radio is the limit. Use 5 GHz.

## Before the test (with internet)

Workstation:

```bash
sudo apt install iperf3 ros-humble-rosbag2-storage-mcap ros-humble-teleop-twist-keyboard
sudo ufw status   # if active, DDS (UDP) is blocked: disable it for the test
```

Robot host: `sudo apt install iperf3` (answer **No** to starting it as a daemon).

Router:

- 5 GHz on a fixed channel in **36-48 or 149-161**. The factory image boots
  with `cfg80211.ieee80211_regdom=CN` and the radio reports `US`; DFS channels
  (52-144) may be invisible to the robot or slow to join.
- Client/AP isolation **off**, or robot and workstation cannot see each other.
- Note the admin page address and login: without internet you cannot look
  them up.

## Put the robot on the router

Add the network as a second connection while the robot is still reachable on
its usual one. Never commit the network name or password.

```bash
ssh -t pi@<robot> 'sudo nmcli --ask connection add type wifi ifname wlan0 con-name router-test \
  ssid "<SSID>" wifi-sec.key-mgmt wpa-psk \
  connection.autoconnect yes connection.autoconnect-priority 5 \
  802-11-wireless.powersave 2'
```

`--ask` prompts for the password, so it stays out of the shell history on both
machines.

`powersave 2` turns WiFi power saving off; with it on, the Pi adds 100-300 ms
ping spikes. Give the usual network a higher priority so the robot keeps
preferring it where both exist. Where only the router exists, the robot joins
it on its own at boot. To move a running robot between saved connections:

```bash
bash switch_network.sh <robot> router-test
```

It also restarts the platform: nodes started on one network are not
discoverable after the robot changes network
([known issue](../../docs/troubleshooting/known_issues/network-change-leaves-nodes-undiscoverable.md)).
SSH drops while it runs; move the workstation to the same network and `ping`
the robot.

Check `nmcli` profile names before relying on them: `raphael` had two profiles
for its usual network, and one had been saved without a password and could
never connect.

## Run order

| Step | Command | Look at |
|---|---|---|
| Link | `bash link_check.sh` | throughput (`-R` is the sensor direction), multicast both ways, 7 nodes, image ~30 Hz on the workstation; ends with step A |
| C | RViz: RobotModel, TF, LaserScan → `bash load_step.sh C` | |
| D | + Image → `bash load_step.sh D` | |
| E | + PointCloud2 → `bash load_step.sh E` | |
| Bags | RViz as in D → `bash bag_compare.sh light`, `bash bag_compare.sh heavy` | workstation >= 90% of the robot per topic |
| Commands | wheels lifted, RViz as in D, then E → `WHEELS_LIFTED=yes bash cmd_latency.sh D`, `WHEELS_LIFTED=yes bash cmd_latency.sh E` | start/stop delay; never `DID NOT STOP` |

`load_step.sh` measures on the robot (`sensor_capability_probe.py`), so the
latency does not depend on the workstation clock: what it shows is whether
remote subscribers slow the robot down for everyone. In RViz, set every
camera, cloud and LaserScan display to **Best Effort**, depth 1; `/scan` is
best effort and a Reliable display receives nothing. `link_check.sh N`
resumes at step N.

## If something fails

- **Ping works, 0 nodes, even from inside the robot's container:** the robot
  changed network after the platform started. Restart
  `rosmaster-platform.service`, wait ~50 s, `ros2 daemon stop` on the
  workstation.
- **Topic names visible, no data:** workstation not on Humble, firewall on, or
  client isolation on the router.
- **Small topics arrive, camera at 1-2 Hz:** bandwidth. Check `iperf3 -R` and
  the signal; drop displays you don't need.
- **Multicast fails in either direction:** the router filters it. Turn off
  client isolation, then IGMP snooping / multicast-to-unicast.
- **Platform not up after a cold boot:** check `lsusb -d 2bc5:060f`; see
  [the Astra cold-boot issue](../../docs/troubleshooting/known_issues/astra-depth-cold-boot-enumeration-failure.md).
- **Clock:** the Pi has no RTC and the router no internet. `link_check.sh`
  corrects offsets over 2 s; stepping the clock under a running platform makes
  `base_node` log a false `Wheel encoders timed out`, which recovers.
