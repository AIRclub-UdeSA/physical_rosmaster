# ROS 2 Humble X3 setup

Despite the historical filename, this document does not install or enable
autostart — see [autostart_setup.md](autostart_setup.md) for that, once this
guide's checks pass. The old routine targeted the former EKF/application tree
and unstable `/dev/ttyUSB*`/`/dev/video*` names. It must not be reused.

This is the fast path for bringing up an *additional* X3 running the
already-accepted `main` platform code, starting from a fresh Yahboom factory
card. It covers only what's genuinely per-robot: giving the unit its own
identity, preparing the host and container, cloning and building, identifying
this unit's hardware, and one smoke check that it's wired correctly. It
deliberately does **not** repeat the one-time platform validation that `x3-c`
already went through — the motor live-loss protocol proof, the active-link-loss
characteristic, and the simulator/physical consumer-parity proof all test the
*code*, not the unit, and don't need re-running per robot. See
[robot_side_verification_todo.md](robot_side_verification_todo.md) and
[robot_side_next_moves.md](robot_side_next_moves.md) if you want that history;
neither is required reading to bring up a new robot.

Work through the sections in order on a factory card. If a unit was already
partly set up, each section starts with a check so you can skip what is done.
`x3-c`, `mikey`, `raphael` and `leo` were all set up this way; the `x3-c` rebuild is recorded in
[robot_artifacts/pr3_recovery_session_2026-08-30.md](../robot_artifacts/pr3_recovery_session_2026-08-30.md).

## 1. Preconditions

- The robot is a ROSMASTER X3; X1 and R2 are unsupported.
- It is a Yahboom factory-image Raspberry Pi 5 (Debian 12) with Docker and
  `Rosmaster_Lib` V3.3.9 installed on the host
  (`/home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py`).
- You can SSH to it **by IP address**. Every factory card has the hostname
  `yahboom`, so `yahboom.local` can answer for any robot on the network
  (including a working one). Don't use it until the unit has its own name
  (section 2).
- The host exposes the motor controller, A1 LiDAR, and Astra USB devices to the
  container. They don't need to be plugged in until section 8; the LiDAR and
  camera can stay disconnected to save battery until then.
- Existing source and generated-state backups are outside
  `/root/yahboomcar_ws`.

If an old service is active, stop it using the robot's existing administration
procedure before opening serial devices manually. Section 4 covers the factory
services that are known to do this.

Inspect storage before cloning, building, or changing Docker state:

```bash
df -h /
docker system df
docker ps -a --size
```

Do not use aggressive pruning as the first response to low disk space. An
active container or image can consume most of the disk while Docker reports no
reclaimable space, and `docker system prune -a` does not shrink an active
container's writable layer. Diagnose the actual consumer first; see
[Root filesystem full causing LightDM login loop and Docker growth](troubleshooting/known_issues/root-filesystem-full-login-loop.md).

## 2. Give the robot its own identity

Every card flashed from the factory image has the same hostname (`yahboom`),
the same SSH host keys, and the same `/etc/machine-id` as every other card.
Duplicate machine-ids can cause DHCP address clashes between robots, and shared
host keys mean one robot's key identifies all of them. Set a unique identity
before anything else:

```bash
# Pick a name nobody else uses (the fleet so far: x3-c, mikey, raphael, leo).
sudo hostnamectl set-hostname <name>
sudo sed -i 's/^127\.0\.1\.1\([[:space:]]\+\)yahboom$/127.0.1.1\1<name>/' /etc/hosts

sudo rm /etc/ssh/ssh_host_*
sudo dpkg-reconfigure openssh-server

sudo rm /etc/machine-id
sudo systemd-machine-id-setup
ls -l /var/lib/dbus/machine-id   # a symlink to /etc/machine-id; nothing more to do
```

A `sudo: unable to resolve host` message between `hostnamectl` and the
`/etc/hosts` edit is harmless. Optionally set the timezone too; the factory
default is `Asia/Shanghai`:

```bash
sudo timedatectl set-timezone America/Argentina/Buenos_Aires
```

Also pick this robot's `ROS_DOMAIN_ID` now, so two robots on the same network
don't share a graph (`raphael` uses 10, `x3-c` 11, `leo` 12, `mikey` 13). You need it in
section 5.

The factory `/etc/hosts` also pins `raw.githubusercontent.com` to
`185.199.110.133`. That line can go stale if GitHub changes address; it is a
factory setting, left in place here.

Regenerating the host keys restarts `sshd`, so the next SSH connection from
your workstation fails with `REMOTE HOST IDENTIFICATION HAS CHANGED`. That is
expected, and it also means a `sudo reboot` sent over that connection never
runs. Read the new fingerprint on the robot, then update only this robot's
entry on the workstation:

```bash
# On the robot:
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub

# On the workstation (never touch another robot's entry):
ssh-keygen -R <robot-ip>
ssh-keyscan -t ed25519 <robot-ip> | ssh-keygen -lf -      # must match the fingerprint above
ssh -o StrictHostKeyChecking=accept-new pi@<robot-ip> hostname
```

Then reboot. Afterwards `<name>.local` should resolve from the workstation; it
needs its own `known_hosts` entry (check its fingerprint the same way). Keep
using the IP for the rest of the setup.

## 3. Expand the disk

The factory root partition is about 36 GB on a 58 GB card, and about 96% full.
Check, expand, and reboot:

```bash
df -h /
sudo raspi-config nonint do_expand_rootfs
sudo reboot
# after the reboot:
df -h /          # about 57G
```

The factory image also carries three unused Foxy images (about 20 GB). Once
`docker ps -a` shows no container using them, remove them by name:

```bash
docker images
docker rmi yahboomtechnology/ros-foxy:4.0.6 yahboomtechnology/ros-foxy:4.0.7 yahboomtechnology/ros-foxy:4.0.8
```

## 4. Disable the factory actuator owners

The factory image starts programs that compete with this platform for the
motor controller. Remove them before the container exists.

**Host autostart.** `~/.config/autostart/rosmaster.desktop` runs
`rosmaster_main.py`, which opens the motor CH340 and listens on LAN TCP ports
6000 and 6500. Check for it, stop it, and move the `.desktop` file out of the
autostart directory:

```bash
ls -l ~/.config/autostart/
ps aux | grep '[r]osmaster_main'
ss -ltnp | grep -E ':6000|:6500'

kill -INT <pid-of-rosmaster_main.py>     # its handler commands zero motion before closing
mkdir -p ~/.config/autostart-disabled
mv ~/.config/autostart/rosmaster.desktop ~/.config/autostart-disabled/
chmod 0644 ~/.config/autostart-disabled/rosmaster.desktop
```

Verify it is gone: no `rosmaster_main.py` process, nothing listening on 6000 or
6500, and (when the motor controller is plugged in) nothing holding its serial
port. Leave `start.desktop` alone; it starts the OLED display. See
[the x3-c recovery record](../robot_artifacts/pr3_recovery_session_2026-08-30.md#factory-host-actuator-conflict)
for the same finding on `x3-c`.

**Factory autostart inside the Humble image.** The image also contains
`/etc/systemd/system/my_ros_service.service`, Supervisor configs, `/root/run_handle.sh`,
and possibly `/root/auto_start.sh`. They are inactive because the container's
PID 1 is `bash`, not systemd or Supervisor. Never enable them; the only
autostart for this platform is the one in [autostart_setup.md](autostart_setup.md).

**Factory udev rules.** `/etc/udev/rules.d/usb.rules` and `myserial.rules` set
`MODE:="0777"` and create the legacy `/dev/myserial` and `/dev/rplidar` names,
which would override the reviewed permissions in section 8. Back up the whole
directory, then move those two files out of the way:

```bash
sudo mkdir -p /var/backups/rosmaster-recovery/udev-preinstall /etc/udev/rules.disabled
sudo cp -a /etc/udev/rules.d/. /var/backups/rosmaster-recovery/udev-preinstall/
sudo mv /etc/udev/rules.d/usb.rules /etc/udev/rules.d/myserial.rules /etc/udev/rules.disabled/
```

The robot-specific rules are installed in section 8, once the devices are
connected.

## 5. Create the container

Pull the Humble image by digest, tag it, and verify the digest (about 3.6 GB to
download):

```bash
docker pull yahboomtechnology/ros-humble@sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3
docker tag yahboomtechnology/ros-humble@sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3 \
  yahboomtechnology/ros-humble:4.1.2
docker images --digests
```

Create the one long-lived container, with this robot's `ROS_DOMAIN_ID`:

```bash
docker run -dit --name rosmaster_humble --net=host --privileged \
  -v /dev:/dev --restart unless-stopped \
  -e ROS_DOMAIN_ID=<id> -e DISPLAY=:0 \
  yahboomtechnology/ros-humble:4.1.2 bash
```

The container's only persistent process is `bash` (PID 1). The platform runs
inside it through `docker exec`, so a missing `ROS_DOMAIN_ID` here is easy to
miss later; check `docker exec rosmaster_humble printenv ROS_DOMAIN_ID`.

The image's `/root/.bashrc` overrides that value with `export ROS_DOMAIN_ID=20`
and sources the factory `~/imu_ws`, `~/gmapping_ws` and `~/yahboomcar_ws`
overlays. The autostart scripts use non-interactive `bash -lc`, which stops
before those lines, but an interactive `docker exec -it rosmaster_humble bash`
(or a Dev Containers attach) lands on domain 20 and won't see the robot's
topics. Keep a copy, then comment those lines out and source the workspace
after `/opt/ros/humble` instead. The block matches any line that exports
`ROS_DOMAIN_ID=20` or sources one of those overlays (with `~` or `/root`, any
trailing text), keeps the first backup if you rerun it, and doesn't add the
`source` line twice:

```bash
docker exec rosmaster_humble bash -c '
set -e
mkdir -p /root/rosmaster-recovery-backups
b=/root/rosmaster-recovery-backups/bashrc.factory-$(date +%F)
[ -e "$b" ] || cp -a /root/.bashrc "$b"
sed -i -E \
  -e "s,^[[:space:]]*export[[:space:]]+ROS_DOMAIN_ID=20\b.*,# &," \
  -e "s,^[[:space:]]*(source|\.)[[:space:]]+(~|/root)/(imu_ws|gmapping_ws|yahboomcar_ws)/install/setup\.bash.*,# &," \
  /root/.bashrc
line="[ -f /root/yahboomcar_ws/install/setup.bash ] && source /root/yahboomcar_ws/install/setup.bash"
grep -qxF "$line" /root/.bashrc || echo "$line" >> /root/.bashrc
'
docker exec rosmaster_humble bash -ic 'echo "$ROS_DOMAIN_ID"; echo "$AMENT_PREFIX_PATH"'
```

The first line should be this robot's id, not 20. `AMENT_PREFIX_PATH` should
list the `/root/yahboomcar_ws/install/...` packages but no `imu_ws` or
`gmapping_ws`. If a factory line has a form the patterns don't match, `sed`
leaves it alone, so check `grep -nE 'ROS_DOMAIN_ID|_ws' /root/.bashrc` too.

Run every ROS command on the robot inside this container, for example with
`docker exec -it rosmaster_humble bash`. The host has no ROS install, and a
workstation on a different ROS distro or domain will not see the topics.

## 6. Prepare the container: apt, packages, and `Rosmaster_Lib`

The container's ROS Python is 3.10, while the host's is 3.11.

**apt signing key.** The image's ROS apt key (`F42ED6FBAB17C654`) expired on
2025-06-01, so `apt-get update` fails with `EXPKEYSIG`. If it does, back up the
old source and keyring, then install the official `ros2-apt-source` package
after verifying its checksum. Never bypass signature checks:

```bash
docker exec -it rosmaster_humble bash
```

```bash
(
set -eo pipefail

backup=/root/rosmaster-recovery-backups/apt-before-ros2-source-$(date +%F)
mkdir -p "$backup"
cp -a /etc/apt/sources.list.d/ros2-latest.list /etc/apt/trusted.gpg "$backup"/
mv /etc/apt/sources.list.d/ros2-latest.list "$backup"/ros2-latest.list.moved

cd /tmp
curl -fsSL -o ros2-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.jammy_all.deb
echo "767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433  ros2-apt-source.deb" | sha256sum -c -
dpkg -i ros2-apt-source.deb
apt-get update
)
```

**Packages.** `python3-serial` is the dependency `Rosmaster_Lib` needs, and
`python3-vcstool` provides the `vcs` command used in section 7:

```bash
apt-get install -y python3-serial python3-vcstool
```

**`Rosmaster_Lib`.** Copy the robot's own copy of the library from the host
into the container and expose it to the ROS Python. Never vendor, patch, or
download another copy. On the host:

```bash
docker exec rosmaster_humble mkdir -p /root/Rosmaster_Lib
docker cp /home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py \
  rosmaster_humble:/root/Rosmaster_Lib/Rosmaster_Lib.py
```

Inside the container:

```bash
sha256sum /root/Rosmaster_Lib/Rosmaster_Lib.py
# must be e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c

python3 -c 'import site; print(site.getsitepackages())'
echo /root/Rosmaster_Lib > /usr/local/lib/python3.10/dist-packages/rosmaster_lib.pth
python3 -c 'import Rosmaster_Lib; print(Rosmaster_Lib.__file__)'   # /root/Rosmaster_Lib/Rosmaster_Lib.py
```

If `site.getsitepackages()` lists a different `dist-packages` directory, put the
`.pth` file there. The repository's own preflight (section 7) re-checks this
hash.

## 7. Clone and build

The Humble image ships its own populated `/root/yahboomcar_ws` (factory
`yahboomcar_*` source, `build`/`install`/`log`, and a core file). Cloning into
it makes `colcon` fail with duplicate package names. Move it outside the
workspace first, instead of deleting it:

```bash
mkdir -p /root/rosmaster-recovery-backups
test ! -e /root/yahboomcar_ws/src/physical_rosmaster \
  && mv /root/yahboomcar_ws /root/rosmaster-recovery-backups/yahboomcar_ws.factory-$(date +%F)
```

(Skip this if `/root/yahboomcar_ws` doesn't exist yet or is already your clean
workspace.) Then, inside the container:

```bash
(
set -eo pipefail

mkdir -p /root/yahboomcar_ws/src
cd /root/yahboomcar_ws/src
git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

cd physical_rosmaster
git fetch origin
git checkout main
git pull --ff-only
test -z "$(git status --porcelain)"
test "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)"

cd ..
vcs import . < physical_rosmaster/physical_rosmaster.repos
test -z "$(git -C ros2_astra_camera status --porcelain)"
test "$(git -C ros2_astra_camera rev-parse HEAD)" = \
  "f7e71d9ce806e788cb48d8580aac2c778fba4214"

cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
echo BUILD_DONE
)
```

Run this block directly and check that it prints `BUILD_DONE`. Don't pipe it
through `tail` or `head`: the pipe's exit status hides a failed build, which
then looks like success. A build takes about three minutes on the Pi 5.

The manifest pins `ros2_astra_camera`; don't build an arbitrary current
camera-driver branch. The workspace inventory should show exactly eight local
packages plus `astra_camera` and `astra_camera_msgs`:

```bash
cd /root/yahboomcar_ws && source install/setup.bash && colcon list -n
```

Verify the robot-provided motor library with the fail-closed preflight:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

The command must exit `0` and report the supported V3.3.9 digest. If it
returns nonzero or can't import the library, stop and restore the Yahboom
host/container integration (section 6) before continuing — don't vendor an
unknown `Rosmaster_Lib` copy into this repository, and don't accept a mismatch
by eyeballing printed output. The only supported SHA256 is
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`; the driver
checks this itself at runtime too, so this is an early, cheap version of the
same check.

## 8. Identify this robot's hardware

Everything above runs without the LiDAR and the Astra. Plug them in now. The
Astra can fail to enumerate its depth function; plugging it in while the host is
running gives it a fresh connection.

On the host:

```bash
lsusb
ls -l /dev/serial/by-id
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

You should see the motor controller `1a86:7523`, the LiDAR adapter `10c4:ea60`,
and both Astra functions, `2bc5:060f` (depth) and `2bc5:050f` (UVC/RGB). If one
is missing, reseat that cable before continuing.

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 run astra_camera list_devices_node
```

Record exact model, vendor/product IDs, and stable identity for the motor
controller, the Slamtec A1 serial adapter, and the Astra-family RGB-D camera.
This is unavoidably per-robot: USB topology, cable routing, and which hub port
things land on vary unit to unit. If no Orbbec/Astra device appears, stop here
and fix the physical connection before continuing.

Use [../config/99-rosmaster-x3.rules.example](../config/99-rosmaster-x3.rules.example)
as a template. Replace placeholders with this robot's observed values, install
it on the host, reload udev rules, reconnect the devices, and verify the final
aliases. Compare with [x3-c's installed rules](../robot_artifacts/99-rosmaster-x3.x3-c.rules).

**Motor controller.** Some CH340 motor controllers expose no serial. Both `x3-c`
and `mikey` are like this: `/dev/serial/by-id` only shows the generic name
`usb-1a86_USB_Serial-if00-port0`, which is not unique to the unit. For those,
bind `/dev/robot/motor` to a dedicated physical USB port using the documented
`KERNELS` fallback, verify that identity survives a reboot, and tell whoever
operates the robot which physical port that is: the cable must stay there. Use
`/dev/robot/motor` as the motor path in the rest of this guide and in
`platform.env`. Only a controller with a real, unique serial should use its
`/dev/serial/by-id/...` path.

**Orbbec rule.** Replace the factory `56-orbbec-usb.rules` with the pinned
driver's rule (the factory file is missing one line the pinned rule has). The original
is already in the backup from section 4:

```bash
docker cp rosmaster_humble:/root/yahboomcar_ws/src/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /tmp/56-orbbec-pinned.rules
sha256sum /tmp/56-orbbec-pinned.rules
# expected fd5bcd51b7b6bb7115868d210c68c0ef87d1706580d6d8bb2a777fbdb0590bcf
sudo install -m 0644 -o root -g root /tmp/56-orbbec-pinned.rules /etc/udev/rules.d/56-orbbec-usb.rules
```

Then install the robot-specific `99-rosmaster-x3.rules`, reload with
`sudo udevadm control --reload-rules`, and reboot. Verify:

```bash
ls -l /dev/robot/ /dev/astradepth /dev/astrauvc     # also check inside the container
ls /dev/myserial /dev/rplidar                       # must not exist
```

`/dev/robot/motor` and `/dev/robot/lidar` should be `0660 root:dialout`;
`/dev/astradepth` and `/dev/astrauvc` should be `0666 root:video`.

**Astra USB port.** The Astra's depth function sometimes fails to enumerate
(see [the known issue](troubleshooting/known_issues/astra-depth-cold-boot-enumeration-failure.md)),
and that failure has been shown to travel with the module, not the port. `x3-c`'s
earlier result — stable on downstream port 4 of the powered Yahboom hub,
failing on a direct Pi port — is evidence for that robot only (see
[robot_side_verification_todo.md](robot_side_verification_todo.md)), not a rule
for every unit. Record which USB port and bus path the Astra is on for each
robot, and confirm that both functions come up after a cold boot and after warm
reboots without touching a cable.

### Configure identities for launch

For one manual launch, pass this robot's identities into the container shell.
`export` in a host shell does not reach `docker exec`, so give `docker exec`
the variables with `-e`:

```bash
docker exec -it \
  -e ROSMASTER_MOTOR_PORT=/dev/robot/motor \
  -e ROSMASTER_LIDAR_PORT=/dev/robot/lidar \
  -e ROSMASTER_ASTRA_SERIAL=REPLACE_WITH_ASTRA_SERIAL \
  rosmaster_humble bash
```

(Or run `export` for the same three variables inside a container shell you
already opened.) Use the Astra serial that `list_devices_node` printed. The
autostart units set the same variables from `/etc/rosmaster/platform.env`
(section 10).

## 9. Smoke-check the launch

This is the one check worth running before calling a new robot done: confirm
the platform actually comes up healthy on *this* unit's wiring. It's not a
re-proof of the driver — that's already established — just a fast sanity
check that nothing's plugged in wrong.

Read the battery voltage before you start. This guide sets no voltage
requirement. For reference, `x3-c`'s almost-full idle reading was about 11.3 V
and `mikey` ran this check at about 10.6 V. Read it passively from inside the
container (it opens the serial port but transmits nothing):

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --port /dev/robot/motor --samples 5 --period 0.5
```

Lift the drive wheels, then, in the container shell from section 8:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

In another container shell (`docker exec -it rosmaster_humble bash`):

```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/physical_contract_probe.py
```

If it doesn't pass on the first try, the probe's output says exactly what's
missing or wrong — usually a device path, a loose connector, or a camera
serial that doesn't match what's plugged in, not a code problem.

Optionally, with wheels still lifted, use the bounded pulse tool
(`tools/safe_cmd_vel_pulse.py`) to confirm forward/strafe/rotation move the
wheels in the expected direction and `/odom` agrees — this checks *this
unit's* encoder and motor wiring, not the driver, and takes under a minute:

```bash
python3 tools/safe_cmd_vel_pulse.py --x 0.10 --duration 1.0
python3 tools/safe_cmd_vel_pulse.py --y 0.10 --duration 1.0
python3 tools/safe_cmd_vel_pulse.py --yaw 0.60 --duration 1.0
```

Use `0.60 rad/s` for the rotation pulse, not `0.30`. At `0.30 rad/s` each wheel
is commanded to only about `0.05 m/s`, below the motors' stiction threshold, so
the response is delayed and uneven even on a healthy robot (on `mikey` only one
wheel responded, and a repeat didn't move at all). That is the owner-accepted
platform characteristic described in Section 6 of
[robot_side_next_moves.md](robot_side_next_moves.md), not a wiring fault.
Compare the wheel signs against the table in
[odometry_validation.md](odometry_validation.md) if anything looks backwards.
Stop the launch with `Ctrl-C` when you are done.

## 10. Install autostart

Autostart is already validated for this stack — follow
[autostart_setup.md](autostart_setup.md) directly, no separate decision gate
needed.
