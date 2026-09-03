# Root Filesystem Full Causing LightDM Login Loop And Docker Growth

## Status

Resolved on `x3-c`; prevention and autostart hardening are still recommended.

## Symptom

The ROSMASTER X3 unexpectedly stopped entering the Raspberry Pi desktop
automatically and showed the LightDM login screen.

At the login screen:

- incorrect passwords were rejected immediately;
- the correct `pi` password was accepted;
- the screen went black for a few seconds;
- LightDM returned to the login screen instead of starting the desktop.

SSH initially appeared to hang, but that was a separate issue: the IP address
being used was stale. The current address was recovered from a local TTY with
`Ctrl+Alt+F2` and `hostname -I`, after which SSH worked normally.

## Affected Environment

- Robot: `x3-c`
- Raspberry Pi OS / Debian host with LightDM
- Yahboom ROS 2 Humble image: `yahboomtechnology/ros-humble:4.1.2`
- Long-lived Docker container: `rosmaster_humble`
- Development through VS Code Remote SSH and Dev Containers

At diagnosis, the robot repository was on branch
`x3-validation-results-2026-08-21` at commit
`aafed4496b711daf4e9b2bfd9f2cfdf34a488ae2`, with an untracked
`robot_artifacts/` directory. Preserve robot-side artifacts before changing
branches or cleaning a workspace.

## Safety / Data-Loss Warning

Do not respond to this symptom by immediately:

- reflashing the SD card;
- resetting the `pi` password;
- deleting `/var/lib/docker` manually;
- deleting the active Docker image or container;
- running `docker system prune -a`;
- removing ROS packages from the live Yahboom image;
- deleting `/root/yahboomcar_ws` or robot validation artifacts.

A correct password followed by a black screen and a return to LightDM can mean
that authentication succeeded but the graphical session could not start. In
this incident, recovery required freeing disk space, not changing credentials.

## Fast Diagnosis

### 1. Confirm That Authentication Succeeds

From SSH:

```bash
systemctl status lightdm --no-pager
journalctl -b -u lightdm --no-pager | tail -50
```

The important pattern observed was:

```text
pam_unix(lightdm:session): session opened for user pi
pam_unix(lightdm:session): session closed for user pi
```

This distinguishes successful authentication followed by a failed desktop
session from an incorrect password.

### 2. Check Root Filesystem Capacity Immediately

```bash
df -h /
```

Observed failure state:

```text
Filesystem       Size  Used  Avail  Use%  Mounted on
/dev/mmcblk0p2    36G   35G      0  100%  /
```

A full root filesystem can prevent the desktop and user session from creating
or updating required files.

### 3. Identify The Largest Host Directories

```bash
sudo du -xhd1 / 2>/dev/null | sort -h
sudo du -xhd1 /var 2>/dev/null | sort -h
du -xhd1 /home/pi 2>/dev/null | sort -h
```

Observed major consumers:

```text
21G  /var
18G  /var/lib/docker
7.9G /home
3.9G /home/pi/.vscode-server
```

### 4. Inspect Docker Before Deleting Anything

```bash
sudo docker system df
sudo docker ps -a --size
sudo docker images
```

Observed:

```text
Yahboom base image:                13.74 GB, active, 0 B reclaimable
rosmaster_humble writable layer:   4.57 GB
rosmaster_humble virtual size:     approximately 18.3 GB
```

The base image is immutable and the running container depends on it. The
writable-layer size is growth accumulated after container creation. Removing
packages inside the running container does not remove the underlying base-image
layers; a smaller base footprint requires a newly built image.

### 5. Inspect The Container Writable Layer

```bash
sudo docker exec -it rosmaster_humble bash

du -xhd1 /root 2>/dev/null | sort -h
du -sh /root/yahboomcar_ws/build \
       /root/yahboomcar_ws/install \
       /root/yahboomcar_ws/log \
       /root/yahboomcar_ws/src 2>/dev/null
du -xhd1 /var 2>/dev/null | sort -h
```

Observed inside the container:

```text
2.7G  /root/.vscode-server
327M  /root/yahboomcar_ws
147M  /root/.cache
163M  /root/.codex
```

The ROS workspace was not the dominant writable-layer consumer. The container's
VS Code Server directory contained approximately `1.3G` in `bin/`, `1.0G` in
`extensions/`, and `421M` in `extensionsCache/`.

## Distinguishing Evidence

Three symptoms occurred together but had different causes:

- Correct password, brief black screen, and immediate return to LightDM, plus a
  root filesystem at `100%`, identified the graphical-session failure.
- Initial SSH attempts used a stale IP address. `hostname -I` recovered the
  current address; this was separate from the login loop.
- `sudo: unable to resolve host x3-c` came from an `/etc/hostname` and
  `/etc/hosts` mismatch. It was also unrelated to the login loop; see
  [Hostname Resolution Warning On The Raspberry Pi Host](hostname-resolution.md).

## Root Cause

The direct cause was exhaustion of the Raspberry Pi root filesystem. Docker was
the largest storage consumer at approximately `18G`, but the largest avoidable
growth came from development tooling:

- container `/root/.vscode-server`: approximately `2.7G`;
- host `/home/pi/.vscode-server`: approximately `3.9G`;
- archived system journals: at least `644.7M` reclaimable during the incident;
- smaller development caches.

The active Yahboom image itself occupied approximately `13.7G` and was not
prunable because the running container depended on it.

## Recovery Performed

### 1. Vacuum Archived Journals

On the host:

```bash
sudo journalctl --vacuum-size=100M
```

This recovered `644.7M` during the incident.

### 2. Clear The Package Cache

```bash
sudo apt clean
```

### 3. Remove Unused VS Code Server State Inside The Container

First verify that no VS Code server is active:

```bash
ps -ef | grep -E 'vscode|code-server' | grep -v grep
```

Only after confirming that no remote session is active, stale development state
can be removed. The incident recovery removed:

```bash
rm -rf /root/.vscode-server
rm -rf /root/.cache/*
```

This reduced the container writable layer from approximately `4.57 GB` to
`1.83 GB`. VS Code will reinstall its server and extensions on the next remote
connection.

### 4. Remove Unused VS Code Server State On The Host

Again, first verify that no host VS Code server is running:

```bash
ps -ef | grep vscode | grep -v grep
rm -rf ~/.vscode-server
```

This removes downloaded server binaries and extension state for that user, so
do it only when the space is needed and a later re-download is acceptable.

### 5. Verify Recovered Capacity

Final observed state:

```text
Filesystem       Size  Used  Avail  Use%  Mounted on
/dev/mmcblk0p2    36G   26G   8.3G   76%  /
```

The desktop recovered after sufficient space was available and LightDM was
restarted.

## Verification

```bash
df -h /
sudo docker ps -a --size
systemctl status lightdm --no-pager
```

Verify that:

- the root filesystem has several GiB free;
- the Docker writable layer is no longer unexpectedly large;
- LightDM remains running;
- the `pi` desktop starts successfully;
- SSH works at the robot's current address;
- `sudo` no longer reports the separate hostname-resolution warning.

For `x3-c`, recovery left approximately `8.3 GB` free at `76%` root usage, and
the `rosmaster_humble` writable layer was approximately `1.83 GB`.

## What Not To Do

### Do Not Automatically Run `docker system prune -a`

The active Yahboom image and ROSMASTER container reported `0 B` reclaimable.
Aggressive pruning can remove stopped development containers, useful images,
caches, or other state without fixing growth in an active container.

### Do Not Manually Remove `/var/lib/docker`

That can corrupt Docker state and destroy the robot environment.

### Do Not Expect `apt remove` To Shrink The Existing Base Image

Base-image layers remain on disk. Cleanup inside a container can reduce
writable-layer growth but cannot retroactively shrink the original image.

### Do Not Put Destructive Cleanup In ROS Bringup

Robot startup should be deterministic and fail clearly. It must not silently
delete development data, ROS bags, calibration, or validation evidence to make
itself start.

## Prevention / Hardening

Prefer a disk guard, bounded logging, monitoring, and separate maintenance over
a generic `cleanup && start_ros` launcher.

### 1. Add A Disk-Space Preflight To Host Autostart

Before starting the container or ROS platform, check both percentage used and
absolute free space. A suggested policy for the observed `36 GB` root partition
is:

- warn at `85%` used;
- require maintenance at `90%` used;
- refuse platform autostart at `95%` used or below `2 GiB` free.

Example host guard:

```bash
#!/usr/bin/env bash
set -euo pipefail

MAX_USED_PCT="${MAX_USED_PCT:-94}"
MIN_FREE_MIB="${MIN_FREE_MIB:-2048}"

read -r used_pct avail_mib < <(
  df -Pm / | awk 'NR==2 {
    gsub("%", "", $5);
    print $5, $4
  }'
)

if (( used_pct > MAX_USED_PCT || avail_mib < MIN_FREE_MIB )); then
  logger -t rosmaster-disk-guard \
    "Refusing startup: root=${used_pct}% used, ${avail_mib} MiB free"
  exit 1
fi
```

Install the reviewed script on the host and invoke it before the ROS launcher:

```ini
ExecStartPre=/usr/local/sbin/rosmaster-disk-guard
```

Keep this host-level protection even if disk usage is later published on ROS
diagnostics: a full filesystem can prevent ROS itself from starting.

### 2. Bound Persistent Journal Growth

Create a drop-in such as `/etc/systemd/journald.conf.d/rosmaster-storage.conf`:

```ini
[Journal]
SystemMaxUse=200M
SystemKeepFree=2G
RuntimeMaxUse=100M
```

Review the values against validation-log retention needs, then restart journald:

```bash
sudo systemctl restart systemd-journald
```

### 3. Rotate Docker Container Logs

Inspect the existing Docker logging configuration first. If the host uses the
`json-file` driver and has no conflicting policy, merge bounded log settings
into `/etc/docker/daemon.json`:

```json
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
```

Do not overwrite an existing daemon configuration. Validate the merged JSON and
plan the Docker restart because it affects the robot container. Log rotation
will not solve VS Code Server growth, but it prevents another unbounded path.

### 4. Keep Maintenance Separate From Autostart

Use a dedicated systemd timer for low-risk housekeeping. Suitable automatic
tasks include bounded journal vacuuming, apt cache cleanup, and storage
reporting. VS Code cleanup should run only when no Remote SSH or Dev Container
session is active, and it should retain the newest one or two server versions.

Never let scheduled maintenance touch:

- ROS bags or `robot_artifacts/`;
- calibration or deployment records;
- repository source;
- `build/`, `install/`, or `log/` during a build or validation run;
- Docker images and containers through aggressive pruning.

### 5. Monitor Docker And VS Code Growth

Useful periodic checks are:

```bash
docker system df
docker ps -a --size
du -sh /var/lib/docker
du -sh /home/pi/.vscode-server 2>/dev/null
docker exec rosmaster_humble \
  du -sh /root/.vscode-server 2>/dev/null
```

For this robot, alert when the `rosmaster_humble` writable layer exceeds `3 GB`
or grows by more than `1 GB` between checks without an expected build event.
Monitor host and container VS Code state separately.

### 6. Preserve Build Headroom And Acceptance Evidence

Require several GiB of free space before `rosdep` operations and colcon builds;
their temporary footprint can exceed the final installed workspace. Treat apt
caches, bounded old journals, and stale VS Code binaries as disposable. Preserve
robot artifacts, bags, calibration, deployment records, hashes, and validation
results.

### 7. Build A Smaller Versioned Image Later

The current Yahboom image is approximately `13.7 GB`. After the accepted
platform dependency set is known, build and validate a smaller versioned image
instead of trying to shrink `yahboomtechnology/ros-humble:4.1.2` in place. Keep
build dependencies separate from runtime dependencies where practical and clear
apt lists and build caches in the same image layers where they are created.

### 8. Make Autostart Fail Clearly

The preferred architecture is:

```text
systemd host service
    |
    +-- ExecStartPre: hostname, device, configuration, and disk checks
    +-- start and verify one versioned ROS container
    +-- launch one accepted X3 platform bringup
    +-- propagate bringup failure to systemd

separate systemd timer
    |
    +-- bounded journal and apt maintenance
    +-- safe VS Code cleanup only when idle
    +-- disk and Docker size reporting
```

Do not hide platform failures behind `nohup`, detached processes, or a launcher
that stays alive after a required ROS process has died.

## Storage-Hardening Acceptance Criteria

Before enabling a future autostart routine:

- [ ] root starts with at least `20%` free space or a documented equivalent;
- [ ] startup refuses to continue below the critical reserve;
- [ ] journald has an explicit storage cap;
- [ ] Docker logs have an explicit rotation policy;
- [ ] no scheduled job runs `docker system prune -a`;
- [ ] maintenance preserves robot acceptance artifacts;
- [ ] VS Code Remote storage is monitored on host and container;
- [ ] the container writable layer has an alert threshold;
- [ ] failure is tested with a raised guard threshold, not by filling the SD card;
- [ ] maintenance actions and guard failures are visible in `journalctl`;
- [ ] the accepted image version and expected baseline footprint are recorded.

## Affected Robot / Incident

- Robot: `x3-c`
- Workstation-local date: 2026-08-26 (`America/Argentina/Buenos_Aires`)
- Robot-local date recorded during diagnosis: 2026-08-27
- Symptom: LightDM login loop after successful authentication
- Direct cause: root filesystem at `100%`
- Major storage path: `/var/lib/docker`
- Major avoidable growth: VS Code Server state on host and container
- Recovered state: approximately `8.3 GB` free / `76%` root usage

## Related Documentation

- [Troubleshooting Index](../README.md)
- [Hostname Resolution Warning](hostname-resolution.md)
- [ROS 2 Humble And Autostart Setup Guide](../../setup_guide_ros2_humble_autostart.md)
