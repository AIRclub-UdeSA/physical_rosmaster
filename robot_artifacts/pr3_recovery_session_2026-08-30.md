# PR #3 `x3-c` recovery and validation session: 2026-08-30

Status: documentation, read-only evidence preservation, fresh-host recovery,
stable device setup, exact source deployment, dependency restoration, clean
build, and isolated tests complete. Strict platform bringup, fail-closed tests,
and all motion remain pending.

This is the durable session record for continuing the `x3-c` host recovery and
PR #3 validation. The workstation repository is the source of truth. The robot
is a deployment and validation target, not a place for uncommitted acceptance
fixes.

## Provenance

This baseline reconciles:

- the local repository runbooks and the 2026-08-26 robot record;
- the supplied `rosmaster_session_handoff_2026-08-30.md`;
- the supplied `rosmaster_systemd_sigill_incident_draft.md`;
- the operator's instruction to begin with documentation while the robot is
  shut down for the workstation-only phases.

Commands found inside the supplied documents were treated as proposed
procedures, not as proof that they were run in this session. The preserved-file
claims are now independently verified in
[the read-only recovery manifest](pr3_recovered_evidence_2026-08-30.md).

## Repository baseline

Observed locally at the start of this session:

```text
branch: platform/simulator-parity
HEAD: 680c6f7b41434b33b54904eb01dfd83c80bc71b4
cached origin/platform/simulator-parity: 680c6f7b41434b33b54904eb01dfd83c80bc71b4
worktree: clean
```

The remote was fetched before deployment. Local HEAD and
`origin/platform/simulator-parity` both resolved to the recorded SHA with
divergence `0 0`.

Previous physical evidence was collected at
`1bdb7a77851d938d1111d134d49b67e7f389d6e1`. It is historical evidence only and
does not accept the current head.

## Recovered-host baseline

Reported in the handoff and independently re-verified over SSH on 2026-08-30:

```text
host: yahboom.local
OS: Debian GNU/Linux 12 (bookworm)
kernel: 6.6.62+rpt-rpi-2712 aarch64
root partition: approximately 57.7G
container: rosmaster_humble
image: yahboomtechnology/ros-humble:4.1.2
image digest: sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3
Docker ROS_DOMAIN_ID: 11
```

At the initial checkpoint the container lacked `Rosmaster_Lib` and a deployed
repository. The factory host contained Rosmaster_Lib 3.3.9 in a Python 3.11
egg; its embedded `Rosmaster_Lib.py` had the required SHA256
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.

## Preserved sources and read-only recovery

Neither source was modified. The full-card image was never mounted; derived
partition copies were inspected read-only and recovered payloads were hashed.

### Full old-card image

```text
path: /home/juan/rosmaster-x3-before-reflash-2026-08-29.img
verified size: 62534975488 bytes
verified SHA256: 006e7d46cc932f3ab2d2966a8eeb203f6bfdace5dbb1cd27b8fa4bdf69f8b529
mode: 0444
```

### Old container metadata archive

```text
path: /home/juan/rosmaster-humble-container-metadata.tgz
old container ID: 7905e90e013d3b522ed00db4da334c3325ce2285e86287a410104a02d3d891ed
verified SHA256: f426b9677c2fb9ca629af5ccab9d49417f22161e5f9e34764a7c3c66ef0df151
```

Large recovered payloads and the 1163-entry SHA256 file manifest are under
`/home/juan/rosmaster-evidence/2026-08-30`. The manifest SHA256 is
`e54f74067bbe469ca7067df66c8d38060c88353da0608041541b6231c23b60ef`.
See [the recovery manifest](pr3_recovered_evidence_2026-08-30.md) for partition
hashes, acquisition limits, bag hashes, recovered logs, host evidence, and
explicit losses.

Important recovered results:

- all expected PR #3 bags, attempt logs, contract output, and final test XML
  survive;
- the final XML reproduces 65 tests, 0 errors, 0 failures, and 3 skips;
- two physical-contract pass records survive, including the post-motion pass;
- all four bag databases return `ok` from read-only SQLite integrity checks and
  subsequently open successfully with `ros2 bag info` in ROS 2 Humble;
- selected systemd, libsystemd, libc, and loader files match the old dpkg MD5
  manifests;
- no PID 1 core survives, and the recovered host core belongs to the earlier
  `runc` incident;
- persistent journals survive but require a newer compatible `journalctl` for
  structured analysis;
- repeated earlier root-journal recoveries and one unreferenced empty Git object
  are preserved as facts, not treated as proof of the SIGILL's cause.

## Canonical new-robot target and recovery delta

This incident does not define a different application architecture. The target
remains the normal new-ROSmaster setup documented in the root
[README](../README.md), the
[Humble setup guide](../docs/setup_guide_ros2_humble_autostart.md), and the
[robot verification checklist](../docs/robot_side_verification_todo.md).

### Expected setup for a new ROSMASTER

1. The host owns the Yahboom OS, Docker lifecycle, systemd, and USB passthrough.
   It must not run a competing process that owns the motor, LiDAR, or camera.
2. ROS 2 Humble runs in `rosmaster_humble`; source is deployed at
   `/root/yahboomcar_ws/src/physical_rosmaster` and generated state stays under
   `/root/yahboomcar_ws`. Source backups stay outside that workspace.
3. The exact robot-provided Rosmaster_Lib 3.3.9 is importable by the container's
   ROS Python. The validated module SHA256 is
   `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`;
   an arbitrary copy is never vendored into this repository.
4. The repository is one exact clean commit. `physical_rosmaster.repos` imports
   the pinned Orbbec driver at
   `f7e71d9ce806e788cb48d8580aac2c778fba4214`; `rosdep` installs declared
   dependencies; the workspace contains eight local platform packages plus the
   two external Astra packages; build and tests start from clean generated
   state.
5. Motor, A1 LiDAR, and Astra identities are observed on the actual robot.
   Reviewed host udev rules create stable motor/LiDAR aliases, the Astra is
   selected by serial, and all identities and permissions are verified inside
   the container and after reboot.
6. Bringup is manual during acceptance. It starts the single strict platform,
   has no default `/cmd_vel` publisher, fails closed when required hardware is
   missing, and must pass the complete public sensor/odometry/TF/diagnostics
   contract.
7. Lifted motion, watchdog/deadman behavior, repeated measured floor trials,
   and unchanged simulator/physical consumer parity pass in that order.
8. Only after one robot passes every gate may a new autostart design begin. It
   must be versioned, start only the strict platform, load per-robot identities,
   propagate failures, and never start joystick, keyboard, calibration, or
   project behavior by default.

### Additional controls for this recovery

The failure history adds controls around that normal target; it does not permit
restoring the old host wholesale or reusing factory/legacy launch paths:

- preserve and hash the failed card, old validation evidence, factory image,
  Docker digest, library, repository revision, and device configuration;
- restore the clean host in independently verified layers rather than copying
  an untrusted old root or container state back onto it;
- remove every competing factory or legacy actuator owner before discovery or
  ROS launch, then reboot-verify the absence;
- before write-heavy builds, package operations, or image pulls, record
  ROSMASTER battery voltage, Pi throttling state, root/Docker usage, and current
  storage warnings;
- use `sudo poweroff`, wait for Linux to halt, and only then remove main power;
- after configuration changes, verify a graceful shutdown, clean reboot,
  current journals, and absence of new SD/MMC, I/O, filesystem-recovery, or
  package-integrity warnings;
- keep persistent health/evidence records and consider high-endurance media as
  risk reduction, without claiming the original microSD caused the incident.

The exact initiating cause of the PID 1 SIGILL remains **Undetermined**. These
controls reduce recurrence and recovery cost; they are not a causal finding.

## Branch, architecture, and acceptance boundary

The locally cached branch structure at this checkpoint is:

```text
aafed44  tag: pre-platform-contract-cleanup
  |
  +-- 3232f53  refactor: make X3 a simulator-parity hardware platform
      |
      +-- 1bdb7a7  add robot-side verification checklist
          |
          +-- 5100be3  commit robot-validation fixes and evidence
              |
              +-- 680c6f7  current platform/simulator-parity HEAD; plan remaining gates

origin/main: cea731a (diverged after aafed44)
```

`platform/simulator-parity` is the new architecture under acceptance. The
pre-cleanup tag and `agents/` reports are historical evidence, not instructions
for rebuilding the old application/EKF graph. The branch removes behavior,
navigation, mapping, localization, EKF, and other application packages from the
hardware-platform default; consolidates strict X3 bringup; makes wheel encoders
own `/odom`; normalizes LiDAR/Astra data to the simulator contract; and keeps
every command publisher opt-in.

The 2026-08-26 evidence is valuable but does not accept current HEAD:

- the report records source `1bdb7a7`, while fixes made during that session were
  committed later as `5100be3`;
- the recovered final old-container tree is at `5100be3`, which does not prove
  every recorded observation was made from that exact clean commit;
- `680c6f7` adds only the remaining-gates documentation, but the physical run
  still left stable host udev installation, all three fail-closed checks,
  charged repeatable lifted motion, operator tools, repeated measured floor
  trials, simulator/physical consumer parity, and second review incomplete.

Before the next acceptance run, fetch the server, require local HEAD to equal
`origin/platform/simulator-parity`, record the exact hash, deploy only a clean
checkout, import the pinned Astra commit, and build/test from clean generated
state. No uncommitted workstation or robot-only change counts as validation.
If code changes, the affected gates restart against the new commit.

### New autostart is a separate architecture follow-up

Neither the factory host `rosmaster.desktop` application nor the image's
`my_ros_service.service`, Supervisor configs, `/root/run_handle.sh`, or any old
`/root/auto_start.sh` is the future autostart design. They target competing
actuator ownership, old package graphs, unstable device names, operator tools,
or detached processes whose failures do not propagate correctly.

Only after the exact new platform revision passes the complete acceptance
checklist should a new branch/PR be created from that accepted head for
autostart. Its design requirements are:

- versioned host service, scripts, and per-robot configuration in the
  repository;
- stable motor/LiDAR aliases and Astra serial, with no `/dev/ttyUSB*` or video
  index fallback;
- one foreground, failure-propagating invocation of the strict platform launch;
- no joystick, keyboard, calibration, project behavior, navigation, mapping,
  localization, or `/cmd_vel` publisher by default;
- pre-start checks for competing device owners, required devices, exact
  deployment identifiers, storage headroom, and recent host/storage warnings;
- health/diagnostic readiness before any external consumer starts;
- bounded restart behavior, persistent logs, and a graceful stop path that lets
  the driver send repeated zero commands before container/host shutdown;
- reboot, failure-injection, and clean-shutdown validation as a new change, not
  reuse of PR #3's manual acceptance evidence.

Autostart remains blocked during this recovery and PR #3 validation.

## Fresh-host read-only preflight

The preflight used key-based SSH to `pi@yahboom.local`. It did not launch ROS,
change the host or container, publish a command, or access the controller from a
second process.

### Host and storage

- Hostname `yahboom`; Debian 12; kernel `6.6.62+rpt-rpi-2712` on AArch64.
- `systemctl is-system-running` returned `running`; there were zero failed
  units. Docker 27.5.0 was active.
- Root is the expanded 57 GB ext4 `c777c139-02`: 29 GB used, 26 GB available,
  54% utilization. The 510 MB FAT boot partition is 15% utilized.
- Read-only `tune2fs -l` reports ext4 state `clean`, mount count 4, and 74 GB
  lifetime writes. No current-boot SD/MMC I/O error or ext4 recovery was found.
- `vcgencmd get_throttled` returned `0x0` and CPU temperature was 30.1 C. This
  verifies Pi throttling flags only; it is not a ROSMASTER battery-voltage
  measurement.
- The preceding shutdown completed through `systemd-poweroff`, filesystem sync,
  and `Journal stopped`.
- This boot renamed a journal as “corrupted or uncleanly shut down.” The active
  system/user journals and that renamed current journal all pass
  `journalctl --verify`. Verification of the entire factory history found 100
  passing files and one corrupt January 2024 journal inherited with the factory
  image. Reboot verification should confirm that no new journal is renamed.
- The RTC begins boots with an old time and later jumps to synchronized time.
  Wall-clock-derived uptime, Docker “Up” duration, and some journal timestamps
  are therefore not reliable elapsed-time measurements.

### Container

The running container was independently confirmed as:

```text
container ID: 7b6fdf7fa72a2657cb9a89eeb5f7b7cdd11d3aa32057d8398cc946f0900368f3
name: rosmaster_humble
image ID: sha256:dedc515c326ad7ac812822e60c7796e70bc639d682b09662d46d882dfc0e89fa
repo digest: yahboomtechnology/ros-humble@sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3
command: bash
network: host
privileged: true
bind: /dev:/dev
restart: unless-stopped
ROS_DOMAIN_ID: 11
```

The only persistent container process was PID 1 `bash`. The domain-11 node list
was empty; the topic-list probe exposed only its own `/parameter_events` and
`/rosout`, and the ROS daemon was not running. The current repository and
`Rosmaster_Lib` are absent.

The image contains latent factory launch configuration:

- `/etc/systemd/system/my_ros_service.service` invokes `/root/run_handle.sh`;
- `/root/run_handle.sh` launches the factory joystick bringup;
- Supervisor configs have `autostart=true` and invoke the same script;
- the Supervisor configs disagree on domains 20 and 30.

None is active because the container runs `bash`, not systemd or Supervisor.
They remain unsafe fallback paths and must not be enabled.

### Factory host actuator conflict

The graphical host session automatically starts this world-writable entry:

```text
/home/pi/.config/autostart/rosmaster.desktop
Exec=gnome-terminal -- bash -c "python3 ~/Rosmaster/rosmaster/rosmaster_main.py;exec bash"
```

The resulting `rosmaster_main.py` process:

- owns `/dev/ttyUSB0`, the CH340 motor controller at topology `1-1.2`;
- listens on LAN TCP ports 6000 and 6500;
- contains live motor and servo command handlers using `set_motor` and UART
  servo calls;
- had no established client connection when inspected.

This is not a ROS graph, but it is a competing, remotely reachable actuator
owner. It must be stopped and persistently disabled before the container
library is restored or any ROS driver is launched. The separate OLED entry
starts `yahboom_oled.py`, which only showed system-statistics references during
this audit; do not disable it by association without a separate reason.

#### Resolution and reboot verification

The operator authorized a reversible disablement on 2026-08-31 robot-local
time. Before the change:

```text
/home/pi/.config/autostart/rosmaster.desktop
mode: 0666
SHA256: 9600e5a244d69523f58a0ed332020be567b61a0e222c0cc5cfec564b77cab396

/home/pi/Rosmaster/rosmaster/rosmaster_main.py
mode: 0777
SHA256: b0fb76b676dce6978bec3baca9718af65c0c2a47541e4d3e88e463e269135549
```

The desktop entry was moved, not deleted, to
`/home/pi/.config/autostart-disabled/rosmaster.desktop` and set to mode `0644`.
Its SHA256 remained unchanged. The running app had no established client; it
was sent SIGINT so its explicit `KeyboardInterrupt` handler commanded zero
chassis motion before closing. The process exited, `/dev/ttyUSB0` became
unowned, and ports 6000/6500 closed.

The host was then rebooted through `sudo systemctl reboot`. Post-reboot boot ID
was `efbce1e1-4d9b-4eec-83b5-d3cfc3dda240`. Verification found:

- no `rosmaster_main.py` process and no owner of `/dev/ttyUSB0`;
- no listener on ports 6000 or 6500;
- no ROS nodes or `/cmd_vel` on legacy domains 11, 20, or 30;
- container PID 1 remained `bash`, with no repository or container library yet;
- the separate OLED process returned normally;
- system state `running`, zero failed units, Pi throttling `0x0`, and ext4 state
  `clean`;
- active system and user journals both passed verification, with no new
  journal-renaming, SD/MMC, ext4 recovery, I/O, or undervoltage warning;
- the preceding reboot ended with filesystem sync and `Journal stopped`.

The factory actuator conflict is resolved. The archived entry must remain
disabled through manual platform acceptance and must not become the basis of
the future autostart design.

### Connected devices

Only the CH340 motor adapter (`1a86:7523`) enumerated. The operator subsequently
confirmed that the LiDAR and Astra had been intentionally disconnected to
reduce battery consumption; their absence at this checkpoint is expected and
is not evidence of device or USB failure. `/dev/robot` aliases do not exist,
while the controller appears as `/dev/ttyUSB0` and by-path
`platform-xhci-hcd.0-usb-0:1.2:1.0-port0`. Only the factory Orbbec udev rules
are currently installed. Reconnect all three devices after charging, then
establish complete device presence and stable aliases as a later gate.

### Passive controller telemetry and battery gate

Inspection of the exact public Rosmaster_Lib 3.3.9 source found that its
constructor opens the serial port and immediately calls
`set_uart_servo_torque(1)`. The prior sampling implementation in
`tools/rosmaster_lib_probe.py` therefore was not fully passive even though it
did not command wheel motion. The worktree probe was corrected to:

- import and hash the library without constructing `Rosmaster`;
- read zipped `.egg` source through its Python loader;
- open the controller with DTR and RTS disabled;
- parse the existing speed, encoder, and voltage auto-report stream; and
- never call `Serial.write()` or send a request frame.

A separate eight-second passive read received 780 checksum-valid frames: 195
each of speed, attitude, encoder, and raw-IMU reports. All 195 voltage samples
were `10.4` V. The revised worktree probe was then executed from standard input,
without deployment to the robot, and reported:

```text
Rosmaster_Lib SHA256: e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c
matches_public_v3_3_9: true
three samples: motion 0/0/0, encoders 0/0/0/0, battery 10.400 V
```

This is an idle powered-system measurement, not the required fully charged
rested baseline. At `10.4` V, restoration and write-heavy build activity remain
blocked pending charge. LiDAR and Astra enumeration remains pending because the
operator intentionally disconnected them to conserve battery, not because a
fault was observed.

The host then accepted `sudo systemctl poweroff`; its network address stopped
responding after shutdown. On the next boot, verify the preceding journal ends
cleanly before treating this as a completed safe-shutdown check.

### Charged device baseline and stable identities

The operator charged the robot to almost full and reconnected the LiDAR and
camera. Controller auto-report telemetry first read `11.3` V at idle, then
`11.2` V and `11.1` V during the restoration/build period. The post-test
passive sample remained at `11.1` V. Historical `x3-c` evidence pairs a
controller reading near `11.3` V with a `11.7` V pack multimeter measurement,
so a generic greater-than-12 V acceptance threshold is not substituted for the
robot-specific baseline. A fully charged reading under command load is still
required before motion.

All powered devices enumerated together:

```text
motor: 1a86:7523 CH340; no USB serial; dedicated topology 1-1.2
LiDAR: 10c4:ea60 CP2102; USB serial 0001; topology 3-2
Astra depth: 2bc5:060f
Astra UVC/RGB: 2bc5:050f; UVC USB serial SN0001
```

The pinned driver's camera-only `list_devices_node` subsequently found exactly
one Astra at URI `2bc5/060f@3/4` and confirmed OpenNI serial `ACRC64300ET`.
The executable exited cleanly and released both camera interfaces; it did not
open either serial device or leave a ROS node running.

Before installation, the reviewed robot rule
`robot_artifacts/99-rosmaster-x3.x3-c.rules` had SHA256
`2fb1a8aa3e79424b76a28c810dc6295d3c2a71a24881642ebb230f7dfa9e7c72`.
The Orbbec rule from pinned commit `f7e71d9` had SHA256
`fd5bcd51b7b6bb7115868d210c68c0ef87d1706580d6d8bb2a777fbdb0590bcf`.
The factory `/etc/udev/rules.d/usb.rules` used final `MODE:="0777"`
assignments and legacy `/dev/myserial` and `/dev/rplidar` aliases, which would
override the reviewed serial-device permissions. Original files were preserved
root-only under
`/var/backups/rosmaster-recovery/2026-08-31-udev-preinstall/`; the factory
`usb.rules` was moved to `/etc/udev/rules.disabled/`, and the two reviewed
rules were installed.

After a controlled reboot, boot ID
`f2ae2b7a-a06d-473a-a5c4-36d04ebdaf00` showed:

```text
/dev/robot/motor -> /dev/ttyUSB0       0660 root:dialout
/dev/robot/lidar -> /dev/ttyUSB1       0660 root:dialout
/dev/astradepth -> bus/usb/003/004     0666 root:video
/dev/astrauvc -> bus/usb/003/005       0666 root:video
```

The same aliases and permissions were visible inside the privileged container.
Legacy `/dev/myserial` and `/dev/rplidar` were absent. Neither serial device had
an owner, factory ports 6000/6500 remained closed, and no ROS nodes were
running.

The earlier poweroff reached filesystem sync and `Journal stopped`, but the
following boot still renamed `system.journal` as corrupted or uncleanly shut
down. The renamed file and current journal both passed verification. The later
udev reboot also ended at `Journal stopped` and did not produce another rename.
This recurrence after an apparently clean poweroff remains an unresolved
storage/journaling warning; it does not establish the earlier SIGILL cause.

### Exact workspace, dependency, build, and test restoration

The image's legacy `/root/yahboomcar_ws` included source, generated state, logs,
and a 679 MB core file. It was preserved outside the active workspace at
`/root/rosmaster-recovery-backups/yahboomcar_ws.factory-2026-08-31`. A fresh
`/root/yahboomcar_ws/src` was then populated with exactly:

- `physical_rosmaster` on clean branch `platform/simulator-parity` at
  `680c6f7b41434b33b54904eb01dfd83c80bc71b4`, equal to the fetched remote;
- `ros2_astra_camera` detached and clean at
  `f7e71d9ce806e788cb48d8580aac2c778fba4214`; and
- the eight local platform packages plus the two pinned Astra packages.

The exact host library was restored to
`/root/Rosmaster_Lib/Rosmaster_Lib.py`, exposed to container Python through a
`.pth` file, and verified at SHA256
`e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
Ubuntu `python3-serial` 3.5-1 supplies its serial dependency.

The first dependency attempt exposed stale factory apt indices and an expired
legacy ROS signing key. It was interrupted without bypassing signature checks;
`dpkg --audit` remained empty and `apt-get check` passed. The obsolete ROS list
and trusted keyring were preserved under
`/root/rosmaster-recovery-backups/apt-before-ros2-source-2026-08-31/`. Official
`ros2-apt-source` release 1.2.0 for Jammy was downloaded, verified against SHA256
`767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433`,
and installed. Package-index refresh and `rosdep install` then completed, and
`rosdep check` reported that all system dependencies were satisfied.

A clean `colcon build --symlink-install` completed all ten packages in 2m39s
with no package failure. Stderr contained only known setuptools deprecations and
upstream LiDAR compiler warnings. The required isolated eight-package test run
then produced:

```text
Summary: 65 tests, 0 errors, 0 failures, 3 skipped
```

The 92 generated log files were copied outside the workspace to the root-only
directory
`/root/rosmaster-recovery-evidence/pr3-2026-08-31-build-test/colcon-log`.
Source and archive relative-path tree hashes both equal
`4ebdf953080c5385bfd1f65ba96fe88cc7e06189111a9a905e7d542f657548b6`.

Post-test health remained acceptable: system state `running`, zero failed
units, root filesystem 59% used with 23 GB available, Pi throttling `0x0`, and
no current-boot SD/MMC, ext4, I/O, corruption, or undervoltage kernel warning.
Both source repositories remained clean at their pinned revisions.

#### Probe execution deviation

During the post-test check, the first corrected command mistakenly invoked the
probe file from the robot's clean remote checkout, not the safety-corrected
workstation copy. That committed probe constructs `Rosmaster`, whose constructor
opens the controller and transmits `set_uart_servo_torque(1)` before starting a
receive thread. It sent no wheel-velocity command. The sample reported zero
motion and four zero encoder counts, then released the port. This invocation is
not counted as passive evidence.

The corrected worktree probe was subsequently streamed to Python standard
input without deployment. Its serial path never calls `Serial.write()`. Three
samples reported zero motion, encoder counts `-3/0/0/0`, and battery `11.1` V;
the `-3` value stayed constant across all three samples. Both serial ports were
released afterward, ports 6000/6500 were closed, and the ROS node list remained
empty. No motion is inferred from the constant three-tick offset, and the raw
observation is retained for later comparison.

### Lifted strict-platform run and Astra CPU correction

The operator confirmed that the robot was lifted and ready before strict
bringup. No joystick, keyboard, calibration, pulse tool, or other `/cmd_vel`
publisher was started.

The first strict run at remote head `680c6f7` brought up all seven expected
nodes and produced every required stream. `/cmd_vel` had zero publishers and
one driver subscription. The first unmodified contract probe failed only the
normalized XYZRGB cloud-rate check: five messages arrived, but the median rate
was `0.73` Hz against the `3.0` Hz minimum. A warm retry passed, but the Astra
adapter sustained about 230% CPU across three OpenBLAS workers and continued to
show large cloud jitter. That run is not counted as the positive-gate pass.

A diagnostic relaunch with only `OPENBLAS_NUM_THREADS=1` added reduced total
container CPU from about 218% to 57% and adapter CPU from about 230% to 32%.
The unchanged contract passed, normalized cloud measurements were about 10--13
Hz, and four wheel positions, four wheel velocities, and odometry stayed
exactly zero/identity over ten seconds. This isolated OpenBLAS oversubscription
as the fresh-environment regression.

No loose robot-side fix was retained. The workstation branch received three
commits:

```text
125c30e fix: make Rosmaster library probe passive
c172098 fix: bound Astra adapter BLAS threads
55caf7a test: ignore launch nodes without extra environment
```

The launch fix sets `OPENBLAS_NUM_THREADS=1` only in the Astra adapter action;
it does not depend on an operator shell or constrain unrelated nodes. Verified
Git bundles were transferred into the container and fast-forwarded into its
clean checkout. The robot and workstation resolved to
`55caf7a2a572aae0ad2682e265147c46e525921c`. At the end of the robot run, the
server branch still resolved to `680c6f7`, so the record correctly treated
`55caf7a` as a proposed rather than remote PR head. After workstation review,
the three commits were pushed without rewriting them. A server-side query then
verified `origin/platform/simulator-parity` at exact head `55caf7a`, with local
divergence `0 0`; the runtime tree tested on the robot did not change.

A clean ten-package runtime build including the launch fix completed in 2m38s
with no failures. The final head differs from that runtime tree only by the
one-line regression-test correction. The complete eight-package test selection
at final head produced:

```text
Summary: 66 tests, 0 errors, 0 failures, 3 skipped
```

The final proposed-head launch did not receive an external OpenBLAS setting.
The adapter inherited `OPENBLAS_NUM_THREADS=1` from the versioned launch;
container CPU measured about 55% and adapter CPU about 30%. The complete
contract passed in 2.7 seconds, the normalized cloud measured approximately
8--15 Hz, and `/cmd_vel` still had zero publishers. A final ten-second sample
held all wheel positions and velocities at zero and odometry at the identity
pose. Battery telemetry across these non-motion runs declined from `11.0` V to
`10.8` V; loaded-motion voltage remains untested.

The final shutdown released all devices and left an empty ROS graph. The pinned
Astra driver again logged its known statically-typed parameter-undeclare errors,
and this shutdown also required launch to escalate the LiDAR from SIGINT to
SIGTERM after five seconds. These remain shutdown-quality findings and are not
silently accepted by the positive data-contract pass.

Final-head colcon logs were copied root-only to
`/root/rosmaster-recovery-evidence/pr3-2026-08-31-55caf7a-build-test/colcon-log`;
the 129-file relative-path tree SHA256 is
`d5cec9d9aadae51fbc6591fdab4173209391e0ba4463c0f8372938a3b9acd7c7`.
Positive-run ROS logs are under
`/root/rosmaster-recovery-evidence/pr3-2026-08-31-55caf7a-positive/roslog`.

### Strict device-absence checks

The checks below were performed one device at a time with the robot lifted,
the strict platform stopped before each physical change, and no command
publisher present.

Camera absence passed. With both Astra interfaces absent but the motor and
LiDAR still present, the camera driver found zero devices. After 20.23 seconds
the strict adapter reported all five required streams missing (`cloud`,
`color`, `color_info`, `depth`, and `depth_info`) and exited with code 1. The
enclosing launch stopped every remaining node. The LiDAR required the launch
system's five-second SIGINT-to-SIGTERM escalation during that shutdown. The
empty graph, released devices, zero controller motion, and `10.8` V battery
were verified afterward. Logs are under
`/root/rosmaster-recovery-evidence/pr3-2026-08-31-55caf7a-fail-camera/roslog`.

After reconnecting the camera, the host exposed both interfaces with the
expected permissions and the pinned driver again identified OpenNI serial
`ACRC64300ET`. The complete strict contract passed in 2.76 seconds and the
post-shutdown graph and device handles were clean. The known Astra
parameter-undeclare warnings repeated, while the LiDAR exited cleanly. Logs are
under
`/root/rosmaster-recovery-evidence/pr3-2026-08-31-55caf7a-restore-after-camera/roslog`.

LiDAR absence also passed. With the CP2102 device and `/dev/robot/lidar` alias
absent but the motor and both Astra interfaces present, `sllidar_node` reported
error `80008004` and exited with code 255. The enclosing launch immediately
sent SIGINT to every remaining node; the motor driver closed its serial port
and all processes finished without escalation. The known Astra shutdown
warnings repeated. Post-shutdown checks found no ROS nodes or `/cmd_vel`, no
device owners, closed factory ports 6000/6500, and three passive samples at
motion `0/0/0`, encoders `-3/0/2/0`, and battery `10.8` V. Logs are under
`/root/rosmaster-recovery-evidence/pr3-2026-08-31-55caf7a-fail-lidar/roslog`.
After reconnection, the CP2102 returned and `/dev/robot/lidar` again resolved to
`/dev/ttyUSB1`; the motor and both Astra interfaces also remained present and
none of the four devices had an owner. The required full positive-contract
rerun was deferred because the operator needed to end the session.

### Session shutdown

No further launch or motion test was started. Immediately before shutdown the
host reported `running`, zero failed units, and Pi throttling flags `0x0`; no
platform process owned any device. `sudo systemctl poweroff` was accepted, and
the host then stopped answering three consecutive network probes. Physical
power removal was authorized only after that offline confirmation. On the next
boot, verify that this journal ends cleanly before resuming with the LiDAR
restoration contract.

### Powered-session resume and reboot-stability gate

The next physical-power boot used boot ID
`35168c7d-38dd-4c4b-a1b7-41d6e574d66e`. The preceding journal showed the
requested `systemd-poweroff` path, filesystem sync, and `Journal stopped`, but
the new boot again renamed `system.journal` as corrupted or uncleanly shut
down. Both the renamed journal and its replacement passed
`journalctl --verify`; the renamed file was offline and ended in the preceding
boot's shutdown sequence. The ext4 filesystem remained `clean`, Pi throttling
flags were `0x0`, and no new MMC, ext4, I/O, corruption, or undervoltage error
was found.

While the platform was idle and ROS remained stopped, a controlled software
reboot produced boot ID `9feeb29a-7c2e-482a-b5fb-4bb7d16452da`. Its preceding
journal again contained the normal reboot, sync, and `Journal stopped`
sequence, and this time no journal was renamed. This narrows the recurring
journal rename to the physical-poweroff/power-removal sequence rather than
every reboot. Network disappearance alone is therefore not sufficient evidence
that physical power can immediately be removed; a later physical-power-cycle
test must use a conservative post-halt interval and verify the result on the
following boot.

The Astra depth function (`2bc5:060f`) was absent on both powered boots while
the UVC/RGB function (`2bc5:050f`) was present. Reseating only the camera hub
restored both functions and `/dev/astradepth`, but the depth function was absent
again after the controlled reboot. Motor and LiDAR aliases remained present,
no device had an owner, the container still ran only `bash`, and no ROS launch
was started. Hotplug is not accepted as the normal startup path, so the stable
hardware-identity gate is reopened pending reboot-stable depth enumeration.

No charger was connected during this powered-session observation. Before the
planned orderly shutdown and offline charging, three passive controller samples
reported motion `0/0/0`, encoders `0/0/0/0`, and battery `10.7` V. The exact
public Rosmaster_Lib 3.3.9 hash still matched. The camera boot test must be
repeated near the established `11.3` V almost-full baseline before treating
power margin as either the cause or an excluded condition.

## Documentation created in this phase

- [`systemd` SIGILL incident](../docs/troubleshooting/incidents/2026-08-29-x3-c-host-systemd-sigill.md)
- [Host storage integrity and safe shutdown](../docs/troubleshooting/known_issues/host-storage-integrity-and-safe-shutdown.md)
- [Read-only recovery manifest](pr3_recovered_evidence_2026-08-30.md)
- troubleshooting index entries for both pages

The incident root cause remains **Undetermined**. Recommended shutdown, power,
backup, and storage controls are documented separately from causal claims.

## Gate status

- [x] Reconcile the handoff with the local repository and runbooks.
- [x] Create and index the incident and reusable hardening documentation.
- [x] Create this durable session record.
- [x] Verify the old image and metadata archive without modification.
- [x] Recover and hash surviving PR #3 and host-failure evidence.
- [x] Record explicitly which expected evidence is missing.
- [x] Re-verify the fresh host and container state.
- [x] Stop and persistently disable the factory host's motion-capable
  `rosmaster.desktop` autostart; reboot-verify that it, its ports, and its
  controller ownership do not return.
- [x] Establish a charged idle battery baseline and enumerate the motor, LiDAR,
  and Astra together before restoration or build work. Full-charge/load voltage
  remains pending before motion.
- [x] Restore and verify `Rosmaster_Lib`.
- [x] Fast-forward the remote PR branch to tested head `55caf7a` and record
  local/robot/remote equality. The clean runtime build and 66-test pass are
  complete for this content.
- [ ] Re-establish and reboot-verify stable hardware identities. The pinned
  driver confirmed Astra OpenNI serial `ACRC64300ET`, but the depth function
  subsequently failed to enumerate across reboot without a camera-hub reseat.
- [ ] The pre-motion strict positive contract passed at published head `55caf7a`;
  camera and LiDAR fail-closed checks passed, camera restoration passed the
  full contract, LiDAR re-enumeration passed, and the LiDAR restoration
  contract, motor absence/restoration, and required post-motion repeat remain.
- [ ] Pass charged lifted motion and operator-tool safety gates.
- [ ] Pass repeated measured floor trials.
- [ ] Prove unchanged simulator/physical consumer parity.
- [ ] Archive final evidence and obtain a second review.

## Safety and stop conditions

- No motion is authorized by this record.
- Do not contact or power the robot during the workstation-only evidence phase
  unless a later step explicitly requires it.
- Do not mount the old image read-write or run a repair against it.
- Do not delete the old image or container metadata archive.
- Do not enable autostart while any PR #3 physical gate remains incomplete.
- Stop at a failed gate; do not weaken strict checks or make an uncommitted
  robot-only fix count as evidence.

## Next phase

After the next power-on, verify this session's clean shutdown and re-establish
the idle device/graph baseline. Keep all command publishers and legacy services
disabled. Pass the full positive contract with the restored LiDAR, then perform
the motor-absence check and its restoration contract. Do not begin motion as
part of these checks. Remote publication of tested head `55caf7a` is complete;
do not rewrite that head while acceptance remains in progress.
