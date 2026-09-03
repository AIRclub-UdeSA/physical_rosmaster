# 2026-08-29/30 `x3-c` host boot failure: systemd SIGILL

## Summary

Robot `x3-c`, a Raspberry Pi 5 ROSMASTER X3, stopped booting its Yahboom
Debian 12 host. Raspberry Pi bootloader diagnostics remained available, but the
original microSD installation eventually exposed an early PID 1 (`systemd`)
SIGILL crash and kernel panic. A fresh Raspberry Pi OS microSD ran normally on
the same hardware, and the host recovered after a full backup and factory-image
reflash.

The exact initiating cause was not isolated. The incident is consistent with a
host storage or write-integrity problem, but the evidence does not prove unsafe
power removal, low or transient voltage, failing microSD media, interrupted
writes, or any other single cause.

For reusable operating and recovery controls, see
[Host storage integrity and safe shutdown](../known_issues/host-storage-integrity-and-safe-shutdown.md).

## Symptom

The visible behavior changed during troubleshooting:

- HDMI initially showed no operating system while the onboard ROSMASTER status
  display still showed CPU and RAM information.
- After later power cycles, neither the onboard status display nor the host OS
  appeared.
- The Raspberry Pi status LED could remain solid green.
- Normal boot from the original card never reached a usable host.

A forced text/debug boot exposed:

```text
Failed to allocate the special init.scope unit: Invalid argument
Caught <ILL>
Freezing execution
```

A later debug boot ended with:

```text
Kernel panic - not syncing: Attempted to kill init! exitcode=0x00000004
```

Signal 4 is SIGILL (`Illegal instruction`). PID 1 was `systemd`.

## Affected environment

- Robot: `x3-c`
- Platform: Yahboom ROSMASTER X3 with Raspberry Pi 5
- Host OS before failure: Debian GNU/Linux 12 (`bookworm`)
- Kernel family: `6.6.62+rpt-rpi-2712`, 64-bit
- ROS deployment: Docker host with `yahboomtechnology/ros-humble:4.1.2`
- Original system card: nominal 64 GB microSD

## Safety / data-loss warning

- Do not repeatedly power-cycle a failing Linux host as the primary diagnostic
  method.
- Do not use the ROSMASTER reset control as a Linux shutdown or reboot method.
- Do not format, repair, or reflash the original card before preserving a full
  image when recovery evidence or non-reconstructible data matters.
- Identify the exact block device before any filesystem command. Use read-only
  checks first and never check a mounted filesystem for repair.
- Do not assume a black HDMI screen proves that the Raspberry Pi hardware is
  dead.

## Fast diagnosis

### 1. Separate bootloader/display health from SD-host health

Power off gracefully when possible, remove the microSD, connect HDMI0, and boot
to the Raspberry Pi bootloader diagnostics.

During this incident, the bootloader displayed its missing-media diagnostics.
That established that the boot ROM/EEPROM path, HDMI0, cable, monitor, and at
least basic power delivery were working.

### 2. Inspect the original card without modifying it

On Linux, identify the partitions and compare the configured root target:

```bash
lsblk -o NAME,SIZE,FSTYPE,LABEL,UUID,PARTUUID
cat /media/<user>/bootfs/cmdline.txt
cat /media/<user>/rootfs/etc/fstab
```

The incident card contained:

```text
c777c139-01  512M   FAT   bootfs
c777c139-02  36.1G  ext4  rootfs
```

Both `cmdline.txt` and `/etc/fstab` selected the matching root partition, so a
wrong PARTUUID was ruled out.

### 3. Run read-only filesystem checks first

With both partitions unmounted:

```bash
sudo fsck.fat -n /dev/<boot-partition>
sudo e2fsck -fn /dev/<root-partition>
```

Observed results:

- FAT `bootfs` had its dirty bit set and reported an improper unmount. A backup
  was made before the dirty bit was cleared.
- ext4 `rootfs` showed no structural corruption requiring repair. A minor
  extent-tree optimization prompt was not treated as corruption.

### 4. Expose early boot output

The diagnostic boot removed `quiet splash` and used a one-line kernel command
line containing options equivalent to:

```text
video=HDMI-A-1:1024x768@60D systemd.unit=multi-user.target systemd.show_status=1 loglevel=7
```

This revealed the early `systemd` failure hidden behind the black display.

### 5. Isolate PID 1 from the kernel/root mount

For diagnosis only, appending the following boot option allowed the original
kernel and root filesystem to enter a stable root shell:

```text
init=/bin/bash
```

This showed that the kernel could mount root and execute basic userspace even
though `systemd` failed as PID 1. Both `cgroup` and `cgroup2` support were listed
in `/proc/filesystems`.

### 6. Test independent known-good media

A separately flashed Raspberry Pi OS Lite 64-bit microSD booted on the same Pi,
expanded its root filesystem, started `systemd`, reached first-user setup, and
remained stable. This was the decisive hardware-control test.

USB boot from a Lexar device was inconclusive: the bootloader enumerated the
mass-storage device but did not progress to its boot partition.

## Distinguishing evidence

Evidence supporting an original-host or media-state problem rather than a
deterministic Raspberry Pi hardware incompatibility:

- Raspberry Pi 5 bootloader diagnostics worked with no microSD inserted.
- HDMI0, the cable, and the monitor worked at bootloader stage.
- The original kernel/root booted stably with `init=/bin/bash`.
- An independent fresh microSD ran Linux and `systemd` normally on the same Pi
  and robot power path.
- Reflashing the physical ROSMASTER card with the factory image restored normal
  host operation.

The following package-integrity checks returned no mismatch output:

```bash
sudo dpkg --root=<old-root> --verify \
  systemd:arm64 \
  libsystemd-shared:arm64 \
  libsystemd0:arm64 \
  libc6:arm64
```

The checked versions were internally consistent with Debian 12:

```text
systemd 252.33-1~deb12u1
libsystemd-shared 252.33-1~deb12u1
libsystemd0 252.33-1~deb12u1
libc6 2.36-9+rpt2+deb12u9
```

These checks did not identify the SIGILL-producing file, library, or runtime
state. A structurally clean filesystem and matching package checksums do not
prove that every file and every runtime input is intact.

### Read-only image forensics on 2026-08-30

The complete source-image SHA256 was independently recomputed and matched the
handoff. Exact derived partition copies were inspected without mounting the
source image and without repair:

- the preserved FAT copy is currently clean; it was imaged after the earlier
  observed dirty bit had been cleared, so it cannot reproduce that initial
  state;
- `e2fsck -fn` found no structural repair requirement in rootfs and only
  offered an extent-tree optimization, which was declined;
- plain archived boot logs record repeated `rootfs: recovering journal` boots
  on the recorded dates August 11, 17, 19, 21, and 27, with some orphaned inode
  cleanup;
- one August 19 boot also records repeated journal/udev service failures and a
  timeout waiting for boot PARTUUID `c777c139-01`;
- the exact recovered `systemd`, systemd core/shared libraries, `libsystemd`,
  `libc`, and AArch64 loader files match the old dpkg MD5 manifests;
- the recovered host `/core` belongs to the earlier `/usr/bin/runc --version`
  SIGILL incident; no `systemd`/PID 1 core was found.

The persistent journal files also survive, but the workstation's older
`journalctl` cannot read their feature set. They remain preserved for later
structured analysis with a compatible tool. The boot-log timestamps repeat
after some boots and should not be treated as a reliable chronology without
time-synchronization context.

The complete recovery provenance, hashes, and limitations are in the
[2026-08-30 read-only recovery manifest](../../../robot_artifacts/pr3_recovered_evidence_2026-08-30.md).
These findings establish earlier host instability, not the initiating cause of
the final PID 1 SIGILL.

## Root cause

**Undetermined.**

The confirmed immediate failure mechanism was:

- PID 1 (`systemd`) received SIGILL during very early userspace initialization.
- Linux panicked because init died.

The available evidence does not prove the initiating cause. Plausible
contributors that require additional evidence include:

- unclean shutdown or power removal during writes;
- transient or low-voltage power behavior;
- microSD media or write-integrity degradation;
- interrupted filesystem or package writes;
- another unidentified host-storage integrity fault.

The same robot previously had a separate corrupted `/usr/bin/runc` binary that
crashed with SIGILL and was detected by package checksum verification. That
earlier incident is documented in
[Docker runc illegal instruction](../known_issues/docker-runc-illegal-instruction.md).
The recurrence justifies stronger host/storage controls, but it does not prove
that the two events share a cause.

## Fix

1. A byte-for-byte image of the original card was created before destructive
   recovery.
2. The backup partition table and a recorded SHA256 were captured.
3. The physical card was reflashed with the official Yahboom Raspberry Pi 5
   factory host image.
4. The factory root partition was expanded to fill the physical card.
5. Unused factory Foxy images were removed after confirming no containers
   depended on them.
6. The required Humble image was pulled using the exact deployment tag and
   digest:

   ```text
   yahboomtechnology/ros-humble:4.1.2
   sha256:5ea154fcd205d812aabda8f8506e2c369fe8624c0911fcd800bfc6a88edbccf3
   ```

7. `rosmaster_humble` was recreated with host networking, privileged device
   access, `/dev:/dev`, and the recorded ROS domain and display settings.

Robot dependencies, stable device identities, and the exact repository revision
remain separate reproducible restoration and PR #3 acceptance steps. The entire
broken host state must not be copied over the fresh installation.

## Verification

Host recovery was considered successful when the reported fresh host:

- booted Debian normally;
- restored the onboard ROSMASTER status display;
- drove HDMI0 at the forced `1920x1080@60` mode;
- exposed an expanded root filesystem of approximately 57.7 GB;
- used the exact Humble `4.1.2` image digest recorded above;
- kept `rosmaster_humble` running with the intended Docker configuration;
- accepted SSH connections over the LAN.

ROS and hardware acceptance are a separate PR #3 process. Host recovery does
not establish that the current PR head passes the physical contract.

## What not to do

- Do not keep resetting or power-cycling a failing Linux host in place of
  diagnosis.
- Do not reflash before preserving evidence when recovery or reproducibility
  matters.
- Do not treat HDMI blackness alone as an operating-system or hardware
  diagnosis.
- Do not assume a clean `e2fsck` proves every file's contents are correct.
- Do not assume SIGILL proves a CPU or RAM failure; compare against known-good
  independent media.
- Do not assume one successful `dpkg --verify` covers every file or runtime
  state.
- Do not claim that the microSD was physically defective, that the battery was
  the cause, or that an unclean shutdown initiated the failure without evidence.

## Prevention or hardening

- Use `sudo poweroff` for routine shutdown and wait for the Pi to halt before
  switching off the robot's main power.
- Record battery voltage before long builds, package operations, Docker pulls,
  or physical validation. Avoid heavy writes when power is low or uncertain.
- Monitor `df -h /` and `docker system df`; confirm factory images have been
  expanded before adding large images.
- Preserve a full-card image or a documented backup of non-reconstructible
  state before high-cost maintenance.
- Record the factory-image checksum, Docker tag and digest, `Rosmaster_Lib`
  hash, repository revision, and stable device configuration.
- Consider high-endurance microSD media for production robots as a reliability
  control, not as a conclusion about this incident card.
- Consider a lightweight startup health record for disk usage, voltage, recent
  SD/MMC or I/O warnings, filesystem recovery, and deployed revisions.

See the reusable
[host storage integrity and safe shutdown](../known_issues/host-storage-integrity-and-safe-shutdown.md)
page for the operational checklist.

## Affected robot or incident

- Robot: `x3-c`
- Date: 2026-08-29 through 2026-08-30
- Full-card image: `/home/juan/rosmaster-x3-before-reflash-2026-08-29.img`
- Recorded image size: `62534975488` bytes
- Recorded image SHA256:
  `006e7d46cc932f3ab2d2966a8eeb203f6bfdace5dbb1cd27b8fa4bdf69f8b529`

The size and complete digest above were independently re-verified on
2026-08-30. The original image remains mode `0444` and was not mounted or
modified during recovery.

## Status

The host recovered after full backup and factory reflash. The old image is
preserved for read-only forensics. The exact root cause remains unresolved.
