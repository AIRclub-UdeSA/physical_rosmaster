# Host storage integrity and safe shutdown

## Symptom

A Raspberry Pi robot host may show one or more of the following:

- a dirty boot filesystem or recovery after an improper unmount;
- SD/MMC, filesystem, or I/O warnings;
- a core host binary that crashes with SIGILL;
- Docker or another host service that stops working despite unchanged robot
  application code;
- an early boot failure even though Raspberry Pi bootloader diagnostics work.

These symptoms overlap. They do not, by themselves, identify a failed microSD,
bad power, package corruption, or any other single root cause.

## Affected environment

- Raspberry Pi hosts that boot from microSD
- Physical ROSMASTER deployments that run ROS inside Docker
- Hosts exposed to operator-controlled main power, battery voltage variation,
  and write-heavy builds or image pulls

## Safety / data-loss warning

- For routine shutdown, run `sudo poweroff`, wait for Linux to halt, and only
  then switch off main robot power.
- Do not use a hardware reset or immediate power removal as a normal Linux
  reboot procedure.
- Do not repair, format, or reflash suspect media before preserving the evidence
  needed for recovery.
- Verify the exact block-device identity and unmount its partitions before
  filesystem checks. Start with read-only options.
- Keep full-card images immutable during forensics. Work from read-only mounts
  or verified copies.

## Fast diagnosis

### 1. Capture the current state

When the host still boots, record:

```bash
date --iso-8601=seconds
uname -a
df -h /
docker system df
systemctl --failed
journalctl -k -b --no-pager
```

Record battery voltage when the hardware driver is available. Preserve exact
error text, timestamps, package versions, and deployed revisions.

### 2. Identify the failing layer

- If the OS display is black, test Raspberry Pi bootloader diagnostics without
  the system card.
- If a command receives SIGILL, identify its path and owning package:

  ```bash
  command -v <command>
  dpkg -S /absolute/path/to/binary
  dpkg --verify <package>
  ```

- If the host cannot boot, inspect the card from another Linux system and
  compare `cmdline.txt`, `/etc/fstab`, UUIDs, and PARTUUIDs.
- Use independent known-good boot media early when core host userspace is
  implicated. This separates an installation/media state from a deterministic
  hardware incompatibility more effectively than repeated retries.

### 3. Check filesystems without repairing first

After confirming that the target partitions are unmounted:

```bash
sudo fsck.fat -n /dev/<boot-partition>
sudo e2fsck -fn /dev/<root-partition>
```

Preserve a full image before allowing repairs when incident evidence matters.
A clean structural check does not verify the contents of every file.

### 4. Review persistent evidence

Inspect available current and previous-boot records for:

- `mmc`, timeout, I/O, ext4, FAT, and recovery messages;
- voltage or undervoltage events;
- package install/upgrade interruptions;
- unexpected resets and unclean shutdowns;
- core dumps from the failing process.

Absence of a log entry is not proof that an event did not occur, especially
when persistent journaling was not enabled or storage writes were interrupted.

## Distinguishing evidence

- Bootloader diagnostics working without the system card establishes a narrower
  boundary than a black HDMI screen alone.
- A package checksum mismatch identifies a changed packaged file, but it does
  not automatically identify why the file changed.
- A clean filesystem structure does not prove every file is semantically valid.
- Successful operation from independent known-good media strongly distinguishes
  host installation/media state from a repeatable CPU instruction-set problem.
- Reflashing and recovering establishes a recovery method, not the initiating
  cause.

## Root cause

There is no single root cause for this issue category. Possible causes include
unclean shutdown, unstable power, failing or marginal media, interrupted writes,
package corruption, and unrelated hardware or software faults. Assign a root
cause only when incident-specific evidence supports it.

## Fix

Choose the narrowest evidence-backed recovery:

1. Preserve non-reconstructible data and relevant logs.
2. Create and hash a full-card image when the recovery cost or forensic value
   warrants it.
3. Repair a verified damaged package when package checks identify the exact
   file.
4. Repair filesystems only after imaging and only against the confirmed device.
5. Reflash from a verified factory image when the installation cannot be
   trusted or repaired reproducibly.
6. Restore application dependencies, container image, repository revision, and
   device configuration as separate verified layers.

Do not restore an entire untrusted host state over a clean recovery image.

## Verification

After recovery, record and verify:

- normal host boot and clean service state;
- root partition size and free space;
- absence of new SD/MMC, I/O, or filesystem warnings;
- exact factory-image checksum when available;
- exact Docker image tag and digest;
- hardware-library hash and repository commit;
- stable device identities and permissions;
- graceful shutdown and subsequent clean boot.

Application and robot-motion acceptance remain separate from host recovery.

## What not to do

- Do not repeatedly hard-reset a failing Linux system.
- Do not infer a dead Raspberry Pi from HDMI behavior alone.
- Do not infer physical media failure from a dirty filesystem alone.
- Do not infer that low voltage caused an incident merely because low voltage
  was observed at another time.
- Do not treat `fsck`, package verification, or a successful reflash as proof
  beyond what each test directly establishes.
- Do not delete old images or logs before hashes, provenance, and retention
  decisions are recorded.

## Prevention or hardening

### Routine shutdown

```bash
sudo poweroff
```

Wait for the Pi to halt before switching off the ROSMASTER main power. Document
this as an operator requirement.

### Maintenance preflight

Before builds, package changes, large Docker pulls, or other write-heavy work:

- record battery voltage and do not proceed when supply health is low or
  uncertain;
- check `df -h /` and `docker system df`;
- confirm a factory-imaged root partition has been expanded;
- make a recovery backup proportional to the cost of reconstruction.

Treat roughly 85–90% root usage as an investigation threshold rather than a
normal operating target.

### Reproducible recovery metadata

Maintain a non-secret record of:

- factory image filename and checksum;
- Docker image tag and digest;
- `Rosmaster_Lib` or equivalent hardware-library hash;
- deployed repository branch and commit;
- udev rules and stable hardware identities;
- required host configuration without credentials.

### Media and health reporting

- Consider high-endurance microSD media for production robots as risk reduction,
  not as a diagnosis.
- Consider a small startup health record that captures root usage, battery
  voltage when available, recent storage/I/O warnings, signs of filesystem
  recovery, and deployed identifiers.
- Preserve persistent journals or export relevant logs to durable storage when
  operational policy permits it.

## Affected robot or incident

- [`x3-c` Docker `runc` SIGILL, 2026-08-16](docker-runc-illegal-instruction.md)
- [`x3-c` host `systemd` SIGILL, 2026-08-29/30](../incidents/2026-08-29-x3-c-host-systemd-sigill.md)

The two incidents justify the same preventive controls but are not proven to
share an initiating cause.

## Status

Reusable guidance. Apply it alongside incident-specific evidence and the
current robot acceptance runbook.
