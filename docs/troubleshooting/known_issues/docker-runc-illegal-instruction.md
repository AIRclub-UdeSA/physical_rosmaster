# Docker runc Illegal Instruction

## Symptom

Docker reports an OCI runtime failure and containers do not start. Running `runc --version` prints `Illegal instruction` and exits with code `132`.

## Affected Environment

- Raspberry Pi host running Docker
- Physical ROSMASTER container such as `rosmaster_humble`

## Safety / Data-Loss Warning

Do not delete the ROS workspace or rebuild containers before checking the host runtime. This failure can be entirely host-side.

## Fast Diagnosis

On the Raspberry Pi host:

```bash
runc --version
dpkg -V containerd.io
```

If `runc --version` crashes, inspect the host package that provides `/usr/bin/runc`.

## Distinguishing Evidence

- `containerd --version` still works
- A brand-new test container fails the same way
- `dpkg -V containerd.io` reports a checksum mismatch for `/usr/bin/runc`

## Root Cause

The Docker-bundled `runc` binary on the Raspberry Pi host was corrupted or modified.

## Fix

On the Raspberry Pi host:

```bash
sudo apt-get install --reinstall containerd.io=1.7.25-1
sudo systemctl restart docker
```

## Verification

On the host:

```bash
runc --version
docker run --rm yahboomtechnology/ros-humble:4.1.2 /bin/true
```

Both should exit cleanly.

## What Not To Do

- Do not assume the ROS container is corrupted.
- Do not wipe the workspace first.
- Do not keep retrying container rebuilds until the host runtime is fixed.

## Prevention / Hardening

- Verify host package integrity when OCI runtime failures appear.
- Keep host repair steps separate from container repair steps.

## Affected Robot / Incident

- `x3-c`
- 2026-08-16 setup incident

## Status

Fixed on the host during the incident.