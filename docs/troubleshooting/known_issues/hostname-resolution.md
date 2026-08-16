# Hostname Resolution Warning On The Raspberry Pi Host

## Symptom

Commands run with `sudo` print `unable to resolve host x3-c: Name or service not known`.

## Affected Environment

- Raspberry Pi host, not the ROS container

## Safety / Data-Loss Warning

This is a host identity issue. Fix it on the Raspberry Pi host, not inside the Docker container.

## Fast Diagnosis

On the host:

```bash
hostname
cat /etc/hosts
```

If the hostname is `x3-c` but `/etc/hosts` lacks a matching `127.0.1.1` entry, the warning is expected.

## Distinguishing Evidence

- The warning appears from `sudo`
- Docker and ROS can still work normally
- The issue is tied to host name resolution, not the robot packages

## Root Cause

The Raspberry Pi host `/etc/hosts` does not map the current hostname to `127.0.1.1`.

## Fix

On the Raspberry Pi host, edit `/etc/hosts` and add or correct the hostname entry:

```text
127.0.1.1 x3-c
```

## Verification

Run a new `sudo` command and confirm the warning is gone.

## What Not To Do

- Do not change the hostname inside the container as a substitute for fixing the host.
- Do not ignore the warning if you are already editing host configuration.

## Prevention / Hardening

- Keep host hostname and `/etc/hosts` in sync after any rename.

## Affected Robot / Incident

- `x3-c`
- 2026-08-16 setup incident

## Status

Open at handoff time; fix on the Raspberry Pi host.