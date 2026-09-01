# Troubleshooting

The dated incidents preserve pre-cleanup history. Mentions of removed SLAM packages, `/odom_raw`, EKF, old device paths, or the old autostart graph are not current platform instructions.

This directory records failures seen on physical ROSMASTER robots and the quickest way to distinguish them from similar problems.

Use the happy-path setup guide for normal installation and bringup. Use these pages when a robot is already failing and you need diagnosis, recovery, and verification steps.

## Structure

Each reusable issue page should cover:

- Symptom
- Affected environment
- Safety or data-loss warning
- Fast diagnosis
- Distinguishing evidence
- Root cause
- Fix
- Verification
- What not to do
- Prevention or hardening
- Affected robot or incident
- Status

## Current Pages

- [2026-08-16 x3-c setup incident](incidents/2026-08-16-x3-c-setup.md)
- [2026-08-29/30 x3-c host systemd SIGILL incident](incidents/2026-08-29-x3-c-host-systemd-sigill.md)
- [Motor controller has no command-loss watchdog](known_issues/motor-controller-no-link-loss-watchdog.md)
- [Docker runc illegal instruction](known_issues/docker-runc-illegal-instruction.md)
- [Host storage integrity and safe shutdown](known_issues/host-storage-integrity-and-safe-shutdown.md)
- [Root filesystem full causing LightDM login loop and Docker growth](known_issues/root-filesystem-full-login-loop.md)
- [Stale colcon install state after a failed package build](known_issues/stale-colcon-install-state.md)
- [Hostname resolution warning on the Raspberry Pi host](known_issues/hostname-resolution.md)
