# Troubleshooting

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
- [Docker runc illegal instruction](known_issues/docker-runc-illegal-instruction.md)
- [Root filesystem full causing LightDM login loop and Docker growth](known_issues/root-filesystem-full-login-loop.md)
- [Stale colcon install state after a failed package build](known_issues/stale-colcon-install-state.md)
- [Hostname resolution warning on the Raspberry Pi host](known_issues/hostname-resolution.md)
