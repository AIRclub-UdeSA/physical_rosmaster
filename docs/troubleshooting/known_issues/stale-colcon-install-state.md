# Stale Colcon Install State After A Failed Package Build

## Symptom

`source /root/yahboomcar_ws/install/setup.bash` warns that a package setup file is missing, such as `yahboomcar_slam/share/yahboomcar_slam/local_setup.bash`.

## Affected Environment

- Workspace that previously had a failed package build
- Generated `build/`, `install/`, and `log/` state still present

## Safety / Data-Loss Warning

Only remove generated workspace state. Do not remove the source tree unless you have another backup.

## Fast Diagnosis

If a package build failed earlier, inspect the generated workspace state and rebuild it cleanly:

```bash
cd /root/yahboomcar_ws
rm -rf build install log
```

## Distinguishing Evidence

- The missing file lives in `install/`, not in the source tree
- The warning appears after a previous build failure
- Rebuilding from a clean generated state clears the warning

## Root Cause

Colcon preserved stale generated state from the failed package build.

## Fix

Remove the generated directories and rebuild the workspace.

```bash
cd /root/yahboomcar_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip yahboomcar_slam
source install/setup.bash
```

## Verification

`source install/setup.bash` should complete without the stale package warning.

## What Not To Do

- Do not assume the source tree is broken.
- Do not keep re-sourcing a stale `install/` tree after a failed build.

## Prevention / Hardening

- Rebuild generated state after a failed package build.
- Keep the optional-artifact issue fixed so the build can finish cleanly.

## Affected Robot / Incident

- `x3-c`
- 2026-08-16 setup incident

## Status

Fixed by a clean rebuild during the incident.