# X3-C Coding Agent Handoff — 2026-08-16

## Purpose

This file is the handoff for a coding agent working directly on the physical Yahboom ROSMASTER X3 robot `x3-c`.

It has two goals:

1. Continue the physical robot setup and validation from the exact state reached on 2026-08-16.
2. Turn the problems discovered during setup into reusable troubleshooting documentation under `docs/troubleshooting/`, so future robots can be diagnosed from symptoms instead of rediscovering the same fixes.

---

## Coding Agent Mission

You are working directly inside the physical ROSMASTER X3 robot container.

### Environment

- Robot hostname: `x3-c`
- Raspberry Pi host user: `pi`
- Robot IP used during setup: `192.168.0.90`
- Docker container: `rosmaster_humble`
- Docker image: `yahboomtechnology/ros-humble:4.1.2`
- ROS distro: ROS 2 Humble
- `ROS_DOMAIN_ID=11`
- ROS workspace: `/root/yahboomcar_ws`
- Repository: `/root/yahboomcar_ws/src/physical_rosmaster`
- Git branch at clone time: `main`
- Clone commit observed during setup: `407d9f6` (`docs: rewrite standalone readme`)
- VS Code workflow: personal computer -> Remote SSH to `pi@192.168.0.90` -> Dev Containers attach to `rosmaster_humble` -> open `/root/yahboomcar_ws/src/physical_rosmaster`

### Read before changing anything

Start by reading:

- `context.md`
- `README.md`
- `docs/`
- `agents/`
- this handoff file

Then run:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git status --short
git log -1 --oneline
```

Do not overwrite robot-side local edits. Do not discard uncommitted changes.

---

## Safety Rules

- Do not publish nonzero `/cmd_vel` until the user confirms the robot is lifted or in a safe open area.
- Always send a zero `/cmd_vel` after any motion test.
- Do not enable autostart until manual robot bringup has been validated.
- Do not vendor `Rosmaster_Lib` into this repo.
- Do not commit `build/`, `install/`, `log/`, ROS bags, caches, `ORBvoc.txt`, or `.pcd` files.
- Treat host-level fixes separately from container-level fixes. If something must be changed on the Raspberry Pi host (`/etc/hosts`, systemd, Docker), tell the user explicitly to run it in a host terminal rather than pretending it can be completed from the container.

Emergency stop command:

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

# Session Summary

## 1. Existing robot/container state

The robot already had:

- Docker Engine 27.5.0
- container `rosmaster_humble`
- image `yahboomtechnology/ros-humble:4.1.2`
- host `Rosmaster_Lib` under `/home/pi/software/py_install/Rosmaster_Lib`
- an existing Yahboom ROS workspace in `/root/yahboomcar_ws`

The Raspberry Pi root filesystem was approximately 93% full, with roughly 2.6-2.7 GB free.

A large crash dump existed at:

```text
/root/yahboomcar_ws/core
```

It was about 679 MB and was identified as disposable crash output rather than ROS source.

---

## 2. Docker/runc failure discovered and repaired

### Symptom

The existing container would not start.

Docker reported an OCI runtime failure and the container remained exited.

Direct test:

```bash
runc --version
```

returned:

```text
Illegal instruction
```

with exit code `132`.

`containerd --version` worked normally.

A brand-new test container using the same image and `/bin/true` failed in the same way, proving the old ROSMASTER container itself was not the root cause.

### Diagnosis

The host was confirmed as `aarch64/arm64`, and the Docker image was also `arm64`, so this was not a platform mismatch.

`/usr/bin/runc` was owned by:

```text
containerd.io
```

The installed package version was:

```text
containerd.io 1.7.25-1
```

Package verification:

```bash
dpkg -V containerd.io
```

reported a checksum mismatch specifically for:

```text
/usr/bin/runc
```

### Root cause

The Docker-bundled `runc` binary on the Raspberry Pi host had been corrupted or modified.

### Fix used

On the Raspberry Pi host:

```bash
sudo apt-get install --reinstall containerd.io=1.7.25-1
sudo systemctl restart docker
```

After reinstall:

```bash
runc --version
```

reported runc 1.2.4 and exited successfully.

A fresh test container then exited with code 0, and `rosmaster_humble` started normally again.

### Important lesson

When Docker reports OCI runtime failures and `runc --version` itself crashes, diagnose the host runtime before deleting containers or rebuilding the ROS workspace.

Do not assume the container is corrupted.

---

## 3. Hostname resolution warning remains

The Raspberry Pi host repeatedly prints:

```text
sudo: unable to resolve host x3-c: Name or service not known
```

This was intentionally not mixed into the Docker repair.

Likely host-side cause: `/etc/hosts` does not contain the correct `127.0.1.1 x3-c` mapping.

This remains pending and should be documented/fixed separately.

---

## 4. Migrated old flat workspace to the Git repository

The old workspace had Yahboom packages directly under:

```text
/root/yahboomcar_ws/src/
```

The source was temporarily renamed to:

```text
/root/yahboomcar_ws/src.before_physical_rosmaster_2026-08-16_152011
```

A clean Git clone was created at:

```text
/root/yahboomcar_ws/src/physical_rosmaster
```

The clone was clean and on `main`.

### Duplicate-package problem

A normal:

```bash
colcon build --symlink-install
```

initially failed because `colcon` recursively discovered both:

```text
/root/yahboomcar_ws/src.before_physical_rosmaster_.../<package>
```

and:

```text
/root/yahboomcar_ws/src/physical_rosmaster/<package>
```

This produced duplicate package name errors for essentially the whole workspace.

### Resolution

The previous source tree was removed from the colcon workspace after the new repository was verified.

### Important lesson

Do not store a backup source tree underneath the colcon workspace root if it contains ROS packages.

For future backups prefer something outside `/root/yahboomcar_ws`, for example:

```text
/root/backups/yahboomcar_ws_src_<timestamp>
```

---

## 5. Installed `Rosmaster_Lib` was physically verified

Inside the container, the actual installed library is:

```text
/usr/lib/python3/dist-packages/Rosmaster_Lib/Rosmaster_Lib.py
```

The repository probe:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --hash-only
```

returned:

```text
source_file: /usr/lib/python3/dist-packages/Rosmaster_Lib/Rosmaster_Lib.py
sha256: e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c
size_bytes: 57997
version_line: # V3.3.9
matches_public_v3_3_9: true
```

This closes a previously open repository question for robot `x3-c`: its deployed `Rosmaster_Lib` exactly matches the public V3.3.9 reference already documented under `agents/`.

Any docs/TODO/context text that still says the exact installed library has not been verified is now stale and should be updated.

---

## 6. Clean clone build exposed an optional-SLAM build bug

A full build after removing the duplicate source tree built 18 packages successfully.

Only:

```text
yahboomcar_slam
```

failed.

Failure:

```text
ament_cmake_symlink_install_directory() can't find
'/root/yahboomcar_ws/src/physical_rosmaster/yahboomcar_slam/pcl'
```

The repository documentation says the large PCD/ORB-SLAM artifacts are intentionally optional and excluded from Git.

Therefore the current clean-clone behavior contradicts the documented contract: a normal full workspace build should not require optional artifacts if normal robot bringup does not require them.

### Temporary workaround used

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip yahboomcar_slam
```

Result:

```text
18 packages finished
```

Core packages resolve correctly:

```bash
ros2 pkg prefix yahboomcar_bringup
ros2 pkg prefix yahboomcar_base_node
ros2 pkg prefix sllidar_ros2
```

### Required code/repo fix

Inspect `yahboomcar_slam/CMakeLists.txt` and any package install logic.

Make missing optional artifact directories safe on a fresh clone.

Preferred behavior:

- core workspace should build from Git without fetching optional PCD/ORB-SLAM assets
- optional directories/files should only be installed when they exist
- downloading `tools/fetch_large_artifacts.sh` should only be necessary when actually using the related SLAM/point-cloud workflows
- update docs if any package must intentionally be excluded from the default build

Do not create fake large artifacts merely to satisfy CMake.

---

## 7. Stale install state after the failed SLAM build

After the failed full build and subsequent `--packages-skip yahboomcar_slam` build, sourcing:

```bash
source /root/yahboomcar_ws/install/setup.bash
```

still printed a warning similar to:

```text
not found: "/root/yahboomcar_ws/install/yahboomcar_slam/share/yahboomcar_slam/local_setup.bash"
```

This is stale generated state from the earlier failed package.

A clean rebuild was recommended:

```bash
cd /root/yahboomcar_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip yahboomcar_slam
source install/setup.bash
```

At handoff time, verify whether this cleanup has already been completed. If sourcing still prints the stale `yahboomcar_slam` warning, perform the clean generated-state rebuild.

---

## 8. Autostart status

During the Docker runtime repair the host autostart service was stopped with:

```bash
sudo systemctl stop rosmaster-autostart.service
```

Do not re-enable/restart autostart yet.

The repository workflow says manual robot bringup should pass before autostart is enabled against the new Git-based workspace.

Autostart validation is a later host-side step.

---

# Troubleshooting Documentation Goal

Create and maintain a troubleshooting knowledge base inside this repository.

Recommended structure:

```text
docs/
└── troubleshooting/
    ├── README.md
    ├── incidents/
    │   └── 2026-08-16-x3-c-setup.md
    └── known_issues/
        ├── docker-runc-illegal-instruction.md
        ├── colcon-duplicate-packages.md
        ├── slam-missing-optional-artifacts.md
        ├── stale-colcon-install-state.md
        └── hostname-resolution.md
```

The exact filenames can be adjusted if a clearer repository convention emerges.

## Purpose of `docs/troubleshooting/`

This documentation should answer:

> "I see symptom X on a physical ROSMASTER robot. What should I inspect first, how do I prove the root cause, how do I fix it safely, and how do I verify the repair?"

It should complement, not duplicate, the setup guide.

The setup guide describes the expected happy path.

The troubleshooting docs should preserve failures, diagnosis techniques, recovery procedures, and verification commands learned from real robots.

## Recommended known-issue format

Each reusable issue should contain:

1. **Symptom**
2. **Affected environment**
3. **Safety / data-loss warning**
4. **Fast diagnosis**
5. **Evidence that distinguishes this issue from similar issues**
6. **Root cause**
7. **Fix**
8. **Verification**
9. **What not to do**
10. **Prevention / hardening**
11. **Incident references / affected robots**
12. **Status** — open, workaround, fixed in repo, or external dependency

For physical robots, always distinguish:

- Raspberry Pi host commands
- Docker container commands
- workstation/personal-computer commands

Never leave the execution context ambiguous.

---

# Immediate Work Plan

Proceed in this order.

## Step 1 — Inspect current state

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git status --short
git log -1 --oneline
```

Also:

```bash
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If sourcing prints the stale `yahboomcar_slam` setup warning, clean `build/ install/ log/` and rebuild the 18 normal packages before continuing.

## Step 2 — Create troubleshooting documentation

Create the `docs/troubleshooting/` structure.

Use this session summary as the source for the first incident and known-issue pages.

Do not silently invent facts that were not observed.

## Step 3 — Reconcile stale repository docs

Update relevant docs/agent notes so they reflect what is now physically verified on `x3-c`, especially:

- installed `Rosmaster_Lib` exactly matches public V3.3.9
- clean-clone build currently exposes the optional `yahboomcar_slam/pcl` assumption
- 18 normal packages build successfully when `yahboomcar_slam` is skipped
- host autostart is intentionally not yet validated against the new workspace

## Step 4 — Fix the optional artifact build problem

Make the repository build behavior match its documented optional-artifact contract.

Run the appropriate build/tests after changes.

Do not download the large artifacts merely to hide the clean-clone bug.

## Step 5 — Stationary hardware probe

Before commanding movement, with the robot stationary:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 100 --period 0.1
```

Capture/report:

- motion values
- four encoder counters
- whether counters remain stable while stationary
- battery voltage if reported
- any serial errors

Update `docs/odometry_validation.md`, troubleshooting docs, or agent notes only with observed results.

## Step 6 — Continue documented hardware validation

Follow `docs/odometry_validation.md`.

Before any nonzero `/cmd_vel`, explicitly ask the user to confirm that the robot is lifted or in a safe test area.

Do not enable autostart until manual bringup is known-good.

---

# Definition of Done for This Handoff

A useful completion of this handoff should leave the repo with:

- clean and accurate robot setup/context docs
- the `x3-c` V3.3.9 verification recorded
- a reusable `docs/troubleshooting/` knowledge base seeded from this incident
- the clean-clone optional SLAM artifact build issue fixed or explicitly/accurately documented
- no stale generated workspace state
- stationary encoder/motion probe results recorded
- a clear next hardware-validation step
- autostart still deferred until manual bringup passes
# Historical evidence notice

> This handoff describes the pre-cleanup workspace and deployment. Use it for incident history only; current setup and autostart gates are in `docs/`.
