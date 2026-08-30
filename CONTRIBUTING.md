# Contributing to physical_rosmaster

Thanks for being here. `physical_rosmaster` is the ROS 2 Humble source
workspace that runs on the real Yahboom ROSMASTER X3 mecanum robots used by
AIR Club UdeSA — the physical counterpart to the
[`yahboom_rosmaster`](https://github.com/AIRclub-UdeSA/yahboom_rosmaster)
simulator. Changes here move real motors on real hardware, so it gets better
when more people run it, validate it carefully, and document what they find.

## Getting set up

There are two separate contexts, and most contributions only need the first:

- **Workstation** — source review, Git work, documentation, hardware-free
  tests:

  ```bash
  mkdir -p ~/rosmaster_physical_ws/src
  cd ~/rosmaster_physical_ws/src
  git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

  cd ~/rosmaster_physical_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select yahboomcar_base_node
  colcon test --packages-select yahboomcar_base_node --ctest-args -R test_x3_odometry
  ```

- **Robot** — inside the robot's `rosmaster_humble` Docker container, cloned
  into `/root/yahboomcar_ws/src/physical_rosmaster`. See
  [Quick Start: Robot](README.md#quick-start-robot) and
  `docs/setup_guide_ros2_humble_autostart.md` for the full clone/build/
  autostart procedure. Back up the existing `src` tree before replacing a
  robot workspace.

`Rosmaster_Lib` is not vendored here — on the robot it's copied in from the
Yahboom host installation.

## Good first contributions

- **Close validation gates.** The odometry path has floor-tested wiring and
  sign conventions, but a true lifted repetition with ground truth is still
  outstanding, and per-wheel magnitude/yaw bias is only provisionally
  characterized. See [Odometry Status](README.md#odometry-status),
  `agents/x3-c_validation_checklist.md`, and `docs/odometry_validation.md`
  for the open items.
- **Clean up licensing.** Several package manifests still have
  `TODO: Package description` and `TODO: License declaration` — see
  [License Caveat](README.md#license-caveat). Tracing provenance package by
  package is welcome.
- **Fix bugs.** `docs/troubleshooting/README.md` is an incident-driven index
  of known robot failure modes.
- **Work through the task list.** `agents/physical_rosmaster_todo.md` and
  `agents/rosmaster_physical_audit.md` track open work and known gaps.

## Repository Layout

Buildable packages (see [Package Inventory](README.md#package-inventory) for
the full, current list): `yahboomcar_bringup`, `yahboomcar_base_node`,
`yahboomcar_ctrl`, `yahboomcar_description`, `yahboomcar_nav`,
`yahboomcar_slam`, `sllidar_ros2`, `robot_pose_publisher`, and others.
`yahboomcar_KCFTracker`, `robot_pose_publisher_ros2`, and `yahboomcar_point`
are present but ignored via `COLCON_IGNORE`.

Read `docs/workstation_and_robot_workflow.md` before deciding whether a
change needs to be validated on the workstation, on the robot, or both.

## Coding style

- Match the existing structure and conventions of the package you're
  editing rather than inventing a new pattern.
- Prefer small, reviewable commits.
- Comment the *why* (a hardware quirk, a workaround, a tuned value) — not
  the obvious *what*.

## Pull requests

Changes to `main` must go through a pull request:

1. Create a feature branch (`git checkout -b feature/my-change`) and push it.
2. Build and test on the workstation first. If your change touches motion,
   the driver, or odometry, follow the validation procedure in
   `docs/odometry_validation.md` / `agents/x3-c_validation_checklist.md` —
   use the bounded pulse tool (`tools/safe_cmd_vel_pulse.py`) with the robot
   securely lifted, never floor-test unsupervised.
3. If you change runtime behavior (a launch arg, a topic, a driver default),
   say so explicitly in the PR description — this affects a shared physical
   robot.
4. Describe what you changed and how you verified it.
5. At least 1 approval from another contributor is required before merging.
6. Resolve all review comments before merging.
7. New commits after an approval will require re-approval.
8. Direct pushes to `main` are blocked, including for repo admins.

## Ground rules

- This repo drives a real robot. If a change could affect motor commands,
  safety limits, or timeout behavior, call that out clearly in the PR so
  reviewers know to check it carefully.
- Don't overstate validation status — if something is floor-tested but not
  ground-truth-verified, say so, the same way `docs/odometry_validation.md`
  already does.
- This repository includes Yahboom-derived source and a vendored copy of
  `sllidar_ros2`; see [License Caveat](README.md#license-caveat) before
  assuming clean licensing on anything you add or modify.
- Be decent to each other. Assume good faith, keep it constructive.
