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
  tests. Install `python3-vcstool` first if the `vcs` command is unavailable:

  ```bash
  mkdir -p ~/rosmaster_physical_ws/src
  cd ~/rosmaster_physical_ws/src
  git clone https://github.com/AIRclub-UdeSA/physical_rosmaster.git

  cd ~/rosmaster_physical_ws
  source /opt/ros/humble/setup.bash
  vcs import src < src/physical_rosmaster/physical_rosmaster.repos
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  ```

  Run the package gate from the workspace, followed by the standalone tool
  checks from the repository root. Package tests, including the motor runtime
  suites, run once through `colcon test`:

  ```bash
  cd ~/rosmaster_physical_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  export ROS_LOG_DIR=/tmp/physical_rosmaster-ros-logs
  mkdir -p "$ROS_LOG_DIR"
  export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  colcon test --packages-select \
    laserscan_to_point_pulisher sllidar_ros2 yahboomcar_astra \
    yahboomcar_base_node yahboomcar_bringup yahboomcar_ctrl \
    yahboomcar_description yahboomcar_visual
  colcon test-result --verbose

  cd ~/rosmaster_physical_ws/src/physical_rosmaster
  export PYTHONPATH="$PWD/yahboomcar_bringup:$PWD/tools:$PYTHONPATH"
  python3 -m pytest -q \
    tools/test_rosmaster_lib_probe.py \
    tools/test_motor_live_loss_probe.py \
    tools/test_motor_live_loss_ros_smoke.py \
    tools/test_physical_contract_probe.py \
    tools/test_safe_cmd_vel_pulse.py
  ```

  The current workstation needs plugin autoload disabled because an unrelated
  globally installed pytest plugin is incompatible with its pytest version.
  ROS tests also require a writable `ROS_LOG_DIR`.
  The live-loss ROS smoke test must run on Linux with permission to enumerate
  local network interfaces and open localhost DDS sockets.

  If the exact reviewed `Rosmaster_Lib.py` is available locally, also run its
  real-pyserial pseudo-terminal gate without copying it into the repository:

  ```bash
  export ROSMASTER_V339_SOURCE=/absolute/path/to/Rosmaster_Lib.py
  python3 -m pytest -q tools/test_rosmaster_v339_pty.py
  ```

  Every pull request runs this same build-and-test gate automatically (see
  [`.github/workflows/hardware-free-checks.yml`](.github/workflows/hardware-free-checks.yml)),
  minus `test_rosmaster_v339_pty.py` — CI never has `Rosmaster_Lib.py`
  available, since it isn't vendored here for licensing reasons. CI also
  skips vendoring `ros2_astra_camera` (`rosdep install --skip-keys
  astra_camera`): it's `exec_depend`-only for `yahboomcar_astra`, and no
  hardware-free test imports or launches it, so building the full camera SDK
  on every run would cost time without adding coverage. Import it locally
  with `vcs` as shown above if you actually need to build or run the camera
  driver.

- **Robot** — inside the robot's `rosmaster_humble` Docker container, cloned
  into `/root/yahboomcar_ws/src/physical_rosmaster`. See
  [Robot setup and manual bringup](README.md#robot-setup-and-manual-bringup) and
  [the setup guide](docs/setup_guide_ros2_humble_autostart.md) for the clone,
  build, and per-robot hardware procedure. Back up the existing `src` tree
  before replacing a robot workspace. Autostart is accepted for this stack;
  see [docs/autostart_setup.md](docs/autostart_setup.md).

`Rosmaster_Lib` is not vendored here — on the robot it's copied in from the
Yahboom host installation. Its allowlisted source hash is checked after Python
imports it. That is a compatibility/version gate, not supply-chain attestation.

## Current priorities

- **Close the remaining validation gates.** `x3-c` already passed the core
  robot deployment, startup-absence/restoration/live-loss sequence, and a
  qualitative floor pass; autostart is separately installed and
  reboot-validated. Still open: the full 3-rep measured floor protocol with
  external measurement, operator-tool (joystick/keyboard/calibration) checks,
  and a simulator/physical consumer-parity proof. Use
  [Rollout status](README.md#rollout-status),
  [the robot checklist](docs/robot_side_verification_todo.md), and
  [odometry validation](docs/odometry_validation.md).
- **Fix bugs.** `docs/troubleshooting/README.md` is an incident-driven index
  of known robot failure modes.

## Repository Layout

The repository contains exactly the eight packages in
[Retained packages](README.md#retained-packages): `yahboomcar_bringup`,
`yahboomcar_base_node`, `yahboomcar_description`, `yahboomcar_ctrl`,
`yahboomcar_astra`, `sllidar_ros2`, `yahboomcar_visual`, and the historically
spelled `laserscan_to_point_pulisher`. Removed navigation, SLAM, localization,
EKF, and application behavior are not hidden local packages; recover them from
Git history only when investigating the former architecture.

Read `docs/workstation_and_robot_workflow.md` before deciding whether a
change needs to be validated on the workstation, on the robot, or both.

## Runtime safety model

Do not infer controller liveness from cached getters. Runtime `a08b097` requires
checksum-valid `0x0A` speed/battery, `0x0D` encoder, and `0x0B`-or-`0x0E` raw-IMU
arrivals to remain fresh. A terminal feedback or observed serial-write failure
suppresses controller-derived output, nonzero motion commands, and all
RGB/buzzer requests; emits an `ERROR` diagnostic; exits the driver; and causes
the strict launch to shut down the graph.

That contract does not prove a physical stop. Host serial-write completion is
not controller acknowledgement, and zero-command attempts are not proof that
the controller executed them. Any PR that changes this path must keep those
claims separate and must defer physical-stop acceptance to a supervised robot
test using available feedback and external observation.

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
   [the robot checklist](docs/robot_side_verification_todo.md) and
   [odometry validation](docs/odometry_validation.md). Use the bounded pulse
   tool (`tools/safe_cmd_vel_pulse.py`) with the robot securely lifted; never
   floor-test unsupervised.
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
  ground-truth-verified, say so. In particular, workstation tests do not close
  the pending robot-validation gate for `a08b097`.
- This repository includes Yahboom-derived source and a vendored copy of
  `sllidar_ros2`; see
  [Provenance and licensing](README.md#provenance-and-licensing) before
  assuming clean licensing on anything you add or modify.
- Be decent to each other. Assume good faith, keep it constructive.
