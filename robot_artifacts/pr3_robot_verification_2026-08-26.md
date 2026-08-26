# PR #3 robot verification record: x3-c

Status: in progress; autostart and PR readiness remain blocked.

Next session: follow
[`docs/robot_side_next_moves.md`](../docs/robot_side_next_moves.md) from a clean
current PR head. Resolve the charged, lifted-wheel gate before any further floor
trial.

## Test record

- Robot identifier: `x3-c`, Raspberry Pi 5 Model B Rev 1.0.
- Tester: Codex robot-side session; human observer pending.
- Date: 2026-08-26 UTC. The operator initially reported that the robot was
  lifted, then corrected the record after testing: the robot remained on the
  floor for every motion command. Exact surface, payload, stop path, and
  observer still need confirmation. Recorded controller voltage was only
  `10.2` to `10.3` V. The operator explicitly elected to proceed with the later
  lifted retry despite that voltage.
- Repository branch and commit: `platform/simulator-parity` at
  `1bdb7a77851d938d1111d134d49b67e7f389d6e1`.
- Draft PR: #3, "Refactor X3 into a simulator-parity hardware platform".
- `Rosmaster_Lib`:
  `/usr/lib/python3/dist-packages/Rosmaster_Lib/Rosmaster_Lib.py`, SHA256
  `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`.
- Astra discovery: model `Astra`, OpenNI URI `2bc5/060f@3/11`, serial
  `ACRC64300ET`. The camera also exposes USB RGB function `2bc5:050f` with USB
  serial `SN0001`.
- Motor controller: `1a86:7523`, no USB serial, dedicated physical topology
  `1-1.2`; current by-path link
  `/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1.2:1.0-port0`.
- LiDAR adapter: `10c4:ea60`, serial `0001`, physical topology `3-2`; current
  by-id link
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
- Proposed manual-launch environment after host udev installation:
  `ROSMASTER_MOTOR_PORT=/dev/robot/motor`,
  `ROSMASTER_LIDAR_PORT=/dev/robot/lidar`, and
  `ROSMASTER_ASTRA_SERIAL=ACRC64300ET`.
- Isolated build/test evidence:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/` (ephemeral; archive before handoff).
- External consumer repository and commit: pending.

## Safety and legacy-state audit

- Found `/root/auto_start.sh` running the August 16 generated install on ROS
  domain 11. It started the pre-cleanup bringup, duplicate robot state
  publishers, EKF odometry, and a LiDAR launch using `/dev/ttyUSB1`.
- The legacy process group was stopped with SIGINT. A subsequent domain-11 graph
  query returned no nodes.
- The filesystem still contains enabled `my_ros_service.service` and
  `supervisor.service` configurations, but this container does not run systemd
  or Supervisor. The external container launcher can still invoke
  `/root/auto_start.sh`; persistent disablement must be verified before reboot.
- The final strict bringup and all command publishers were stopped after the
  recorded tests. The driver watchdog and shutdown paths commanded zero.

## Exact source, dependencies, build, and tests

- The physical repository reports exactly eight local packages.
- Imported `ros2_astra_camera` at pinned commit
  `f7e71d9ce806e788cb48d8580aac2c778fba4214`; the full workspace reports ten
  packages including `astra_camera` and `astra_camera_msgs`.
- Initial `rosdep check` incorrectly passed because the pinned upstream Astra
  manifest omits system build dependencies. A clean build then failed at
  `pkg_search_module(LIBUVC REQUIRED libuvc)`.
- Updated `yahboomcar_astra/package.xml` to declare the upstream dependency
  bundle. `rosdep install` installed `libgflags-dev`, `libgoogle-glog-dev`,
  `libusb-1.0-0-dev`, and `libuvc-dev`; the final `rosdep check` passes.
- Clean isolated build result: all ten packages finished successfully. Expected
  upstream compiler and setuptools deprecation warnings were recorded; there
  were no build failures.
- Required local test result: 65 tests, 0 errors, 0 failures, 3 intentional
  skips. The teleop ROS test was rerun on isolated ROS domain 99 because the
  filesystem/network sandbox denied socket creation on the first attempt.

## Device stabilization

- Proposed robot rules are recorded in `99-rosmaster-x3.x3-c.rules`.
- The pinned Orbbec rules contain entries for both observed product IDs
  (`2bc5:060f` and `2bc5:050f`).
- Host installation, udev reload, reconnect, and post-reboot alias verification
  remain pending because this session is inside the ROS container and has no
  host `udevadm`.

## Remaining mandatory gates

- The operator confirmed all four wheels were lifted for the retry; exact
  restraint, observer, and stop path still need recording. Surface and payload
  also remain unrecorded.
- Persistently disable the external legacy autostart path.
- Install/reload the staged motor/LiDAR rules and pinned Orbbec permissions on
  the host; reconnect and reboot-verify all identities.
- Perform one-at-a-time camera, LiDAR, and motor fail-closed checks.
- Review the uneven lifted-wheel response and weak yaw before floor acceptance;
  complete the joystick, keyboard, and inert-calibration operator-tool tests.
- Complete bounded repeated floor trials.
- Run one unchanged external consumer against simulator commit `772ba25` and
  this physical robot.
- Archive evidence and obtain a second review.

The pre-existing `robot_artifacts/x3_lifted_probe` bag is dated 2026-08-16 and
contains the historical `/odom_raw` contract. It is not evidence for this PR's
current `/odom` acceptance gate.

## Strict bringup attempt 1

- Strict bringup used the clean isolated install and the recorded motor, LiDAR,
  and Astra identities. The robot was stationary on the floor; this attempt had
  no command publisher and sent no motion command.
- Motor, IMU filtering, encoder odometry, LiDAR, and Astra depth started. LiDAR
  reported health OK and device serial
  `94BCECF0C3E09ED2A0EA98F307644110`.
- The camera adapter failed closed after 20 seconds because color, color camera
  info, and the XYZRGB cloud were absent. The enclosing launch shut down every
  required process; no `/cmd_vel` publisher or motion command was started.
- Root cause: this camera exposes OpenNI depth on `2bc5:060f` and RGB on a
  separate UVC function `2bc5:050f`, but `astra_platform.launch.py` had not
  enabled the pinned driver's UVC path. The launch was updated to match the
  pinned upstream Astra Pro Plus configuration. A clean retry is pending.
- Launch logs:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-19-51-35-831984-x3-c-30566/`.

## Strict bringup attempt 2

- UVC color and valid color camera info passed strict startup after configuring
  the confirmed `2bc5:050f` interface.
- The adapter still failed closed because the upstream XYZRGB cloud processor
  never received color camera info. The driver reported a reliability-QoS
  incompatibility on `/_hardware/astra/color/camera_info`.
- Root cause: the upstream XYZRGB processor uses its misleadingly named
  `depth_camera_info_qos` parameter for the color camera-info subscription. It
  remained reliable while the UVC publisher correctly offered sensor-data
  best-effort QoS. The strict launch now sets that processor input to
  `sensor_data`; another clean retry is pending.
- Launch logs:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-19-54-30-483825-x3-c-33030/`.

## Strict bringup attempts 3 through 5

- Attempt 3 stayed up with every normalized stream present after the camera-info
  QoS fix, but the contract probe received only 2 of 5 required colored clouds
  in 35 seconds. At 640x480@30, the adapter used about 113% CPU and the native
  camera process about 74%. Measured cloud cadence was approximately 0.1 Hz.
  Logs:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-19-56-16-560690-x3-c-34636/`.
- Attempts 4 and 5 confirmed that generated ROS Python byte-array setters and
  continuous full-resolution cloud transforms saturated the single-threaded
  adapter. Avoiding per-byte property validation was necessary but did not make
  640x480@30 sustainable. No motion command was sent during either attempt.
  Logs:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-20-04-07-298363-x3-c-40750/`
  and
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-20-06-39-026715-x3-c-42741/`.

## Strict non-motion gate: pass

- Attempt 6 used the Astra's native aligned 320x240@30 depth and UVC RGB modes.
  The normalized adapter fell to about 42% CPU and the native driver to about
  27%, from roughly 131% and 101% at 640x480 during the preceding attempt.
- The physical contract passed in approximately 2.5 seconds with five samples
  each from scan, IMU, calibrated RGB8 color, metric 32FC1 depth, color/depth
  camera info, XYZRGB points in `cam_1_depth_frame`, joint states, odometry, and
  dynamic TF; it received both static TF samples and five diagnostics.
- The passing probe also verified expected topic types, exactly one publisher
  for each unique public stream, no default `/cmd_vel` publisher, healthy motor
  and odometry diagnostics, valid finite messages, required wheel names,
  canonical frames, and timestamped transforms from `odom` to every sensor.
- The two `/tf_static` publishers were confirmed as `robot_state_publisher` and
  `/_hardware/astra/camera`. The probe's transient-local reader history was
  increased from 1 to 100 so both writers' latched samples are retained.
- Passing launch logs:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-20-09-48-287465-x3-c-45110/`.
- Probe result:
  `Physical contract PASSED: /scan=5, /imu/data=5,`
  `/cam_1/color/image_raw=5, /cam_1/depth/image_raw=5,`
  `/cam_1/color/camera_info=5, /cam_1/depth/camera_info=5,`
  `/cam_1/depth/color/points=5, /joint_states=5, /odom=5, /tf=5,`
  `/tf_static=2, /diagnostics=5`.

## Motion evidence: robot was on the floor

- Primary bag:
  `robot_artifacts/x3_lifted_pr3_2026-08-26/`.
- The directory name reflects the test condition reported at the time and is
  incorrect. The operator later confirmed the robot was on the floor. This bag
  is not evidence for the lifted gate, and it is not sufficient for the floor
  gate because the trials were not repeated and had no external measurements.
- The bounded pulse tool required the recorder, found only `driver_node` as an
  actuator subscriber, rejected competing publishers, and sent repeated zeroes
  after every one-second pulse.
- Forward `+X=0.10`: wheel signs were `+ + + +` and odometry moved `+X`, but
  wheel deltas were uneven (`+0.7008, +1.3412, +0.6767, +1.3291` rad) and yaw
  drift was `+0.0646` rad.
- Left strafe `+Y=0.10`: wheel signs were the expected `- + + -` and odometry
  moved `+Y`, but the rear-wheel response was weak; deltas were
  `-0.7552, +0.9123, +0.1269, -0.0121` rad and yaw drift was `+0.0764` rad.
- CCW `+yaw=0.30`: the prescribed floor pulse did not produce a usable rotation;
  deltas were `-0.0060, +0.0181, +0.0000, +0.0121` rad and odometry yaw changed
  only `+0.0018` rad. This gate is a failure and must be repeated after charging
  the battery; no geometry, CPR, scale, covariance, or encoder mapping was
  changed from this single run. The lifted motion gate has not been attempted.
- Voltage during every primary trial was `10.2` to `10.3` V. This is recorded as
  a likely test-condition contributor, not yet established as the root cause.
- Separate watchdog bag:
  `robot_artifacts/x3_watchdog_pr3_2026-08-26/`.
- One `+X=0.05` message was published with no final zero. The driver logged
  `cmd_vel timeout after 0.522s; commanding zero velocity`; only negligible
  movement followed (`+0.0002` m odometry X, about `0.0121` rad on FL and BR),
  and hardware diagnostics remained healthy. The watchdog behavior passes
  under this floor load, but it must remain part of the eventual lifted and
  bounded floor acceptance sequence.

## Confirmed lifted retry

- Before this retry, the operator explicitly confirmed the robot was lifted and
  instructed testing to continue despite the recorded low voltage.
- Bag: `robot_artifacts/x3_lifted_retry_pr3_2026-08-26/`.
- Launch logs:
  `/tmp/physical_rosmaster_pr3_1bdb7a7/roslog/2026-08-26-20-20-27-735191-x3-c-53841/`.
- The bounded safety tool again required the recorder, verified `driver_node` as
  the only actuator subscriber, found no competing `/cmd_vel` publisher, and
  sent repeated zero commands after each normal pulse.
- Forward `+X=0.10`: signs passed as `+ + + +`; wheel deltas were
  `+1.1962, +2.9181, +0.7491, +1.5466` rad and odometry X increased by
  `+0.0526` m. Uneven wheel response produced `+0.1260` rad yaw drift.
- Left strafe `+Y=0.10`: signs passed as `- + + -`; wheel deltas were
  `-1.3352, +2.5676, +0.1390, -1.0271` rad and odometry Y increased by
  `+0.0407` m. Uneven response produced `+0.1368` rad yaw drift.
- CCW `+yaw=0.30`: signs passed as `- + - +` and odometry yaw increased by
  `+0.0124` rad. Wheel deltas were only
  `-0.1390, +0.0906, -0.0060, +0.0121` rad, so direction passes but weak yaw
  remains a review item.
- A single `+X=0.05` watchdog command was sent without a final zero. The driver
  logged `cmd_vel timeout after 0.509s; commanding zero velocity`; the bag shows
  negligible post-command displacement and repeated healthy controller and
  wheel-odometry diagnostics.
- No stale encoder, discontinuity, non-finite, read-failure, or required-motor
  error was present in the bag. Voltage remained `10.2` to `10.3` V.
- The full physical contract passed again after motion with all message counts,
  TF checks, graph checks, and diagnostics intact.
- On shutdown, the motor driver and adapter exited cleanly and the driver sent
  three zero commands. The pinned upstream Astra driver still prints parameter
  undeclare errors during teardown, and the SLLidar process required launch's
  SIGTERM escalation after its five-second SIGINT grace period; both should be
  tracked as shutdown-quality review items.
