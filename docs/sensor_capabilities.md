# Measured X3 sensor capabilities

What the physical platform's sensors actually deliver, measured on hardware.
[config/robot_contract.yaml](../config/robot_contract.yaml) says which
interfaces must exist; this document says how well they perform, so the
simulator can be calibrated against real behavior instead of assumptions.

Reproduce any number here with:

```bash
python3 tools/sensor_capability_probe.py --group camera --duration 30 --output camera.json
```

Run it **on the robot**. Latency is `receive clock - header.stamp`, so a
workstation with an unsynchronized clock reports clock offset, not pipeline
latency.

## Measurement record

| | |
|---|---|
| Robot | `x3-c` (host still named `yahboom`) |
| Date | 2026-09-17 |
| Commit | `01e1a60` (`main`), deployed and contract-verified this session |
| Astra serial | `ACRC64300ET`, pinned Orbbec driver `f7e71d9` |
| Battery | 11.2 V |
| Condition | Stationary, wheels on floor, no `/cmd_vel` publisher |
| Raw evidence | [robot_artifacts/x3c_sensor_capability_2026-09-17/](../robot_artifacts/x3c_sensor_capability_2026-09-17/) |

Rates below are measured over 30-40 s windows. The probe is itself a
subscriber, so it perturbs what it measures; this matters only for the point
cloud, which is called out separately.

## Rates and latency

| Topic | Rate (Hz) | Period median / p95 (ms) | Latency median (ms) |
|---|---|---|---|
| `/cam_1/color/image_raw` | 30.01 | 32.4 / 36.4 | 5.5 |
| `/cam_1/color/camera_info` | 29.97 | 32.5 / 36.6 | 6.2 |
| `/cam_1/depth/image_raw` | 30.00 | 33.4 / 39.2 | 2.2 |
| `/cam_1/depth/camera_info` | 30.01 | 33.3 / 38.9 | 2.9 |
| `/cam_1/depth/color/points` | **2.8 - 10.9** | 68 - 264 / 237 - 1013 | 40.7 - 45.4 |
| `/scan`, `/scan_filtered` | 7.17 | 135.3 / 148.8 | 135.7 |
| `/imu/data`, `/imu/data_raw`, `/imu/mag` | 10.00 | 100.0 / 102.1 | 2.5 - 3.0 |
| `/odom`, `/joint_states`, `/vel_raw`, `/voltage` | 10.00 | 100.0 / 102.6 | 1.6 - 3.1 |
| `/diagnostics` | 2.02 | bimodal, two publishers | 2.1 |

`/scan` latency is one full scan period by construction: the stamp marks scan
start and the message is published after the revolution completes. It is not
a pipeline delay.

The `/diagnostics` "dropouts" the probe reports are an artifact of applying a
median-period heuristic to a topic with two independent ~1 Hz publishers, not
a fault.

## Camera

Depth is **registered to color**. The depth image and its `camera_info`
therefore carry `cam_1_color_optical_frame` and *color* intrinsics; there is no
separate depth intrinsic set. A simulator that publishes depth in its own
optical frame with its own intrinsics does not match this.

| | |
|---|---|
| Resolution | 320x240, both streams |
| Intrinsics | `fx = fy = 271.809`, `cx = 158.719`, `cy = 120.092` |
| Derived FOV | **60.97 deg horizontal, 47.64 deg vertical** |
| Distortion | `plumb_bob` with **all-zero coefficients** |
| Color encoding | `rgb8` (public), MJPEG over UVC upstream |
| Depth encoding | `32FC1` metres (public), `16UC1` mm upstream |
| Depth validity | **38.5% valid, 61.5% NaN** |
| Observed depth range | 1.38 - 5.83 m (median 5.21) |

Zero distortion coefficients mean undistortion is a no-op, so a simulator
pinhole model matches the physical camera exactly.

The 61.5% NaN fraction is the number most likely to surprise a consumer
written against the simulator: a simulated depth camera returns dense depth,
this one does not.

Whether a reported metre *is* a metre is a separate question, answered in
[Astra depth calibration](depth_camera_calibration.md): the camera under-reports
by about 1.1%, its noise follows `sigma ~= max(0.002, 0.0019 * d^2.36)` metres,
and it stops working below roughly 0.6 m.

## Point cloud

`/cam_1/depth/color/points` does not reach its 30 Hz target. Measured across
four windows: **2.83, 5.63, 8.20, 10.93 Hz**, with gaps up to 1.6 s. Each
additional subscriber measurably degrades it, so there is no single correct
figure - record the subscriber count alongside any measurement.

Ruled out as causes, with evidence:

| Hypothesis | Verdict | Evidence |
|---|---|---|
| USB 2.0 bandwidth | **No** | depth and color both sustain 30 Hz over the same bus |
| CPU saturation | **No** | 84% idle; adapter 0.38 cores, driver 0.21 cores |
| Thermal throttling | **No** | 46.6 C, `throttled=0x0` |
| UDP socket buffers | **No** | +1 `RcvbufErrors` in 20 s; datagram count far too low for a fragmented 2.4 MB message - it travels over shared memory |
| Color/depth frame pairing | **No** | 99.8% of depth frames pair within half a frame period (median 7.3 ms) |

What remains is the message itself. Each cloud is **2,457,600 bytes**, and
**half of that is padding**: `point_step` is 32 bytes carrying 16 bytes of
`x,y,z,rgb` (offsets 0, 4, 8, 16). At 30 Hz that is 74 MB/s of intra-host DDS
traffic. Repacking to 16 bytes per point is the next thing to try, and the
adapter already rewrites the cloud, so it is nearly free to do there.

The adapter contributes but is not the dominant loss: in a paired window the
raw hardware topic ran 10.93 Hz and the public topic 8.20 Hz, with the adapter
adding ~12 ms of latency.

### This fails the boot-ready gate

The cloud rate is not only a usability problem. `tools/physical_contract_probe.py`
requires `/cam_1/depth/color/points` within `3.0..40.0 Hz`, and on the
2026-09-18 boot it measured **1.15 Hz** and failed:

```
Physical contract FAILED: /cam_1/depth/color/points: measured 1.15 Hz outside 3.0..40.0 Hz
```

`rosmaster-platform-ready` runs that probe before signalling, so the buzzer and
the steady `RGBLight` battery display never ran and the robot gave no
boot-ready indication. Re-running the service by hand once the system had
settled passed at 9.63 Hz and signalled normally.

The cloud therefore sits right on the probe's lower bound: healthy enough after
settling, below it while the machine is still busy at boot. Any change here
should be re-checked against that 3 Hz floor, not just against RViz smoothness.

### Reading a rate from this tool

`rate_hz` is measured between the first and last message, so a slow
subscription match does not understate a healthy sensor. That would hide a
topic that published briefly and then stopped, so the probe separately warns
when messages cover much less of the window than requested, and prints the
window-averaged rate alongside. A reported rate with no coverage warning
describes the whole window; one with a warning describes only the active span.
Treat any coverage warning as "this publisher stopped", which is how the
2026-09-18 motor-controller dropout was first spotted.

## LiDAR

Matches the simulator's recorded contract closely.

| | measured | simulator contract |
|---|---|---|
| Rate | 7.17 Hz | 7.40 Hz |
| Rays | 1080 | 1080 |
| Angle | -pi to +pi | same |
| Range | 0.05 - 12.0 m | same |
| `scan_time` | 0.1343 s | 0.1343 s |
| `time_increment` | 0.0001245 s | 0.0001244 s |

The cable/self-return mask rejects **15 of 1080 rays**; those appear on
`/scan_filtered`. In the test environment `/scan` carried 807 finite returns
of 1080.

## IMU noise floor

A controlled stationary capture, 400 samples over 40 s. The simulator's
`real_robot_contract.yaml` carries provisional figures explicitly flagged as
*not* a controlled capture; these supersede them.

| | simulator (provisional) | **measured** |
|---|---|---|
| gyro x/y stddev (rad/s) | 0.0075 - 0.0121 | **0.0054 / 0.0051** |
| gyro z stddev (rad/s) | 0.0184 - 0.0263 | **0.0052** |
| accel stddev (m/s^2) | 0.15 - 0.31 | **0.102 - 0.135** |
| gravity magnitude (m/s^2) | 9.800 - 9.848 | **9.7554** |

The simulator models **3.5 - 5x more yaw-gyro noise than the robot has**.
Gravity convention is negative-z, as the contract states.

`/imu/data` and `/imu/data_raw` carry effectively identical angular velocity
and linear acceleration; Madgwick contributes orientation only.

## Stationary odometry

Zero drift. Over 40 s: `twist.linear.x`, `twist.linear.y` and `twist.angular.z`
were all exactly `0.0` with zero standard deviation, and pose held constant.
Wheel joint velocities were exactly `0.0`.

This also confirms the simulator's real-robot reference is stale: it records
`/joint_states` position as "always zero" and velocity as "never reported",
whereas this platform reports real values for both.

## Sensor mounts

Measured from live TF at `01e1a60`, relative to `base_link`.

| Frame | xyz (m) | rpy (deg) |
|---|---|---|
| `base_footprint` | `0, 0, -0.07140` | `0, 0, 0` |
| `laser_link` | `0.04350, 0.00005, 0.11000` | `0, 0, 180` |
| `imu_link` | `-0.06000, 0.01000, 0.01000` | `180, 0, -90` |
| `cam_1_link` | `0.05711, 0.00002, 0.03755` | `0, 0, 0` |
| `cam_1_color_frame` | `0.05494, 0.02509, 0.03741` | `-0.02, 0.27, 0.20` |
| `cam_1_depth_frame` | `0.05711, 0.00002, 0.03755` | `0, 0, 0` |

Frames below `cam_1_link` are owned by the Orbbec driver's calibration, not by
the Xacro. The color sensor sits 25.1 mm to the left of the depth sensor,
which is visible on the physical camera.

### Physical confirmation

Tape measurements on `x3-c`, 2026-09-17, approximately +/- 2 mm:

- **Camera height above floor: 105 - 109 mm.** The pre-PR-#31 description
  predicted 119.1 mm and `main` predicts 108.95 mm. This excludes the old
  value and confirms PR #31's `base_joint` fix against hardware, which the PR
  itself never had.
- **Camera set back ~40 mm from the chassis front edge.** The chassis front is
  at `x = +116.5` mm, so `camera_link.STL` predicts 39.3 mm and
  `cad_visual/camera.obj` predicts 98.2 mm.

The second measurement resolves the camera half of issue #33: the
**collision mesh and `camera_mount_joint` are correct, and `camera.obj` is
wrong** - its x-centroid sits 58.9 mm from the STL's. The camera has been drawn
about 6 cm too far back in RViz since PR #26. Because the joint is confirmed
correct, there is no `camera_mount_joint` compensation to unwind.

Rough measurements were sufficient here: the competing hypotheses were 59 mm
and 10 mm apart.
