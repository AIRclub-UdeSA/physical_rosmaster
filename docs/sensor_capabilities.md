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

The point cloud was re-measured on 2026-09-24, after its pipeline changed, and
has its own record in [Point cloud](#point-cloud). Its row in the table below
comes from that session; every other row is from 2026-09-17.

## Rates and latency

| Topic | Rate (Hz) | Period median / p95 (ms) | Latency median (ms) |
|---|---|---|---|
| `/cam_1/color/image_raw` | 30.01 | 32.4 / 36.4 | 5.5 |
| `/cam_1/color/camera_info` | 29.97 | 32.5 / 36.6 | 6.2 |
| `/cam_1/depth/image_raw` | 30.00 | 33.4 / 39.2 | 2.2 |
| `/cam_1/depth/camera_info` | 30.01 | 33.3 / 38.9 | 2.9 |
| `/cam_1/depth/color/points` | **7.3 - 9.6** (2026-09-24) | 67 - 100 / 272 - 400 | 43 - 53 |
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

Measured on 2026-09-24 against the pipeline `main` ships at `b144d31`. The
adapter packs each point to 16 bytes (#38) and by default drops non-finite
returns (`cloud_strip_nan=true`, #41), so `/cam_1/depth/color/points` is an
unorganized cloud (`height=1`) whose size follows the scene.
`cloud_decimation` is 1. These figures supersede the
[2026-09-17 ones](#2026-09-17-figures-superseded).

### Measurement record

| | |
|---|---|
| Robot | `x3-c`, stationary, facing the same view throughout |
| Date | 2026-09-24 |
| Commit | `b144d31` (`main`), clean worktree |
| Cloud | about 33,700 points, 538 - 542 KB per message; scene 2.0 - 3.5 m ahead |
| Probe | `tools/sensor_capability_probe.py` from #44, `--group camera --topic /voltage --duration 40 --per-message` |
| Battery | 10.1 V at boot, 9.9 V at the last run |
| USB | Astra depth at `1-2.1`, on bus 1 with the motor's hub; on 2026-09-17 the camera had a bus to itself. 0 disconnects, `throttled=0x0`, SoC 48 - 49 C |
| Boot | Warm reboot (`systemctl reboot`); both Astra functions enumerated; the ready gate passed |
| Raw evidence | [robot_artifacts/x3c_sensor_capability_2026-09-24/](../robot_artifacts/x3c_sensor_capability_2026-09-24/), including the scripts that ran each condition |

"Cloud subscribers" counts the probe. Except in the paired run, the probe also
subscribes to both images and both `camera_info` topics, the same set the boot
gate subscribes to.

| Condition | Platform age | Cloud subscribers | Rate (Hz) | Stamp period median / p95 / p99 / max (ms) | Frames delivered | Latency median / p95 (ms) | Dropouts | Gate windows under 3 Hz |
|---|---|---|---|---|---|---|---|---|
| Boot | 25 s | 1 | 9.56 | 67 / 272 / 556 / 900 | 31.9% | 52 / 55 | 51 | 7 of 375 (min 1.88 Hz) |
| Settled, run 1 | 10.3 min | 1 | 7.29 | 100 / 400 / 600 / 667 | 24.3% | 52 / 54 | 44 | 17 of 288 (min 2.00 Hz) |
| Settled, run 2 | 11.0 min | 1 | 9.01 | 67 / 300 / 627 / 1133 | 30.0% | 53 / 56 | 61 | 2 of 357 (min 2.31 Hz) |
| Settled, run 3 | 11.7 min | 1 | 8.51 | 67 / 367 / 609 / 1069 | 28.4% | 45 / 51 | 56 | 11 of 334 (min 2.31 Hz) |
| Settled, previous boot | 22.7 min | 1 | 8.67 | 100 / 300 / 467 / 567 | 28.9% | 53 / 57 | 35 | 7 of 342 (min 2.50 Hz) |
| Settled + RViz over WiFi | 13.1 min | 2 | 8.88 | 100 / 200 / 267 / 700 | 29.6% | 343 / 444 | 3 | 0 of 350 (min 3.75 Hz) |
| Settled, RViz closed again | 14.3 min | 1 | 9.18 | 67 / 307 / 500 / 900 | 30.6% | 43 / 48 | 56 | 7 of 334 (min 2.00 Hz) |
| Paired: public topic | 15.0 min | 1 | 8.70 | 34 / 400 / 689 / 1100 | 29.0% | 53 / 55 | 124 | 12 of 343 (min 1.87 Hz) |
| Paired: driver topic | 15.0 min | 2 | 8.64 | 33 / 467 / 725 / 933 | 28.8% | 43 / 44 | 112 | not gated |

- **Frames delivered** is the share of 30 Hz camera frames that became a
  cloud, counted from header stamps. **Dropouts** is the probe's older count
  of arrival gaps longer than 2.5 times the median gap. With a median gap
  already two or three frames long, that count misses most lost frames, so use
  frames delivered instead.
- Nothing changed between the settled runs, yet they span 7.3 - 9.2 Hz.
  Compare any condition against that spread, not against a single run.
- **Boot was not slower.** The boot capture started 25 s after the platform
  and measured 9.56 Hz, just above the settled runs, with 52 ms latency across
  the whole window. It did not reproduce the 1.15 Hz of 2026-09-18, which ran
  the older 32-byte organized cloud on a different USB layout.

### Timing model for the simulator

For a consumer on the robot with nothing else subscribed to the cloud:

- **Latency is about 50 ms, and effectively constant.** Medians are 43 - 53 ms
  across runs, with a per-run standard deviation of 1 - 3 ms. `header.stamp` is
  the capture time, so a simulated cloud should be published about 50 ms after
  its frame and keep that frame's stamp.
- **Gaps are whole camera frames.** All 2,747 stamp gaps measured on the
  public topic landed on a 33.3 ms frame boundary. Consecutive gaps are
  uncorrelated: their lag-1 autocorrelation is 0.01 - 0.06, inside the range
  that shuffling the same gaps produces. Drawing each gap independently from
  the distribution below therefore reproduces what was measured.

Frames from one cloud to the next, pooled over the three settled runs (988
gaps):

| Frames | Gap (ms) | Probability | Cumulative |
|---|---|---|---|
| 1 | 33 | 0.3310 | 0.3310 |
| 2 | 67 | 0.1852 | 0.5162 |
| 3 | 100 | 0.1538 | 0.6700 |
| 4 | 133 | 0.0901 | 0.7601 |
| 5 | 167 | 0.0526 | 0.8128 |
| 6 | 200 | 0.0466 | 0.8593 |
| 7 | 233 | 0.0283 | 0.8877 |
| 8 | 267 | 0.0223 | 0.9099 |
| 9 | 300 | 0.0202 | 0.9302 |
| 10 | 333 | 0.0132 | 0.9433 |
| 11 | 367 | 0.0132 | 0.9565 |
| 12 | 400 | 0.0061 | 0.9626 |
| 13 | 433 | 0.0091 | 0.9717 |
| 14 | 467 | 0.0051 | 0.9767 |
| 15 | 500 | 0.0040 | 0.9808 |
| 16 | 533 | 0.0010 | 0.9818 |
| 17 | 567 | 0.0040 | 0.9858 |
| 18 | 600 | 0.0040 | 0.9899 |
| 19 | 633 | 0.0020 | 0.9919 |
| 20 | 667 | 0.0020 | 0.9939 |
| 23 | 767 | 0.0010 | 0.9949 |
| 24 | 800 | 0.0010 | 0.9960 |
| 25 | 833 | 0.0010 | 0.9970 |
| 28 | 933 | 0.0010 | 0.9980 |
| 32 | 1067 | 0.0010 | 0.9990 |
| 34 | 1133 | 0.0010 | 1.0000 |

The mean gap is 3.63 frames, which is 8.3 Hz with 27.6% of frames delivered.
The median is 2 frames, p95 is 11 frames (367 ms), and the longest gap was 34
frames (1.13 s). The raw JSON keeps per-message stamps for refitting.

The boot run delivered 31.9% of frames, close to the settled runs' 24.3 -
30.6%, so the same model covers boot. The RViz run below does not fit it.

### Margin against the boot gate

`rosmaster-platform-ready` runs `tools/physical_contract_probe.py`, which
rates the cloud from the first five clouds it receives: the upper median of
their four stamp periods must fall within 3.0 - 40.0 Hz. Applying that exact
statistic to every five-cloud window of the seven runs with one cloud
subscriber:

- **63 of 2,373 windows (2.65%) rate below 3.0 Hz**, the lowest at 1.87 Hz.
  They come from settled runs as well as from the boot run.
- The median window rates 10 - 15 Hz. The floor is usually cleared by a wide
  margin, but a few long gaps inside five clouds are enough to fail it.
- The first window of the boot capture rated exactly 3.00 Hz. The gate itself
  passed on this boot.

The gate samples one window per boot. If that window behaves like any other,
about one boot in 40 fails the gate on the point cloud alone, on a healthy and
idle robot. A failed gate sends no buzzer or LED boot signal (see the
[2026-09-18 incident](troubleshooting/incidents/2026-09-18-x3-c-usb-hub-dropout.md)).

### Where frames are lost

The paired run subscribed to the driver's topic and the public topic at once,
and matched clouds by stamp, which the adapter preserves:

- **The driver is the main loss.** Over 1,192 camera frames it published at
  least 429 clouds, so it produces a cloud for only about 36 - 38% of frames.
- **The adapter republished about 76 - 81% of the driver's clouds**, adding
  10.8 ms median latency (p95 12.6 ms).
- The probe's own subscription to the driver's topic missed 85 clouds that the
  adapter did republish, about as many as the adapter missed (83). Two
  independent subscribers each losing about a fifth of the 2.4 MB messages
  points at delivery of large best-effort messages rather than at the
  adapter's processing, but this was not isolated.

### A remote RViz delays every subscriber

With RViz2 on a workstation subscribed to the cloud over WiFi, the probe on
the robot still received 8.88 Hz (29.6% of frames), but **343 ms after capture
instead of about 50 ms**. The p95 was 444 ms, no cloud arrived in under
250 ms, and the delay held steady across the window. Gaps also became more
regular, and no window fell under the gate floor. In the next run, with RViz
closed, latency was back to 43 ms.

The cause was not isolated. What matters for consumers is the effect: one
off-robot viewer delays the cloud for every consumer on the robot, which a
simulator running on one machine does not show. The RViz configuration used
is `cloud_only.rviz` in the raw evidence.

### 2026-09-17 figures (superseded)

Measured on `01e1a60` with the driver's 32-byte, organized cloud
(2,457,600 bytes per message) and the camera on its own USB bus: **2.83, 5.63,
8.20 and 10.93 Hz** across four windows, with gaps up to 1.6 s. In a paired
window the driver's topic ran 10.93 Hz and the public topic 8.20 Hz, with the
adapter adding about 12 ms.

Ruled out as causes of the low rate, with evidence from that session:

| Hypothesis | Verdict | Evidence |
|---|---|---|
| USB 2.0 bandwidth | **No** | depth and color both sustain 30 Hz over the same bus |
| CPU saturation | **No** | 84% idle; adapter 0.38 cores, driver 0.21 cores |
| Thermal throttling | **No** | 46.6 C, `throttled=0x0` |
| UDP socket buffers | **No** | +1 `RcvbufErrors` in 20 s; datagram count far too low for a fragmented 2.4 MB message - it travels over shared memory |
| Color/depth frame pairing | **No** | 99.8% of depth frames pair within half a frame period (median 7.3 ms) |

That left the message size. Repacking to 16 bytes (#38) halved it and
stripping NaNs (#41) roughly halved it again, to about 540 KB. Neither
measurably changed the rate, and the 2026-09-24 paired run places most of the
loss inside the driver.

On the 2026-09-18 boot the gate measured the cloud at 1.15 Hz and failed:

```
Physical contract FAILED: /cam_1/depth/color/points: measured 1.15 Hz outside 3.0..40.0 Hz
```

Re-running the ready service by hand once the system had settled passed at
9.63 Hz and signalled normally.

### Reading a rate from this tool

`rate_hz` is measured between the first and last message, so a slow
subscription match does not understate a healthy sensor. That would hide a
topic that published briefly and then stopped, so the probe separately warns
when messages cover much less of the window than requested, and prints the
window-averaged rate alongside. A reported rate with no coverage warning
describes the whole window; one with a warning describes only the active span.
Treat any coverage warning as "this publisher stopped", which is how the
2026-09-18 motor-controller dropout was first spotted.

Arrival-based fields (`period_ms`, `dropouts`) describe what a subscriber
experienced. Stamp-based fields describe what the publisher produced:
`stamp_period_ms`, `frame_cadence` (camera types only, gaps in whole frames),
and `contract_rate_hz` (the boot gate's statistic over every window, for the
topics it checks).

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
