# x3-c point cloud timing, 2026-09-24

Raw evidence for the "Point cloud" section of
[docs/sensor_capabilities.md](../../docs/sensor_capabilities.md) and issue #43.

Every JSON file was written on the robot by `tools/sensor_capability_probe.py`
at git blob `a4f9ea938e6cb1c243f0748811c6ffc3b94c4f09` (PR #44), run from a
copy outside the workspace while the platform ran `b144d31` (`main`, clean
worktree). Each file records its own conditions under
`measurement.notes` (commit, platform age, ready-gate result, NTP state, Astra
USB path, disconnect count, throttling, SoC temperature),
`measurement.adapter_parameters`, and each topic's `other_subscribers`.
Battery voltage is the `/voltage` topic's `series.value`. With `--per-message`
each topic also carries parallel `arrival_s`, `stamp_s`, `latency_ms` and
`payload_bytes` columns.

| File | Condition | Cloud subscribers |
|---|---|---|
| `camera_boot.json` | Started 0.2 s after the boot-ready gate finished, 24.8 s after `rosmaster-platform` started (warm reboot) | probe only |
| `camera_settled_1.json` .. `_3.json` | Back to back, starting 10.3, 11.0 and 11.7 min after platform start | probe only |
| `camera_precheck.json` | Previous boot, 22.7 min after platform start | probe only |
| `camera_settled_rviz.json` | RViz2 on a Humble workstation over WiFi, displaying only the cloud (`cloud_only.rviz`) | probe + `rviz` |
| `camera_settled_after_rviz.json` | Control, taken right after RViz was closed | probe only |
| `camera_paired.json` | Driver topic and public topic together | public: probe only; driver: adapter + probe |

All runs used `--group camera --topic /voltage --duration 40 --per-message`,
except `camera_paired.json`, which subscribed to the two cloud topics and
`/voltage` only.

Scripts, as run on the robot host:

- `capture.sh` collects the host-side conditions and runs the probe inside the
  `rosmaster_humble` container.
- `boot_wait.sh` started the boot capture once the ready gate's own contract
  probe had exited (so it was not a second cloud subscriber) and NTP had
  synchronized (so no latency spans a clock step). Its output is `boot.log`.
- `settled.sh` waited for 10+ minutes of platform uptime and took the three
  settled runs. Its output is `settled.log`.

Robot stationary throughout, facing the same scene: about 33,700 valid points
per cloud, 2.0 - 3.5 m ahead.
