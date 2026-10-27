# `raphael` on the competition router, 2026-10-16

Raw evidence for [raphael_router_wifi_2026-10-16.md](../raphael_router_wifi_2026-10-16.md).

| File | What |
|---|---|
| `probe_A.json` | Step A: no remote subscriber |
| `probe_C.json` | Step C: remote RViz with RobotModel, TF, LaserScan |
| `probe_D.json` | Step D: C + Image |
| `probe_E.json` | Step E: D + PointCloud2 |
| `robot_bag_light.txt`, `workstation_bag_light.txt` | `ros2 bag info` of the light bag, robot and workstation copies |
| `robot_bag_heavy.txt`, `workstation_bag_heavy.txt` | Same for the heavy bag (with the color image) |

The probe files were written on the robot by `tools/sensor_capability_probe.py
--group camera --duration 30` (probe blob `a572b9f`, recorded in each file's
`measurement`). They were taken with the Spanish draft of the scripts, so the
step label is under `measurement.notes.escalon` rather than `step`. The bags
themselves were not kept: the heavy one is 389 MB.
