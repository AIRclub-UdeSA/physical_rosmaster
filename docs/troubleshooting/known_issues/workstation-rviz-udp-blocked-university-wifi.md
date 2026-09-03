# Workstation RViz Cannot Reach the Robot (ROS 2 Distro Mismatch)

## Status

Resolved on 2026-09-03. Originally diagnosed on `x3-c` on 2026-09-02 and
parked with WiFi UDP blocking as the leading theory; retested on 2026-09-03
from a second workstation running ROS 2 Humble (matching the robot) on
`ROS_DOMAIN_ID=11`, which worked immediately — full topic discovery and live
data (`ros2 topic echo /voltage`, RViz2). The original Jazzy workstation was
never retried against a matching robot-side distro, so the primary cause is
recorded as the ROS 2 Humble/Jazzy mismatch, not the WiFi network. See
[Root Cause](#root-cause) for the one piece of evidence that doesn't fully
fit this explanation and remains unresolved.

## Symptom

RViz2 launched on a workstation with `ROS_DOMAIN_ID` matching the robot's
container shows `No tf data. Actual error: Frame [odom] does not exist` and
`No Image` on camera displays — even though `ros2 topic list` on the
workstation successfully discovers the robot's topics by name (`/scan`,
`/odom`, `/tf`, `/robot_description`, `/cam_1/color/image_raw`, etc.).

## Affected Environment

- Workstation: ROS 2 Jazzy, connected over WiFi (university network),
  `172.23.16.99/22`
- Robot: `x3-c`, ROS 2 Humble inside the `rosmaster_humble` container (host
  networking), `172.23.16.183`
- Both on `ROS_DOMAIN_ID=11`; neither side sets `RMW_IMPLEMENTATION`, so both
  default to `rmw_fastrtps_cpp`
- Same subnet. Ping healthy when tested (0% loss, ~3-7 ms typical). SSH (TCP)
  reliable throughout, including while the symptom below was reproduced.

## Fast Diagnosis

Topic and node *names* are visible (`ros2 topic list` succeeds), but zero
message *data* ever arrives: `ros2 topic echo <topic> --once` timed out with
no output for every topic tried, including the lowest-rate one available
(`/rosout`), not just high-rate ones like `/tf`. That rules out a
QoS-reliability or bandwidth problem, which would typically still let some
data through, or degrade a high-rate topic while sparing a quiet one — here
nothing gets through at all.

Direct confirmation used `ros2 multicast send` (from inside the robot's
container) against `ros2 multicast receive` (on the workstation): the robot
sends successfully, the workstation receives nothing, reproduced immediately
alongside a concurrently-verified 0%-loss ping between the same two hosts —
ruling out ordinary link flakiness as the explanation.

## Root Cause

**Primary cause, confirmed 2026-09-03:** the original workstation ran ROS 2
Jazzy against the robot's ROS 2 Humble container. Cross-distro RMW/Fast DDS
interoperability is not guaranteed by the ROS 2 project (different default
Fast DDS versions per distro, differing XTypes/type-discovery behavior), and
matches the exact symptom: participant/topic *names* discovered fine
(basic RTPS participant discovery), while actual data delivery and dynamic
type resolution (`ros2 topic echo`, `ros2 topic info -v`) failed completely.
Retesting from a second workstation on ROS 2 Humble — otherwise the same
`ROS_DOMAIN_ID=11`, same general network — worked immediately with no other
change.

**Unresolved wrinkle:** the 2026-09-02 diagnosis also used `ros2 multicast
send`/`ros2 multicast receive`, a raw UDP multicast probe bundled with the
ROS 2 CLI that does not go through rclpy, DDS participant creation, or any
distro-specific typesupport — it should behave identically regardless of ROS
distro. That test showed the robot sending and the original workstation
receiving nothing, which points at a genuine network-level block and is not
explained by a distro mismatch alone. The 2026-09-03 retest did not control
for this: it was not confirmed whether the second workstation reached the
robot over the same WiFi segment as the original one, or by a different path
(wired, different network). So it remains possible both factors were real —
distro mismatch breaking ROS-level tools, plus an independent network
restriction affecting raw UDP — but only the distro fix was verified needed
to reach a working state, so it's what's recorded as the fix here. If RViz
or CLI tools break again from a workstation already confirmed on ROS 2
Humble, revisit the WiFi/UDP theory in this section's history.

## Decision

Considered resolved by the project owner on 2026-09-03: use a workstation
running the same ROS 2 distro as the robot (Humble) for any live RViz/CLI
session against it. No further network-level workaround pursued unless the
wrinkle above resurfaces on a distro-matched machine.

## What Would Close the Unresolved Wrinkle

If a distro-matched workstation ever fails the same way, these are the
remaining paths, not yet needed:

- **A WebSocket/TCP-based viewer** (e.g. `rosbridge_server` inside the
  container plus a browser client) — sidesteps DDS/UDP entirely by riding on
  a single TCP connection, the same transport SSH already proves reliable
  across this network. Needs one small additional process on the robot; does
  not require changing the validated autostart units.
- **A different network** for the workstation and/or robot that does not
  isolate wireless clients from each other (a wired switch, a non-restrictive
  WiFi network, or a direct link) — no engineering if one is available, but
  depends on physical access.
- **Fast DDS TCP transport plus a discovery server** — keeps native RViz2 and
  `ros2` CLI tooling working as-is, but needs matching transport/discovery
  configuration on both the workstation and the robot's container; more
  moving parts than the other two options.

## Related Documentation

- [Troubleshooting Index](../README.md)
- [docs/setup_guide_ros2_humble_autostart.md](../../setup_guide_ros2_humble_autostart.md)
