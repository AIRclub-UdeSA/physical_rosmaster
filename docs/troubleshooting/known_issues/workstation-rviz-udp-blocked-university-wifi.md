# Workstation RViz Cannot Reach the Robot Over University WiFi (UDP Blocked)

## Status

Open, deferred. Diagnosed on `x3-c` on 2026-09-02. Not blocking: the full
acceptance sequence and autostart were both already validated independently
of this. Parked for the project owner to pick up later; no fix attempted yet.

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

The WiFi network between the workstation and the robot does not forward UDP
traffic between wireless clients. Confirmed directly for multicast (above).
A targeted workaround — a Fast DDS `initialPeersList` profile pointing the
workstation's discovery directly at the robot's unicast IP, bypassing
multicast for discovery entirely — also produced zero data, so the
restriction is broader than multicast alone; ordinary UDP unicast between
these two clients appears blocked too. TCP (SSH) is unaffected. This is
consistent with a common university/enterprise WiFi client-isolation policy,
which this network is presumed to run, though the access point's
configuration itself was not inspected.

ROS 2's default RMW (Fast DDS, and DDS implementations generally) uses UDP
for both discovery and data. Neither endpoint's own ROS/DDS configuration can
work around a network-level policy that drops UDP between clients — this is
not fixable from ROS settings alone.

## Decision

Parked by the project owner on 2026-09-02. Nothing delivered today depends on
this; revisit when convenient.

## What Would Close This Properly

Three realistic paths, not yet chosen between:

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
