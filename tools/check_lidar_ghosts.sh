#!/usr/bin/env bash
# Read-only LiDAR ghost check for donatello. Run it with the robot stationary in open space
# (nothing within 0.5 m in the scan plane, nobody touching the robot or the LiDAR cable).
# It subscribes to /scan and /scan_filtered for DURATION seconds, writes nothing on the robot
# and publishes nothing (no /cmd_vel).
#
# PASS (exit 0): /scan has no return closer than NEAR_M in any scan, the rate is healthy, and the
#                cable returns rejected into /scan_filtered stay away from the mask edges.
# FAIL (exit 1): any /scan return under NEAR_M, or too few scans.
# WARN (still exit 0 unless STRICT=1): cable returns within EDGE_DEG of a mask edge or beyond the
#                mask distance, meaning the cable is close to leaking through the filter.
#
# Usage:  tools/check_lidar_ghosts.sh [host]       (default host: donatello)
#         ON_ROBOT=1 tools/check_lidar_ghosts.sh   (run on the robot itself)
# Env:    DURATION=30 NEAR_M=0.25 EDGE_DEG=3 MIN_HZ=5 STRICT=0 CONTAINER=rosmaster_humble
set -u

HOST="${1:-donatello}"
DURATION="${DURATION:-30}"
NEAR_M="${NEAR_M:-0.25}"
EDGE_DEG="${EDGE_DEG:-3}"
MIN_HZ="${MIN_HZ:-5}"
STRICT="${STRICT:-0}"
CONTAINER="${CONTAINER:-rosmaster_humble}"

read -r -d '' PY <<'PYEOF'
import math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DURATION = float(os.environ["DURATION"])
NEAR = float(os.environ["NEAR_M"])
EDGE = float(os.environ["EDGE_DEG"])
MIN_HZ = float(os.environ["MIN_HZ"])


def raw_deg(m, i):
    # driver is reversed with inverted=false: laser_link angle = 180 - raw (verified 2026-11-05)
    return (180.0 - math.degrees(m.angle_min + i * m.angle_increment)) % 360.0


class Probe(Node):
    def __init__(self):
        super().__init__("lidar_ghost_check")
        self.scans = 0
        self.filtered = 0
        self.dirty_scans = 0
        self.near = {}        # raw angle bin (deg) -> [count, min dist]
        self.cable = []       # (raw, dist) rejected points
        self.range_min = 0.05
        self.t_first = self.t_last = None
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_filtered", self.on_filtered, qos_profile_sensor_data)

    def on_scan(self, m):
        now = time.monotonic()
        self.t_first = self.t_first or now
        self.t_last = now
        self.scans += 1
        self.range_min = max(m.range_min, 0.05)
        dirty = False
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and self.range_min <= r < NEAR:
                dirty = True
                b = int(round(raw_deg(m, i)))
                e = self.near.setdefault(b, [0, r])
                e[0] += 1
                e[1] = min(e[1], r)
        self.dirty_scans += dirty

    def on_filtered(self, m):
        self.filtered += 1
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and r > 0.0:
                self.cable.append((raw_deg(m, i), r))


def get_mask(node):
    mask = {"cable_angle_min": 152.0, "cable_angle_max": 170.0, "cable_max_distance": 0.14, "source": "default"}
    try:
        from rcl_interfaces.srv import GetParameters
        cli = node.create_client(GetParameters, "/sllidar_node/get_parameters")
        if cli.wait_for_service(timeout_sec=3.0):
            req = GetParameters.Request()
            req.names = list(k for k in mask if k != "source")
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
            if fut.done():
                for k, v in zip(req.names, fut.result().values):
                    mask[k] = float(v.double_value)
                mask["source"] = "/sllidar_node"
    except Exception as exc:  # keep the check usable even if the param service is unreachable
        mask["source"] = "default (param read failed: %s)" % exc
    return mask


rclpy.init()
n = Probe()
mask = get_mask(n)
end = time.monotonic() + DURATION
while time.monotonic() < end:
    rclpy.spin_once(n, timeout_sec=0.2)

fails, warns = [], []
hz = (n.scans - 1) / (n.t_last - n.t_first) if n.scans > 1 and n.t_last > n.t_first else 0.0
print("duration %.0fs  /scan %d msgs (%.1f Hz)  /scan_filtered %d msgs" % (DURATION, n.scans, hz, n.filtered))
print("mask %s: raw %.0f-%.0f deg, <= %.3f m" % (mask["source"], mask["cable_angle_min"], mask["cable_angle_max"], mask["cable_max_distance"]))
if n.scans == 0:
    print("FAIL  no /scan messages (is the platform up and ROS_DOMAIN_ID right?)")
    sys.exit(1)
if hz < MIN_HZ:
    fails.append("/scan rate %.1f Hz < %.1f Hz" % (hz, MIN_HZ))

if n.near:
    frac = 100.0 * n.dirty_scans / n.scans
    fails.append("returns under %.2f m reach /scan in %d of %d scans (%.0f%%)" % (NEAR, n.dirty_scans, n.scans, frac))
    # merge contiguous 1-degree bins into spans: a narrow span is a ghost/cable, a wide one is an object
    spans, cur = [], None
    for b in sorted(n.near):
        c, d = n.near[b]
        if cur and b - cur["hi"] <= 1:
            cur["hi"] = b
            cur["min"] = min(cur["min"], d)
            cur["n"] += c
        else:
            cur = {"lo": b, "hi": b, "min": d, "n": c}
            spans.append(cur)
    for s in spans:
        print("  /scan near return: raw %3d-%3d deg (laser_link %4d..%4d)  min %.3f m  %d samples (%.1f/scan)%s"
              % (s["lo"], s["hi"], ((180 - s["lo"] + 180) % 360) - 180, ((180 - s["hi"] + 180) % 360) - 180,
                 s["min"], s["n"], s["n"] / n.scans,
                 "  <- wide: probably an object near the robot, not the cable" if s["hi"] - s["lo"] >= 15 else ""))

if n.cable:
    lo, hi, mx = mask["cable_angle_min"], mask["cable_angle_max"], mask["cable_max_distance"]
    angs = [a for a, _ in n.cable]
    dists = [d for _, d in n.cable]
    print("cable returns rejected into /scan_filtered: raw %.0f-%.0f deg, %.3f-%.3f m, %.1f points/scan"
          % (min(angs), max(angs), min(dists), max(dists), len(n.cable) / max(n.filtered, 1)))
    if min(angs) - lo < EDGE or hi - max(angs) < EDGE:
        warns.append("cable returns within %.0f deg of a mask edge (%.0f-%.0f deg)" % (EDGE, lo, hi))
    if max(dists) > mx - 0.01:
        warns.append("cable returns reach %.3f m, at the mask distance limit %.3f m" % (max(dists), mx))
else:
    print("no cable returns in /scan_filtered (cable not in the scan plane, or /scan_filtered not published)")

for w in warns:
    print("WARN  " + w)
for f in fails:
    print("FAIL  " + f)
if fails:
    sys.exit(1)
sys.exit(2 if warns else 0)
PYEOF

run_py() {
  if [ "${ON_ROBOT:-0}" = "1" ]; then
    docker exec -i -e DURATION="$DURATION" -e NEAR_M="$NEAR_M" -e EDGE_DEG="$EDGE_DEG" -e MIN_HZ="$MIN_HZ" "$CONTAINER" \
      bash -lc 'source /opt/ros/humble/setup.bash; python3 -c "$(cat)"' <<<"$PY"
  else
    ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" \
      "docker exec -i -e DURATION=$DURATION -e NEAR_M=$NEAR_M -e EDGE_DEG=$EDGE_DEG -e MIN_HZ=$MIN_HZ $CONTAINER \
       bash -lc 'source /opt/ros/humble/setup.bash; python3 -c \"\$(cat)\"'" <<<"$PY"
  fi
}

echo "LiDAR ghost check on ${ON_ROBOT:+robot}${ON_ROBOT:-$HOST}: robot must be stationary, open space, hands off the cable"
run_py
rc=$?
case $rc in
  0) echo "RESULT: PASS" ;;
  2) echo "RESULT: PASS WITH WARNINGS (cable near the mask edge)"; [ "$STRICT" = "1" ] && exit 1; rc=0 ;;
  *) echo "RESULT: FAIL" ;;
esac
exit $rc
