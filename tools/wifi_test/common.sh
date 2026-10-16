# Shared settings for the WiFi test scripts. Sourced, not run.
#
# Override per run: ROBOT=leo.local ROS_DOMAIN_ID=12 bash link_check.sh
ROBOT="${ROBOT:-raphael.local}"
CONTAINER="${CONTAINER:-rosmaster_humble}"
WIFI_TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS_DIR="$(dirname "$WIFI_TEST_DIR")"
# Outside the checkout, so bags and logs never end up in a commit.
RESULTS="${RESULTS:-$HOME/wifi_test_results}"
mkdir -p "$RESULTS"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export PYTHONUNBUFFERED=1
unset ROS_LOCALHOST_ONLY

step() { echo; echo "=================== $* ($(date +%H:%M:%S))"; }
expect() { echo "   expected: $*"; }
rssh() { ssh -o BatchMode=yes -o ConnectTimeout=8 pi@"$ROBOT" "$@"; }
# Runs a script inside the robot's container; the script goes through stdin.
cexec() { rssh "docker exec -i $CONTAINER bash -l" <<<"$1"; }

# Camera rates and latency measured on the robot for 30 s, so the latency does
# not depend on the workstation clock. Usage: robot_probe <label>
robot_probe() {
  cexec "source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
mkdir -p /root/wifi-test
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/sensor_capability_probe.py --group camera --duration 30 \
  --note step=$1 --output /root/wifi-test/$1.json >/dev/null 2>&1
cat /root/wifi-test/$1.json" > "$RESULTS/probe_$1.json"
  python3 "$WIFI_TEST_DIR/probe_summary.py" "$RESULTS/probe_$1.json"
}
