#!/usr/bin/env bash
# Records the same topics for 60 s on the workstation (over WiFi) and on the
# robot (local), then compares per-topic message counts.
# Usage: bash bag_compare.sh light|heavy
source "$(dirname "$0")/common.sh"
KIND="${1:?usage: bash bag_compare.sh light|heavy}"
TOPICS="/tf /tf_static /odom /scan /imu/data /joint_states /voltage"
[ "$KIND" = heavy ] && TOPICS="$TOPICS /cam_1/color/image_raw /cam_1/color/camera_info"
NAME="${KIND}_$(date +%H%M)"
LOG="$RESULTS/bag_$NAME.txt"
exec > >(tee -a "$LOG") 2>&1

step "Bag $KIND, 60 s on the workstation and on the robot at once"
# The robot copy is counted and deleted at once: a bag left in the container
# grows its writable layer (see the root-filesystem-full known issue).
cexec "source /opt/ros/humble/setup.bash; mkdir -p /root/wifi-test; cd /root/wifi-test
timeout -s INT 60 ros2 bag record -o robot_$NAME $TOPICS >/dev/null 2>&1
ros2 bag info robot_$NAME | grep -E 'Duration|Messages|Topic'
rm -rf robot_$NAME" > "$RESULTS/robot_info_$NAME.txt" &
pid=$!
sleep 2
# </dev/null: under `timeout` the recorder is in a background process group,
# and its keyboard controls reading the terminal stop it with SIGTTIN.
# shellcheck disable=SC2086
(cd "$RESULTS" && timeout -s INT 60 ros2 bag record -s mcap -o "workstation_$NAME" $TOPICS </dev/null >/dev/null 2>&1)
wait $pid
python3 "$WIFI_TEST_DIR/compare_bags.py" "$RESULTS/robot_info_$NAME.txt" \
  <(ros2 bag info "$RESULTS/workstation_$NAME")
expect "workstation >= 90% of the robot on every topic"
