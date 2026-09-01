#!/usr/bin/env bash
# Announces that the strict platform launch is up and contract-verified: a
# short buzzer pattern, a brief marquee-light transition, then one steady
# RGBLight state left on indefinitely — confirmed 2026-09-02 on x3-c,
# effect 6 renders as a green bar-graph proportional to charge. Only
# reached after tools/physical_contract_probe.py has already exited 0 for
# this boot (rosmaster-ready-launch enforces that order) — never publish
# this signal for an unverified graph. Runs inside the container with ROS
# already sourced. Deployed with `docker cp` to /usr/local/sbin inside the
# container, the same way the host-side scripts are copied to
# /usr/local/sbin on the host — not read from the tracked workspace
# checkout, so it does not require a git push/pull cycle to update.
# docker exec on the host inherits this script's stdout into the calling
# systemd unit's journal, so plain echo is enough here.
set -euo pipefail

READY_RGB_TRANSIENT_EFFECT="${READY_RGB_TRANSIENT_EFFECT:-2}"
READY_RGB_TRANSIENT_SECONDS="${READY_RGB_TRANSIENT_SECONDS:-3}"
READY_RGB_EFFECT="${READY_RGB_EFFECT:-6}"

beep() {
  local on_seconds="$1" pause_seconds="$2"
  ros2 topic pub -1 /Buzzer std_msgs/msg/Bool "data: true" >/dev/null
  sleep "$on_seconds"
  ros2 topic pub -1 /Buzzer std_msgs/msg/Bool "data: false" >/dev/null
  sleep "$pause_seconds"
}

beep 0.1 0.08
beep 0.1 0.08
beep 0.4 0.08

ros2 topic pub -1 /RGBLight std_msgs/msg/Int32 "data: ${READY_RGB_TRANSIENT_EFFECT}" >/dev/null
sleep "$READY_RGB_TRANSIENT_SECONDS"

ros2 topic pub -1 /RGBLight std_msgs/msg/Int32 "data: ${READY_RGB_EFFECT}" >/dev/null

echo "Sent boot-ready buzzer + RGBLight (transient=${READY_RGB_TRANSIENT_EFFECT} for ${READY_RGB_TRANSIENT_SECONDS}s, then steady=${READY_RGB_EFFECT})"
