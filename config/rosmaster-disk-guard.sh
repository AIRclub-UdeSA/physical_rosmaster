#!/usr/bin/env bash
# Refuses platform autostart when the host root filesystem is low on space.
# Install as /usr/local/sbin/rosmaster-disk-guard (mode 0755, root-owned).
# See docs/troubleshooting/known_issues/root-filesystem-full-login-loop.md.
set -euo pipefail

MAX_USED_PCT="${MAX_USED_PCT:-94}"
MIN_FREE_MIB="${MIN_FREE_MIB:-2048}"

read -r used_pct avail_mib < <(
  df -Pm / | awk 'NR==2 {
    gsub("%", "", $5);
    print $5, $4
  }'
)

if (( used_pct > MAX_USED_PCT || avail_mib < MIN_FREE_MIB )); then
  logger -t rosmaster-disk-guard \
    "Refusing startup: root=${used_pct}% used, ${avail_mib} MiB free"
  exit 1
fi

logger -t rosmaster-disk-guard "OK: root=${used_pct}% used, ${avail_mib} MiB free"
