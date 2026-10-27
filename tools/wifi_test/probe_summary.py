"""Summarize a sensor_capability_probe.py JSON: rate, latency and dropouts.

Usage: python3 probe_summary.py <probe.json>
"""

import json
import sys


def summary_rows(result):
    """Return (topic, rate Hz, latency median ms, latency p95 ms, dropouts)."""
    rows = []
    for topic in result["topics"]:
        latency = topic.get("latency_ms") or {}
        dropouts = topic.get("dropouts") or {}
        rows.append((
            topic["topic"],
            topic.get("rate_hz") or 0.0,
            latency.get("median", 0.0),
            latency.get("p95", 0.0),
            dropouts.get("count"),
        ))
    return rows


def main(argv):
    try:
        with open(argv[1]) as result_file:
            result = json.load(result_file)
    except (OSError, ValueError) as error:
        sys.exit(f"   NO PROBE RESULT ({error})")
    print(f"   {'topic':38} {'Hz':>6} {'lat med':>8} {'lat p95':>8} {'drops':>6}")
    for topic, rate, median, p95, drops in summary_rows(result):
        drops = "-" if drops is None else drops
        print(f"   {topic:38} {rate:6.1f} {median:8.0f} {p95:8.0f} {drops:>6}")


if __name__ == "__main__":
    main(sys.argv)
