"""Hardware-free checks for the parsers in tools/wifi_test."""

import importlib.util
import json
from pathlib import Path

WIFI_TEST = Path(__file__).resolve().parent / "wifi_test"
EVIDENCE = (
    Path(__file__).resolve().parent.parent
    / "robot_artifacts" / "raphael_router_wifi_2026-10-16"
)


def load(name):
    spec = importlib.util.spec_from_file_location(name, WIFI_TEST / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compare_bags = load("compare_bags")
probe_summary = load("probe_summary")


def test_parse_counts_reads_ros2_bag_info_output():
    text = (EVIDENCE / "robot_bag_heavy.txt").read_text()
    counts = compare_bags.parse_counts(text)
    assert counts["/cam_1/color/image_raw"] == 1748
    assert counts["/tf_static"] == 2
    assert len(counts) == 9


def test_compare_flags_only_graded_topics_below_threshold():
    robot = {"/odom": 600, "/scan": 430, "/tf_static": 2, "/voltage": 600}
    workstation = {"/odom": 590, "/scan": 300, "/tf_static": 1}
    rows = {row[0]: row for row in compare_bags.compare(robot, workstation)}
    assert not rows["/odom"][4]
    assert rows["/scan"][4]
    assert not rows["/tf_static"][4]
    assert rows["/voltage"][2:] == (0, 0.0, True)


def test_compare_handles_topic_missing_on_robot():
    rows = list(compare_bags.compare({}, {"/odom": 5}))
    assert rows == [("/odom", 0, 5, None, False)]


def test_recorded_heavy_bag_delivered_every_topic():
    robot = compare_bags.parse_counts((EVIDENCE / "robot_bag_heavy.txt").read_text())
    workstation = compare_bags.parse_counts(
        (EVIDENCE / "workstation_bag_heavy.txt").read_text()
    )
    assert not any(row[4] for row in compare_bags.compare(robot, workstation))


def test_summary_rows_read_probe_json():
    result = json.loads((EVIDENCE / "probe_E.json").read_text())
    rows = {row[0]: row for row in probe_summary.summary_rows(result)}
    topic, rate, median, p95, drops = rows["/cam_1/color/image_raw"]
    assert 17 < rate < 19
    assert median > 100
    assert drops == 0


def test_summary_rows_tolerate_missing_fields():
    rows = probe_summary.summary_rows({"topics": [{"topic": "/x"}]})
    assert rows == [("/x", 0.0, 0.0, 0.0, None)]
