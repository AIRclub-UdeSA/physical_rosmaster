# Copyright 2026 AIRclub UdeSA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the timing evidence the sensor capability probe derives."""

from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Image, Imu, PointCloud2

from physical_contract_probe import median_stamp_rate
from physical_contract_probe import PhysicalContractProbe
from physical_contract_probe import RATE_LIMITS_HZ
from sensor_capability_probe import contract_windows
from sensor_capability_probe import frame_cadence
from sensor_capability_probe import git_blob_id
from sensor_capability_probe import other_endpoints
from sensor_capability_probe import parse_notes
from sensor_capability_probe import payload_size
from sensor_capability_probe import summarize
from sensor_capability_probe import TopicRecord


FRAME = 1.0 / 30.0
CLOUD = "/cam_1/depth/color/points"


def stamps_from_frames(frame_counts, start=1000.0):
    """Return stamps separated by the given numbers of camera frames."""
    stamps = [start]
    for count in frame_counts:
        stamps.append(stamps[-1] + count * FRAME)
    return stamps


def cloud_record(stamps, window_start=50.0, latency=0.045, payload=450000):
    """Build a cloud record whose messages arrive a fixed latency after capture."""
    record = TopicRecord(topic=CLOUD, message_type="PointCloud2")
    origin = stamps[0]
    for stamp in stamps:
        record.arrivals.append(window_start + 0.5 + (stamp - origin) + latency)
        record.receipts.append(stamp + latency)
        record.stamps.append(stamp)
        record.payload_bytes.append(payload)
    return record


def test_median_stamp_rate_uses_the_upper_median_of_increasing_periods():
    stamps = [0.0, 0.1, 0.3, 0.6, 1.0]
    assert median_stamp_rate(stamps) == pytest.approx(1.0 / 0.3)
    assert median_stamp_rate([0.0, 0.0, 0.5]) == pytest.approx(2.0)
    assert median_stamp_rate([1.0]) is None
    assert median_stamp_rate([1.0, 1.0]) is None


def test_contract_gate_still_rejects_the_recorded_boot_rate():
    probe = object.__new__(PhysicalContractProbe)
    messages = []
    for index in range(5):
        message = PointCloud2()
        stamp = 100.0 + index / 1.15
        message.header.stamp.sec = int(stamp)
        message.header.stamp.nanosec = int(round((stamp % 1.0) * 1e9))
        messages.append(message)
    probe.messages = {CLOUD: messages}
    errors = []
    probe.validate_rate(CLOUD, *RATE_LIMITS_HZ[CLOUD], errors)
    assert errors == ["%s: measured 1.15 Hz outside 3.0..40.0 Hz" % CLOUD]


def test_frame_cadence_counts_gaps_in_whole_frames():
    gaps = [FRAME, 2 * FRAME, 3 * FRAME, FRAME]
    cadence = frame_cadence(gaps, 30.0)
    assert cadence["frames_per_gap"] == {"1": 2, "2": 1, "3": 1}
    assert cadence["skipped_frames"] == 3
    assert cadence["delivered_fraction"] == pytest.approx(4.0 / 7.0)
    assert cadence["off_grid_gaps"] == 0


def test_frame_cadence_tolerates_jitter_but_flags_off_grid_gaps():
    gaps = [FRAME + 0.004, 2 * FRAME - 0.004, 1.5 * FRAME]
    cadence = frame_cadence(gaps, 30.0)
    assert cadence["off_grid_gaps"] == 1
    assert sum(cadence["frames_per_gap"].values()) == 3


def test_frame_cadence_is_disabled_without_a_frame_rate():
    assert frame_cadence([FRAME], 0.0) is None
    assert frame_cadence([0.0, -FRAME], 30.0) is None


def test_contract_windows_find_a_stall_the_first_window_misses():
    # Steady 15 Hz, then one 1.2 s stall long enough to fail a later window.
    stamps = stamps_from_frames([2] * 8 + [36] * 3 + [2] * 8)
    gate = contract_windows(stamps, (3.0, 40.0))
    assert gate["window_messages"] == 5
    assert gate["windows"] == len(stamps) - 4
    assert gate["first_window_hz"] == pytest.approx(15.0)
    assert gate["min_hz"] == pytest.approx(30.0 / 36.0)
    assert gate["windows_outside_limits"] > 0


def test_contract_windows_need_a_full_window():
    assert contract_windows([1.0, 2.0, 3.0], (3.0, 40.0)) is None


def test_summarize_reports_stamp_cadence_gate_and_payload_for_the_cloud():
    record = cloud_record(stamps_from_frames([2, 2, 3, 2, 2, 2]))
    record.other_subscribers = []
    summary = summarize(record, 40.0, 2.5, window_start=50.0, frame_rate=30.0)
    assert summary["stamp_period_ms"]["median"] == pytest.approx(2000.0 / 30.0)
    assert summary["frame_cadence"]["skipped_frames"] == 7
    assert summary["contract_rate_hz"]["limits_hz"] == [3.0, 40.0]
    assert summary["latency_ms"]["median"] == pytest.approx(45.0)
    assert summary["payload_bytes"]["median"] == 450000
    assert summary["other_subscriber_count"] == 0
    assert "per_message" not in summary


def test_summarize_keeps_parallel_per_message_columns_when_asked():
    record = cloud_record(stamps_from_frames([1, 2]))
    record.stamps[1] = None
    summary = summarize(record, 40.0, 2.5, window_start=50.0, per_message=True)
    columns = summary["per_message"]
    assert {len(values) for values in columns.values()} == {3}
    assert columns["arrival_s"][0] == pytest.approx(0.545)
    assert columns["latency_ms"] == [pytest.approx(45.0), None, pytest.approx(45.0)]
    assert columns["stamp_s"][1] is None
    assert columns["payload_bytes"] == [450000] * 3


def test_summarize_leaves_frame_cadence_to_camera_types():
    record = TopicRecord(topic="/imu/data", message_type="Imu")
    for index in range(20):
        record.arrivals.append(index * 0.1)
        record.receipts.append(500.0 + index * 0.1)
        record.stamps.append(500.0 + index * 0.1)
        record.payload_bytes.append(None)
    summary = summarize(record, 2.0, 2.5, frame_rate=30.0)
    assert "stamp_period_ms" in summary
    assert "frame_cadence" not in summary
    assert "payload_bytes" not in summary
    assert "other_subscriber_count" not in summary


def test_summarize_counts_non_increasing_stamps():
    record = cloud_record([10.0, 10.1, 10.1, 10.05, 10.2])
    summary = summarize(record, 1.0, 2.5, frame_rate=30.0)
    assert summary["non_increasing_stamps"] == 2


def test_payload_size_covers_bulk_types_only():
    image = Image()
    image.data = bytes(12)
    cloud = PointCloud2()
    cloud.data = bytes(32)
    assert payload_size(image) == 12
    assert payload_size(cloud) == 32
    assert payload_size(Imu()) is None


def test_other_endpoints_excludes_only_this_probe():
    endpoints = [
        SimpleNamespace(node_name="sensor_capability_probe", node_namespace="/"),
        SimpleNamespace(node_name="rviz2", node_namespace="/"),
        SimpleNamespace(node_name="sensor_capability_probe", node_namespace="/other"),
    ]
    assert other_endpoints(endpoints, "sensor_capability_probe", "/") == [
        {"node": "rviz2", "namespace": "/"},
        {"node": "sensor_capability_probe", "namespace": "/other"},
    ]


def test_git_blob_id_matches_git_hash_object():
    # `printf 'hello\n' | git hash-object --stdin`
    assert git_blob_id(b"hello\n") == "ce013625030ba8dba906f756967f9e9ca394464a"


def test_parse_notes_keeps_values_with_equals_signs():
    assert parse_notes(["commit=b144d31", " condition = boot ", "cmd=a=b"]) == {
        "commit": "b144d31",
        "condition": "boot",
        "cmd": "a=b",
    }


@pytest.mark.parametrize("entry", ["no-separator", "=value"])
def test_parse_notes_rejects_malformed_entries(entry):
    with pytest.raises(SystemExit):
        parse_notes([entry])
