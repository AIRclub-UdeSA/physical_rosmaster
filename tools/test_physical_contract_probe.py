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

"""Focused tests for the physical probe: diagnostics, targets and the TF check."""

from collections import deque
import dataclasses
import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
import pytest
from rclpy.time import Time
from tf2_ros.buffer import Buffer

import physical_contract_probe as probe_module
from physical_contract_probe import finite_positive
from physical_contract_probe import PhysicalContractProbe
from physical_contract_probe import RequiredDiagnosticObservation
from physical_contract_probe import resolve_target
from physical_contract_probe import Target
from physical_contract_probe import TARGETS


MOTOR_NAME = "yahboomcar_bringup: motor controller and onboard sensors"
ODOMETRY_NAME = "yahboomcar_base_node: wheel encoder odometry"


class FakeClock:
    """Deterministic monotonic clock for receive-age checks."""

    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now


def make_status(name, level=DiagnosticStatus.OK):
    """Return one complete healthy required status."""
    status = DiagnosticStatus()
    status.name = name
    status.level = level
    status.message = "healthy" if level == DiagnosticStatus.OK else "failed"
    if name == MOTOR_NAME:
        values = {
            "feedback_state": "healthy",
            "feedback_report_sequence": "12",
            "feedback_timeout_seconds": "0.500000",
        }
        for channel in ("speed", "encoder", "imu_raw"):
            values["feedback_%s_age_seconds" % channel] = "0.100000"
            values["feedback_%s_stale" % channel] = "false"
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
    return status


def make_array(*statuses):
    """Wrap statuses in one aggregate diagnostics message."""
    message = DiagnosticArray()
    message.status = list(statuses)
    return message


def bare_probe(clock, max_age=2.0, diagnostic_samples=3):
    """Build only the state exercised by capture and diagnostic validation."""
    probe = object.__new__(PhysicalContractProbe)
    probe._monotonic_clock = clock
    probe.diagnostic_max_age = max_age
    probe.required_counts = {"/diagnostics": diagnostic_samples}
    probe.messages = {"/diagnostics": []}
    probe.first_arrivals = {}
    probe.observed_dynamic_tf_edges = set()
    probe.latest_required_diagnostics = {}
    return probe


def test_required_sources_survive_aggregate_window_eviction():
    """Unrelated aggregate traffic cannot evict required source state."""
    clock = FakeClock(10.0)
    probe = bare_probe(clock)
    probe.capture("/diagnostics", make_array(make_status(MOTOR_NAME)))
    clock.now = 10.25
    probe.capture("/diagnostics", make_array(make_status(ODOMETRY_NAME)))

    noise_names = []
    for index in range(5):
        clock.now += 0.1
        noise_name = "unrelated source %d" % index
        noise_names.append(noise_name)
        probe.capture("/diagnostics", make_array(make_status(noise_name)))

    aggregate_names = {
        status.name
        for message in probe.messages["/diagnostics"]
        for status in message.status
    }
    assert aggregate_names == set(noise_names[-3:])
    assert set(probe.latest_required_diagnostics) == {
        MOTOR_NAME,
        ODOMETRY_NAME,
    }
    assert (
        probe.latest_required_diagnostics[MOTOR_NAME].received_at == 10.0
    )
    assert (
        probe.latest_required_diagnostics[ODOMETRY_NAME].received_at == 10.25
    )

    errors = []
    probe.validate_diagnostics(errors)
    assert errors == []


def test_completion_requires_each_independently_tracked_source():
    """Aggregate count alone cannot end collection before both owners report."""
    clock = FakeClock()
    probe = bare_probe(clock)
    probe.messages["/diagnostics"] = [make_array()] * 3
    probe.observed_dynamic_tf_edges.add(probe_module.REQUIRED_DYNAMIC_TF_EDGE)

    assert not probe.complete()
    probe.latest_required_diagnostics[MOTOR_NAME] = RequiredDiagnosticObservation(
        make_status(MOTOR_NAME), clock.now
    )
    assert not probe.complete()
    probe.latest_required_diagnostics[ODOMETRY_NAME] = (
        RequiredDiagnosticObservation(make_status(ODOMETRY_NAME), clock.now)
    )
    assert probe.complete()


def test_completion_waits_for_fresh_required_diagnostics():
    """Collection uses its remaining timeout instead of validating stale state."""
    clock = FakeClock(3.0)
    probe = bare_probe(clock, max_age=2.0)
    probe.messages["/diagnostics"] = [make_array()] * 3
    probe.observed_dynamic_tf_edges.add(probe_module.REQUIRED_DYNAMIC_TF_EDGE)
    probe.latest_required_diagnostics = {
        MOTOR_NAME: RequiredDiagnosticObservation(
            make_status(MOTOR_NAME), 0.0
        ),
        ODOMETRY_NAME: RequiredDiagnosticObservation(
            make_status(ODOMETRY_NAME), 3.0
        ),
    }

    assert not probe.complete()
    probe.latest_required_diagnostics[MOTOR_NAME] = (
        RequiredDiagnosticObservation(make_status(MOTOR_NAME), 3.0)
    )
    assert probe.complete()


def test_stale_required_source_fails_even_when_status_is_ok():
    """A formerly healthy source cannot pass after its receive evidence ages out."""
    clock = FakeClock(20.0)
    probe = bare_probe(clock, max_age=2.0)
    probe.capture("/diagnostics", make_array(make_status(MOTOR_NAME)))
    clock.now = 21.0
    probe.capture("/diagnostics", make_array(make_status(ODOMETRY_NAME)))
    clock.now = 22.001

    errors = []
    probe.validate_diagnostics(errors)

    assert any(MOTOR_NAME in error and "maximum is 2.000s" in error for error in errors)
    assert not any(
        ODOMETRY_NAME in error and "maximum is" in error for error in errors
    )


def test_new_status_replaces_level_and_refreshes_receive_time():
    """Validation uses the newest status and receive time for each owner."""
    clock = FakeClock(30.0)
    probe = bare_probe(clock)
    probe.capture(
        "/diagnostics",
        make_array(make_status(ODOMETRY_NAME, DiagnosticStatus.ERROR)),
    )
    clock.now = 32.0
    probe.capture("/diagnostics", make_array(make_status(ODOMETRY_NAME)))
    probe.capture("/diagnostics", make_array(make_status(MOTOR_NAME)))
    clock.now = 32.1

    errors = []
    probe.validate_diagnostics(errors)

    assert errors == []
    observation = probe.latest_required_diagnostics[ODOMETRY_NAME]
    assert observation.status.level == DiagnosticStatus.OK
    assert observation.received_at == 32.0


@pytest.mark.parametrize("received_at", [6.0, math.inf, math.nan])
def test_nonfinite_or_backward_receive_age_is_rejected(received_at):
    """Monotonic age evidence must itself be finite and nonnegative."""
    clock = FakeClock(5.0)
    probe = bare_probe(clock)
    probe.latest_required_diagnostics = {
        MOTOR_NAME: RequiredDiagnosticObservation(
            make_status(MOTOR_NAME), received_at
        ),
        ODOMETRY_NAME: RequiredDiagnosticObservation(
            make_status(ODOMETRY_NAME), 5.0
        ),
    }

    errors = []
    probe.validate_diagnostics(errors)

    assert any(
        MOTOR_NAME in error and "invalid monotonic receive age" in error
        for error in errors
    )


@pytest.mark.parametrize("value", [0.0, -1.0, math.inf, math.nan, "invalid"])
def test_diagnostic_max_age_must_be_finite_and_positive(value):
    """Unsafe diagnostic freshness limits are rejected during setup."""
    with pytest.raises(ValueError, match="diagnostic_max_age"):
        finite_positive(value, "diagnostic_max_age")


def test_simulator_target_differs_from_hardware_in_exactly_two_ways():
    """The simulator has no driver, so it skips /diagnostics and has one static TF owner."""
    assert {field.name for field in dataclasses.fields(Target)} == {
        "diagnostics",
        "tf_static_messages",
    }
    hardware, simulator = TARGETS["hardware"], TARGETS["simulator"]
    assert (hardware.diagnostics, hardware.tf_static_messages) == (True, 2)
    assert (simulator.diagnostics, simulator.tf_static_messages) == (False, 1)
    assert set(TARGETS) == {"hardware", "simulator"}


@pytest.mark.parametrize("name", ["sim", "", "Hardware", None])
def test_unknown_target_is_rejected(name):
    """A typo must not silently grade the wrong platform."""
    with pytest.raises(ValueError, match="target"):
        resolve_target(name)


def test_simulator_target_drops_only_diagnostics_from_the_topics():
    """Every other topic and type is graded on both targets."""
    probe = object.__new__(PhysicalContractProbe)
    probe.configure_target("hardware")
    hardware_topics = dict(probe.topic_types)
    probe.configure_target("simulator")
    assert hardware_topics == probe_module.TOPIC_TYPES
    assert probe.topic_types == {
        topic: spec
        for topic, spec in hardware_topics.items()
        if topic != "/diagnostics"
    }
    assert probe.required_diagnostic_sources == frozenset()
    probe.configure_target("hardware")
    assert probe.required_diagnostic_sources == probe_module.REQUIRED_DIAGNOSTIC_SOURCES


def complete_probe(target):
    """Build a probe that has every non-diagnostic message the target needs."""
    probe = bare_probe(FakeClock(), diagnostic_samples=3)
    probe.configure_target(target)
    probe.required_counts = {topic: 1 for topic in probe.topic_types}
    probe.messages = {topic: [object()] for topic in probe.topic_types}
    probe.messages.pop("/diagnostics", None)
    probe.messages["/diagnostics"] = []
    probe.observed_dynamic_tf_edges.add(probe_module.REQUIRED_DYNAMIC_TF_EDGE)
    return probe


def test_simulator_collection_completes_without_diagnostics():
    """The simulator never publishes /diagnostics, so waiting for it would run out the timeout."""
    assert complete_probe("simulator").complete()
    assert not complete_probe("hardware").complete()


def stamped(seconds, frame):
    """Return a kept (stamp, frame) pair."""
    return (Time(seconds=seconds), frame)


def tf_probe(history, kept, odom_frame="base_footprint"):
    """
    Build a probe whose TF buffer holds odom -> base_footprint over ``history``.

    The buffer is a real tf2 one, so its refusal to look up a time outside the
    history is tf2's own. ``kept`` maps a topic to its (stamp, frame) pairs and
    every other checked topic gets one stamp in the middle of the history.
    """
    buffer = Buffer()
    if history is not None:
        start, end = history
        for index in range(int(round((end - start) * 10)) + 1):
            transform = TransformStamped()
            transform.header.frame_id = "odom"
            transform.child_frame_id = odom_frame
            seconds = start + index * 0.1
            transform.header.stamp.sec = int(seconds)
            transform.header.stamp.nanosec = int(round((seconds % 1) * 1e9))
            transform.transform.rotation.w = 1.0
            buffer.set_transform(transform, "test")
    for child in ("laser_link", "cam_1_color_optical_frame"):
        static = TransformStamped()
        static.header.frame_id = "base_footprint"
        static.child_frame_id = child
        static.transform.rotation.w = 1.0
        buffer.set_transform_static(static, "test")
    probe = object.__new__(PhysicalContractProbe)
    probe.tf_buffer = buffer
    middle = 20.0 if history is None else (history[0] + history[1]) / 2.0
    probe.recent_stamps = {
        topic: deque(
            kept.get(topic, [stamped(middle, "laser_link")]),
            maxlen=probe_module.RECENT_STAMPS,
        )
        for topic in probe_module.TF_CHECKED_TOPICS
    }
    return probe


def test_first_stamps_before_the_tf_history_do_not_fail_the_check():
    """
    The listener hears /tf after the sensors: the first stamps can predate it.

    This is the 3 of 20 simulator runs on a GPU that failed with "Lookup would
    require extrapolation into the past". The topic's later stamp is inside the
    history and resolves.
    """
    kept = {
        "/scan": [
            stamped(19.933, "laser_link"),
            stamped(20.1, "laser_link"),
            stamped(21.0, "laser_link"),
        ]
    }
    probe = tf_probe((20.238, 21.564), kept)
    errors = []
    probe.validate_timestamped_tf(errors)
    assert errors == []


def test_a_stamp_newer_than_the_newest_transform_is_not_judged():
    """tf2 does not extrapolate forward either, so a newer stamp cannot fail the check."""
    kept = {"/scan": [stamped(21.0, "laser_link"), stamped(21.6, "laser_link")]}
    probe = tf_probe((20.238, 21.564), kept)
    errors = []
    probe.validate_timestamped_tf(errors)
    assert errors == []


def test_a_frame_that_does_not_resolve_still_fails():
    """A frame missing from TF fails at every stamp, so the relaxed check keeps it."""
    kept = {"/scan": [stamped(21.0, "no_such_link")]}
    probe = tf_probe((20.238, 21.564), kept)
    errors = []
    probe.validate_timestamped_tf(errors)
    assert errors == [
        "/scan: cannot resolve odom -> no_such_link at any of its last 1 stamps "
        "(21.000..21.000 s)"
    ]


@pytest.mark.parametrize("seconds", [1.79e9, 5.0, 30.0])
def test_stamps_outside_the_tf_time_base_fail(seconds):
    """A wall-clock, zero-ish or future stamp is wrong, not early."""
    kept = {"/imu/data": [stamped(seconds, "laser_link")]}
    probe = tf_probe((20.238, 21.564), kept)
    errors = []
    probe.validate_timestamped_tf(errors)
    assert len(errors) == 1
    assert errors[0].startswith(
        "/imu/data: cannot resolve odom -> laser_link at any of its last 1 stamps"
    )


def test_a_buffer_without_the_odometry_edge_fails_every_topic():
    """No odom -> base_footprint history means nothing can be judged."""
    probe = tf_probe(None, {})
    errors = []
    probe.validate_timestamped_tf(errors)
    assert len(errors) == len(probe_module.TF_CHECKED_TOPICS)
    assert all("cannot resolve odom -> " in error for error in errors)


def test_a_topic_with_no_kept_stamps_fails():
    """Nothing to judge is a failure, not a pass."""
    probe = tf_probe((20.238, 21.564), {"/scan": []})
    errors = []
    probe.validate_timestamped_tf(errors)
    assert errors == ["/scan: no stamps were kept for the TF check"]


def test_capture_keeps_the_stamp_and_frame_of_checked_topics():
    """The TF check judges recent stamps, so capture records them as they arrive."""
    probe = bare_probe(FakeClock())
    probe.required_counts["/scan"] = 1
    probe.messages["/scan"] = []
    probe.recent_stamps = {"/scan": deque(maxlen=probe_module.RECENT_STAMPS)}
    message = SimpleNamespace(
        header=SimpleNamespace(
            stamp=TimeMsg(sec=20, nanosec=500_000_000),
            frame_id="laser_link",
        )
    )
    probe.capture("/scan", message)
    probe.capture("/scan", message)
    assert [(stamp.nanoseconds, frame) for stamp, frame in probe.recent_stamps["/scan"]] == [
        (20_500_000_000, "laser_link")
    ] * 2
    assert len(probe.messages["/scan"]) == 1
