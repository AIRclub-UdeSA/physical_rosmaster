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

"""Focused tests for required diagnostic tracking in the physical probe."""

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest

import physical_contract_probe as probe_module
from physical_contract_probe import finite_positive
from physical_contract_probe import PhysicalContractProbe
from physical_contract_probe import RequiredDiagnosticObservation


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
