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

"""Deterministic, ROS-free tests for the no-motion live-loss probe."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace

import motor_live_loss_probe as probe
import pytest

from motor_live_loss_probe import CONTROLLER_TOPICS
from motor_live_loss_probe import LiveLossEvaluator
from motor_live_loss_probe import ProbeConfig
from motor_live_loss_probe import ProbePhase
from motor_live_loss_probe import STRICT_GRAPH_TOPICS


CONFIG = ProbeConfig(
    baseline_duration=0.1,
    baseline_timeout=1.0,
    baseline_min_messages=1,
    topic_max_age=0.5,
    diagnostic_max_age=0.5,
    feedback_freshness_bound=0.5,
    loss_wait_timeout=0.8,
    quiet_deadline=0.5,
    diagnostic_deadline=0.4,
    driver_exit_deadline=0.8,
    graph_drain_dwell=0.1,
    overall_deadline=1.2,
)


def healthy_freshness(age="0.01"):
    """Return the complete structured baseline diagnostic contract."""
    values = {"feedback_state": "healthy"}
    for stream in ("speed", "encoder", "imu_raw"):
        values["feedback_%s_age_seconds" % stream] = age
        values["feedback_%s_stale" % stream] = "false"
    return values


def failed_freshness(reason="controller feedback stale after report timeout"):
    """Return complete structured live-loss diagnostic evidence."""
    values = {
        "feedback_state": "failed",
        "feedback_reason": reason,
    }
    for stream, age in (("speed", "0.51"), ("encoder", "0.52"), ("imu_raw", "0.53")):
        values["feedback_%s_age_seconds" % stream] = age
        values["feedback_%s_stale" % stream] = "true"
    return values


def test_ros_wrapper_contains_no_mutating_ros_factories():
    """The physical probe must remain an observer, never an actuator client."""
    source = Path(__file__).with_name("motor_live_loss_probe.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    attributes = {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}

    assert not attributes & {
        "create_publisher",
        "create_service",
        "create_client",
        "create_action_server",
        "publish",
    }
    assert not names & {"ActionClient", "ActionServer"}


def test_graph_snapshot_is_timestamped_after_collection():
    source = Path(__file__).with_name("motor_live_loss_probe.py").read_text(
        encoding="utf-8"
    )
    snapshot_at = source.index("snapshot = _graph_snapshot(")
    timestamp_at = source.index("graph_now = time.monotonic()", snapshot_at)
    observe_at = source.index(
        "evaluator.observe_graph(now=graph_now", timestamp_at
    )
    tick_at = source.index("evaluator.tick(graph_now)", observe_at)

    assert snapshot_at < timestamp_at < observe_at < tick_at


def test_motor_diagnostic_requires_exact_name_and_hardware_id():
    expected = SimpleNamespace(
        name=probe.MOTOR_DIAGNOSTIC_NAME,
        hardware_id="/dev/robot/motor",
    )
    foreign_device = SimpleNamespace(
        name=probe.MOTOR_DIAGNOSTIC_NAME,
        hardware_id="/dev/robot/other-motor",
    )
    foreign_name = SimpleNamespace(
        name="another motor diagnostic",
        hardware_id="/dev/robot/motor",
    )

    assert probe._is_expected_motor_diagnostic(
        expected, "/dev/robot/motor"
    )
    assert not probe._is_expected_motor_diagnostic(
        foreign_device, "/dev/robot/motor"
    )
    assert not probe._is_expected_motor_diagnostic(
        foreign_name, "/dev/robot/motor"
    )


def graph(
    evaluator,
    now,
    *,
    exact=True,
    driver=True,
    driver_endpoints=None,
    strict_endpoints=None,
    strict_topic_counts=None,
    cmd_publishers=(),
):
    """Feed one compact graph snapshot."""
    if driver_endpoints is None:
        driver_endpoints = len(CONTROLLER_TOPICS) if driver else 0
    if strict_topic_counts is None:
        if strict_endpoints is None:
            strict_endpoints = len(STRICT_GRAPH_TOPICS) if driver else 1
        strict_topic_counts = {
            topic: int(index < strict_endpoints)
            for index, topic in enumerate(STRICT_GRAPH_TOPICS)
        }
    if strict_endpoints is None:
        strict_endpoints = sum(strict_topic_counts.values())
    evaluator.observe_graph(
        driver_endpoints_exact=exact,
        driver_present=driver,
        driver_endpoint_count=driver_endpoints,
        strict_endpoint_count=strict_endpoints,
        strict_topic_counts=strict_topic_counts,
        cmd_publishers=cmd_publishers,
        evidence={"synthetic": True},
        now=now,
    )


def arm(evaluator, start_at=0.0, duration_margin=0.0):
    """Establish the minimal healthy stationary baseline and arm."""
    evaluator.observe_device(True, start_at)
    graph(evaluator, start_at)
    evaluator.observe_motor_diagnostic(
        0, "healthy", healthy_freshness(), start_at
    )
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(
            topic, start_at, stationary=stationary
        )
    evaluator.tick(start_at)
    armed_at = start_at + CONFIG.baseline_duration + duration_margin
    graph(evaluator, armed_at)
    evaluator.tick(armed_at)
    assert evaluator.phase == ProbePhase.ARMED


def lose(evaluator, now=0.2):
    """Apply the required present-to-absent trigger."""
    evaluator.observe_device(False, now)
    assert evaluator.phase == ProbePhase.LOSS
    return now


def freshness_error(evaluator, now):
    """Publish a qualifying motor-loss diagnostic."""
    evaluator.observe_motor_diagnostic(
        2,
        "Controller receive stream stale",
        failed_freshness(),
        now,
    )


def test_humble_byte_diagnostic_levels_are_normalized():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)

    evaluator.observe_motor_diagnostic(
        b"\x00", "healthy", healthy_freshness(), 0.0
    )
    assert evaluator.latest_diagnostic_level == 0

    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    evaluator.observe_motor_diagnostic(
        bytearray(b"\x02"),
        "Controller receive stream stale",
        failed_freshness(),
        0.15,
    )
    assert evaluator.latest_diagnostic_level == 2
    assert evaluator.pending_error_at == pytest.approx(0.15)

    loss_at = lose(evaluator, 0.2)
    complete_loss(evaluator, loss_at)

    assert evaluator.phase == ProbePhase.PASSED


@pytest.mark.parametrize(
    "level",
    [b"", b"\x00\x02", -1, 256, True, 2.0, "2", None],
)
def test_invalid_diagnostic_level_is_rejected(level):
    evaluator = LiveLossEvaluator(CONFIG)

    with pytest.raises(ValueError, match="diagnostic level"):
        evaluator.observe_motor_diagnostic(
            level, "invalid", healthy_freshness(), 0.0
        )


def test_event_timestamps_must_be_nondecreasing():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.1)
    evaluator.tick(0.1)

    with pytest.raises(ValueError, match="nondecreasing"):
        evaluator.tick(0.09)


def complete_loss(evaluator, loss_at):
    """Supply prompt driver and strict-graph teardown evidence."""
    graph(
        evaluator,
        loss_at + 0.2,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    graph(
        evaluator,
        loss_at + 0.2 + CONFIG.graph_drain_dwell + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    evaluator.tick(loss_at + CONFIG.quiet_deadline + 0.01)


def test_failure_diagnostic_then_device_loss_passes():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    diagnostic_at = 0.19
    freshness_error(evaluator, diagnostic_at)

    assert evaluator.phase == ProbePhase.ARMED
    assert evaluator.error_at is None
    assert evaluator.pending_error_at == pytest.approx(diagnostic_at)

    loss_at = lose(evaluator, 0.2)
    assert evaluator.error_at == pytest.approx(diagnostic_at)
    assert evaluator.pending_error_at is None

    complete_loss(evaluator, loss_at)

    assert evaluator.phase == ProbePhase.PASSED
    report = evaluator.report()
    assert report["timing_offsets_seconds"]["failure_started"] == pytest.approx(
        diagnostic_at
    )
    assert report["timing_offsets_seconds"]["freshness_error"] == pytest.approx(
        diagnostic_at
    )
    assert report["accepted_failure_diagnostic"] == {
        "level": 2,
        "at": pytest.approx(diagnostic_at),
        "message": "Controller receive stream stale",
        "values": failed_freshness(),
    }


def test_device_loss_then_failure_diagnostic_passes():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    loss_at = lose(evaluator, 0.2)
    diagnostic_at = loss_at + 0.01
    freshness_error(evaluator, diagnostic_at)

    assert evaluator.error_at == pytest.approx(diagnostic_at)
    assert evaluator.pending_error_at is None

    complete_loss(evaluator, loss_at)

    assert evaluator.phase == ProbePhase.PASSED


def test_pre_arm_failure_diagnostic_cannot_satisfy_live_loss():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)
    freshness_error(evaluator, 0.0)

    assert evaluator.phase == ProbePhase.BASELINE
    assert evaluator.pending_error_at is None

    arm(evaluator)
    loss_at = lose(evaluator)

    assert evaluator.error_at is None
    evaluator.tick(loss_at + CONFIG.diagnostic_deadline + 0.01)
    assert evaluator.phase == ProbePhase.FAILED
    assert "no freshness-specific motor ERROR" in evaluator.reasons[0]


def test_udev_lag_retains_pending_failure_until_device_loss():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    assert evaluator.pending_error_at == pytest.approx(0.15)

    evaluator.observe_device(True, 0.16)
    evaluator.tick(0.16)
    evaluator.observe_device(True, 0.17)
    evaluator.tick(0.17)
    assert evaluator.phase == ProbePhase.ARMED
    assert evaluator.pending_error_at == pytest.approx(0.15)

    loss_at = lose(evaluator, 0.18)
    assert evaluator.error_at == pytest.approx(0.15)
    complete_loss(evaluator, loss_at)

    assert evaluator.phase == ProbePhase.PASSED


def test_pending_failure_without_device_loss_fails_closed():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    diagnostic_at = 0.15
    freshness_error(evaluator, diagnostic_at)

    evaluator.observe_device(True, diagnostic_at + 0.1)
    evaluator.tick(diagnostic_at + CONFIG.diagnostic_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    assert "ERROR was not followed by device loss" in evaluator.reasons[0]


def test_repeated_pending_errors_do_not_extend_device_loss_window():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    first_error_at = 0.15
    freshness_error(evaluator, first_error_at)

    evaluator.observe_device(True, 0.5)
    evaluator.tick(0.5)
    freshness_error(evaluator, 0.5)

    assert evaluator.pending_error_at == pytest.approx(first_error_at)
    evaluator.observe_device(
        False, first_error_at + CONFIG.diagnostic_deadline + 0.01
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "ERROR was not followed by device loss" in evaluator.reasons[0]


def test_stale_pending_failure_diagnostic_is_not_accepted():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.11)

    evaluator.observe_device(
        False, 0.11 + CONFIG.diagnostic_deadline + 0.01
    )

    assert evaluator.error_at is None
    assert evaluator.phase == ProbePhase.FAILED
    assert "ERROR was not followed by device loss" in evaluator.reasons[0]


def test_nonqualifying_diagnostic_after_accepted_error_fails_terminally():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    loss_at = lose(evaluator)
    freshness_error(evaluator, loss_at + 0.05)
    accepted = evaluator.report()["accepted_failure_diagnostic"]

    evaluator.observe_motor_diagnostic(
        0,
        "healthy again",
        healthy_freshness(),
        loss_at + 0.1,
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "contradicted the accepted terminal" in evaluator.reasons[0]
    assert evaluator.report()["accepted_failure_diagnostic"] == accepted


def test_invalid_error_after_accepted_error_fails_terminally():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    loss_at = lose(evaluator)
    freshness_error(evaluator, loss_at + 0.05)

    evaluator.observe_motor_diagnostic(
        2,
        "generic error",
        failed_freshness(reason="generic controller failure"),
        loss_at + 0.1,
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "contradicted the accepted terminal" in evaluator.reasons[0]
    assert evaluator.report()["invalid_error_evidence"]


def test_repeated_qualifying_error_keeps_first_accepted_payload():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    loss_at = lose(evaluator)
    first_error_at = loss_at + 0.01
    freshness_error(evaluator, first_error_at)

    repeated_error_at = loss_at + CONFIG.diagnostic_deadline + 0.01
    freshness_error(evaluator, repeated_error_at)

    assert evaluator.phase == ProbePhase.LOSS
    assert evaluator.error_at == pytest.approx(first_error_at)
    assert evaluator.report()["accepted_failure_diagnostic"][
        "at"
    ] == pytest.approx(first_error_at)


def test_pending_failure_reset_discards_all_tentative_evidence():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    evaluator.observe_controller_topic("/voltage", 0.2)
    graph(
        evaluator,
        0.2,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )

    assert evaluator.topics["/voltage"].post_loss_count == 1
    assert evaluator.driver_gone_at == pytest.approx(0.2)
    assert evaluator.strict_zero_since == pytest.approx(0.2)

    evaluator.observe_motor_diagnostic(
        0, "healthy again", healthy_freshness(), 0.25
    )

    assert evaluator.phase == ProbePhase.BASELINE
    assert evaluator.healthy_since is None
    assert evaluator.armed_at is None
    assert evaluator.baseline_strict_topic_counts is None
    assert evaluator.pending_error_at is None
    assert evaluator.pending_error_evidence is None
    assert evaluator.accepted_error_evidence is None
    assert evaluator.failure_started_at is None
    assert evaluator.driver_gone_at is None
    assert evaluator.strict_zero_since is None
    assert evaluator.strict_drained_at is None
    assert evaluator.topics["/voltage"].post_loss_count == 0
    assert evaluator.topics["/voltage"].last_post_loss_at is None
    assert evaluator.topics["/voltage"].count == 0
    assert not evaluator.graph_seen

    arm(evaluator, start_at=0.26, duration_margin=0.01)
    assert evaluator.armed_at - 0.26 >= CONFIG.baseline_duration

    loss_at = lose(evaluator, 0.4)
    freshness_error(evaluator, 0.41)
    complete_loss(evaluator, loss_at)

    assert evaluator.phase == ProbePhase.PASSED
    assert evaluator.report()["timing_offsets_seconds"][
        "failure_started"
    ] == pytest.approx(loss_at)


def test_repeated_pending_resets_do_not_extend_baseline_timeout():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)

    freshness_error(evaluator, 0.15)
    evaluator.observe_motor_diagnostic(
        0, "healthy again", healthy_freshness(), 0.16
    )
    arm(evaluator, start_at=0.17, duration_margin=0.01)

    freshness_error(evaluator, 0.3)
    evaluator.observe_motor_diagnostic(
        0, "healthy again", healthy_freshness(), 0.31
    )

    evaluator.observe_device(True, 0.32)
    graph(evaluator, 0.32)
    evaluator.observe_motor_diagnostic(
        0, "healthy", healthy_freshness(), 0.32
    )
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(
            topic, 0.32, stationary=stationary
        )
    evaluator.tick(0.32)
    graph(evaluator, CONFIG.baseline_timeout + 0.01)
    evaluator.tick(CONFIG.baseline_timeout + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    assert "baseline was not established" in evaluator.reasons[0]
    assert evaluator.report()["timing_offsets_seconds"][
        "baseline_started"
    ] == pytest.approx(0.0)


def test_device_loss_cannot_bypass_pending_reset_rebaseline():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    graph(
        evaluator,
        0.2,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    evaluator.observe_motor_diagnostic(
        0, "healthy again", healthy_freshness(), 0.25
    )

    evaluator.observe_device(False, 0.26)

    assert evaluator.phase == ProbePhase.FAILED
    assert "before the probe armed" in evaluator.reasons[0]


def test_device_loss_requires_current_pre_edge_baseline_without_pending_error():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    evaluator.observe_motor_diagnostic(
        1, "warning", healthy_freshness(), 0.15
    )

    evaluator.observe_device(False, 0.16)

    assert evaluator.phase == ProbePhase.FAILED
    assert evaluator.loss_at is None
    assert "healthy baseline was lost" in evaluator.reasons[0]


def test_pending_failure_preserves_timely_pre_device_shutdown_evidence():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    graph(
        evaluator,
        0.25,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    graph(
        evaluator,
        0.25 + CONFIG.graph_drain_dwell + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )

    lose(evaluator, 0.5)
    evaluator.tick(0.15 + CONFIG.quiet_deadline + 0.01)

    assert evaluator.phase == ProbePhase.PASSED
    assert evaluator.driver_gone_at == pytest.approx(0.25)
    assert evaluator.strict_drained_at == pytest.approx(
        0.25 + CONFIG.graph_drain_dwell + 0.01
    )


def test_pre_device_failure_anchors_topic_quiet_deadline():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    lose(evaluator, 0.5)

    evaluator.observe_controller_topic(
        "/voltage", 0.15 + CONFIG.quiet_deadline + 0.01
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "published after" in evaluator.reasons[0]


def test_pre_device_failure_anchors_driver_exit_deadline():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    lose(evaluator, 0.5)

    graph(
        evaluator,
        0.15 + CONFIG.driver_exit_deadline + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=2,
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "driver disappeared after" in evaluator.reasons[0]


def test_pre_device_failure_anchors_strict_graph_deadline():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    freshness_error(evaluator, 0.15)
    lose(evaluator, 0.5)
    graph(
        evaluator,
        0.6,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=2,
    )

    graph(
        evaluator,
        0.15 + CONFIG.overall_deadline + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "strict graph drained after" in evaluator.reasons[0]


def test_passing_trace_records_all_required_evidence():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    evaluator.observe_controller_topic("/voltage", t0 + 0.1)
    freshness_error(evaluator, t0 + 0.2)
    graph(
        evaluator,
        t0 + 0.3,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=3,
    )
    graph(
        evaluator,
        t0 + 0.4,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    graph(
        evaluator,
        t0 + 0.4 + CONFIG.graph_drain_dwell + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    evaluator.tick(t0 + CONFIG.quiet_deadline + 0.02)

    assert evaluator.phase == ProbePhase.PASSED
    report = evaluator.report()
    assert report["outcome"] == "PASS"
    assert report["exit_code"] == 0
    assert report["topics"]["/voltage"]["post_loss_count"] == 1
    assert report["topics"]["/voltage"]["post_failure_count"] == 1
    assert report["topics"]["/voltage"][
        "last_post_failure_at"
    ] == report["topics"]["/voltage"]["last_post_loss_at"]
    assert report["baseline_freshness_evidence"]["values"][
        "feedback_state"
    ] == "healthy"
    assert report["timing_offsets_seconds"]["freshness_error"] == pytest.approx(
        t0 + 0.2
    )
    assert all(
        count >= 1
        for count in report["graph"][
            "baseline_strict_topic_counts"
        ].values()
    )


def test_baseline_does_not_arm_with_one_strict_topic_missing():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)
    strict_counts = {topic: 1 for topic in STRICT_GRAPH_TOPICS}
    strict_counts["/scan"] = 0
    graph(evaluator, 0.0, strict_topic_counts=strict_counts)
    evaluator.observe_motor_diagnostic(
        0, "healthy", healthy_freshness(), 0.0
    )
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(topic, 0.0, stationary=stationary)

    evaluator.tick(0.0)
    evaluator.tick(CONFIG.baseline_duration + 0.01)

    assert evaluator.phase == ProbePhase.BASELINE
    assert evaluator.baseline_strict_topic_counts is None


def test_strict_graph_requires_observed_zero_dwell_before_pass():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    freshness_error(evaluator, t0 + 0.1)
    graph(
        evaluator,
        t0 + 0.2,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )

    evaluator.tick(
        t0 + 0.2 + CONFIG.graph_drain_dwell - 0.01
    )
    assert evaluator.phase == ProbePhase.LOSS
    assert evaluator.strict_drained_at is None

    graph(
        evaluator,
        t0 + 0.2 + CONFIG.graph_drain_dwell + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    evaluator.tick(t0 + CONFIG.quiet_deadline + 0.01)

    assert evaluator.phase == ProbePhase.PASSED
    assert evaluator.strict_drained_at is not None


def test_strict_graph_reappearance_restarts_zero_dwell():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    freshness_error(evaluator, t0 + 0.1)
    graph(
        evaluator,
        t0 + 0.2,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    graph(
        evaluator,
        t0 + 0.25,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=1,
    )
    graph(
        evaluator,
        t0 + 0.3,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    graph(
        evaluator,
        t0 + 0.3 + CONFIG.graph_drain_dwell - 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )

    assert evaluator.strict_drained_at is None

    graph(
        evaluator,
        t0 + 0.3 + CONFIG.graph_drain_dwell + 0.01,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=0,
    )
    assert evaluator.strict_drained_at is not None


def test_baseline_does_not_arm_without_exact_driver_publishers():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)
    graph(evaluator, 0.0, exact=False)
    evaluator.observe_motor_diagnostic(
        0, "healthy", healthy_freshness(), 0.0
    )
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(topic, 0.0, stationary=stationary)
    evaluator.tick(0.0)
    evaluator.tick(CONFIG.baseline_duration + 0.01)

    assert evaluator.phase == ProbePhase.BASELINE


def test_baseline_rejects_incomplete_freshness_fields():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)
    graph(evaluator, 0.0)
    incomplete = healthy_freshness()
    del incomplete["feedback_imu_raw_age_seconds"]
    evaluator.observe_motor_diagnostic(0, "healthy", incomplete, 0.0)
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(topic, 0.0, stationary=stationary)
    evaluator.tick(0.0)
    evaluator.tick(CONFIG.baseline_duration + 0.01)

    assert evaluator.phase == ProbePhase.BASELINE
    report = evaluator.report()
    assert not report["latest_motor_diagnostic"]["baseline_freshness_valid"]
    assert any(
        "feedback_imu_raw_age_seconds" in error
        for error in report["latest_motor_diagnostic"][
            "baseline_freshness_errors"
        ]
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("feedback_state", "failed"),
        ("feedback_speed_age_seconds", "nan"),
        ("feedback_encoder_age_seconds", "-0.1"),
        ("feedback_imu_raw_age_seconds", "0.51"),
        ("feedback_speed_stale", "true"),
    ],
)
def test_baseline_rejects_false_or_out_of_bound_freshness(field, value):
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)
    graph(evaluator, 0.0)
    values = healthy_freshness()
    values[field] = value
    evaluator.observe_motor_diagnostic(0, "healthy", values, 0.0)
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(topic, 0.0, stationary=stationary)
    evaluator.tick(0.0)
    evaluator.tick(CONFIG.baseline_duration + 0.01)

    assert evaluator.phase == ProbePhase.BASELINE
    assert not evaluator.latest_baseline_freshness_valid


def test_armed_probe_fails_if_structured_health_becomes_invalid():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    invalid = healthy_freshness()
    invalid["feedback_encoder_stale"] = "true"
    evaluator.observe_motor_diagnostic(0, "healthy", invalid, 0.15)
    evaluator.tick(0.15)

    assert evaluator.phase == ProbePhase.FAILED
    assert "healthy baseline was lost" in evaluator.reasons[0]
    assert evaluator.report()["exit_code"] == 2


def test_cached_publication_after_quiet_deadline_fails():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    evaluator.observe_controller_topic(
        "/joint_states", t0 + CONFIG.quiet_deadline + 0.01, stationary=True
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "published after" in evaluator.reasons[0]


def test_missing_freshness_error_fails_at_deadline():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    evaluator.observe_motor_diagnostic(
        2, "generic error", {"failure_count": "1"}, t0 + 0.2
    )
    evaluator.tick(t0 + CONFIG.diagnostic_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    assert "no freshness-specific motor ERROR" in evaluator.reasons[0]
    assert evaluator.report()["invalid_error_evidence"]


def test_complete_but_generic_error_does_not_qualify():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    values = failed_freshness(reason="generic controller failure")
    evaluator.observe_motor_diagnostic(2, "generic error", values, t0 + 0.2)
    evaluator.tick(t0 + CONFIG.diagnostic_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    invalid = evaluator.report()["invalid_error_evidence"][-1]
    assert any("failure reason" in error for error in invalid["validation_errors"])


def test_error_with_healthy_feedback_state_does_not_qualify():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    values = failed_freshness()
    values["feedback_state"] = "healthy"
    evaluator.observe_motor_diagnostic(
        2, "Controller reports are stale", values, t0 + 0.2
    )
    evaluator.tick(t0 + CONFIG.diagnostic_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    invalid = evaluator.report()["invalid_error_evidence"][-1]
    assert "feedback_state must be failed" in invalid["validation_errors"]


def test_failure_error_rejects_incomplete_per_stream_evidence():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    values = failed_freshness()
    del values["feedback_encoder_stale"]
    evaluator.observe_motor_diagnostic(
        2, "Controller reports are stale", values, t0 + 0.2
    )
    evaluator.tick(t0 + CONFIG.diagnostic_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    invalid = evaluator.report()["invalid_error_evidence"][-1]
    assert any(
        "feedback_encoder_stale" in error
        for error in invalid["validation_errors"]
    )


def test_driver_survival_past_deadline_fails():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    freshness_error(evaluator, t0 + 0.2)
    graph(evaluator, t0 + 0.7)
    evaluator.tick(t0 + CONFIG.driver_exit_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    assert "driver remained present" in evaluator.reasons[0]


def test_driver_exit_before_diagnostic_fails():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    graph(
        evaluator,
        t0 + 0.2,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=2,
    )

    assert evaluator.phase == ProbePhase.FAILED
    assert "before a freshness-specific ERROR" in evaluator.reasons[0]

    freshness_error(evaluator, t0 + 0.3)
    assert evaluator.phase == ProbePhase.FAILED
    assert evaluator.error_at is None
    assert "before a freshness-specific ERROR" in evaluator.reasons[0]


@pytest.mark.parametrize("event_kind", ["publisher", "message"])
@pytest.mark.parametrize("phase", ["baseline", "armed", "loss"])
def test_cmd_vel_activity_is_unsafe_in_every_phase(event_kind, phase):
    evaluator = LiveLossEvaluator(CONFIG)
    if phase in ("armed", "loss"):
        arm(evaluator)
    else:
        evaluator.observe_device(True, 0.0)
    event_time = 0.05 if phase == "baseline" else 0.15
    if phase == "loss":
        lose(evaluator)
        event_time = 0.25

    if event_kind == "publisher":
        graph(evaluator, event_time, cmd_publishers=("/unsafe_source",))
    else:
        evaluator.observe_cmd_message(event_time)

    assert evaluator.phase == ProbePhase.FAILED
    assert evaluator.unsafe
    assert evaluator.outcome == "UNSAFE_FAIL"
    assert "/cmd_vel" in evaluator.reasons[0]
    assert evaluator.report()["exit_code"] == 2


def test_device_loss_before_arm_fails():
    evaluator = LiveLossEvaluator(CONFIG)
    evaluator.observe_device(True, 0.0)
    evaluator.observe_device(False, 0.05)

    assert evaluator.phase == ProbePhase.FAILED
    assert "before the probe armed" in evaluator.reasons[0]
    assert evaluator.report()["exit_code"] == 2


def test_no_device_loss_before_wait_timeout_fails():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    final_time = 0.1 + CONFIG.loss_wait_timeout + 0.01
    graph(evaluator, final_time)
    evaluator.observe_motor_diagnostic(
        0, "healthy", healthy_freshness(), final_time
    )
    for topic in CONTROLLER_TOPICS:
        stationary = True if topic in ("/joint_states", "/vel_raw") else None
        evaluator.observe_controller_topic(
            topic, final_time, stationary=stationary
        )
    evaluator.tick(final_time)

    assert evaluator.phase == ProbePhase.FAILED
    assert "was not removed" in evaluator.reasons[0]


def test_device_observation_cannot_bypass_loss_wait_timeout():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    observed_at = evaluator.armed_at + CONFIG.loss_wait_timeout + 0.01

    evaluator.observe_device(False, observed_at)

    assert evaluator.phase == ProbePhase.FAILED
    assert evaluator.loss_at is None
    assert "was not removed" in evaluator.reasons[0]
    assert evaluator.report()["exit_code"] == 2


def test_slow_strict_teardown_fails():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    t0 = lose(evaluator)
    freshness_error(evaluator, t0 + 0.2)
    graph(
        evaluator,
        t0 + 0.3,
        exact=False,
        driver=False,
        driver_endpoints=0,
        strict_endpoints=2,
    )
    evaluator.tick(t0 + CONFIG.overall_deadline + 0.01)

    assert evaluator.phase == ProbePhase.FAILED
    assert "strict graph remained active" in evaluator.reasons[0]
    assert evaluator.report()["exit_code"] == 1


def test_post_loss_unsafe_failure_uses_safety_exit_code_two():
    evaluator = LiveLossEvaluator(CONFIG)
    arm(evaluator)
    lose(evaluator)
    evaluator.observe_cmd_message(0.25)

    assert evaluator.unsafe
    assert evaluator.report()["exit_code"] == 2


def test_setup_interrupt_emits_exit_code_130(monkeypatch, capsys):
    def interrupt(_args):
        raise KeyboardInterrupt

    monkeypatch.setattr(probe, "_run_ros", interrupt)

    assert probe.main(["--confirm-wheels-secured"]) == 130
    payload = json.loads(capsys.readouterr().out)
    assert payload["exit_code"] == 130
