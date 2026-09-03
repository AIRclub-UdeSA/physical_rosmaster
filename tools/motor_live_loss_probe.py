#!/usr/bin/env python3
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

"""
Observe the no-motion ROS contract during live motor-controller loss.

The deterministic evaluator in this module has no ROS dependency.  The ROS
adapter is loaded only by ``main`` and is deliberately observer-only: it creates
subscriptions and inspects the graph, but creates no publishers, services, or
actions and sends no actuator command.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from enum import Enum
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, Mapping, Optional, Sequence, Tuple


MOTOR_DIAGNOSTIC_NAME = (
    "yahboomcar_bringup: motor controller and onboard sensors"
)
CONTROLLER_TOPICS = (
    "/joint_states",
    "/vel_raw",
    "/voltage",
    "/edition",
    "/imu/data_raw",
    "/imu/mag",
)
STATIONARY_TOPICS = {"/joint_states", "/vel_raw"}
FEEDBACK_STREAMS = ("speed", "encoder", "imu_raw")
STRICT_GRAPH_TOPICS = (
    *CONTROLLER_TOPICS,
    "/odom",
    "/imu/data",
    "/scan",
    "/cam_1/color/image_raw",
    "/cam_1/depth/image_raw",
    "/cam_1/depth/color/points",
    "/tf",
    "/tf_static",
)


class ProbePhase(str, Enum):
    """State of the deterministic live-loss contract."""

    BASELINE = "baseline"
    ARMED = "armed"
    LOSS = "loss"
    PASSED = "passed"
    FAILED = "failed"


@dataclass(frozen=True)
class ProbeConfig:
    """Timing and sampling limits for a live-loss observation."""

    baseline_duration: float = 2.0
    baseline_timeout: float = 30.0
    baseline_min_messages: int = 3
    topic_max_age: float = 0.5
    diagnostic_max_age: float = 1.5
    feedback_freshness_bound: float = 0.5
    loss_wait_timeout: float = 60.0
    quiet_deadline: float = 0.75
    diagnostic_deadline: float = 0.75
    driver_exit_deadline: float = 1.5
    graph_drain_dwell: float = 0.25
    overall_deadline: float = 8.0

    def validate(self) -> None:
        """Reject nonsensical limits before a physical observation starts."""
        numeric = {
            key: value
            for key, value in asdict(self).items()
            if key != "baseline_min_messages"
        }
        if any(not math.isfinite(value) or value <= 0.0 for value in numeric.values()):
            raise ValueError("all timing limits must be finite and positive")
        if self.baseline_min_messages <= 0:
            raise ValueError("baseline_min_messages must be positive")
        if self.diagnostic_deadline > self.driver_exit_deadline:
            raise ValueError("diagnostic_deadline must not exceed driver_exit_deadline")
        if self.driver_exit_deadline > self.overall_deadline:
            raise ValueError("driver_exit_deadline must not exceed overall_deadline")
        if self.quiet_deadline > self.overall_deadline:
            raise ValueError("quiet_deadline must not exceed overall_deadline")
        if self.graph_drain_dwell > self.overall_deadline:
            raise ValueError(
                "graph_drain_dwell must not exceed overall_deadline"
            )


@dataclass
class TopicEvidence:
    """Bounded timing evidence for one controller-derived topic."""

    count: int = 0
    last_at: Optional[float] = None
    post_loss_count: int = 0
    last_post_loss_at: Optional[float] = None


def _diagnostic_level_value(level: object) -> int:
    """Normalize ROS Humble's one-byte diagnostic level representation."""
    if isinstance(level, (bytes, bytearray, memoryview)):
        encoded = bytes(level)
        if len(encoded) != 1:
            raise ValueError("diagnostic level must contain exactly one byte")
        return encoded[0]
    if isinstance(level, bool) or not isinstance(level, int):
        raise ValueError("diagnostic level must be an integer or one byte")
    parsed = level
    if not 0 <= parsed <= 255:
        raise ValueError("diagnostic level must be between 0 and 255")
    return parsed


def _is_expected_motor_diagnostic(status: object, hardware_id: str) -> bool:
    """Match only the configured controller's exact diagnostic status."""
    return (
        getattr(status, "name", None) == MOTOR_DIAGNOSTIC_NAME
        and getattr(status, "hardware_id", None) == hardware_id
    )


class LiveLossEvaluator:
    """Evaluate timestamped observations without ROS or hardware dependencies."""

    def __init__(self, config: ProbeConfig, started_at: float = 0.0) -> None:
        config.validate()
        if not math.isfinite(started_at):
            raise ValueError("started_at must be finite")
        self.config = config
        self.started_at = started_at
        self.last_event_at = started_at
        self.baseline_started_at = started_at
        self.phase = ProbePhase.BASELINE
        self.outcome = "RUNNING"
        self.unsafe = False
        self.failure_exit_code: Optional[int] = None
        self.reasons = []
        self.healthy_since: Optional[float] = None
        self.armed_at: Optional[float] = None
        self.loss_at: Optional[float] = None
        self.failure_started_at: Optional[float] = None
        self.error_at: Optional[float] = None
        self.pending_error_at: Optional[float] = None
        self.pending_error_evidence: Optional[Dict[str, object]] = None
        self.accepted_error_evidence: Optional[Dict[str, object]] = None
        self.driver_gone_at: Optional[float] = None
        self.strict_zero_since: Optional[float] = None
        self.strict_drained_at: Optional[float] = None
        self.completed_at: Optional[float] = None
        self.device_present: Optional[bool] = None
        self.device_was_present = False
        self.latest_diagnostic_level: Optional[int] = None
        self.latest_diagnostic_at: Optional[float] = None
        self.latest_diagnostic_message = ""
        self.latest_diagnostic_values: Dict[str, str] = {}
        self.latest_baseline_freshness_valid = False
        self.latest_baseline_freshness_errors = []
        self.baseline_freshness_evidence: Optional[Dict[str, object]] = None
        self.invalid_error_evidence = []
        self.graph_seen = False
        self.latest_graph_at: Optional[float] = None
        self.driver_endpoints_exact = False
        self.driver_present = False
        self.driver_endpoint_count = 0
        self.strict_endpoint_count = 0
        self.strict_topic_counts = {
            topic: 0 for topic in STRICT_GRAPH_TOPICS
        }
        self.baseline_strict_topic_counts: Optional[Dict[str, int]] = None
        self.latest_graph_evidence: Dict[str, object] = {}
        self.stationary_topics_seen = set()
        self.topics = {topic: TopicEvidence() for topic in CONTROLLER_TOPICS}

    @property
    def terminal(self) -> bool:
        """Return whether the evaluator reached a pass or failure result."""
        return self.phase in (ProbePhase.PASSED, ProbePhase.FAILED)

    def _relative(self, timestamp: Optional[float]) -> Optional[float]:
        if timestamp is None:
            return None
        return round(timestamp - self.started_at, 6)

    def _check_time(self, now: float) -> None:
        if not math.isfinite(now) or now < self.started_at:
            raise ValueError("event time must be finite and not precede started_at")
        if now < self.last_event_at:
            raise ValueError("event time must be nondecreasing")
        self.last_event_at = now

    def _fail(self, reason: str, now: float, unsafe: bool = False) -> None:
        self._check_time(now)
        if self.terminal:
            return
        self.phase = ProbePhase.FAILED
        self.outcome = "UNSAFE_FAIL" if unsafe else "FAIL"
        self.unsafe = unsafe
        self.failure_exit_code = (
            2 if unsafe or self.loss_at is None else 1
        )
        self.reasons.append(reason)
        self.completed_at = now

    def abort(self, reason: str, now: float, unsafe: bool = False) -> None:
        """Terminate an incomplete run with a machine-readable failure."""
        self._fail(reason, now, unsafe=unsafe)

    def _failure_diagnostic_evidence(
        self, message: str, values: Mapping[str, str], now: float
    ) -> Dict[str, object]:
        """Snapshot one qualifying diagnostic independently of later status."""
        return {
            "level": 2,
            "at": now,
            "message": str(message),
            "values": {str(key): str(value) for key, value in values.items()},
        }

    def _accept_failure_diagnostic(
        self, evidence: Mapping[str, object]
    ) -> None:
        """Retain the first confirmed terminal diagnostic and its payload."""
        if self.error_at is not None:
            return
        self.error_at = float(evidence["at"])
        self.accepted_error_evidence = {
            "level": int(evidence["level"]),
            "at": self.error_at,
            "message": str(evidence["message"]),
            "values": dict(evidence["values"]),
        }

    def _reset_pending_failure(self) -> None:
        """Discard tentative evidence and require a wholly fresh baseline."""
        self.phase = ProbePhase.BASELINE
        self.healthy_since = None
        self.armed_at = None
        self.baseline_strict_topic_counts = None
        self.pending_error_at = None
        self.pending_error_evidence = None
        self.failure_started_at = None
        self.driver_gone_at = None
        self.strict_zero_since = None
        self.strict_drained_at = None
        self.graph_seen = False
        self.latest_graph_at = None
        self.driver_endpoints_exact = False
        self.driver_present = False
        self.driver_endpoint_count = 0
        self.strict_endpoint_count = 0
        self.strict_topic_counts = {
            topic: 0 for topic in STRICT_GRAPH_TOPICS
        }
        self.latest_graph_evidence = {}
        self.stationary_topics_seen.clear()
        if not (
            self.latest_diagnostic_level == 0
            and self.latest_baseline_freshness_valid
        ):
            self.baseline_freshness_evidence = None
        for evidence in self.topics.values():
            evidence.count = 0
            evidence.last_at = None
            evidence.post_loss_count = 0
            evidence.last_post_loss_at = None

    def observe_device(self, present: bool, now: float) -> None:
        """Observe the stable motor alias and identify its present-to-absent edge."""
        self._check_time(now)
        if self.terminal:
            return
        previous = self.device_present
        pre_edge_baseline_healthy = (
            self.phase == ProbePhase.ARMED
            and previous is True
            and self.pending_error_at is None
            and self._baseline_healthy(now)
        )
        self.device_present = bool(present)
        self.device_was_present = self.device_was_present or bool(present)

        if not present and self.phase in (ProbePhase.BASELINE, ProbePhase.ARMED):
            if self.phase != ProbePhase.ARMED or previous is not True:
                self._fail("motor device loss occurred before the probe armed", now)
                return
            if self.armed_at is None or (
                now - self.armed_at > self.config.loss_wait_timeout
            ):
                self._fail("motor device was not removed before timeout", now)
                return
            pending_error_at = self.pending_error_at
            pending_error_evidence = self.pending_error_evidence
            if pending_error_at is not None:
                lead = now - pending_error_at
                if not 0.0 <= lead <= self.config.diagnostic_deadline:
                    self._fail(
                        "motor freshness ERROR was not followed by device loss "
                        "within %.3fs" % self.config.diagnostic_deadline,
                        now,
                    )
                    return
            elif not pre_edge_baseline_healthy:
                self._fail(
                    "healthy baseline was lost before the motor device trigger",
                    now,
                )
                return
            self.phase = ProbePhase.LOSS
            self.loss_at = now
            if pending_error_at is not None:
                if pending_error_evidence is None:
                    self._fail("pending motor ERROR evidence was incomplete", now)
                    return
                self._accept_failure_diagnostic(pending_error_evidence)
                self.pending_error_at = None
                self.pending_error_evidence = None
            else:
                self.failure_started_at = now
            return

        if present and self.phase == ProbePhase.LOSS:
            self._fail("motor device reappeared before the live-loss result", now)

    def observe_cmd_message(self, now: float) -> None:
        """Fail immediately if any command message is received."""
        self._fail("unsafe /cmd_vel message observed", now, unsafe=True)

    def observe_controller_topic(
        self, topic: str, now: float, stationary: Optional[bool] = None
    ) -> None:
        """Record one controller-derived message and enforce quiescence."""
        self._check_time(now)
        if self.terminal:
            return
        if topic not in self.topics:
            raise ValueError("unexpected controller topic %s" % topic)
        evidence = self.topics[topic]
        evidence.count += 1
        evidence.last_at = now

        if stationary is False:
            self._fail(
                "unsafe non-stationary feedback observed on %s" % topic,
                now,
                unsafe=True,
            )
            return
        if topic in STATIONARY_TOPICS and stationary is True:
            self.stationary_topics_seen.add(topic)

        if self.failure_started_at is not None and now >= self.failure_started_at:
            evidence.post_loss_count += 1
            evidence.last_post_loss_at = now
            if now - self.failure_started_at > self.config.quiet_deadline:
                self._fail(
                    "%s published after the %.3fs quiet deadline"
                    % (topic, self.config.quiet_deadline),
                    now,
                )

    def observe_motor_diagnostic(
        self,
        level: object,
        message: str,
        values: Mapping[str, str],
        now: float,
    ) -> None:
        """Record the motor status and accept only a freshness-specific loss error."""
        self._check_time(now)
        if self.terminal:
            return
        diagnostic_level = _diagnostic_level_value(level)
        self.latest_diagnostic_level = diagnostic_level
        self.latest_diagnostic_at = now
        self.latest_diagnostic_message = str(message)
        self.latest_diagnostic_values = {
            str(key): str(value) for key, value in values.items()
        }

        baseline_errors = _baseline_freshness_errors(
            self.latest_diagnostic_values,
            self.config.feedback_freshness_bound,
        )
        self.latest_baseline_freshness_errors = baseline_errors
        self.latest_baseline_freshness_valid = not baseline_errors
        if (
            diagnostic_level == 0
            and not baseline_errors
            and self.loss_at is None
        ):
            self.baseline_freshness_evidence = {
                "at": self._relative(now),
                "message": str(message),
                "values": dict(self.latest_diagnostic_values),
                "freshness_bound_seconds": self.config.feedback_freshness_bound,
            }

        failure_errors = (
            _failure_freshness_errors(
                str(message), self.latest_diagnostic_values
            )
            if diagnostic_level == 2
            else []
        )
        qualifying_error = diagnostic_level == 2 and not failure_errors

        if self.error_at is not None:
            if not qualifying_error:
                if failure_errors:
                    self.invalid_error_evidence.append(
                        {
                            "at": self._relative(now),
                            "message": str(message),
                            "values": dict(self.latest_diagnostic_values),
                            "validation_errors": failure_errors,
                        }
                    )
                self._fail(
                    "motor diagnostic contradicted the accepted terminal "
                    "freshness ERROR",
                    now,
                )
            return

        if not qualifying_error:
            if diagnostic_level == 2 and failure_errors:
                if self.loss_at is not None or self.phase == ProbePhase.ARMED:
                    self.invalid_error_evidence.append(
                        {
                            "at": self._relative(now),
                            "message": str(message),
                            "values": dict(self.latest_diagnostic_values),
                            "validation_errors": failure_errors,
                        }
                    )
            if self.loss_at is None and self.pending_error_at is not None:
                self._reset_pending_failure()
            return

        failure_evidence = self._failure_diagnostic_evidence(
            str(message), self.latest_diagnostic_values, now
        )
        if self.loss_at is None:
            if (
                self.phase == ProbePhase.ARMED
                and self.armed_at is not None
                and now >= self.armed_at
                and self.pending_error_at is None
            ):
                self.pending_error_at = now
                self.pending_error_evidence = failure_evidence
                self.failure_started_at = now
            return

        if self.failure_started_at is None:
            self._fail("motor loss deadline anchor was not established", now)
            return
        elapsed = now - self.failure_started_at
        if elapsed > self.config.diagnostic_deadline:
            self._fail(
                "motor freshness ERROR arrived after the %.3fs deadline"
                % self.config.diagnostic_deadline,
                now,
            )
            return
        self._accept_failure_diagnostic(failure_evidence)

    def observe_graph(
        self,
        *,
        driver_endpoints_exact: bool,
        driver_present: bool,
        driver_endpoint_count: int,
        strict_endpoint_count: int,
        strict_topic_counts: Mapping[str, int],
        cmd_publishers: Sequence[str],
        evidence: Optional[Mapping[str, object]],
        now: float,
    ) -> None:
        """Record graph state, including command and strict-shutdown evidence."""
        self._check_time(now)
        if self.terminal:
            return
        self.graph_seen = True
        self.latest_graph_at = now
        self.driver_endpoints_exact = bool(driver_endpoints_exact)
        self.driver_present = bool(driver_present)
        self.driver_endpoint_count = int(driver_endpoint_count)
        normalized_counts = {
            topic: int(strict_topic_counts.get(topic, -1))
            for topic in STRICT_GRAPH_TOPICS
        }
        if any(count < 0 for count in normalized_counts.values()):
            raise ValueError(
                "strict graph evidence must include nonnegative counts for "
                "every required topic"
            )
        observed_total = sum(normalized_counts.values())
        if int(strict_endpoint_count) != observed_total:
            raise ValueError(
                "strict graph endpoint total does not match per-topic evidence"
            )
        self.strict_topic_counts = normalized_counts
        self.strict_endpoint_count = observed_total
        self.latest_graph_evidence = dict(evidence or {})

        if cmd_publishers:
            self._fail(
                "unsafe /cmd_vel publisher endpoint(s) observed: %s"
                % sorted(str(name) for name in cmd_publishers),
                now,
                unsafe=True,
            )
            return

        if self.failure_started_at is None:
            return

        elapsed = now - self.failure_started_at
        driver_gone = not driver_present and driver_endpoint_count == 0
        if self.driver_gone_at is not None and not driver_gone:
            self._fail("driver graph evidence reappeared after exit", now)
            return
        if self.strict_drained_at is not None and observed_total != 0:
            self._fail("strict graph publishers reappeared after teardown", now)
            return
        if driver_gone and self.driver_gone_at is None:
            if self.error_at is None and self.pending_error_at is None:
                self._fail(
                    "driver disappeared before a freshness-specific ERROR was observed",
                    now,
                )
                return
            if elapsed > self.config.driver_exit_deadline:
                self._fail(
                    "driver disappeared after the %.3fs exit deadline"
                    % self.config.driver_exit_deadline,
                    now,
                )
                return
            self.driver_gone_at = now

        if observed_total == 0 and self.strict_drained_at is None:
            if self.strict_zero_since is None:
                self.strict_zero_since = now
            if elapsed > self.config.overall_deadline:
                self._fail(
                    "strict graph drained after the %.3fs overall deadline"
                    % self.config.overall_deadline,
                    now,
                )
                return
            if now - self.strict_zero_since >= self.config.graph_drain_dwell:
                self.strict_drained_at = now
        elif observed_total != 0 and self.strict_drained_at is None:
            self.strict_zero_since = None

    def _baseline_healthy(self, now: float) -> bool:
        if self.device_present is not True:
            return False
        if (
            not self.graph_seen
            or self.latest_graph_at is None
            or now - self.latest_graph_at > self.config.topic_max_age
            or not self.driver_endpoints_exact
        ):
            return False
        if not self.driver_present or self.driver_endpoint_count != len(CONTROLLER_TOPICS):
            return False
        if any(
            self.strict_topic_counts.get(topic, 0) < 1
            for topic in STRICT_GRAPH_TOPICS
        ):
            return False
        if self.latest_diagnostic_level != 0 or self.latest_diagnostic_at is None:
            return False
        if not self.latest_baseline_freshness_valid:
            return False
        if now - self.latest_diagnostic_at > self.config.diagnostic_max_age:
            return False
        if self.stationary_topics_seen != STATIONARY_TOPICS:
            return False
        for evidence in self.topics.values():
            if evidence.count < self.config.baseline_min_messages:
                return False
            if evidence.last_at is None or now - evidence.last_at > self.config.topic_max_age:
                return False
        return True

    def tick(self, now: float) -> None:
        """Advance all time-based gates at ``now``."""
        self._check_time(now)
        if self.terminal:
            return

        if self.phase == ProbePhase.BASELINE:
            if now - self.baseline_started_at > self.config.baseline_timeout:
                self._fail("healthy baseline was not established before timeout", now)
                return
            if self._baseline_healthy(now):
                if self.healthy_since is None:
                    self.healthy_since = now
                if now - self.healthy_since >= self.config.baseline_duration:
                    self.phase = ProbePhase.ARMED
                    self.armed_at = now
                    self.baseline_strict_topic_counts = dict(
                        self.strict_topic_counts
                    )
            else:
                self.healthy_since = None
            return

        if self.phase == ProbePhase.ARMED:
            if self.armed_at is not None and (
                now - self.armed_at > self.config.loss_wait_timeout
            ):
                self._fail("motor device was not removed before timeout", now)
                return
            if self.pending_error_at is not None:
                pending_age = now - self.pending_error_at
                if pending_age <= self.config.diagnostic_deadline:
                    # The serial receive failure can precede removal of the
                    # udev alias.  Allow that bounded ordering window without
                    # requiring an impossible OK diagnostic in between.
                    return
                self._fail(
                    "motor freshness ERROR was not followed by device loss "
                    "within %.3fs" % self.config.diagnostic_deadline,
                    now,
                )
                return
            if not self._baseline_healthy(now):
                self._fail(
                    "healthy baseline was lost before the motor device trigger",
                    now,
                )
                return
            return

        if self.phase != ProbePhase.LOSS or self.loss_at is None:
            return

        if self.failure_started_at is None:
            self._fail("motor loss deadline anchor was not established", now)
            return
        elapsed = now - self.failure_started_at
        if self.error_at is None and elapsed > self.config.diagnostic_deadline:
            self._fail(
                "no freshness-specific motor ERROR within %.3fs"
                % self.config.diagnostic_deadline,
                now,
            )
            return
        if self.driver_gone_at is None and elapsed > self.config.driver_exit_deadline:
            self._fail(
                "driver remained present beyond the %.3fs exit deadline"
                % self.config.driver_exit_deadline,
                now,
            )
            return
        if self.strict_drained_at is None and elapsed > self.config.overall_deadline:
            self._fail(
                "strict graph remained active beyond the %.3fs overall deadline"
                % self.config.overall_deadline,
                now,
            )
            return

        quiet_window_observed = elapsed >= self.config.quiet_deadline
        if (
            quiet_window_observed
            and self.error_at is not None
            and self.accepted_error_evidence is not None
            and self.driver_gone_at is not None
            and self.strict_drained_at is not None
        ):
            self.phase = ProbePhase.PASSED
            self.outcome = "PASS"
            self.completed_at = now

    def report(self) -> Dict[str, object]:
        """Return bounded, JSON-serializable result and evidence."""
        return {
            "schema_version": 1,
            "probe": "motor_live_loss_no_motion",
            "scope": (
                "ROS data, diagnostics, and process fail-closed behavior only; "
                "not active-motion physical stop behavior"
            ),
            "outcome": self.outcome,
            "phase": self.phase.value,
            "unsafe": self.unsafe,
            "reasons": list(self.reasons),
            "exit_code": (
                0
                if self.phase == ProbePhase.PASSED
                else self.failure_exit_code
                if self.failure_exit_code is not None
                else 2
            ),
            "config": asdict(self.config),
            "timing_offsets_seconds": {
                "baseline_started": self._relative(self.baseline_started_at),
                "armed": self._relative(self.armed_at),
                "failure_started": self._relative(self.failure_started_at),
                "device_loss": self._relative(self.loss_at),
                "freshness_error": self._relative(self.error_at),
                "driver_gone": self._relative(self.driver_gone_at),
                "strict_graph_zero_started": self._relative(
                    self.strict_zero_since
                ),
                "strict_graph_drained": self._relative(self.strict_drained_at),
                "completed": self._relative(self.completed_at),
            },
            "device": {
                "was_present": self.device_was_present,
                "present_at_end": self.device_present,
            },
            "latest_motor_diagnostic": {
                "level": self.latest_diagnostic_level,
                "at": self._relative(self.latest_diagnostic_at),
                "message": self.latest_diagnostic_message,
                "values": dict(self.latest_diagnostic_values),
                "baseline_freshness_valid": self.latest_baseline_freshness_valid,
                "baseline_freshness_errors": list(
                    self.latest_baseline_freshness_errors
                ),
            },
            "accepted_failure_diagnostic": (
                {
                    "level": self.accepted_error_evidence["level"],
                    "at": self._relative(
                        float(self.accepted_error_evidence["at"])
                    ),
                    "message": self.accepted_error_evidence["message"],
                    "values": dict(self.accepted_error_evidence["values"]),
                }
                if self.accepted_error_evidence is not None
                else None
            ),
            "invalid_error_evidence": list(self.invalid_error_evidence),
            "baseline_freshness_evidence": (
                dict(self.baseline_freshness_evidence)
                if self.baseline_freshness_evidence is not None
                else None
            ),
            "topics": {
                topic: {
                    "count": evidence.count,
                    "last_at": self._relative(evidence.last_at),
                    "post_loss_count": evidence.post_loss_count,
                    "last_post_loss_at": self._relative(
                        evidence.last_post_loss_at
                    ),
                    "post_failure_count": evidence.post_loss_count,
                    "last_post_failure_at": self._relative(
                        evidence.last_post_loss_at
                    ),
                }
                for topic, evidence in self.topics.items()
            },
            "graph": {
                "driver_endpoints_exact": self.driver_endpoints_exact,
                "driver_present": self.driver_present,
                "driver_endpoint_count": self.driver_endpoint_count,
                "strict_endpoint_count": self.strict_endpoint_count,
                "latest_at": self._relative(self.latest_graph_at),
                "strict_topic_counts": dict(self.strict_topic_counts),
                "baseline_strict_topic_counts": (
                    dict(self.baseline_strict_topic_counts)
                    if self.baseline_strict_topic_counts is not None
                    else None
                ),
                "latest_evidence": dict(self.latest_graph_evidence),
            },
        }


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def _finite_nonnegative(value: object) -> bool:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(parsed) and parsed >= 0.0


def _baseline_freshness_errors(
    values: Mapping[str, str], freshness_bound: float
) -> list:
    """Return why an OK diagnostic lacks complete healthy receive evidence."""
    errors = []
    if str(values.get("feedback_state", "")).lower() != "healthy":
        errors.append("feedback_state must be healthy")
    for stream in FEEDBACK_STREAMS:
        age_key = "feedback_%s_age_seconds" % stream
        stale_key = "feedback_%s_stale" % stream
        age_value = values.get(age_key)
        if not _finite_nonnegative(age_value):
            errors.append("%s must be finite and nonnegative" % age_key)
        elif float(age_value) > freshness_bound:
            errors.append(
                "%s exceeds %.6fs" % (age_key, freshness_bound)
            )
        if str(values.get(stale_key, "")).lower() != "false":
            errors.append("%s must be false" % stale_key)
    return errors


def _failure_reason_identifies_live_loss(
    message: str, values: Mapping[str, str]
) -> bool:
    text = "%s %s" % (message, values.get("feedback_reason", ""))
    text = text.lower()
    stale_reports = "stale" in text and any(
        token in text for token in ("feedback", "report", "stream")
    )
    receiver_failure = any(
        token in text for token in ("receive thread", "receiver", "serial read")
    ) and any(
        token in text
        for token in ("fail", "raised", "exit", "stopp", "error", "exception")
    )
    return stale_reports or receiver_failure


def _failure_freshness_errors(
    message: str, values: Mapping[str, str]
) -> list:
    """Return why an ERROR is not complete live receive-loss evidence."""
    errors = []
    if str(values.get("feedback_state", "")).lower() != "failed":
        errors.append("feedback_state must be failed")
    for stream in FEEDBACK_STREAMS:
        age_key = "feedback_%s_age_seconds" % stream
        stale_key = "feedback_%s_stale" % stream
        if not _finite_nonnegative(values.get(age_key)):
            errors.append("%s must be finite and nonnegative" % age_key)
        if str(values.get(stale_key, "")).lower() not in ("true", "false"):
            errors.append("%s must be present and boolean" % stale_key)
    if not _failure_reason_identifies_live_loss(message, values):
        errors.append("failure reason must identify stale reports or receiver failure")
    return errors


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse physical probe arguments without initializing ROS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/robot/motor")
    parser.add_argument("--driver-node", default="/driver_node")
    parser.add_argument("--baseline-duration", type=_positive_float, default=2.0)
    parser.add_argument("--baseline-timeout", type=_positive_float, default=30.0)
    parser.add_argument("--baseline-min-messages", type=_positive_int, default=3)
    parser.add_argument("--topic-max-age", type=_positive_float, default=0.5)
    parser.add_argument("--diagnostic-max-age", type=_positive_float, default=1.5)
    parser.add_argument(
        "--feedback-freshness-bound", type=_positive_float, default=0.5
    )
    parser.add_argument("--loss-wait-timeout", type=_positive_float, default=60.0)
    parser.add_argument("--quiet-deadline", type=_positive_float, default=0.75)
    parser.add_argument("--diagnostic-deadline", type=_positive_float, default=0.75)
    parser.add_argument("--driver-exit-deadline", type=_positive_float, default=1.5)
    parser.add_argument(
        "--graph-drain-dwell", type=_positive_float, default=0.25
    )
    parser.add_argument("--overall-deadline", type=_positive_float, default=8.0)
    parser.add_argument("--max-wheel-speed", type=_positive_float, default=0.05)
    parser.add_argument("--max-position-drift", type=_positive_float, default=0.02)
    parser.add_argument("--max-chassis-speed", type=_positive_float, default=0.002)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--confirm-wheels-secured",
        action="store_true",
        help="Required acknowledgement that all wheels are safely restrained",
    )
    args = parser.parse_args(argv)
    if not args.confirm_wheels_secured:
        parser.error("--confirm-wheels-secured is required")
    config = ProbeConfig(
        baseline_duration=args.baseline_duration,
        baseline_timeout=args.baseline_timeout,
        baseline_min_messages=args.baseline_min_messages,
        topic_max_age=args.topic_max_age,
        diagnostic_max_age=args.diagnostic_max_age,
        feedback_freshness_bound=args.feedback_freshness_bound,
        loss_wait_timeout=args.loss_wait_timeout,
        quiet_deadline=args.quiet_deadline,
        diagnostic_deadline=args.diagnostic_deadline,
        driver_exit_deadline=args.driver_exit_deadline,
        graph_drain_dwell=args.graph_drain_dwell,
        overall_deadline=args.overall_deadline,
    )
    try:
        config.validate()
    except ValueError as error:
        parser.error(str(error))
    args.config = config
    return args


def _endpoint_name(endpoint) -> str:
    namespace = str(getattr(endpoint, "node_namespace", "/") or "/")
    name = str(getattr(endpoint, "node_name", ""))
    return "/" + "/".join(
        part for part in (namespace.strip("/"), name.strip("/")) if part
    )


def _graph_snapshot(node, driver_node: str) -> Dict[str, object]:
    publishers = {}
    for topic in sorted(set(STRICT_GRAPH_TOPICS) | {"/cmd_vel"}):
        publishers[topic] = sorted(
            _endpoint_name(endpoint)
            for endpoint in node.get_publishers_info_by_topic(topic)
        )
    driver_publishers = {
        topic: names for topic, names in publishers.items() if topic in CONTROLLER_TOPICS
    }
    driver_endpoints_exact = all(
        names == [driver_node] for names in driver_publishers.values()
    )
    driver_endpoint_count = sum(
        name == driver_node
        for names in driver_publishers.values()
        for name in names
    )
    node_names = sorted(
        "/" + "/".join(part for part in (namespace.strip("/"), name) if part)
        for name, namespace in node.get_node_names_and_namespaces()
    )
    strict_endpoint_count = sum(
        len(publishers[topic]) for topic in STRICT_GRAPH_TOPICS
    )
    strict_topic_counts = {
        topic: len(publishers[topic]) for topic in STRICT_GRAPH_TOPICS
    }
    return {
        "driver_endpoints_exact": driver_endpoints_exact,
        "driver_present": driver_node in node_names,
        "driver_endpoint_count": driver_endpoint_count,
        "strict_endpoint_count": strict_endpoint_count,
        "strict_topic_counts": strict_topic_counts,
        "cmd_publishers": publishers["/cmd_vel"],
        "evidence": {
            "publishers": publishers,
            "nodes": node_names,
        },
    }


def _run_ros(args: argparse.Namespace) -> Tuple[int, Dict[str, object]]:
    """Run the observer-only ROS adapter and return its report."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState, MagneticField
    from std_msgs.msg import Float32

    started_at = time.monotonic()
    evaluator = LiveLossEvaluator(args.config, started_at=started_at)
    wall_started = datetime.now(timezone.utc).isoformat()
    rclpy.init(args=[])
    node = Node(
        "motor_live_loss_probe",
        enable_rosout=False,
        start_parameter_services=False,
    )
    subscriptions = []
    joint_extrema: Dict[str, list] = {}
    wheel_names = (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "back_left_wheel_joint",
        "back_right_wheel_joint",
    )

    def observe_simple(topic: str):
        def callback(_message) -> None:
            evaluator.observe_controller_topic(topic, time.monotonic())

        return callback

    def observe_velocity(message: Twist) -> None:
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        stationary = all(
            math.isfinite(value) and abs(value) <= args.max_chassis_speed
            for value in values
        )
        evaluator.observe_controller_topic(
            "/vel_raw", time.monotonic(), stationary=stationary
        )

    def observe_joints(message: JointState) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        stationary = all(name in indices for name in wheel_names)
        for name in wheel_names:
            if not stationary:
                break
            index = indices[name]
            if index >= len(message.position) or index >= len(message.velocity):
                stationary = False
                break
            position = float(message.position[index])
            velocity = float(message.velocity[index])
            if not math.isfinite(position) or not math.isfinite(velocity):
                stationary = False
                break
            extrema = joint_extrema.setdefault(name, [position, position])
            extrema[0] = min(extrema[0], position)
            extrema[1] = max(extrema[1], position)
            if (
                abs(velocity) > args.max_wheel_speed
                or extrema[1] - extrema[0] > args.max_position_drift
            ):
                stationary = False
                break
        evaluator.observe_controller_topic(
            "/joint_states", time.monotonic(), stationary=stationary
        )

    def observe_diagnostics(message: DiagnosticArray) -> None:
        now = time.monotonic()
        for status in message.status:
            if not _is_expected_motor_diagnostic(status, args.device):
                continue
            values = {item.key: item.value for item in status.values}
            evaluator.observe_motor_diagnostic(
                status.level,
                status.message,
                values,
                now,
            )

    def observe_cmd(_message: Twist) -> None:
        evaluator.observe_cmd_message(time.monotonic())

    subscriptions.extend(
        [
            node.create_subscription(
                JointState,
                "/joint_states",
                observe_joints,
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Twist, "/vel_raw", observe_velocity, qos_profile_sensor_data
            ),
            node.create_subscription(
                Float32,
                "/voltage",
                observe_simple("/voltage"),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Float32,
                "/edition",
                observe_simple("/edition"),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Imu,
                "/imu/data_raw",
                observe_simple("/imu/data_raw"),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                MagneticField,
                "/imu/mag",
                observe_simple("/imu/mag"),
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                observe_diagnostics,
                qos_profile_sensor_data,
            ),
            node.create_subscription(
                Twist, "/cmd_vel", observe_cmd, qos_profile_sensor_data
            ),
        ]
    )

    previous_phase = evaluator.phase
    interrupted = False
    try:
        while rclpy.ok() and not evaluator.terminal:
            rclpy.spin_once(node, timeout_sec=0.05)
            device_now = time.monotonic()
            evaluator.observe_device(os.path.exists(args.device), device_now)
            if evaluator.terminal:
                break
            snapshot = _graph_snapshot(node, args.driver_node)
            graph_now = time.monotonic()
            evaluator.observe_graph(now=graph_now, **snapshot)
            evaluator.tick(graph_now)
            if evaluator.phase != previous_phase:
                if evaluator.phase == ProbePhase.ARMED:
                    print(
                        "ARMED: disconnect only %s; do not reconnect yet"
                        % args.device,
                        file=sys.stderr,
                    )
                elif evaluator.phase == ProbePhase.LOSS:
                    print("Motor device loss observed", file=sys.stderr)
                previous_phase = evaluator.phase
    except KeyboardInterrupt:
        interrupted = True
        evaluator.abort("probe interrupted", time.monotonic())
    except Exception as error:
        evaluator.abort(
            "observer exception: %s: %s" % (type(error).__name__, error),
            time.monotonic(),
        )
    finally:
        subscriptions.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    report = evaluator.report()
    report.update(
        {
            "wall_started_utc": wall_started,
            "device_path": args.device,
            "expected_driver_node": args.driver_node,
            "observer_only": True,
        }
    )
    if interrupted:
        report["exit_code"] = 130
        return 130, report
    return int(report["exit_code"]), report


def _emit_report(report: Mapping[str, object], output: Optional[Path]) -> None:
    payload = json.dumps(report, sort_keys=True, separators=(",", ":"))
    print(payload)
    if output is not None:
        output.write_text(payload + "\n", encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the ROS observer and always emit a machine-readable result."""
    args = parse_args(argv)
    try:
        exit_code, report = _run_ros(args)
    except ImportError as error:
        exit_code = 2
        report = {
            "schema_version": 1,
            "probe": "motor_live_loss_no_motion",
            "outcome": "FAIL",
            "phase": ProbePhase.FAILED.value,
            "unsafe": False,
            "reasons": ["ROS dependency unavailable: %s" % error],
            "exit_code": exit_code,
            "observer_only": True,
        }
    except KeyboardInterrupt:
        exit_code = 130
        report = {
            "schema_version": 1,
            "probe": "motor_live_loss_no_motion",
            "outcome": "FAIL",
            "phase": ProbePhase.FAILED.value,
            "unsafe": False,
            "reasons": ["probe interrupted during ROS setup"],
            "exit_code": exit_code,
            "observer_only": True,
        }
    except Exception as error:
        exit_code = 2
        report = {
            "schema_version": 1,
            "probe": "motor_live_loss_no_motion",
            "outcome": "FAIL",
            "phase": ProbePhase.FAILED.value,
            "unsafe": False,
            "reasons": [
                "observer setup failed: %s: %s" % (type(error).__name__, error)
            ],
            "exit_code": exit_code,
            "observer_only": True,
        }
    _emit_report(report, args.output)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
