"""Focused tests for lifted-pulse ROS graph gating."""

from types import SimpleNamespace

import pytest

import safe_cmd_vel_pulse


class FakeNode:
    """Supply the graph methods used by validate_graph."""

    def __init__(self, subscribers, publishers=()):
        """Store fake graph endpoints."""
        self.subscribers = subscribers
        self.publishers = publishers

    def get_subscriptions_info_by_topic(self, _topic):
        """Return configured subscription endpoints."""
        return self.subscribers

    def get_publishers_info_by_topic(self, _topic):
        """Return configured publisher endpoints."""
        return self.publishers

    def get_name(self):
        """Return the safety tool's node name."""
        return "safe_cmd_vel_pulse"


class FakePublisher:
    """Expose the compatible DDS subscription count."""

    def __init__(self, subscription_count):
        """Store a fake compatible match count."""
        self.subscription_count = subscription_count

    def get_subscription_count(self):
        """Return the configured compatible match count."""
        return self.subscription_count


def endpoint(name):
    """Create the endpoint shape consumed by validate_graph."""
    return SimpleNamespace(node_name=name)


def test_required_recorder_must_be_qos_compatible(monkeypatch):
    """A visible but incompatible recorder must not permit motion."""
    monkeypatch.setattr(
        safe_cmd_vel_pulse.rclpy, "spin_once", lambda *a, **k: None
    )
    node = FakeNode([endpoint("driver_node"), endpoint("rosbag2_recorder")])

    with pytest.raises(RuntimeError, match="compatible_subscription_count=1"):
        safe_cmd_vel_pulse.validate_graph(
            node,
            FakePublisher(1),
            timeout=0.001,
            require_recorder=True,
        )


def test_required_recorder_accepts_two_compatible_matches(monkeypatch):
    """The driver plus a compatible recorder satisfy the gated graph."""
    monkeypatch.setattr(
        safe_cmd_vel_pulse.rclpy, "spin_once", lambda *a, **k: None
    )
    node = FakeNode([endpoint("driver_node"), endpoint("rosbag2_recorder")])

    safe_cmd_vel_pulse.validate_graph(
        node,
        FakePublisher(2),
        timeout=0.001,
        require_recorder=True,
    )
