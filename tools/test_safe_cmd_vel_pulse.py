"""Focused tests for bounded-pulse ROS graph gating and the stop path."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

import safe_cmd_vel_pulse
from test_motor_live_loss_ros_smoke import isolated_ros_domain


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


FAKE_DRIVER = r"""
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("driver_node")
moving = []
zeros_after_motion = []
report_requested = threading.Event()


def on_cmd(msg):
    nonzero = msg.linear.x or msg.linear.y or msg.angular.z
    if nonzero:
        if not moving:
            print("MOVING", flush=True)
        moving.append(msg)
        zeros_after_motion.clear()
    elif moving:
        zeros_after_motion.append(msg)


def wait_for_report():
    for line in sys.stdin:
        if line.strip() == "report":
            break
    report_requested.set()


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
threading.Thread(target=wait_for_report, daemon=True).start()
deadline = time.monotonic() + 40.0
while not report_requested.is_set() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.02)
for _ in range(25):
    rclpy.spin_once(node, timeout_sec=0.02)
print("ZEROS %d" % len(zeros_after_motion), flush=True)
"""


def ros_env(domain):
    """Isolate the test graph on localhost and a reserved domain."""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["PYTHONUNBUFFERED"] = "1"
    return env


def read_line_until(stream, prefix, deadline):
    """Return the first output line starting with prefix before deadline."""
    while time.monotonic() < deadline:
        line = stream.readline()
        if not line:
            break
        if line.startswith(prefix):
            return line.strip()
    raise AssertionError("did not see %r from the fake driver" % prefix)


@pytest.mark.parametrize(
    "stop_signal", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"]
)
def test_interrupted_pulse_still_publishes_zero(stop_signal):
    """Ctrl-C or SIGTERM mid-pulse must still reach the driver with zeros."""
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("ROS 2 Python runtime unavailable: rclpy")
    with isolated_ros_domain() as domain:
        run_interrupted_pulse(ros_env(domain), [stop_signal])


@pytest.mark.parametrize(
    "stop_signals",
    [
        [signal.SIGINT, signal.SIGINT],
        [signal.SIGTERM, signal.SIGINT],
    ],
    ids=["INT-then-INT", "TERM-then-INT"],
)
def test_second_stop_signal_does_not_cut_the_zeros_short(stop_signals):
    """A second Ctrl-C while the zeros go out must not skip any of them."""
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("ROS 2 Python runtime unavailable: rclpy")
    with isolated_ros_domain() as domain:
        run_interrupted_pulse(ros_env(domain), stop_signals, min_zeros=5)


# Lands the second signal inside the finally block's 5 zeros (20 ms apart).
SECOND_SIGNAL_DELAY = 0.02


def run_interrupted_pulse(env, stop_signals, min_zeros=1):
    """Interrupt a running pulse and require zeros at the fake driver."""
    driver = subprocess.Popen(
        [sys.executable, "-c", FAKE_DRIVER],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        env=env,
    )
    pulse = None
    try:
        pulse = subprocess.Popen(
            [
                sys.executable,
                str(Path(safe_cmd_vel_pulse.__file__)),
                "--x", "0.05",
                "--duration", "10",
                "--discovery-timeout", "10",
            ],
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )
        read_line_until(driver.stdout, "MOVING", time.monotonic() + 20)
        for index, stop_signal in enumerate(stop_signals):
            if index:
                time.sleep(SECOND_SIGNAL_DELAY)
            pulse.send_signal(stop_signal)
        _, stderr = pulse.communicate(timeout=10)
        assert pulse.returncode == 2, stderr
        assert "context is invalid" not in stderr, stderr
        assert "Traceback" not in stderr, stderr

        driver.stdin.write("report\n")
        driver.stdin.flush()
        report = read_line_until(driver.stdout, "ZEROS", time.monotonic() + 10)
        assert int(report.split()[1]) >= min_zeros, stderr
    finally:
        for process in (pulse, driver):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
