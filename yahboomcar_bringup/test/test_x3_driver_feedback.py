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

"""Hardware-free feedback-state tests for the X3 ROS driver."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from std_msgs.msg import Int32

from yahboomcar_bringup.Mcnamu_driver_X3 import cleanup_driver
from yahboomcar_bringup.Mcnamu_driver_X3 import YahboomCarDriver
from yahboomcar_bringup.rosmaster_transport import TransportState
from yahboomcar_bringup.rosmaster_transport import TransportStatus


_DATA_TOPICS = (
    "edition",
    "voltage",
    "joint_states",
    "vel_raw",
    "/imu/data_raw",
    "/imu/mag",
)
_REPORT_STREAMS = ("speed", "encoder", "imu_raw")


class ManualClock:
    """Deterministic monotonic and ROS clock used by the driver test."""

    def __init__(self, seconds=10.0):
        """Start the clock at a finite positive time."""
        self.seconds = float(seconds)

    def __call__(self):
        """Return the current monotonic time."""
        return self.seconds

    def now(self):
        """Return the same instant as an rclpy time value."""
        return Time(nanoseconds=int(self.seconds * 1e9))

    def advance(self, seconds):
        """Advance the deterministic clock."""
        self.seconds += float(seconds)


class RecordingPublisher:
    """Record published ROS messages without creating DDS entities."""

    def __init__(self):
        """Create an empty message record."""
        self.messages = []

    def publish(self, message):
        """Retain an independent copy of one published message."""
        self.messages.append(deepcopy(message))


class FakeLogger:
    """Accept driver log calls without configuring ROS logging."""

    def debug(self, *args, **kwargs):
        """Discard a debug record."""

    def info(self, *args, **kwargs):
        """Discard an informational record."""

    def warning(self, *args, **kwargs):
        """Discard a warning record."""

    warn = warning

    def error(self, *args, **kwargs):
        """Discard an error record."""


class FakeParameter:
    """Expose the rclpy parameter value interface used by the driver."""

    def __init__(self, value):
        """Store a parameter value."""
        self.value = value


class FakeVendorController:
    """Return valid cached values while recording every actuator request."""

    def __init__(self):
        """Create deterministic telemetry and empty call records."""
        self.motion_commands = []
        self.light_commands = []
        self.buzzer_commands = []
        self.car_types = []
        self.getter_calls = {
            "encoder": 0,
            "version": 0,
            "battery": 0,
            "accelerometer": 0,
            "gyroscope": 0,
            "magnetometer": 0,
            "speed": 0,
        }

    def set_car_type(self, car_type):
        """Record the selected firmware car type."""
        self.car_types.append(car_type)

    def set_car_motion(self, vx, vy, wz):
        """Record a chassis command."""
        self.motion_commands.append((vx, vy, wz))

    def set_colorful_effect(self, effect, duration, parm=1):
        """Record an RGB effect request."""
        self.light_commands.append((effect, duration, parm))

    def set_beep(self, state):
        """Record a buzzer request."""
        self.buzzer_commands.append(state)

    def get_motor_encoder(self):
        """Return one valid cached encoder sample."""
        self.getter_calls["encoder"] += 1
        return (10, 20, 30, 40)

    def get_version(self):
        """Return one valid cached firmware version."""
        self.getter_calls["version"] += 1
        return 3.3

    def get_battery_voltage(self):
        """Return one valid cached battery sample."""
        self.getter_calls["battery"] += 1
        return 11.4

    def get_accelerometer_data(self):
        """Return one valid cached acceleration sample."""
        self.getter_calls["accelerometer"] += 1
        return (0.1, 0.2, 9.7)

    def get_gyroscope_data(self):
        """Return one valid cached angular-rate sample."""
        self.getter_calls["gyroscope"] += 1
        return (0.01, 0.02, 0.03)

    def get_magnetometer_data(self):
        """Return one valid cached magnetic-field sample."""
        self.getter_calls["magnetometer"] += 1
        return (1.0, 2.0, 3.0)

    def get_motion_data(self):
        """Return one valid cached firmware-speed sample."""
        self.getter_calls["speed"] += 1
        return (0.0, 0.0, 0.0)


class FakeTransport:
    """Expose the monitored-controller contract without a serial device."""

    def __init__(self, vendor, status):
        """Wrap a fake vendor at an injected feedback state."""
        self.vendor = vendor
        self.current_status = status
        self.start_count = 0
        self.close_count = 0
        self.latched_reasons = []
        self.serial_write_failure_count = 0
        self.latest_serial_write_failure = None

    def start(self):
        """Record receive-path startup."""
        self.start_count += 1

    def status(self):
        """Return the current immutable feedback status."""
        return self.current_status

    def require_healthy(self):
        """Return the vendor only while all required reports are fresh."""
        if self.current_status.state is not TransportState.HEALTHY:
            raise RuntimeError(self.current_status.reason or "feedback not healthy")
        return self.vendor

    def read_cached(self, reader):
        """Run one coherent cached-value reader while feedback is healthy."""
        self.require_healthy()
        return reader(self.vendor)

    def perform_while_healthy(self, _label, action):
        """Run an action between two deterministic health checks."""
        self.require_healthy()
        result = action(self.vendor)
        self.require_healthy()
        return result

    def latch_failure(self, reason):
        """Record an explicit terminal failure request."""
        self.latched_reasons.append(reason)
        self.current_status = _status(
            TransportState.FAILED,
            reason=reason,
            ages=self.current_status.report_ages,
            sequence=self.current_status.report_sequence,
        )

    def close(self):
        """Record transport cleanup."""
        self.close_count += 1


def _status(state, reason="", ages=None, sequence=0):
    """Build one immutable transport status."""
    if ages is None:
        ages = {stream: None for stream in _REPORT_STREAMS}
    return TransportStatus(
        state=state,
        reason=reason,
        report_ages=ages,
        report_sequence=sequence,
    )


def _install_node_harness(
    monkeypatch, clock, ros_clock=None, parameter_overrides=None
):
    """Replace Node side effects with in-memory parameters and publishers."""
    parameters = {
        "serial_port": "/dev/test-motor",
        "feedback_startup_timeout": 0.4,
        "feedback_timeout": 0.5,
        "feedback_failure_exit_delay": 0.25,
        "serial_write_timeout": 0.05,
    }
    parameters.update(parameter_overrides or {})
    publishers = {}

    def declare_parameter(node, name, default_value, *args, **kwargs):
        parameters.setdefault(name, default_value)
        return FakeParameter(parameters[name])

    def get_parameter(node, name):
        return FakeParameter(parameters[name])

    def create_publisher(node, message_type, topic, qos, *args, **kwargs):
        publisher = RecordingPublisher()
        publishers[topic] = publisher
        return publisher

    monkeypatch.setattr(Node, "__init__", lambda node, name, *args, **kwargs: None)
    monkeypatch.setattr(Node, "declare_parameter", declare_parameter)
    monkeypatch.setattr(Node, "get_parameter", get_parameter)
    monkeypatch.setattr(Node, "create_publisher", create_publisher)
    monkeypatch.setattr(
        Node,
        "create_subscription",
        lambda node, *args, **kwargs: object(),
    )
    monkeypatch.setattr(
        Node,
        "create_timer",
        lambda node, *args, **kwargs: object(),
    )
    monkeypatch.setattr(
        Node, "get_clock", lambda node: ros_clock or clock
    )
    monkeypatch.setattr(Node, "get_logger", lambda node: FakeLogger())
    monkeypatch.setattr(Node, "destroy_node", lambda node: True)
    return parameters, publishers


def _exercise_nonzero_actuators(driver):
    """Offer nonzero motion, light, and buzzer commands to the driver."""
    command = Twist()
    command.linear.x = 0.2
    command.linear.y = -0.1
    command.angular.z = 0.3
    driver.cmd_vel_callback(command)
    driver.rgb_light_callback(Int32(data=2))
    driver.buzzer_callback(Bool(data=True))


def _topic_counts(publishers):
    """Return controller-derived output counts by topic."""
    return {topic: len(publishers[topic].messages) for topic in _DATA_TOPICS}


def _diagnostic_values(publishers):
    """Return the latest controller diagnostic status and values."""
    message = publishers["/diagnostics"].messages[-1]
    assert len(message.status) == 1
    status = message.status[0]
    return status, {item.key: item.value for item in status.values}


def test_driver_gates_cached_feedback_and_actuators(monkeypatch):
    """Only fresh reports authorize output; terminal loss exits after grace."""
    clock = ManualClock()
    parameters, publishers = _install_node_harness(monkeypatch, clock)
    vendor = FakeVendorController()
    transport = FakeTransport(vendor, _status(TransportState.WAITING))
    factory_arguments = {}

    def controller_factory(**kwargs):
        factory_arguments.update(kwargs)
        return transport

    driver = YahboomCarDriver(
        "driver_node",
        controller_factory=controller_factory,
        monotonic_clock=clock,
    )

    assert transport.start_count == 1
    assert factory_arguments["com"] == parameters["serial_port"]
    assert factory_arguments["clock"] is clock
    assert factory_arguments["startup_timeout"] == pytest.approx(0.4)
    assert factory_arguments["stale_timeout"] == pytest.approx(0.5)
    assert factory_arguments["write_timeout"] == pytest.approx(0.05)

    driver.publish_data()
    assert _topic_counts(publishers) == {topic: 0 for topic in _DATA_TOPICS}
    starting_motion_count = len(vendor.motion_commands)
    _exercise_nonzero_actuators(driver)
    assert all(
        command == (0.0, 0.0, 0.0)
        for command in vendor.motion_commands[starting_motion_count:]
    )
    assert vendor.light_commands == []
    assert vendor.buzzer_commands == []

    transport.current_status = _status(
        TransportState.HEALTHY,
        ages={"speed": 0.01, "encoder": 0.02, "imu_raw": 0.03},
        sequence=7,
    )
    driver.publish_data()
    assert _topic_counts(publishers) == {topic: 1 for topic in _DATA_TOPICS}
    assert vendor.getter_calls == {
        "encoder": 1,
        "version": 1,
        "battery": 1,
        "accelerometer": 1,
        "gyroscope": 1,
        "magnetometer": 1,
        "speed": 1,
    }
    stamped_messages = (
        publishers["joint_states"].messages[-1],
        publishers["/imu/data_raw"].messages[-1],
        publishers["/imu/mag"].messages[-1],
    )
    stamps = {
        (message.header.stamp.sec, message.header.stamp.nanosec)
        for message in stamped_messages
    }
    assert len(stamps) == 1

    failure_reason = "controller feedback stale after receive thread stopped"
    transport.current_status = _status(
        TransportState.FAILED,
        reason=failure_reason,
        ages={"speed": 0.8, "encoder": 0.9, "imu_raw": 0.7},
        sequence=7,
    )
    healthy_counts = _topic_counts(publishers)
    healthy_getter_calls = dict(vendor.getter_calls)
    failure_motion_count = len(vendor.motion_commands)

    driver.publish_data()
    driver.publish_health()
    assert _topic_counts(publishers) == healthy_counts
    assert vendor.getter_calls == healthy_getter_calls
    assert len(vendor.motion_commands) >= failure_motion_count + 3
    assert all(
        command == (0.0, 0.0, 0.0)
        for command in vendor.motion_commands[failure_motion_count:]
    )

    diagnostic, values = _diagnostic_values(publishers)
    assert diagnostic.level == DiagnosticStatus.ERROR
    assert values["feedback_state"] == TransportState.FAILED.value
    assert values["feedback_reason"] == failure_reason
    assert values["feedback_report_sequence"] == "7"
    for stream, age in transport.current_status.report_ages.items():
        assert float(values["feedback_%s_age_seconds" % stream]) == pytest.approx(age)
        assert values["feedback_%s_stale" % stream].lower() == "true"

    failed_light_count = len(vendor.light_commands)
    failed_buzzer_count = len(vendor.buzzer_commands)
    failed_motion_count = len(vendor.motion_commands)
    _exercise_nonzero_actuators(driver)
    assert all(
        command == (0.0, 0.0, 0.0)
        for command in vendor.motion_commands[failed_motion_count:]
    )
    assert len(vendor.light_commands) == failed_light_count
    assert len(vendor.buzzer_commands) == failed_buzzer_count
    assert _topic_counts(publishers) == healthy_counts
    assert vendor.getter_calls == healthy_getter_calls

    clock.advance(0.24)
    driver.publish_data()
    assert _topic_counts(publishers) == healthy_counts
    clock.advance(0.02)
    with pytest.raises(RuntimeError, match="controller feedback stale"):
        driver.publish_data()


def test_cleanup_stop_failure_does_not_escape():
    """A failed final zero attempt does not prevent transport and node cleanup."""
    events = []

    class FailingStopDriver:
        """Provide only the cleanup surface with a failing zero command."""

        def __init__(self):
            self.transport = self
            self.logger = FakeLogger()

        def stop_motion(self, repeat=1):
            events.append(("stop", repeat))
            raise OSError("serial device disappeared")

        def close(self):
            events.append(("close", None))

        def destroy_node(self):
            events.append(("destroy", None))

        def get_logger(self):
            return self.logger

    cleanup_driver(FailingStopDriver())
    assert events == [("stop", 3), ("close", None), ("destroy", None)]


def test_constructor_failure_stops_and_closes_started_transport(monkeypatch):
    """A post-start initialization error cannot leak serial or receiver state."""
    clock = ManualClock()
    _install_node_harness(monkeypatch, clock)
    monkeypatch.setattr(
        Node,
        "create_timer",
        lambda node, *args, **kwargs: (_ for _ in ()).throw(
            RuntimeError("timer creation failed")
        ),
    )
    vendor = FakeVendorController()
    transport = FakeTransport(vendor, _status(TransportState.WAITING))

    with pytest.raises(RuntimeError, match="timer creation failed"):
        YahboomCarDriver(
            "driver_node",
            controller_factory=lambda **_kwargs: transport,
            monotonic_clock=clock,
        )

    assert transport.start_count == 1
    assert transport.close_count == 1
    assert vendor.motion_commands == [(0.0, 0.0, 0.0)] * 6


def test_cached_getter_failure_is_terminal_and_publishes_no_data(monkeypatch):
    """A successful status check cannot excuse a cache-reader exception."""
    clock = ManualClock()
    _, publishers = _install_node_harness(monkeypatch, clock)
    vendor = FakeVendorController()
    transport = FakeTransport(
        vendor,
        _status(
            TransportState.HEALTHY,
            ages={"speed": 0.01, "encoder": 0.02, "imu_raw": 0.03},
            sequence=4,
        ),
    )
    driver = YahboomCarDriver(
        "driver_node",
        controller_factory=lambda **_kwargs: transport,
        monotonic_clock=clock,
    )

    def failed_battery_read():
        raise OSError("cached battery unavailable")

    vendor.get_battery_voltage = failed_battery_read
    driver.publish_data()

    assert _topic_counts(publishers) == {topic: 0 for topic in _DATA_TOPICS}
    assert transport.current_status.state is TransportState.FAILED
    assert "cached battery unavailable" in transport.current_status.reason
    diagnostic, values = _diagnostic_values(publishers)
    assert diagnostic.level == DiagnosticStatus.ERROR
    assert values["feedback_state"] == "failed"
    assert vendor.motion_commands[-3:] == [(0.0, 0.0, 0.0)] * 3

    clock.advance(0.26)
    with pytest.raises(RuntimeError, match="cached battery unavailable"):
        driver.publish_data()


def test_best_effort_stop_attempts_every_zero_after_write_errors():
    """Failed writes cannot suppress retries or claim that motion stopped."""
    attempts = []

    class FailingVendor:
        def set_car_motion(self, vx, vy, wz):
            attempts.append((vx, vy, wz))
            raise OSError("serial disconnected")

    driver = object.__new__(YahboomCarDriver)
    driver.car = FailingVendor()
    driver.motion_safety = SimpleNamespace(motion_stopped=False)

    errors = driver.best_effort_stop_motion(repeat=3)

    assert attempts == [(0.0, 0.0, 0.0)] * 3
    assert len(errors) == 3
    assert all("serial disconnected" in error for error in errors)
    assert not driver.motion_safety.motion_stopped


def test_cmd_vel_watchdog_uses_monotonic_not_frozen_ros_time(monkeypatch):
    """Paused ROS time cannot keep a prior nonzero motion command alive."""
    monotonic_clock = ManualClock(10.0)
    frozen_ros_clock = ManualClock(500.0)
    _parameters, _publishers = _install_node_harness(
        monkeypatch,
        monotonic_clock,
        ros_clock=frozen_ros_clock,
    )
    vendor = FakeVendorController()
    transport = FakeTransport(
        vendor,
        _status(
            TransportState.HEALTHY,
            ages={"speed": 0.01, "encoder": 0.01, "imu_raw": 0.01},
            sequence=3,
        ),
    )
    driver = YahboomCarDriver(
        "driver_node",
        controller_factory=lambda **_kwargs: transport,
        monotonic_clock=monotonic_clock,
    )
    command = Twist()
    command.linear.x = 0.2
    driver.cmd_vel_callback(command)
    assert vendor.motion_commands[-1] == (0.2, 0.0, 0.0)

    monotonic_clock.advance(driver.cmd_vel_timeout + 0.01)
    driver.check_cmd_vel_watchdog()

    assert frozen_ros_clock.seconds == 500.0
    assert vendor.motion_commands[-3:] == [(0.0, 0.0, 0.0)] * 3
    assert driver.motion_safety.motion_stopped


def test_swallowed_zero_write_failure_latches_and_keeps_stop_unproven(
    monkeypatch,
):
    """A vendor-hidden watchdog write error remains terminal and observable."""
    clock = ManualClock()
    _parameters, publishers = _install_node_harness(monkeypatch, clock)
    vendor = FakeVendorController()
    transport = FakeTransport(
        vendor,
        _status(
            TransportState.HEALTHY,
            ages={"speed": 0.01, "encoder": 0.01, "imu_raw": 0.01},
            sequence=3,
        ),
    )
    driver = YahboomCarDriver(
        "driver_node",
        controller_factory=lambda **_kwargs: transport,
        monotonic_clock=clock,
    )
    command = Twist()
    command.linear.x = 0.2
    driver.cmd_vel_callback(command)

    original_set_motion = vendor.set_car_motion

    def swallow_failed_zero(vx, vy, wz):
        original_set_motion(vx, vy, wz)
        if (vx, vy, wz) == (0.0, 0.0, 0.0):
            reason = "controller serial write raised OSError: disconnected"
            transport.serial_write_failure_count += 1
            transport.latest_serial_write_failure = reason
            transport.latch_failure(reason)

    vendor.set_car_motion = swallow_failed_zero
    clock.advance(driver.cmd_vel_timeout + 0.01)
    driver.check_cmd_vel_watchdog()

    assert transport.current_status.state is TransportState.FAILED
    assert transport.serial_write_failure_count >= 4
    assert not driver.motion_safety.motion_stopped
    diagnostic, values = _diagnostic_values(publishers)
    assert diagnostic.level == DiagnosticStatus.ERROR
    assert values["feedback_state"] == "failed"
    assert int(values["serial_write_failure_count"]) >= 1


def test_malformed_cached_tuple_is_terminal_without_publication(monkeypatch):
    """Malformed cache shapes receive the same diagnostic failure path."""
    clock = ManualClock()
    _, publishers = _install_node_harness(monkeypatch, clock)
    vendor = FakeVendorController()
    vendor.get_accelerometer_data = lambda: (0.1, 0.2)
    transport = FakeTransport(
        vendor,
        _status(
            TransportState.HEALTHY,
            ages={"speed": 0.01, "encoder": 0.01, "imu_raw": 0.01},
            sequence=3,
        ),
    )
    driver = YahboomCarDriver(
        "driver_node",
        controller_factory=lambda **_kwargs: transport,
        monotonic_clock=clock,
    )

    driver.publish_data()

    assert _topic_counts(publishers) == {topic: 0 for topic in _DATA_TOPICS}
    assert transport.current_status.state is TransportState.FAILED
    assert "cache validation failed" in transport.current_status.reason
    diagnostic, values = _diagnostic_values(publishers)
    assert diagnostic.level == DiagnosticStatus.ERROR
    assert values["feedback_state"] == "failed"


@pytest.mark.parametrize(
    "parameter,value",
    [
        ("cmd_vel_timeout", float("nan")),
        ("encoder_cpr", float("inf")),
        ("xlinear_limit", float("nan")),
        ("serial_write_timeout", float("inf")),
    ],
)
def test_nonfinite_safety_parameters_are_rejected(
    monkeypatch, parameter, value
):
    """Non-finite configuration cannot disable a runtime safety bound."""
    clock = ManualClock()
    _install_node_harness(
        monkeypatch,
        clock,
        parameter_overrides={parameter: value},
    )

    with pytest.raises(ValueError, match="finite"):
        YahboomCarDriver(
            "driver_node",
            controller_factory=lambda **_kwargs: pytest.fail(
                "controller factory must not run"
            ),
            monotonic_clock=clock,
        )
