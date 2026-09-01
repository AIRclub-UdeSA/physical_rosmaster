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

"""Hardware-free tests for checksum-valid Rosmaster report monitoring."""

import gc
import hashlib
from pathlib import Path
import math
import sys
import threading
from types import SimpleNamespace

import pytest

from yahboomcar_bringup.rosmaster_transport import RosmasterCompatibilityError
from yahboomcar_bringup.rosmaster_transport import RosmasterTransportError
from yahboomcar_bringup.rosmaster_transport import RosmasterTransport
from yahboomcar_bringup.rosmaster_transport import TransportState
import yahboomcar_bringup.rosmaster_transport as transport_module


PARSER_FAILURE = 0xEE
NONREQUIRED_REPORT = 0x51


class FakeClock:
    """Provide deterministic monotonic time to the transport."""

    def __init__(self, now=0.0):
        """Initialize the fake clock."""
        self.now = now

    def __call__(self):
        """Return the configured time or raise a configured exception."""
        if isinstance(self.now, BaseException):
            raise self.now
        return self.now

    def advance(self, seconds):
        """Advance deterministic time."""
        self.now += seconds


class FakeSerial:
    """Release a fake receive loop when serial closes."""

    def __init__(self, release_event):
        """Store the event controlled by close."""
        self.release_event = release_event
        self.closed = False
        self.close_count = 0
        self.writes = []
        self.write_exception = None
        self.short_write = None

    def close(self):
        """Close idempotently and release the reader."""
        self.closed = True
        self.close_count += 1
        self.release_event.set()

    def write(self, payload):
        """Record a write or emulate a failure hidden by the vendor."""
        self.writes.append(tuple(payload))
        if self.write_exception is not None:
            raise self.write_exception
        if self.short_write is not None:
            return self.short_write
        return len(payload)


class FakeRosmaster:
    """Implement the exact private hook names used by V3.3.9."""

    def __init__(self, receive_exception=None):
        """Initialize cached data and a controllable receive loop."""
        self.release_event = threading.Event()
        self.ser = FakeSerial(self.release_event)
        self.receive_exception = receive_exception
        self.parsed_types = []
        self.cached_encoders = (7, 7, 7, 7)
        self.cached_getter_calls = 0

    def _Rosmaster__parse_data(self, ext_type, _ext_data):
        """Accept a report or emulate a vendor parser failure."""
        if ext_type == PARSER_FAILURE:
            raise ValueError("malformed report")
        self.parsed_types.append(ext_type)

    def _Rosmaster__receive_data(self):
        """Wait until the test requests normal exit or an exception."""
        self.release_event.wait(timeout=10.0)
        if self.receive_exception is not None:
            raise self.receive_exception

    def emit_report(self, ext_type, payload=b"unchanged"):
        """Route a report through dynamic private-method dispatch."""
        self._Rosmaster__parse_data(ext_type, payload)

    def get_motor_encoder(self):
        """Return unchanged cached data without creating report freshness."""
        self.cached_getter_calls += 1
        return self.cached_encoders

    def set_car_motion(self, vx, vy, wz):
        """Model the deployed V3.3.9 bare-except serial-write behavior."""
        try:
            self.ser.write((vx, vy, wz))
        except BaseException:
            pass

    def release_receiver(self):
        """Let the receive loop return normally."""
        self.release_event.set()


def started_transport(clock=None, **vendor_kwargs):
    """Return a started fake transport and its deterministic clock."""
    fake_clock = clock or FakeClock()
    transport = RosmasterTransport(
        FakeRosmaster,
        clock=fake_clock,
        **vendor_kwargs,
    )
    transport.start()
    return transport, fake_clock


def emit_complete_feedback(transport, imu_type=0x0B):
    """Emit the three required report channels."""
    transport.vendor.emit_report(0x0A)
    transport.vendor.emit_report(0x0D)
    transport.vendor.emit_report(imu_type)


def wait_for_receiver_exit(transport):
    """Join the retained fake receiver and require deterministic completion."""
    thread = transport.receiver_thread
    assert thread is not None
    thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_initial_state_waits_for_all_required_reports():
    """Startup stays non-healthy until speed, encoder, and IMU arrive."""
    transport = RosmasterTransport(FakeRosmaster, clock=FakeClock())
    assert transport.status().state is TransportState.CREATED

    transport.start()
    try:
        status = transport.status()
        assert status.state is TransportState.WAITING
        assert set(status.report_ages) == {"speed", "encoder", "imu_raw"}
        assert all(age is None for age in status.report_ages.values())
    finally:
        transport.close()


@pytest.mark.parametrize("imu_type", [0x0B, 0x0E])
def test_either_raw_imu_report_completes_required_feedback(imu_type):
    """Both supported raw-IMU report IDs satisfy the same health channel."""
    transport, _ = started_transport()
    try:
        emit_complete_feedback(transport, imu_type=imu_type)
        status = transport.status()
        assert status.state is TransportState.HEALTHY
        assert status.report_sequence == 3
    finally:
        transport.close()


def test_identical_repeated_reports_refresh_health():
    """Unchanged stationary values remain fresh when real reports repeat."""
    transport, clock = started_transport()
    try:
        emit_complete_feedback(transport)
        clock.advance(0.4)
        emit_complete_feedback(transport)
        clock.advance(0.5)

        status = transport.status()
        assert status.state is TransportState.HEALTHY
        assert status.report_sequence == 6
        assert all(age == pytest.approx(0.5) for age in status.report_ages.values())
    finally:
        transport.close()


def test_partial_streams_fail_after_startup_timeout_not_at_boundary():
    """Startup accepts its exact boundary but fails once it is exceeded."""
    transport, clock = started_transport()
    try:
        transport.vendor.emit_report(0x0A)
        transport.vendor.emit_report(0x0D)
        clock.advance(2.0)
        assert transport.status().state is TransportState.WAITING

        clock.advance(0.000001)
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "imu_raw" in status.reason
    finally:
        transport.close()


def test_freshness_boundary_and_transient_subtimeout_refresh():
    """Sub-timeout delays recover and the exact stale boundary remains valid."""
    transport, clock = started_transport()
    try:
        emit_complete_feedback(transport)
        clock.advance(0.49)
        emit_complete_feedback(transport)
        clock.advance(0.5)
        assert transport.status().state is TransportState.HEALTHY

        clock.advance(0.000001)
        assert transport.status().state is TransportState.FAILED
    finally:
        transport.close()


def test_cached_getters_never_refresh_feedback():
    """Reading unchanged vendor caches cannot disguise a stopped receiver."""
    transport, clock = started_transport()
    try:
        emit_complete_feedback(transport)
        clock.advance(0.500001)

        assert transport.vendor.get_motor_encoder() == (7, 7, 7, 7)
        assert transport.vendor.get_motor_encoder() == (7, 7, 7, 7)
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert status.report_sequence == 3
        assert transport.vendor.cached_getter_calls == 2
    finally:
        transport.close()


def test_atomic_cache_reader_requires_fresh_reports_before_and_after_read():
    """A coherent cache copy is authorized only by live report timestamps."""
    transport, clock = started_transport()
    try:
        with pytest.raises(RosmasterTransportError, match="waiting"):
            transport.read_cached(lambda vendor: vendor.get_motor_encoder())

        emit_complete_feedback(transport)
        assert transport.read_cached(
            lambda vendor: vendor.get_motor_encoder()
        ) == (7, 7, 7, 7)

        clock.advance(0.500001)
        with pytest.raises(RosmasterTransportError, match="stale"):
            transport.read_cached(lambda vendor: vendor.get_motor_encoder())
    finally:
        transport.close()


def test_cache_reader_exception_latches_terminal_failure():
    """A cache conversion failure cannot leave actuator authorization healthy."""
    transport, _ = started_transport()
    try:
        emit_complete_feedback(transport)

        def fail_reader(_vendor):
            raise ValueError("bad cached value")

        with pytest.raises(ValueError, match="bad cached value"):
            transport.read_cached(fail_reader)
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "cache reader raised ValueError" in status.reason
    finally:
        transport.close()


def test_receive_exception_is_captured_as_terminal_failure():
    """An exception escaping the vendor receive loop is retained clearly."""
    transport, _ = started_transport(
        receive_exception=OSError("controller disconnected")
    )
    transport.vendor.release_receiver()
    wait_for_receiver_exit(transport)
    try:
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "OSError" in status.reason
        assert "controller disconnected" in status.reason
    finally:
        transport.close()


def test_failed_status_keeps_report_ages_advancing():
    """A receive exception freezes values, not their diagnostic ages."""
    transport, clock = started_transport(
        receive_exception=OSError("controller disconnected")
    )
    try:
        emit_complete_feedback(transport)
        clock.advance(0.2)
        transport.vendor.release_receiver()
        wait_for_receiver_exit(transport)
        first = transport.status()
        assert all(
            age == pytest.approx(0.2)
            for age in first.report_ages.values()
        )

        clock.advance(0.3)
        later = transport.status()
        assert later.state is TransportState.FAILED
        assert all(
            age == pytest.approx(0.5)
            for age in later.report_ages.values()
        )
    finally:
        transport.close()


def test_unexpected_normal_receive_exit_is_failure():
    """The V3.3.9 infinite receive loop must never return while active."""
    transport, _ = started_transport()
    transport.vendor.release_receiver()
    wait_for_receiver_exit(transport)
    try:
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "exited unexpectedly" in status.reason
    finally:
        transport.close()


def test_failure_is_terminal_even_if_reports_resume():
    """Late reports cannot erase a feedback-loss decision."""
    transport, clock = started_transport()
    try:
        emit_complete_feedback(transport)
        clock.advance(0.500001)
        failed = transport.status()
        assert failed.state is TransportState.FAILED

        clock.advance(0.1)
        emit_complete_feedback(transport, imu_type=0x0E)
        resumed = transport.status()
        assert resumed.state is TransportState.FAILED
        assert resumed.reason == failed.reason
        assert resumed.report_sequence == failed.report_sequence
    finally:
        transport.close()


def test_vendor_swallowed_serial_write_error_is_terminal():
    """The adapter sees write errors hidden by the V3.3.9 bare except."""
    transport, _ = started_transport()
    try:
        emit_complete_feedback(transport)
        serial_port = transport.vendor.ser._serial_port
        serial_port.write_exception = OSError("USB write failed")

        transport.vendor.set_car_motion(0.2, 0.0, 0.0)

        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "USB write failed" in status.reason
        assert transport.serial_write_failure_count == 1
        assert "USB write failed" in transport.latest_serial_write_failure
    finally:
        transport.close()


def test_vendor_swallowed_short_write_is_terminal():
    """A partial outbound frame is treated as a failed serial write."""
    transport, _ = started_transport()
    try:
        emit_complete_feedback(transport)
        transport.vendor.ser._serial_port.short_write = 1

        transport.vendor.set_car_motion(0.2, 0.0, 0.0)

        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "short write" in status.reason
        assert transport.serial_write_failure_count == 1
    finally:
        transport.close()


def test_healthy_action_is_rejected_after_failure_latches():
    """A terminal state cannot authorize a later actuator operation."""
    transport, _ = started_transport()
    actions = []
    try:
        emit_complete_feedback(transport)
        transport.latch_failure("receiver failed first")

        with pytest.raises(RosmasterTransportError, match="receiver failed"):
            transport.perform_while_healthy(
                "test action", lambda _vendor: actions.append("ran")
            )
        assert actions == []
    finally:
        transport.close()


def test_healthy_action_and_failure_latch_have_one_lock_order():
    """A concurrent failure cannot interleave inside an authorized action."""
    transport, _ = started_transport()
    action_entered = threading.Event()
    release_action = threading.Event()
    failure_returned = threading.Event()
    events = []

    def action(_vendor):
        action_entered.set()
        assert release_action.wait(timeout=1.0)
        events.append("action")

    def fail_transport():
        transport.latch_failure("concurrent receive failure")
        events.append("failure")
        failure_returned.set()

    try:
        emit_complete_feedback(transport)
        action_thread = threading.Thread(
            target=lambda: transport.perform_while_healthy(
                "test action", action
            )
        )
        action_thread.start()
        assert action_entered.wait(timeout=1.0)
        failure_thread = threading.Thread(target=fail_transport)
        failure_thread.start()
        assert not failure_returned.wait(timeout=0.05)

        release_action.set()
        action_thread.join(timeout=1.0)
        failure_thread.join(timeout=1.0)
        assert not action_thread.is_alive()
        assert not failure_thread.is_alive()
        assert events == ["action", "failure"]
        assert transport.status().state is TransportState.FAILED
    finally:
        release_action.set()
        transport.close()


def test_parser_exception_fails_without_marking_report():
    """A checksum-valid but malformed payload cannot refresh health."""
    transport, _ = started_transport()
    try:
        with pytest.raises(ValueError, match="malformed report"):
            transport.vendor.emit_report(PARSER_FAILURE)
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "parser raised ValueError" in status.reason
        assert status.report_sequence == 0
    finally:
        transport.close()


def test_nonrequired_report_id_does_not_refresh_startup():
    """A valid response unrelated to consumed telemetry is not liveness."""
    transport, clock = started_transport()
    try:
        transport.vendor.emit_report(NONREQUIRED_REPORT)
        assert transport.status().state is TransportState.WAITING
        assert transport.status().report_sequence == 0

        clock.advance(2.000001)
        assert transport.status().state is TransportState.FAILED
    finally:
        transport.close()


@pytest.mark.parametrize(
    "vendor_base,missing_name",
    [
        (type("NoHooks", (), {}), "_Rosmaster__parse_data"),
        (
            type(
                "OnlyParser",
                (),
                {"_Rosmaster__parse_data": lambda self, kind, data: None},
            ),
            "_Rosmaster__receive_data",
        ),
    ],
)
def test_missing_exact_private_hooks_fail_clearly(vendor_base, missing_name):
    """Near-compatible vendor classes must not silently bypass monitoring."""
    with pytest.raises(RosmasterCompatibilityError, match=missing_name):
        RosmasterTransport(vendor_base, clock=FakeClock())


@pytest.mark.parametrize(
    "timeout_name", ["startup_timeout", "stale_timeout", "write_timeout"]
)
@pytest.mark.parametrize("invalid_timeout", [0.0, -1.0, math.inf, math.nan])
def test_invalid_timeouts_are_rejected(timeout_name, invalid_timeout):
    """Timeout configuration must be finite and positive."""
    with pytest.raises(ValueError, match="finite positive"):
        RosmasterTransport(
            FakeRosmaster,
            clock=FakeClock(),
            **{timeout_name: invalid_timeout},
        )


def test_serial_write_timeout_is_applied_before_runtime_commands():
    """Every post-construction vendor write uses a bounded pyserial timeout."""
    transport = RosmasterTransport(
        FakeRosmaster, clock=FakeClock(), write_timeout=0.025
    )
    try:
        assert transport.vendor.ser._serial_port.write_timeout == pytest.approx(
            0.025
        )
    finally:
        transport.close()


@pytest.mark.parametrize("invalid_time", [math.inf, math.nan])
def test_nonfinite_clock_time_latches_failure(invalid_time):
    """A non-finite monotonic-clock result cannot produce healthy data."""
    transport = RosmasterTransport(FakeRosmaster, clock=FakeClock(invalid_time))
    try:
        transport.start()
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "non-finite" in status.reason
    finally:
        transport.close()


def test_clock_exception_latches_failure():
    """Clock exceptions become explicit terminal transport failures."""
    transport = RosmasterTransport(
        FakeRosmaster,
        clock=FakeClock(RuntimeError("clock unavailable")),
    )
    try:
        transport.start()
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "clock read failed" in status.reason
    finally:
        transport.close()


def test_backward_clock_latches_failure():
    """The injected monotonic source must never move backwards."""
    clock = FakeClock(10.0)
    transport, _ = started_transport(clock=clock)
    try:
        emit_complete_feedback(transport)
        assert transport.status().state is TransportState.HEALTHY

        clock.now = 9.0
        status = transport.status()
        assert status.state is TransportState.FAILED
        assert "moved backwards" in status.reason
    finally:
        transport.close()


def test_close_is_expected_and_idempotent():
    """Explicit close does not misclassify receiver termination as failure."""
    transport, _ = started_transport()
    transport.close()
    transport.close()
    assert transport.status().state is TransportState.CLOSED


def test_status_cannot_observe_a_published_but_unstarted_thread(monkeypatch):
    """Starting and publishing the receiver thread is one atomic transition."""
    original_thread = threading.Thread
    start_entered = threading.Event()
    release_start = threading.Event()

    class BlockingStartThread(original_thread):
        def start(self):
            start_entered.set()
            assert release_start.wait(timeout=1.0)
            return super().start()

    monkeypatch.setattr(transport_module.threading, "Thread", BlockingStartThread)
    transport = RosmasterTransport(FakeRosmaster, clock=FakeClock())
    statuses = []
    starter = original_thread(target=transport.start)
    starter.start()
    assert start_entered.wait(timeout=1.0)
    observer = original_thread(target=lambda: statuses.append(transport.status()))
    observer.start()

    release_start.set()
    starter.join(timeout=1.0)
    observer.join(timeout=1.0)
    try:
        assert not starter.is_alive()
        assert not observer.is_alive()
        assert statuses[0].state is TransportState.WAITING
    finally:
        transport.close()


def test_close_cannot_be_followed_by_a_late_receiver_start(monkeypatch):
    """A concurrent close waits until receiver startup is fully committed."""
    original_thread = threading.Thread
    start_entered = threading.Event()
    release_start = threading.Event()

    class BlockingStartThread(original_thread):
        def start(self):
            start_entered.set()
            assert release_start.wait(timeout=1.0)
            return super().start()

    monkeypatch.setattr(transport_module.threading, "Thread", BlockingStartThread)
    transport = RosmasterTransport(FakeRosmaster, clock=FakeClock())
    starter = original_thread(target=transport.start)
    closer = original_thread(target=transport.close)
    starter.start()
    assert start_entered.wait(timeout=1.0)
    closer.start()

    release_start.set()
    starter.join(timeout=1.0)
    closer.join(timeout=1.0)
    assert not starter.is_alive()
    assert not closer.is_alive()
    assert transport.status().state is TransportState.CLOSED
    receiver = transport.receiver_thread
    assert receiver is not None
    receiver.join(timeout=1.0)
    assert not receiver.is_alive()


def test_partial_vendor_constructor_is_cleaned_without_destructor_error():
    """A failed vendor constructor still closes an initialized serial member."""
    constructed_serials = []

    class FailingVendor(FakeRosmaster):
        def __init__(self):
            release_event = threading.Event()
            self.ser = FakeSerial(release_event)
            constructed_serials.append(self.ser)
            raise RuntimeError("constructor failed")

    with pytest.raises(RuntimeError, match="constructor failed"):
        RosmasterTransport(FailingVendor, clock=FakeClock())
    gc.collect()

    assert len(constructed_serials) == 1
    assert constructed_serials[0].closed


def test_unverified_vendor_source_is_rejected_before_construction(monkeypatch):
    """Private-hook monitoring never instantiates an unknown vendor source."""
    constructions = []

    class UnknownVendor(FakeRosmaster):
        def __init__(self):
            constructions.append(True)
            super().__init__()

    monkeypatch.setitem(
        sys.modules,
        "Rosmaster_Lib",
        SimpleNamespace(Rosmaster=UnknownVendor),
    )
    monkeypatch.setattr(
        transport_module,
        "rosmaster_source_sha256",
        lambda _vendor: (Path("/test/Rosmaster_Lib.py"), "0" * 64),
    )

    with pytest.raises(RosmasterCompatibilityError, match="unsupported"):
        transport_module.create_verified_rosmaster_transport(clock=FakeClock())
    assert constructions == []


def test_source_hash_reads_a_plain_python_module():
    """The deployed x3-c plain-file installation shape hashes directly."""
    source_path, digest = transport_module.rosmaster_source_sha256(FakeRosmaster)

    assert source_path == Path(__file__)
    assert digest == hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def test_source_hash_uses_a_spec_only_loader(monkeypatch):
    """A zip-style loader may be exposed only through the module spec."""
    source = b"# V3.3.9\nclass Rosmaster: pass\n"

    class SpecLoader:
        def get_data(self, path):
            assert path == "/virtual/Rosmaster_Lib.py"
            return source

    vendor_base = type(
        "Rosmaster",
        (),
        {"__module__": "spec_only_rosmaster"},
    )
    module = SimpleNamespace(
        __file__="/virtual/Rosmaster_Lib.py",
        __loader__=None,
        __spec__=SimpleNamespace(loader=SpecLoader()),
    )
    monkeypatch.setitem(sys.modules, "spec_only_rosmaster", module)

    source_path, digest = transport_module.rosmaster_source_sha256(vendor_base)

    assert source_path == Path("/virtual/Rosmaster_Lib.py")
    assert digest == hashlib.sha256(source).hexdigest()


def test_verified_vendor_source_is_wrapped(monkeypatch):
    """The exact allowlisted source proceeds to monitored construction."""
    monkeypatch.setitem(
        sys.modules,
        "Rosmaster_Lib",
        SimpleNamespace(Rosmaster=FakeRosmaster),
    )
    monkeypatch.setattr(
        transport_module,
        "rosmaster_source_sha256",
        lambda _vendor: (
            Path("/test/Rosmaster_Lib.py"),
            transport_module.PUBLIC_V3_3_9_SHA256,
        ),
    )

    transport = transport_module.create_verified_rosmaster_transport(
        clock=FakeClock()
    )
    try:
        assert transport.status().state is TransportState.CREATED
    finally:
        transport.close()
