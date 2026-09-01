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

"""Exercise the exact recovered Rosmaster V3.3.9 over a real POSIX PTY."""

from __future__ import annotations

import hashlib
import importlib.util
import os
from pathlib import Path
import pty
import struct
import time

import pytest

from yahboomcar_bringup.rosmaster_transport import PUBLIC_V3_3_9_SHA256
from yahboomcar_bringup.rosmaster_transport import RosmasterTransport
from yahboomcar_bringup.rosmaster_transport import TransportState


SOURCE_ENV = "ROSMASTER_V339_SOURCE"
IMU_VALUES = (1000, -2000, 3000, 4000, -5000, 6000, 7000, -8000, 9000)


@pytest.fixture(scope="module")
def exact_rosmaster_class():
    """Load only the externally supplied source with the reviewed digest."""
    try:
        __import__("serial")
    except ImportError as exc:
        pytest.fail(
            "python3-serial is required for this compatibility gate: %s"
            % exc
        )

    raw_path = os.environ.get(SOURCE_ENV)
    if not raw_path:
        pytest.fail(
            "%s must name the recovered Rosmaster_Lib.py" % SOURCE_ENV
        )
    source_path = Path(raw_path).expanduser().resolve()
    assert source_path.is_file(), "%s is not a file" % source_path
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == (
        PUBLIC_V3_3_9_SHA256
    )

    spec = importlib.util.spec_from_file_location(
        "rosmaster_exact_v339_pty_test", source_path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Rosmaster


@pytest.fixture
def pty_transport(exact_rosmaster_class):
    """Construct the exact vendor class against a real pyserial PTY."""
    master_fd, slave_fd = pty.openpty()
    transport = None
    try:
        slave_name = os.ttyname(slave_fd)
        transport = RosmasterTransport(
            exact_rosmaster_class,
            com=slave_name,
            delay=0.0,
            startup_timeout=0.4,
            stale_timeout=0.2,
            write_timeout=0.05,
        )
        yield transport, master_fd
    finally:
        receiver = (
            transport.receiver_thread if transport is not None else None
        )
        try:
            if transport is not None:
                transport.close(join_timeout=0.5)
        finally:
            for descriptor in (master_fd, slave_fd):
                try:
                    os.close(descriptor)
                except OSError:
                    pass
        if receiver is not None and receiver.is_alive():
            receiver.join(timeout=0.25)
            assert not receiver.is_alive(), (
                "exact V3.3.9 receiver thread leaked from PTY fixture"
            )


def _report_frame(function, payload):
    """Encode one controller-to-host frame accepted by V3.3.9."""
    length = len(payload) + 3
    checksum = (length + function + sum(payload)) & 0xFF
    return bytes([0xFF, 0xFB, length, function, *payload, checksum])


def _corrupt_checksum(frame):
    """Change only a frame checksum so its otherwise-valid payload is rejected."""
    return frame[:-1] + bytes([(frame[-1] + 1) & 0xFF])


def _required_reports(imu_type=0x0E, corrupt=False):
    """Encode one batch of the three controller-liveness report channels."""
    speed = struct.pack("<hhhB", 125, -250, 375, 124)
    encoders = struct.pack("<iiii", 11, -22, 33, -44)
    imu = struct.pack("<hhhhhhhhh", *IMU_VALUES)
    frames = (
        _report_frame(0x0A, speed),
        _report_frame(0x0D, encoders),
        _report_frame(imu_type, imu),
    )
    if corrupt:
        frames = tuple(_corrupt_checksum(frame) for frame in frames)
    return b"".join(frames)


def _imu_expected(imu_type):
    """Return exact-library cache values for either supported IMU report."""
    if imu_type == 0x0B:
        return (
            (
                IMU_VALUES[0] / 3754.9,
                -IMU_VALUES[1] / 3754.9,
                -IMU_VALUES[2] / 3754.9,
            ),
            tuple(value / 1671.84 for value in IMU_VALUES[3:6]),
            tuple(float(value) for value in IMU_VALUES[6:9]),
        )
    return (
        tuple(value / 1000.0 for value in IMU_VALUES[0:3]),
        tuple(value / 1000.0 for value in IMU_VALUES[3:6]),
        tuple(value / 1000.0 for value in IMU_VALUES[6:9]),
    )


def _wait_for(predicate, timeout=1.0):
    """Wait briefly for a receive-thread or serial-state transition."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.005)
    return predicate()


def _status_in_state(transport, expected_state):
    """Return one status snapshot only when it has the expected state."""
    status = transport.status()
    return status if status.state is expected_state else None


def _make_healthy(transport, master_fd, imu_type=0x0E):
    """Feed all required valid report types through the PTY."""
    reports = _required_reports(imu_type=imu_type)

    transport.start()
    deadline = time.monotonic() + 0.3
    last_status = None
    while time.monotonic() < deadline:
        os.write(master_fd, reports)
        last_status = transport.status()
        if last_status.state is TransportState.HEALTHY:
            return last_status
        if last_status.state is TransportState.FAILED:
            pytest.fail(last_status.reason)
        time.sleep(0.005)

    assert last_status is not None
    pytest.fail(
        "transport did not become healthy while reports were injected: %s"
        % last_status.reason
    )


@pytest.mark.parametrize("imu_type", [0x0B, 0x0E])
def test_exact_reports_are_parsed_then_staleness_fails_closed(
    pty_transport, imu_type
):
    """Both exact raw-IMU variants authorize caches only while fresh."""
    transport, master_fd = pty_transport
    status = _make_healthy(transport, master_fd, imu_type=imu_type)

    assert status.report_sequence >= 3
    assert transport.read_cached(lambda vendor: vendor.get_motion_data()) == (
        0.125,
        -0.25,
        0.375,
    )
    assert transport.read_cached(
        lambda vendor: vendor.get_motor_encoder()
    ) == (11, -22, 33, -44)
    assert transport.read_cached(
        lambda vendor: vendor.get_battery_voltage()
    ) == pytest.approx(12.4)
    gyro, acceleration, magnetic = _imu_expected(imu_type)
    assert transport.read_cached(
        lambda vendor: vendor.get_gyroscope_data()
    ) == pytest.approx(gyro)
    assert transport.read_cached(
        lambda vendor: vendor.get_accelerometer_data()
    ) == pytest.approx(acceleration)
    assert transport.read_cached(
        lambda vendor: vendor.get_magnetometer_data()
    ) == pytest.approx(magnetic)

    failed = _wait_for(
        lambda: _status_in_state(transport, TransportState.FAILED),
        timeout=0.5,
    )
    assert failed is not None
    assert "controller feedback stale" in failed.reason


def test_corrupt_checksums_cannot_establish_liveness(pty_transport):
    """A synchronized corrupt batch leaves every report channel missing."""
    transport, master_fd = pty_transport
    transport.start()

    first_attitude = _report_frame(0x0C, struct.pack("<hhh", 1000, 2000, 3000))
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        os.write(master_fd, first_attitude)
        if transport.vendor.get_imu_attitude_data(ToAngle=False) == (
            0.1,
            0.2,
            0.3,
        ):
            break
        time.sleep(0.005)
    else:
        pytest.fail("exact receiver did not consume the synchronization frame")

    second_attitude = _report_frame(
        0x0C, struct.pack("<hhh", 1100, 2200, 3300)
    )
    os.write(
        master_fd,
        _required_reports(corrupt=True) + second_attitude,
    )
    synchronized = _wait_for(
        lambda: transport.vendor.get_imu_attitude_data(ToAngle=False)
        == (0.11, 0.22, 0.33),
        timeout=0.3,
    )
    assert synchronized
    status = transport.status()
    assert status.state is TransportState.WAITING
    assert status.report_sequence == 0
    assert all(age is None for age in status.report_ages.values())


def test_corrupt_checksums_cannot_refresh_liveness(pty_transport):
    """Continuous corrupt reports cannot prevent a healthy link going stale."""
    transport, master_fd = pty_transport
    _make_healthy(transport, master_fd)

    attitude = _report_frame(0x0C, struct.pack("<hhh", 1230, 4560, 7890))
    os.write(master_fd, attitude)
    synchronized = _wait_for(
        lambda: transport.vendor.get_imu_attitude_data(ToAngle=False)
        == (0.123, 0.456, 0.789),
        timeout=0.3,
    )
    assert synchronized, "exact receiver did not drain the valid-frame backlog"
    initial_sequence = transport.status().report_sequence

    deadline = time.monotonic() + 0.5
    failed = None
    while time.monotonic() < deadline:
        os.write(master_fd, _required_reports(corrupt=True))
        status = transport.status()
        if status.state is TransportState.FAILED:
            failed = status
            break
        time.sleep(0.005)

    assert failed is not None
    assert "controller feedback stale" in failed.reason
    assert failed.report_sequence == initial_sequence


def test_real_pty_hangup_is_a_terminal_receive_failure(pty_transport):
    """A disconnected controller terminates the exact blocking receive loop."""
    transport, master_fd = pty_transport
    _make_healthy(transport, master_fd)
    os.close(master_fd)

    failed = _wait_for(
        lambda: _status_in_state(transport, TransportState.FAILED)
    )
    assert failed is not None
    assert "controller receive thread raised SerialException" in failed.reason


@pytest.mark.parametrize("write_result", [OSError("disconnected"), "short"])
def test_exact_vendor_swallowed_write_failures_are_latched(
    pty_transport, monkeypatch, write_result
):
    """Raised and short real-serial writes cannot hide in vendor bare excepts."""
    transport, master_fd = pty_transport
    _make_healthy(transport, master_fd)
    serial_port = transport.vendor.ser._serial_port

    def failed_write(payload):
        if isinstance(write_result, BaseException):
            raise write_result
        return len(payload) - 1

    monkeypatch.setattr(serial_port, "write", failed_write)
    transport.vendor.set_beep(0)

    status = transport.status()
    assert status.state is TransportState.FAILED
    assert transport.serial_write_failure_count == 1
    if write_result == "short":
        assert "serial short write" in status.reason
    else:
        assert "serial write raised OSError: disconnected" in status.reason


def test_close_unblocks_the_exact_receive_thread(pty_transport):
    """Closing the monitored transport leaves no live receiver thread."""
    transport, master_fd = pty_transport
    _make_healthy(transport, master_fd)
    receiver = transport.receiver_thread
    assert receiver is not None and receiver.is_alive()

    started = time.monotonic()
    transport.close(join_timeout=0.5)

    assert time.monotonic() - started < 0.75
    assert transport.status().state is TransportState.CLOSED
    assert not receiver.is_alive()
