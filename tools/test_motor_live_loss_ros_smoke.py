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

"""Real ROS 2/DDS process smoke test for the no-motion live-loss probe."""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import tempfile
import time

import pytest


REQUIRED_ROS_MODULES = (
    "ament_index_python.packages",
    "diagnostic_msgs.msg",
    "geometry_msgs.msg",
    "launch",
    "launch_ros.actions",
    "nav_msgs.msg",
    "sensor_msgs.msg",
    "std_msgs.msg",
    "tf2_msgs.msg",
)


def require_ros() -> None:
    """Skip without ROS; fail when an installed ROS lacks a dependency."""
    try:
        rclpy_available = importlib.util.find_spec("rclpy") is not None
    except (ImportError, ModuleNotFoundError):
        rclpy_available = False
    if not rclpy_available:
        pytest.skip("ROS 2 Python runtime unavailable: rclpy")

    missing = []
    for module in REQUIRED_ROS_MODULES:
        try:
            available = importlib.util.find_spec(module) is not None
        except (ImportError, ModuleNotFoundError):
            available = False
        if not available:
            missing.append(module)
    if missing:
        pytest.fail(
            "ROS 2 is installed but smoke-test dependencies are unavailable: "
            + ", ".join(missing)
        )


@contextmanager
def isolated_ros_domain():
    """Reserve one nondefault, Linux-safe ROS domain across local test runs."""
    domains = list(range(20, 100))
    start = os.getpid() % len(domains)
    for offset in range(len(domains)):
        domain = domains[(start + offset) % len(domains)]
        lock_path = Path(tempfile.gettempdir()) / (
            "physical-rosmaster-domain-%d.lock" % domain
        )
        descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(descriptor)
            continue
        try:
            yield domain
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)
        return
    raise RuntimeError("no isolated ROS_DOMAIN_ID was available")


def process_group_exists(process_group: int) -> bool:
    """Return whether the isolated process group still has any members."""
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    return True


def wait_for_process_group_exit(
    process: subprocess.Popen,
    process_group: int,
    timeout: float,
) -> bool:
    """Reap the leader and wait until every group member has disappeared."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process.poll()
        if not process_group_exists(process_group):
            return True
        time.sleep(0.02)
    process.poll()
    return not process_group_exists(process_group)


def terminate_process(
    process: subprocess.Popen | None,
    process_group: int | None,
) -> None:
    """Bound cleanup of an isolated group, even after its leader exits."""
    if process is None or process_group is None:
        return
    if process_group_exists(process_group):
        try:
            os.killpg(process_group, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if not wait_for_process_group_exit(process, process_group, timeout=3.0):
        try:
            os.killpg(process_group, signal.SIGKILL)
        except ProcessLookupError:
            pass
        if not wait_for_process_group_exit(
            process, process_group, timeout=3.0
        ):
            raise RuntimeError(
                "process group %d survived SIGKILL" % process_group
            )
    if process.poll() is None:
        process.wait(timeout=1.0)


def read_log(path: Path) -> str:
    """Read a file-backed process log, including partial failure output."""
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return "<log file was not created>"


def wait_for_ready(
    process: subprocess.Popen,
    ready_file: Path,
    stdout_path: Path,
    stderr_path: Path,
    timeout: float,
) -> None:
    """Wait for helper construction without hiding an early process error."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ready_file.exists():
            return
        if process.poll() is not None:
            pytest.fail(
                "strict launch supervisor exited before %s\n"
                "stdout:\n%s\nstderr:\n%s"
                % (
                    ready_file.name,
                    read_log(stdout_path),
                    read_log(stderr_path),
                )
            )
        time.sleep(0.02)
    pytest.fail(
        "%s was not created within %.1fs\nstdout:\n%s\nstderr:\n%s"
        % (
            ready_file.name,
            timeout,
            read_log(stdout_path),
            read_log(stderr_path),
        )
    )


def wait_for_stderr_token(
    process: subprocess.Popen,
    token: str,
    timeout: float,
    supervisor: subprocess.Popen,
    supervisor_stdout_path: Path,
    supervisor_stderr_path: Path,
) -> str:
    """Read line-buffered progress with a hard deadline."""
    assert process.stderr is not None
    observed = []
    deadline = time.monotonic() + timeout
    selector = selectors.DefaultSelector()
    selector.register(process.stderr, selectors.EVENT_READ)
    try:
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            events = selector.select(timeout=min(0.2, remaining))
            for key, _mask in events:
                line = key.fileobj.readline()
                if line:
                    observed.append(line)
                    if token in line:
                        return "".join(observed)
            if process.poll() is not None:
                observed.append(process.stderr.read())
                stdout = ""
                if process.stdout is not None:
                    stdout = process.stdout.read()
                pytest.fail(
                    "probe exited before %r\nstdout:\n%s\nstderr:\n%s\n"
                    "supervisor stdout:\n%s\nsupervisor stderr:\n%s"
                    % (
                        token,
                        stdout,
                        "".join(observed),
                        read_log(supervisor_stdout_path),
                        read_log(supervisor_stderr_path),
                    )
                )
            if supervisor.poll() is not None:
                pytest.fail(
                    "strict launch supervisor exited before probe emitted %r\n"
                    "stdout:\n%s\nstderr:\n%s"
                    % (
                        token,
                        read_log(supervisor_stdout_path),
                        read_log(supervisor_stderr_path),
                    )
                )
    finally:
        selector.close()
    pytest.fail(
        "probe did not emit %r within %.1fs\nstderr:\n%s\n"
        "supervisor stdout:\n%s\nsupervisor stderr:\n%s"
        % (
            token,
            timeout,
            "".join(observed),
            read_log(supervisor_stdout_path),
            read_log(supervisor_stderr_path),
        )
    )


def parse_report(stdout: str) -> dict:
    """Return the probe's final JSON object or fail with its full stdout."""
    for line in reversed(stdout.splitlines()):
        if not line.strip():
            continue
        try:
            report = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(report, dict):
            return report
    pytest.fail("probe emitted no JSON report\nstdout:\n%s" % stdout)


def test_real_dds_motor_loss_contract_passes(tmp_path):
    """Drive the real observer through healthy, loss, ERROR, and graph drain."""
    require_ros()
    tools_dir = Path(__file__).resolve().parent
    helper_path = tools_dir / "synthetic_motor_loss_driver.py"
    probe_path = tools_dir / "motor_live_loss_probe.py"
    device = tmp_path / "motor-device"
    driver_ready_file = tmp_path / "synthetic-driver-ready"
    sentinel_ready_file = tmp_path / "strict-sentinel-ready"
    sentinel_stopped_file = tmp_path / "strict-sentinel-stopped"
    supervisor_stdout_path = tmp_path / "strict-supervisor.stdout.log"
    supervisor_stderr_path = tmp_path / "strict-supervisor.stderr.log"
    ros_log_dir = tmp_path / "ros-logs"
    device.touch()
    ros_log_dir.mkdir()

    supervisor = None
    supervisor_group = None
    supervisor_stdout_handle = None
    supervisor_stderr_handle = None
    probe = None
    probe_group = None
    with isolated_ros_domain() as domain:
        environment = os.environ.copy()
        environment.update(
            {
                "PYTHONUNBUFFERED": "1",
                "ROS_DOMAIN_ID": str(domain),
                "ROS_LOCALHOST_ONLY": "1",
                "ROS_LOG_DIR": str(ros_log_dir),
            }
        )
        try:
            supervisor_stdout_handle = supervisor_stdout_path.open(
                "w", encoding="utf-8"
            )
            supervisor_stderr_handle = supervisor_stderr_path.open(
                "w", encoding="utf-8"
            )
            supervisor = subprocess.Popen(
                [
                    sys.executable,
                    str(helper_path),
                    "supervisor",
                    "--device",
                    str(device),
                    "--driver-ready-file",
                    str(driver_ready_file),
                    "--sentinel-ready-file",
                    str(sentinel_ready_file),
                    "--sentinel-stopped-file",
                    str(sentinel_stopped_file),
                    "--error-delay",
                    "0.20",
                    "--exit-delay",
                    "0.80",
                ],
                cwd=tools_dir,
                env=environment,
                stdout=supervisor_stdout_handle,
                stderr=supervisor_stderr_handle,
                text=True,
                start_new_session=True,
            )
            supervisor_group = supervisor.pid
            wait_for_ready(
                supervisor,
                driver_ready_file,
                supervisor_stdout_path,
                supervisor_stderr_path,
                timeout=5.0,
            )
            wait_for_ready(
                supervisor,
                sentinel_ready_file,
                supervisor_stdout_path,
                supervisor_stderr_path,
                timeout=5.0,
            )

            probe = subprocess.Popen(
                [
                    sys.executable,
                    str(probe_path),
                    "--device",
                    str(device),
                    "--confirm-wheels-secured",
                    "--baseline-duration",
                    "0.60",
                    "--baseline-timeout",
                    "10.0",
                    "--loss-wait-timeout",
                    "8.0",
                    "--quiet-deadline",
                    "0.50",
                    "--diagnostic-deadline",
                    "2.0",
                    "--driver-exit-deadline",
                    "6.0",
                    "--graph-drain-dwell",
                    "0.30",
                    "--overall-deadline",
                    "12.0",
                ],
                cwd=tools_dir,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            probe_group = probe.pid
            progress = wait_for_stderr_token(
                probe,
                "ARMED:",
                timeout=12.0,
                supervisor=supervisor,
                supervisor_stdout_path=supervisor_stdout_path,
                supervisor_stderr_path=supervisor_stderr_path,
            )
            device.unlink()

            try:
                probe_stdout, probe_stderr = probe.communicate(timeout=16.0)
            except subprocess.TimeoutExpired:
                terminate_process(probe, probe_group)
                pytest.fail(
                    "probe did not finish within the post-loss deadline\n"
                    "supervisor stdout:\n%s\nsupervisor stderr:\n%s"
                    % (
                        read_log(supervisor_stdout_path),
                        read_log(supervisor_stderr_path),
                    )
                )
            progress += probe_stderr

            try:
                supervisor.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                terminate_process(supervisor, supervisor_group)
                pytest.fail(
                    "strict launch supervisor did not shut down\n"
                    "stdout:\n%s\nstderr:\n%s"
                    % (
                        read_log(supervisor_stdout_path),
                        read_log(supervisor_stderr_path),
                    )
                )
            supervisor_stdout = read_log(supervisor_stdout_path)
            supervisor_stderr = read_log(supervisor_stderr_path)

            report = parse_report(probe_stdout)
            assert probe.returncode == 0, (
                "probe failed\nstdout:\n%s\nstderr:\n%s\n"
                "supervisor stdout:\n%s\nsupervisor stderr:\n%s"
                % (
                    probe_stdout,
                    progress,
                    supervisor_stdout,
                    supervisor_stderr,
                )
            )
            assert supervisor.returncode == 0, (
                "strict launch supervisor failed\nstdout:\n%s\nstderr:\n%s"
                % (supervisor_stdout, supervisor_stderr)
            )
            assert sentinel_stopped_file.exists(), (
                "strict sentinel did not record launch-driven termination\n"
                "stdout:\n%s\nstderr:\n%s"
                % (supervisor_stdout, supervisor_stderr)
            )
            assert report["outcome"] == "PASS"
            assert report["phase"] == "passed"
            assert report["exit_code"] == 0
            assert report["unsafe"] is False
            assert report["reasons"] == []
            assert "ARMED:" in progress
            assert "Motor device loss observed" in progress

            timing = report["timing_offsets_seconds"]
            assert timing["armed"] is not None
            assert timing["device_loss"] is not None
            assert timing["freshness_error"] is not None
            assert timing["driver_gone"] is not None
            assert timing["strict_graph_drained"] is not None
            assert timing["strict_graph_drained"] >= timing["driver_gone"]
            accepted_error = report["accepted_failure_diagnostic"]
            assert accepted_error["level"] == 2
            assert accepted_error["values"]["feedback_state"] == "failed"
            assert report["graph"]["strict_endpoint_count"] == 0
            assert all(
                count >= 1
                for count in report["graph"][
                    "baseline_strict_topic_counts"
                ].values()
            )
            assert report["graph"]["baseline_strict_topic_counts"][
                "/odom"
            ] >= 2
            assert report["graph"]["latest_evidence"]["publishers"][
                "/cmd_vel"
            ] == []
        finally:
            try:
                try:
                    terminate_process(probe, probe_group)
                finally:
                    terminate_process(supervisor, supervisor_group)
            finally:
                if supervisor_stdout_handle is not None:
                    supervisor_stdout_handle.close()
                if supervisor_stderr_handle is not None:
                    supervisor_stderr_handle.close()
