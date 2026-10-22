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

# A platform stop sends the driver SIGTERM (the stop sweep) and SIGINT (ros2
# launch) in either order. These tests run the real main() in a separate
# process with a fake driver node whose final stop is slow, send one signal to
# stop it and another while the zeros are going out, and require every zero,
# the transport close and the node destruction to happen anyway. The stop
# handler itself is also checked in-process: it records the request and raises
# nothing, because an exception raised from a signal handler can be discarded
# silently inside a __del__.

"""A second stop signal must not cut the driver's final zero commands short."""

import os
import signal
import subprocess
import sys
import time

import pytest

from yahboomcar_bringup import Mcnamu_driver_X3 as driver_module


FAKE_DRIVER_MAIN = r"""
import os
import time

from rclpy.node import Node

from yahboomcar_bringup import Mcnamu_driver_X3 as driver_module


class FakeDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.transport = self
        self.ticks = 0
        self.create_timer(0.05, self.tick)
        print("SPINNING", flush=True)

    def tick(self):
        self.ticks += 1
        if os.environ.get("FAKE_FAULT") and self.ticks >= 5:
            raise RuntimeError("Required motor-controller feedback failed: fake")

    def best_effort_stop_motion(self, repeat=3):
        print("CLEANUP STARTED", flush=True)
        for attempt in range(repeat):
            time.sleep(0.3)
            print("ZERO %d" % (attempt + 1), flush=True)
        return []

    def close(self):
        print("CLOSED", flush=True)

    def destroy_node(self):
        print("DESTROYED", flush=True)
        super().destroy_node()


driver_module.YahboomCarDriver = FakeDriver
driver_module.main()
print("MAIN RETURNED", flush=True)
"""

COMPLETE_CLEANUP = ["CLEANUP STARTED", "ZERO 1", "ZERO 2", "ZERO 3", "CLOSED", "DESTROYED"]


def _start_fake_driver(fault=False):
    """Start the real main() around the fake driver, isolated on localhost."""
    env = dict(os.environ)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["PYTHONUNBUFFERED"] = "1"
    env.pop("FAKE_FAULT", None)
    if fault:
        env["FAKE_FAULT"] = "1"
    return subprocess.Popen(
        [sys.executable, "-c", FAKE_DRIVER_MAIN],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )


def _wait_for_line(process, expected, seen, timeout=20.0):
    """Read stdout into ``seen`` until ``expected`` appears."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if not line:
            break
        seen.append(line.strip())
        if seen[-1] == expected:
            return
    raise AssertionError("never saw %r; output so far: %r" % (expected, seen))


def _finish(process, seen):
    """Collect the rest of the output and the exit status."""
    try:
        stdout, stderr = process.communicate(timeout=20)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    seen.extend(line.strip() for line in stdout.splitlines())
    return process.returncode, stderr


@pytest.mark.parametrize(
    "first, second",
    [
        (signal.SIGTERM, signal.SIGINT),
        (signal.SIGINT, signal.SIGTERM),
        (signal.SIGINT, signal.SIGINT),
        (signal.SIGTERM, signal.SIGTERM),
    ],
    ids=["TERM-then-INT", "INT-then-TERM", "INT-then-INT", "TERM-then-TERM"],
)
def test_a_second_stop_signal_does_not_skip_the_final_zeros(first, second):
    """Both stop signals, in either order, still leave a complete cleanup."""
    process = _start_fake_driver()
    seen = []
    try:
        _wait_for_line(process, "SPINNING", seen)
        time.sleep(0.2)
        process.send_signal(first)
        _wait_for_line(process, "CLEANUP STARTED", seen)
        process.send_signal(second)
    finally:
        returncode, stderr = _finish(process, seen)
    assert returncode == 0, stderr
    assert "KeyboardInterrupt" not in stderr, stderr
    assert seen[seen.index("CLEANUP STARTED"):] == COMPLETE_CLEANUP + ["MAIN RETURNED"]


def test_a_stop_signal_during_a_fault_exit_does_not_skip_the_final_zeros():
    """The stop sweep can arrive while a terminal fault is already stopping."""
    process = _start_fake_driver(fault=True)
    seen = []
    try:
        _wait_for_line(process, "CLEANUP STARTED", seen)
        process.send_signal(signal.SIGTERM)
    finally:
        returncode, stderr = _finish(process, seen)
    # The fault still propagates after cleanup, so the launch sees a failure.
    assert returncode == 1, stderr
    assert "Required motor-controller feedback failed: fake" in stderr
    assert "KeyboardInterrupt" not in stderr, stderr
    assert seen[seen.index("CLEANUP STARTED"):] == COMPLETE_CLEANUP


def test_the_stop_handler_records_the_first_signal_and_raises_nothing():
    """Only a flag: later signals are ignored, and no exception is raised."""
    previous = {
        stop_signal: signal.getsignal(stop_signal)
        for stop_signal in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        stop_requested = driver_module.install_stop_signal_handlers()
        assert not stop_requested()
        signal.raise_signal(signal.SIGTERM)
        assert stop_requested()
        assert signal.getsignal(signal.SIGINT) is signal.SIG_IGN
        assert signal.getsignal(signal.SIGTERM) is signal.SIG_IGN
        # Ignored, so neither terminates nor interrupts this process.
        signal.raise_signal(signal.SIGINT)
        signal.raise_signal(signal.SIGTERM)
        assert stop_requested()
    finally:
        for stop_signal, handler in previous.items():
            signal.signal(stop_signal, handler)
