"""Tests for the ROS 2 shutdown request path (issue #18), without hardware.

Three layers: the listener's confirmation check; the listener as a real ROS 2
process on an isolated localhost domain (wrong values ignored, the confirmed
value exits with the trigger status, a latched request from before it started
never fires); and the host script against stub docker/systemctl/logger
commands (platform stopped before poweroff, nothing done without a request).
"""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

import ros_shutdown_listener
from test_motor_live_loss_ros_smoke import isolated_ros_domain


REPOSITORY = Path(__file__).resolve().parents[1]
LISTENER = REPOSITORY / "tools" / "ros_shutdown_listener.py"
HOST_SCRIPT = REPOSITORY / "config" / "rosmaster-shutdown-listen.sh"
CONFIRM = "testbot"


@pytest.mark.parametrize(
    "data, expected, confirmed",
    [
        ("testbot", "testbot", True),
        ("  testbot\n", "testbot", True),
        ("testbot", " testbot ", True),
        ("mikey", "testbot", False),
        ("TESTBOT", "testbot", False),
        ("testbot2", "testbot", False),
        ("test", "testbot", False),
        ("", "testbot", False),
        ("", "", False),
        ("   ", "   ", False),
        ("anything", "", False),
    ],
)
def test_only_the_exact_confirmation_value_is_a_request(data, expected, confirmed):
    assert ros_shutdown_listener.is_confirmed_request(data, expected) is confirmed


def test_the_listener_refuses_to_start_without_a_confirmation_value(monkeypatch):
    """An empty value would make every message, or none, a request."""
    monkeypatch.delenv(ros_shutdown_listener.CONFIRM_ENVIRONMENT_VARIABLE, raising=False)
    with pytest.raises(SystemExit) as raised:
        ros_shutdown_listener.parse_args([])
    assert raised.value.code == 2
    with pytest.raises(SystemExit):
        ros_shutdown_listener.parse_args(["--confirm", "  "])


def test_the_confirmation_value_comes_from_the_environment(monkeypatch):
    monkeypatch.setenv(ros_shutdown_listener.CONFIRM_ENVIRONMENT_VARIABLE, "mikey")
    assert ros_shutdown_listener.parse_args([]).confirm == "mikey"


def test_the_subscription_cannot_receive_a_latched_request():
    """Volatile durability is what keeps a stored request from firing at boot."""
    qos = ros_shutdown_listener.SHUTDOWN_QOS
    assert qos.durability == ros_shutdown_listener.DurabilityPolicy.VOLATILE
    assert qos.reliability == ros_shutdown_listener.ReliabilityPolicy.RELIABLE


def test_the_host_script_and_the_listener_agree_on_the_trigger_status():
    script = HOST_SCRIPT.read_text(encoding="utf-8")
    assert "EXIT_SHUTDOWN_REQUESTED=%d\n" % ros_shutdown_listener.EXIT_SHUTDOWN_REQUESTED in script
    assert ros_shutdown_listener.EXIT_NO_REQUEST != ros_shutdown_listener.EXIT_SHUTDOWN_REQUESTED


REQUESTER = r"""
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

durability = (
    DurabilityPolicy.TRANSIENT_LOCAL if sys.argv[1] == "latched" else DurabilityPolicy.VOLATILE
)
rclpy.init()
node = rclpy.create_node("shutdown_requester")
publisher = node.create_publisher(
    String,
    "/system/shutdown",
    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=durability),
)
for line in sys.stdin:
    command, _, argument = line.rstrip("\n").partition(" ")
    if command == "pub":
        publisher.publish(String(data=argument))
        print("PUBLISHED", flush=True)
    elif command == "wait":
        deadline = time.monotonic() + 20.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        print("MATCHED %d" % publisher.get_subscription_count(), flush=True)
"""


def ros_env(domain):
    """Isolate the test graph on localhost and a reserved domain."""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    env["PYTHONUNBUFFERED"] = "1"
    env.pop(ros_shutdown_listener.CONFIRM_ENVIRONMENT_VARIABLE, None)
    return env


class Requester:
    """A separate ROS process that publishes on /system/shutdown on command."""

    def __init__(self, env, durability):
        self.process = subprocess.Popen(
            [sys.executable, "-c", REQUESTER, durability],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            env=env,
        )

    def command(self, line, reply):
        self.process.stdin.write(line + "\n")
        self.process.stdin.flush()
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            output = self.process.stdout.readline()
            if not output:
                break
            if output.startswith(reply):
                return output.strip()
        raise AssertionError("requester did not answer %r with %r" % (line, reply))

    def wait_for_listener(self):
        assert self.command("wait", "MATCHED") == "MATCHED 1"

    def publish(self, data):
        self.command("pub " + data, "PUBLISHED")


def start_listener(env):
    return subprocess.Popen(
        [sys.executable, str(LISTENER), "--confirm", CONFIRM],
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )


def stop_all(*processes):
    for process in processes:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()


def require_rclpy():
    if importlib.util.find_spec("rclpy") is None:
        pytest.skip("ROS 2 Python runtime unavailable: rclpy")


def still_running_after(process, seconds):
    try:
        process.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        return True
    return False


def test_wrong_values_are_ignored_and_the_confirmed_value_exits_with_the_trigger():
    require_rclpy()
    with isolated_ros_domain() as domain:
        env = ros_env(domain)
        listener = requester = None
        try:
            listener = start_listener(env)
            requester = Requester(env, "volatile")
            requester.wait_for_listener()
            for wrong in ("mikey", "", "TESTBOT", "testbot please"):
                requester.publish(wrong)
            assert still_running_after(listener, 2.0), listener.stderr.read()

            requester.publish(" testbot ")
            listener.wait(timeout=15)
            stderr = listener.stderr.read()
            assert listener.returncode == ros_shutdown_listener.EXIT_SHUTDOWN_REQUESTED, stderr
            assert stderr.count("Ignored /system/shutdown request") == 4, stderr
            assert "Confirmed shutdown request" in stderr, stderr
            # The expected value is never logged on its own (it may be a secret).
            assert "'mikey'" in stderr
        finally:
            stop_all(listener, requester and requester.process)


def test_a_latched_request_from_before_the_listener_started_does_not_fire():
    """A transient-local request left over from before a reboot must not power off again."""
    require_rclpy()
    with isolated_ros_domain() as domain:
        env = ros_env(domain)
        listener = requester = None
        try:
            requester = Requester(env, "latched")
            requester.publish(CONFIRM)
            listener = start_listener(env)
            requester.wait_for_listener()
            assert still_running_after(listener, 3.0), listener.stderr.read()

            # Positive control: the same publisher reaches the listener live.
            requester.publish(CONFIRM)
            listener.wait(timeout=15)
            assert listener.returncode == ros_shutdown_listener.EXIT_SHUTDOWN_REQUESTED
        finally:
            stop_all(listener, requester and requester.process)


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
def test_stopping_the_listener_is_not_a_request(stop_signal):
    require_rclpy()
    with isolated_ros_domain() as domain:
        env = ros_env(domain)
        listener = requester = None
        try:
            listener = start_listener(env)
            requester = Requester(env, "volatile")
            requester.wait_for_listener()
            listener.send_signal(stop_signal)
            listener.wait(timeout=15)
            assert listener.returncode == ros_shutdown_listener.EXIT_NO_REQUEST, listener.stderr.read()
        finally:
            stop_all(listener, requester and requester.process)


STUBS = {
    "docker": """#!/bin/bash
printf 'docker %s\\n' "$*" >> "$FAKE_LOG"
if [ "$1" = exec ] && [ "$2" = -e ]; then
  printf 'listener-env %s\\n' "${ROSMASTER_SHUTDOWN_CONFIRM:-}" >> "$FAKE_LOG"
  exit "$FAKE_LISTENER_STATUS"
fi
exit 1
""",
    "systemctl": """#!/bin/bash
printf 'systemctl %s\\n' "$*" >> "$FAKE_LOG"
[ "$1" = stop ] && exit "$FAKE_STOP_STATUS"
exit 0
""",
    "logger": """#!/bin/bash
printf 'logger %s\\n' "$*" >> "$FAKE_LOG"
""",
    "hostname": """#!/bin/bash
echo testbot
""",
}


def run_host_script(tmp_path, listener_status, stop_status=0, confirm=None):
    """Run the host script with stub commands; return (exit status, logged calls)."""
    stub_dir = tmp_path / "bin"
    stub_dir.mkdir()
    for name, body in STUBS.items():
        stub = stub_dir / name
        stub.write_text(body, encoding="utf-8")
        stub.chmod(0o755)
    log = tmp_path / "calls.log"
    log.touch()
    env = {
        "PATH": "%s:/usr/bin:/bin" % stub_dir,
        "FAKE_LOG": str(log),
        "FAKE_LISTENER_STATUS": str(listener_status),
        "FAKE_STOP_STATUS": str(stop_status),
    }
    if confirm is not None:
        env["ROSMASTER_SHUTDOWN_CONFIRM"] = confirm
    result = subprocess.run(["bash", str(HOST_SCRIPT)], env=env, timeout=30)
    return result.returncode, log.read_text(encoding="utf-8").splitlines()


def systemctl_calls(calls):
    return [call for call in calls if call.startswith("systemctl ")]


def test_a_request_stops_the_platform_before_powering_off(tmp_path):
    status, calls = run_host_script(tmp_path, listener_status=42)
    assert status == 0
    assert systemctl_calls(calls) == [
        "systemctl stop rosmaster-platform.service",
        "systemctl poweroff",
    ]
    # Orphan sweep first, then the listener, and only then any systemctl call.
    assert calls[0] == "docker exec rosmaster_humble pkill -TERM -f ros_shutdown_listener.py"
    listener_call = calls.index("listener-env testbot")
    assert listener_call < calls.index("systemctl stop rosmaster-platform.service")
    assert any("Powering off the host" in call for call in calls)


def test_a_failed_platform_stop_is_logged_and_still_powers_off(tmp_path):
    status, calls = run_host_script(tmp_path, listener_status=42, stop_status=1)
    assert status == 0
    assert systemctl_calls(calls)[-1] == "systemctl poweroff"
    assert any(call.startswith("logger -p user.err") and "failed with status 1" in call for call in calls)


@pytest.mark.parametrize("listener_status", [0, 1, 2, 125, 137])
def test_no_request_means_no_stop_and_no_poweroff(tmp_path, listener_status):
    """Container gone, listener killed or crashed: systemd restarts it, nothing else."""
    status, calls = run_host_script(tmp_path, listener_status=listener_status)
    assert status == 1
    assert systemctl_calls(calls) == []
    assert any("status %d and no shutdown request" % listener_status in call for call in calls)


def test_platform_env_overrides_the_hostname_without_exposing_it(tmp_path):
    status, calls = run_host_script(tmp_path, listener_status=0, confirm="not-the-hostname")
    assert "listener-env not-the-hostname" in calls
    docker_calls = [call for call in calls if call.startswith("docker ")]
    assert docker_calls and not any("not-the-hostname" in call for call in docker_calls)
