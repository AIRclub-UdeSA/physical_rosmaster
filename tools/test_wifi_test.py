"""Hardware-free checks for the parsers in tools/wifi_test."""

import importlib.util
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import pytest

WIFI_TEST = Path(__file__).resolve().parent / "wifi_test"
EVIDENCE = (
    Path(__file__).resolve().parent.parent
    / "robot_artifacts" / "raphael_router_wifi_2026-10-16"
)


def load(name):
    spec = importlib.util.spec_from_file_location(name, WIFI_TEST / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compare_bags = load("compare_bags")
probe_summary = load("probe_summary")


def test_parse_counts_reads_ros2_bag_info_output():
    text = (EVIDENCE / "robot_bag_heavy.txt").read_text()
    counts = compare_bags.parse_counts(text)
    assert counts["/cam_1/color/image_raw"] == 1748
    assert counts["/tf_static"] == 2
    assert len(counts) == 9


def test_compare_flags_only_graded_topics_below_threshold():
    robot = {"/odom": 600, "/scan": 430, "/tf_static": 2, "/voltage": 600}
    workstation = {"/odom": 590, "/scan": 300, "/tf_static": 1}
    rows = {row[0]: row for row in compare_bags.compare(robot, workstation)}
    assert not rows["/odom"][4]
    assert rows["/scan"][4]
    assert not rows["/tf_static"][4]
    assert rows["/voltage"][2:] == (0, 0.0, True)


def test_compare_handles_topic_missing_on_robot():
    rows = list(compare_bags.compare({}, {"/odom": 5}))
    assert rows == [("/odom", 0, 5, None, False)]


def test_recorded_heavy_bag_delivered_every_topic():
    robot = compare_bags.parse_counts((EVIDENCE / "robot_bag_heavy.txt").read_text())
    workstation = compare_bags.parse_counts(
        (EVIDENCE / "workstation_bag_heavy.txt").read_text()
    )
    assert not any(row[4] for row in compare_bags.compare(robot, workstation))


def test_summary_rows_read_probe_json():
    result = json.loads((EVIDENCE / "probe_E.json").read_text())
    rows = {row[0]: row for row in probe_summary.summary_rows(result)}
    topic, rate, median, p95, drops = rows["/cam_1/color/image_raw"]
    assert 17 < rate < 19
    assert median > 100
    assert drops == 0


def test_summary_rows_tolerate_missing_fields():
    rows = probe_summary.summary_rows({"topics": [{"topic": "/x"}]})
    assert rows == [("/x", 0.0, 0.0, 0.0, None)]


def load_cmd_latency():
    pytest.importorskip("rclpy", reason="ROS 2 Python runtime unavailable: rclpy")
    return load("cmd_latency")


def test_cmd_latency_requires_the_wheels_lifted_confirmation():
    cmd_latency = load_cmd_latency()
    with pytest.raises(SystemExit):
        cmd_latency.parse_args([])
    args = cmd_latency.parse_args(["--confirm-wheels-lifted"])
    assert (args.pulses, args.speed) == (5, 0.10)


@pytest.mark.parametrize("speed", ["0", "-0.1", "0.21", "nan", "inf"])
def test_cmd_latency_rejects_speeds_outside_the_limit(speed):
    cmd_latency = load_cmd_latency()
    with pytest.raises(SystemExit):
        cmd_latency.parse_args(["--confirm-wheels-lifted", "--speed", speed])


def test_cmd_latency_accepts_the_speed_limit_itself():
    cmd_latency = load_cmd_latency()
    args = cmd_latency.parse_args(["--confirm-wheels-lifted", "--speed", "0.2"])
    assert args.speed == cmd_latency.MAX_SPEED


@pytest.mark.parametrize("pulses", ["0", "-1"])
def test_cmd_latency_rejects_no_pulses(pulses):
    cmd_latency = load_cmd_latency()
    with pytest.raises(SystemExit):
        cmd_latency.parse_args(["--confirm-wheels-lifted", "--pulses", pulses])


def test_odom_age_is_infinite_before_the_first_message():
    cmd_latency = load_cmd_latency()
    assert cmd_latency.odom_age(None, 5.0) == float("inf")
    assert cmd_latency.odom_age(4.75, 5.0) == pytest.approx(0.25)


# A stand-in robot: answers /cmd_vel with /odom at 10 Hz, vx equal to the
# commanded speed. "freeze" stops /odom at the first motion command, "gap" stops
# it between pulses, "silent" publishes for one second and then stops, "deaf"
# never moves and "runaway" ignores zeros. Prints MOVING at the first motion
# command and, on request, the zeros received since the last one.
FAKE_ROBOT = r"""
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

mode = sys.argv[1]
# Pulse 1 ends 3.5 s after its first command and pulse 2 starts 1 s later.
GAP_AFTER_S = 3.5
rclpy.init()
node = rclpy.create_node("driver_node")
odom = node.create_publisher(Odometry, "/odom", qos_profile_sensor_data)
state = {"vx": 0.0, "frozen": False, "moved_at": None, "zeros": 0}
started = time.monotonic()
report_requested = threading.Event()


def on_cmd(msg):
    if msg.linear.x:
        if state["moved_at"] is None:
            state["moved_at"] = time.monotonic()
            print("MOVING", flush=True)
        state["zeros"] = 0
    elif state["moved_at"] is not None:
        state["zeros"] += 1
    if mode == "deaf" or (mode == "runaway" and not msg.linear.x):
        return
    state["vx"] = msg.linear.x
    if mode == "freeze" and msg.linear.x:
        state["frozen"] = True


def tick():
    if state["frozen"] or (mode == "silent" and time.monotonic() - started > 1.0):
        return
    moved_at = state["moved_at"]
    if mode == "gap" and moved_at and time.monotonic() - moved_at > GAP_AFTER_S:
        return
    message = Odometry()
    message.twist.twist.linear.x = state["vx"]
    odom.publish(message)


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
node.create_timer(0.1, tick)
threading.Thread(
    target=lambda: (sys.stdin.readline(), report_requested.set()), daemon=True
).start()
deadline = time.monotonic() + 40.0
while not report_requested.is_set() and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.02)
for _ in range(25):
    rclpy.spin_once(node, timeout_sec=0.02)
print("ZEROS %d" % state["zeros"], flush=True)
"""


def run_cmd_latency(mode, pulses=1):
    """Run cmd_latency.py to the end against the fake robot."""
    load_cmd_latency()
    from test_motor_live_loss_ros_smoke import isolated_ros_domain
    from test_safe_cmd_vel_pulse import ros_env

    with isolated_ros_domain() as domain:
        env = ros_env(domain)
        robot = subprocess.Popen(
            [sys.executable, "-c", FAKE_ROBOT, mode],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, env=env,
        )
        try:
            return subprocess.run(
                [sys.executable, str(WIFI_TEST / "cmd_latency.py"),
                 "--confirm-wheels-lifted", "--pulses", str(pulses)],
                capture_output=True, text=True, env=env, timeout=45,
            )
        finally:
            robot.kill()
            robot.wait()


def test_cmd_latency_times_a_pulse_with_fresh_odom():
    result = run_cmd_latency("normal")
    assert result.returncode == 0, result.stderr
    assert "pulse 1: start" in result.stdout, result.stdout
    assert "DID NOT" not in result.stdout, result.stdout
    assert "counted 1 of 1 pulses (0 stale)" in result.stdout, result.stdout
    assert "start: median" in result.stdout and "stop: median" in result.stdout


def test_cmd_latency_stops_at_the_first_stale_pulse():
    result = run_cmd_latency("freeze", pulses=3)
    assert result.returncode == 1, result.stdout
    assert "pulse 1: /odom STALE" in result.stdout, result.stdout
    assert "pulse 2" not in result.stdout, result.stdout
    assert "DID NOT" not in result.stdout, result.stdout
    assert "counted 0 of 3 pulses (1 stale)" in result.stdout, result.stdout
    assert "median" not in result.stdout, result.stdout


def test_cmd_latency_does_not_send_a_pulse_after_odom_goes_stale_between_pulses():
    result = run_cmd_latency("gap", pulses=3)
    assert result.returncode == 1, result.stdout
    assert "pulse 1: start" in result.stdout, result.stdout
    assert "pulse 2: /odom STALE" in result.stdout, result.stdout
    assert "not sent" in result.stdout, result.stdout
    assert "counted 1 of 3 pulses (1 stale)" in result.stdout, result.stdout


def test_cmd_latency_stops_when_the_robot_does_not_stop():
    result = run_cmd_latency("runaway", pulses=3)
    assert result.returncode == 1, result.stdout
    assert "pulse 1: start" in result.stdout, result.stdout
    assert "DID NOT STOP" in result.stdout, result.stdout
    assert "pulse 2" not in result.stdout, result.stdout
    assert "counted 0 of 3 pulses (0 stale)" in result.stdout, result.stdout


def test_cmd_latency_fails_when_no_pulse_is_counted():
    result = run_cmd_latency("deaf")
    assert result.returncode == 1, result.stdout
    assert "DID NOT MOVE" in result.stdout, result.stdout
    assert "counted 0 of 1 pulses (0 stale)" in result.stdout, result.stdout


def test_cmd_latency_refuses_to_start_on_stale_odom():
    result = run_cmd_latency("silent")
    assert result.returncode == 1, result.stdout
    assert "/odom is stale" in result.stderr, result.stderr
    assert "pulse" not in result.stdout, result.stdout


# Lands the second signal inside the final zeros (20 ms apart).
SECOND_SIGNAL_DELAY = 0.02


def interrupt_cmd_latency(stop_signals, pulses, after_counted_pulse=False):
    """Signal cmd_latency.py during a pulse; return its result and the zeros."""
    load_cmd_latency()
    from test_motor_live_loss_ros_smoke import isolated_ros_domain
    from test_safe_cmd_vel_pulse import read_line_until, ros_env

    with isolated_ros_domain() as domain:
        env = ros_env(domain)
        robot = subprocess.Popen(
            [sys.executable, "-c", FAKE_ROBOT, "normal"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, env=env,
        )
        tool = None
        try:
            tool = subprocess.Popen(
                [sys.executable, str(WIFI_TEST / "cmd_latency.py"),
                 "--confirm-wheels-lifted", "--pulses", str(pulses)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env,
            )
            read_line_until(robot.stdout, "MOVING", time.monotonic() + 20)
            output = ""
            if after_counted_pulse:
                output = read_line_until(tool.stdout, "pulse 1:", time.monotonic() + 20)
                time.sleep(1.3)  # 1 s between pulses, then into pulse 2
            for index, stop_signal in enumerate(stop_signals):
                if index:
                    time.sleep(SECOND_SIGNAL_DELAY)
                tool.send_signal(stop_signal)
            stdout, stderr = tool.communicate(timeout=10)
            robot.stdin.write("report\n")
            robot.stdin.flush()
            report = read_line_until(robot.stdout, "ZEROS", time.monotonic() + 10)
            return tool.returncode, output + stdout, stderr, int(report.split()[1])
        finally:
            for process in (tool, robot):
                if process is not None and process.poll() is None:
                    process.kill()
                    process.wait()


@pytest.mark.parametrize(
    "stop_signals",
    [
        [signal.SIGINT],
        [signal.SIGTERM],
        [signal.SIGINT, signal.SIGINT],
        [signal.SIGTERM, signal.SIGINT],
        [signal.SIGINT, signal.SIGTERM],
        [signal.SIGTERM, signal.SIGTERM],
    ],
    ids=["INT", "TERM", "INT-then-INT", "TERM-then-INT", "INT-then-TERM", "TERM-then-TERM"],
)
def test_cmd_latency_sends_every_final_zero_when_stopped(stop_signals):
    status, stdout, stderr, zeros = interrupt_cmd_latency(stop_signals, pulses=1)
    assert status == 2, stderr
    assert zeros == 10, stderr
    assert "Traceback" not in stderr, stderr
    assert "counted 0 of 1 pulses (0 stale)" in stdout, stdout


def test_cmd_latency_keeps_the_partial_summary_when_stopped():
    status, stdout, stderr, zeros = interrupt_cmd_latency(
        [signal.SIGINT], pulses=3, after_counted_pulse=True
    )
    assert status == 2, stderr
    assert zeros == 10, stderr
    assert "counted 1 of 3 pulses (0 stale)" in stdout, stdout
    assert re.search(r"start: median \d+ ms", stdout), stdout
    assert re.search(r"stop: median \d+ ms", stdout), stdout
