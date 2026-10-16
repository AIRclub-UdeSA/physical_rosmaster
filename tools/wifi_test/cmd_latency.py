"""Measure how long the robot takes to act on /cmd_vel sent from the workstation.

Sends short forward pulses and times, on the workstation clock, until /odom
shows motion, and from the zero command until /odom is still again. The time
includes WiFi both ways, the driver, motor stiction and the 10 Hz /odom period.

ONLY WITH THE WHEELS LIFTED. Refuses to start while another /cmd_vel publisher
exists.

Usage: python3 cmd_latency.py --confirm-wheels-lifted [--pulses 5] [--speed 0.10]
"""

import argparse
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

PULSE_S = 1.5
STOP_WINDOW_S = 2.0
PUBLISH_PERIOD_S = 0.05  # 20 Hz, well inside the driver's 0.5 s cmd_vel_timeout
MAX_SPEED = 0.2


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--confirm-wheels-lifted", action="store_true", required=True)
    parser.add_argument("--pulses", type=int, default=5)
    parser.add_argument("--speed", type=float, default=0.10)
    args = parser.parse_args(argv)
    if not 0 < args.speed <= MAX_SPEED:
        parser.error(f"--speed must be in (0, {MAX_SPEED}] m/s")
    return args


def main(argv=None):
    args = parse_args(argv)
    moving = 0.3 * args.speed  # /odom vx that counts as "moving"
    still = 0.2 * moving

    rclpy.init()
    node = rclpy.create_node("wifi_cmd_latency")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"vx": None}

    def on_odom(msg):
        state["vx"] = msg.twist.twist.linear.x

    node.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)

    spin_for(3.0)
    others = [
        info for info in node.get_publishers_info_by_topic("/cmd_vel")
        if info.node_name != node.get_name()
    ]
    if others:
        sys.exit(f">>> another /cmd_vel publisher exists ({others[0].node_name})")
    if state["vx"] is None:
        sys.exit(">>> no /odom reaches the workstation")

    forward, zero = Twist(), Twist()
    forward.linear.x = args.speed
    starts, stops = [], []
    try:
        for pulse in range(1, args.pulses + 1):
            began, started = time.monotonic(), None
            while time.monotonic() - began < PULSE_S:
                publisher.publish(forward)
                spin_for(PUBLISH_PERIOD_S)
                if started is None and abs(state["vx"]) > moving:
                    started = time.monotonic() - began
            zeroed, stopped = time.monotonic(), None
            while time.monotonic() - zeroed < STOP_WINDOW_S:
                publisher.publish(zero)
                spin_for(PUBLISH_PERIOD_S)
                if stopped is None and abs(state["vx"]) < still:
                    stopped = time.monotonic() - zeroed
            start_text = f"{started * 1000:5.0f} ms" if started is not None else "  DID NOT MOVE"
            stop_text = f"{stopped * 1000:5.0f} ms" if stopped is not None else "  DID NOT STOP"
            print(f"pulse {pulse}: start {start_text}   stop {stop_text}", flush=True)
            if started is not None:
                starts.append(started)
            if stopped is not None:
                stops.append(stopped)
            spin_for(1.0)
    finally:
        for _ in range(10):
            publisher.publish(zero)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()

    for name, values in (("start", starts), ("stop", stops)):
        if values:
            print(f"{name}: median {statistics.median(values) * 1000:.0f} ms,"
                  f" max {max(values) * 1000:.0f} ms")


if __name__ == "__main__":
    main()
