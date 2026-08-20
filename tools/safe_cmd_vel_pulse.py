#!/usr/bin/env python3
"""Publish a bounded X3 velocity pulse and redundantly stop afterward."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MAX_LINEAR_SPEED = 0.30
MAX_ANGULAR_SPEED = 0.75
MAX_DURATION = 15.0
PUBLISH_RATE = 20.0


def finite_float(value: str) -> float:
    """Parse a finite floating-point command-line value."""
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("value must be finite")
    return parsed


def parse_args() -> argparse.Namespace:
    """Parse a deliberately small, bounded motion request."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=finite_float, default=0.0)
    parser.add_argument("--y", type=finite_float, default=0.0)
    parser.add_argument("--yaw", type=finite_float, default=0.0)
    parser.add_argument("--duration", type=finite_float, required=True)
    parser.add_argument("--discovery-timeout", type=finite_float, default=3.0)
    args = parser.parse_args()

    if abs(args.x) > MAX_LINEAR_SPEED or abs(args.y) > MAX_LINEAR_SPEED:
        parser.error(
            "linear commands are limited to +/- %.2f m/s"
            % MAX_LINEAR_SPEED
        )
    if abs(args.yaw) > MAX_ANGULAR_SPEED:
        parser.error(
            "yaw commands are limited to +/- %.2f rad/s"
            % MAX_ANGULAR_SPEED
        )
    if not 0.0 < args.duration <= MAX_DURATION:
        parser.error(
            "duration must be greater than zero and at most %.1fs"
            % MAX_DURATION
        )
    if args.x == 0.0 and args.y == 0.0 and args.yaw == 0.0:
        parser.error("at least one commanded axis must be nonzero")
    if args.discovery_timeout <= 0.0:
        parser.error("discovery-timeout must be positive")
    return args


def publish_repeatedly(
    node: Node, publisher, message: Twist, count: int
) -> None:
    """Publish a message repeatedly while servicing discovery events."""
    for _ in range(count):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.02)


def validate_graph(node: Node, timeout: float) -> None:
    """Require one motor driver and no competing velocity publisher."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        subscribers = node.get_subscriptions_info_by_topic("/cmd_vel")
        actuator_subscribers = [
            endpoint
            for endpoint in subscribers
            if endpoint.node_name != "rosbag2_recorder"
        ]
        publishers = node.get_publishers_info_by_topic("/cmd_vel")
        other_publishers = [
            endpoint
            for endpoint in publishers
            if endpoint.node_name != node.get_name()
        ]
        if (
            len(actuator_subscribers) == 1
            and actuator_subscribers[0].node_name == "driver_node"
            and not other_publishers
        ):
            return

    subscribers = node.get_subscriptions_info_by_topic("/cmd_vel")
    actuator_names = sorted(
        endpoint.node_name
        for endpoint in subscribers
        if endpoint.node_name != "rosbag2_recorder"
    )
    publishers = node.get_publishers_info_by_topic("/cmd_vel")
    other_names = sorted(
        endpoint.node_name
        for endpoint in publishers
        if endpoint.node_name != node.get_name()
    )
    raise RuntimeError(
        "unsafe ROS graph: expected only one driver_node actuator and no "
        "other publishers; actuator_subscribers=%s other_publishers=%s"
        % (actuator_names, other_names)
    )


def main() -> int:
    """Run the pulse, always attempting multiple zero commands on exit."""
    args = parse_args()
    rclpy.init()
    node = Node("safe_cmd_vel_pulse")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    zero = Twist()

    try:
        validate_graph(node, args.discovery_timeout)
        publish_repeatedly(node, publisher, zero, 3)

        command = Twist()
        command.linear.x = args.x
        command.linear.y = args.y
        command.angular.z = args.yaw

        node.get_logger().warning(
            "Starting bounded pulse: x=%.3f y=%.3f yaw=%.3f duration=%.3fs"
            % (args.x, args.y, args.yaw, args.duration)
        )
        period = 1.0 / PUBLISH_RATE
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        return 0
    except (KeyboardInterrupt, RuntimeError) as exc:
        if str(exc):
            print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    finally:
        publish_repeatedly(node, publisher, zero, 5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
