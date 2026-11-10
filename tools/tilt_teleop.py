#!/usr/bin/env python3
"""Drive /cmd_vel from a handheld tilt controller sending UDP packets.

Run on the robot, after the platform is up, with a person at main power:
    python3 tools/tilt_teleop.py --ros-args -p robot_id:=4
"""

from __future__ import annotations

import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tilt_teleop_core import Limits, TiltGate

# Zero commands published after motion ends, then the node goes quiet and
# the driver's own 0.5 s timeout keeps the robot stopped.
STOP_REPEATS = 5


class TiltTeleop(Node):
    """Receive controller packets and publish bounded velocity commands."""

    def __init__(self):
        """Declare parameters, open the UDP socket and start the timer."""
        super().__init__("tilt_teleop")
        port = self.declare_parameter("port", 9870).value
        robot_id = self.declare_parameter("robot_id", 0).value
        timeout_s = self.declare_parameter("timeout_s", 0.3).value
        rate_hz = self.declare_parameter("publish_rate_hz", 20.0).value
        limits = Limits(
            deadzone_deg=self.declare_parameter("deadzone_deg", 6.0).value,
            full_scale_deg=self.declare_parameter("full_scale_deg", 30.0).value,
            min_linear=self.declare_parameter("min_linear", 0.10).value,
            max_linear=self.declare_parameter("max_linear", 0.35).value,
            min_angular=self.declare_parameter("min_angular", 0.6).value,
            max_angular=self.declare_parameter("max_angular", 1.0).value,
        )
        self.allowed_ip = self.declare_parameter("allowed_ip", "").value
        self.gate = TiltGate(robot_id, timeout_s, limits)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.stop_ticks_left = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0", port))
        self.rate_hz = rate_hz
        self.get_logger().info(
            "listening on UDP %d for robot_id %d" % (port, robot_id)
        )

    def start(self):
        """Begin publishing; called only after the single-publisher check."""
        self.create_timer(1.0 / self.rate_hz, self.tick)

    def other_publishers(self) -> int:
        """Count /cmd_vel publishers other than this node."""
        infos = self.get_publishers_info_by_topic(self.publisher.topic_name)
        return sum(1 for info in infos if info.node_name != self.get_name())

    def drain_socket(self):
        """Feed every pending datagram to the gate."""
        while True:
            try:
                data, (ip, _port) = self.sock.recvfrom(64)
            except BlockingIOError:
                return
            if self.allowed_ip and ip != self.allowed_ip:
                continue
            self.gate.feed(data, time.monotonic())

    def tick(self):
        """Publish the current command, or zeros for a few ticks after it ends."""
        self.drain_socket()
        x, y, z = self.gate.command(time.monotonic())
        moving = bool(x or y or z)
        if moving:
            self.stop_ticks_left = STOP_REPEATS
        elif self.stop_ticks_left > 0:
            self.stop_ticks_left -= 1
        else:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, z
        self.publisher.publish(msg)


def main():
    """Refuse to start if another node already publishes /cmd_vel."""
    rclpy.init()
    node = TiltTeleop()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.other_publishers():
            node.get_logger().error("another node publishes cmd_vel; exiting")
            return 1
        node.start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
