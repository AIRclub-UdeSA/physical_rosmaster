#!/usr/bin/env python3
"""Wait for a confirmed /system/shutdown request, then exit for the host to act.

Runs inside the robot's container, started by the host's
rosmaster-shutdown-listener.service (config/rosmaster-shutdown-listen.sh). It
never powers anything off itself: the container cannot, and the host must stop
the platform first so the driver sends its zero commands. Its only output is
its exit status. EXIT_SHUTDOWN_REQUESTED tells the host script that a request
carrying this robot's confirmation value arrived; any other status means none
did, and the host only restarts the listener.

The confirmation value (the robot's hostname unless platform.env overrides it)
keeps a stray message, or one sent on the wrong ROS_DOMAIN_ID, from powering
off a robot nobody meant to stop. It is not authentication: anything on the
robot's domain that knows the value can send it.
"""

from __future__ import annotations

import argparse
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


TOPIC = "/system/shutdown"
CONFIRM_ENVIRONMENT_VARIABLE = "ROSMASTER_SHUTDOWN_CONFIRM"
EXIT_SHUTDOWN_REQUESTED = 42
EXIT_NO_REQUEST = 0
# Received values are logged for diagnosis; cap them so one oversized message
# cannot flood the journal.
LOGGED_VALUE_LIMIT = 64

# Volatile, deliberately: a volatile reader never receives a transient-local
# publisher's stored sample, so a latched request published before this
# process started (for example, before the robot rebooted) cannot fire again.
SHUTDOWN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def is_confirmed_request(data: str, expected: str) -> bool:
    """Return whether a request carries this robot's confirmation value."""
    expected = expected.strip()
    return bool(expected) and data.strip() == expected


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the confirmation value; the environment supplies it on the robot."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--confirm",
        default=os.environ.get(CONFIRM_ENVIRONMENT_VARIABLE, ""),
        help="value a request must carry (default: $%s)"
        % CONFIRM_ENVIRONMENT_VARIABLE,
    )
    args = parser.parse_args(argv)
    if not args.confirm.strip():
        parser.error(
            "no confirmation value: set %s or pass --confirm"
            % CONFIRM_ENVIRONMENT_VARIABLE
        )
    return args


class ShutdownListener(Node):
    """Record the first confirmed request; ignore and log everything else."""

    def __init__(self, expected: str) -> None:
        super().__init__("system_shutdown_listener")
        self.expected = expected
        self.requested = False
        self.create_subscription(String, TOPIC, self.on_request, SHUTDOWN_QOS)
        self.get_logger().info(
            "Listening on %s for this robot's confirmation value" % TOPIC
        )

    def on_request(self, msg: String) -> None:
        received = repr(msg.data[:LOGGED_VALUE_LIMIT])
        if is_confirmed_request(msg.data, self.expected):
            self.get_logger().warning(
                "Confirmed shutdown request on %s (%s); handing over to the "
                "host to stop the platform and power off" % (TOPIC, received)
            )
            self.requested = True
            return
        # The expected value is not logged: platform.env may set it to
        # something meant to stay unguessable for an event.
        self.get_logger().warning(
            "Ignored %s request %s: it does not match this robot's "
            "confirmation value" % (TOPIC, received)
        )


def main(argv: list[str] | None = None) -> int:
    """Spin until a confirmed request arrives or the process is stopped."""
    args = parse_args(argv)
    rclpy.init()
    node = None
    try:
        node = ShutdownListener(args.confirm)
        while rclpy.ok() and not node.requested:
            rclpy.spin_once(node, timeout_sec=0.5)
        return EXIT_SHUTDOWN_REQUESTED if node.requested else EXIT_NO_REQUEST
    except (KeyboardInterrupt, ExternalShutdownException):
        return EXIT_NO_REQUEST
    finally:
        if node is not None:
            node.destroy_node()
        # A signal handler can shut the context down between the check and
        # the call; that must not turn a clean stop into a traceback.
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
