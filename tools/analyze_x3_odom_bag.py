#!/usr/bin/env python3
"""Summarize bounded X3 motion trials from a ROS 2 validation bag."""

from __future__ import annotations

import argparse
import bisect
import math
from pathlib import Path
import re
from typing import Any, NamedTuple, Sequence

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NANOSECONDS_PER_SECOND = 1_000_000_000
WHEEL_NAMES = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
)
PULSE_LOG_PATTERN = re.compile(
    r"Starting bounded pulse: "
    r"x=(?P<x>[+-]?\d+(?:\.\d+)?) "
    r"y=(?P<y>[+-]?\d+(?:\.\d+)?) "
    r"yaw=(?P<yaw>[+-]?\d+(?:\.\d+)?) "
    r"duration=(?P<duration>\d+(?:\.\d+)?)s"
)


class StampedMessage(NamedTuple):
    """A deserialized ROS message with its bag timestamp."""

    timestamp_ns: int
    message: Any


class CommandWindow(NamedTuple):
    """A contiguous interval containing one nonzero velocity command."""

    start_ns: int
    end_ns: int
    x: float
    y: float
    yaw: float


def parse_args() -> argparse.Namespace:
    """Parse bag path and settling-time arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=1.0,
        help="Time after the final nonzero command used for settled state",
    )
    return parser.parse_args()


def read_topics(bag_path: Path) -> dict[str, list[StampedMessage]]:
    """Deserialize only the topics used in the validation summary."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_names = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    wanted = {
        "/cmd_vel",
        "/joint_states",
        "/odom",
        "/rosout",
        "/vel_raw",
        "/voltage",
    }
    message_types = {
        name: get_message(type_name)
        for name, type_name in type_names.items()
        if name in wanted
    }
    messages: dict[str, list[StampedMessage]] = {
        name: [] for name in message_types
    }

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        messages[topic].append(StampedMessage(timestamp_ns, message))
    return messages


def is_nonzero_twist(message: Any) -> bool:
    """Return whether a Twist contains a commanded planar movement."""
    return any(
        abs(value) > 1e-9
        for value in (
            message.linear.x,
            message.linear.y,
            message.angular.z,
        )
    )


def command_windows(commands: Sequence[StampedMessage]) -> list[CommandWindow]:
    """Convert recorded Twist samples into contiguous nonzero windows."""
    windows: list[CommandWindow] = []
    start_ns = None
    last_nonzero_ns = None
    command = (0.0, 0.0, 0.0)

    for stamped in commands:
        if is_nonzero_twist(stamped.message):
            current = (
                float(stamped.message.linear.x),
                float(stamped.message.linear.y),
                float(stamped.message.angular.z),
            )
            if start_ns is None:
                start_ns = stamped.timestamp_ns
                command = current
            elif current != command:
                windows.append(
                    CommandWindow(
                        start_ns, last_nonzero_ns, *command
                    )
                )
                start_ns = stamped.timestamp_ns
                command = current
            last_nonzero_ns = stamped.timestamp_ns
        elif start_ns is not None:
            windows.append(
                CommandWindow(start_ns, last_nonzero_ns, *command)
            )
            start_ns = None
            last_nonzero_ns = None

    if start_ns is not None and last_nonzero_ns is not None:
        windows.append(CommandWindow(start_ns, last_nonzero_ns, *command))
    return windows


def command_windows_from_logs(
    logs: Sequence[StampedMessage],
) -> list[CommandWindow]:
    """Recover safety-tool windows when rosbag missed transient commands."""
    windows = []
    for stamped in logs:
        if stamped.message.name != "safe_cmd_vel_pulse":
            continue
        match = PULSE_LOG_PATTERN.search(stamped.message.msg)
        if match is None:
            continue
        duration_ns = int(
            float(match.group("duration")) * NANOSECONDS_PER_SECOND
        )
        windows.append(
            CommandWindow(
                stamped.timestamp_ns,
                stamped.timestamp_ns + duration_ns,
                float(match.group("x")),
                float(match.group("y")),
                float(match.group("yaw")),
            )
        )
    return windows


def nearest_at_or_before(
    messages: Sequence[StampedMessage], timestamp_ns: int
) -> StampedMessage:
    """Return the nearest message at or before a timestamp."""
    timestamps = [stamped.timestamp_ns for stamped in messages]
    index = max(0, bisect.bisect_right(timestamps, timestamp_ns) - 1)
    return messages[index]


def nearest_at_or_after(
    messages: Sequence[StampedMessage], timestamp_ns: int
) -> StampedMessage:
    """Return the nearest message at or after a timestamp."""
    timestamps = [stamped.timestamp_ns for stamped in messages]
    index = min(
        len(messages) - 1,
        bisect.bisect_left(timestamps, timestamp_ns),
    )
    return messages[index]


def wheel_positions(message: Any) -> tuple[float, float, float, float]:
    """Return FL, FR, BL, BR positions from a JointState message."""
    positions = dict(zip(message.name, message.position))
    return tuple(float(positions[name]) for name in WHEEL_NAMES)


def yaw_from_odometry(message: Any) -> float:
    """Return planar yaw from an Odometry pose quaternion."""
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def values_between(
    messages: Sequence[StampedMessage], start_ns: int, end_ns: int
) -> list[Any]:
    """Return message values whose bag timestamps fall in an interval."""
    return [
        stamped.message
        for stamped in messages
        if start_ns <= stamped.timestamp_ns <= end_ns
    ]


def format_vector(values: Sequence[float]) -> str:
    """Format a compact vector with explicit signs."""
    return "[" + ", ".join(f"{value:+.4f}" for value in values) + "]"


def summarize(
    messages: dict[str, list[StampedMessage]], settle_seconds: float
) -> None:
    """Print one deterministic summary for each recorded command window."""
    required = {"/joint_states", "/odom"}
    missing = sorted(required - messages.keys())
    if missing:
        raise RuntimeError(f"bag is missing required topics: {missing}")

    windows = command_windows(messages.get("/cmd_vel", []))
    window_source = "/cmd_vel"
    if not windows:
        windows = command_windows_from_logs(messages.get("/rosout", []))
        window_source = "/rosout safety-tool fallback"
    print(f"command_window_source: {window_source}")
    print(f"command_windows: {len(windows)}")
    settle_ns = int(settle_seconds * NANOSECONDS_PER_SECOND)

    for index, window in enumerate(windows, start=1):
        start_joint = nearest_at_or_before(
            messages["/joint_states"], window.start_ns
        )
        end_joint = nearest_at_or_after(
            messages["/joint_states"], window.end_ns + settle_ns
        )
        start_positions = wheel_positions(start_joint.message)
        end_positions = wheel_positions(end_joint.message)
        wheel_deltas = tuple(
            end - start
            for start, end in zip(start_positions, end_positions)
        )

        start_odom = nearest_at_or_before(
            messages["/odom"], window.start_ns
        ).message
        end_odom = nearest_at_or_after(
            messages["/odom"], window.end_ns + settle_ns
        ).message
        odom_delta = (
            end_odom.pose.pose.position.x
            - start_odom.pose.pose.position.x,
            end_odom.pose.pose.position.y
            - start_odom.pose.pose.position.y,
            yaw_from_odometry(end_odom) - yaw_from_odometry(start_odom),
        )

        velocities = values_between(
            messages.get("/vel_raw", []),
            window.start_ns,
            window.end_ns + settle_ns,
        )
        peak_velocity = tuple(
            max(
                (abs(value) for value in axis_values),
                default=0.0,
            )
            for axis_values in (
                [message.linear.x for message in velocities],
                [message.linear.y for message in velocities],
                [message.angular.z for message in velocities],
            )
        )
        voltages = values_between(
            messages.get("/voltage", []),
            window.start_ns,
            window.end_ns + settle_ns,
        )
        voltage_values = [float(message.data) for message in voltages]
        voltage_range = (
            min(voltage_values, default=float("nan")),
            max(voltage_values, default=float("nan")),
        )
        duration = (
            window.end_ns - window.start_ns
        ) / NANOSECONDS_PER_SECOND

        print(f"trial_{index}:")
        print(
            "  command_xyz: "
            + format_vector((window.x, window.y, window.yaw))
        )
        print(f"  recorded_nonzero_duration_s: {duration:.3f}")
        print(f"  wheel_delta_rad_fl_fr_bl_br: {format_vector(wheel_deltas)}")
        print(f"  odom_delta_x_y_yaw: {format_vector(odom_delta)}")
        print(f"  peak_abs_vel_raw_x_y_yaw: {format_vector(peak_velocity)}")
        print(
            "  voltage_min_max: "
            + format_vector(voltage_range)
        )


def main() -> int:
    """Read the requested bag and print its bounded-trial summary."""
    args = parse_args()
    if args.settle_seconds < 0.0:
        raise ValueError("--settle-seconds must be nonnegative")
    summarize(read_topics(args.bag), args.settle_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
