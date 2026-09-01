#!/usr/bin/env python3
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

"""
Exercise strict launch shutdown around a synthetic motor-driver failure.

The helper has three process roles used by the real ROS/DDS smoke test:

* ``driver`` publishes a healthy platform graph, then fails on device loss;
* ``sentinel`` persistently owns one extra strict endpoint; and
* ``supervisor`` starts both through ROS launch and applies the repository's
  real strict-process exit handler to the driver.

No role creates a ``/cmd_vel`` publisher or sends an actuator command.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, MagneticField
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage


MOTOR_DIAGNOSTIC_NAME = (
    "yahboomcar_bringup: motor controller and onboard sensors"
)
WHEEL_NAMES = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
)


class SyntheticDriver(Node):
    """Own every strict endpoint until a synthetic controller disappears."""

    def __init__(
        self,
        device: Path,
        error_delay: float,
        exit_delay: float,
    ) -> None:
        super().__init__(
            "driver_node",
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.device = device
        self.error_delay = error_delay
        self.exit_delay = exit_delay
        self.loss_at = None
        self.done = False

        self.joint_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.velocity_publisher = self.create_publisher(Twist, "/vel_raw", 10)
        self.voltage_publisher = self.create_publisher(Float32, "/voltage", 10)
        self.edition_publisher = self.create_publisher(Float32, "/edition", 10)
        self.imu_raw_publisher = self.create_publisher(
            Imu, "/imu/data_raw", 10
        )
        self.magnetic_publisher = self.create_publisher(
            MagneticField, "/imu/mag", 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        self.strict_publishers = (
            (self.create_publisher(Odometry, "/odom", 10), Odometry()),
            (self.create_publisher(Imu, "/imu/data", 10), Imu()),
            (self.create_publisher(LaserScan, "/scan", 10), LaserScan()),
            (
                self.create_publisher(
                    Image, "/cam_1/color/image_raw", 10
                ),
                Image(),
            ),
            (
                self.create_publisher(
                    Image, "/cam_1/depth/image_raw", 10
                ),
                Image(),
            ),
            (
                self.create_publisher(
                    PointCloud2, "/cam_1/depth/color/points", 10
                ),
                PointCloud2(),
            ),
            (self.create_publisher(TFMessage, "/tf", 10), TFMessage()),
            (
                self.create_publisher(TFMessage, "/tf_static", 10),
                TFMessage(),
            ),
        )
        self.timer = self.create_timer(0.05, self.tick)

    @staticmethod
    def _diagnostic_values(failed: bool) -> list[KeyValue]:
        state = "failed" if failed else "healthy"
        reason = (
            "controller receive thread raised OSError: synthetic device loss"
            if failed
            else "all required controller report channels are fresh"
        )
        values = {
            "feedback_state": state,
            "feedback_reason": reason,
            "feedback_report_sequence": "42",
            "feedback_timeout_seconds": "0.500000",
        }
        for stream in ("speed", "encoder", "imu_raw"):
            values["feedback_%s_age_seconds" % stream] = (
                "0.600000" if failed else "0.020000"
            )
            values["feedback_%s_stale" % stream] = (
                "true" if failed else "false"
            )
        return [KeyValue(key=key, value=value) for key, value in values.items()]

    def publish_diagnostic(self, failed: bool) -> None:
        """Publish complete structured receive-path evidence."""
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = MOTOR_DIAGNOSTIC_NAME
        status.hardware_id = str(self.device)
        status.level = (
            DiagnosticStatus.ERROR if failed else DiagnosticStatus.OK
        )
        status.message = (
            "Terminal controller receive thread failure"
            if failed
            else "Fresh synthetic controller reports are healthy"
        )
        status.values = self._diagnostic_values(failed)
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def publish_baseline(self) -> None:
        """Publish stationary controller topics and all strict endpoints."""
        now = self.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = now
        joints.name = list(WHEEL_NAMES)
        joints.position = [0.0] * len(WHEEL_NAMES)
        joints.velocity = [0.0] * len(WHEEL_NAMES)
        self.joint_publisher.publish(joints)
        self.velocity_publisher.publish(Twist())
        self.voltage_publisher.publish(Float32(data=11.4))
        self.edition_publisher.publish(Float32(data=3.3))

        imu = Imu()
        imu.header.stamp = now
        self.imu_raw_publisher.publish(imu)
        magnetic = MagneticField()
        magnetic.header.stamp = now
        self.magnetic_publisher.publish(magnetic)
        for publisher, message in self.strict_publishers:
            publisher.publish(message)
        self.publish_diagnostic(failed=False)

    def tick(self) -> None:
        """Stop controller topics, diagnose loss, then end the process."""
        now = time.monotonic()
        if self.loss_at is None:
            if not os.path.exists(self.device):
                self.loss_at = now
                return
            self.publish_baseline()
            return

        elapsed = now - self.loss_at
        if elapsed >= self.error_delay:
            self.publish_diagnostic(failed=True)
        if elapsed >= self.exit_delay:
            self.done = True


class StrictEndpointSentinel(Node):
    """Persist until launch propagates driver exit to the whole graph."""

    def __init__(self) -> None:
        super().__init__(
            "strict_shutdown_sentinel",
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.timer = self.create_timer(0.05, self.publish)

    def publish(self) -> None:
        """Keep one strict endpoint live until this process is terminated."""
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def positive_float(value: str) -> float:
    """Parse a positive finite timing argument."""
    parsed = float(value)
    if not 0.0 < parsed < float("inf"):
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def add_driver_arguments(parser: argparse.ArgumentParser) -> None:
    """Add controls shared by the direct driver and launch supervisor."""
    parser.add_argument("--device", required=True, type=Path)
    parser.add_argument("--error-delay", type=positive_float, default=0.20)
    parser.add_argument("--exit-delay", type=positive_float, default=0.80)


def parse_args() -> argparse.Namespace:
    """Parse one of the synthetic launch-process roles."""
    parser = argparse.ArgumentParser(description=__doc__)
    roles = parser.add_subparsers(dest="role", required=True)

    driver = roles.add_parser("driver")
    add_driver_arguments(driver)
    driver.add_argument("--ready-file", required=True, type=Path)

    sentinel = roles.add_parser("sentinel")
    sentinel.add_argument("--ready-file", required=True, type=Path)
    sentinel.add_argument("--stopped-file", required=True, type=Path)

    supervisor = roles.add_parser("supervisor")
    add_driver_arguments(supervisor)
    supervisor.add_argument("--driver-ready-file", required=True, type=Path)
    supervisor.add_argument("--sentinel-ready-file", required=True, type=Path)
    supervisor.add_argument("--sentinel-stopped-file", required=True, type=Path)

    args = parser.parse_args()
    if args.role in {"driver", "supervisor"} and (
        args.error_delay >= args.exit_delay
    ):
        parser.error("error-delay must be less than exit-delay")
    return args


def run_driver(args: argparse.Namespace) -> int:
    """Run until the device disappears and failure evidence has been sent."""
    rclpy.init(args=[])
    node = SyntheticDriver(args.device, args.error_delay, args.exit_delay)
    try:
        args.ready_file.write_text("ready\n", encoding="utf-8")
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_sentinel(args: argparse.Namespace) -> int:
    """Publish until the launch service terminates this required peer."""
    rclpy.init(args=[])
    node = StrictEndpointSentinel()
    try:
        args.ready_file.write_text("ready\n", encoding="utf-8")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.10)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        args.stopped_file.write_text("stopped\n", encoding="utf-8")


def load_required_process():
    """Load the strict shutdown helper from the repository launch file."""
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "yahboomcar_bringup"
        / "launch"
        / "yahboomcar_bringup_X3_launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "physical_rosmaster_strict_launch", launch_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load strict launch module: %s" % launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._required_process


def run_supervisor(args: argparse.Namespace) -> int:
    """Use the real strict handler to propagate driver exit to its peer."""
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess

    helper_path = Path(__file__).resolve()
    driver = ExecuteProcess(
        cmd=[
            sys.executable,
            str(helper_path),
            "driver",
            "--device",
            str(args.device),
            "--ready-file",
            str(args.driver_ready_file),
            "--error-delay",
            str(args.error_delay),
            "--exit-delay",
            str(args.exit_delay),
        ],
        name="synthetic_motor_driver",
        output="screen",
        sigterm_timeout="1.0",
        sigkill_timeout="1.0",
    )
    sentinel = ExecuteProcess(
        cmd=[
            sys.executable,
            str(helper_path),
            "sentinel",
            "--ready-file",
            str(args.sentinel_ready_file),
            "--stopped-file",
            str(args.sentinel_stopped_file),
        ],
        name="strict_endpoint_sentinel",
        output="screen",
        sigterm_timeout="1.0",
        sigkill_timeout="1.0",
    )
    required_process = load_required_process()
    description = LaunchDescription(
        [
            driver,
            sentinel,
            required_process(driver, "synthetic motor driver"),
        ]
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(description)
    return service.run()


def main() -> int:
    """Dispatch the selected synthetic launch-process role."""
    args = parse_args()
    if args.role == "driver":
        return run_driver(args)
    if args.role == "sentinel":
        return run_sentinel(args)
    return run_supervisor(args)


if __name__ == "__main__":
    raise SystemExit(main())
