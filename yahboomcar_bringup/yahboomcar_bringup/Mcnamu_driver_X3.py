#!/usr/bin/env python3
"""ROS 2 hardware driver for the Yahboom ROSMASTER X3 base."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Bool, Float32, Int32

from Rosmaster_Lib import Rosmaster

from .x3_driver_utils import compute_encoder_deltas
from .x3_driver_utils import map_encoder_counts
from .x3_driver_utils import MotionSafetyController
from .x3_driver_utils import validate_encoder_config


class YahboomCarDriver(Node):
    """Bridge ROS commands and telemetry to the X3 motor controller."""

    def __init__(self, name: str) -> None:
        """Initialize controller I/O, safety state, and ROS interfaces."""
        super().__init__(name)

        self.declare_parameter("car_type", "X3")
        self.declare_parameter("imu_link", "imu_link")
        self.declare_parameter("Prefix", "")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("xlinear_limit", 1.0)
        self.declare_parameter("ylinear_limit", 1.0)
        self.declare_parameter("angular_limit", 5.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("encoder_cpr", 1040.0)
        self.declare_parameter("encoder_order", [0, 2, 1, 3])
        self.declare_parameter("encoder_signs", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("encoder_max_delta_ticks", 5000)

        self.car_type = str(self.get_parameter("car_type").value)
        self.imu_link = str(self.get_parameter("imu_link").value)
        self.prefix = str(self.get_parameter("Prefix").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.xlinear_limit = float(self.get_parameter("xlinear_limit").value)
        self.ylinear_limit = float(self.get_parameter("ylinear_limit").value)
        self.angular_limit = float(self.get_parameter("angular_limit").value)
        self.cmd_vel_timeout = float(
            self.get_parameter("cmd_vel_timeout").value
        )
        self.encoder_cpr = float(self.get_parameter("encoder_cpr").value)
        self.encoder_order = list(self.get_parameter("encoder_order").value)
        self.encoder_signs = list(self.get_parameter("encoder_signs").value)
        self.encoder_max_delta_ticks = int(
            self.get_parameter("encoder_max_delta_ticks").value
        )

        if self.encoder_cpr <= 0.0:
            raise ValueError("encoder_cpr must be positive")
        if self.encoder_max_delta_ticks <= 0:
            raise ValueError("encoder_max_delta_ticks must be positive")
        if self.cmd_vel_timeout <= 0.0:
            raise ValueError("cmd_vel_timeout must be positive")
        validate_encoder_config(self.encoder_order, self.encoder_signs)

        self.car = Rosmaster(com=self.serial_port)
        self.car.set_car_type(1)
        self.car.create_receive_threading()

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.create_subscription(
            Int32, "RGBLight", self.rgb_light_callback, 100
        )
        self.create_subscription(Bool, "Buzzer", self.buzzer_callback, 100)

        self.edition_publisher = self.create_publisher(Float32, "edition", 100)
        self.voltage_publisher = self.create_publisher(Float32, "voltage", 100)
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 100
        )
        self.velocity_publisher = self.create_publisher(Twist, "vel_raw", 50)
        self.imu_publisher = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.magnetic_field_publisher = self.create_publisher(
            MagneticField, "/imu/mag", 100
        )

        self.previous_encoders: Optional[tuple[int, int, int, int]] = None
        self.previous_encoder_time = None
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.encoder_fault_active = False

        self.motion_safety = MotionSafetyController(
            self.set_motion,
            self.xlinear_limit,
            self.ylinear_limit,
            self.angular_limit,
            self.cmd_vel_timeout,
        )
        self.stop_motion(repeat=3)

        self.data_timer = self.create_timer(0.1, self.publish_data)
        self.watchdog_timer = self.create_timer(
            0.05, self.check_cmd_vel_watchdog
        )

        self.get_logger().info(
            "X3 driver ready: serial_port=%s, cmd_vel_timeout=%.3fs, "
            "encoder_order=%s, encoder_signs=%s"
            % (
                self.serial_port,
                self.cmd_vel_timeout,
                self.encoder_order,
                self.encoder_signs,
            )
        )

    def set_motion(self, vx: float, vy: float, wz: float) -> None:
        """Send a motion command to the controller."""
        self.car.set_car_motion(vx, vy, wz)

    def stop_motion(self, repeat: int = 1) -> None:
        """Send redundant zero commands and mark the driver stopped."""
        self.motion_safety.stop(repeat=repeat)

    def cmd_vel_callback(self, message: Twist) -> None:
        """Clamp and forward a ROS velocity command to the controller."""
        self.motion_safety.command(
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self.get_clock().now().nanoseconds / 1e9,
        )

    def check_cmd_vel_watchdog(self) -> None:
        """Stop persistent motion when command updates cease."""
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_seconds - self.motion_safety.last_command_time
        if self.motion_safety.enforce_timeout(now_seconds):
            self.get_logger().warning(
                "cmd_vel timeout after %.3fs; commanding zero velocity"
                % elapsed
            )

    def rgb_light_callback(self, message: Int32) -> None:
        """Set the expansion-board RGB effect."""
        for _ in range(3):
            self.car.set_colorful_effect(message.data, 6, parm=1)

    def buzzer_callback(self, message: Bool) -> None:
        """Set the expansion-board buzzer state."""
        for _ in range(3):
            self.car.set_beep(1 if message.data else 0)

    def make_joint_state(self, now) -> Optional[JointState]:
        """Read encoders and return a normalized four-wheel joint state."""
        state = JointState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = "joint_states"
        base_names = [
            "front_left_joint",
            "front_right_joint",
            "back_left_joint",
            "back_right_joint",
        ]
        state.name = [self.prefix + name for name in base_names]

        try:
            raw_encoders = self.car.get_motor_encoder()
            current_encoders = map_encoder_counts(
                raw_encoders, self.encoder_order, self.encoder_signs
            )
        except Exception as exc:
            if not self.encoder_fault_active:
                self.get_logger().error("Encoder read failed: %s" % exc)
                self.encoder_fault_active = True
            return None

        if self.encoder_fault_active:
            self.get_logger().info("Encoder feedback recovered")
            self.encoder_fault_active = False

        velocities = [0.0, 0.0, 0.0, 0.0]
        if (
            self.previous_encoders is not None
            and self.previous_encoder_time is not None
        ):
            elapsed = (now - self.previous_encoder_time).nanoseconds / 1e9
            deltas = compute_encoder_deltas(
                current_encoders,
                self.previous_encoders,
                self.encoder_max_delta_ticks,
            )
            if deltas is None:
                self.get_logger().warning(
                    "Rejected implausible encoder jump; "
                    "rebasing wheel counters"
                )
            elif elapsed > 0.0:
                radians_per_tick = 2.0 * math.pi / self.encoder_cpr
                for index, delta_ticks in enumerate(deltas):
                    delta_radians = delta_ticks * radians_per_tick
                    self.joint_positions[index] += delta_radians
                    velocities[index] = delta_radians / elapsed

        self.previous_encoders = current_encoders
        self.previous_encoder_time = now
        state.position = list(self.joint_positions)
        state.velocity = velocities
        return state

    def publish_data(self) -> None:
        """Publish encoder, IMU, magnetic, battery, and firmware telemetry."""
        now = self.get_clock().now()
        joint_state = self.make_joint_state(now)
        if joint_state is not None:
            self.joint_state_publisher.publish(joint_state)

        try:
            edition = Float32(data=float(self.car.get_version()))
            battery = Float32(data=float(self.car.get_battery_voltage()))
            ax, ay, az = self.car.get_accelerometer_data()
            gx, gy, gz = self.car.get_gyroscope_data()
            mx, my, mz = self.car.get_magnetometer_data()
            vx, vy, angular = self.car.get_motion_data()
        except Exception as exc:
            self.get_logger().error(
                "Controller telemetry read failed: %s" % exc
            )
            return

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)

        magnetic_field = MagneticField()
        magnetic_field.header.stamp = now.to_msg()
        magnetic_field.header.frame_id = self.imu_link
        magnetic_field.magnetic_field.x = float(mx)
        magnetic_field.magnetic_field.y = float(my)
        magnetic_field.magnetic_field.z = float(mz)

        velocity = Twist()
        velocity.linear.x = float(vx)
        velocity.linear.y = float(vy)
        velocity.angular.z = float(angular)

        self.velocity_publisher.publish(velocity)
        self.imu_publisher.publish(imu)
        self.magnetic_field_publisher.publish(magnetic_field)
        self.voltage_publisher.publish(battery)
        self.edition_publisher.publish(edition)


def main() -> None:
    """Run the X3 driver and guarantee a stop attempt during shutdown."""
    rclpy.init()
    driver: Optional[YahboomCarDriver] = None
    try:
        driver = YahboomCarDriver("driver_node")
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if driver is not None:
            driver.stop_motion(repeat=3)
            driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
