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

"""ROS 2 hardware driver for the Yahboom ROSMASTER X3 base."""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Bool, Float32, Int32

from .rosmaster_transport import create_verified_rosmaster_transport
from .rosmaster_transport import RosmasterTransport
from .rosmaster_transport import RosmasterTransportError
from .rosmaster_transport import TransportState
from .rosmaster_transport import TransportStatus
from .x3_driver_utils import compute_encoder_deltas
from .x3_driver_utils import map_encoder_counts
from .x3_driver_utils import MotionSafetyController
from .x3_driver_utils import validate_encoder_config


ControllerFactory = Callable[..., RosmasterTransport]


def _create_controller_transport(**kwargs) -> RosmasterTransport:
    """Load the robot-only vendor dependency and wrap its receive path."""
    return create_verified_rosmaster_transport(**kwargs)


class YahboomCarDriver(Node):
    """Bridge ROS commands and telemetry to the X3 motor controller."""

    def __init__(
        self,
        name: str,
        controller_factory: Optional[ControllerFactory] = None,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
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
        self.declare_parameter("feedback_startup_timeout", 2.0)
        self.declare_parameter("feedback_timeout", 0.5)
        self.declare_parameter("feedback_failure_exit_delay", 0.2)
        self.declare_parameter("serial_write_timeout", 0.05)

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
        self.feedback_startup_timeout = float(
            self.get_parameter("feedback_startup_timeout").value
        )
        self.feedback_timeout = float(
            self.get_parameter("feedback_timeout").value
        )
        self.feedback_failure_exit_delay = float(
            self.get_parameter("feedback_failure_exit_delay").value
        )
        self.serial_write_timeout = float(
            self.get_parameter("serial_write_timeout").value
        )
        self._monotonic_clock = monotonic_clock

        if not math.isfinite(self.encoder_cpr) or self.encoder_cpr <= 0.0:
            raise ValueError("encoder_cpr must be finite and positive")
        if self.encoder_max_delta_ticks <= 0:
            raise ValueError("encoder_max_delta_ticks must be positive")
        if not math.isfinite(self.cmd_vel_timeout) or self.cmd_vel_timeout <= 0.0:
            raise ValueError("cmd_vel_timeout must be finite and positive")
        for parameter_name, parameter_value in (
            ("xlinear_limit", self.xlinear_limit),
            ("ylinear_limit", self.ylinear_limit),
            ("angular_limit", self.angular_limit),
        ):
            if not math.isfinite(parameter_value) or parameter_value < 0.0:
                raise ValueError(
                    "%s must be finite and nonnegative" % parameter_name
                )
        for parameter_name, parameter_value in (
            ("feedback_startup_timeout", self.feedback_startup_timeout),
            ("feedback_timeout", self.feedback_timeout),
            ("feedback_failure_exit_delay", self.feedback_failure_exit_delay),
            ("serial_write_timeout", self.serial_write_timeout),
        ):
            if not math.isfinite(parameter_value) or parameter_value <= 0.0:
                raise ValueError("%s must be finite and positive" % parameter_name)
        validate_encoder_config(self.encoder_order, self.encoder_signs)

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
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        self.previous_encoders: Optional[tuple[int, int, int, int]] = None
        self.previous_encoder_time = None
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.feedback_failure_observed_at: Optional[float] = None
        self.feedback_failure_reason: Optional[str] = None
        self.feedback_failure_exit_checks = 0
        self.failure_stop_attempts = 0
        self.last_stop_attempt = "not required"

        factory = controller_factory or _create_controller_transport
        self.transport = factory(
            com=self.serial_port,
            clock=self._monotonic_clock,
            startup_timeout=self.feedback_startup_timeout,
            stale_timeout=self.feedback_timeout,
            write_timeout=self.serial_write_timeout,
        )
        self.car = self.transport.vendor
        try:
            self.car.set_car_type(1)
            self.transport.start()
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
            self.health_timer = self.create_timer(1.0, self.publish_health)
        except BaseException:
            try:
                if hasattr(self, "motion_safety"):
                    self.best_effort_stop_motion(repeat=3)
            finally:
                self.transport.close()
            raise

        self.get_logger().info(
            "X3 driver ready: serial_port=%s, cmd_vel_timeout=%.3fs, "
            "feedback_timeout=%.3fs, encoder_order=%s, encoder_signs=%s"
            % (
                self.serial_port,
                self.cmd_vel_timeout,
                self.feedback_timeout,
                self.encoder_order,
                self.encoder_signs,
            )
        )

    def publish_diagnostic(
        self,
        level: int,
        message: str,
        transport_status: Optional[TransportStatus] = None,
    ) -> None:
        """Publish standard health with actual report-freshness evidence."""
        feedback = transport_status or self.transport.status()
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "yahboomcar_bringup: motor controller and onboard sensors"
        status.message = message
        status.hardware_id = self.serial_port
        values = {
            "feedback_state": feedback.state.value,
            "feedback_reason": feedback.reason,
            "feedback_report_sequence": str(feedback.report_sequence),
            "feedback_timeout_seconds": "%.6f" % self.feedback_timeout,
            "serial_write_timeout_seconds": "%.6f"
            % self.serial_write_timeout,
            "serial_write_failure_count": str(
                getattr(self.transport, "serial_write_failure_count", 0)
            ),
            "latest_serial_write_failure": str(
                getattr(self.transport, "latest_serial_write_failure", None)
                or "none"
            ),
            "stop_attempt": self.last_stop_attempt,
        }
        stale_channels = []
        for channel in ("speed", "encoder", "imu_raw"):
            age = feedback.report_ages.get(channel)
            stale = age is None or age > self.feedback_timeout
            if stale:
                stale_channels.append(channel)
            values["feedback_%s_age_seconds" % channel] = (
                "unknown" if age is None else "%.6f" % age
            )
            values["feedback_%s_stale" % channel] = str(stale).lower()
        values["feedback_stale_channels"] = ",".join(stale_channels)
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        diagnostic.status = [status]
        self.diagnostic_publisher.publish(diagnostic)

    def publish_health(self) -> None:
        """Report receive-path health even when the state has not changed."""
        feedback = self.transport.status()
        if feedback.state is TransportState.HEALTHY:
            try:
                self.transport.perform_while_healthy(
                    "healthy diagnostic publication",
                    lambda _car: self.publish_diagnostic(
                        DiagnosticStatus.OK,
                        "Fresh motor, encoder, and onboard-sensor reports "
                        "are healthy",
                        feedback,
                    ),
                )
            except Exception:
                current = self.transport.status()
                if current.state is TransportState.FAILED:
                    self._observe_feedback_failure(current, allow_exit=False)
            return
        if feedback.state in (TransportState.CREATED, TransportState.WAITING):
            self.publish_diagnostic(
                DiagnosticStatus.WARN, feedback.reason, feedback
            )
            return
        self.publish_diagnostic(
            DiagnosticStatus.ERROR, feedback.reason, feedback
        )

    def _observe_feedback_failure(
        self, feedback: TransportStatus, allow_exit: bool
    ) -> None:
        """Stop, diagnose, and eventually exit after a terminal transport fault."""
        try:
            observed_now = float(self._monotonic_clock())
            if not math.isfinite(observed_now):
                observed_now = None
        except Exception:
            observed_now = None

        if self.feedback_failure_observed_at is None:
            self.feedback_failure_observed_at = observed_now
            self.feedback_failure_reason = feedback.reason
            self.get_logger().error(
                "Terminal motor-controller feedback failure: %s"
                % feedback.reason
            )
            stop_errors = self.best_effort_stop_motion(repeat=3)
            if stop_errors:
                self.last_stop_attempt = (
                    "attempted 3 zero commands; %d raised; delivery is not proven"
                    % len(stop_errors)
                )
                self.get_logger().error(
                    "Best-effort zero command error(s): %s"
                    % "; ".join(stop_errors)
                )
            else:
                self.last_stop_attempt = (
                    "attempted 3 zero commands; delivery is not proven"
                )
            self.failure_stop_attempts += 1

        self.publish_diagnostic(
            DiagnosticStatus.ERROR,
            "Terminal controller feedback failure: %s" % feedback.reason,
            feedback,
        )

        if not allow_exit:
            return
        self.feedback_failure_exit_checks += 1
        started_at = self.feedback_failure_observed_at
        fallback_checks = math.ceil(
            self.feedback_failure_exit_delay / 0.1
        ) + 1
        if (
            observed_now is None
            or started_at is None
            or observed_now - started_at >= self.feedback_failure_exit_delay
            or self.feedback_failure_exit_checks >= fallback_checks
        ):
            raise RuntimeError(
                "Required motor-controller feedback failed: %s"
                % (self.feedback_failure_reason or feedback.reason)
            )

    def _perform_actuator(
        self, label: str, action: Callable[[object], object]
    ) -> bool:
        """Run one actuator write atomically with fresh-feedback authority."""
        try:
            self.transport.perform_while_healthy(label, action)
            return True
        except Exception as exc:
            feedback = self.transport.status()
            if feedback.state is TransportState.HEALTHY:
                self.transport.latch_failure(
                    "%s failed: %s: %s" % (label, type(exc).__name__, exc)
                )
                feedback = self.transport.status()
        self.get_logger().warning(
            "Rejected %s while controller feedback is %s: %s"
            % (label, feedback.state.value, feedback.reason)
        )
        if feedback.state is TransportState.FAILED:
            self._observe_feedback_failure(feedback, allow_exit=False)
        return False

    def set_motion(self, vx: float, vy: float, wz: float) -> None:
        """Send nonzero motion atomically; always permit a monitored zero."""
        if (vx, vy, wz) != (0.0, 0.0, 0.0):
            sent = self._perform_actuator(
                "motion command",
                lambda car: car.set_car_motion(vx, vy, wz),
            )
            if not sent:
                raise RosmasterTransportError(
                    "motion command rejected by controller transport"
                )
            return

        failures_before = getattr(
            self.transport, "serial_write_failure_count", 0
        )
        self.car.set_car_motion(vx, vy, wz)
        failures_after = getattr(
            self.transport, "serial_write_failure_count", failures_before
        )
        if failures_after > failures_before:
            raise RosmasterTransportError(
                getattr(self.transport, "latest_serial_write_failure", None)
                or "zero-motion serial write failed"
            )

    def stop_motion(self, repeat: int = 1) -> None:
        """Send redundant zero commands and mark the driver stopped."""
        self.motion_safety.stop(repeat=repeat)

    def best_effort_stop_motion(self, repeat: int = 3) -> list[str]:
        """Attempt every redundant zero even if an individual write raises."""
        errors = []
        attempts = max(1, repeat)
        transport = getattr(self, "transport", None)
        for _ in range(attempts):
            failures_before = getattr(
                transport, "serial_write_failure_count", 0
            )
            try:
                self.car.set_car_motion(0.0, 0.0, 0.0)
            except Exception as exc:
                errors.append("%s: %s" % (type(exc).__name__, exc))
            failures_after = getattr(
                transport,
                "serial_write_failure_count",
                failures_before,
            )
            if failures_after > failures_before:
                errors.append(
                    getattr(
                        transport, "latest_serial_write_failure", None
                    )
                    or "zero-motion serial write failed"
                )
        self.motion_safety.motion_stopped = not errors
        return errors

    def _monotonic_now(self, context: str) -> Optional[float]:
        """Read the safety clock or turn its failure into a terminal fault."""
        try:
            now = float(self._monotonic_clock())
        except Exception as exc:
            reason = "%s monotonic clock raised %s: %s" % (
                context,
                type(exc).__name__,
                exc,
            )
        else:
            if math.isfinite(now):
                return now
            reason = "%s monotonic clock returned non-finite time %r" % (
                context,
                now,
            )
        self.transport.latch_failure(reason)
        self._observe_feedback_failure(
            self.transport.status(), allow_exit=False
        )
        return None

    def cmd_vel_callback(self, message: Twist) -> None:
        """Clamp and forward a ROS velocity command to the controller."""
        now_seconds = self._monotonic_now("cmd_vel")
        if now_seconds is None:
            return
        try:
            self.motion_safety.command(
                message.linear.x,
                message.linear.y,
                message.angular.z,
                now_seconds,
            )
        except RosmasterTransportError:
            return

    def check_cmd_vel_watchdog(self) -> None:
        """Stop persistent motion when command updates cease."""
        feedback = self.transport.status()
        if feedback.state is TransportState.FAILED:
            self._observe_feedback_failure(feedback, allow_exit=False)
            return
        now_seconds = self._monotonic_now("cmd_vel watchdog")
        if now_seconds is None:
            return
        elapsed = now_seconds - self.motion_safety.last_command_time
        try:
            expired = self.motion_safety.enforce_timeout(now_seconds)
        except RosmasterTransportError:
            feedback = self.transport.status()
            if feedback.state is TransportState.FAILED:
                self._observe_feedback_failure(feedback, allow_exit=False)
            return
        if expired:
            self.get_logger().warning(
                "cmd_vel timeout after %.3fs; commanding zero velocity"
                % elapsed
            )

    def rgb_light_callback(self, message: Int32) -> None:
        """Set the expansion-board RGB effect."""
        for _ in range(3):
            if not self._perform_actuator(
                "RGB command",
                lambda car: car.set_colorful_effect(
                    message.data, 6, parm=1
                ),
            ):
                return

    def buzzer_callback(self, message: Bool) -> None:
        """Set the expansion-board buzzer state."""
        for _ in range(3):
            if not self._perform_actuator(
                "buzzer command",
                lambda car: car.set_beep(1 if message.data else 0),
            ):
                return

    def make_joint_state(self, now, raw_encoders) -> JointState:
        """Build a normalized four-wheel state from one authorized snapshot."""
        state = JointState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = "joint_states"
        base_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "back_left_wheel_joint",
            "back_right_wheel_joint",
        ]
        state.name = [self.prefix + name for name in base_names]

        current_encoders = map_encoder_counts(
            raw_encoders, self.encoder_order, self.encoder_signs
        )

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
        """Publish one set only when new controller reports remain fresh."""
        feedback = self.transport.status()
        if feedback.state in (TransportState.CREATED, TransportState.WAITING):
            return
        if feedback.state is not TransportState.HEALTHY:
            self._observe_feedback_failure(feedback, allow_exit=True)
            return

        try:
            # get_version may send one request and wait for its response.  Keep
            # it outside the cache lock so the receive thread can parse that
            # response, then atomically copy the auto-report-backed caches.
            edition_value = float(self.car.get_version())
            (
                raw_encoders,
                battery_value,
                acceleration,
                angular_velocity,
                magnetic_field_values,
                motion,
            ) = self.transport.read_cached(
                lambda car: (
                    car.get_motor_encoder(),
                    float(car.get_battery_voltage()),
                    car.get_accelerometer_data(),
                    car.get_gyroscope_data(),
                    car.get_magnetometer_data(),
                    car.get_motion_data(),
                )
            )
        except Exception as exc:
            self.transport.latch_failure(
                "controller cache read failed: %s: %s"
                % (type(exc).__name__, exc)
            )
            self._observe_feedback_failure(
                self.transport.status(), allow_exit=True
            )
            return

        try:
            if len(raw_encoders) != 4:
                raise ValueError(
                    "raw encoder data must contain exactly four counters"
                )
            ax, ay, az = acceleration
            gx, gy, gz = angular_velocity
            mx, my, mz = magnetic_field_values
            vx, vy, angular = motion
            telemetry = (
                edition_value,
                battery_value,
                *raw_encoders,
                ax,
                ay,
                az,
                gx,
                gy,
                gz,
                mx,
                my,
                mz,
                vx,
                vy,
                angular,
            )
            if not all(math.isfinite(float(value)) for value in telemetry):
                raise ValueError("non-finite telemetry")
        except Exception as exc:
            self.transport.latch_failure(
                "controller cache validation failed: %s: %s"
                % (type(exc).__name__, exc)
            )
            self._observe_feedback_failure(
                self.transport.status(), allow_exit=True
            )
            return

        # A receive exception may have arrived while values were copied.  Do
        # not stamp or publish that cache if the monitored path has failed.
        feedback = self.transport.status()
        if feedback.state is not TransportState.HEALTHY:
            self._observe_feedback_failure(feedback, allow_exit=True)
            return

        now = self.get_clock().now()
        try:
            joint_state = self.make_joint_state(now, raw_encoders)
        except Exception as exc:
            self.transport.latch_failure(
                "wheel encoder conversion failed: %s: %s"
                % (type(exc).__name__, exc)
            )
            self._observe_feedback_failure(
                self.transport.status(), allow_exit=True
            )
            return

        edition = Float32(data=edition_value)
        battery = Float32(data=battery_value)

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

        def publish_batch(_car) -> None:
            self.joint_state_publisher.publish(joint_state)
            self.velocity_publisher.publish(velocity)
            self.imu_publisher.publish(imu)
            self.magnetic_field_publisher.publish(magnetic_field)
            self.voltage_publisher.publish(battery)
            self.edition_publisher.publish(edition)

        try:
            self.transport.perform_while_healthy(
                "controller data publication", publish_batch
            )
        except Exception:
            feedback = self.transport.status()
            if feedback.state is TransportState.FAILED:
                self._observe_feedback_failure(feedback, allow_exit=True)


def cleanup_driver(driver: YahboomCarDriver) -> None:
    """Best-effort shutdown that never masks the primary driver failure."""
    try:
        best_effort_stop = getattr(driver, "best_effort_stop_motion", None)
        if callable(best_effort_stop):
            stop_errors = best_effort_stop(repeat=3)
            if stop_errors:
                driver.get_logger().error(
                    "Final zero-command error(s): %s"
                    % "; ".join(stop_errors)
                )
        else:
            driver.stop_motion(repeat=3)
    except Exception as exc:
        driver.get_logger().error(
            "Final best-effort zero command raised: %s" % exc
        )
    try:
        driver.transport.close()
    except Exception as exc:
        driver.get_logger().error("Controller transport close raised: %s" % exc)
    try:
        driver.destroy_node()
    except Exception as exc:
        driver.get_logger().error("ROS node destruction raised: %s" % exc)


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
            cleanup_driver(driver)
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
