"""Measure how long the robot takes to act on /cmd_vel sent from the workstation.

Sends short forward pulses and times, on the workstation clock, until /odom
shows motion, and from the zero command until /odom is still again. The time
includes WiFi both ways, the driver, motor stiction and the 10 Hz /odom period.
A pulse during which /odom stopped arriving for more than ODOM_STALE_S is
reported as stale and left out of the summary, since its times would come from
an old velocity.

The run stops at the first stale /odom or the first pulse that does not stop:
it sends zeros, prints what it counted and exits non-zero, also if no pulse was
counted (1) or it was interrupted (2).

ONLY WITH THE WHEELS LIFTED. Refuses to start while another /cmd_vel publisher
exists or /odom is stale.

Usage: python3 cmd_latency.py --confirm-wheels-lifted [--pulses 5] [--speed 0.10]
"""

import argparse
import signal
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

PULSE_S = 1.5
STOP_WINDOW_S = 2.0
PUBLISH_PERIOD_S = 0.05  # 20 Hz, well inside the driver's 0.5 s cmd_vel_timeout
MAX_SPEED = 0.2
# /odom arrives at 10 Hz; five missed periods means the velocity is stale.
ODOM_STALE_S = 0.5


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--confirm-wheels-lifted", action="store_true", required=True)
    parser.add_argument("--pulses", type=int, default=5)
    parser.add_argument("--speed", type=float, default=0.10)
    args = parser.parse_args(argv)
    if not 0 < args.speed <= MAX_SPEED:
        parser.error(f"--speed must be in (0, {MAX_SPEED}] m/s")
    if args.pulses < 1:
        parser.error("--pulses must be at least 1")
    return args


def odom_age(received_at, now):
    """Seconds since the last /odom, or infinity if none has arrived."""
    return float("inf") if received_at is None else now - received_at


def raise_keyboard_interrupt(_signum, _frame):
    """Turn SIGTERM into the same stop path as Ctrl-C."""
    raise KeyboardInterrupt


def main(argv=None):
    args = parse_args(argv)
    moving = 0.3 * args.speed  # /odom vx that counts as "moving"
    still = 0.2 * moving

    # Without rclpy's SIGINT handler, Ctrl-C reaches the finally below with the
    # context still valid, so the zeros go out.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)
    node = rclpy.create_node("wifi_cmd_latency")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"vx": None, "at": None}

    def on_odom(msg):
        state["vx"] = msg.twist.twist.linear.x
        state["at"] = time.monotonic()

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
    age = odom_age(state["at"], time.monotonic())
    if age > ODOM_STALE_S:
        sys.exit(f">>> /odom is stale: the last one arrived {age:.1f} s ago")

    forward, zero = Twist(), Twist()
    forward.linear.x = args.speed
    starts, stops, stale = [], [], 0
    status = 0
    try:
        for pulse in range(1, args.pulses + 1):
            # The driver has no watchdog on /odom: never command motion blind.
            age = odom_age(state["at"], time.monotonic())
            if age > ODOM_STALE_S:
                print(f"pulse {pulse}: /odom STALE (no message for {age * 1000:.0f} ms);"
                      " not sent, stopping", flush=True)
                stale += 1
                status = 1
                break
            began, started, max_age = time.monotonic(), None, 0.0
            while time.monotonic() - began < PULSE_S:
                publisher.publish(forward)
                spin_for(PUBLISH_PERIOD_S)
                max_age = max(max_age, odom_age(state["at"], time.monotonic()))
                # Only a velocity measured after the command went out counts.
                if started is None and state["at"] >= began and abs(state["vx"]) > moving:
                    started = time.monotonic() - began
            zeroed, stopped = time.monotonic(), None
            while time.monotonic() - zeroed < STOP_WINDOW_S:
                publisher.publish(zero)
                spin_for(PUBLISH_PERIOD_S)
                max_age = max(max_age, odom_age(state["at"], time.monotonic()))
                if stopped is None and state["at"] >= zeroed and abs(state["vx"]) < still:
                    stopped = time.monotonic() - zeroed
            if max_age > ODOM_STALE_S:
                print(f"pulse {pulse}: /odom STALE (no message for {max_age * 1000:.0f} ms);"
                      " not counted, stopping", flush=True)
                stale += 1
                status = 1
                break
            start_text = f"{started * 1000:5.0f} ms" if started is not None else "  DID NOT MOVE"
            stop_text = f"{stopped * 1000:5.0f} ms" if stopped is not None else "  DID NOT STOP"
            print(f"pulse {pulse}: start {start_text}   stop {stop_text}", flush=True)
            if stopped is None:
                print(">>> the robot did not stop on zeros, stopping", flush=True)
                status = 1
                break
            if started is not None:
                starts.append(started)
                stops.append(stopped)
            spin_for(1.0)
    except KeyboardInterrupt:
        status = 2
    finally:
        # A second Ctrl-C (or a SIGTERM after one) would otherwise raise
        # KeyboardInterrupt here and cut the zero commands short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for _ in range(10):
            publisher.publish(zero)
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"counted {len(starts)} of {args.pulses} pulses ({stale} stale)")
    for name, values in (("start", starts), ("stop", stops)):
        if values:
            print(f"{name}: median {statistics.median(values) * 1000:.0f} ms,"
                  f" max {max(values) * 1000:.0f} ms")
    return status or (0 if starts else 1)


if __name__ == "__main__":
    sys.exit(main())
