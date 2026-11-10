"""Hardware-free tests for the tilt controller packet and safety logic."""

import pytest

from tilt_teleop_core import (
    FLAG_CALIBRATED,
    FLAG_HOLD,
    FLAG_TURN_MODE,
    Limits,
    TiltGate,
    axis_speed,
    decode_packet,
    encode_packet,
)

DRIVE = FLAG_HOLD | FLAG_CALIBRATED
IDLE = FLAG_CALIBRATED
ROBOT = 4


def pkt(seq, fwd=0, right=0, flags=DRIVE, robot=ROBOT):
    """Encode a packet with angles in centi-degrees."""
    return encode_packet(robot, seq, fwd, right, flags)


def armed_gate():
    """A gate that already saw a released hold button."""
    gate = TiltGate(ROBOT, 0.3, Limits())
    assert gate.feed(pkt(1, flags=IDLE), 0.0)
    return gate


def test_axis_speed_deadzone_ramp_and_cap():
    args = (6.0, 30.0, 0.10, 0.35)
    assert axis_speed(5.9, *args) == 0.0
    assert axis_speed(6.0, *args) == pytest.approx(0.10)
    assert axis_speed(18.0, *args) == pytest.approx(0.225)
    assert axis_speed(30.0, *args) == pytest.approx(0.35)
    assert axis_speed(80.0, *args) == pytest.approx(0.35)
    assert axis_speed(-18.0, *args) == pytest.approx(-0.225)


def test_decode_rejects_bad_packets():
    good = pkt(1)
    assert decode_packet(good, ROBOT) is not None
    assert decode_packet(good, ROBOT + 1) is None  # other robot
    assert decode_packet(good[:-1], ROBOT) is None  # short
    assert decode_packet(good + b"\x00", ROBOT) is None  # long
    corrupt = bytearray(good)
    corrupt[9] ^= 0xFF
    assert decode_packet(bytes(corrupt), ROBOT) is None  # checksum
    bad_magic = bytearray(good)
    bad_magic[0] ^= 0x01
    assert decode_packet(bytes(bad_magic), ROBOT) is None


def test_forward_and_right_tilt_signs():
    gate = armed_gate()
    gate.feed(pkt(2, fwd=3000), 0.05)
    assert gate.command(0.06) == pytest.approx((0.35, 0.0, 0.0))
    gate.feed(pkt(3, right=3000), 0.10)  # right strafes to negative y
    assert gate.command(0.11) == pytest.approx((0.0, -0.35, 0.0))
    gate.feed(pkt(4, right=3000, flags=DRIVE | FLAG_TURN_MODE), 0.15)
    assert gate.command(0.16) == pytest.approx((0.0, 0.0, -1.0))


def test_release_and_timeout_give_zero():
    gate = armed_gate()
    gate.feed(pkt(2, fwd=3000), 0.05)
    assert gate.command(0.1)[0] > 0
    assert gate.command(0.36) == (0.0, 0.0, 0.0)  # 0.31 s of silence
    gate.feed(pkt(3, fwd=3000, flags=IDLE), 0.4)  # hold released
    assert gate.command(0.41) == (0.0, 0.0, 0.0)


def test_uncalibrated_never_drives():
    gate = armed_gate()
    gate.feed(pkt(2, fwd=3000, flags=FLAG_HOLD), 0.05)
    assert gate.command(0.06) == (0.0, 0.0, 0.0)


def test_stale_and_duplicate_sequence_rejected():
    gate = armed_gate()
    assert gate.feed(pkt(5, fwd=3000), 0.05)
    assert not gate.feed(pkt(5, fwd=3000), 0.06)  # duplicate
    assert not gate.feed(pkt(4, fwd=3000), 0.07)  # older
    assert gate.feed(pkt(6, fwd=3000), 0.08)


def test_sequence_wraparound_accepted():
    gate = TiltGate(ROBOT, 0.3, Limits())
    assert gate.feed(pkt(0xFFFFFFFF, flags=IDLE), 0.0)
    assert gate.feed(pkt(0, flags=IDLE), 0.05)


def test_no_motion_until_hold_released_after_start():
    gate = TiltGate(ROBOT, 0.3, Limits())
    gate.feed(pkt(1, fwd=3000), 0.0)  # hold already pressed at start
    assert gate.command(0.01) == (0.0, 0.0, 0.0)
    gate.feed(pkt(2, flags=IDLE), 0.05)
    gate.feed(pkt(3, fwd=3000), 0.10)
    assert gate.command(0.11)[0] > 0


def test_controller_reboot_resets_sequence_and_disarms():
    gate = armed_gate()
    gate.feed(pkt(900, fwd=3000), 0.05)
    assert gate.command(0.06)[0] > 0
    # controller reboots: 2 s gap, sequence restarts, hold still pressed
    assert gate.feed(pkt(1, fwd=3000), 2.1)
    assert gate.command(2.11) == (0.0, 0.0, 0.0)
    gate.feed(pkt(2, flags=IDLE), 2.15)
    gate.feed(pkt(3, fwd=3000), 2.2)
    assert gate.command(2.21)[0] > 0


def test_no_packet_yet_gives_zero():
    gate = TiltGate(ROBOT, 0.3, Limits())
    assert gate.command(5.0) == (0.0, 0.0, 0.0)
