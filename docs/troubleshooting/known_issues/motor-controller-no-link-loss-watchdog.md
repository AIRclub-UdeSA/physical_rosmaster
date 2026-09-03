# Motor Controller Has No Command-Loss Watchdog

## Status

Confirmed on `x3-c` on 2026-09-02. Root cause identified: the vendor motor
controller protocol has no command-timeout/watchdog capability. The project
owner has accepted this as a known platform limitation rather than a blocking
acceptance gate; see [Decision](#decision) below. A hardware-level mitigation
remains a valid future improvement, not scheduled.

## Symptom

During a supervised, securely lifted active-link-loss trial, a low commanded
speed (`+0.05 m/s` x) was streamed to `/cmd_vel`, and the motor controller's
USB/serial link was then physically disconnected while the command was
actively being executed. All four wheels continued rotating, actively driven,
for at least 28 seconds after disconnect (the length of the observation
window before power was cut) with no sign of decay. The robot was fully
lifted and secured throughout; no injury, damage, or floor travel occurred.
The operator cut main power at the switch to stop it, per the predeclared
safety procedure for this trial.

## Affected Environment

- Robot: `x3-c`
- Runtime head under test: `e34f8a35a75fb824add197d18fa330d3934eb89b`
  (`platform/simulator-parity`, containing both `a08b097` and `bc965a6`)
- `Rosmaster_Lib.py` SHA256: `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`
  (verified public V3.3.9)
- Wheels lifted and secured; battery at `10.6` V

## Safety Warning

A command-link loss during active motion is not a rare or unusual fault for a
wired mobile robot — a working-loose connector, a snagged cable, or vibration
during normal driving are ordinary causes. Treat this as a standing platform
characteristic for any motion, lifted or floor, not a one-off test artifact:

- **The only proven stop mechanism for this failure mode is a human cutting
  main power.** No software running on the Raspberry Pi can help, because by
  the time the link is lost, the Pi has no channel left to reach the
  controller at all.
- Do not rely on `sudo poweroff`, SSH, or any ROS-side command to stop the
  robot once the motor-controller link is suspected lost — none of those can
  reach a controller that is no longer connected. Only physical power removal
  (main switch or battery disconnect) works.
- Keep a human able to reach main power during any commanded motion.

## Fast Diagnosis

Distinguishing this from a coast-down or sensor glitch: log `/joint_states`
velocity through the event. A coast on lifted, low-friction wheels decays
smoothly toward zero within a second or two. What was observed instead was a
sustained, rhythmic oscillation (rise/fall roughly every 1-1.5 s) between
approximately `1.3` and `3.7 rad/s` per wheel, with no decay across the full
recorded window — the signature of the controller's own closed-loop speed
control still actively regulating toward the last commanded setpoint, not
friction winding down.

## Root Cause

The installed `Rosmaster_Lib.py` (hash-verified, byte-identical to the
allowlisted public V3.3.9 source) was pulled from the robot and its full
protocol surface was audited: all 30 `FUNC_*` command codes
(`0x01`-`0x06`, `0x0A`-`0x15`, `0x20`-`0x24`, `0x30`-`0x31`, `0x50`-`0x51`,
`0xA0`) and every public `set_*`/`get_*` method. None relate to a
command-timeout, watchdog, heartbeat, or auto-stop-on-silence capability. The
only "timeout" references in the source are unrelated serial-response retry
loops in the UART-servo angle getters.

**There is no firmware-configurable command-loss watchdog available through
this protocol.** This is not a case of the capability existing but going
unused; it is not present in the command set at all.

This also explains why the driver-side `cmd_vel_timeout` watchdog
(`Mcnamu_driver_X3.py`'s `check_cmd_vel_watchdog`) passes its own gate (command
stream interrupted, link healthy) but cannot cover this one: that watchdog
works by having the Pi actively send an explicit zero command, which the
still-connected controller receives and obeys. It is the Pi stopping the
controller, not the controller stopping itself. Once the serial link itself
is gone, there is no channel left for that zero command, or any command, to
arrive by. A pure software fix is therefore not feasible for this specific
failure mode — not because it would be difficult, but because "the link is
gone" is definitionally the one case Pi-side software cannot act around.

## Decision

The project owner reviewed this finding on 2026-09-02 and decided to accept
the residual risk rather than block platform acceptance on it. Reasoning
recorded here for future reference:

- A software mitigation is not feasible (see Root Cause).
- The available hardware mitigations (an independent relay/E-stop cutting
  motor power on loss of a Pi heartbeat, or a physical inline kill switch on
  the motor power line) are real, low-effort options, but a hardware change is
  out of scope for now.
- This is an educational robot; the owner judged the residual risk acceptable
  to keep the project moving rather than block indefinitely on a hardware
  change with no committed timeline.

As a result, the previously required "bounded active-motion physical stop"
gate is **no longer a blocking acceptance requirement** for lifted motion,
floor motion, or autostart. The finding above and this decision remain on the
record; they are not being hidden or treated as resolved. No blanket
low-speed operating rule has been added as a condition of this decision.

## What Would Close This Properly

Documented here for whenever it becomes worth doing, not as a current
requirement:

- An independent hardware watchdog: e.g. a Pi GPIO heartbeat driving a relay
  that cuts motor-controller power if the heartbeat stops, or a simple
  physical inline kill switch/E-stop on the motor power line reachable faster
  than the main switch.
- Re-run this exact trial (lifted, low speed, predeclared stop bound, human at
  power) against any such mitigation before trusting it.

## Related Documentation

- [Troubleshooting Index](../README.md)
- [docs/robot_side_next_moves.md](../../robot_side_next_moves.md)
- [docs/robot_side_verification_todo.md](../../robot_side_verification_todo.md)
- [tools/motor_live_loss_probe.py](../../../tools/motor_live_loss_probe.py) —
  the observer-only, no-command variant of this risk, which does pass
