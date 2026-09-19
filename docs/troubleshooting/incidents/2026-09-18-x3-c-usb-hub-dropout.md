# 2026-09-18 `x3-c`: USB hub dropout takes the platform down, and no boot-ready signal

Two unrelated failures on the same power-on. The reported symptom ("the
battery LED did not turn on") was the less serious of the two.

## Symptom

Operator powered the robot on, reached it over SSH, and saw no boot-ready
buzzer or `RGBLight` battery display. By the time it was investigated the
entire ROS graph was gone, which the operator had not observed.

## Failure 1: no boot-ready signal (point-cloud rate)

`rosmaster-platform-ready` runs `tools/physical_contract_probe.py` before
signalling. It failed:

```
Physical contract FAILED: /cam_1/depth/color/points: measured 1.15 Hz outside 3.0..40.0 Hz
```

The probe requires 3.0-40.0 Hz. The XYZRGB cloud has been measured at
2.83-10.93 Hz (see [sensor capabilities](../../sensor_capabilities.md)), so it
sits on the probe's lower bound and fell under it while the machine was still
busy at boot.

This is [issue #16](https://github.com/AIRclub-UdeSA/physical_rosmaster/issues/16)
presenting as a functional boot failure rather than an RViz annoyance.

**Resolution:** re-running `rosmaster-platform-ready` after the system settled
passed at 9.63 Hz and signalled normally. Not a fix - the underlying rate is
unchanged.

## Failure 2: USB hub dropout terminated the platform

About five minutes after boot the VIA Labs hub carrying the motor controller
and the LiDAR disconnected and re-enumerated:

```
[306.747] usb 1-1: USB disconnect, device number 2
[306.747] usb 1-1.2: USB disconnect, device number 3
[306.747] usb 1-1: clear tt 2 (9032) error -71
[309.417] usb 1-1.2: ch341-uart converter now attached to ttyUSB0
```

The driver observed the serial failure and fail-closed as designed:

```
Terminal motor-controller feedback failure: controller receive thread raised
SerialException: device reports readiness to read but returned no data
RuntimeError: Required motor-controller feedback failed
```

`driver_node` exited, and the strict launch drained the rest of the graph.
`rosmaster-platform.service` then reported `ExecStart` status 0 and
`inactive (dead)` - a clean shutdown, so nothing in systemd's state flagged
that the robot had lost its platform.

**This is correct designed behavior**, not a software defect: a required
process died and strict bringup refused to continue with a partial graph.

### Not a power sag

Battery measured 11.0 V immediately after restart, consistent with the 11.2 V
recorded the previous session. A hub-level `clear tt ... error -71` five
minutes into a run points at the physical link rather than supply voltage.
Check the cable between the Pi and the Yahboom powered hub, and the hub's own
power connector, before suspecting anything else. If it recurs, treat it as a
hardware fault.

### Device nodes swapped

Across the re-enumeration the motor moved from `ttyUSB1` to `ttyUSB0` and the
LiDAR from `ttyUSB0` to `ttyUSB1`. This was harmless because the platform
selects devices by `/dev/serial/by-id/` path.

Worth recording: the kernel reports the CH340 with `SerialNumber=0` - it
exposes no serial at all - so `usb-1a86_USB_Serial-if00-port0` is derived from
vendor and product alone and would be ambiguous if a second CH340 were ever
attached to this robot. The setup guide already warns about this; this is the
hardware confirmation.

## Fast diagnosis next time

`systemctl is-active rosmaster-platform` reporting `active` is not proof the
graph is alive, and a clean `inactive (dead)` is not proof nothing went wrong.
Check for nodes directly:

```bash
docker exec rosmaster_humble /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && source /root/yahboomcar_ws/install/setup.bash && ros2 node list"
```

An empty list with the service reporting `active` means the launch is mid-drain
or already gone. Then read the trigger, skipping the Astra parameter-undeclare
shutdown noise:

```bash
journalctl -u rosmaster-platform | grep -iE "process has died|Terminal .* failure" | tail
```

## Status

Both failures understood. The platform was restarted and verified: all seven
nodes up, contract probe passing, boot-ready signal sent. The cloud rate
remains the open item, tracked in
[issue #16](https://github.com/AIRclub-UdeSA/physical_rosmaster/issues/16); the
hub dropout has not recurred and needs a physical cable check.
