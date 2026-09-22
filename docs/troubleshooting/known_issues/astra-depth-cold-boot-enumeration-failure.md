# Astra Depth Sensor Fails To Enumerate On Cold Boot

## Status

Confirmed reproducible on `x3-c`, 2026-09-22. Not root-caused to a specific
internal fault; current mitigation is a physical unplug/replug before each
session. Escalated from an intermittent nuisance to a reliable every-boot
failure — see [Root Cause](#root-cause).

## Symptom

On boot, the Astra's depth (structured-light) USB function never enumerates,
while its RGB (UVC) function enumerates normally. This blocks
`rosmaster-wait-for-platform`'s `ExecStartPre` gate (`astra_present()`
requires both `2bc5:060f` and `2bc5:050f`), so `rosmaster-platform.service`
fails and autostart does not fire — no boot-ready buzzer/RGB signal, platform
not running. A physical unplug and reseat of the camera's USB connection
reliably brings the depth function up.

Previously intermittent ("sometimes"); as of 2026-09-22 it reproduces on
every cold boot (Juan, direct observation).

## Affected Environment

- Robot: `x3-c`
- Astra module: serial `ACRC64300ET`, internal 4-port USB2.0 hub
  (`idVendor=05e3, idProduct=0608`, Genesys Logic) fanning out to depth
  (`2bc5:060f`, "ORBBEC Depth Sensor") and RGB (`2bc5:050f`, "USB 2.0
  Camera", Sonix Technology) as separate child ports
- Host: Raspberry Pi, kernel `6.6.62+rpt-rpi-2712` (Pi 5), physical bus path
  `1-2` (hub) → `1-2.1` (depth) / `1-2.2` (RGB)
- `rosmaster-platform.service`'s `ExecStartPre` chain: `rosmaster-disk-guard`,
  then `rosmaster-wait-for-platform` (60s bounded poll, checks container +
  motor + LiDAR + both Astra USB IDs)

## Safety / Data-Loss Warning

No safety or data-loss implication by itself. Operational consequence: since
it now happens every boot, unattended autostart is currently unreliable
end-to-end — treat the robot as requiring a manual camera reseat before every
session until this is resolved.

## Fast Diagnosis

```bash
lsusb -d 2bc5:060f   # depth function — absent when this issue is active
lsusb -d 2bc5:050f   # RGB function — present even when depth is missing
lsusb -t              # confirm the internal 05e3:0608 hub is present; RGB
                       # directly reachable while depth's sibling port shows
                       # nothing at all
journalctl -k -b 0 | grep -i 2bc5   # depth function never appears anywhere
                                     # in this boot's kernel log if active
systemctl status rosmaster-platform.service   # ExecStartPre
                                               # rosmaster-wait-for-platform
                                               # exits 1/FAILURE
journalctl -t rosmaster-wait-for-platform -b 0   # explicit "Timed out ...
                                                  # waiting for: astra:usb-2bc5"
```

Distinguishing from a hub-level dropout (the 2026-09-18 incident): that
incident showed explicit `USB disconnect` / `clear tt ... error -71` kernel
messages and affected the motor+LiDAR hub (bus 1's VIA Labs hub), not the
Astra's own internal hub. This issue shows **no disconnect or error messages
at all** — the depth function's port simply never comes up in the first
place. `journalctl -k -b 0 | grep -icE "usb disconnect|clear tt"` returns `0`
while this issue is reproducing.

## Distinguishing Evidence

Confirmed live on 2026-09-22: with the platform down and the depth function
absent (`lsusb -d 2bc5:060f` empty, 43+ minutes after boot, no kernel error),
a physical unplug/replug of the camera caused the entire internal hub to
re-enumerate under a fresh device number, and **both** child ports came up
cleanly (`New USB device found` for both `2bc5:060f` and `2bc5:050f`, moments
apart). This points at a per-port (or per-device) initialization race
specific to the depth function, not a wiring-level dropout, marginal power
rail (RGB shares the same hub and the same power domain and enumerates
fine), or a host-side USB/xHCI fault (no controller errors, no disconnects,
other devices on the same host bus tree unaffected).

## Root Cause

Not established. The depth function's port on the Astra's internal hub does
not come up during normal cold-boot power sequencing, for reasons not yet
isolated on this robot. Two competing explanations, not yet distinguished:

- A timing/init race internal to the Astra module (the depth stream engine
  takes longer to be ready than the hub's enumeration window allows) — would
  predict a roughly constant failure rate, which does not match the observed
  sometimes-to-always progression.
- A physically degrading connection (a working-loose connector, a flexing
  cable, contact wear) specific to the depth port's wiring inside the module
  — consistent with a worsening failure rate over time. Not yet inspected.

## Fix

Currently: physical unplug and reseat of the Astra's USB connection before
starting the platform. This has a 100% success rate in every reproduction so
far (one controlled reproduction on 2026-09-22, consistent with Juan's prior
ad hoc experience).

No software fix identified. `rosmaster-wait-for-platform`'s 60s bounded wait
is working as designed (it correctly fails closed rather than hanging or
racing a partial launch); raising its timeout would not help, because the
depth port has been observed to not come up on its own within any window
tried, including 43+ minutes without intervention.

## Verification

After reseating: `lsusb -d 2bc5:060f` and `lsusb -d 2bc5:050f` both present,
`sudo systemctl start rosmaster-platform.service` succeeds with both
`ExecStartPre` checks `0/SUCCESS`, `ros2 node list` shows all seven nodes,
`rosmaster-platform-ready.service` passes the physical contract probe and
sends the boot-ready buzzer/RGB signal.

## What Not To Do

- Do not raise `PLATFORM_WAIT_TIMEOUT` as a workaround — the depth port has
  been observed absent for 43+ minutes without ever coming up on its own; a
  longer timeout just delays discovering the same failure.
- Do not assume the RGB function enumerating means the camera is fine —
  `astra_present()` (and the physical contract probe) require both
  functions; checking only `lsusb -d 2bc5:050f` will miss this issue.
- Do not treat this as the same failure mode as the 2026-09-18 hub dropout
  (`docs/troubleshooting/incidents/2026-09-18-x3-c-usb-hub-dropout.md`) —
  different hub, no disconnect signature, different fix.

## Prevention or Hardening

Not yet decided — open for the project owner. Options for whenever this is
prioritized, not a current requirement:

- Physically inspect the Astra's internal USB wiring/connector for wear,
  matching the sometimes-to-always progression, before assuming this is a
  fixed firmware characteristic.
- A boot-time retry loop that power-cycles just the Astra (if the hub or host
  port supports per-port power switching) before falling back to the current
  bounded-wait-and-fail behavior, so a session does not require a human
  physically present to reseat a cable.
- If the module is confirmed physically degrading, replacement is the
  durable fix.

## Affected Robot / Incident

- Robot: `x3-c`
- Workstation-local date: 2026-09-22 (`America/Argentina/Buenos_Aires`)
- Robot-local date recorded during diagnosis: 2026-09-23 (robot's system
  timezone is `Asia/Shanghai`)
- Symptom: `rosmaster-platform.service` fails `ExecStartPre`, no autostart,
  no boot-ready signal
- Direct cause: Astra depth USB function (`2bc5:060f`) does not enumerate on
  cold boot
- Confirmed mitigation: physical unplug/reseat of the camera

## Related Documentation

- [Troubleshooting Index](../README.md)
- [2026-09-18 x3-c USB hub dropout and missing boot-ready signal](../incidents/2026-09-18-x3-c-usb-hub-dropout.md)
- [docs/autostart_setup.md](../../autostart_setup.md)
