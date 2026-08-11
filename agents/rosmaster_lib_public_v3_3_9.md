# Rosmaster Lib Public V3.3.9 Notes

Date: 2026-08-11

This records the public Yahboom `Rosmaster_Lib` reference inspected while investigating physical X3 odometry. The downloaded files are intentionally not vendored into this repo.

## Source

- Yahboom official GitHub repository: `YahboomTechnology/ROSMASTERX3`
- Download-link file: `All_File_Download_Link/All_File_Download_Link.txt`
- Yahboom driver-library documentation page: `2. Install Rosmaster driver library`
- Google Drive folder entry inspected: `6.Annex_File/B_ROSMASTER drive library/py_install_V3.3.9.zip`

## Local Inspection Copy

- Zip: `/tmp/rosmaster_lib_download/py_install_V3.3.9.zip`
- Extracted library: `/tmp/rosmaster_lib_download/py_install_V3.3.9/py_install/Rosmaster_Lib/Rosmaster_Lib.py`
- Zip SHA256: `1761c5873b6d1407afe5b4f4c5c4fb05787e07730448e786243071fe9e8b6ce7`
- `Rosmaster_Lib.py` SHA256: `e9fd0f6bb015cda7dba58f4db6994402d83865cc125ab33035dbb39e978b1a8c`
- `Rosmaster_Lib.py` size: `57997` bytes
- Version comment in source: `# V3.3.9`

## Motion Feedback Findings

Public V3.3.9 does not show Python-side echoing of `cmd_vel` into the cached velocity returned by `get_motion_data()`.

- `set_car_motion(v_x, v_y, v_z)` packs commanded chassis velocities as signed 16-bit values scaled by `1000` and sends a serial command with function code `FUNC_MOTION = 0x12`.
- `get_motion_data()` only returns cached `__vx`, `__vy`, and `__vz`.
- Those cached fields are updated in `__parse_data()` when the received serial packet type is `FUNC_REPORT_SPEED = 0x0A`.
- The speed packet layout is:
  - `ext_data[0:2]`: signed int16 `vx`, divided by `1000.0`
  - `ext_data[2:4]`: signed int16 `vy`, divided by `1000.0`
  - `ext_data[4:6]`: signed int16 `vz`, divided by `1000.0`
  - `ext_data[6:7]`: unsigned battery byte
- `create_receive_threading()` starts the serial receive/parser thread.
- `set_auto_report_state()` documents that auto-report is enabled by default; the MCU sends four packet types every 10 ms, so each packet refreshes about every 40 ms.

Interpretation: in this public library, `get_motion_data()` is firmware/controller-reported chassis velocity, not direct Python echo of the ROS `cmd_vel`. That still is not the same as integrating four wheel encoder position deltas in ROS.

## Encoder Findings

The public library exposes four motor encoder counters.

- `FUNC_REPORT_ENCODER = 0x0D`
- `__parse_data()` decodes four signed 32-bit values:
  - `ext_data[0:4]`: motor 1 encoder
  - `ext_data[4:8]`: motor 2 encoder
  - `ext_data[8:12]`: motor 3 encoder
  - `ext_data[12:16]`: motor 4 encoder
- `get_motor_encoder()` returns the cached `(m1, m2, m3, m4)` values.

Interpretation: real encoder-position odometry is probably possible if these counters update reliably on the physical robot and if we confirm ticks-per-revolution, wheel radius, and wheel ordering/signs.

## Current Repo Usage

`yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py`:

- Subscribes to `cmd_vel`.
- Calls `self.car.set_car_motion(vx, vy, angular)`.
- Calls `self.car.get_motion_data()` in `pub_data()`.
- Publishes those values to `vel_raw`.
- Creates a `joint_states` publisher and a `JointState` object, but the current X3 driver does not publish wheel encoder positions from `get_motor_encoder()`.

`yahboomcar_base_node/src/base_node_X3.cpp`:

- Subscribes to `vel_raw`.
- Integrates body-frame velocity into `/odom_raw`.
- Does not currently consume wheel encoder position deltas.

## Robot-Side Confirmation Needed

When the robot is available, compare the installed library with the public reference:

```bash
python3 - <<'PY'
import hashlib
import inspect
from Rosmaster_Lib import Rosmaster

path = inspect.getsourcefile(Rosmaster)
data = open(path, "rb").read()
print(path)
print(hashlib.sha256(data).hexdigest())
print(len(data))
PY
```

Then run a hardware sanity check with wheels lifted and on the floor:

```bash
python3 - <<'PY'
import time
from Rosmaster_Lib import Rosmaster

car = Rosmaster()
car.create_receive_threading()

for i in range(100):
    print(i, "motion=", car.get_motion_data(), "encoders=", car.get_motor_encoder())
    time.sleep(0.1)
PY
```

Expected useful signal: encoder values should change when wheels rotate, keep their latest count when stopped, and diverge if one wheel slips or is resisted. If that holds, the physical odometry fix should use encoder deltas, not `vel_raw` integration alone.
