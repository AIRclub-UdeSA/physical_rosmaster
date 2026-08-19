# X3-C Odometry & Motion Recovery Checklist

Date: 2026-08-19  
Target: Physical Yahboom ROSMASTER X3 (`x3-c`)  
Container: `rosmaster_humble` (`ROS_DOMAIN_ID=11`)  
Workspace: `/root/yahboomcar_ws/src/physical_rosmaster`

---

## Quick Reference / Emergency Stop

```bash
# Emergency stop command (safe zero velocity)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Phase 0: Pre-Flight & Physical Checks

> [!IMPORTANT]
> The previous floor test failure at 10.1 V was likely caused by motor under-voltage / deadband. Ensure the battery is charged.

- [ ] **Battery Level**: Verify the 3S LiPo battery is charged (> 12.0 V, never below 10.5 V).
- [ ] **Motor Power Switch**: Ensure the motor board power switch / toggle switch on the expansion board is in the **ON** position.
- [ ] **Lift the Robot**: Place the robot on a stand so all four Mecanum wheels spin freely off the ground.
- [ ] **Host Identity (Optional Host Fix)**: On Raspberry Pi host, confirm `/etc/hosts` contains `127.0.1.1 x3-c` to silence `sudo` resolution warnings.

---

## Phase 1: Workspace & Code Update (Inside Container)

Run inside `rosmaster_humble`:

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
git fetch origin
git checkout main
git pull origin main

# Rebuild driver and base node
cd /root/yahboomcar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select yahboomcar_bringup yahboomcar_base_node
source install/setup.bash
```

- [ ] Build succeeded with 0 errors.
- [ ] Verify `yahboom_joy_node` is not launched unless `use_joy:=true` is passed.

---

## Phase 2: Per-Wheel Encoder Calibration (Robot Lifted, No ROS Nodes)

Before running any motion commands, calibrate the motor index mapping (`m1..m4`) and sign convention by hand.

### Step 2.1: Run the Hardware Probe

```bash
cd /root/yahboomcar_ws/src/physical_rosmaster
python3 tools/rosmaster_lib_probe.py --samples 500 --period 0.1
```

### Step 2.2: Rotate Each Wheel Forward By Hand & Record Feedback

1. **Front-Left (FL)** wheel: Rotate forward ~2 turns by hand.
   - Channel changed (`m1`, `m2`, `m3`, or `m4`): `____`
   - Direction: `[+]` or `[-]`
2. **Front-Right (FR)** wheel: Rotate forward ~2 turns by hand.
   - Channel changed: `____`
   - Direction: `[+]` or `[-]`
3. **Back-Left (BL)** wheel: Rotate forward ~2 turns by hand.
   - Channel changed: `____`
   - Direction: `[+]` or `[-]`
4. **Back-Right (BR)** wheel: Rotate forward ~2 turns by hand.
   - Channel changed: `____`
   - Direction: `[+]` or `[-]`

### Step 2.3: Verify / Fix Driver Mapping

Check `yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py`:
```python
m1, m2, m3, m4 = self.car.get_motor_encoder()
# Verify this mapping matches your observations for [FL, FR, BL, BR]
curr_encoders = [m2, m1, m4, m3]
```
If signs or channel orders differ, update `curr_encoders` and rebuild.

---

## Phase 3: Lifted Kinematic Validation (Robot Lifted)

### Step 3.1: Launch the Clean Bringup Stack

**Terminal 1 (Launch Bringup without Joystick):**
```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

### Step 3.2: Verify Clean ROS Graph

**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash

# Confirm exactly one publisher on /cmd_vel (should be 0 when idle)
ros2 topic info /cmd_vel
# Confirm /joint_states and /odom_raw rates
ros2 topic hz /joint_states
ros2 topic hz /odom_raw
```

### Step 3.3: Execute Isolated Motion Tests

In Terminal 2, monitor `/odom_raw`:
```bash
ros2 topic echo /odom_raw --field pose.pose
```

In **Terminal 3**, execute the following test pulses (at `0.20 m/s` to overcome any firmware deadband):

```bash
# 1. FORWARD TEST (+X)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1.5
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# CHECK:
# -> delta X must be POSITIVE (> +0.10 m)
# -> delta Y must be approx 0
# -> delta Yaw must be approx 0
```

```bash
# 2. STRAFE LEFT TEST (+Y)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.20, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 1.5
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# CHECK:
# -> delta X must be approx 0
# -> delta Y must be POSITIVE (> +0.10 m)  <-- THIS FAILED PREVIOUSLY (was -0.22 m)
# -> delta Yaw must be approx 0
```

```bash
# 3. ROTATE CCW TEST (+Z)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.50}}"
sleep 1.5
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# CHECK:
# -> delta X, delta Y must be approx 0
# -> delta Yaw / orientation must be POSITIVE (> +0.30 rad)
```

---

## Phase 4: Floor Deadband & Motion Validation (Robot on Floor)

> [!CAUTION]
> Ensure a 2-meter clear perimeter around the robot before running floor motion commands.

Place robot on smooth level floor.

### Step 4.1: Find Breakaway Velocity Threshold

Send brief pulses with increasing linear velocity to determine the minimum speed overcoming friction:

```bash
# Test 0.15 m/s
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}" ; sleep 1 ; ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"

# Test 0.20 m/s
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}}" ; sleep 1 ; ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"

# Test 0.25 m/s
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}" ; sleep 1 ; ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
```

- [ ] Note the minimum speed where the robot starts moving smoothly on the floor.
- [ ] Confirm `/joint_states` and `/odom_raw` increment with physical floor translation.

---

## Phase 5: Measured Distance & Angle Calibration

### Step 5.1: 1.0-Meter Linear Translation Test

1. Mark a start line and a 1.00 m end line on the floor with tape.
2. Align robot front wheels on the start line.
3. Record starting odom position:
   ```bash
   ros2 topic echo /odom_raw --field pose.pose.position -n 1
   ```
4. Drive forward until the front wheels reach the 1.00 m mark:
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.20}}"
   # Ctrl+C when reaching the line, then immediately stop:
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
   ```
5. Record final odom position:
   ```bash
   ros2 topic echo /odom_raw --field pose.pose.position -n 1
   ```
6. **Error Calculation**:
   $$\text{Scale Error} = \frac{\Delta x_{\text{odom}}}{1.00\text{ m}}$$
   If odom reads $0.95\text{ m}$ or $1.05\text{ m}$, calibrate `linear_scale_x` in `base_node_X3` parameters or check `encoder_cpr` (1040.0).

### Step 5.2: 360-Degree Rotation Calibration

1. Mark starting robot orientation.
2. Send CCW rotation until 1 full revolution ($2\pi = 6.283\text{ rad}$) is completed:
   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.50}}"
   # Stop after exactly 1 full rotation:
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
   ```
3. Check heading angle reported by `/odom_raw` and `/odom` (EKF).
4. Verify if yaw reached approximately $6.28\text{ rad}$ ($0\text{ rad}$ wrapped).

---

## Phase 6: Validation Sign-off & Report

- [ ] Lifted tests: X, Y, Yaw signs match commands.
- [ ] Floor tests: Breakaway velocity confirmed; robot moves smoothly.
- [ ] Odometry vs ground truth: Linear error $< 5\%$, rotation error $< 5\%$.
- [ ] Commit any necessary parameter updates (`linear_scale_x`, `linear_scale_y`, `encoder_cpr`) and record results in `agents/x3-c_odom_validation_<DATE>.md`.
