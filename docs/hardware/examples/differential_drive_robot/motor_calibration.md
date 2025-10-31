# Motor Calibration Guide - Differential Drive Robot

**Difficulty**: Intermediate
**Time**: 30-45 minutes
**Goal**: Achieve matched motor speeds and accurate odometry

---

## Overview

Without proper calibration, your robot will:
- Drift left or right when moving forward
- Spin unexpectedly during turns
- Accumulate large odometry errors
- Fail at autonomous navigation

This guide ensures both motors spin at the same speed and encoders accurately report distance.

---

## Calibration Process

### Step 1: Measure Wheel Diameter

**Accuracy required**: ±2mm

**Method 1: Direct Measurement**
```
1. Place wheel on flat surface
2. Mark starting point on wheel rim with marker
3. Roll wheel exactly one complete rotation
4. Mark ending point on surface
5. Measure distance between marks
6. Divide by π (3.14159) to get diameter

Example:
  - Distance rolled: 20.5 cm
  - Diameter = 20.5 / 3.14159 = 6.52 cm
```

**Method 2: Calculated from Circumference**
```
1. Wrap string around wheel exactly once
2. Mark both ends where string meets
3. Remove string and measure marked section
4. This is the circumference
5. Diameter = Circumference / π

Example:
  - Circumference: 20.5 cm
  - Diameter = 20.5 / 3.14159 = 6.52 cm
```

**Record your measurements**:
```
Wheel Diameter:
  Left Wheel:  ______ cm
  Right Wheel: ______ cm
  Difference:  ______ cm

Note: If difference > 5mm, verify wheel mounting
```

**Update Arduino Code**:
```cpp
// In arduino_code.ino
#define WHEEL_DIAMETER 0.0652  // Your measured value in meters (6.52cm = 0.0652m)
```

### Step 2: Measure Wheel Separation

**Accuracy required**: ±1cm

**Definition**: Distance from center of left wheel to center of right wheel

**Method**:
```
1. Mark center of robot (midpoint between wheels)
2. Measure horizontal distance from left wheel center to right wheel center
3. This is wheel separation distance

Example:
  - Measured distance: 15.2 cm
```

**Update Arduino Code**:
```cpp
// In arduino_code.ino
#define WHEEL_SEPARATION 0.152  // Your measured value in meters (15.2cm = 0.152m)
```

### Step 3: Determine Encoder Ticks Per Revolution

**Why**: Each encoder may have different pulse count

**Method 1: Datasheet**
- Quadrature encoders typically have 20-30 ppr (pulses per revolution)
- Check encoder documentation

**Method 2: Experimental Measurement**
```
1. Place robot on elevated platform (wheels off ground, won't move)
2. Connect Arduino and open Serial Monitor (115200 baud)
3. Send motor command: M100100 (both motors forward, medium speed)
4. Spin left wheel by hand slowly (one complete rotation)
5. Note encoder count increase

Example:
  - Encoder started at: 0
  - After 1 wheel rotation: 20 ticks
  - Ticks per revolution: 20
```

**For Quadrature Encoders**:
- Some encoders count both rising edges = 2x ticks
- Others count only one edge = standard count
- Adjust if counts are unexpectedly high

**Update Arduino Code**:
```cpp
// In arduino_code.ino
#define TICKS_PER_REV 20  // Your measured encoder ticks per revolution
```

### Step 4: Speed Matching Test (Bench Test)

**Setup**:
```
1. Elevate robot so wheels spin freely (not touching ground)
2. Secure robot to prevent tipping
3. Connect Arduino to computer
4. Open Serial Monitor at 115200 baud
```

**Test Procedure**:
```
1. Send command: M100100
   (Both motors same PWM value)

2. Monitor encoder output for 10 seconds:
   - Watch encoder L and R tick counts
   - Should increase roughly equally

3. After 10 seconds, stop: M000000

4. Compare final counts:
   - Expected difference: < 5% variation
   - Bad match: > 10% difference = recalibrate
```

**Example Output**:
```
Sent: M100100
Encoder L: 0 → 234 ticks (10 seconds)
Encoder R: 0 → 236 ticks (10 seconds)
Difference: 236 - 234 = 2 ticks = 0.8% error ✓ GOOD
```

### Step 5: Identify Slower Motor

**If motors don't match**:

```
Case 1: Left motor slower
  - Left encoder: 200 ticks
  - Right encoder: 220 ticks
  - Solution: Increase left motor PWM

Case 2: Right motor slower
  - Left encoder: 220 ticks
  - Right encoder: 200 ticks
  - Solution: Increase right motor PWM

Case 3: Both motors too slow
  - Both counts < 150 ticks
  - Solution: Use higher PWM (M150150)
```

### Step 6: PWM Adjustment

**Goal**: Match encoder counts to within 5%

**Method**:
```
Step 1: Record baseline (both motors at M100100)
  Left: 234 ticks
  Right: 236 ticks

Step 2: Calculate adjustment percentage
  % error = (236 - 234) / 235 = 0.8%  ← Already excellent

Step 3: If error > 5%, increase slower motor
  - Left is slower by 5%
  - Increase left PWM by 5%
  - Try M105100 instead of M100100

Step 4: Re-test and iterate
```

**Calibration Values to Track**:
```
At PWM 100/100:
  Left:  ___ ticks
  Right: ___ ticks
  Error: ___ %

If error > 5%:
  Increase slower motor by 2-5% and re-test

Repeat until error < 5%
```

### Step 7: Gear Ratio Compensation (Optional)

If motor speeds don't match even after PWM adjustment, you may need to compensate in software:

```cpp
// In ROS2 Python node (differential_drive.py)

def set_motor_commands(self, v, omega):
    """Calculate motor PWM with calibration factors"""

    # Calibration factors (from testing)
    left_factor = 1.0    # Default
    right_factor = 0.98  # Right motor 2% faster

    # Calculate base PWM
    pwm_left = int(127 + v * 50)
    pwm_right = int(127 + v * 50)

    # Apply calibration
    pwm_left = int(pwm_left * left_factor)
    pwm_right = int(pwm_right * right_factor)

    # Send to Arduino
    self.send_motor_command(pwm_left, pwm_right)
```

---

## On-Floor Movement Test

### Forward/Backward Straight-Line Test

**Purpose**: Verify robot moves in straight line

**Setup**:
1. Clear 2m x 0.5m open space
2. Tape line on floor (or chalk line)
3. Position robot centered on line

**Test Procedure**:
```
1. Command robot forward: rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

2. Observe robot for 2 meters:
   - Should stay within ±10cm of line
   - If drifts left: right motor is faster
   - If drifts right: left motor is faster

3. Measure actual drift:
   Starting position: marked on floor
   Ending position: after 2m forward
   Drift distance: measured perpendicular to motion

4. Calculate drift angle:
   Drift = 15cm off line after 2m
   Drift angle = arctan(0.15/2.0) = 4.3 degrees
```

**Adjustment if Drifting**:
```
If drift angle > 5 degrees:
  1. Identify which motor is faster (visual inspection)
  2. Reduce that motor's PWM in firmware
  3. Re-test until drift < 5 degrees

Before: M100100 caused 15cm drift
Adjust: M10098 (right motor slightly slower)
After re-test: M10098 causes only 3cm drift ✓ Good
```

### Rotation Test

**Purpose**: Verify spinning is centered

**Setup**:
```
1. Mark center point on floor
2. Place robot centered on point
3. Tape line extending from center
```

**Test Procedure**:
```
1. Command robot rotation: rostopic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

2. After 360° rotation:
   - Robot should return to same starting point
   - If offset, motors are mismatched

3. Measure offset:
   Expected: < 5cm displacement
   Bad: > 10cm indicates motor mismatch

4. Calculate accuracy:
   Offset = 3cm after full rotation
   Diameter = 30cm (wheel separation)
   Error = 3 / (π * 0.30) = 3.2% ✓ Acceptable
```

---

## Advanced: Encoder Validation

### Quadrature Decoding Verification

If robot moves at wrong speeds or counts ticks incorrectly:

**Test Encoder Signals**:
```cpp
// Temporary Arduino code to debug encoders
void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
}

void loop() {
  int a = digitalRead(ENCODER_A_PIN);
  int b = digitalRead(ENCODER_B_PIN);
  Serial.print("A=");
  Serial.print(a);
  Serial.print(" B=");
  Serial.println(b);
  delay(10);
}
```

**Expected Output** (while spinning motor A slowly):
```
A=1 B=0
A=1 B=1
A=0 B=1
A=0 B=0
A=1 B=0   ← Returns to start, pattern repeats
...
```

If pattern is wrong or missing, check:
1. Encoder signal wires connected to correct pins
2. Encoder disc not slipping on motor shaft
3. Pull-up resistors present and correct value

---

## Odometry Verification

### Distance Measurement Test

**Purpose**: Verify distance calculations are accurate

**Setup**:
1. Tape 2-meter line on floor
2. Measure with measuring tape

**Test Procedure**:
```bash
# Terminal 1: Launch motor node
ros2 run my_differential_robot motor_command_node

# Terminal 2: Reset odometry and move robot
ros2 service call /reset_odometry std_srvs/Empty

# Send forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop after 2 meters (manually)
# Let robot travel exactly 2 meters by floor measurement

# Check reported odometry
ros2 topic echo /odom
```

**Expected Result**:
```yaml
pose:
  position:
    x: 2.0        # Should be ~2.0 (within ±0.1m)
    y: 0.0        # Should be ~0.0
    z: 0.0
```

**Adjustment if Error Too High**:
If odometry reports 1.85m when robot actually moved 2.0m:
```
Measured error = 2.0 - 1.85 = 0.15m
Error percentage = 0.15 / 2.0 = 7.5%

Likely causes:
1. Wheel diameter slightly wrong
2. Wheel slip on floor
3. Encoder not counting all ticks

Fix:
1. Remeasure wheel diameter precisely
2. Test on rougher surface (less slip)
3. Check encoder connection
```

---

## Temperature Compensation (Advanced)

Motor speeds vary with temperature. For precise odometry:

```cpp
// Temperature compensation (if thermistor available)
float compensate_pwm(int pwm_input, float temp_celsius) {
  // Temperature affects motor speed
  // Typical: -0.2% per degree C

  float reference_temp = 25.0;  // Room temperature
  float temp_delta = temp_celsius - reference_temp;
  float compensation = 1.0 + (temp_delta * -0.002);

  return pwm_input * compensation;
}
```

---

## Calibration Data Sheet

**Print and fill this out**:

```
DIFFERENTIAL DRIVE ROBOT CALIBRATION RECORD
Date: __/__/____
Robot ID: ________________

MEASUREMENTS:
  Left Wheel Diameter:    _______ cm
  Right Wheel Diameter:   _______ cm
  Wheel Separation:       _______ cm
  Encoder Ticks/Rev:      _______

BENCH TEST (Motors elevated):
  Command: M100100 (10 seconds)
  Left Encoder:  _______ ticks
  Right Encoder: _______ ticks
  % Difference:  _______ %

  If > 5%, adjustment needed:
    Slower motor: [ ] Left [ ] Right
    New command: M____M____

FLOOR TEST (2 meter forward):
  Starting position: (0, 0)
  Ending position: (____m, ____m)
  Drift distance: ____m
  Drift angle: ____°
  Status: [ ] Pass (< 5°) [ ] Fail (> 5°)

ODOMETRY TEST (2 meters):
  Measured distance: 2.0 m
  Reported distance: ____m
  Error: ____m (____%)
  Status: [ ] Pass (< 5%) [ ] Fail (> 5%)

ROTATION TEST (360°):
  Starting position: marked
  Ending offset: ____cm
  Status: [ ] Pass (< 5cm) [ ] Fail (> 5cm)

NOTES:
_________________________________
_________________________________
_________________________________

Signed: ________________ Date: __/__/____
```

---

## ROS2 Parameter Configuration

Once calibrated, save parameters in YAML file:

**File**: `calibration.yaml`
```yaml
differential_drive:
  wheel_diameter: 0.065      # meters
  wheel_separation: 0.15      # meters
  ticks_per_revolution: 20

  # Motor PWM calibration factors
  motor_left_factor: 1.0      # No adjustment needed
  motor_right_factor: 0.98    # 2% slower, needs boost

  # Encoder calibration
  encoder_l_direction: 1      # 1 = normal, -1 = reversed
  encoder_r_direction: 1

  # Safety
  max_linear_velocity: 1.0    # m/s
  max_angular_velocity: 2.0   # rad/s
  watchdog_timeout: 1000      # ms
```

**Use in ROS2 Node**:
```python
from ament_index_python.packages import get_package_share_directory

def load_calibration():
    pkg_dir = get_package_share_directory('my_differential_robot')
    config_file = os.path.join(pkg_dir, 'config', 'calibration.yaml')

    with open(config_file) as f:
        config = yaml.safe_load(f)

    return config['differential_drive']
```

---

## Troubleshooting Calibration

| Problem | Cause | Solution |
|---------|-------|----------|
| Motors unequal speed | Manufacturing tolerance, gearbox wear | Adjust PWM in software |
| Encoder not counting | Disc slipping on shaft | Tighten encoder coupling |
| Odometry error > 10% | Wrong wheel diameter | Remeasure using actual rolling distance |
| Robot drifts in circles | Wheel slip or asymmetric traction | Test on different surface |
| Rotation inaccurate | Wheel separation measured wrong | Remeasure using actual robot pivot point |

---

## Next Steps

1. **Complete all calibration tests** - use datasheet above
2. **Update Arduino parameters** with your measurements
3. **Record calibration factors** in ROS2 config file
4. **Test autonomous navigation** with calibrated values
5. **Iterate if needed** - verify with multiple test runs

---

## References

- [Differential Drive Kinematics](../README.md#step-2-understand-differential-drive-kinematics)
- [Motor Control Firmware](arduino_code.ino)
- [ROS2 Integration](ros2_integration.md)
- [Testing Guide](testing_guide.md)

---

**Congratulations!** Your robot is now calibrated and ready for accurate navigation.
