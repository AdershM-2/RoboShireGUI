# Line Following Robot - Complete Hardware Example

**Difficulty**: Beginner to Intermediate
**Time**: 2-3 hours total
**Cost**: ~$30-50 USD
**Skills Learned**: Sensor arrays, PID control, reactive behavior, vision-based navigation

---

## Project Overview

Build an autonomous robot that follows a black line on white floor using an IR sensor array. This teaches fundamental robotics concepts: sensor integration, feedback control, and autonomous decision-making.

### What You'll Build

```
┌──────────────────────────┐
│   Line Following Robot   │
│                          │
│   Front:                 │
│  ┌──────────────────┐   │
│  │ IR Sensor Array  │   │
│  │ (5-8 sensors)    │   │
│  └────────┬─────────┘   │
│           │              │
│  Left Motor  Right Motor │
│    │           │        │
│    └─────┬─────┘        │
│          │              │
│      Arduino Uno        │
│                          │
│  Control Logic:         │
│  - Read sensors         │
│  - Calculate error      │
│  - PID control          │
│  - Drive motors         │
└──────────────────────────┘

Behavior:
  - Line centered under middle sensors
    → Both motors equal speed (go straight)

  - Line on left side
    → Reduce left motor, boost right motor (turn right)

  - Line on right side
    → Reduce right motor, boost left motor (turn left)

  - No line detected
    → Emergency behavior (back up and search)
```

### Key Features

- **IR Sensor Array**: 5-8 front-facing sensors for line detection
- **PID Control**: Proportional-Integral-Derivative feedback control
- **Reactive Behavior**: Real-time response to sensor input
- **Motor Speed Control**: PWM-based motor speed adjustment
- **Serial Monitoring**: Real-time sensor data visualization
- **ROS2 Integration**: Optional publish sensor data and receive commands
- **Beginner-Friendly**: Simple electronics, easy to debug

### Learning Outcomes

After completing this project:
- ✅ Understand IR sensor operation and arrays
- ✅ Implement PID control algorithms
- ✅ Debug sensor-based behavior
- ✅ Optimize tuning parameters
- ✅ Integrate with ROS2 topics
- ✅ Build autonomous reactive systems

---

## Project Files

| File | Description |
|------|-------------|
| [hardware_bom.md](hardware_bom.md) | Bill of materials |
| [sensor_array_setup.md](sensor_array_setup.md) | Sensor mounting and calibration |
| [arduino_code.ino](arduino_code.ino) | Line following firmware with PID |
| [pid_tuning_guide.md](pid_tuning_guide.md) | Tuning P, I, D parameters |
| [ros2_integration.md](ros2_integration.md) | Publishing to ROS2 topics |

---

## Quick Start

### Prerequisites Checklist

- [ ] RoboShire v2.4.0+ installed
- [ ] Arduino IDE installed
- [ ] Hardware components purchased (see [BOM](hardware_bom.md))
- [ ] Test track (tape or white/black surface)
- [ ] USB cable for Arduino

### 30-Second Overview

1. **Build chassis** (30 minutes) - Same as differential drive robot
2. **Mount sensor array** (20 minutes) - Align 5-8 IR sensors at front
3. **Wire electronics** (30 minutes) - Arduino + motors + sensors
4. **Flash firmware** (5 minutes) - Upload line following code
5. **Calibrate sensors** (10 minutes) - Test detection on actual track
6. **Tune PID parameters** (30 minutes) - Get smooth following behavior

**Total time**: ~2-2.5 hours for basic operation

---

## Detailed Step-by-Step

### Step 1: Get Mechanical Chassis

Use the same robot chassis as the Differential Drive example:
- 2WD robot platform with motors and wheels
- Mounting space for sensors and Arduino
- See [Differential Drive Robot](../differential_drive_robot/README.md) for assembly

**Key Difference**:
- Differential drive needs encoders and IMU
- Line follower only needs IR sensors
- Can skip encoder/IMU to save cost

### Step 2: Design and Mount Sensor Array

**IR Sensor Placement**:

```
Front view of robot:
┌──────────────────────────┐
│   FRONT OF ROBOT         │
│                          │
│  S1 S2 S3 S4 S5 S6 S7 S8  ← Sensors (5-8 typical)
│  ▯  ▯  ▯  ▯  ▯  ▯  ▯  ▯  ← Center crosshair for reference
│  └──────────┬──────────┘  ← Centerline
│             │
│       Mounting bar
│
└──────────────────────────┘

Spacing: 1-2 cm between sensors (depends on track width)
Height: 0.5-1 cm above ground
Angle: 45° downward to see line clearly
```

**Sensor Types**:

**Option 1: Analog Reflectance Sensors (Recommended)**
- Examples: QTR-8A (8 analog sensors), QTR-5A (5 sensors)
- Cost: $15-25
- Advantages: Easy to use, multiple sensors in one module
- Connects directly to Arduino analog inputs

**Option 2: Individual IR Modules**
- Example: Arduino IR obstacle avoidance module
- Cost: $1-2 per module
- Advantages: Flexible placement, independent control
- Disadvantage: Need individual wiring for each sensor

**Option 3: Digital Line Sensors**
- Example: MQ-5, TCS230 color sensor array
- Cost: $5-15
- Advantages: Better precision, built-in calibration
- Disadvantage: Slower response than analog

**Recommendation**: Use QTR-8A (8 analog sensors in one module)

**Installation**:
```
1. Mount sensor module under robot front edge
   - Use 3D-printed bracket or zip ties
   - Position 0.5-1 cm above ground

2. Orientation:
   - Point straight down at track
   - LEDs should have clear view of ground
   - No obstruction from body or wheels

3. Wiring (QTR-8A):
   VCC → Arduino 5V
   GND → Arduino GND
   OUT0 → Arduino A0
   OUT1 → Arduino A1
   ... (up to 8 sensors)
```

### Step 3: Build Motor Control

Same as differential drive robot. See [Differential Drive Wiring](../differential_drive_robot/wiring_diagram.md).

**Quick Summary**:
- Arduino Uno/Mega
- L298N motor driver
- 2x DC motors
- 2x AA or 4x AA battery holder
- Optional: Caster wheel

### Step 4: Wire Sensor Array

**QTR-8A Sensor Array Connections**:

```
QTR-8A Module:
┌─────────────────────────────┐
│ VCC GND OUT0 OUT1 ... OUT7  │
│  │   │    │    │       │    │
│  │   │    │    │       │    │
└──┼───┼────┼────┼───────┼────┘
   │   │    │    │       │
   │   │    │    │       └─→ Arduino A7
   │   │    │    └─→ Arduino A1
   │   │    └─→ Arduino A0
   │   │
   │   └─→ Arduino GND
   └─→ Arduino 5V
```

**Complete Wiring**:

| QTR-8A | Arduino | Purpose |
|--------|---------|---------|
| VCC | 5V | Power |
| GND | GND | Ground |
| OUT0 | A0 | Sensor 1 analog value |
| OUT1 | A1 | Sensor 2 analog value |
| OUT2 | A2 | Sensor 3 analog value |
| OUT3 | A3 | Sensor 4 analog value |
| OUT4 | A4 | Sensor 5 analog value |
| OUT5 | A5 | Sensor 6 analog value |
| OUT6 | A6 | Sensor 7 analog value |
| OUT7 | A7 | Sensor 8 analog value |

**Alternative: Individual Sensors**

If using separate IR modules:
```
Each IR Sensor:
  VCC → Arduino 5V
  GND → Arduino GND
  OUT → Arduino AN (analog pin A0-A7)

Example (5 sensors):
  Sensor L2 → A0  (far left)
  Sensor L1 → A1  (left)
  Sensor C  → A2  (center)
  Sensor R1 → A3  (right)
  Sensor R2 → A4  (far right)
```

### Step 5: Upload Firmware

**Line Following Algorithm**:

```cpp
Pseudo-code:

LOOP:
  1. Read all 8 analog sensors
  2. Calculate weighted position of line
     position = (sum of (sensor_value × position_weight)) / sum of sensors

  3. Calculate error:
     error = position - CENTER

  4. PID control:
     output = Kp × error + Ki × integral(error) + Kd × derivative(error)

  5. Motor control:
     left_speed = BASE_SPEED - output
     right_speed = BASE_SPEED + output

  6. Drive motors with calculated speeds
  7. Sleep 50ms, repeat
```

**Complete Arduino Code** (see [arduino_code.ino](arduino_code.ino) for full implementation)

Key sections:
- Sensor calibration
- PID controller
- Motor speed adjustment
- Serial debugging output

### Step 6: Calibrate Sensor Array

**Calibration Procedure**:

1. **Baseline Measurement**:
```
  Power on robot
  Place on white surface
  Record sensor values (should be low, ~50-200)
  Place on black line
  Record sensor values (should be high, ~700-1000)
```

2. **Thresholding**:
```cpp
// In Arduino code
#define WHITE_THRESHOLD 300
#define BLACK_THRESHOLD 600

// Sensor reading > threshold = black line detected
if (sensor_value > BLACK_THRESHOLD) {
  line_detected = true;
}
```

3. **Sensor Placement Verification**:
- Test on actual track
- All sensors should see line when robot is centered
- Center sensor should have strongest signal
- Outer sensors should detect when robot drifts

### Step 7: Flash Firmware

1. Connect Arduino via USB
2. Open Arduino IDE
3. Open [arduino_code.ino](arduino_code.ino)
4. Select board: Tools → Board → Arduino Uno (or Mega if used)
5. Select port: Tools → Port → /dev/ttyACM0
6. Click **Upload**

**Verify Upload**:
```
Serial Monitor output should show:
[INFO] Line Follower Robot Initialized
[INFO] Sensor calibration: WHITE=150 BLACK=850
[INFO] Ready to follow line
[INFO] Place robot on track...
```

### Step 8: Test on Track

**Test Track Setup**:

```
Create test track:

Option 1: Tape on white floor
┌────────────────────────────────┐
│ White floor                    │
│                                │
│  ┌──────────────────────────┐  │
│  │ Black electrical tape    │  │
│  │ (width: 2-3 inches)      │  │
│  └──────────────────────────┘  │
│                                │
└────────────────────────────────┘

Option 2: White paper on black floor
┌────────────────────────────────┐
│ Black floor/cardboard          │
│                                │
│  ┌──────────────────────────┐  │
│  │ White paper path         │  │
│  │ (width: 2-3 inches)      │  │
│  └──────────────────────────┘  │
│                                │
└────────────────────────────────┘

Option 3: Commercial track
- RoboShop sells ready-made tracks
- Consistent coloring
- Better for testing
```

**Test Procedure**:

1. **Straight Line Test**:
```
  ┌─────────────────┐
  │ ▲ Robot start   │
  │ │               │
  │ └───────────┘   │ Expected: Robot stays on line
  │               │
  └─────────────────┘
```

2. **Curve Test**:
```
  ┌──────────────────┐
  │ ▲ Robot starts   │
  │ │                │
  │  ╭──────────┐    │ Expected: Smooth curve following
  │  │          │    │
  │  └──────────┘    │
  │                  │
  └──────────────────┘
```

3. **Sharp Turn Test**:
```
  ┌──────────────┬──────┐
  │ ▲ Start      │      │ Expected: Possible overshoot,
  │ │            │      │          then stabilize
  │ └────────────┘      │
  │                     │
  └─────────────────────┘
```

### Step 9: Tune PID Parameters

**See [PID Tuning Guide](pid_tuning_guide.md) for detailed tuning**

**Quick Tuning Process**:

1. **Start with P only** (I=0, D=0):
   - Start with Kp=1
   - Test on straight line
   - Increase Kp until slight oscillation
   - Kp where oscillates slightly = good starting point (~5-10)

2. **Add Derivative** (I=0, D=value):
   - Add Kd to dampen oscillations
   - Typical: Kd = Kp/10 (~0.5-1)
   - Reduces overshoot

3. **Add Integral** (Kp and Kd, then Ki):
   - Small Ki initially (0.1)
   - Helps with systematic offset
   - Too much causes integrator windup

**Example Tuning Sequence**:

```
Initial: Kp=8, Ki=0, Kd=0
  Result: Oscillates heavily

Adjust: Kp=6, Ki=0, Kd=1
  Result: Better, some overshoot on curves

Tune: Kp=5, Ki=0.1, Kd=1.5
  Result: Smooth line following ✓

Final: Kp=5, Ki=0.15, Kd=1.8
  Result: Stable, fast response, handles curves well ✓✓
```

---

## PID Control Deep Dive

### How PID Works

```
Error = desired_position - current_position

output = Kp × error              Proportional term
       + Ki × ∫error dt          Integral term
       + Kd × (d(error)/dt)      Derivative term
```

**Proportional (P)**:
- Proportional to current error
- Too high: Oscillates
- Too low: Sluggish response

**Integral (I)**:
- Accumulates error over time
- Fixes systematic offset
- Too high: Integrator windup

**Derivative (D)**:
- Responds to rate of change
- Dampens oscillations
- Too high: Noisy/jerky

### Implementation

```cpp
// In loop()
error = desired - current;
integral += error × dt;
derivative = (error - last_error) / dt;

output = Kp × error + Ki × integral + Kd × derivative;

last_error = error;
```

---

## ROS2 Integration (Optional)

**Publish Sensor Data to ROS2**:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import struct

class LineFollowerSensorNode(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Serial connection
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1.0)

        # Publisher for sensor data
        self.sensor_pub = self.create_publisher(Joy, '/sensors', 10)

        # Timer for reading sensors
        self.timer = self.create_timer(0.05, self.publish_sensors)

    def publish_sensors(self):
        if self.serial.in_waiting >= 16:
            data = self.serial.read(16)

            # Parse 8 sensor values (2 bytes each)
            sensors = []
            for i in range(8):
                val = struct.unpack('>H', data[i*2:i*2+2])[0]
                sensors.append(val / 1000.0)  # Normalize to 0-1

            # Publish as Joy message (compatible with standard ROS)
            msg = Joy()
            msg.axes = sensors
            self.sensor_pub.publish(msg)

def main():
    rclpy.init()
    node = LineFollowerSensorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## Vision-Based Alternative

**Using USB Camera Instead of IR Sensors**:

```python
import cv2
import numpy as np

def find_line_position(frame):
    """Find black line position in image"""
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for black color
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 255, 100])

    # Threshold
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Find moments
    M = cv2.moments(mask)

    # Calculate centroid
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx
    else:
        return None  # No line found
```

**Advantages**:
- More robust to lighting conditions
- Can detect curved lines better
- Potential for color differentiation

**Disadvantages**:
- More computationally intensive
- Requires camera on robot
- Higher latency

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot goes in circles | PID values wrong | Re-tune with slower speeds first |
| Sensors always high/low | Thresholds wrong | Recalibrate on actual track |
| One motor faster | Motor mismatch | Add speed adjustment factor |
| Robot oscillates | Kd too low or Kp too high | Increase Kd, decrease Kp |
| Robot doesn't turn | I gain too low | Increase Ki slightly |
| Lost connection to Arduino | Serial port issue | Check `/dev/ttyACM0` permissions |

---

## Performance Metrics

**Expected Performance**:
| Metric | Value | Notes |
|--------|-------|-------|
| Tracking accuracy | ±2-5 cm | Depends on line width |
| Speed on track | 0.3-0.5 m/s | ~30 RPM motors |
| Curve following | 90° turns | No obvious overshoot |
| Response time | <100 ms | Sensor to motor |
| Battery runtime | 1-3 hours | Depends on battery capacity |

---

## Educational Applications

### Classroom Activity (2 Hours)

**Learning Objectives**:
1. Understand sensor-based control
2. Implement PID feedback control
3. Tune control parameters
4. Debug autonomous behavior

**Activity Structure**:
- Hour 1: Build and calibrate robot
- Hour 2: Tune PID on test track

**Assessment**:
- Robot follows line without losing it
- Completes track in < 30 seconds
- Smooth curves without excessive oscillation

### Robotics Competition

**Challenge**: "Line Following Race"
- Track with sharp turns, curves, dead ends
- Fastest robot wins
- Teams compete on their tuning

**Variants**:
- "Obstacle Avoidance": Add objects on line
- "Multi-line": Follow specific colored line
- "Speed Challenge": Maintain line at maximum speed

---

## Next Steps

1. **Master Straight-Line Following**
2. **Optimize Curve Performance** (see PID tuning)
3. **Add Additional Sensors**:
   - Ultrasonic for obstacle detection
   - Gyro for drift compensation
4. **Upgrade to Differential Drive** with encoders and IMU
5. **Add Computer Vision** for advanced line detection

---

## References

- [Hardware BOM](hardware_bom.md)
- [Sensor Array Setup](sensor_array_setup.md)
- [PID Tuning Guide](pid_tuning_guide.md)
- [Arduino Code](arduino_code.ino)
- PID Control Tutorial: [w3schools](https://www.w3schools.com/whatis/whatis_pid.asp)

---

## FAQ

**Q: What's a good track width?**
A: 2-3 inches (5-8 cm) works well for most sensor arrays.

**Q: Can I use a USB camera instead of IR sensors?**
A: Yes! See Vision-Based Alternative section.

**Q: What if sensors drift over time?**
A: Implement periodic recalibration or use white balance correction.

**Q: Can I make it faster?**
A: Increase PWM, but verify PID tuning still works.

**Q: How do I add obstacle avoidance?**
A: Add ultrasonic sensor and check for obstacles during line following.

---

## License

This example is licensed under Apache 2.0.

---

**Ready to build?** Start with the [Bill of Materials](hardware_bom.md)!
