# Differential Drive Robot - Complete Hardware Example

**Difficulty**: Intermediate
**Time**: 3-4 hours total
**Cost**: ~$50-80 USD
**Skills Learned**: Motor control, odometry, IMU integration, sensor fusion, navigation

---

## Project Overview

Build a mobile robot using two DC motors with differential drive, an IMU for orientation sensing, and encoders for odometry. This is the foundation for any mobile robot platform and teaches essential robotics concepts.

### What You'll Build

```
┌────────────────────────────────────┐
│     Differential Drive Robot       │
│                                    │
│    ┌──────────────────────────┐   │
│    │   Arduino Mega (MCU)     │   │
│    │                          │   │
│    │ ┌─────────────────────┐  │   │
│    │ │   L298N Driver      │  │   │
│    │ │  (Dual Motor Ctrl)  │  │   │
│    │ └─────┬───────┬───────┘  │   │
│    │       │       │          │   │
│    │   DC Motor    DC Motor   │   │
│    │  (Left)       (Right)    │   │
│    │     │            │       │   │
│    │     ↓            ↓       │   │
│    │  Encoder      Encoder    │   │
│    │                          │   │
│    │  ┌──────────────────┐    │   │
│    │  │   MPU6050 IMU    │    │   │
│    │  │ (6-axis, I2C)    │    │   │
│    │  └──────────────────┘    │   │
│    │                          │   │
│    │  Caster Wheel (3rd)     │   │
│    └──────────────────────────┘   │
│                                    │
└────────────────────────────────────┘
                 │
        ┌────────┴────────┐
        │                 │
   ┌────▼────┐      ┌─────▼─────┐
   │  Odometry│      │   IMU     │
   │Calculation│     │ Fusion    │
   └────┬────┘      └─────┬─────┘
        │                 │
        └────────┬────────┘
                 ↓
          ┌─────────────┐
          │ ROS2 Nav    │
          │ Stack       │
          └─────────────┘
```

### Key Features

- **Dual Motor Control**: Independent speed control via L298N PWM driver
- **Odometry**: Wheel encoder integration for position tracking
- **IMU Fusion**: 6-axis motion tracking with gyroscope and accelerometer
- **ROS2 Integration**: Standard nav_msgs for navigation
- **Differential Drive Kinematics**: Implemented in firmware
- **Emergency Stop**: Hardware and software safety mechanisms
- **Beginner-Friendly Assembly**: No soldering required (breadboard possible)

### Learning Outcomes

After completing this project:
- ✅ Understand DC motor control and PWM
- ✅ Implement quadrature encoder reading
- ✅ Integrate IMU sensors via I2C
- ✅ Calculate robot odometry from wheel data
- ✅ Use ROS2 control_msgs for motor commands
- ✅ Implement differential drive kinematics
- ✅ Debug hardware communication issues
- ✅ Test with RoboShire Hardware Testing Panel

---

## Project Files

| File | Description |
|------|-------------|
| [hardware_bom.md](hardware_bom.md) | Complete bill of materials with suppliers |
| [wiring_diagram.md](wiring_diagram.md) | Detailed pin connections and circuit layout |
| [arduino_code.ino](arduino_code.ino) | Complete microcontroller firmware |
| [motor_calibration.md](motor_calibration.md) | Calibrating motor speeds and encoder values |
| [odometry_setup.md](odometry_setup.md) | Odometry calculations and ROS2 integration |
| [ros2_integration.md](ros2_integration.md) | Motor command node and transforms |
| [testing_guide.md](testing_guide.md) | Verification procedures and troubleshooting |

---

## Quick Start

### Prerequisites Checklist

- [ ] RoboShire v2.4.0+ installed (includes Hardware Testing Panel)
- [ ] ROS2 Humble+ installed
- [ ] Arduino IDE installed
- [ ] Hardware components purchased (see [BOM](hardware_bom.md))
- [ ] USB cable for Arduino Mega

### 30-Second Overview

1. **Assemble hardware** (30 minutes) - Follow [wiring diagram](wiring_diagram.md)
2. **Flash firmware** (5 minutes) - Upload [arduino_code.ino](arduino_code.ino)
3. **Verify connection** (5 minutes) - Use RoboShire Hardware Testing Panel
4. **Build ROS2 package** (5 minutes) - Workflow Wizard + code generation
5. **Test movement** (10 minutes) - Keyboard teleop or autonomous navigation

**Total time**: ~1 hour for basic functionality

---

## Detailed Step-by-Step

### Step 1: Purchase Hardware

See [Bill of Materials](hardware_bom.md) for:
- Exact part numbers and suppliers
- Cost breakdown (minimal, recommended, premium builds)
- Alternative motor options
- Battery selection guidance

**Typical Bill of Materials**:
- Arduino Mega 2560: $8
- L298N Motor Driver: $3
- 2x DC Motors (3-6V, ~100 RPM): $8
- 2x Quadrature Encoders: $6
- MPU6050 IMU: $3
- Robot chassis kit: $15
- Batteries, wiring, breadboard: $8
- **Total**: ~$50 for basic, ~$80 for complete

### Step 2: Understand Differential Drive Kinematics

**Key Concept**: A differential drive robot moves by controlling two independent wheels.

```
Velocity Commands:
  v = (v_left + v_right) / 2     [Linear velocity]
  ω = (v_right - v_left) / L     [Angular velocity]

Where L = wheel separation distance (cm)

Inverse (given v and ω, calculate motor speeds):
  v_left = v - (ω * L / 2)
  v_right = v + (ω * L / 2)
```

This robot will:
- Move **forward**: Both motors at same speed
- Move **backward**: Both motors reverse
- **Turn left**: Right motor faster than left
- **Turn right**: Left motor faster than right
- **Spin in place**: Motors opposite direction

### Step 3: Assemble Robot Chassis

Follow [Wiring Diagram](wiring_diagram.md):

**Mechanical Assembly** (30 minutes):
1. Mount motors to chassis (typically screws or brackets)
2. Attach wheels to motor shafts
3. Mount caster wheel at front or back
4. Attach encoder discs to motor shafts
5. Mount breadboard for electronics

**Electrical Assembly** (20 minutes):
1. Wire motors to L298N driver
2. Connect L298N to Arduino (PWM pins)
3. Connect encoders to Arduino (interrupt pins)
4. Connect MPU6050 via I2C
5. Battery connections with power distribution

**Safety Check**:
- Double-check all power/ground connections
- Verify encoder polarity (both should rotate same direction)
- Test motor rotation direction (reverse if needed)

### Step 4: Upload Firmware

**Using Arduino IDE**:

1. Open [arduino_code.ino](arduino_code.ino) in Arduino IDE
2. Install required libraries:
   - MPU6050 library (by Jeff Rowberg)
   - Or use basic I2C commands (code includes both)
3. Select board: Tools → Board → Arduino Mega 2560
4. Select port: Tools → Port → /dev/ttyACM0 (or detected)
5. Click **Upload**

**Verification**:
```
Output should show:
[INFO] Initializing motors...
[INFO] Initializing MPU6050...
[INFO] Ready for commands on /motor_commands topic
```

### Step 5: Test Hardware with RoboShire Panel

**New in v2.4.0: Hardware Testing Panel**

```bash
# In RoboShire GUI:
# Press Ctrl+T to open Hardware Testing Panel
```

**Panel Features**:
- Real-time sensor monitoring (IMU accel, gyro)
- Motor speed slider (0-255 PWM)
- Emergency stop button (red)
- Encoder reading display
- Speed calculation (RPM, m/s)

**Quick Test**:
1. Launch panel (Ctrl+T)
2. Move motor slider slowly
3. Verify motors spin smoothly
4. Check encoder values increment
5. Verify IMU updates

**GUI Enhancement - Motor Testing**:
```
┌─ Hardware Testing Panel ─────┐
│                              │
│ Device: Arduino Mega         │
│ Port: /dev/ttyACM0          │
│ Status: Connected            │
│                              │
│ Motors:                      │
│ ┌─ Left Motor ──────────┐   │
│ │ PWM: 0   [████ ]100% │   │
│ │ Speed: 0 RPM          │   │
│ │ Direction: STOP       │   │
│ └───────────────────────┘   │
│ ┌─ Right Motor ─────────┐   │
│ │ PWM: 0   [████ ]100% │   │
│ │ Speed: 0 RPM          │   │
│ │ Direction: STOP       │   │
│ └───────────────────────┘   │
│                              │
│ Emergency Stop: [     STOP   ]
│                              │
│ Sensors:                     │
│ Encoder L: 1234 ticks        │
│ Encoder R: 1231 ticks        │
│ Accel X: 0.02 m/s²           │
│ Gyro Z: 1.5 deg/s            │
│                              │
└──────────────────────────────┘
```

### Step 6: Create ROS2 Package

**Using Workflow Wizard**:
1. Launch RoboShire
2. Tools → Workflow Wizard
3. Step 1: Select "With URDF (mobile robot)"
4. Step 2: Select "Differential Drive Robot" preset
5. Configure project name: `my_differential_robot`
6. Generate package

**Manual Creation**:
```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace/src
ros2 pkg create --build-type ament_python my_differential_robot \
  --dependencies rclpy geometry_msgs nav_msgs sensor_msgs std_msgs
```

### Step 7: Implement Motor Command Node

**ROS2 Topics**:
- **Subscribe**: `/cmd_vel` (geometry_msgs/Twist) - velocity commands
- **Publish**: `/odom` (nav_msgs/Odometry) - odometry data
- **Publish**: `/imu/data` (sensor_msgs/Imu) - IMU measurements

**Quick Motor Command Node**:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import struct
import time

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive')

        # Parameters
        self.wheel_separation = 0.15  # 15cm between wheels
        self.wheel_radius = 0.03      # 3cm wheel radius
        self.ticks_per_rev = 20       # Encoder ticks per revolution

        # Serial connection
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer for reading sensors
        self.timer = self.create_timer(0.05, self.read_sensors)  # 20Hz

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_tick_l = 0
        self.last_tick_r = 0
        self.last_time = time.time()

    def cmd_vel_callback(self, msg):
        """Convert /cmd_vel to motor commands"""
        v = msg.linear.x
        omega = msg.angular.z

        # Calculate wheel velocities
        v_left = v - (omega * self.wheel_separation / 2)
        v_right = v + (omega * self.wheel_separation / 2)

        # Convert to PWM (0-255)
        pwm_left = int(127 + v_left * 50)  # Scale factor
        pwm_right = int(127 + v_right * 50)

        # Clamp to valid range
        pwm_left = max(0, min(255, pwm_left))
        pwm_right = max(0, min(255, pwm_right))

        # Send to Arduino
        self.send_motor_command(pwm_left, pwm_right)

    def send_motor_command(self, pwm_left, pwm_right):
        """Send motor PWM values to Arduino"""
        cmd = struct.pack('BBB', 0x01, pwm_left, pwm_right)
        self.serial.write(cmd)

    def read_sensors(self):
        """Read encoder and IMU data from Arduino"""
        if self.serial.in_waiting >= 20:
            data = self.serial.read(20)

            # Parse encoder data (2 bytes each)
            tick_l = struct.unpack('H', data[1:3])[0]
            tick_r = struct.unpack('H', data[3:5])[0]

            # Parse IMU data (6x2 bytes)
            accel_x = struct.unpack('h', data[5:7])[0] / 16384.0
            accel_y = struct.unpack('h', data[7:9])[0] / 16384.0
            accel_z = struct.unpack('h', data[9:11])[0] / 16384.0
            gyro_x = struct.unpack('h', data[11:13])[0] / 131.0
            gyro_y = struct.unpack('h', data[13:15])[0] / 131.0
            gyro_z = struct.unpack('h', data[15:17])[0] / 131.0

            # Update odometry
            self.update_odometry(tick_l, tick_r)

            # Publish IMU
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            self.imu_pub.publish(imu_msg)

    def update_odometry(self, tick_l, tick_r):
        """Calculate odometry from encoder ticks"""
        current_time = time.time()
        dt = current_time - self.last_time

        # Calculate distance traveled
        delta_l = (tick_l - self.last_tick_l) / self.ticks_per_rev
        delta_r = (tick_r - self.last_tick_r) / self.ticks_per_rev

        distance_l = delta_l * 2 * 3.14159 * self.wheel_radius
        distance_r = delta_r * 2 * 3.14159 * self.wheel_radius

        # Update position
        distance = (distance_l + distance_r) / 2
        delta_theta = (distance_r - distance_l) / self.wheel_separation

        self.x += distance * cos(self.theta)
        self.y += distance * sin(self.theta)
        self.theta += delta_theta

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2)
        odom.pose.pose.orientation.w = cos(self.theta / 2)
        self.odom_pub.publish(odom)

        # Update last values
        self.last_tick_l = tick_l
        self.last_tick_r = tick_r
        self.last_time = current_time

def main():
    rclpy.init()
    node = DifferentialDriveNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Step 8: Build and Test Navigation

**Build Package**:
```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace
colcon build --packages-select my_differential_robot
source install/setup.bash
```

**Test Keyboard Teleop**:
```bash
# Terminal 1: Launch motor control node
ros2 run my_differential_robot motor_command_node

# Terminal 2: Start keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel

# Terminal 3: Verify odometry
ros2 topic echo /odom
```

**Verification Checklist**:
- [ ] Motors respond to keyboard commands
- [ ] Robot moves forward with 'w' key
- [ ] Robot turns with 'a' and 'd' keys
- [ ] Odometry updates as robot moves
- [ ] IMU data stream active

### Step 9: Calibration and Tuning

See [Motor Calibration Guide](motor_calibration.md) for:
- Motor speed matching (left vs right)
- Wheel diameter measurement
- Encoder validation
- Dead zone elimination

### Step 10: Autonomous Navigation

**Option 1: Simple Waypoint Following**
```bash
ros2 run nav2_simple_commander nav_to_pose_client --pose "x:1.0 y:0.0 theta:0"
```

**Option 2: Full Navigation Stack**
See [ROS2 Integration Guide](ros2_integration.md) for setup instructions

---

## Hardware Safety

### Critical Safety Mechanisms

1. **Emergency Stop Hardware**
   - Pushbutton directly cuts power to motor driver
   - Implemented in Arduino firmware
   - Uses software watchdog (1 second timeout)

2. **Voltage Monitoring**
   - Arduino monitors battery voltage
   - Reduces speed if voltage drops
   - Stops motors if voltage too low

3. **Motor Current Limits**
   - L298N has thermal protection
   - Set maximum PWM to 200 (not 255)
   - Monitor for thermal shutdown

4. **Serial Timeout**
   - If no command received for 1 second, stop motors
   - Prevents runaway robot if connection lost

### Pre-Flight Checklist

- [ ] All wires secure, no loose connections
- [ ] Motor shafts spin freely
- [ ] Battery fully charged
- [ ] Arduino power LED on
- [ ] Serial connection working (check in Hardware Panel)
- [ ] Encoders ticking (verify with panel)
- [ ] IMU responding (panel shows values)
- [ ] Robot in safe area (open space, obstacles removed)
- [ ] Emergency stop functional
- [ ] No weight on motors (test baseline first)

---

## Troubleshooting

### Issue: Motors won't move

**Symptoms**: Motor pins get 5V but motors don't spin

**Solutions**:
1. Check L298N is powered from battery (not Arduino 5V)
2. Verify motor connections to OUT1/OUT2, OUT3/OUT4
3. Test with Arduino's analogWrite() directly (no ROS2)
4. Check motor direction (may need wires reversed)
5. Measure motor current with multimeter (should be <1A at startup)

### Issue: Encoders not reading

**Symptoms**: Encoder values stay at 0

**Solutions**:
1. Verify encoder wires connected to interrupt pins (2, 3 on Mega)
2. Test encoder output with multimeter (should see 0-5V transitions)
3. Check encoder disc alignment (should rotate freely with motor shaft)
4. Verify pullup resistors installed (10k from signal to 5V)
5. Use hardware test panel to check encoder readings

### Issue: IMU not working

**Symptoms**: IMU data all zeros or doesn't update

**Solutions**:
1. Verify I2C wiring (SDA pin 20, SCL pin 21 on Mega)
2. Add I2C pullup resistors (4.7k from SDA/SCL to 3.3V)
3. Use I2C scanner code to find device address
4. Check for address conflicts with other I2C devices
5. Verify 3.3V power supply (not 5V)

### Issue: Robot moves in circles

**Symptoms**: Moving forward but drifting left or right

**Solutions**:
1. Calibrate motor speeds (see [Motor Calibration](motor_calibration.md))
2. Check wheel sizes are equal (measure diameter)
3. Verify encoder tick counts are equal
4. Check for wheel friction or binding
5. Measure wheel separation distance accurately

### Issue: ROS2 topics don't update

**Symptoms**: Topics exist but no messages appear

**Solutions**:
1. Verify serial node is running: `ros2 node list`
2. Check serial port with: `ls -la /dev/tty*`
3. Verify permissions: `sudo usermod -aG dialout $USER` (logout/login)
4. Monitor serial connection: `screen /dev/ttyACM0 115200`
5. Check Arduino serial output in Hardware Panel

---

## Performance Specifications

**Expected Performance**:
| Metric | Typical Value | Notes |
|--------|---------------|-------|
| Max forward speed | 1.0 m/s | Depends on motor RPM and wheel size |
| Turning radius | 0.25 m | 30cm separation with 3cm wheels |
| Acceleration | 0.5 m/s² | Limited by traction and motor torque |
| IMU sample rate | 200 Hz | MPU6050 internal rate |
| Encoder resolution | 20 ticks/rev | Typical quadrature encoder |
| Odometry error | 5-10% | Accumulated after 10m travel |
| Battery runtime | 2-4 hours | Depends on load and battery capacity |

**Optimization Tips**:
- Reduce publish rate if CPU overloaded
- Use lightweight batteries (LiPo instead of AA)
- Ensure wheels roll smoothly (minimize friction)
- Match motor speeds precisely (see calibration guide)

---

## Advanced Features

### Sensor Fusion with Kalman Filter

Replace dead-reckoning odometry with EKF:

```bash
# Install robot_localization package
sudo apt install ros-humble-robot-localization

# Launch EKF node
ros2 launch robot_localization ekf.launch.py
```

### Obstacle Avoidance

Add ultrasonic or LiDAR:
```cpp
// In Arduino code
int distance = readUltrasonic();
if (distance < 20) {  // 20cm
  stop_motors();
  backup_and_turn();
}
```

### SLAM Integration

Combine odometry with visual SLAM:
```bash
ros2 launch slam_toolbox online_async.launch.py
```

---

## Educational Use

### Classroom Lesson Plan (3 Hours)

**Hour 1: Assembly & Electronics**
- Student groups assemble robot chassis
- Solder/breadboard connections
- Identify components and functions

**Hour 2: Firmware Programming**
- Upload Arduino code
- Understand motor control PWM
- Test with Hardware Panel

**Hour 3: ROS2 Integration**
- Create ROS2 package
- Implement motor command node
- Drive robot via keyboard teleop

**Assessment**:
- Robot navigates 2-meter square autonomously
- Measures accuracy of odometry vs actual distance
- Analyzes IMU noise characteristics

### Workshop Challenge

**"Maze Solver"** (4 hours):
- Build and assemble robot (1 hour)
- Implement wall-following algorithm (1.5 hours)
- Test through maze (1 hour)
- Optimize for speed/accuracy (0.5 hour)

---

## Next Steps

### After Completing Basic Robot

**Beginner Path**:
1. Add ultrasonic sensor (obstacle detection)
2. Implement wall-following behavior
3. Create simple web dashboard to monitor location

**Intermediate Path**:
1. Add LiDAR for SLAM
2. Implement autonomous navigation with Nav2
3. Sensor fusion (Kalman filter)

**Advanced Path**:
1. Implement visual odometry with camera
2. ROS2 action server for complex behaviors
3. Distributed computing (multiple robots)

---

## References

### Documentation
- [Motor Calibration Guide](motor_calibration.md)
- [Odometry Setup](odometry_setup.md)
- [ROS2 Integration](ros2_integration.md)
- [Testing Guide](testing_guide.md)
- [Serial Bridge Implementation](../../communication/serial_bridge_implementation.md)

### Datasheets
- [L298N Motor Driver](https://www.st.com/resource/en/datasheet/l298n.pdf)
- [MPU6050 IMU](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [Arduino Mega 2560](https://store.arduino.cc/products/arduino-mega-2560-rev3)

### Books & Tutorials
- "Programming Robots with ROS" by Quigley, Gerkey, Smart
- "Differential Drive Robot Kinematics" (online tutorials)
- ROS2 Navigation Tutorial: [nav2.org](https://nav2.org/)

---

## Community & Support

### Share Your Robot!
- Post photos to RoboShire community
- Share improvements to Arduino code
- Contribute calibration data for different motors

### Get Help
- Check [Testing Guide](testing_guide.md) for troubleshooting
- Review [Motor Calibration](motor_calibration.md) if movement issues
- Ask on ROS2 Discourse: [discourse.ros.org](https://discourse.ros.org/)

---

## FAQ

**Q: What if my motors are too fast/slow?**
A: See [Motor Calibration Guide](motor_calibration.md) - adjust PWM or gear ratios.

**Q: Can I use different wheel sizes?**
A: Yes, but update `wheel_radius` parameter in odometry code.

**Q: How do I add more sensors?**
A: Modify arduino_code.ino to read additional sensors, update ROS2 node to publish new topics.

**Q: Can I make it autonomous?**
A: Yes! Use Nav2 navigation stack (see [ROS2 Integration](ros2_integration.md)).

**Q: What about battery runtime?**
A: Use higher capacity battery (5000mAh LiPo) or add power management (sleep between movements).

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 2.0.0 | 2025-10-30 | Complete rewrite with v2.4.0 Hardware Testing Panel |
| 1.0.0 | 2025-10-29 | Initial release |

---

## License

This example project is licensed under Apache 2.0.

**Hardware designs**: Creative Commons BY-SA 4.0
**Software code**: Apache 2.0
**Documentation**: Creative Commons BY 4.0

---

**Ready to build?** Start with the [Bill of Materials](hardware_bom.md) to order your components!
