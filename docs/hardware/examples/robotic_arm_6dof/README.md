# 6-DOF Robotic Arm - Complete Hardware Example

**Difficulty**: Advanced
**Time**: 4-6 hours total
**Cost**: ~$100-200 USD
**Skills Learned**: Servo control, kinematics, MoveIt2 integration, trajectory planning

---

## Project Overview

Build a 6-degree-of-freedom (6-DOF) robotic arm with servo motors. This arm can reach any point in its workspace, pick up objects, and execute complex manipulation tasks integrated with ROS2 and MoveIt2.

### What You'll Build

```
                    ┌─ End Effector
                    │ (Gripper)
                    ▼
        ┌──────────────────┐
        │     Wrist        │
        │  (J5, J6: Roll)  │
        └────────┬─────────┘
                 │
        ┌────────▼─────────┐
        │   Forearm        │
        │  (J3, J4)        │
        └────────┬─────────┘
                 │
        ┌────────▼─────────┐
        │    Shoulder      │
        │   (J1, J2)       │
        └────────┬─────────┘
                 │
        ┌────────▼─────────┐
        │   Base Joint     │
        │   (J0: Rotate)   │
        └────────┬─────────┘
                 │
        ┌────────▼─────────┐
        │   Motor Control  │
        │  (Arduino)       │
        └────────┬─────────┘
                 │
        ┌────────▼─────────┐
        │   ROS2 Package   │
        │  (Planning,      │
        │   Control)       │
        └──────────────────┘
```

### Key Features

- **6-Axis Manipulation**: Reach any point in space with desired orientation
- **Servo Control**: 6 high-torque servo motors with precision control
- **Gripper Integration**: Add 2-finger or 3-finger gripper
- **ROS2 Control**: Full integration with standard ROS2 control interfaces
- **MoveIt2 Compatible**: Motion planning, collision checking, trajectory execution
- **Inverse Kinematics**: Calculate joint angles from end-effector position
- **Simulation**: Test movements in MuJoCo before hardware
- **Safety Limits**: Joint angle and torque limits

### Learning Outcomes

After completing this project:
- ✅ Understand servo motor control and PWM frequency
- ✅ Implement serial communication for multi-servo control
- ✅ Learn forward and inverse kinematics
- ✅ Use ROS2 control_msgs for joint commands
- ✅ Integrate with MoveIt2 motion planning framework
- ✅ Debug hardware communication issues
- ✅ Implement safety mechanisms for robotic arms

---

## Project Files

| File | Description |
|------|-------------|
| [hardware_bom.md](hardware_bom.md) | Bill of materials and servo selection |
| [servo_control.ino](servo_control.ino) | Arduino firmware for 6 servo control |
| [kinematics_basics.md](kinematics_basics.md) | Forward/inverse kinematics explanations |
| [moveit2_integration.md](moveit2_integration.md) | MoveIt2 configuration and planning |
| [safety_guide.md](safety_guide.md) | Safety mechanisms and limits |

---

## Quick Start

### Prerequisites Checklist

- [ ] RoboShire v2.4.0+ installed
- [ ] ROS2 Humble+ installed
- [ ] Arduino IDE installed
- [ ] Hardware components purchased (see [BOM](hardware_bom.md))
- [ ] Python 3.10+ with kinematics libraries

### 30-Second Overview

1. **Assemble arm structure** (1-2 hours) - 3D-printed or aluminum parts
2. **Mount servo motors** (1 hour) - Connect 6 servos to mechanical structure
3. **Wire control electronics** (30 minutes) - Arduino with servo driver
4. **Flash firmware** (5 minutes) - Upload servo control code
5. **Create ROS2 package** (30 minutes) - URDF, controllers, MoveIt2 config
6. **Test movements** (30 minutes) - Basic joint tests, then MoveIt2 planning

**Total time**: ~4 hours for basic functionality

---

## Detailed Step-by-Step

### Step 1: Design Selection

**Option A: 3D-Printed Arm** (DIY, ~$100)
- Lightweight PLA plastic
- Print time: 10-20 hours
- Easy to modify
- Less durable long-term
- Popular design: PhantomX Reactor arm

**Option B: Aluminum Extrusion Kit** (Pre-engineered, ~$200+)
- Higher precision
- More durable
- Limited customization
- Professional appearance
- Examples: UR5e, Dobot M1

**Option C: Commercial Kit** (Complete, ~$300+)
- All parts included
- Tested design
- Excellent documentation
- Less learning opportunity
- Examples: Arduino/ROS-based kits

**Recommendation for learning**: Start with 3D-printed design (Option A)

### Step 2: Acquire Mechanical Structure

**If 3D Printing**:
1. Find design on Thingiverse or GitHub
2. Download STL files for all parts
3. Slice files with Cura or PrusaSlicer
4. Print on FDM printer (PLA or ABS)
5. Post-process parts (support removal, light sanding)

**Example: 3D Printed 6-DOF Arm**
```
Search terms:
  - "6DOF robotic arm STL"
  - "Arduino servo arm 3D printed"
  - "Open-source robot arm designs"

Popular sources:
  - Thingiverse.com
  - Printables.com (formerly Printing)
  - GitHub robotics projects
```

**If Purchasing Kit**:
1. Order complete kit (includes all plastic/metal parts)
2. Verify all parts included upon delivery
3. Check for damage or defects
4. Follow kit assembly instructions

### Step 3: Assemble Mechanical Structure

**Generic Assembly Steps**:

1. **Base**: Assemble turntable/rotating base
2. **Shoulder**: Mount upper arm servo and linkage
3. **Elbow**: Attach forearm to shoulder joint
4. **Wrist**: Connect wrist joint assembly
5. **End Effector**: Mount gripper or tool

**For Each Joint**:
- Insert servo into mount
- Align servo horn with linkage
- Secure with fasteners
- Verify smooth rotation (no binding)
- Check joint range of motion

**Assembly Tips**:
- Use thread-lock on screw joints (prevents loosening)
- Leave servo control arms loose initially (tune later)
- Test each joint alone before full assembly
- Take photos during assembly (easier disassembly later)

### Step 4: Select and Install Servo Motors

**Servo Selection Criteria**:

| Parameter | Importance | Typical Range |
|-----------|------------|----------------|
| Torque | Critical | 10-50 kg-cm |
| Speed | Important | 0.1-0.3 s/60° |
| Weight | Moderate | 40-100g |
| Voltage | Must match | 4.8V or 6.0V |
| Cost | Budget | $8-30 each |

**Recommended Servos for 6-DOF Arm**:
- **Base/Shoulder (J0, J1)**: High torque (30+ kg-cm)
  - Example: MG996R, XL430-W250
- **Elbow/Wrist (J2-J5)**: Medium torque (20+ kg-cm)
  - Example: MG995, XL320
- **Wrist Roll (J6)**: Lower torque (15+ kg-cm)
  - Example: SG90, XL320

**Installation**:
1. Insert servo into bracket
2. Align servo horn with control linkage
3. Secure servo with fasteners
4. Verify smooth motion before securing
5. Install servo horn, set to neutral position

### Step 5: Electronics and Wiring

**Components Needed**:
- 1x Arduino Mega 2560
- 1x 16-channel servo driver module (PCA9685)
- 1x Power distribution board
- Servo cable connectors
- 6V power supply (sufficient current for all servos)

**Wiring Diagram**:

```
┌─────────────────────────────┐
│     Arduino Mega 2560       │
│                             │
│  SDA (Pin 20) ─────────┐   │
│  SCL (Pin 21) ─────────┼───┼─→ I2C
│  GND          ─────────┘   │
│  5V           ──┐          │
└───────────────────┼─────────┘
                    │
         ┌──────────▼──────────┐
         │  PCA9685 Servo      │
         │  Driver (I2C)       │
         │                     │
         │  Out 0 ───→ Servo J0
         │  Out 1 ───→ Servo J1
         │  Out 2 ───→ Servo J2
         │  Out 3 ───→ Servo J3
         │  Out 4 ───→ Servo J4
         │  Out 5 ───→ Servo J5
         │  Out 6 ───→ Gripper
         │                     │
         └──────────┬──────────┘
                    │
         ┌──────────▼──────────┐
         │   6V Power Supply   │
         │   (10A recommended) │
         │                     │
         │   Feeds all servos  │
         └─────────────────────┘
```

**I2C Pull-up Resistors**:
- PCA9685 requires 4.7k resistors on SDA and SCL
- Most breakout boards have built-in pullups

**Safety Features**:
- Add capacitor across power (100µF)
- Use separate power for servos (not Arduino USB power)
- Install fuse or circuit breaker on main power line
- Heat shrink all connections

### Step 6: Upload Servo Control Firmware

**Arduino Code** (see [servo_control.ino](servo_control.ino)):
- Communicates with PCA9685 servo driver via I2C
- Receives joint commands from ROS2
- Controls 6 servo motors independently
- Implements safety limits (joint angle bounds)
- Sends back joint angle feedback

**Upload Process**:
1. Open Arduino IDE
2. Install PCA9685 library: Adafruit PWM Servo Driver Library
3. Open [servo_control.ino](servo_control.ino)
4. Select board: Arduino Mega 2560
5. Select port: /dev/ttyACM0 (or detected)
6. Click Upload

**Verification**:
```
Expected serial output:
[INFO] Initializing PCA9685 servo driver...
[INFO] Servo driver found at address 0x40
[INFO] Calibrating servo ranges...
[INFO] Ready for joint commands
```

### Step 7: Create URDF Robot Description

**URDF Example**:
```xml
<?xml version="1.0"?>
<robot name="6dof_arm">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="gray"/>
    </visual>
  </link>

  <!-- Joint 0: Base Rotation -->
  <joint name="joint_0" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-pi}" upper="${pi}" effort="50" velocity="1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.10"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Joint 1: Shoulder Pitch -->
  <joint name="joint_1" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-pi/2}" upper="${pi/2}" effort="40" velocity="1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.15"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <!-- Joints 2-5: Elbow, Wrist Pitch, Wrist Roll (similar pattern) -->
  <!-- ... (continue for remaining joints) ... -->

  <!-- End Effector Link -->
  <link name="tool_frame"/>
  <joint name="tool_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="tool_frame"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

</robot>
```

### Step 8: Create ROS2 Joint Control Node

**Python Node** for joint command interface:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import serial
import struct
import time

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control')

        # Parameters
        self.joint_names = ['joint_0', 'joint_1', 'joint_2',
                           'joint_3', 'joint_4', 'joint_5']

        # Serial connection to Arduino
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
        time.sleep(2)  # Wait for Arduino initialization

        # Subscribers
        self.traj_sub = self.create_subscription(
            JointTrajectory, '/arm_controller/follow_joint_trajectory/goal',
            self.trajectory_callback, 10)

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # Timer for reading feedback
        self.timer = self.create_timer(0.05, self.read_joint_feedback)

        # Current joint positions
        self.current_positions = [0.0] * 6

        self.get_logger().info('Arm Control Node Initialized')

    def trajectory_callback(self, msg):
        """Execute joint trajectory command"""
        # Get last waypoint (final desired position)
        final_point = msg.points[-1]

        # Send joint angles to Arduino
        self.send_joint_command(final_point.positions)

    def send_joint_command(self, positions):
        """Send 6 joint angles to Arduino via serial"""
        # Limit checking
        positions = self.apply_joint_limits(positions)

        # Convert radians to servo angles (0-180°)
        servo_values = []
        for i, pos_rad in enumerate(positions):
            # Map [-π, π] to [0, 180]°
            angle_deg = (pos_rad + 3.14159) * (180 / 6.28318)
            angle_deg = max(0, min(180, angle_deg))  # Clamp
            servo_values.append(int(angle_deg * 1.5625))  # Convert to servo range

        # Format: S<a0><a1><a2><a3><a4><a5> (servo command)
        cmd = b'S' + struct.pack('BBBBBB', *servo_values)
        self.serial.write(cmd)

        self.get_logger().info(f'Sent servo command: {servo_values}')

    def read_joint_feedback(self):
        """Read current joint positions from Arduino"""
        if self.serial.in_waiting >= 13:
            data = self.serial.read(13)
            if data[0] == ord('J'):  # Joint feedback packet
                # Parse 6 servo angles (2 bytes each, big-endian)
                servo_values = []
                for i in range(6):
                    val = struct.unpack('>H', data[1+i*2:3+i*2])[0]
                    servo_values.append(val)

                # Convert servo values back to radians
                self.current_positions = [
                    (val / 1.5625) * (6.28318 / 180) - 3.14159
                    for val in servo_values
                ]

                # Publish joint state
                self.publish_joint_state()

    def publish_joint_state(self):
        """Publish current joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6

        self.joint_state_pub.publish(msg)

    def apply_joint_limits(self, positions):
        """Enforce joint angle limits"""
        # Define joint limits (in radians)
        limits = [
            (-3.14159, 3.14159),  # J0: ±180°
            (-1.57079, 1.57079),  # J1: ±90°
            (-1.57079, 1.57079),  # J2: ±90°
            (-1.57079, 1.57079),  # J3: ±90°
            (-1.57079, 1.57079),  # J4: ±90°
            (-3.14159, 3.14159),  # J5: ±180°
        ]

        limited = []
        for i, pos in enumerate(positions):
            min_lim, max_lim = limits[i]
            limited.append(max(min_lim, min(max_lim, pos)))

        return limited

def main():
    rclpy.init()
    node = ArmControlNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Step 9: Calibrate Servo Ranges

**Neutral Position Calibration**:
1. Power on arm with all servos de-energized
2. Manually position arm to "zero" position (all joints at neutral)
3. While in this position, energize servos and send neutral command
4. Verify all joints return to position
5. Repeat if needed until neutral is consistent

**Range Verification**:
```bash
# Test individual servo ranges
ros2 topic pub /arm_controller/command trajectory_msgs/JointTrajectory \
  "{header: {frame_id: base_link}, joint_names: [joint_0], \
    points: [{positions: [0.0], time_from_start: {sec: 2}}]}"
```

### Step 10: Test with MoveIt2 (Optional)

For autonomous motion planning:

1. Install MoveIt2:
   ```bash
   sudo apt install ros-humble-moveit
   ```

2. Use MoveIt Setup Assistant to configure:
   ```bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```

3. Test motion planning:
   ```bash
   ros2 launch my_arm_moveit_config demo.launch.py
   ```

---

## Safety Mechanisms

### Critical Safety Features

1. **Joint Angle Limits**
   - Each joint has min/max angle limits
   - Software enforces limits before sending to servos
   - Hardware servo limits also prevent over-rotation

2. **Torque Limits**
   - Monitor servo current draw
   - Reduce speed if torque too high
   - Emergency stop if stall detected

3. **Speed Limits**
   - Cap maximum servo speed
   - Gradual acceleration (no sudden jerks)
   - Slow speed near singular configurations

4. **Collision Detection** (MoveIt2)
   - Check for self-collisions
   - Verify workspace boundaries
   - Prevent unreachable goals

### Pre-Operation Checklist

- [ ] All servo connections secure
- [ ] Power supply properly rated
- [ ] Robot positioned in safe area
- [ ] No people/objects within arm reach
- [ ] Emergency stop button accessible
- [ ] Serial communication working
- [ ] Joint angles within limits
- [ ] Servo torque reasonable

---

## Kinematics Basics

### Forward Kinematics

Given 6 joint angles, calculate end-effector position:

```
P = f(θ0, θ1, θ2, θ3, θ4, θ5)

Where:
  θi = joint angle
  P = (x, y, z) end-effector position
  f = kinematics function
```

**Using DH Parameters** (Denavit-Hartenberg):
```
For each joint i:
  a_i = link length
  d_i = link offset
  alpha_i = twist angle
  theta_i = joint angle

T_0_6 = T_0_1 × T_1_2 × ... × T_5_6
```

### Inverse Kinematics

Given desired end-effector position, find joint angles:

```
θ0...θ5 = f_inverse(x, y, z, roll, pitch, yaw)
```

**Methods**:
1. **Analytical (closed-form)** - Fast, limited to some geometries
2. **Numerical (iterative)** - General purpose, slower
3. **Learning-based (ML)** - Advanced, data-hungry

**Python Library**:
```python
from ikpy.chain import Chain

# Load URDF and build kinematics chain
arm = Chain.from_urdf_file("arm.urdf")

# Solve inverse kinematics
target = [0.3, 0.1, 0.5]  # x, y, z in meters
joint_angles = arm.inverse_kinematics(target)
```

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Servos won't move | I2C not working | Check PCA9685 I2C address with scanner |
| Arm jerks during movement | Sudden speed changes | Implement trajectory smoothing |
| Joint limits violated | Control code error | Verify limit checking logic |
| Servo overheating | Too much load | Reduce speed or check mechanical binding |
| Serial communication fails | Port permission | `sudo usermod -aG dialout $USER` |

---

## Next Steps

1. Complete [Safety Guide](safety_guide.md)
2. Study [Kinematics Basics](kinematics_basics.md)
3. Integrate with [MoveIt2](moveit2_integration.md)
4. Add gripper control
5. Implement computer vision for picking tasks

---

## References

- [Hardware BOM](hardware_bom.md)
- [Servo Control Firmware](servo_control.ino)
- [MoveIt2 Configuration Guide](moveit2_integration.md)
- ROS2 Documentation: [docs.ros.org](https://docs.ros.org/)
- MoveIt2: [moveit.ros.org](https://moveit.ros.org/)

---

## FAQ

**Q: Can I use different servos?**
A: Yes, but verify torque is sufficient for arm weight and payload.

**Q: How do I add a gripper?**
A: Connect gripper servo to PCA9685 output 6 and control like any joint.

**Q: What's the payload capacity?**
A: Depends on servo torque and arm geometry. Typically 0.5-1 kg.

**Q: Can I use wireless control?**
A: Yes, use WiFi or Bluetooth bridge instead of USB serial.

**Q: Is MoveIt2 necessary?**
A: No, but highly recommended for complex motion planning.

---

## License

This example is licensed under Apache 2.0.

---

**Ready to build?** Start with the [Bill of Materials](hardware_bom.md)!
