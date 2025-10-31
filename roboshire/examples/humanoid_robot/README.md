# Humanoid Robot Example

**Difficulty**: Advanced
**Category**: Humanoid Robotics
**Total Mass**: 50.0 kg
**Height**: 1.5 m
**DOF**: 12

---

## Overview

This example features a simplified humanoid robot designed for bipedal locomotion and manipulation research. It demonstrates human-like structure with realistic proportions and mass distribution.

### Key Features

- **Upper Body**: Torso, head, 2 arms (shoulder + elbow joints)
- **Lower Body**: Pelvis, 2 legs (hip + knee joints)
- **12 DOF Total**: 4 per arm (2 shoulder, 1 elbow) + 2 per leg (hip, knee)
- **Realistic Proportions**: Based on average human dimensions
- **Balance Point**: Center of mass at pelvis for stability
- **IMU Sensor**: For real-time balance feedback

---

## Robot Structure

### Dimensions (Human-like Proportions)

**Total Height**: 1.5 m (5 ft)
- Head: 0.2 m (8 inches)
- Torso: 0.5 m (20 inches)
- Upper leg: 0.4 m (16 inches)
- Lower leg: 0.4 m (16 inches)

**Arm Span**: 1.5 m (shoulder to shoulder ~0.4m, arm length 0.55m each)

### Link Specifications

#### Upper Body

1. **Head**
   - Shape: Sphere (0.15m radius)
   - Mass: 4.0 kg
   - Contains: Camera, microphone

2. **Torso**
   - Shape: Box (0.35m × 0.25m × 0.5m)
   - Mass: 20.0 kg (includes battery, computer)
   - Contains: Main computer, battery, IMU

3. **Upper Arms** (2x)
   - Shape: Cylinder (0.05m radius, 0.3m length)
   - Mass: 2.0 kg each
   - Joints: Shoulder (pitch + roll)

4. **Lower Arms** (2x)
   - Shape: Cylinder (0.04m radius, 0.25m length)
   - Mass: 1.5 kg each
   - Joints: Elbow (pitch)
   - Includes: Gripper mount point

#### Lower Body

1. **Pelvis**
   - Shape: Box (0.3m × 0.2m × 0.15m)
   - Mass: 5.0 kg
   - Balance point for whole body

2. **Upper Legs** (Thighs, 2x)
   - Shape: Cylinder (0.06m radius, 0.4m length)
   - Mass: 4.0 kg each
   - Joints: Hip (pitch + roll)

3. **Lower Legs** (Shins, 2x)
   - Shape: Cylinder (0.05m radius, 0.4m length)
   - Mass: 3.0 kg each
   - Joints: Knee (pitch)

4. **Feet** (2x)
   - Shape: Box (0.25m × 0.12m × 0.08m)
   - Mass: 1.0 kg each
   - Contact point with ground

**Total Mass**: 50.0 kg (similar to lightweight adult)

### Coordinate Frames

```
base_link (pelvis/center of mass)
├── torso
│   ├── head
│   ├── left_shoulder
│   │   └── left_upper_arm
│   │       └── left_lower_arm
│   │           └── left_hand
│   └── right_shoulder
│       └── right_upper_arm
│           └── right_lower_arm
│               └── right_hand
├── left_hip
│   └── left_upper_leg
│       └── left_lower_leg
│           └── left_foot
└── right_hip
    └── right_upper_leg
        └── right_lower_leg
            └── right_foot
```

---

## Joint Configuration

### Upper Body Joints

| Joint | Type | Range | Purpose |
|-------|------|-------|---------|
| neck_pitch | Revolute | -45° to +45° | Head nod |
| left_shoulder_pitch | Revolute | -180° to +180° | Arm forward/back |
| left_shoulder_roll | Revolute | -90° to +90° | Arm left/right |
| left_elbow | Revolute | 0° to +150° | Arm bend |
| right_shoulder_pitch | Revolute | -180° to +180° | Arm forward/back |
| right_shoulder_roll | Revolute | -90° to +90° | Arm left/right |
| right_elbow | Revolute | 0° to +150° | Arm bend |

### Lower Body Joints

| Joint | Type | Range | Purpose |
|-------|------|-------|---------|
| left_hip_pitch | Revolute | -120° to +120° | Leg swing |
| left_hip_roll | Revolute | -45° to +45° | Leg abduction |
| left_knee | Revolute | 0° to +150° | Leg bend |
| right_hip_pitch | Revolute | -120° to +120° | Leg swing |
| right_hip_roll | Revolute | -45° to +45° | Leg abduction |
| right_knee | Revolute | 0° to +150° | Leg bend |

---

## Bipedal Walking

### Walking Gait Phases

**Full Walking Cycle** (1 second = 1 Hz cadence):

1. **Double Support** (0.0s - 0.1s)
   - Both feet on ground
   - Shift weight to left foot
   - Prepare to lift right foot

2. **Right Swing** (0.1s - 0.5s)
   - Left foot support
   - Right leg swings forward
   - Torso maintains upright

3. **Double Support** (0.5s - 0.6s)
   - Right foot lands
   - Both feet on ground
   - Shift weight to right foot

4. **Left Swing** (0.6s - 1.0s)
   - Right foot support
   - Left leg swings forward
   - Return to start position

### Center of Mass (COM) Trajectory

During walking, COM must:
- Stay within support polygon (foot contact area)
- Move smoothly to avoid falling
- Height: ~0.9m (pelvis height)

**COM Path**:
- Lateral: ±0.05m (slight sway left/right)
- Vertical: ±0.02m (slight bob up/down)
- Forward: 0.5 m/s walking speed

### Zero Moment Point (ZMP)

For stable walking:
- ZMP must be inside support foot
- When both feet down: ZMP in middle
- When one foot up: ZMP under support foot

---

## Balance Control

### Sensing

**IMU (in torso)**:
- Measures: Roll, pitch, yaw angles
- Detects: Falling, tilting
- Update rate: 100 Hz

**Foot Pressure Sensors** (optional):
- Measure: Force on each foot
- Detect: Contact state, ZMP location
- Used for: Gait state machine

### Control Strategy

**Ankle Strategy** (small disturbances):
- Adjust ankle torque
- Keep body upright
- Fast response

**Hip Strategy** (medium disturbances):
- Move hips to shift COM
- Bend torso forward/back
- Recover balance

**Step Strategy** (large disturbances):
- Take recovery step
- Widen stance
- Prevent falling

### PID Control

**Attitude Control**:
- Maintain upright posture
- P=10.0, D=2.0 (no integral)
- Compensate for tilt

**COM Control**:
- Track desired COM trajectory
- P=5.0, I=0.5, D=1.0
- Smooth walking motion

---

## Node Graph

The included node graph provides:

1. **Walking Controller** (Lifecycle)
   - Gait generation (FSM)
   - Foot trajectory planning
   - Joint angle computation (IK)
   - Publishes joint commands

2. **Balance Controller**
   - IMU feedback
   - Ankle/hip strategy
   - Fall detection
   - Emergency stop

3. **Joint State Publisher**
   - Broadcasts joint positions
   - Publishes TF tree
   - Odometry estimation

4. **Motion Planner**
   - Path planning for walking
   - Obstacle avoidance
   - Step planning

5. **Manipulation Controller** (optional)
   - Arm control
   - Grasping
   - Object manipulation

---

## Getting Started

### 1. Load the Example
- Open RoboShire
- File > New from Example
- Select "Humanoid Robot (Simple Biped)"

### 2. Explore the URDF
- Inspect upper/lower body structure
- Note realistic human proportions
- Check joint ranges

### 3. Test Joint Motion
- View in RViz2
- Use Joint State Publisher GUI
- Move individual joints
- Find stable standing pose

### 4. Generate Walking Code
- Review node graph
- Click "Generate Code"
- Examine walking controller logic

### 5. Simulate Walking
- Launch in Gazebo
- Start walking gait
- Monitor balance (IMU data)
- Adjust parameters if unstable

---

## Integration Guides

### Gazebo Simulation

**Add Foot Contact Sensors**:
```xml
<gazebo reference="left_foot">
  <sensor name="left_foot_contact" type="contact">
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
    <plugin name="left_foot_contact_plugin"
            filename="libgazebo_ros_bumper.so">
      <bumperTopicName>left_foot_contact</bumperTopicName>
    </plugin>
  </sensor>
</gazebo>
```

**Launch Walking Simulation**:
```bash
ros2 launch humanoid_robot gazebo.launch.py

# Start walking
ros2 service call /walking_controller/configure std_srvs/srv/Trigger
ros2 service call /walking_controller/activate std_srvs/srv/Trigger

# Set walking speed
ros2 param set /walking_controller walking_speed 0.3
```

### MoveIt2 Integration (Arms)

For arm manipulation:

1. **Generate MoveIt2 Config**:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Configure:
# - Planning groups: left_arm, right_arm, both_arms
# - End effectors: left_hand, right_hand
# - Collision checking
```

2. **Plan Arm Motions**:
```python
# Example: Reach forward with right arm
from moveit_py import MoveItPy

robot = MoveItPy()
arm = robot.get_planning_group("right_arm")
arm.set_named_target("reach_forward")
arm.plan_and_execute()
```

### Whole-Body Control

For coordinated motion (walking + manipulation):

```bash
# Use ros2_control with whole_body_controller
ros2 run controller_manager spawner whole_body_controller

# Commands both arms and legs
ros2 topic pub /whole_body_controller/commands ...
```

---

## Tuning Tips

### Stable Standing

First, achieve stable standing before walking:
1. Set all joints to zero (home position)
2. Adjust foot width (0.2m - 0.3m apart)
3. Verify COM is between feet
4. Test balance with small pushes

### Walking Parameters

**Step Length**: Start small (0.1m), increase gradually
**Step Height**: 0.05m (just clear ground)
**Walking Speed**: 0.2 m/s (slow), 0.5 m/s (normal), 1.0 m/s (fast)
**Step Frequency**: 0.5 Hz (slow), 1.0 Hz (normal), 1.5 Hz (fast)

### Stability Margins

- **Static Margin**: COM 5cm inside support polygon
- **Dynamic Margin**: Allow for COM velocity
- **Recovery Margin**: Space for correction steps

---

## Common Issues

### Robot Falls Forward
- **Cause**: COM ahead of feet
- **Fix**: Lean torso back, adjust COM height

### Robot Falls Sideways
- **Cause**: Foot spacing too narrow
- **Fix**: Wider stance, adjust hip roll

### Knees Buckle
- **Cause**: Knee gains too low, heavy load
- **Fix**: Increase knee stiffness, reduce mass

### Slow Walking
- **Cause**: Step frequency low, timid gait
- **Fix**: Increase cadence, longer steps

---

## Safety Considerations

⚠️ **Physical Robot Warnings**:
- Humanoid robots can be dangerous
- Heavy (50kg) - can cause injury if falls
- Start in simulation, test thoroughly
- Use soft padding during initial tests
- Have emergency stop button
- Never test alone

**Simulation First**:
1. Perfect walking in Gazebo
2. Test on small physical prototype
3. Scale up to full-size robot
4. Add safety features (padding, limits)

---

## Learning Resources

### Bipedal Robotics
- "Introduction to Humanoid Robotics" by Kajita et al.
- "Springer Handbook of Robotics" - Humanoid Chapter
- Honda ASIMO Research Papers

### Walking Algorithms
- ZMP (Zero Moment Point) method
- Inverse Kinematics for leg trajectories
- Gait pattern generation

### ROS2 Packages
- `humanoid_description` - URDF examples
- `bipedal_locomotion_framework` - Walking control
- `whole_body_control` - Coordinated motion

### Simulation
- Gazebo humanoid models
- MuJoCo for fast dynamics
- Webots humanoid samples

---

## Next Steps

### Beginner
1. Understand joint structure
2. Find stable standing pose
3. Test individual joint motions

### Intermediate
1. Implement static walking (slow, stable)
2. Add IMU feedback for balance
3. Test different walking speeds

### Advanced
1. Dynamic walking (natural gait)
2. Walking on slopes/stairs
3. Whole-body coordination (walk + manipulate)
4. Running and jumping

---

## Future Enhancements

**Hands/Grippers**:
- Add articulated hands
- 5 fingers per hand
- Grasping control

**Head Tracking**:
- Add camera pan/tilt
- Track objects visually
- Human face detection

**Full Body IK**:
- Inverse kinematics solver
- Reach targets with feet/hands
- Redundancy resolution

**Social Behaviors**:
- Gestures and expressions
- Speech synthesis
- Human-robot interaction

---

## Credits

Inspired by research humanoid robots: ASIMO (Honda), Atlas (Boston Dynamics), NAO (Aldebaran). This is a simplified educational example for learning bipedal robotics.

**Created by**: RoboShire Team
**Version**: 1.0.0
**License**: MIT
**Tested with**: ROS2 Humble, Gazebo Classic 11
