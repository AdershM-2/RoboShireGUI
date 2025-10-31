# Quadruped Robot Example

**Difficulty**: Intermediate
**Category**: Legged Robotics
**Total Mass**: 25.0 kg
**DOF**: 12 (3 per leg)

---

## Overview

This example features a four-legged robot inspired by Boston Dynamics' Spot robot. It demonstrates legged locomotion principles with realistic proportions and mass distribution.

### Key Features

- **12 Degrees of Freedom**: Each leg has 3 revolute joints (hip yaw/pitch, shoulder, knee)
- **Realistic Kinematics**: Proper leg lengths and joint ranges for stable walking
- **Mass Distribution**: 15kg body + 2.5kg per leg (realistic weight distribution)
- **Modular Design**: 4 identical legs attached to central body
- **Controller Ready**: Includes node graph for gait generation

---

## Robot Structure

### Body
- **Main Body**: Central chassis (60cm × 40cm × 20cm)
- **Mass**: 15.0 kg
- **Sensors**: IMU mounting point on top

### Legs (4x identical)
Each leg consists of:
1. **Hip Link**: Connects to body, rotates around Z-axis (yaw)
   - Range: ±45° (90° total)
   - Mass: 1.0 kg

2. **Upper Leg**: Hip to shoulder joint
   - Range: -60° to +60° (120° total)
   - Length: 30cm
   - Mass: 0.8 kg

3. **Lower Leg**: Shoulder to foot
   - Range: -120° to -10° (110° total)
   - Length: 30cm
   - Mass: 0.7 kg

**Total Leg Mass**: 2.5 kg per leg

### Coordinate Frames
```
base_link (body center)
├── front_left_hip
│   └── front_left_upper_leg
│       └── front_left_lower_leg
├── front_right_hip
│   └── front_right_upper_leg
│       └── front_right_lower_leg
├── rear_left_hip
│   └── rear_left_upper_leg
│       └── rear_left_lower_leg
└── rear_right_hip
    └── rear_right_upper_leg
        └── rear_right_lower_leg
```

---

## Gait Patterns

### Walking Gait (Default)
- **Speed**: Slow, stable
- **Pattern**: Diagonal pairs (FL+RR, FR+RL)
- **Use**: Rough terrain, precise positioning

### Trotting Gait
- **Speed**: Medium
- **Pattern**: Diagonal pairs (faster cadence)
- **Use**: Flat terrain, moderate speed

### Galloping Gait (Advanced)
- **Speed**: Fast
- **Pattern**: All legs coordinated, flight phase
- **Use**: Speed on flat terrain

---

## Node Graph

The included node graph provides:

1. **Gait Controller** (Lifecycle)
   - Generates leg trajectories
   - Publishes joint commands
   - Subscribes to IMU for balance

2. **Joint State Publisher**
   - Publishes current joint positions
   - Broadcasts TF tree

3. **Odometry Node**
   - Estimates robot position
   - Publishes odometry data

---

## Getting Started

### 1. Load the Example
- Open RoboShire
- File > New from Example
- Select "Quadruped Robot (Spot-like)"

### 2. Explore the URDF
- Inspect the leg structure in URDF tree
- Note the joint limits and ranges
- Check mass distribution

### 3. Generate Code
- Review the node graph
- Click "Generate Code"
- Examine generated ROS2 nodes

### 4. Build & Run
- Click "Build" (Ctrl+B)
- Launch nodes (Ctrl+R)
- Monitor in Node Status tab

### 5. Visualize
- Click "View in RViz2"
- Use Joint State Publisher GUI to test leg motion
- Observe stable configurations

---

## Integration Guides

### Gazebo Simulation
```bash
# Add Gazebo plugins to URDF (in ros2_control section)
ros2 launch quadruped_robot gazebo.launch.py

# Control legs with ros2_control
ros2 topic pub /joint_commands ...
```

### ros2_control Integration
```bash
# Use joint_trajectory_controller
ros2 run controller_manager spawner joint_trajectory_controller

# Monitor joint states
ros2 topic echo /joint_states
```

### Nav2 Integration
For autonomous navigation:
1. Add 2D LiDAR sensor to body
2. Configure Nav2 parameters for legged robots
3. Adjust footprint for leg positions
4. Use recovery behaviors for leg control

---

## Tuning Tips

### Stability
- Lower center of gravity (reduce body mass)
- Wider stance (adjust hip positions)
- Stiffer joints (increase joint damping)

### Speed
- Longer legs (adjust link lengths)
- Faster actuators (increase velocity limits)
- Optimize gait timing

### Terrain Adaptation
- Add foot sensors (contact detection)
- Implement adaptive gait
- Use terrain mapping

---

## Common Issues

### Robot Falls Over
- **Cause**: Unstable leg configuration
- **Fix**: Check joint limits, ensure proper weight distribution

### Legs Collide
- **Cause**: Joint range too large
- **Fix**: Add collision geometry, adjust joint limits

### Slow Movement
- **Cause**: Low velocity limits
- **Fix**: Increase joint velocity limits in URDF

---

## Learning Resources

### Papers
- "Omnidirectional Walking for Quadruped Robots" (MIT, 2020)
- "Optimization-Based Locomotion Planning for Quadrupeds" (Boston Dynamics, 2018)

### ROS2 Packages
- `quad_utils` - Quadruped utilities
- `champ` - Quadruped controller
- `ros2_control` - Hardware abstraction

### Tutorials
- [ROS2 Quadruped Tutorial](https://docs.ros.org/en/humble/)
- [Gait Planning Basics](https://robotics.stackexchange.com/)

---

## Next Steps

### Beginner
1. Test different joint positions manually
2. Observe stable standing configurations
3. Understand leg kinematics

### Intermediate
1. Implement simple gait controller
2. Add IMU feedback for balance
3. Test in Gazebo simulation

### Advanced
1. Implement adaptive gait planning
2. Add terrain perception
3. Optimize for efficiency and speed
4. Add dynamic behaviors (jumping, stairs)

---

## Credits

Inspired by Boston Dynamics' Spot robot. This is an educational example for learning quadruped robotics principles.

**Created by**: RoboShire Team
**Version**: 1.0.0
**License**: MIT
**Tested with**: ROS2 Humble
