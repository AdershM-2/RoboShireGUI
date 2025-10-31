# Simple Differential Drive Robot

A beginner-friendly two-wheeled robot with caster wheel, perfect for learning ROS2 basics and mobile robot control.

## Overview

This example provides a simple differential drive robot that serves as an excellent starting point for learning mobile robotics with ROS2. The robot has two independently controlled wheels and a passive caster wheel for stability.

## Robot Specifications

- **Chassis**: 0.4m x 0.3m x 0.15m rectangular box
- **Wheels**: 0.1m radius, 0.05m width
- **Weight**: ~6 kg total
- **Wheel Base**: 0.35m between wheels
- **Drive Type**: Differential drive (two driven wheels)

## Components

### Links
1. **base_link**: Main chassis (blue box)
2. **left_wheel**: Left driven wheel (black cylinder)
3. **right_wheel**: Right driven wheel (black cylinder)
4. **caster_wheel**: Passive support wheel (grey sphere)
5. **imu_link**: IMU sensor (optional, red box)

### Joints
- **left_wheel_joint**: Continuous joint for left wheel
- **right_wheel_joint**: Continuous joint for right wheel
- **caster_wheel_joint**: Fixed joint for caster
- **imu_joint**: Fixed joint for IMU

## ROS2 Nodes

### 1. velocity_controller (Lifecycle Node)
- **Type**: Lifecycle Publisher
- **Topic**: `/cmd_vel`
- **Message**: `geometry_msgs/Twist`
- **Rate**: 10 Hz
- **Purpose**: Controls wheel velocities based on velocity commands

### 2. odometry_publisher
- **Type**: Publisher
- **Topic**: `/odom`
- **Message**: `nav_msgs/Odometry`
- **Rate**: 50 Hz
- **Purpose**: Publishes robot odometry from wheel encoders

### 3. wheel_encoder_reader
- **Type**: Subscriber
- **Topic**: `/wheel_encoders`
- **Message**: `sensor_msgs/JointState`
- **Purpose**: Reads wheel encoder data

## Getting Started

### 1. Load the Example
- Open RoboShire
- Go to `File > New from Example` or `Help > Robot Examples`
- Select "Simple Differential Drive Robot"
- Click "Load Example"

### 2. Explore the URDF
- The robot URDF will load automatically
- View the robot structure in the URDF tree
- Modify dimensions, colors, or add sensors as needed

### 3. Customize the Node Graph
- The pre-configured node graph will load
- Add or modify nodes as needed
- Connect nodes to create your control system

### 4. Generate and Build
- Click "Generate Code" to create ROS2 package
- Click "Build" to compile with colcon
- Click "Run" to launch nodes

## Control the Robot

### Using Command Line
```bash
# Publish velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}"

# View odometry
ros2 topic echo /odom

# Monitor wheel encoders
ros2 topic echo /wheel_encoders
```

### Using Lifecycle Commands
```bash
# List lifecycle nodes
ros2 lifecycle nodes

# Configure the velocity controller
ros2 lifecycle set /velocity_controller configure

# Activate it
ros2 lifecycle set /velocity_controller activate

# Deactivate when done
ros2 lifecycle set /velocity_controller deactivate
```

## Customization Ideas

### Easy Modifications
1. **Change Robot Size**: Modify box dimensions in URDF
2. **Add More Sensors**: Add camera, lidar, or ultrasonic sensors
3. **Change Wheel Size**: Adjust cylinder radius
4. **Add Controller Nodes**: Create PID controllers for precise motion

### Intermediate Modifications
1. **Add Simulation**: Export to Gazebo for physics simulation
2. **Implement Odometry**: Calculate odometry from wheel velocities
3. **Add Path Following**: Create nodes to follow waypoints
4. **Implement Obstacle Avoidance**: Use sensors to avoid obstacles

### Advanced Modifications
1. **SLAM Integration**: Add laser scanner and integrate with slam_toolbox
2. **Navigation Stack**: Integrate with Nav2 for autonomous navigation
3. **Visual Odometry**: Add camera and implement visual odometry
4. **Multi-Robot System**: Clone and coordinate multiple robots

## Learning Resources

### ROS2 Tutorials
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Writing a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Understanding ROS2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

### Mobile Robotics
- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
- [ROS2 Navigation Stack (Nav2)](https://navigation.ros.org/)
- [Mobile Robot Programming](https://www.rosroboticslearning.com/)

## Troubleshooting

### Robot doesn't move
- Check that velocity_controller is in ACTIVE state
- Verify /cmd_vel topic is being published
- Check wheel joint types are "continuous"

### Odometry drift
- Tune wheel radius and base width parameters
- Add IMU sensor fusion
- Calibrate wheel encoders

### Build errors
- Ensure all message dependencies are installed
- Check package.xml has correct dependencies
- Run `rosdep install --from-paths src --ignore-src -r -y`

## Next Steps

After mastering this simple robot, try:
1. **TurtleBot3 Example**: More complex robot with laser scanner
2. **Robotic Arm Example**: Learn manipulation
3. **Mobile Manipulator**: Combine navigation and manipulation

## License

Apache 2.0 - Free to use and modify

## Contributing

Found a bug or have an improvement? Please contribute to RoboShire!

---

*Example created by RoboShire Team*
*Last updated: 2025-10-20*
