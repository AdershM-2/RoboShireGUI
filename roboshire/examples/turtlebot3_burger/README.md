# TurtleBot3 Burger

Industry-standard mobile robot platform with 360° LiDAR, perfect for SLAM, navigation, and autonomous robotics research.

## Overview

TurtleBot3 Burger is one of the most popular mobile robot platforms for ROS2 education and research. This example provides a complete TurtleBot3 Burger setup with all sensors and controllers, ready for SLAM, navigation, and autonomous robotics experiments.

## Robot Specifications

- **Dimensions**: 138mm (diameter) x 192mm (height)
- **Weight**: ~1kg
- **Wheel Base**: 160mm between wheels
- **Wheel Diameter**: 66mm
- **Max Speed**: 0.22 m/s
- **Drive Type**: Differential drive
- **Battery**: 1,800mAh LiPo (11.1V)

## Hardware Components

### Actuators
- **Motors**: 2x Dynamixel XL430-W250-T servos
- **Wheels**: 2x 66mm diameter wheels
- **Caster**: Ball caster for rear support

### Sensors
- **LiDAR**: LDS-01 360° laser distance sensor
  - Range: 120mm ~ 3,500mm
  - Angular Resolution: 1°
  - Scan Rate: 5.5 Hz (300 RPM)
- **IMU**: 9-axis gyroscope/accelerometer/magnetometer
  - Update Rate: 100 Hz
- **Camera**: Raspberry Pi Camera Module v2
  - Resolution: 3280 x 2464 pixels
  - Frame Rate: 30 fps

### Controller
- **OpenCR1.0**: ARM Cortex-M7 @ 216MHz
- **SBC**: Raspberry Pi 4B (optional)

## ROS2 Nodes

### 1. base_controller (Lifecycle)
- **Type**: Lifecycle Subscriber
- **Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Purpose**: Translates velocity commands to wheel motor commands
- **Features**:
  - Differential drive kinematics
  - Velocity limiting
  - Emergency stop capability

### 2. odometry_fusion (Lifecycle)
- **Type**: Lifecycle Publisher
- **Topic**: `/odom` (nav_msgs/Odometry)
- **Rate**: 50 Hz
- **Purpose**: Fuses wheel encoders and IMU for accurate odometry
- **Features**:
  - Extended Kalman Filter
  - Covariance estimation
  - TF broadcasting (odom → base_link)

### 3. lidar_driver (Lifecycle)
- **Type**: Lifecycle Publisher
- **Topic**: `/scan` (sensor_msgs/LaserScan)
- **Rate**: 5 Hz
- **Purpose**: Publishes 360° laser scan data
- **Specifications**:
  - 360 points per scan
  - 12m maximum range
  - Intensity data included

### 4. imu_publisher
- **Type**: Publisher
- **Topic**: `/imu` (sensor_msgs/Imu)
- **Rate**: 100 Hz
- **Purpose**: Publishes 9-axis IMU data
- **Data**: Linear acceleration, angular velocity, orientation

### 5. camera_node
- **Type**: Publisher
- **Topic**: `/camera/image_raw` (sensor_msgs/Image)
- **Rate**: 30 fps
- **Purpose**: Publishes camera images
- **Format**: RGB8, 640x480

## Getting Started

### 1. Load the Example
```
File > New from Example > TurtleBot3 Burger
```

### 2. Review the URDF
- Explore all links and joints
- Note sensor placements
- Understand coordinate frames

### 3. Customize Node Graph
- Modify sensor rates if needed
- Add custom processing nodes
- Connect to navigation stack

### 4. Build and Run
```bash
# Generate code
Click "Generate Code" → Enter package name: "turtlebot3_bringup"

# Build
Click "Build"

# Launch
Click "Run"
```

## Using with ROS2 Navigation

### Install Nav2
```bash
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

### Run SLAM
```bash
# Terminal 1: Launch robot
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Run SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Drive around to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### Navigate Autonomously
```bash
# Launch navigation with saved map
ros2 launch nav2_bringup bringup_launch.py map:=~/my_map.yaml

# Send navigation goals via RViz or CLI
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

## Advanced Features

### TF Tree
```
odom
└── base_footprint
    └── base_link
        ├── wheel_left_link
        ├── wheel_right_link
        ├── caster_back_link
        ├── imu_link
        ├── base_scan (LiDAR)
        └── camera_link
            └── camera_optical_link
```

### Topics Overview
```bash
# List all topics
ros2 topic list

# Key topics:
/cmd_vel          # Velocity commands (input)
/odom             # Odometry (output)
/scan             # LiDAR data (output)
/imu              # IMU data (output)
/camera/image_raw # Camera images (output)
/tf               # Transform tree
/tf_static        # Static transforms
```

### Lifecycle Management
```bash
# List lifecycle nodes
ros2 lifecycle nodes

# Configure all nodes
ros2 lifecycle set /base_controller configure
ros2 lifecycle set /odometry_fusion configure
ros2 lifecycle set /lidar_driver configure

# Activate all nodes
ros2 lifecycle set /base_controller activate
ros2 lifecycle set /odometry_fusion activate
ros2 lifecycle set /lidar_driver activate
```

## Use Cases

### 1. SLAM and Mapping
- Build maps of indoor environments
- Use slam_toolbox or cartographer
- Save and load maps

### 2. Autonomous Navigation
- Navigate to goal poses
- Obstacle avoidance
- Path planning with Nav2

### 3. Research and Education
- Learn ROS2 fundamentals
- Test algorithms
- Multi-robot experiments

### 4. Delivery Robots
- Indoor delivery tasks
- Waypoint following
- Return to dock

### 5. Inspection
- Automated inspection tours
- Data collection
- Environmental monitoring

## Customization Ideas

### Beginner
- Change sensor rates
- Add more lifecycle nodes
- Modify robot dimensions

### Intermediate
- Add custom costmap layers
- Implement recovery behaviors
- Create path planning algorithms

### Advanced
- Multi-robot coordination
- Visual SLAM with camera
- Semantic mapping
- Behavior tree integration

## Troubleshooting

### LiDAR not spinning
- Check power connection
- Verify USB serial port
- Check /scan topic: `ros2 topic hz /scan`

### Odometry drift
- Calibrate wheel diameter
- Tune IMU fusion parameters
- Check for wheel slippage

### Navigation fails
- Verify all lifecycle nodes are ACTIVE
- Check TF tree: `ros2 run tf2_tools view_frames`
- Validate map: `ros2 topic echo /map -n 1`

### Camera not publishing
- Check Raspberry Pi camera enable
- Verify camera cable connection
- Test with: `ros2 topic hz /camera/image_raw`

## Performance Tips

1. **Reduce Sensor Rates**: Lower rates if system is overloaded
2. **Optimize Network**: Use wired connection for video
3. **Tune Parameters**: Adjust Nav2 parameters for your environment
4. **Monitor Resources**: Use `top` or `htop` on Raspberry Pi

## References

- [Official TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS2 Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3 GitHub](https://github.com/ROBOTIS-GIT/turtlebot3)

## Next Steps

1. Try autonomous navigation
2. Implement custom behaviors
3. Add more sensors
4. Connect to AI/ML models
5. Build multi-robot systems

## License

Apache 2.0 (Based on ROBOTIS TurtleBot3)

---

*Example created by RoboShire Team*
*Based on ROBOTIS TurtleBot3 Burger*
*Last updated: 2025-10-20*
