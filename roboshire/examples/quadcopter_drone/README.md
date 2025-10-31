# Quadcopter Drone Example

**Difficulty**: Advanced
**Category**: Aerial Robotics
**Total Mass**: 1.5 kg
**DOF**: 6 (3 position + 3 orientation)

---

## Overview

This example features a quadcopter drone with X-configuration, inspired by PX4 and ArduPilot flight control systems. It demonstrates aerial robotics principles including flight dynamics, attitude control, and autonomous navigation.

### Key Features

- **4 Rotors**: Continuous rotation motors in X-configuration
- **IMU Sensor**: 9-DOF (accelerometer, gyroscope, magnetometer)
- **GPS Sensor**: For outdoor positioning
- **Barometer**: For altitude measurement
- **Flight Controller**: PID-based attitude and position control
- **6 DOF Movement**: Full 3D position and orientation control

---

## Robot Structure

### Body Frame
- **Main Body**: Central frame (25cm × 25cm × 10cm)
- **Mass**: 1.0 kg (includes battery, electronics, sensors)
- **Material**: Carbon fiber (lightweight, strong)
- **Sensors**: IMU at center, GPS on top

### Rotor Configuration (X-Pattern)
```
     Front
       ^
   1   |   2
      \|/
  <--- X --->
      /|\
   4   |   3
       v
     Rear
```

**Rotor Positions**:
- **Rotor 1** (Front-Left): +X, +Y, 45° from forward
- **Rotor 2** (Front-Right): +X, -Y, 45° from forward
- **Rotor 3** (Rear-Right): -X, -Y, 45° from forward
- **Rotor 4** (Rear-Left): -X, +Y, 45° from forward

**Rotor Specifications**:
- Diameter: 10 inches (0.254 m)
- Mass per rotor: 0.125 kg (motor + propeller)
- Max thrust: 5N per rotor (20N total)
- Max RPM: 10,000
- Motor type: Brushless DC

### Flight Dynamics

**Control Inputs** (4 channels):
1. **Throttle**: Collective thrust (up/down)
2. **Roll**: Rotation around X-axis (left/right tilt)
3. **Pitch**: Rotation around Y-axis (forward/back tilt)
4. **Yaw**: Rotation around Z-axis (heading)

**Thrust Mixing**:
```
Rotor 1 (FL): +Throttle +Roll -Pitch +Yaw
Rotor 2 (FR): +Throttle -Roll -Pitch -Yaw
Rotor 3 (RR): +Throttle -Roll +Pitch +Yaw
Rotor 4 (RL): +Throttle +Roll +Pitch -Yaw
```

**Flight Modes**:
- **Manual**: Direct pilot control
- **Stabilize**: Attitude hold (level on stick release)
- **Altitude Hold**: Maintain fixed altitude
- **Position Hold**: GPS-based hover
- **Auto**: Waypoint navigation

---

## Coordinate Frames

### Standard Aviation Conventions
- **X-axis**: Forward (nose direction)
- **Y-axis**: Right wing
- **Z-axis**: Down (NED convention) or Up (ROS convention)

### Frame Tree
```
map (global frame)
└── odom (odometry frame)
    └── base_link (drone body)
        ├── imu_link
        ├── gps_link
        ├── rotor_1_link
        ├── rotor_2_link
        ├── rotor_3_link
        └── rotor_4_link
```

---

## Sensors

### IMU (Inertial Measurement Unit)
- **Type**: 9-DOF (MPU9250 equivalent)
- **Update Rate**: 100 Hz
- **Measurements**:
  - Accelerometer: 3-axis linear acceleration
  - Gyroscope: 3-axis angular velocity
  - Magnetometer: 3-axis magnetic field
- **Use**: Attitude estimation, stabilization

### GPS (Global Positioning System)
- **Type**: GPS/GNSS receiver
- **Update Rate**: 5-10 Hz
- **Accuracy**: ~3m (consumer grade)
- **Measurements**: Latitude, longitude, altitude
- **Use**: Position estimation, waypoint navigation

### Barometer
- **Type**: Pressure sensor (BMP280 equivalent)
- **Update Rate**: 20 Hz
- **Accuracy**: ±1m altitude
- **Use**: Altitude hold, climb rate

---

## Flight Controller

### Control Architecture

**Cascaded PID Control**:
```
Position Controller (outer loop)
  ↓
Velocity Controller (middle loop)
  ↓
Attitude Controller (inner loop)
  ↓
Motor Mixing
  ↓
Rotor Thrust Commands
```

### PID Tuning Parameters

**Attitude Control** (50-100 Hz):
- Roll: P=4.5, I=0.0, D=0.18
- Pitch: P=4.5, I=0.0, D=0.18
- Yaw: P=2.8, I=0.0, D=0.0

**Position Control** (10-20 Hz):
- X/Y: P=1.0, I=0.0, D=0.5
- Z (altitude): P=5.0, I=2.0, D=0.5

**Velocity Control** (20-50 Hz):
- X/Y: P=2.0, I=0.5, D=0.0
- Z: P=3.0, I=1.0, D=0.0

---

## Node Graph

The included node graph provides:

1. **Flight Controller** (Lifecycle)
   - Inner loop: Attitude control (roll/pitch/yaw)
   - Outer loop: Position control (x/y/z)
   - Motor mixing and thrust commands
   - Subscribes to: IMU, GPS, barometer, setpoints
   - Publishes to: Motor commands

2. **State Estimator**
   - Sensor fusion (IMU + GPS + Barometer)
   - Estimates: position, velocity, attitude
   - Extended Kalman Filter (EKF)
   - Publishes: Odometry, TF transforms

3. **Mission Planner**
   - Waypoint management
   - Path planning (A*, RRT)
   - Obstacle avoidance
   - Publishes: Position setpoints

4. **RC Input Handler**
   - Processes pilot commands
   - Mode switching (manual/auto)
   - Safety checks (low battery, signal loss)

5. **Telemetry Logger**
   - Records flight data
   - Real-time monitoring
   - Diagnostics

---

## Getting Started

### 1. Load the Example
- Open RoboShire
- File > New from Example
- Select "Quadcopter Drone (PX4 Style)"

### 2. Explore the URDF
- Inspect the X-configuration in URDF tree
- Note the rotor positions and orientations
- Check sensor placements

### 3. Generate Code
- Review the node graph (flight controller architecture)
- Click "Generate Code"
- Examine generated flight controller node

### 4. Build & Run
- Click "Build" (Ctrl+B)
- Launch nodes (Ctrl+R)
- Monitor in Node Status tab

### 5. Simulate Flight
- Launch Gazebo simulation
- Test manual control (keyboard/joystick)
- Try different flight modes

---

## Integration Guides

### Gazebo Simulation

**Add Physics Plugins**:
```xml
<!-- In URDF, add to each rotor link -->
<gazebo reference="rotor_1_link">
  <plugin name="rotor_1_motor" filename="libgazebo_motor_model.so">
    <motorNumber>0</motorNumber>
    <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
  </plugin>
</gazebo>
```

**Launch Simulation**:
```bash
ros2 launch quadcopter_drone gazebo.launch.py

# Arm and takeoff
ros2 service call /flight_controller/arm std_srvs/srv/Trigger
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "..."
```

### PX4 Integration

For hardware flights with real PX4 flight controller:

1. **MAVROS Bridge**:
```bash
ros2 run mavros mavros_node --fcu_url=/dev/ttyACM0:921600

# RoboShire talks to MAVROS, MAVROS talks to PX4
```

2. **Message Mapping**:
- RoboShire cmd_vel → MAVROS setpoint_velocity
- MAVROS imu_data → RoboShire sensor input
- MAVROS local_position → RoboShire odometry

3. **Mode Sync**:
- Map flight modes between RoboShire and PX4
- Monitor safety states

### ArduPilot Integration

Similar to PX4, but use ArduPilot-specific parameters:

```bash
# MAVLink connection
ros2 run mavros mavros_node --fcu_url=udp://127.0.0.1:14550@

# ArduPilot parameter configuration
ros2 service call /mavros/param/set ...
```

---

## Tuning & Configuration

### PID Tuning Process

1. **Attitude Control (Inner Loop)**:
   - Start with low P gains (1.0)
   - Increase P until oscillations appear
   - Reduce P by 50%
   - Add D term (typically P/10)
   - Add I term only if drift occurs

2. **Position Control (Outer Loop)**:
   - Tune attitude first (must be stable)
   - Start with P=0.5
   - Increase until responsive
   - Add D for damping

3. **Testing**:
   - Test in simulation first
   - Start in manual mode
   - Progress to stabilize → altitude hold → position hold
   - Test auto mode last

### Safety Configuration

**Geofence**:
- Max altitude: 120m (400ft)
- Max distance: 500m from home
- Return-to-home on boundary

**Failsafes**:
- Low battery: Auto-land
- RC signal loss: Return-to-home
- GPS loss: Switch to manual/stabilize
- Motor failure: Emergency land

---

## Flight Checklist

### Pre-Flight
- [ ] Battery fully charged (>95%)
- [ ] Propellers secure and undamaged
- [ ] GPS has fix (>6 satellites)
- [ ] IMU calibrated
- [ ] Compass calibrated
- [ ] RC transmitter on and bound
- [ ] Failsafes configured
- [ ] Flight area clear

### Post-Flight
- [ ] Review flight logs
- [ ] Check for errors/warnings
- [ ] Inspect airframe for damage
- [ ] Verify motor temperatures normal
- [ ] Save flight data for analysis

---

## Advanced Topics

### Sensor Fusion (EKF)

The state estimator uses Extended Kalman Filter to fuse:
- IMU: High-rate attitude updates
- GPS: Low-rate position updates
- Barometer: Medium-rate altitude
- Magnetometer: Heading reference

**State Vector**: [x, y, z, vx, vy, vz, roll, pitch, yaw, ...]

### Path Planning

**A* Algorithm** for global planning:
- Grid-based map
- Considers altitude constraints
- Avoids no-fly zones

**RRT** for dynamic replanning:
- Rapidly-exploring random tree
- Online obstacle avoidance
- Suitable for complex environments

### Computer Vision Integration

Add camera for visual tasks:

1. **Downward Camera**: Ground tracking, landing
2. **Forward Camera**: Obstacle detection, inspection
3. **360° Camera**: Full situational awareness

**Visual SLAM** (Simultaneous Localization and Mapping):
- ORB-SLAM2, RTAB-Map
- GPS-denied navigation
- Indoor flights

---

## Common Issues

### Drone Drifts
- **Cause**: IMU not calibrated, wind, trim incorrect
- **Fix**: Recalibrate IMU on level surface, adjust trim

### Unstable Hover
- **Cause**: PID gains too aggressive
- **Fix**: Reduce P gain, increase D gain

### GPS Not Working
- **Cause**: Poor satellite visibility, interference
- **Fix**: Move to open area, check antenna, wait for fix

### Motors Not Spinning
- **Cause**: Not armed, safety switch, ESC calibration
- **Fix**: Arm vehicle, check safety procedures

---

## Learning Resources

### Flight Dynamics
- "Small Unmanned Aircraft" by Randal W. Beard
- "Quadcopter Dynamics" - MIT OpenCourseWare
- PX4 User Guide: https://docs.px4.io/

### ROS2 Integration
- `px4_ros_com` - PX4 ROS2 interface
- `mavros` - MAVLink to ROS bridge
- `drone_controller` - Example implementations

### Simulation
- Gazebo drone models
- AirSim (Microsoft) - Photorealistic simulator
- Webots - Robot simulator with drone support

### Communities
- PX4 Discussion Forum
- ArduPilot Forum
- ROS Aerial Robotics SIG

---

## Next Steps

### Beginner
1. Understand X-configuration vs +-configuration
2. Learn basic flight controls (roll, pitch, yaw, throttle)
3. Practice in simulation (no risk!)

### Intermediate
1. Implement simple waypoint navigation
2. Add telemetry visualization
3. Test altitude hold mode

### Advanced
1. Implement full autonomous mission
2. Add obstacle avoidance
3. Multi-drone coordination (swarm)
4. Computer vision integration

---

## Safety Warning

⚠️ **IMPORTANT**: Quadcopters are potentially dangerous:
- Propellers can cause serious injury
- Always test in simulation first
- Follow local regulations (FAA Part 107 in USA)
- Never fly near people, airports, or restricted areas
- Maintain line of sight
- Register drone if required

---

## Credits

Inspired by PX4 Autopilot and ArduPilot projects. This is an educational example for learning drone control systems.

**Created by**: RoboShire Team
**Version**: 1.0.0
**License**: MIT
**Tested with**: ROS2 Humble, Gazebo Classic 11
