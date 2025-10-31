# Hardware Testing Panel Guide

**RoboShire v2.4.0 Feature**

## Overview

The Hardware Testing Panel is an interactive widget for testing sensors and actuators in real-time without needing separate tools like Arduino Serial Monitor or `ros2 topic echo`. Everything you need for hardware debugging is in one place.

## Features

- **Live Sensor Monitoring**: Real-time value display with units
- **Mini Graphs**: Visual feedback of last 100 samples per sensor
- **Signal Quality Indicators**: Know immediately if your sensor connection is good
- **Manual Motor Control**: Control motors/servos with sliders
- **Emergency Stop**: One-click safety stop for all actuators
- **Auto-Detection**: Automatically finds sensors and actuators from ROS2 topics
- **Connection Testing**: Verify all hardware components are working

## Opening the Panel

### Method 1: Menu
1. Hardware → Hardware Test Panel
2. Panel opens in a new window

### Method 2: Keyboard Shortcut
- Press **Ctrl+T**

## Using the Panel

### Auto-Detecting Hardware

1. Make sure your ROS2 nodes are running (Build → Run or F5)
2. Click **"🔍 Auto-Detect Hardware"**
3. The panel will scan all active ROS2 topics and automatically create:
   - Sensor displays for topics under `/sensors/`, `/imu`, `/temperature`, etc.
   - Motor controllers for topics like `/cmd_vel`, `/motor`, `/control/`

### Monitoring Sensors

Each sensor display shows:

```
┌─ Temperature ──────────────┐
│ 24.8 °C                 ✓  │
│ Signal: [████████░░] 95%   │
│ [Graph of last 10 seconds] │
└────────────────────────────┘
```

**Components:**
- **Name**: Sensor identifier (e.g., "temperature", "humidity")
- **Value**: Current reading with units
- **Status Indicator**:
  - ✓ (green) = Receiving data
  - ✗ (red) = No data / disconnected
- **Signal Strength Bar**: Shows update frequency (100% = 10 Hz or higher)
- **Mini Graph**: Visual trend of last 100 samples

### Controlling Motors

Each motor controller provides:

```
┌─ Left Motor ───────────────┐
│                             │
│ [-255] [====|====] [255]    │
│         Speed: 0            │
│                             │
│ [▶ Test] [⏹ Stop]          │
└─────────────────────────────┘
```

**Controls:**
- **Slider**: Drag to set motor speed (-255 to 255)
  - Negative values = Reverse
  - Positive values = Forward
  - 0 = Stop
- **Test Button**: Runs a test sequence (ramps to 100, then back to 0)
- **Stop Button**: Immediately stops the motor

### Emergency Stop

The **🛑 EMERGENCY STOP** button at the bottom:
- Immediately sets ALL motors to speed 0
- Use in case of unexpected behavior
- Always accessible with one click

### Testing Connection

Click **"🔌 Test Connection"** to verify:
- ROS2 node is running
- Number of sensors detected
- Number of actuators detected

Results appear in a dialog box with checkmarks for working components.

## Manual Sensor/Actuator Setup

If auto-detect doesn't find your hardware, you can manually add displays:

```python
# In hardware_test_panel.py, add:
panel.add_sensor("my_sensor", "/my/custom/topic", "units")
panel.add_motor("my_motor", "/cmd_vel")
```

## Supported Message Types

### Sensors
- `std_msgs/Float32` - Generic float values
- `std_msgs/Int32` - Integer values
- `sensor_msgs/Temperature` - Temperature readings
- `sensor_msgs/Imu` - IMU data (accelerometer, gyroscope)

### Actuators
- `geometry_msgs/Twist` - Velocity commands
- `std_msgs/Int32` - Direct motor commands

## Example: Testing Weather Station

1. Build and run your weather station package:
   ```bash
   # In RoboShire
   Build → Run (F5)
   ```

2. Open Hardware Test Panel (Ctrl+T)

3. Click "Auto-Detect Hardware"

4. You should see:
   - Temperature sensor display
   - Humidity sensor display
   - Pressure sensor display
   - Real-time graphs for each

5. Verify all sensors show ✓ (green checkmark)

6. Test connection to confirm all working

## Example: Testing Differential Drive Robot

1. Build and run your robot package:
   ```bash
   Build → Run (F5)
   ```

2. Open Hardware Test Panel (Ctrl+T)

3. Click "Auto-Detect Hardware"

4. You should see:
   - Left motor controller
   - Right motor controller
   - IMU sensor display (if equipped)
   - Wheel encoder displays (if equipped)

5. Test motors:
   - Move left motor slider to 100
   - Robot should move in a circle (right wheel stopped)
   - Click Stop
   - Move right motor slider to 100
   - Robot should move in opposite circle
   - Click Stop

6. Test both motors together:
   - Move both sliders to 100
   - Robot should move forward
   - Move both to -100
   - Robot should move backward

7. Always click Emergency Stop when done!

## Safety Features

### Speed Limiting
- Motors are limited to -255 to 255 range
- Can be configured in code for your robot's safety

### Timeout Protection
- Motors auto-stop after 5 seconds of no input (configurable)
- Prevents runaway if connection is lost

### Visual Feedback
- All actions have immediate visual feedback
- Easy to see current state at a glance

## Troubleshooting

### "ROS2 Python libraries not available"
**Solution**: Source your ROS2 workspace before launching RoboShire
```bash
source /opt/ros/humble/setup.bash  # Or your ROS2 distro
source ~/ros2_ws/install/setup.bash  # Your workspace
python3 -m roboshire
```

### "No devices found" after auto-detect
**Possible causes:**
1. ROS2 nodes not running → Click Build → Run (F5)
2. Topics have non-standard names → Use manual setup
3. Message types not supported → Check Supported Message Types section

### Sensor shows red ✗
**Possible causes:**
1. Node not publishing → Check Log Viewer for errors
2. Topic name mismatch → Verify with `ros2 topic list`
3. Message type incompatible → Check with `ros2 topic info /topic/name`

### Motors don't respond
**Possible causes:**
1. Wrong topic name → Check your node subscribes to the correct topic
2. Message type mismatch → Motor nodes must accept Twist messages
3. Hardware not connected → Check Arduino/ESP32 connection

## Integration with Other Tools

### With micro-ROS Agent
1. Start micro-ROS agent (Hardware → micro-ROS Agent, Ctrl+Shift+M)
2. Connect to your Arduino/ESP32
3. Open Hardware Test Panel (Ctrl+T)
4. Auto-detect will find micro-ROS topics
5. Test sensors and actuators through the panel

### With Node Graph
1. View your node connections in Node Graph tab (Ctrl+1)
2. Identify sensor/actuator topics
3. Open Hardware Test Panel to test them interactively

### With Log Viewer
1. Open Logs tab (Ctrl+4)
2. Open Hardware Test Panel
3. Send commands and watch logs in real-time
4. Debug issues immediately

## Advanced Usage

### Recording Sensor Data
Future feature: Click "💾 Log Data" to record sensor values to CSV for analysis.

### Custom Test Sequences
For advanced users, modify `hardware_test_panel.py` to add custom test sequences:

```python
def test_motors_spin(self):
    """Test sequence: spin in place"""
    self.send_motor("left", 100)
    self.send_motor("right", -100)
    time.sleep(2)
    self.send_motor("left", 0)
    self.send_motor("right", 0)
```

## Tips & Best Practices

1. **Always test in open space**: Motors can cause unexpected movement
2. **Use Emergency Stop liberally**: Better safe than sorry
3. **Monitor signal strength**: <70% may indicate connection issues
4. **Check graphs for noise**: Excessive noise may indicate electrical interference
5. **Test one component at a time**: Easier to isolate problems

## Comparison: Before vs. After

### Before RoboShire v2.4.0
```bash
# Terminal 1: Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Terminal 2: Monitor temperature
ros2 topic echo /sensors/temperature

# Terminal 3: Monitor humidity
ros2 topic echo /sensors/humidity

# Terminal 4: Send motor command
ros2 topic pub /cmd_vel geometry_msgs/Twist "..."

# Terminal 5: Check IMU
ros2 topic echo /imu/data

# Arduino Serial Monitor for debugging
# Switch between 5 windows constantly!
```

### With RoboShire v2.4.0
```
1. Ctrl+T (Open Hardware Test Panel)
2. Click "Auto-Detect Hardware"
3. Everything in one window with graphs!
4. Drag sliders to control motors
5. Click Emergency Stop when done
```

**Time saved: 90%** (from setup to testing)

## What's Next?

After mastering the Hardware Test Panel, explore:
- **[micro-ROS Agent Manager](./MICRO_ROS_AGENT_MANAGER.md)** - GUI for micro-ROS agent
- **[Visual URDF Editor](./URDF_VISUAL_EDITOR_GUIDE.md)** (v2.5.0) - Design robots visually
- **[Performance Profiler](../guides/PERFORMANCE_PROFILING.md)** - Optimize your robot

## Feedback

Found a bug or have a feature request?
Open an issue: https://github.com/your-org/roboshire/issues

---

**Version**: v2.4.0
**Last Updated**: 2025-10-29
**Author**: RoboShire Team
