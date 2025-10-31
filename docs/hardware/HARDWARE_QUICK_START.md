# Hardware Quick Start: Build Your First Robot in 15 Minutes

**Difficulty**: Beginner
**Time**: 15-30 minutes
**Cost**: $0 (using simulation), or $20 (with real hardware)
**What You'll Build**: A weather monitoring station that reads sensors and publishes data to ROS2

---

## What You'll Learn

By the end of this guide, you'll:
- ✅ Use RoboShire's Workflow Wizard to design a hardware project
- ✅ Generate Arduino code automatically from the GUI
- ✅ Understand how hardware connects to ROS2 topics
- ✅ Build and run a complete ROS2 package
- ✅ See sensor data flowing through the system

---

## Prerequisites

### Required
- RoboShire GUI installed and working
- ROS2 Humble or newer installed
- Basic understanding of sensors (e.g., "temperature sensor measures degrees")

### Optional (for real hardware)
- ESP32 development board (~$8)
- DHT22 temperature/humidity sensor (~$3)
- BMP280 pressure sensor (~$4)
- Breadboard and jumper wires (~$5)
- USB cable for ESP32

**Don't have hardware?** No problem! This guide works in **simulation mode** too.

---

## Step 1: Launch RoboShire and Open Workflow Wizard (2 minutes)

### 1.1 Start RoboShire
```bash
cd /mnt/hgfs/ROS2_PROJECT
python3 -m roboshire
```

**Expected**: RoboShire GUI opens with welcome screen

### 1.2 Open Workflow Wizard
- Click **"Workflow Wizard"** from the welcome screen, OR
- Menu: **Tools → Workflow Wizard**, OR
- Keyboard shortcut: **Ctrl+W**

**Expected**: Wizard window opens showing "Step 1: URDF Selection"

---

## Step 2: Choose Weather Station Preset (3 minutes)

### 2.1 Select "Without URDF" for Sensor Station

On Step 1:
1. Select radio button: **"Without URDF (sensor stations, data loggers)"**
2. **Why?** Weather stations don't need robot structure (no moving parts)
3. Click **"Next"** button

### 2.2 Load Weather Station Preset

On Step 2 (Hardware Configuration):
1. Look for **"Quick Presets"** section at the top
2. Click **"Weather Monitor Station"** button

**Expected**: Form auto-fills with:
- **Microcontroller**: ESP32 (WiFi + Bluetooth)
- **Sensors**: DHT22, BMP280, Light Sensor (LDR)
- **Communication**: Serial (UART)

### 2.3 Review Hardware Selection

Take a moment to see what was selected:
- **Sensors tab**: Temperature, humidity, pressure, light sensors
- **Actuators tab**: None (this is a passive monitoring station)
- **Communication tab**: Serial UART checked

**Optional**: Click "Add More Hardware" to explore options, but preset is perfect for this guide

Click **"Next"** to proceed

---

## Step 3: Configure Project Settings (2 minutes)

### 3.1 Set Project Name

On Step 3 (Project Goals):
1. **Project Name**: `weather_station_demo`
   - Use lowercase with underscores (snake_case)
   - No spaces or special characters
2. **Workspace Folder**: Click **"Browse"** and select `workspace`
   - Default: `/mnt/hgfs/ROS2_PROJECT/workspace`

### 3.2 Choose Project Goal

1. **Project Goal**: Select **"Environmental Monitoring - Weather, air quality"**
2. **Description**: (auto-filled, but you can customize)
   ```
   Weather monitoring station that reads temperature, humidity,
   pressure, and light level. Data published to ROS2 topics for
   logging and visualization.
   ```

Click **"Next"**

---

## Step 4: Get Arduino Code (3 minutes)

### 4.1 View Generated Code

On Step 4 (Microcontroller Code):
1. **Template**: Should show **"Standard Arduino (.ino) - Recommended"**
2. **Code Preview**: Scroll through the generated code

**Key sections to notice**:
```cpp
// Pin definitions
#define DHT_PIN 4
#define LIGHT_SENSOR_PIN 34

// Sensor objects
DHT dht(DHT_PIN, DHT22);
Adafruit_BMP280 bmp;

// In loop():
float temp = dht.readTemperature();
float humidity = dht.readHumidity();
Serial.print("TEMP:"); Serial.println(temp);
```

### 4.2 Save Arduino Code (if using real hardware)

**Option A: Copy from GUI** (Current method):
1. Click anywhere in code preview
2. **Ctrl+A** (select all)
3. **Ctrl+C** (copy)
4. Open Arduino IDE
5. **Ctrl+V** (paste)
6. Save as `weather_station_demo.ino`

**Option B: Use Example** (Easier):
1. Close wizard for now
2. Navigate to: `roboshire/examples/weather_station/arduino_code.ino`
3. Open directly in Arduino IDE

**Note**: Auto-save feature coming in future version

Click **"Next"** to continue

---

## Step 5: Understand Data Flow (2 minutes)

### 5.1 View Data Flow Diagram

On Step 5 (ROS2 Data Flow):
- Review the visual diagram showing:
  - Arduino reads sensors
  - Serial sends data to ROS2
  - ROS2 nodes process data
  - Topics distribute data to subscribers

### 5.2 Note the Topics

Your weather station will publish to:
- `/sensors/temperature` (Float32)
- `/sensors/humidity` (Float32)
- `/sensors/pressure` (Float32)
- `/sensors/light` (Int32)

**These topic names matter!** You'll use them later to view data.

Click **"Next"**

---

## Step 6: Skip Lifecycle (Optional Feature) (1 minute)

On Step 5a (Lifecycle Nodes):
- **For beginners**: Click **"Skip (Use Standard Nodes)"**
- **Why skip?** Lifecycle adds complexity for production systems
- **When to use?** Advanced projects needing fault tolerance

Click **"Skip"** or **"Next"**

---

## Step 7: Review Build Guide and Finish (2 minutes)

### 7.1 Read Build Steps

On Step 6 (Build & Test Guide):
1. Review the checklist showing:
   - Hardware assembly steps
   - Arduino upload procedure
   - ROS2 build commands
   - Testing procedures

### 7.2 Finish Wizard

1. Click **"Finish and Generate Project"** button
2. **Expected**: Success dialog appears
3. Click **"OK"**

**What just happened?**
- ROS2 package created at: `workspace/src/weather_station_demo/`
- Node graph design saved
- Project added to RoboShire's project list

---

## Step 8: Explore Generated ROS2 Package (3 minutes)

### 8.1 View Generated Files

Open terminal:
```bash
cd workspace/src/weather_station_demo
ls -la
```

**You should see**:
```
weather_station_demo/
├── package.xml          # ROS2 package metadata
├── setup.py             # Python package installer
├── setup.cfg            # Setup configuration
├── resource/            # Package markers
├── weather_station_demo/  # Python module
│   ├── __init__.py
│   ├── temperature_sensor.py
│   ├── humidity_sensor.py
│   ├── pressure_sensor.py
│   └── light_sensor.py
└── launch/
    └── weather_station_demo_launch.py
```

### 8.2 Examine a Generated Node

```bash
cat weather_station_demo/temperature_sensor.py
```

**Key parts**:
```python
class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('temperature_sensor')

        # Publisher for temperature data
        self.publisher = self.create_publisher(
            Float32,
            '/sensors/temperature',
            10
        )

        # Timer to publish at 1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Read sensor and publish data"""
        msg = Float32()
        # TODO: Read from serial bridge
        msg.data = 25.0  # Placeholder
        self.publisher.publish(msg)
```

**Note**: Generated nodes are templates - you'll connect them to the serial bridge later

---

## Step 9: Build the ROS2 Package (2 minutes)

### 9.1 Build with colcon

```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace
colcon build --packages-select weather_station_demo
```

**Expected output**:
```
Starting >>> weather_station_demo
Finished <<< weather_station_demo [0.75s]

Summary: 1 package finished [1.02s]
```

**If build fails**: Check error message, usually:
- Missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Python syntax errors: Review generated code

### 9.2 Source the Workspace

```bash
source install/setup.bash
```

**Important**: Run this in every new terminal before using your package

---

## Step 10: Test the System (3 minutes)

### 10.1 Launch Nodes (Simulation Mode)

```bash
ros2 launch weather_station_demo weather_station_demo_launch.py
```

**Expected output**:
```
[INFO] [temperature_sensor]: temperature_sensor starting...
[INFO] [humidity_sensor]: humidity_sensor starting...
[INFO] [pressure_sensor]: pressure_sensor starting...
[INFO] [light_sensor]: light_sensor starting...
[INFO] [temperature_sensor]: Publishing to /sensors/temperature
```

### 10.2 View Published Data

**Open a new terminal** (keep launch running):
```bash
source install/setup.bash
ros2 topic list
```

**Expected**:
```
/sensors/temperature
/sensors/humidity
/sensors/pressure
/sensors/light
/rosout
```

### 10.3 Echo Temperature Data

```bash
ros2 topic echo /sensors/temperature
```

**Expected** (placeholder values):
```yaml
data: 25.0
---
data: 25.0
---
```

**Press Ctrl+C to stop**

### 10.4 Check All Topics

```bash
ros2 topic hz /sensors/temperature
```

**Expected**:
```
average rate: 1.000
```

Perfect! Your weather station is publishing at 1Hz as designed.

---

## 🎉 Success! What You've Accomplished

In just 15-30 minutes, you:
- ✅ Used RoboShire Workflow Wizard to design a hardware project
- ✅ Generated Arduino code for ESP32 + sensors
- ✅ Created a complete ROS2 package automatically
- ✅ Built and launched ROS2 nodes
- ✅ Verified data flowing through topics

---

## Next Steps

### Level 1: Connect Real Hardware (if you have it)
**Time**: +30 minutes
**Follow**: [Weather Station Complete Example](examples/weather_station/README.md)
**What you'll do**:
- Wire up ESP32, DHT22, BMP280 sensors
- Flash Arduino code to ESP32
- Implement serial bridge to read real sensor data
- See live temperature/humidity updates

### Level 2: Visualize Data
**Time**: +15 minutes
**What you'll do**:
- Use RoboShire's Data Plotter widget
- Graph temperature over time
- Set up alerts for threshold values
- Export data to CSV

### Level 3: Build a Mobile Robot
**Time**: +2 hours
**Follow**: [Differential Drive Robot Example](examples/differential_drive_robot/README.md)
**What you'll do**:
- Add motors, IMU, ultrasonic sensor
- Implement odometry and control
- Navigate using keyboard teleop
- Simulate in MuJoCo before hardware

### Level 4: Implement Serial Bridge (Critical Skill)
**Time**: +45 minutes
**Follow**: [Serial Bridge Implementation Guide](communication/serial_bridge_implementation.md)
**What you'll do**:
- Learn PySerial for Arduino ↔ ROS2 communication
- Parse sensor data from serial messages
- Handle errors and reconnection
- Test bidirectional communication

---

## Understanding What Happened

### The Workflow Wizard Created:

1. **Arduino Code Template** (Step 4)
   - Hardware-specific code for your sensors
   - Serial communication protocol
   - Timing and data formatting
   - Ready to flash to ESP32

2. **ROS2 Package** (Auto-generated)
   - Individual Python nodes for each sensor
   - Publishers for each data topic
   - Launch file to start all nodes
   - Package metadata (package.xml, setup.py)

3. **Project Configuration**
   - Saved in RoboShire's project database
   - Can reload and modify later
   - Node graph design preserved

### The Data Flow:

```
┌─────────────┐         ┌──────────────┐         ┌────────────┐
│   ESP32     │ Serial  │ Serial Bridge│  ROS2   │  Sensor    │
│   Arduino   │────────>│  (PySerial)  │ Topics ─>│  Nodes     │
│             │  UART   │   ROS2 Node  │         │            │
└─────────────┘         └──────────────┘         └────────────┘
     ↑                                                  │
     │                                                  ↓
┌─────────────┐                                  ┌────────────┐
│   Sensors   │                                  │ Subscribers│
│ DHT22, BMP  │                                  │ Data Logger│
└─────────────┘                                  └────────────┘
```

**Current implementation**: Sensor nodes are placeholders (publish dummy data)
**Next step**: Implement serial bridge to connect real hardware

---

## Troubleshooting Quick Start

### Wizard doesn't open
**Solution**:
- Check RoboShire is fully loaded (wait 5 seconds)
- Try menu: Tools → Workflow Wizard
- Restart RoboShire if needed

### Build fails with "package not found"
**Solution**:
```bash
# Install missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Launch file not found
**Solution**:
```bash
# Make sure you sourced the workspace
source install/setup.bash
# Check package was built
ros2 pkg list | grep weather_station
```

### Topics not visible
**Solution**:
```bash
# Verify nodes are running
ros2 node list
# Should show: /temperature_sensor, /humidity_sensor, etc.

# Check if topics exist
ros2 topic list
```

### Want to modify project
**Solution**:
- Open RoboShire GUI
- File → Open Project
- Select "weather_station_demo"
- Edit node graph visually
- Re-generate code

---

## Quick Reference Commands

```bash
# Build ROS2 package
cd workspace
colcon build --packages-select weather_station_demo

# Source workspace
source install/setup.bash

# Launch all nodes
ros2 launch weather_station_demo weather_station_demo_launch.py

# List topics
ros2 topic list

# View data (in new terminal)
ros2 topic echo /sensors/temperature

# Check publish rate
ros2 topic hz /sensors/temperature

# View node graph
ros2 run rqt_graph rqt_graph

# Kill all nodes
Ctrl+C (in launch terminal)
```

---

## Concepts Explained (ELI10)

### What is a ROS2 Topic?
Think of it like a radio station. The sensor node is the radio transmitter broadcasting temperature data. Any other node can tune in to that station (`/sensors/temperature`) and listen to the broadcasts. Multiple nodes can listen at once!

### What is a Publisher?
The part of the code that broadcasts data. Like a person speaking into a microphone at a radio station. In our case, the temperature sensor node publishes Float32 messages.

### What is a Node?
An independent program that does one job. We have separate nodes for temperature, humidity, pressure, and light. They all run at the same time (concurrently) and communicate via topics.

### What is Serial Communication?
How computers talk to Arduino. It's like a phone call - data goes back and forth over a USB cable using a protocol (rules for how to format messages).

### What is a Launch File?
A script that starts multiple nodes at once. Instead of running 4 commands to start 4 nodes, one launch file starts them all. Like pressing a "start everything" button.

---

## What's Different from Manual Coding?

**Traditional approach** (manual):
```
1. Write package.xml by hand (20 lines of XML)
2. Write setup.py by hand (30 lines of Python)
3. Create directory structure manually
4. Write each node from scratch (50+ lines per node)
5. Write launch file by hand (40+ lines)
Total time: 2-3 hours for 4 nodes
```

**RoboShire approach** (visual):
```
1. Click through Workflow Wizard (6 steps, 15 minutes)
2. Auto-generates everything
3. Build and test immediately
Total time: 15-30 minutes
```

**Time saved**: 1.5-2.5 hours per project

---

## FAQs

**Q: Do I need to know Python to use RoboShire?**
A: For basic projects, no. The wizard generates code for you. For advanced customization, basic Python helps.

**Q: Can I modify the generated code?**
A: Yes! Generated code is clean, commented, and meant to be edited. After generation, it's yours to customize.

**Q: What if I need a sensor not in the wizard?**
A: Use a similar sensor as a template, generate the code, then modify it for your specific sensor. See [Sensors Guide](SENSORS_GUIDE.md).

**Q: How do I connect multiple Arduinos?**
A: Run multiple serial bridge nodes with different ports (e.g., `/dev/ttyACM0`, `/dev/ttyACM1`). See [Serial Bridge Guide](communication/serial_bridge_implementation.md).

**Q: Can I use micro-ROS instead of serial?**
A: Yes! Select micro-ROS option in Step 2. This runs ROS2 directly on ESP32/Teensy (more advanced). See wizard documentation.

**Q: Where is the node graph editor?**
A: After finishing wizard, click "Open Node Graph" in project view. You can add logic nodes, connect topics visually, and regenerate code.

---

## Congratulations!

You've completed the RoboShire Hardware Quick Start and built your first robotics project in record time. You're ready to explore more advanced features, add real hardware, and create custom robots.

**Where to go next?** Pick your path:
- 🔧 **Hardware Focus**: [Weather Station Complete Example](examples/weather_station/README.md)
- 🤖 **Mobile Robotics**: [Differential Drive Robot](examples/differential_drive_robot/README.md)
- 💻 **Serial Communication**: [Serial Bridge Implementation](communication/serial_bridge_implementation.md)
- 📚 **Deep Dive**: [Sensors Guide](SENSORS_GUIDE.md) + [Actuators Guide](ACTUATORS_GUIDE.md)

Happy building!

---

**Document Info**
- **Version**: 1.0.0
- **Last Updated**: 2025-10-29
- **Tested On**: RoboShire v2.3.0, ROS2 Humble, Ubuntu 22.04
- **Feedback**: Report issues or suggest improvements
