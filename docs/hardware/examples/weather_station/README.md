# Weather Station Example - Complete Hardware Project

**Difficulty**: Beginner
**Time**: 2-3 hours total
**Cost**: ~$20 USD
**Skills Learned**: Sensor reading, serial communication, ROS2 topics, data visualization

---

## Project Overview

Build a complete IoT weather monitoring station that reads temperature, humidity, atmospheric pressure, and light levels, then publishes data to ROS2 topics for logging, visualization, and analysis.

### What You'll Build

```
┌─────────────────────────────────────────┐
│       ESP32 Weather Station             │
│                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────┐ │
│  │  DHT22   │  │  BMP280  │  │ LDR  │ │
│  │ Temp+Hum │  │ Pressure │  │Light │ │
│  └─────┬────┘  └─────┬────┘  └───┬──┘ │
│        │             │            │    │
│        └─────────────┴────────────┘    │
│                  ESP32                  │
│                   │                     │
└───────────────────┼─────────────────────┘
                    │ USB
                    ↓
              ┌─────────────┐
              │  Computer   │
              │  ROS2 Node  │
              │ Serial Bridge│
              └─────────────┘
                    │
        ┌───────────┴───────────┐
        ↓                       ↓
  ┌──────────┐          ┌──────────┐
  │ Data     │          │ Plotting │
  │ Logger   │          │ Widget   │
  └──────────┘          └──────────┘
```

### Features

- **4 Sensors**: Temperature, humidity, pressure, light
- **Real-time Updates**: 1Hz sensor publishing
- **ROS2 Integration**: Standard ROS2 topics for interoperability
- **Data Logging**: CSV export for analysis
- **Visualization**: Real-time graphs in RoboShire GUI
- **Low Cost**: Total hardware cost ~$20
- **Beginner-Friendly**: No soldering required (breadboard)

### Learning Outcomes

After completing this project:
- ✅ Understand sensor integration with microcontrollers
- ✅ Implement serial communication protocols
- ✅ Work with ROS2 topics and publishers
- ✅ Read sensor datasheets and configure I2C/digital pins
- ✅ Debug hardware-software integration issues
- ✅ Visualize sensor data in real-time

---

## Project Files

| File | Description |
|------|-------------|
| [hardware_bom.md](hardware_bom.md) | Bill of Materials - shopping list with links |
| [wiring_diagram.md](wiring_diagram.md) | Pin connections and circuit diagram |
| [setup_guide.md](setup_guide.md) | Step-by-step assembly and configuration |
| [testing_guide.md](testing_guide.md) | Verification and troubleshooting procedures |

---

## Quick Start

### Prerequisites Checklist

- [ ] RoboShire v2.3.0+ installed
- [ ] ROS2 Humble+ installed
- [ ] Arduino IDE installed
- [ ] PySerial installed (`pip3 install pyserial`)
- [ ] Hardware components purchased (see [BOM](hardware_bom.md))
- [ ] USB cable for ESP32

### 30-Second Overview

1. **Wire hardware** (15 minutes) - Follow [wiring diagram](wiring_diagram.md)
2. **Flash Arduino code** (5 minutes) - Upload provided `.ino` file
3. **Build ROS2 package** (5 minutes) - Use RoboShire Workflow Wizard
4. **Test system** (5 minutes) - Verify data flow

**Total time**: ~30 minutes if you have all hardware ready

---

## Detailed Step-by-Step

### Step 1: Purchase Hardware

See [Bill of Materials](hardware_bom.md) for:
- Exact part numbers
- Shopping links (Amazon, Adafruit, AliExpress)
- Cost breakdown
- Alternative component options

**Budget**:
- Minimal: $18 (ESP32 + DHT22 only)
- Recommended: $22 (ESP32 + DHT22 + BMP280 + LDR)
- Deluxe: $30 (Add enclosure + power supply)

### Step 2: Assemble Hardware

Follow [Wiring Diagram](wiring_diagram.md):
- Pin-by-pin connections
- Breadboard layout
- Power connections
- I2C bus setup
- Pull-up resistors

**Time**: 15-20 minutes
**Tools needed**: None (breadboard, jumper wires only)

### Step 3: Install Arduino Libraries

Open Arduino IDE:
```
Tools → Manage Libraries → Search and install:
1. DHT sensor library (by Adafruit)
2. Adafruit BMP280 Library
3. Adafruit Unified Sensor
```

**Alternative** (faster):
```bash
# Using Arduino CLI
arduino-cli lib install "DHT sensor library"
arduino-cli lib install "Adafruit BMP280 Library"
arduino-cli lib install "Adafruit Unified Sensor"
```

### Step 4: Flash Arduino Code

**Option A: Use Pre-made Example** (Easiest)
```bash
# Open the existing example
cd /mnt/hgfs/ROS2_PROJECT/roboshire/examples/weather_station
# Open arduino_code.ino in Arduino IDE
```

**Option B: Generate from RoboShire**
1. Launch RoboShire GUI
2. Tools → Workflow Wizard
3. Select "Weather Monitor Station" preset
4. Copy generated code from Step 4

**Upload to ESP32**:
1. Connect ESP32 via USB
2. Tools → Board → ESP32 Dev Module
3. Tools → Port → /dev/ttyUSB0 (or detected port)
4. Click Upload button
5. Wait for "Done uploading"

**Verify**:
- Open Serial Monitor (Tools → Serial Monitor)
- Set baud rate to 115200
- Should see:
  ```
  STATUS:READY
  TEMP:25.3
  HUMIDITY:60.5
  PRESSURE:1013.25
  LIGHT:450
  ```

### Step 5: Create ROS2 Package

**Using RoboShire Workflow Wizard**:
1. Launch RoboShire
2. Workflow Wizard → Weather Monitor preset
3. Set project name: `weather_station_demo`
4. Finish wizard → generates ROS2 package

**Manual alternative**:
```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace/src
ros2 pkg create --build-type ament_python weather_station_demo \
  --dependencies rclpy std_msgs sensor_msgs
```

### Step 6: Implement Serial Bridge

Follow [Serial Bridge Implementation Guide](../../communication/serial_bridge_implementation.md):

**Quick version**:
1. Copy serial bridge node to your package
2. Add entry point to setup.py
3. Configure for weather station topics

**Or use the complete implementation** from the guide (300 lines, production-ready).

### Step 7: Build and Test

```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace

# Build package
colcon build --packages-select weather_station_demo

# Source workspace
source install/setup.bash

# Launch serial bridge
ros2 run weather_station_demo serial_bridge
```

**Expected output**:
```
[INFO] [serial_bridge]: Connected to /dev/ttyUSB0 at 115200 baud
[INFO] [serial_bridge]: Publishing temperature data
```

**Verify topics** (new terminal):
```bash
source install/setup.bash
ros2 topic list
# Should show:
#   /sensors/temperature
#   /sensors/humidity
#   /sensors/pressure
#   /sensors/light

ros2 topic echo /sensors/temperature
# Should show real-time data:
# data: 25.3
# ---
# data: 25.4
```

### Step 8: Visualize Data

**Option A: RoboShire Data Plotter**
1. Open RoboShire GUI
2. Widgets → Data Plotter
3. Add topic: `/sensors/temperature`
4. See real-time graph

**Option B: rqt_plot**
```bash
ros2 run rqt_plot rqt_plot /sensors/temperature/data /sensors/humidity/data
```

**Option C: Custom Python Subscriber**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class WeatherMonitor(Node):
    def __init__(self):
        super().__init__('weather_monitor')
        self.create_subscription(Float32, '/sensors/temperature', self.temp_callback, 10)

    def temp_callback(self, msg):
        self.get_logger().info(f'Temperature: {msg.data:.1f}°C')

rclpy.init()
node = WeatherMonitor()
rclpy.spin(node)
```

### Step 9: Data Logging

**CSV Logging**:
```bash
# Log all sensor data to CSV
ros2 topic echo /sensors/temperature --csv > weather_data.csv
```

**ROS2 Bag Recording**:
```bash
# Record all weather topics
ros2 bag record /sensors/temperature /sensors/humidity /sensors/pressure /sensors/light

# Playback later
ros2 bag play rosbag2_<timestamp>
```

### Step 10: Extend and Customize

**Add features**:
- Email alerts when temperature exceeds threshold
- Cloud upload (AWS IoT, Google Cloud)
- Web dashboard (Flask + Chart.js)
- Battery power with deep sleep
- Outdoor weatherproof enclosure

---

## Project Variants

### Variant 1: Indoor Air Quality Monitor
**Add**: MQ-135 gas sensor, MQ-7 CO sensor
**Use case**: Monitor indoor air quality, VOCs

### Variant 2: Greenhouse Controller
**Add**: Soil moisture sensor, relay for fan
**Use case**: Automated greenhouse climate control

### Variant 3: Solar-Powered Remote Station
**Add**: Solar panel, LiPo battery, LoRa module
**Use case**: Off-grid weather monitoring

---

## Common Issues and Solutions

### Issue 1: No Serial Data

**Symptoms**: Serial Monitor blank or garbage characters

**Solutions**:
- Check baud rate is 115200
- Verify USB cable is data cable (not charge-only)
- Try different USB port
- Check ESP32 driver installed (CP210x or CH340)

### Issue 2: Sensor Not Found

**Symptoms**: `ERROR:101:Sensor not found`

**Solutions**:
- Verify wiring matches diagram exactly
- Check I2C address with scanner:
  ```cpp
  Wire.begin();
  Wire.beginTransmission(0x76);  // BMP280 address
  if (Wire.endTransmission() == 0) {
    Serial.println("BMP280 found!");
  }
  ```
- Try alternate I2C address (0x76 vs 0x77 for BMP280)
- Check 3.3V power supply is stable

### Issue 3: Incorrect Temperature Readings

**Symptoms**: Temperature shows 185°C or -999°C

**Solutions**:
- DHT22 takes 2 seconds to stabilize after power-on
- Add `delay(2000)` in `setup()`
- Check for `isnan()` before publishing:
  ```cpp
  float temp = dht.readTemperature();
  if (!isnan(temp)) {
    Serial.print("TEMP:");
    Serial.println(temp);
  }
  ```

### Issue 4: ROS2 Topics Not Updating

**Symptoms**: Topics exist but no new messages

**Solutions**:
- Check serial bridge is running
- Verify port in bridge matches ESP32 port:
  ```bash
  ls /dev/tty*  # List all ports
  ```
- Check permissions:
  ```bash
  sudo usermod -aG dialout $USER
  # Logout and login
  ```

**More troubleshooting**: See [Testing Guide](testing_guide.md)

---

## Performance Metrics

**Expected Performance**:
| Metric | Value | Notes |
|--------|-------|-------|
| Sensor read rate | 1 Hz | Adjustable in Arduino code |
| Temperature accuracy | ±0.5°C | DHT22 specification |
| Humidity accuracy | ±2% RH | DHT22 specification |
| Pressure accuracy | ±1 hPa | BMP280 specification |
| Serial latency | <10 ms | Typical USB latency |
| ROS2 topic rate | 1 Hz | Matches sensor read rate |
| Power consumption | ~150 mA | ESP32 + sensors (5V) |

**Optimization**:
- Reduce sensor rate for lower power
- Enable ESP32 deep sleep between readings
- Use ESP32-C3 for lower power consumption

---

## Educational Use

### Classroom Activity (2-Hour Lab)

**Learning objectives**:
1. Understand sensor interfacing (digital, I2C, analog)
2. Practice serial communication protocols
3. Learn ROS2 pub/sub architecture
4. Data collection and analysis

**Lab structure**:
- **Hour 1**: Hardware assembly, Arduino programming
- **Hour 2**: ROS2 integration, data visualization

**Assessment**:
- Students plot temperature over 10 minutes
- Calculate average, min, max values
- Write report on sensor accuracy

### Workshop / Hackathon

**Challenge**: Build weather station and predict indoor conditions
**Teams**: 3-4 people
**Duration**: 4 hours
**Extensions**:
- Best visualization dashboard
- Most creative sensor fusion
- Most accurate prediction model

---

## Next Steps

### After Completing Basic Weather Station

**Beginner Path**:
1. Add more sensors (soil moisture, UV index)
2. Create simple web dashboard
3. Set up automated data logging

**Intermediate Path**:
1. Implement sensor calibration
2. Add data filtering (Kalman filter)
3. Build [Differential Drive Robot](../differential_drive_robot/README.md)

**Advanced Path**:
1. Deploy to multiple locations with time sync
2. Implement weather prediction ML model
3. Cloud integration with AWS IoT or Azure

---

## Resources

### Documentation
- [Serial Bridge Implementation](../../communication/serial_bridge_implementation.md)
- [Serial Protocol Specification](../../communication/serial_protocol.md)
- [Sensors Guide](../../SENSORS_GUIDE.md)

### Datasheets
- [DHT22 Datasheet](https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf)
- [BMP280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)

### Code Repository
- Complete Arduino code: `roboshire/examples/weather_station/arduino_code.ino`
- ROS2 serial bridge: See [implementation guide](../../communication/serial_bridge_implementation.md)

---

## Project Photos and Diagrams

### Breadboard Layout
*[Screenshot placeholder: Breadboard wiring diagram]*

### Assembled Station
*[Photo placeholder: Completed weather station]*

### ROS2 Topic Graph
*[Screenshot placeholder: rqt_graph showing topic connections]*

### Data Visualization
*[Screenshot placeholder: Temperature graph over time]*

---

## Community Contributions

**Share your weather station!**
- Post photo to RoboShire community
- Share improvements to Arduino code
- Contribute sensor calibration data
- Write tutorial for additional sensors

**GitHub**: Submit PRs for documentation improvements
**Discord**: Share your project in #hardware-projects channel

---

## FAQ

**Q: Can I use Arduino Uno instead of ESP32?**
A: Yes, but ESP32 is recommended for WiFi capability and more memory. Arduino Uno works fine for local USB connection.

**Q: How accurate is the DHT22?**
A: ±0.5°C for temperature, ±2% for humidity. Good for hobby projects, consider BME280 for better accuracy.

**Q: Can I add more sensors?**
A: Absolutely! ESP32 has many free GPIO pins. See [Sensors Guide](../../SENSORS_GUIDE.md) for integration instructions.

**Q: How do I make it wireless?**
A: Use ESP32's WiFi with micro-ROS or MQTT bridge. Tutorial coming soon.

**Q: Can I power it with batteries?**
A: Yes. Use LiPo battery + TP4056 charging module. Enable deep sleep for longer runtime.

**Q: Is this waterproof for outdoor use?**
A: Not by default. Use IP65 enclosure and consider BME280 which handles humidity better.

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-10-29 | Initial release with complete documentation |

---

## License

This example project is licensed under Apache 2.0. Feel free to modify and share!

**Hardware designs**: Creative Commons BY-SA 4.0
**Software code**: Apache 2.0
**Documentation**: Creative Commons BY 4.0

---

**Ready to build?** Start with the [Bill of Materials](hardware_bom.md) to order your hardware!
