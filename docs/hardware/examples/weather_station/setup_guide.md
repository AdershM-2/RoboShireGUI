# Weather Station Setup Guide

**Prerequisites**: Hardware assembled per [Wiring Diagram](wiring_diagram.md)
**Time**: 30-40 minutes
**Outcome**: Working weather station publishing data to ROS2

---

## Setup Overview

```
1. Install Arduino Libraries (5 min)
2. Upload Arduino Code (5 min)
3. Test Arduino Serial Output (5 min)
4. Create/Build ROS2 Package (10 min)
5. Test End-to-End System (10 min)
```

---

## Step 1: Install Arduino IDE and Libraries

### Install Arduino IDE
```bash
# Download from arduino.cc
wget https://downloads.arduino.cc/arduino-ide/arduino-ide_latest_Linux_64bit.AppImage
chmod +x arduino-ide_*.AppImage
./arduino-ide_*.AppImage
```

### Add ESP32 Board Support
1. File → Preferences
2. Additional Board Manager URLs: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Tools → Board → Boards Manager
4. Search "esp32" → Install "ESP32 by Espressif Systems"

### Install Required Libraries
Tools → Manage Libraries → Install:
- `DHT sensor library` by Adafruit
- `Adafruit BMP280 Library`
- `Adafruit Unified Sensor`

---

## Step 2: Upload Arduino Code

### Get the Code
**Option A**: Use pre-made example
```bash
# Code location
open /mnt/hgfs/ROS2_PROJECT/roboshire/examples/weather_station/arduino_code.ino
```

**Option B**: Generate from Workflow Wizard
1. Launch RoboShire → Workflow Wizard
2. Select "Weather Monitor Station" preset
3. Copy code from Step 4 preview

### Configure Board and Port
1. Tools → Board → ESP32 Dev Module
2. Tools → Port → `/dev/ttyUSB0` (your ESP32 port)
3. Tools → Upload Speed → 115200

### Upload
1. Click Upload button (→)
2. Wait for "Done uploading" message
3. If errors, see troubleshooting below

---

## Step 3: Verify Arduino Output

### Open Serial Monitor
1. Tools → Serial Monitor
2. Set baud rate: **115200**
3. Set line ending: **Newline**

### Expected Output
```
STATUS:READY
TEMP:25.3
HUMIDITY:60.5
PRESSURE:1013.25
LIGHT:450
TEMP:25.4
HUMIDITY:60.6
...
```

**If you see this, Arduino is working perfectly!**

---

## Step 4: Create ROS2 Package

### Using RoboShire Workflow Wizard
```bash
# Launch RoboShire
cd /mnt/hgfs/ROS2_PROJECT
python3 -m roboshire
```

1. Tools → Workflow Wizard
2. Step 1: Select "Without URDF" → "Weather Monitor Station"
3. Step 2: (Already configured by preset)
4. Step 3: Project name = `weather_station`
5. Click through remaining steps
6. Finish

### Add Serial Bridge Node
Copy the serial bridge implementation:
```bash
cd workspace/src/weather_station
mkdir -p weather_station

# Copy from documentation example or create file
# See: docs/hardware/communication/serial_bridge_implementation.md
```

Add to `setup.py`:
```python
entry_points={
    'console_scripts': [
        'serial_bridge = weather_station.serial_bridge:main',
    ],
},
```

---

## Step 5: Build and Launch

### Build Package
```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace
colcon build --packages-select weather_station
source install/setup.bash
```

### Launch Serial Bridge
```bash
ros2 run weather_station serial_bridge
```

**Expected output**:
```
[INFO] [serial_bridge]: Auto-detecting Arduino...
[INFO] [serial_bridge]: Found Arduino: /dev/ttyUSB0 (CP2102 USB to UART)
[INFO] [serial_bridge]: Connected to /dev/ttyUSB0 at 115200 baud
```

---

## Step 6: Verify ROS2 Topics

**Open new terminal**:
```bash
source install/setup.bash

# List topics
ros2 topic list

# Should show:
# /sensors/temperature
# /sensors/humidity
# /sensors/pressure
# /sensors/light
# /serial_bridge/status

# Echo temperature data
ros2 topic echo /sensors/temperature
```

**Success!** Data flows from Arduino → ROS2 topics.

---

## Common Setup Issues

### Arduino Upload Fails
**Error**: `Connecting........_____....`

**Fix**:
- Hold BOOT button on ESP32 during upload
- Check USB cable (must be data cable)
- Install CP210x driver: `sudo apt install brltty-`
- Try different USB port

### Serial Port Not Found
**Error**: `/dev/ttyUSB0: No such file or directory`

**Fix**:
```bash
# List all ports
ls /dev/tty*

# Check permissions
sudo usermod -aG dialout $USER
# Logout and login

# Use correct port in serial bridge
ros2 run weather_station serial_bridge --ros-args -p port:=/dev/ttyACM0
```

### Library Not Found
**Error**: `DHT.h: No such file or directory`

**Fix**:
- Verify library installation: Sketch → Include Library → (see installed)
- Restart Arduino IDE after install
- Manual install: Download .zip from GitHub → Sketch → Include Library → Add .ZIP

---

## Configuration Files

### Arduino Pin Configuration
Edit these in `.ino` file if using different pins:
```cpp
#define DHT_PIN 4           // DHT22 data pin
#define LIGHT_SENSOR_PIN 34 // LDR analog pin
// BMP280 uses I2C (GPIO 21, 22) - fixed
```

### ROS2 Serial Bridge Parameters
Create `config/serial_bridge.yaml`:
```yaml
serial_bridge:
  ros__parameters:
    port: '/dev/ttyUSB0'
    baudrate: 115200
    auto_detect_port: true
    timeout: 1.0
```

Load with launch file:
```python
parameters=[
    os.path.join(get_package_share_directory('weather_station'),
                 'config', 'serial_bridge.yaml')
]
```

---

## Calibration (Optional)

### Temperature Calibration
```cpp
// Add offset if sensor reads incorrectly
float temp = dht.readTemperature();
temp = temp + TEMP_OFFSET;  // e.g., -1.5°C correction
```

### Light Sensor Calibration
Map ADC values to lux:
```cpp
int raw = analogRead(LIGHT_SENSOR_PIN);
float lux = map(raw, 0, 4095, 0, 1000);  // Approximate
```

For accurate lux, use BH1750 digital light sensor instead.

---

## Next Steps

Proceed to [Testing Guide](testing_guide.md) for verification procedures.

---

**Document Version**: 1.0.0
