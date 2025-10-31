# micro-ROS Agent Manager Guide

**RoboShire v2.4.0 Feature**

## Overview

The micro-ROS Agent Manager eliminates the need to manually start the micro-ROS agent in a terminal every time you want to connect to your Arduino or ESP32. It provides a graphical interface for managing the agent connection with auto-reconnect, port detection, and status monitoring.

## Problem Solved

### Before v2.4.0
```bash
# Every single time you want to work with Arduino/ESP32:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# If you unplug/replug USB, you must:
# 1. Stop the agent (Ctrl+C)
# 2. Find new port with: ls /dev/ttyUSB* or /dev/ttyACM*
# 3. Restart agent with new port
# 4. Hope you got it right!
```

### With v2.4.0
```
1. Hardware → micro-ROS Agent (Ctrl+Shift+M)
2. Click "Refresh" to find Arduino/ESP32
3. Click "Start Agent"
4. Status turns green when connected
5. Auto-reconnects if USB is replugged!
```

## Features

- **Auto-Detect Serial Ports**: Automatically finds Arduino, ESP32, and other microcontrollers
- **One-Click Start/Stop**: No terminal commands needed
- **Connection Status Indicator**: Visual feedback (green = connected, red = disconnected)
- **Live Agent Log**: See exactly what the agent is doing
- **Auto-Reconnect**: Automatically reconnects if USB cable is replugged
- **WiFi/UDP Support**: Works with ESP32 over WiFi
- **Save Configurations**: Remember your settings for next time
- **Status Bar Integration**: Agent status always visible in main window

## Opening the Agent Manager

### Method 1: Menu
1. Hardware → micro-ROS Agent

### Method 2: Keyboard Shortcut
- Press **Ctrl+Shift+M**

### Method 3: Status Bar
- Click on the "Agent: ● Disconnected" indicator in the status bar (bottom right)

## Using the Agent Manager

### Serial Connection (USB)

#### 1. Connect your Arduino/ESP32
Plug in your device via USB

#### 2. Open Agent Manager
- Hardware → micro-ROS Agent (Ctrl+Shift+M)

#### 3. Detect Device
- Click **"Refresh"** button
- The Serial Port dropdown will populate with detected devices:
  ```
  /dev/ttyUSB0 - USB-SERIAL CH340
  /dev/ttyACM0 - Arduino Uno
  ```

#### 4. Select Baud Rate
- Default: **115200** (most common)
- Other options: 9600, 230400, 460800, 921600
- **Must match your Arduino sketch!**

#### 5. Start Agent
- Click **"Start Agent"**
- Watch the log for connection messages
- Status should turn **green** when microcontroller connects

#### 6. Verify Connection
Watch for these messages in the log:
```
[INFO] Starting serial agent on /dev/ttyUSB0 @ 115200 baud
[INFO] Client connected
[INFO] session established
```

### WiFi Connection (UDP)

For ESP32 with WiFi micro-ROS:

#### 1. Select Connection Type
- Click the **"WiFi (UDP)"** radio button

#### 2. Set UDP Port
- Default: **8888**
- Must match your ESP32 configuration

#### 3. Start Agent
- Click **"Start Agent"**
- Agent listens for UDP connections from ESP32

#### 4. Verify Connection
Your ESP32 should connect automatically when it's powered on and has WiFi connectivity.

## Auto-Reconnect Feature

### Enabling Auto-Reconnect
- Check the **"Auto-reconnect on disconnect"** checkbox
- Agent will automatically restart if connection is lost

### Use Cases
1. **USB Replug**: Unplug and replug your Arduino - agent automatically reconnects
2. **Microcontroller Reset**: Hit reset button on Arduino - agent stays connected
3. **Power Cycle**: Turn device off and on - agent reconnects when device is back

### How It Works
- Monitors agent process every second
- Detects if agent terminates
- Waits 3 seconds
- Restarts agent with last-used configuration

## Save & Load Configurations

### Saving Configuration
1. Set up your connection (serial port, baud rate, etc.)
2. Click **"Save Configuration"**
3. Configuration saved to: `~/.roboshire/micro_ros_agent_config.json`

### Loading Configuration
- Configuration automatically loads when you open the Agent Manager
- Your last-used settings are restored

### What's Saved
- Connection type (Serial or WiFi)
- Serial port (if applicable)
- Baud rate
- UDP port (for WiFi)
- Auto-reconnect enabled/disabled

## Agent Log Viewer

The log viewer shows real-time output from the micro-ROS agent:

```
[INFO] Starting serial agent on /dev/ttyUSB0 @ 115200 baud
[INFO] Agent started on port 8888
[INFO] Client connected
[INFO] Created client 0x7f1234567890
[INFO] session established
[DATA] /temperature publishing...
[ERROR] Client disconnected
```

### Log Message Types
- **[INFO]**: Informational messages
- **[WARNING]**: Non-critical issues
- **[ERROR]**: Problems that need attention
- **[DATA]**: Data transmission updates

### Auto-Scroll
- Log automatically scrolls to show latest messages
- Scroll up to view history
- Log keeps last 1000 lines

## Status Bar Integration

The main RoboShire window shows agent status in the status bar (bottom right):

```
Agent: ● Connected (ESP32 /dev/ttyUSB0) [⚙️]
       ↑            ↑
   Indicator    Details
```

### Status Colors
- **Green (●)**: Agent running and connected
- **Red (●)**: Agent stopped or disconnected
- **Orange (●)**: Agent starting up (transitional state)

### Clicking Status
- Click the status indicator to open the Agent Manager
- Quick access without using menu

## Supported Devices

### Tested Arduino Boards
- ✅ Arduino Uno (ATmega328P)
- ✅ Arduino Mega 2560
- ✅ Arduino Due
- ✅ Arduino Nano
- ✅ Arduino MKR series

### Tested ESP32 Boards
- ✅ ESP32 DevKit
- ✅ ESP32-WROOM-32
- ✅ ESP32-S2
- ✅ ESP32-S3
- ✅ ESP32-C3

### USB-Serial Chips Detected
- **CH340**: Common on clone boards
- **FTDI**: FT232, FT2232
- **CP210x**: Silicon Labs (common on ESP32)
- **Prolific**: PL2303

## Troubleshooting

### "No devices found" after clicking Refresh

**Possible causes:**
1. Device not plugged in
2. USB cable is charging-only (no data lines)
3. Driver not installed (especially on Windows)
4. Permissions issue (Linux)

**Solutions:**

**Linux - Permission Error:**
```bash
# Add your user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in
```

**Check if device is detected:**
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
# Should show your device
```

**Install drivers:**
```bash
# CH340 driver (if not included in kernel)
sudo apt install ch341-dkms  # Ubuntu/Debian
```

### Agent starts but status stays red

**Possible causes:**
1. Microcontroller not running micro-ROS sketch
2. Baud rate mismatch
3. Microcontroller crashed/reset

**Solutions:**
1. Upload micro-ROS example sketch to Arduino
2. Verify baud rate in Arduino sketch matches agent
3. Press reset button on Arduino after agent starts

### "Created client" but no data

**Possible causes:**
1. Microcontroller code not publishing topics yet
2. Initialization delay

**Solution:**
- Wait 5-10 seconds for microcontroller to initialize
- Check Arduino Serial Monitor for debug messages

### Agent process terminates unexpectedly

**Check logs for:**
```
[ERROR] Unable to open serial port
[ERROR] Permission denied
```

**Solutions:**
1. Check permissions (dialout group on Linux)
2. Device might be used by another program
3. Try different USB port

### "pyserial not installed" warning

**Solution:**
```bash
pip install pyserial
# Or
sudo apt install python3-serial
```

## Example Workflows

### Workflow 1: Arduino Temperature Sensor

1. **Connect Hardware**
   - Connect DHT22 temperature sensor to Arduino
   - Upload micro-ROS + DHT22 sketch
   - Plug in USB

2. **Start Agent**
   - Ctrl+Shift+M (Open Agent Manager)
   - Click "Refresh"
   - Select Arduino port
   - Click "Start Agent"

3. **Test Sensor**
   - Open Hardware Test Panel (Ctrl+T)
   - Click "Auto-Detect Hardware"
   - See temperature reading with live graph

4. **Leave Running**
   - Enable "Auto-reconnect"
   - Close Agent Manager window
   - Agent keeps running in background
   - Status bar shows connection

### Workflow 2: ESP32 Over WiFi

1. **Configure ESP32**
   ```cpp
   // In your ESP32 sketch
   set_microros_wifi_transports("SSID", "password", "192.168.1.100", 8888);
   ```

2. **Start Agent**
   - Ctrl+Shift+M (Open Agent Manager)
   - Select "WiFi (UDP)" radio button
   - Port: 8888
   - Click "Start Agent"

3. **Power On ESP32**
   - ESP32 connects over WiFi
   - Agent log shows "Client connected"

4. **Monitor Topics**
   - Open Topics tab (Ctrl+6)
   - See ESP32 topics appear
   - Use Hardware Test Panel to interact

### Workflow 3: Differential Drive Robot

1. **Hardware Setup**
   - Arduino with L298N motor driver
   - micro-ROS sketch for motor control
   - Wheel encoders (optional)

2. **Start Agent**
   - Ctrl+Shift+M
   - Refresh and select Arduino
   - Start Agent

3. **Test Motors**
   - Ctrl+T (Hardware Test Panel)
   - Auto-detect finds motor topics
   - Use sliders to test motors
   - Emergency stop if needed

4. **Run Autonomous Code**
   - Switch to Node Graph
   - Add navigation nodes
   - Build and Run
   - Robot uses Arduino motors via micro-ROS!

## Integration with RoboShire Features

### With Code Generation
1. Design robot in Node Graph
2. Generate code (Ctrl+G)
3. Build (Ctrl+B)
4. Start agent (Ctrl+Shift+M)
5. Run (F5)
6. Robot code communicates with Arduino automatically!

### With URDF Validator
1. Create robot URDF
2. Validate (Tools → Validate URDF)
3. Export for simulation
4. Connect real hardware via agent
5. Test in both simulation and real hardware

### With MuJoCo Viewer
1. Test robot in MuJoCo simulation
2. Switch to real hardware
3. Start agent
4. Same code works on both!

## Best Practices

1. **Enable Auto-Reconnect**: Always check this box for seamless operation
2. **Save Configuration**: Save your settings so they're ready next time
3. **Check Status Bar**: Glance at status bar to verify connection
4. **Monitor Logs**: Watch for warnings or errors in agent log
5. **Use Correct Baud Rate**: Must match your Arduino sketch exactly
6. **Keep Agent Manager Open**: During debugging to see live logs

## Technical Details

### Port Detection Algorithm
The agent manager detects serial ports by checking USB Vendor IDs (VIDs):
- 0x2341: Official Arduino boards
- 0x1A86: CH340 (clone boards)
- 0x0403: FTDI chips
- 0x10C4: CP210x (ESP32)
- 0x067B: Prolific adapters

### Process Management
- Agent runs as subprocess
- Output captured and displayed in real-time
- Graceful termination with SIGTERM
- Force kill after 5 second timeout

### Configuration File Format
```json
{
  "connection_type": "serial",
  "serial_port": "/dev/ttyUSB0",
  "baud_rate": "115200",
  "udp_port": "8888",
  "auto_reconnect": true
}
```

Stored in: `~/.roboshire/micro_ros_agent_config.json`

## FAQ

**Q: Can I run multiple agents at once?**
A: No, only one agent per serial port. You can run multiple agents for different ports.

**Q: Does this work on Windows?**
A: Yes! Ports will be COM1, COM2, etc. Auto-detection works the same.

**Q: What if my microcontroller uses a non-standard baud rate?**
A: Edit the baud rate dropdown in `micro_ros_agent_widget.py` to add your rate.

**Q: Can I see micro-ROS topics in the Topics tab?**
A: Yes! Once agent is connected, micro-ROS topics appear alongside regular ROS2 topics.

**Q: Does agent need to restart when I reboot my Arduino?**
A: No! With auto-reconnect enabled, the agent reconnects automatically.

**Q: Can I use this with PlatformIO instead of Arduino IDE?**
A: Absolutely! The agent works with any micro-ROS client, regardless of how it's compiled.

## What's Next?

After mastering the micro-ROS Agent Manager:
- **[Hardware Testing Panel](./HARDWARE_TESTING_PANEL.md)** - Test your sensors and actuators
- **[Serial Bridge Implementation](./communication/serial_bridge_implementation.md)** - Deep dive into micro-ROS
- **[Weather Station Tutorial](../tutorials/WEATHER_STATION_TUTORIAL.md)** - Build a complete project

## Changelog

### v2.4.0 (2025-10-29)
- Initial release
- Serial and UDP support
- Auto-reconnect feature
- Configuration save/load
- Status bar integration

---

**Version**: v2.4.0
**Last Updated**: 2025-10-29
**Author**: RoboShire Team
