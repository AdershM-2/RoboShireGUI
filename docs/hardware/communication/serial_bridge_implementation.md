# Serial Bridge Implementation: Connecting Arduino to ROS2

**Difficulty**: Intermediate
**Time**: 45-60 minutes
**Critical**: This is THE missing piece for hardware integration
**What You'll Build**: A PySerial-based ROS2 node that reads Arduino sensor data and publishes to ROS2 topics

---

## Why This Guide Exists

**THE #1 GAP in RoboShire**: The Workflow Wizard generates Arduino code and ROS2 nodes, but doesn't provide the "glue" between them - the **serial bridge**. This guide fills that gap with a complete, production-ready implementation.

### What is a Serial Bridge?

A serial bridge is a ROS2 node that:
1. Opens a serial connection to Arduino/ESP32 via USB
2. Reads sensor data from the serial port
3. Parses messages according to a protocol
4. Publishes data to appropriate ROS2 topics
5. (Optional) Sends commands from ROS2 back to Arduino

**Think of it as a translator** between Arduino's serial language and ROS2's topic language.

---

## Prerequisites

### Knowledge
- Basic Python programming
- Understanding of ROS2 nodes and topics (see [Quick Start](../HARDWARE_QUICK_START.md))
- Familiarity with serial communication (helpful but not required)

### Software
- RoboShire v2.3.0+ installed
- ROS2 Humble or newer
- Python 3.10+
- PySerial library

### Hardware
- Arduino, ESP32, or similar microcontroller
- USB cable
- (For testing) LEDs, sensors, or use example code

### Install PySerial
```bash
pip3 install pyserial
```

---

## Architecture Overview

### The Complete System

```
┌───────────────────────────────────────────────────────────────┐
│                        COMPLETE SYSTEM                        │
└───────────────────────────────────────────────────────────────┘

┌──────────────┐         ┌──────────────┐         ┌─────────────┐
│   ARDUINO    │         │ SERIAL BRIDGE│         │  ROS2 NODES │
│  (Hardware)  │  Serial │  (This Guide)│  Topics │ (Generated) │
│              │────────>│              │────────>│             │
│ - Sensors    │ UART    │ - PySerial   │         │ - Data      │
│ - Actuators  │         │ - Parser     │         │   Processing│
│ - Timing     │         │ - Publisher  │         │ - Control   │
│              │<────────│ - Subscriber │<────────│   Logic     │
└──────────────┘         └──────────────┘         └─────────────┘
      ↑                                                   │
      │                                                   ↓
┌──────────────┐                                   ┌─────────────┐
│   SENSORS    │                                   │  OUTPUT     │
│ DHT22, IMU   │                                   │ Logs, Plots │
│ Ultrasonic   │                                   │ Navigation  │
└──────────────┘                                   └─────────────┘
```

### Data Flow Example (Weather Station)

```
1. Arduino reads DHT22 temperature → 25.3°C
2. Arduino formats message → "TEMP:25.3\n"
3. Arduino sends via Serial.println() → USB cable
4. Serial Bridge receives → line = "TEMP:25.3"
5. Serial Bridge parses → key="TEMP", value=25.3
6. Serial Bridge publishes → Float32 msg to /sensors/temperature
7. ROS2 node receives → processes, logs, visualizes
```

---

## Serial Protocol Specification

Before implementing the bridge, understand the message format.

### Standard Protocol (TEXT-BASED)

**Arduino → ROS2 (Sensor Data)**:
```
FORMAT: KEY:value\n
EXAMPLES:
  TEMP:25.3
  HUMIDITY:60.5
  DISTANCE:145.2
  IMU:0.12,-0.03,9.81,0.5,0.2,-0.1
```

**ROS2 → Arduino (Commands)**:
```
FORMAT: COMMAND:parameters\n
EXAMPLES:
  MOTOR:150,-150
  SERVO:90
  LED:ON
  STOP
```

**Arduino Acknowledgments**:
```
FORMAT: ACK:COMMAND:parameters\n
EXAMPLES:
  ACK:MOTOR:150,-150
  ACK:LED:ON
```

### Protocol Rules

1. **Line-based**: Every message ends with newline (`\n`)
2. **Colon separator**: Key and value separated by `:`
3. **Comma for arrays**: Multiple values separated by commas
4. **Uppercase keys**: Keys in UPPERCASE for clarity
5. **Float precision**: 2-3 decimal places typical

**Why TEXT instead of BINARY?**
- Easy to debug (readable in Serial Monitor)
- No endianness issues
- Simple parsing
- Cross-platform compatibility

**Tradeoff**: Slightly slower than binary, but negligible for typical robot data rates (<100 Hz)

---

## Implementation: Basic Serial Bridge

### Step 1: Create Package for Serial Bridge

```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace/src

# Create package
ros2 pkg create --build-type ament_python serial_bridge_pkg \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

cd serial_bridge_pkg
```

### Step 2: Write Serial Bridge Node

Create file: `serial_bridge_pkg/serial_bridge_node.py`

```python
#!/usr/bin/env python3
"""
Serial Bridge Node - Connect Arduino/ESP32 to ROS2
Reads sensor data from serial port and publishes to ROS2 topics

Author: RoboShire Community
License: Apache 2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Bool
from geometry_msgs.msg import Vector3, Twist
import serial
import serial.tools.list_ports
import threading
import time


class SerialBridge(Node):
    """
    Serial Bridge for Arduino ↔ ROS2 Communication

    Features:
    - Automatic port detection
    - Message parsing (KEY:value protocol)
    - Multiple publishers for different sensors
    - Error handling and reconnection
    - Bidirectional communication
    """

    def __init__(self):
        super().__init__('serial_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('auto_detect_port', True)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.auto_detect = self.get_parameter('auto_detect_port').value

        # Publishers for sensor data
        self.temp_pub = self.create_publisher(Float32, '/sensors/temperature', 10)
        self.humidity_pub = self.create_publisher(Float32, '/sensors/humidity', 10)
        self.pressure_pub = self.create_publisher(Float32, '/sensors/pressure', 10)
        self.light_pub = self.create_publisher(Int32, '/sensors/light', 10)
        self.distance_pub = self.create_publisher(Float32, '/sensors/distance', 10)
        self.imu_accel_pub = self.create_publisher(Vector3, '/sensors/imu/accel', 10)
        self.imu_gyro_pub = self.create_publisher(Vector3, '/sensors/imu/gyro', 10)
        self.status_pub = self.create_publisher(String, '/serial_bridge/status', 10)

        # Subscriber for commands (Arduino ← ROS2)
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Serial connection
        self.serial = None
        self.running = True

        # Statistics
        self.messages_received = 0
        self.parse_errors = 0

        # Connect to serial port
        self._connect_serial()

        # Start reading thread
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()

        # Status timer
        self.status_timer = self.create_timer(5.0, self._publish_status)

        self.get_logger().info('Serial Bridge initialized')

    def _connect_serial(self):
        """Connect to serial port with auto-detection"""
        if self.auto_detect:
            self.port = self._auto_detect_port()
            if self.port is None:
                self.get_logger().error('No Arduino detected. Retrying...')
                return False

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=1.0
            )
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f'Connected to {self.port} at {self.baudrate} baud')
            return True

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.port}: {e}')
            return False

    def _auto_detect_port(self):
        """Auto-detect Arduino/ESP32 port"""
        self.get_logger().info('Auto-detecting Arduino...')

        # Known vendor IDs for Arduino/ESP32
        arduino_vids = [0x2341, 0x1A86, 0x0403, 0x10C4]  # Arduino, CH340, FTDI, CP210x

        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid in arduino_vids:
                self.get_logger().info(f'Found Arduino: {port.device} ({port.description})')
                return port.device

        # Fallback: Try common ports
        common_ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1']
        for port in common_ports:
            try:
                test_serial = serial.Serial(port, self.baudrate, timeout=0.5)
                test_serial.close()
                self.get_logger().info(f'Found port: {port}')
                return port
            except:
                continue

        return None

    def _read_serial_loop(self):
        """Main loop for reading serial data"""
        while self.running:
            if self.serial is None or not self.serial.is_open:
                # Try to reconnect
                self.get_logger().warn('Serial disconnected. Attempting reconnection...')
                time.sleep(self.reconnect_interval)
                self._connect_serial()
                continue

            try:
                # Read line from serial
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self._parse_and_publish(line)
                        self.messages_received += 1

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.serial = None

            except UnicodeDecodeError as e:
                self.get_logger().warn(f'Decode error: {e}')

            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')

    def _parse_and_publish(self, line: str):
        """
        Parse Arduino message and publish to appropriate topic

        Format: KEY:value
        Examples:
          TEMP:25.3
          IMU:0.12,-0.03,9.81,0.5,0.2,-0.1
        """
        if ':' not in line:
            self.get_logger().debug(f'Ignoring non-protocol line: {line}')
            return

        try:
            key, value = line.split(':', 1)
            key = key.strip().upper()
            value = value.strip()

            # Temperature
            if key == 'TEMP':
                msg = Float32()
                msg.data = float(value)
                self.temp_pub.publish(msg)
                self.get_logger().debug(f'Published temperature: {msg.data}°C')

            # Humidity
            elif key == 'HUMIDITY':
                msg = Float32()
                msg.data = float(value)
                self.humidity_pub.publish(msg)
                self.get_logger().debug(f'Published humidity: {msg.data}%')

            # Pressure
            elif key == 'PRESSURE':
                msg = Float32()
                msg.data = float(value)
                self.pressure_pub.publish(msg)
                self.get_logger().debug(f'Published pressure: {msg.data} hPa')

            # Light sensor
            elif key == 'LIGHT':
                msg = Int32()
                msg.data = int(float(value))
                self.light_pub.publish(msg)
                self.get_logger().debug(f'Published light: {msg.data}')

            # Distance (ultrasonic)
            elif key == 'DISTANCE':
                msg = Float32()
                msg.data = float(value)
                self.distance_pub.publish(msg)
                self.get_logger().debug(f'Published distance: {msg.data} cm')

            # IMU (accelerometer + gyroscope)
            elif key == 'IMU':
                # Format: ax,ay,az,gx,gy,gz
                values = [float(v) for v in value.split(',')]
                if len(values) >= 6:
                    # Accelerometer
                    accel_msg = Vector3()
                    accel_msg.x = values[0]
                    accel_msg.y = values[1]
                    accel_msg.z = values[2]
                    self.imu_accel_pub.publish(accel_msg)

                    # Gyroscope
                    gyro_msg = Vector3()
                    gyro_msg.x = values[3]
                    gyro_msg.y = values[4]
                    gyro_msg.z = values[5]
                    self.imu_gyro_pub.publish(gyro_msg)

                    self.get_logger().debug('Published IMU data')

            # Acknowledgment (Arduino confirming command)
            elif key == 'ACK':
                self.get_logger().info(f'Arduino ACK: {value}')

            # Unknown key
            else:
                self.get_logger().warn(f'Unknown message key: {key}')

        except ValueError as e:
            self.get_logger().warn(f'Parse error for "{line}": {e}')
            self.parse_errors += 1

        except Exception as e:
            self.get_logger().error(f'Unexpected parse error: {e}')
            self.parse_errors += 1

    def cmd_vel_callback(self, msg: Twist):
        """
        Send motor commands to Arduino

        Converts Twist message to motor speeds and sends via serial
        """
        if self.serial is None or not self.serial.is_open:
            return

        try:
            # Convert twist to differential drive motor speeds
            # This is a simplified conversion - adjust for your robot
            linear = msg.linear.x  # Forward/backward
            angular = msg.angular.z  # Rotation

            # Differential drive: left and right wheel speeds
            left_speed = int((linear - angular * 0.5) * 255)  # Scale to PWM
            right_speed = int((linear + angular * 0.5) * 255)

            # Constrain to valid PWM range
            left_speed = max(-255, min(255, left_speed))
            right_speed = max(-255, min(255, right_speed))

            # Send to Arduino
            command = f'MOTOR:{left_speed},{right_speed}\n'
            self.serial.write(command.encode('utf-8'))

            self.get_logger().debug(f'Sent motor command: L={left_speed}, R={right_speed}')

        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

    def _publish_status(self):
        """Publish bridge status"""
        msg = String()
        if self.serial and self.serial.is_open:
            msg.data = f'Connected to {self.port} | RX: {self.messages_received} | Errors: {self.parse_errors}'
        else:
            msg.data = 'Disconnected - Attempting reconnection'

        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Serial Bridge...')
        self.running = False

        if self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)

        if self.serial and self.serial.is_open:
            self.serial.close()

        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Update setup.py

Edit `setup.py` to add the entry point:

```python
entry_points={
    'console_scripts': [
        'serial_bridge = serial_bridge_pkg.serial_bridge_node:main',
    ],
},
```

### Step 4: Build the Package

```bash
cd /mnt/hgfs/ROS2_PROJECT/workspace
colcon build --packages-select serial_bridge_pkg
source install/setup.bash
```

---

## Usage: Running the Serial Bridge

### Basic Usage

```bash
# Start serial bridge with defaults
ros2 run serial_bridge_pkg serial_bridge

# With specific port
ros2 run serial_bridge_pkg serial_bridge --ros-args -p port:=/dev/ttyUSB0

# With custom baud rate
ros2 run serial_bridge_pkg serial_bridge --ros-args -p baudrate:=9600

# Disable auto-detection
ros2 run serial_bridge_pkg serial_bridge --ros-args -p auto_detect_port:=false
```

### Launch File

Create `launch/serial_bridge_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino'
        ),

        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baud rate'
        ),

        Node(
            package='serial_bridge_pkg',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'auto_detect_port': True,
                'timeout': 1.0,
                'reconnect_interval': 5.0,
            }]
        ),
    ])
```

Launch:
```bash
ros2 launch serial_bridge_pkg serial_bridge_launch.py port:=/dev/ttyUSB0
```

---

## Testing the Serial Bridge

### Test 1: Echo Arduino Messages

**Terminal 1** - Start bridge:
```bash
ros2 run serial_bridge_pkg serial_bridge
```

**Terminal 2** - Monitor topics:
```bash
# List available topics
ros2 topic list

# Echo temperature data
ros2 topic echo /sensors/temperature

# Echo IMU data
ros2 topic echo /sensors/imu/accel
```

**Expected**: See sensor data updating in real-time

### Test 2: Send Commands to Arduino

**Terminal 1** - Bridge running

**Terminal 2** - Publish motor commands:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

**Expected**: Arduino receives `MOTOR:127,127` and motors respond

### Test 3: Check Bridge Status

```bash
ros2 topic echo /serial_bridge/status
```

**Expected**:
```
data: 'Connected to /dev/ttyACM0 | RX: 245 | Errors: 0'
```

---

## Arduino Example Code

### Weather Station (Sensor Reading)

```cpp
/*
 * Weather Station - Arduino Code
 * Compatible with Serial Bridge
 */

#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Pins
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define LIGHT_PIN 34

// Sensors
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP280 bmp;

// Timing
unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 1000;  // 1Hz

void setup() {
  Serial.begin(115200);
  dht.begin();
  bmp.begin(0x76);
  pinMode(LIGHT_PIN, INPUT);

  Serial.println("ACK:READY");
}

void loop() {
  if (millis() - lastRead >= READ_INTERVAL) {
    lastRead = millis();

    // Read sensors
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    float pressure = bmp.readPressure() / 100.0F;
    int light = analogRead(LIGHT_PIN);

    // Send data (KEY:value format)
    if (!isnan(temp)) {
      Serial.print("TEMP:");
      Serial.println(temp, 2);
    }

    if (!isnan(humidity)) {
      Serial.print("HUMIDITY:");
      Serial.println(humidity, 2);
    }

    Serial.print("PRESSURE:");
    Serial.println(pressure, 2);

    Serial.print("LIGHT:");
    Serial.println(light);
  }
}
```

### Differential Drive Robot (Bidirectional)

```cpp
/*
 * Differential Drive Robot - Arduino Code
 * Receives motor commands, sends sensor data
 */

#include <Wire.h>
#include <MPU6050.h>

// Motor pins (L298N)
#define MOTOR_L_IN1 7
#define MOTOR_L_IN2 8
#define MOTOR_L_PWM 9
#define MOTOR_R_IN1 4
#define MOTOR_R_IN2 5
#define MOTOR_R_PWM 6

// Ultrasonic
#define TRIG_PIN 12
#define ECHO_PIN 13

MPU6050 mpu;

unsigned long lastIMU = 0;
unsigned long lastUltrasonic = 0;

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // IMU
  Wire.begin();
  mpu.initialize();

  Serial.println("ACK:READY");
}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }

  // Publish IMU data at 20Hz
  if (millis() - lastIMU >= 50) {
    lastIMU = millis();
    publishIMU();
  }

  // Publish ultrasonic at 10Hz
  if (millis() - lastUltrasonic >= 100) {
    lastUltrasonic = millis();
    publishUltrasonic();
  }
}

void processCommand(String cmd) {
  if (cmd.startsWith("MOTOR:")) {
    // Parse: MOTOR:left_speed,right_speed
    int commaIndex = cmd.indexOf(',', 6);
    int leftSpeed = cmd.substring(6, commaIndex).toInt();
    int rightSpeed = cmd.substring(commaIndex + 1).toInt();

    setMotors(leftSpeed, rightSpeed);
    Serial.print("ACK:MOTOR:");
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.println(rightSpeed);
  }
  else if (cmd == "STOP") {
    setMotors(0, 0);
    Serial.println("ACK:STOP");
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    analogWrite(MOTOR_L_PWM, leftSpeed);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    analogWrite(MOTOR_L_PWM, -leftSpeed);
  }

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_R_PWM, rightSpeed);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    analogWrite(MOTOR_R_PWM, -rightSpeed);
  }
}

void publishIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to m/s² and rad/s
  float accel_x = ax / 16384.0 * 9.81;
  float accel_y = ay / 16384.0 * 9.81;
  float accel_z = az / 16384.0 * 9.81;
  float gyro_x = gx / 131.0 * 0.0174533;  // deg/s to rad/s
  float gyro_y = gy / 131.0 * 0.0174533;
  float gyro_z = gz / 131.0 * 0.0174533;

  Serial.print("IMU:");
  Serial.print(accel_x, 3); Serial.print(",");
  Serial.print(accel_y, 3); Serial.print(",");
  Serial.print(accel_z, 3); Serial.print(",");
  Serial.print(gyro_x, 3); Serial.print(",");
  Serial.print(gyro_y, 3); Serial.print(",");
  Serial.println(gyro_z, 3);
}

void publishUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  float distance = duration * 0.034 / 2.0;  // cm

  if (distance > 0 && distance < 400) {
    Serial.print("DISTANCE:");
    Serial.println(distance, 1);
  }
}
```

---

## Advanced Features

### Feature 1: Parameter Configuration

Create `config/serial_bridge.yaml`:

```yaml
serial_bridge:
  ros__parameters:
    port: '/dev/ttyACM0'
    baudrate: 115200
    timeout: 1.0
    reconnect_interval: 5.0
    auto_detect_port: true

    # Topic names (customize for your robot)
    temperature_topic: '/sensors/temperature'
    humidity_topic: '/sensors/humidity'
    distance_topic: '/sensors/distance'

    # Filtering
    enable_filtering: true
    temp_filter_size: 5
```

Load with launch file:
```python
parameters=[os.path.join(
    get_package_share_directory('serial_bridge_pkg'),
    'config', 'serial_bridge.yaml'
)]
```

### Feature 2: Message Filtering

Add to serial bridge node:

```python
from collections import deque

class SerialBridge(Node):
    def __init__(self):
        # ... existing code ...

        # Moving average filter
        self.temp_buffer = deque(maxlen=5)
        self.enable_filtering = True

    def _parse_and_publish(self, line: str):
        # ... existing parsing ...

        if key == 'TEMP':
            raw_value = float(value)

            if self.enable_filtering:
                self.temp_buffer.append(raw_value)
                filtered_value = sum(self.temp_buffer) / len(self.temp_buffer)
                msg.data = filtered_value
            else:
                msg.data = raw_value

            self.temp_pub.publish(msg)
```

### Feature 3: Data Logging

```python
import csv
from datetime import datetime

class SerialBridge(Node):
    def __init__(self):
        # ... existing code ...

        # CSV logger
        self.log_file = open('sensor_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Timestamp', 'Key', 'Value'])

    def _parse_and_publish(self, line: str):
        # ... existing parsing ...

        # Log to CSV
        timestamp = datetime.now().isoformat()
        self.csv_writer.writerow([timestamp, key, value])
        self.log_file.flush()
```

### Feature 4: Binary Protocol (Advanced)

For high-frequency data (>100 Hz), use binary protocol:

**Arduino**:
```cpp
struct SensorData {
  float temp;
  float humidity;
  float pressure;
  uint16_t light;
} __attribute__((packed));

void publishBinary() {
  SensorData data;
  data.temp = dht.readTemperature();
  data.humidity = dht.readHumidity();
  data.pressure = bmp.readPressure() / 100.0F;
  data.light = analogRead(LIGHT_PIN);

  Serial.write((uint8_t*)&data, sizeof(data));
}
```

**ROS2**:
```python
import struct

def _read_binary(self):
    if self.serial.in_waiting >= 14:  # sizeof(SensorData)
        data = self.serial.read(14)
        temp, humidity, pressure, light = struct.unpack('<fffH', data)
        # Publish...
```

---

## Troubleshooting

### Issue 1: Port Not Found

**Error**: `FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyACM0'`

**Solutions**:
```bash
# List all serial ports
ls /dev/tty*

# Check USB devices
lsusb

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Add user to dialout group (permanent fix)
sudo usermod -aG dialout $USER
# Logout and login again
```

### Issue 2: Permission Denied

**Error**: `serial.serialutil.SerialException: [Errno 13] Permission denied: '/dev/ttyACM0'`

**Solution**:
```bash
# Quick fix (temporary)
sudo chmod 666 /dev/ttyACM0

# Permanent fix
sudo usermod -aG dialout $USER
# Logout and login

# Verify group membership
groups $USER
```

### Issue 3: No Data Received

**Symptoms**: Bridge connects but no topics update

**Debug steps**:
1. Test Arduino Serial Monitor first
   ```
   Arduino IDE → Tools → Serial Monitor → 115200 baud
   Should see: TEMP:25.3, HUMIDITY:60.2, etc.
   ```

2. Check ROS2 topics exist
   ```bash
   ros2 topic list
   ```

3. Monitor serial bridge status
   ```bash
   ros2 topic echo /serial_bridge/status
   ```

4. Enable debug logging
   ```bash
   ros2 run serial_bridge_pkg serial_bridge --ros-args --log-level debug
   ```

### Issue 4: Parse Errors

**Error**: Lots of "Parse error" messages in bridge

**Common causes**:
- Incorrect message format from Arduino
- Missing newline characters
- Extra debug prints from Arduino
- Baud rate mismatch

**Solutions**:
```cpp
// Arduino: Ensure consistent format
Serial.println("TEMP:25.3");  // ✅ Good
Serial.print("Temperature: 25.3°C");  // ❌ Bad (no KEY:value)

// Arduino: Remove debug prints or prefix them
Serial.println("DEBUG: Starting...");  // ❌ Causes parse errors
// DEBUG prints won't match KEY:value format
```

### Issue 5: Arduino Keeps Resetting

**Symptom**: Arduino restarts every time serial bridge connects

**Cause**: DTR signal resets Arduino on connection

**Solutions**:

**Option A**: Add delay in Arduino:
```cpp
void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial to stabilize
  Serial.println("ACK:READY");
}
```

**Option B**: Disable DTR reset (hardware):
- Cut the RESET-EN trace on Arduino
- Add 10µF capacitor between RESET and GND

**Option C**: Use ESP32 (doesn't reset on DTR)

---

## Performance Optimization

### Tip 1: Adjust Baud Rate

Higher baud rate = more data throughput
```
9600 baud   = ~960 bytes/s  (basic sensors)
115200 baud = ~11.5 KB/s    (recommended, multiple sensors)
921600 baud = ~92 KB/s      (high-frequency IMU, camera metadata)
```

**Arduino**:
```cpp
Serial.begin(921600);  // Fast!
```

**ROS2**:
```bash
ros2 run serial_bridge_pkg serial_bridge --ros-args -p baudrate:=921600
```

### Tip 2: Reduce Publish Frequency

Don't publish faster than needed:
```cpp
// Temperature changes slowly - 1Hz is fine
unsigned long TEMP_INTERVAL = 1000;  // 1Hz

// IMU for control loop - 50Hz typical
unsigned long IMU_INTERVAL = 20;  // 50Hz

// Ultrasonic for obstacle avoidance - 10Hz sufficient
unsigned long ULTRASONIC_INTERVAL = 100;  // 10Hz
```

### Tip 3: Batch Similar Data

```cpp
// Instead of separate messages:
Serial.println("TEMP:25.3");
Serial.println("HUMIDITY:60.2");
Serial.println("PRESSURE:1013.2");

// Use single message:
Serial.print("ENV:");
Serial.print(temp, 1); Serial.print(",");
Serial.print(humidity, 1); Serial.print(",");
Serial.println(pressure, 1);

// ROS2 bridge parses: ENV:25.3,60.2,1013.2
```

---

## Integration with RoboShire Workflow

### Step-by-Step Integration

1. **Create Project in Workflow Wizard**
   - Follow [Hardware Quick Start](../HARDWARE_QUICK_START.md)
   - Generate Arduino code and ROS2 package

2. **Copy Serial Bridge into Project**
   ```bash
   cp serial_bridge_pkg/serial_bridge_node.py \
      your_project_pkg/your_project_pkg/serial_bridge.py
   ```

3. **Add to setup.py**
   ```python
   entry_points={
       'console_scripts': [
           'serial_bridge = your_project_pkg.serial_bridge:main',
           # ... other nodes ...
       ],
   },
   ```

4. **Update Launch File**
   ```python
   Node(
       package='your_project_pkg',
       executable='serial_bridge',
       name='serial_bridge',
       parameters=[{'port': '/dev/ttyACM0', 'baudrate': 115200}]
   ),
   ```

5. **Build and Test**
   ```bash
   colcon build --packages-select your_project_pkg
   source install/setup.bash
   ros2 launch your_project_pkg your_project_launch.py
   ```

---

## Next Steps

### You Now Have Complete Hardware Integration!

Your serial bridge connects Arduino sensors to ROS2. What's next?

1. **Add More Sensors** - Extend `_parse_and_publish()` for new sensors
2. **Implement Control** - Add subscribers for actuators (motors, servos)
3. **Sensor Fusion** - Combine IMU + wheel encoders for better odometry
4. **Navigation** - Use sensor data with Nav2 stack
5. **Data Logging** - Record sensor data for analysis
6. **Visualization** - Use RViz2 or RoboShire's Data Plotter

### Related Guides

- [Serial Protocol Specification](serial_protocol.md) - Complete protocol details
- [Sensors Guide](../SENSORS_GUIDE.md) - Adding specific sensors
- [Arduino Code Workflow](../ARDUINO_CODE_WORKFLOW.md) - Code generation process
- [Weather Station Example](../examples/weather_station/README.md) - Complete project
- [Differential Drive Example](../examples/differential_drive_robot/README.md) - Mobile robot

---

## Summary

You've learned:
- ✅ Why serial bridge is THE critical component for hardware integration
- ✅ How to implement a production-ready serial bridge with PySerial
- ✅ The KEY:value text protocol for Arduino ↔ ROS2 communication
- ✅ How to auto-detect ports, handle errors, and reconnect
- ✅ Bidirectional communication (sensors → ROS2, ROS2 → actuators)
- ✅ Testing procedures and troubleshooting techniques
- ✅ Integration with RoboShire Workflow Wizard projects

**The serial bridge is no longer a gap - it's your superpower!**

---

**Document Info**
- **Version**: 1.0.0
- **Last Updated**: 2025-10-29
- **Tested On**: RoboShire v2.3.0, ROS2 Humble, Python 3.10, PySerial 3.5
- **Code Repository**: [Link to serial_bridge_pkg if published]
- **Feedback**: Report issues or improvements
