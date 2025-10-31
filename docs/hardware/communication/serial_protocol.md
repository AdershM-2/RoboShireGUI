# Serial Communication Protocol Specification

**Version**: 1.0.0
**Last Updated**: 2025-10-29
**Status**: Stable

---

## Overview

This document defines the standard serial communication protocol used between Arduino/ESP32 microcontrollers and ROS2 systems in RoboShire projects.

### Protocol Characteristics

- **Type**: Text-based, line-delimited
- **Encoding**: UTF-8 / ASCII
- **Baud Rate**: 115200 (default), configurable up to 921600
- **Direction**: Bidirectional (Arduino ↔ ROS2)
- **Framing**: Newline (`\n`) delimited messages
- **Format**: `KEY:value\n`

### Design Goals

1. **Human-readable**: Easy to debug with serial monitor
2. **Simple parsing**: No complex binary protocols
3. **Extensible**: Easy to add new message types
4. **Robust**: Handles noise and malformed messages gracefully
5. **Cross-platform**: Works on all Arduino/ESP32 boards

---

## Message Format

### Basic Structure

```
KEY:value\n
```

**Components**:
- `KEY`: Message identifier (UPPERCASE)
- `:`: Separator (colon)
- `value`: Data payload (numbers, strings, comma-separated arrays)
- `\n`: Line terminator (newline)

### Examples

```
TEMP:25.3
HUMIDITY:60.5
DISTANCE:145.2
IMU:0.12,-0.03,9.81,0.5,0.2,-0.1
ACK:MOTOR:150,-150
STATUS:READY
```

---

## Sensor Data Messages (Arduino → ROS2)

### Temperature

**Format**: `TEMP:float`
**Units**: Degrees Celsius
**Precision**: 1-2 decimal places
**Example**: `TEMP:25.3`

**Arduino Implementation**:
```cpp
float temp = dht.readTemperature();
Serial.print("TEMP:");
Serial.println(temp, 1);
```

**ROS2 Topic**: `/sensors/temperature`
**Message Type**: `std_msgs/Float32`

---

### Humidity

**Format**: `HUMIDITY:float`
**Units**: Percentage (0-100)
**Precision**: 1 decimal place
**Example**: `HUMIDITY:60.5`

**Arduino Implementation**:
```cpp
float humidity = dht.readHumidity();
Serial.print("HUMIDITY:");
Serial.println(humidity, 1);
```

**ROS2 Topic**: `/sensors/humidity`
**Message Type**: `std_msgs/Float32`

---

### Pressure

**Format**: `PRESSURE:float`
**Units**: hectoPascals (hPa)
**Precision**: 1-2 decimal places
**Example**: `PRESSURE:1013.25`

**Arduino Implementation**:
```cpp
float pressure = bmp.readPressure() / 100.0F;  // Pa to hPa
Serial.print("PRESSURE:");
Serial.println(pressure, 2);
```

**ROS2 Topic**: `/sensors/pressure`
**Message Type**: `std_msgs/Float32`

---

### Light Level

**Format**: `LIGHT:integer`
**Units**: ADC value (0-1023 for 10-bit, 0-4095 for 12-bit)
**Example**: `LIGHT:450`

**Arduino Implementation**:
```cpp
int light = analogRead(LIGHT_SENSOR_PIN);
Serial.print("LIGHT:");
Serial.println(light);
```

**ROS2 Topic**: `/sensors/light`
**Message Type**: `std_msgs/Int32`

---

### Distance (Ultrasonic)

**Format**: `DISTANCE:float`
**Units**: Centimeters
**Precision**: 1 decimal place
**Range**: 2-400 cm (typical for HC-SR04)
**Example**: `DISTANCE:145.2`

**Arduino Implementation**:
```cpp
long duration = pulseIn(ECHO_PIN, HIGH, 30000);
float distance = duration * 0.034 / 2.0;

if (distance > 0 && distance < 400) {
  Serial.print("DISTANCE:");
  Serial.println(distance, 1);
}
```

**ROS2 Topic**: `/sensors/distance`
**Message Type**: `std_msgs/Float32`

---

### IMU (Inertial Measurement Unit)

**Format**: `IMU:ax,ay,az,gx,gy,gz`
**Units**:
- Acceleration: m/s²
- Gyroscope: rad/s
**Precision**: 3 decimal places
**Example**: `IMU:0.120,-0.030,9.810,0.050,0.020,-0.100`

**Arduino Implementation** (MPU6050):
```cpp
int16_t ax, ay, az, gx, gy, gz;
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

// Convert to SI units
float accel_x = ax / 16384.0 * 9.81;  // m/s²
float accel_y = ay / 16384.0 * 9.81;
float accel_z = az / 16384.0 * 9.81;
float gyro_x = gx / 131.0 * 0.0174533;  // rad/s
float gyro_y = gy / 131.0 * 0.0174533;
float gyro_z = gz / 131.0 * 0.0174533;

Serial.print("IMU:");
Serial.print(accel_x, 3); Serial.print(",");
Serial.print(accel_y, 3); Serial.print(",");
Serial.print(accel_z, 3); Serial.print(",");
Serial.print(gyro_x, 3); Serial.print(",");
Serial.print(gyro_y, 3); Serial.print(",");
Serial.println(gyro_z, 3);
```

**ROS2 Topics**:
- `/sensors/imu/accel` (Vector3): ax, ay, az
- `/sensors/imu/gyro` (Vector3): gx, gy, gz

---

### GPS

**Format**: `GPS:lat,lon,alt,speed`
**Units**:
- Latitude: Decimal degrees
- Longitude: Decimal degrees
- Altitude: Meters
- Speed: m/s
**Example**: `GPS:37.7749,-122.4194,15.2,1.5`

**Arduino Implementation**:
```cpp
if (gps.location.isValid()) {
  Serial.print("GPS:");
  Serial.print(gps.location.lat(), 6); Serial.print(",");
  Serial.print(gps.location.lng(), 6); Serial.print(",");
  Serial.print(gps.altitude.meters(), 1); Serial.print(",");
  Serial.println(gps.speed.mps(), 2);
}
```

**ROS2 Topic**: `/sensors/gps`
**Message Type**: `sensor_msgs/NavSatFix`

---

### Encoder (Wheel Odometry)

**Format**: `ENCODER:left_ticks,right_ticks`
**Units**: Encoder ticks (integer)
**Example**: `ENCODER:12450,12380`

**Arduino Implementation**:
```cpp
volatile long leftTicks = 0;
volatile long rightTicks = 0;

void leftEncoderISR() { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

void publishEncoders() {
  Serial.print("ENCODER:");
  Serial.print(leftTicks); Serial.print(",");
  Serial.println(rightTicks);
}
```

**ROS2 Topic**: `/sensors/encoders`
**Message Type**: Custom or `std_msgs/Int32MultiArray`

---

### Battery Voltage

**Format**: `BATTERY:voltage`
**Units**: Volts
**Precision**: 2 decimal places
**Example**: `BATTERY:11.85`

**Arduino Implementation**:
```cpp
float voltage = analogRead(BATTERY_PIN) * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
Serial.print("BATTERY:");
Serial.println(voltage, 2);
```

**ROS2 Topic**: `/diagnostics/battery`
**Message Type**: `std_msgs/Float32`

---

## Command Messages (ROS2 → Arduino)

### Motor Control

**Format**: `MOTOR:left_speed,right_speed`
**Units**: PWM value (-255 to 255)
**Negative**: Reverse direction
**Example**: `MOTOR:150,-150` (turn in place)

**Arduino Implementation**:
```cpp
void processCommand(String cmd) {
  if (cmd.startsWith("MOTOR:")) {
    int commaIndex = cmd.indexOf(',', 6);
    int leftSpeed = cmd.substring(6, commaIndex).toInt();
    int rightSpeed = cmd.substring(commaIndex + 1).toInt();

    setMotors(leftSpeed, rightSpeed);
    Serial.print("ACK:MOTOR:");
    Serial.print(leftSpeed); Serial.print(",");
    Serial.println(rightSpeed);
  }
}
```

**ROS2 Source**: `/cmd_vel` (geometry_msgs/Twist)
**Conversion**: Bridge converts Twist to motor speeds

---

### Servo Control

**Format**: `SERVO:servo_id,angle`
**Units**: Degrees (0-180)
**Example**: `SERVO:1,90`

**Arduino Implementation**:
```cpp
if (cmd.startsWith("SERVO:")) {
  int commaIndex = cmd.indexOf(',', 6);
  int servoId = cmd.substring(6, commaIndex).toInt();
  int angle = cmd.substring(commaIndex + 1).toInt();

  servos[servoId].write(angle);
  Serial.print("ACK:SERVO:");
  Serial.print(servoId); Serial.print(",");
  Serial.println(angle);
}
```

---

### LED Control

**Format**: `LED:state`
**Values**: `ON`, `OFF`, or RGB value `R,G,B`
**Examples**:
- `LED:ON`
- `LED:OFF`
- `LED:255,0,0` (red)

**Arduino Implementation**:
```cpp
if (cmd.startsWith("LED:")) {
  String state = cmd.substring(4);
  if (state == "ON") {
    digitalWrite(LED_PIN, HIGH);
  } else if (state == "OFF") {
    digitalWrite(LED_PIN, LOW);
  } else {
    // Parse RGB
    int r = state.substring(0, state.indexOf(',')).toInt();
    // ... set RGB LED
  }
  Serial.println("ACK:LED:" + state);
}
```

---

### Emergency Stop

**Format**: `STOP`
**No parameters**
**Example**: `STOP`

**Arduino Implementation**:
```cpp
if (cmd == "STOP") {
  setMotors(0, 0);
  stopAllActuators();
  Serial.println("ACK:STOP");
}
```

---

### Configuration

**Format**: `CONFIG:parameter,value`
**Examples**:
- `CONFIG:PID_KP,1.5`
- `CONFIG:MAX_SPEED,200`

**Arduino Implementation**:
```cpp
if (cmd.startsWith("CONFIG:")) {
  int commaIndex = cmd.indexOf(',', 7);
  String param = cmd.substring(7, commaIndex);
  float value = cmd.substring(commaIndex + 1).toFloat();

  if (param == "PID_KP") {
    pidKp = value;
  } else if (param == "MAX_SPEED") {
    maxSpeed = (int)value;
  }

  Serial.print("ACK:CONFIG:");
  Serial.print(param); Serial.print(",");
  Serial.println(value, 3);
}
```

---

## Status and System Messages

### Acknowledgment

**Format**: `ACK:command:parameters`
**Purpose**: Confirm command received and executed
**Examples**:
- `ACK:MOTOR:150,-150`
- `ACK:LED:ON`
- `ACK:STOP`

**Usage**: Arduino sends ACK after executing command

---

### Ready Signal

**Format**: `STATUS:READY`
**Purpose**: Arduino finished initialization
**Example**: `STATUS:READY`

**Arduino Implementation**:
```cpp
void setup() {
  Serial.begin(115200);
  // ... initialize sensors ...
  delay(100);
  Serial.println("STATUS:READY");
}
```

---

### Error Messages

**Format**: `ERROR:code:description`
**Examples**:
- `ERROR:101:Sensor not found`
- `ERROR:202:Motor overcurrent`
- `ERROR:303:Battery low`

**Arduino Implementation**:
```cpp
if (!dht.begin()) {
  Serial.println("ERROR:101:DHT22 not found");
}
```

---

### Debug Messages

**Format**: `DEBUG:message`
**Purpose**: Debugging information
**Example**: `DEBUG:IMU calibration complete`

**Note**: Serial bridge should ignore or log DEBUG messages separately

---

## Protocol Examples

### Weather Station

**Arduino sends** (every 1 second):
```
TEMP:25.3
HUMIDITY:60.5
PRESSURE:1013.25
LIGHT:450
```

**No commands needed** (read-only sensors)

---

### Differential Drive Robot

**Arduino sends** (continuous):
```
IMU:0.12,-0.03,9.81,0.05,0.02,-0.10
DISTANCE:145.2
ENCODER:12450,12380
BATTERY:11.85
```

**ROS2 sends** (on demand):
```
MOTOR:150,-150
```

**Arduino acknowledges**:
```
ACK:MOTOR:150,-150
```

---

### Robotic Arm

**ROS2 sends**:
```
SERVO:0,90
SERVO:1,45
SERVO:2,120
SERVO:3,60
SERVO:4,90
```

**Arduino acknowledges each**:
```
ACK:SERVO:0,90
ACK:SERVO:1,45
...
```

---

## Timing and Frequency

### Recommended Publishing Rates

| Sensor Type | Frequency | Interval | Rationale |
|-------------|-----------|----------|-----------|
| Temperature | 1 Hz | 1000 ms | Slow-changing |
| Humidity | 1 Hz | 1000 ms | Slow-changing |
| Pressure | 1 Hz | 1000 ms | Slow-changing |
| Light | 2 Hz | 500 ms | Moderate changes |
| Distance (Ultrasonic) | 10 Hz | 100 ms | Obstacle detection |
| IMU | 50 Hz | 20 ms | Control loop |
| GPS | 5 Hz | 200 ms | Position updates |
| Encoders | 50 Hz | 20 ms | Odometry |
| Battery | 0.1 Hz | 10000 ms | Very slow |

### Arduino Timing Example

```cpp
unsigned long lastTemp = 0;
unsigned long lastIMU = 0;
unsigned long lastDistance = 0;

void loop() {
  // Temperature at 1Hz
  if (millis() - lastTemp >= 1000) {
    lastTemp = millis();
    publishTemperature();
  }

  // IMU at 50Hz
  if (millis() - lastIMU >= 20) {
    lastIMU = millis();
    publishIMU();
  }

  // Distance at 10Hz
  if (millis() - lastDistance >= 100) {
    lastDistance = millis();
    publishDistance();
  }

  // Check for incoming commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}
```

---

## Error Handling

### Malformed Messages

**Examples of invalid messages**:
- Missing colon: `TEMP25.3`
- Missing newline: `TEMP:25.3` (no `\n`)
- Empty value: `TEMP:`
- Wrong type: `TEMP:hot`

**Bridge Behavior**:
- Log warning
- Discard message
- Continue processing
- Don't crash

### Out-of-Range Values

**Examples**:
- `TEMP:999.9` (unrealistic)
- `SERVO:0,270` (servo > 180°)
- `MOTOR:500,500` (PWM > 255)

**Arduino should**:
- Constrain values: `angle = constrain(angle, 0, 180);`
- Validate before use
- Send ERROR message if critical

### Serial Buffer Overflow

**Prevention**:
```cpp
// Clear buffer if too much junk accumulates
if (Serial.available() > 100) {
  while (Serial.available()) Serial.read();
  Serial.println("ERROR:BUFFER_OVERFLOW");
}
```

---

## Advanced Features

### Binary Protocol (Optional)

For high-frequency data (>100 Hz), use binary:

**Structure**:
```cpp
struct __attribute__((packed)) {
  uint8_t header;  // 0xFF
  uint8_t type;    // Message type ID
  float data[6];   // Payload
  uint8_t checksum;
} BinaryMessage;
```

**Pros**:
- Faster parsing
- Less bandwidth
- Higher frequency

**Cons**:
- Harder to debug
- Endianness issues
- More complex implementation

**Recommendation**: Start with text protocol, upgrade to binary if needed

---

### Compression

For bandwidth-constrained links:
```cpp
// Instead of: TEMP:25.3\nHUMIDITY:60.5\n
// Use: ENV:25.3,60.5\n
```

Saves ~20% bandwidth

---

### Checksums (Reliability)

Add checksum for noisy environments:
```
TEMP:25.3:*A5\n
       └─ XOR checksum in hex
```

**Arduino**:
```cpp
uint8_t calculateChecksum(String msg) {
  uint8_t checksum = 0;
  for (char c : msg) checksum ^= c;
  return checksum;
}
```

---

## Testing Protocol

### Arduino Serial Monitor Test

1. Upload code to Arduino
2. Open Serial Monitor (115200 baud)
3. Observe output:
   ```
   STATUS:READY
   TEMP:25.3
   HUMIDITY:60.5
   PRESSURE:1013.25
   ```

4. Send commands:
   ```
   MOTOR:150,-150
   ```

5. Verify acknowledgment:
   ```
   ACK:MOTOR:150,-150
   ```

### Python Test Script

```python
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino reset

# Read sensor data
for i in range(10):
    line = ser.readline().decode('utf-8').strip()
    print(f"Received: {line}")

# Send command
ser.write(b'MOTOR:100,100\n')
time.sleep(0.1)
ack = ser.readline().decode('utf-8').strip()
print(f"ACK: {ack}")

ser.close()
```

---

## Protocol Extension Guide

### Adding New Sensor

1. **Define message format**:
   ```
   NEWSENSOR:value1,value2
   ```

2. **Arduino implementation**:
   ```cpp
   float val1 = sensor.read1();
   float val2 = sensor.read2();
   Serial.print("NEWSENSOR:");
   Serial.print(val1, 2); Serial.print(",");
   Serial.println(val2, 2);
   ```

3. **ROS2 bridge**:
   ```python
   elif key == 'NEWSENSOR':
       values = [float(v) for v in value.split(',')]
       msg = YourMessageType()
       msg.field1 = values[0]
       msg.field2 = values[1]
       self.new_sensor_pub.publish(msg)
   ```

4. **Document in this file**

---

## Best Practices

### Do's ✅

- Use UPPERCASE for keys
- Include units in documentation
- Send newline after every message
- Validate data before publishing
- Send ACK for commands
- Use consistent precision
- Test with Serial Monitor first

### Don'ts ❌

- Don't use spaces in keys
- Don't send too fast (buffer overflow)
- Don't forget newlines
- Don't publish invalid data
- Don't block in loop() (use millis())
- Don't use Serial.print() for debug in production

---

## Comparison: Text vs Binary

| Feature | Text Protocol | Binary Protocol |
|---------|---------------|-----------------|
| **Readability** | ✅ Human-readable | ❌ Requires tools |
| **Debugging** | ✅ Easy (Serial Monitor) | ❌ Need hex viewer |
| **Speed** | ⚠️ ~10-50 Hz typical | ✅ 100+ Hz possible |
| **Bandwidth** | ⚠️ ~50 bytes/msg | ✅ ~10-20 bytes/msg |
| **Implementation** | ✅ Simple | ⚠️ Complex |
| **Cross-platform** | ✅ Always works | ⚠️ Endianness issues |
| **Error handling** | ✅ Easy to recover | ⚠️ Sync problems |

**Recommendation**: Use text protocol unless you need >100 Hz or have bandwidth constraints

---

## Summary

The RoboShire serial protocol provides:
- ✅ Simple, extensible message format
- ✅ Bidirectional communication
- ✅ Easy debugging
- ✅ Robust error handling
- ✅ Comprehensive sensor coverage
- ✅ Command acknowledgment
- ✅ Real-world tested in weather stations and mobile robots

**Next Step**: Implement using [Serial Bridge Implementation Guide](serial_bridge_implementation.md)

---

**Document Info**
- **Version**: 1.0.0
- **Status**: Stable
- **Compatibility**: Arduino Uno/Mega/Nano, ESP32, ESP8266, Teensy, STM32
- **Tested With**: RoboShire v2.3.0, ROS2 Humble
- **License**: Creative Commons BY 4.0
