# RoboShire Hardware Integration Guide

**Version:** 2.4.0
**Last Updated:** 2025-10-29
**Target Audience:** Hardware engineers, embedded developers, robotics hobbyists

---

## Overview

This documentation provides comprehensive guidance for integrating real hardware (Arduino, ESP32, sensors, motors) with RoboShire's ROS2 development environment. Whether you're building a weather station, mobile robot, or custom sensor array, these guides will help you go from hardware design to working ROS2 system.

## What Can RoboShire Do for Hardware Engineers?

- **Visual Robot Design**: Create robots using drag-and-drop node graph editor
- **Arduino/ESP32 Code Generation**: Automatically generate microcontroller code from GUI
- **ROS2 Integration**: Connect hardware to ROS2 topics via serial communication
- **URDF Robot Modeling**: Create and validate robot descriptions visually
- **3D Simulation**: Test robots in MuJoCo before hardware deployment
- **Parameter Configuration**: Tune hardware settings via YAML files
- **Complete Examples**: Pre-built projects for common robot types
- **🆕 micro-ROS Agent GUI**: No-terminal management of micro-ROS connections *(v2.4.0)*
- **🆕 Hardware Testing Panel**: Interactive sensor/actuator testing in real-time *(v2.4.0)*

---

## Quick Start

### Brand New to RoboShire?
Start here for a 15-minute hands-on introduction:
- **[Hardware Quick Start](HARDWARE_QUICK_START.md)** - Build a weather station from GUI to working hardware

### Have Arduino Experience?
Jump straight to integration:
- **[Arduino Code Workflow](ARDUINO_CODE_WORKFLOW.md)** - GUI → Code → Flash → Test
- **[Serial Bridge Implementation](communication/serial_bridge_implementation.md)** - Connect Arduino to ROS2

### Want Complete Examples?
Learn from working projects:
- **[Weather Station](examples/weather_station/README.md)** - Sensors + ESP32 + ROS2
- **[Differential Drive Robot](examples/differential_drive_robot/README.md)** - Motors + IMU + Navigation

---

## Documentation Structure

### Getting Started
Perfect for first-time hardware integration:

| Guide | Description | Time to Complete |
|-------|-------------|------------------|
| [Hardware Quick Start](HARDWARE_QUICK_START.md) | Build your first RoboShire hardware project | 15-30 minutes |
| [Hardware Selection Guide](getting_started/hardware_selection_guide.md) | Choose the right microcontroller for your project | 10 minutes |
| [Workflow Wizard for Hardware](gui_guides/workflow_wizard_hardware.md) | Using the GUI wizard for hardware projects | 15 minutes |

### 🆕 v2.4.0 GUI Tools (NEW!)
No more terminal commands - manage everything from the GUI:

| Tool | Description | Shortcut | Time Saved |
|------|-------------|----------|------------|
| **[micro-ROS Agent Manager](MICRO_ROS_AGENT_MANAGER.md)** | GUI for starting/stopping micro-ROS agent, auto-detect Arduino/ESP32, auto-reconnect | Ctrl+Shift+M | 90% |
| **[Hardware Testing Panel](HARDWARE_TESTING_PANEL.md)** | Interactive sensor monitoring and motor control with live graphs and emergency stop | Ctrl+T | 85% |

**Why these matter:**
- **Before**: 5 terminal windows, constant `ros2 topic echo` commands, manual motor commands
- **After**: Everything in one GUI with real-time visualization

### Hardware Components

#### Sensors
Complete guides for integrating sensors:

**[Sensors Guide](SENSORS_GUIDE.md)** includes:
- IMU (Inertial Measurement Unit) - MPU6050, BNO055
- Ultrasonic Distance Sensors - HC-SR04
- Temperature & Humidity - DHT22, DS18B20
- Pressure Sensors - BMP280
- GPS Modules
- Cameras and LiDAR
- Encoders for odometry
- Environmental sensors (gas, light, soil)

#### Actuators
Complete guides for motor and actuator control:

**[Actuators Guide](ACTUATORS_GUIDE.md)** includes:
- DC Motors - L298N, TB6612 motor drivers
- Servo Motors - Standard PWM control
- Stepper Motors - A4988, DRV8825 drivers
- Brushless Motors - ESC control
- Linear Actuators
- Relays and solenoids

### Communication

| Guide | Description | Critical? |
|-------|-------------|-----------|
| [Serial Protocol Specification](communication/serial_protocol.md) | Message format and data encoding | ⭐ Required Reading |
| [Serial Bridge Implementation](communication/serial_bridge_implementation.md) | Complete PySerial ROS2 node guide | ⭐ Critical for Arduino↔ROS2 |

### Robot Modeling

| Guide | Description |
|-------|-------------|
| [URDF Hardware Guide](URDF_HARDWARE_GUIDE.md) | Creating robot descriptions for real hardware |
| [Parameter Configuration](PARAMETER_CONFIGURATION.md) | Hardware tuning via YAML files |

### Complete Examples

#### 1. Weather Station (Beginner-Friendly)
**Hardware**: ESP32, DHT22, BMP280, Light Sensor
**Cost**: ~$20
**Time**: 1-2 hours
**Learn**: Sensor reading, serial communication, ROS2 topics

**Files**:
- [Overview](examples/weather_station/README.md)
- [Bill of Materials](examples/weather_station/hardware_bom.md)
- [Wiring Diagram](examples/weather_station/wiring_diagram.md)
- [Setup Guide](examples/weather_station/setup_guide.md)
- [Testing Guide](examples/weather_station/testing_guide.md)

#### 2. Differential Drive Robot (Intermediate)
**Hardware**: Arduino Mega, L298N, MPU6050, DC Motors, Encoders
**Cost**: ~$60-75
**Time**: 3-4 hours
**Learn**: Motor control, odometry, sensor fusion, navigation, ROS2 integration

**Files**:
- [Complete README](examples/differential_drive_robot/README.md) - Full step-by-step guide
- [Bill of Materials](examples/differential_drive_robot/hardware_bom.md) - Components & suppliers
- [Wiring Diagram](examples/differential_drive_robot/wiring_diagram.md) - Pin connections
- [Arduino Code](examples/differential_drive_robot/arduino_code.ino) - Complete firmware
- [Motor Calibration](examples/differential_drive_robot/motor_calibration.md) - Tuning procedures

#### 3. 6-DOF Robotic Arm (Advanced)
**Hardware**: Arduino Mega, PCA9685, 6x Servo Motors
**Cost**: ~$140-180
**Time**: 4-6 hours
**Learn**: Servo control, kinematics, MoveIt2, trajectory planning

**Files**:
- [Complete README](examples/robotic_arm_6dof/README.md) - Full assembly guide
- [Bill of Materials](examples/robotic_arm_6dof/hardware_bom.md) - Servo selection guide
- [Servo Control Code](examples/robotic_arm_6dof/servo_control.ino) - Firmware

#### 4. Line Following Robot (Beginner)
**Hardware**: Arduino Uno, QTR-8A IR Sensors, DC Motors, L298N
**Cost**: ~$45-57
**Time**: 2-3 hours
**Learn**: Sensor arrays, PID control, reactive behavior

**Files**:
- [Complete README](examples/line_following_robot/README.md) - Beginner-friendly guide
- [Bill of Materials](examples/line_following_robot/hardware_bom.md) - Cost breakdown
- [Arduino Code with PID](examples/line_following_robot/arduino_code.ino) - Control firmware

### Troubleshooting

| Guide | Description |
|-------|-------------|
| [Common Hardware Issues](TROUBLESHOOTING_HARDWARE.md) | FAQ and solutions for typical problems |

### Reference

| Guide | Description |
|-------|-------------|
| [Supported Hardware List](reference/supported_hardware.md) | Complete compatibility list for MCUs, sensors, actuators |

---

## Typical Hardware Workflow

```
1. Design Robot in GUI
   └─> Use Workflow Wizard
   └─> Select microcontroller
   └─> Choose sensors/actuators
   └─> Configure parameters

2. Generate Arduino Code
   └─> Wizard generates .ino file
   └─> Code includes sensor reading
   └─> Serial communication built-in

3. Flash to Microcontroller
   └─> Open in Arduino IDE
   └─> Select board and port
   └─> Upload code

4. Create ROS2 Package
   └─> Visual node graph in GUI
   └─> Auto-generate ROS2 nodes
   └─> Build with colcon

5. Connect Hardware to ROS2
   └─> Implement serial bridge node
   └─> Map Arduino data to ROS2 topics
   └─> Test communication

6. Test and Iterate
   └─> Monitor topics with ros2 topic echo
   └─> Tune parameters
   └─> Validate in simulation (MuJoCo)

7. Deploy to Robot
   └─> SSH to Ubuntu VM or embedded Linux
   └─> Launch complete system
   └─> Monitor and debug
```

---

## Hardware Capabilities Summary

### ✅ What RoboShire Provides

| Feature | Status | Description |
|---------|--------|-------------|
| Arduino Code Templates | ✅ Working | Pre-built code for sensors, motors, serial |
| ROS2 Package Generation | ✅ Working | Complete packages from visual node graphs |
| URDF Validation | ✅ Working | 50+ validation rules with fix suggestions |
| Workflow Wizard | ✅ Working | Step-by-step hardware project setup |
| MuJoCo Simulation | ✅ Working | 3D visualization before hardware testing |
| Parameter System | ✅ Working | YAML-based configuration |
| Code Examples | ✅ Working | Weather station, differential drive robot |

### ⚠️ What Requires Manual Implementation

| Feature | Status | Workaround |
|---------|--------|------------|
| Serial Bridge Node | ⚠️ Template Only | Follow [Serial Bridge Guide](communication/serial_bridge_implementation.md) |
| Arduino Code Auto-Save | ⚠️ Manual Copy | Copy from preview window and save as .ino |
| Hardware Test Panel | ⚠️ Not Available | Use Arduino Serial Monitor + `ros2 topic echo` |
| Visual URDF Editor | ⚠️ Validator Only | Edit XML manually, start from examples |

---

## Supported Hardware

### Microcontrollers
- ✅ **Arduino**: Uno, Mega 2560, Nano, Micro
- ✅ **ESP32**: DevKit, WROOM, WROVER (WiFi/Bluetooth)
- ✅ **ESP8266**: NodeMCU, Wemos D1
- ✅ **Teensy**: 3.x, 4.0, 4.1 (high-performance)
- ✅ **STM32**: Nucleo, Blue Pill (ARM Cortex-M)
- ✅ **Raspberry Pi Pico**: RP2040 (dual-core)
- ⚠️ **micro-ROS**: ESP32, Teensy, STM32 (run ROS2 on MCU)

### Sensors (Extensive Support)
- IMU, Ultrasonic, Temperature, Humidity, Pressure
- GPS, Encoders, Camera, LiDAR, Radar
- Current, Voltage, Gas, PIR Motion, Light
- See [Supported Hardware List](reference/supported_hardware.md) for complete catalog

### Actuators
- DC Motors, Servo Motors, Stepper Motors
- Brushless Motors (ESC), Linear Actuators
- Relays, Solenoids, LED Strips

---

## Prerequisites

### Hardware Requirements
- Development computer (Windows/Linux/macOS)
- Microcontroller (Arduino, ESP32, etc.)
- USB cable for programming
- Sensors/actuators for your project
- Breadboard and jumper wires (prototyping)

### Software Requirements
- **RoboShire GUI** (installed)
- **Arduino IDE** or **PlatformIO** (for flashing code)
- **ROS2 Humble** or newer (for ROS integration)
- **Python 3.10+** with PySerial (for serial bridge)

### Knowledge Requirements
- Basic electronics (wiring, power, ground)
- Arduino programming (helpful but not required)
- ROS2 concepts (topics, nodes - basics covered in guides)

---

## Learning Path

### Beginner Path (No Prior Experience)
**Time**: 2-4 hours

1. Read [Hardware Quick Start](HARDWARE_QUICK_START.md) (15 min)
2. Follow [Weather Station Example](examples/weather_station/README.md) (2 hours)
3. Review [Serial Protocol](communication/serial_protocol.md) (15 min)
4. Experiment with parameters and sensors (1 hour)

**Outcome**: Working weather station publishing data to ROS2

### Intermediate Path (Have Arduino Experience)
**Time**: 4-6 hours

1. Read [Arduino Code Workflow](ARDUINO_CODE_WORKFLOW.md) (20 min)
2. Complete [Serial Bridge Implementation](communication/serial_bridge_implementation.md) (1 hour)
3. Build [Differential Drive Robot](examples/differential_drive_robot/README.md) (3 hours)
4. Learn [URDF Hardware Guide](URDF_HARDWARE_GUIDE.md) (1 hour)

**Outcome**: Mobile robot with motors, sensors, and navigation

### Advanced Path (Building Custom Robot)
**Time**: 8-12 hours

1. Review [Hardware Selection Guide](getting_started/hardware_selection_guide.md) (30 min)
2. Study [Sensors Guide](SENSORS_GUIDE.md) + [Actuators Guide](ACTUATORS_GUIDE.md) (2 hours)
3. Create custom URDF with [URDF Hardware Guide](URDF_HARDWARE_GUIDE.md) (2 hours)
4. Implement custom sensors and actuators (4 hours)
5. Tune with [Parameter Configuration](PARAMETER_CONFIGURATION.md) (1 hour)
6. Deploy and iterate (2+ hours)

**Outcome**: Custom robot tailored to your application

---

## Getting Help

### Documentation Issues
If you find errors or unclear sections:
- **GitHub Issues**: https://github.com/YOUR_REPO/roboshire/issues
- **Tag**: `documentation` + `hardware`

### Hardware Problems
Check troubleshooting first:
- [Troubleshooting Hardware](TROUBLESHOOTING_HARDWARE.md)
- Search for error messages
- Check wiring diagrams

### Community
- **ROS2 Discourse**: https://discourse.ros.org/
- **Arduino Forums**: https://forum.arduino.cc/
- **RoboShire Discussions**: (Link to your discussion forum)

---

## Contributing

### Improving Hardware Docs
We welcome contributions!

**Easy contributions**:
- Report unclear sections
- Suggest missing examples
- Share your hardware projects

**Medium contributions**:
- Add sensor integration guides
- Write troubleshooting solutions
- Create wiring diagrams

**Advanced contributions**:
- Complete hardware example projects
- Video tutorials
- Testing and validation

See **CONTRIBUTING.md** for guidelines.

---

## What's Next?

### Recommended First Step
👉 **Start with [Hardware Quick Start](HARDWARE_QUICK_START.md)** to build your first project in 15 minutes

### After Quick Start
- Explore complete examples
- Learn about sensors and actuators
- Build custom robot
- Share your projects!

---

## Document Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-10-29 | Initial comprehensive hardware documentation |

---

**Ready to build robots?** Start with the [Hardware Quick Start](HARDWARE_QUICK_START.md)!
