# RoboShireGUI - Visual IDE for ROS2 Robotics

<div align="center">

![RoboShireGUI Logo](docs/images/roboshire_logo.png)

**The beginner-friendly GUI for ROS2 that makes robotics accessible to Arduino-level users**

[![Version](https://img.shields.io/badge/version-2.5.0-blue.svg)](https://github.com/AdershM-2/RoboShireGUI/releases)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-MIT-orange.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.10%2B-yellow.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/platform-Ubuntu%20%7C%20Jetson%20%7C%20RPi-lightgrey.svg)](docs/installation/CROSS_PLATFORM_INSTALLATION.md)

[**Quick Start**](#-quick-start) • [**Features**](#-features) • [**Documentation**](#-documentation) • [**Examples**](#-examples)

</div>

---

## 🎯 What is RoboShireGUI?

RoboShireGUI is a **visual IDE for ROS2** that transforms complex robotics development into a drag-and-drop experience. Build professional robots without wrestling with command-line tools, cryptic error messages, or QoS configuration nightmares!

### The Problem
```bash
# Traditional ROS2 workflow:
$ ros2 pkg create my_robot --build-type ament_python
$ cd my_robot && mkdir my_robot/nodes
$ nano my_robot/nodes/motor_controller.py
# ... 200 lines of boilerplate code ...
$ colcon build
ERROR: QoS incompatible!
$ # Hours of debugging...
```

### The RoboShireGUI Solution
1. 🎨 **Drag & Drop** - Visual node graph editor
2. ⚙️ **Smart Presets** - QoS made simple
3. 🔨 **One-Click Build** - Press Ctrl+B
4. ▶️ **One-Click Run** - Press Ctrl+R
5. ✅ **100% Standard ROS2** - No vendor lock-in

---

## 🚀 Quick Start

### One-Line Installation (Ubuntu 22.04/24.04)

```bash
curl -fsSL https://raw.githubusercontent.com/AdershM-2/RoboShireGUI/main/install_roboshire_ubuntu.sh | bash
```

Or manual installation:

```bash
# 1. Clone repository
git clone https://github.com/AdershM-2/RoboShireGUI.git
cd RoboShireGUI

# 2. Run installer (auto-installs ROS2 if needed)
./install_roboshire_ubuntu.sh

# 3. Launch RoboShireGUI
roboshire
```

### First Steps

1. **Launch**: Run `roboshire` from terminal or desktop icon
2. **Try Example**: File → New from Example → Weather Station
3. **Build**: Press Ctrl+B
4. **Run**: Press Ctrl+R
5. **Done**: See live data in Topic Inspector!

---

## ✨ Features

### Visual Development
- 🎨 **Drag-and-Drop Node Graph** - Create ROS2 nodes without coding
- 🤖 **URDF Visual Editor** - Build robot models graphically
- 📊 **Real-Time Monitoring** - Live topics, parameters, and node status
- 🎮 **MuJoCo Simulation** - Physics-based 3D simulation (60 FPS)
- 🚀 **Command Palette** - VS Code-style fuzzy search (Ctrl+Shift+P)

### Code Generation
- 🐍 **100% Standard ROS2** - Generates professional Python packages
- ⚡ **Arduino Firmware** - Auto-generate microcontroller code
- ⚙️ **Smart QoS Presets** - Visual configuration with compatibility checking
- 📦 **Multi-Package Support** - Complex projects with dependencies

### Hardware Integration
- 🔌 **Serial Bridge** - UART communication with Arduino/ESP32
- 🧪 **Hardware Test Panel** - Interactive sensor/actuator testing
- 📡 **micro-ROS Support** - Seamless microcontroller integration
- 🚁 **Remote Deployment** - Deploy to Jetson/Raspberry Pi via SSH

### Developer Tools
- 💻 **Built-in Code Editor** - Syntax highlighting and auto-complete
- 🐛 **Error Translator** - Friendly error messages with solutions
- 📈 **Performance Profiler** - Monitor node CPU/memory usage
- 🌳 **TF Tree Visualizer** - Visualize coordinate transforms
- 📋 **Stack Trace Viewer** - Debug ROS2 errors easily

### Navigation & Planning
- 🧭 **Nav2 Integration Wizard** - 6-step navigation setup
- 🌲 **Behavior Tree Editor** - Visual behavior tree design
- 🗺️ **AI-Powered Recommendations** - Smart planner suggestions

### Quality of Life
- 💾 **Auto-Save** - Every 5 minutes
- ↩️ **Undo/Redo** - Full history in Node Graph
- 🪟 **Layout Persistence** - Remembers your workspace
- 🎨 **70+ Icons** - Fast visual navigation
- ⌨️ **40+ Keyboard Shortcuts** - Power-user friendly

---

## 🤖 Included Examples

8 complete robot projects ready to run:

1. **Differential Drive Robot** - Mobile robot with navigation
2. **Weather Station** - IoT sensor hub
3. **6-DOF Robotic Arm** - Manipulation example
4. **Quadcopter Drone** - Flight control basics
5. **Line Following Robot** - Vision-based navigation
6. **Humanoid Robot** - Bipedal locomotion
7. **Quadruped Robot** - Four-legged walking
8. **TurtleBot3 Burger** - Standard platform

Each includes node graphs, URDF models, launch files, and Arduino firmware (where applicable).

---

## 📚 Documentation

| Guide | Description |
|-------|-------------|
| [Installation Guide](docs/installation/UBUNTU_INSTALLATION_GUIDE.md) | Complete setup instructions |
| [Quick Reference](docs/ROBOSHIRE_V2.5.0_QUICK_REFERENCE.md) | All features at a glance |
| [Tutorials](docs/tutorials/README.md) | Step-by-step learning path |
| [Hardware Guide](docs/hardware/README.md) | Arduino/ESP32 integration |
| [Deployment Guide](docs/guides/JETSON_RPI_DEPLOYMENT.md) | Jetson/Raspberry Pi setup |

### Popular Tutorials
- [Weather Station in 10 Minutes](docs/tutorials/WEATHER_STATION_TUTORIAL.md)
- [Autonomous Robot in 30 Minutes](docs/tutorials/ROBOSHIRE_SELF_DRIVING_TUTORIAL.md)
- [Hardware Setup from URDF](docs/guides/URDF_TO_ROBOT_TUTORIAL.md)

---

## 🎓 Who Should Use RoboShireGUI?

### ✅ Perfect For:
- **Beginners** learning ROS2
- **Educators** teaching robotics
- **Hobbyists** building robots
- **Students** working on projects
- **Researchers** prototyping quickly

### ⚠️ Maybe Not For:
- ROS1 users (ROS2 Humble only)
- Pure CLI enthusiasts
- Production deployments (use standard ROS2 tools)

---

## 💻 System Requirements

### Minimum
- **OS**: Ubuntu 22.04 or 24.04
- **RAM**: 4 GB (8 GB recommended)
- **Disk**: 10 GB free space
- **Python**: 3.10+
- **ROS2**: Humble (auto-installed)

### Optional Hardware
- Arduino / ESP32 for hardware projects
- NVIDIA Jetson or Raspberry Pi for deployment
- Sensors: IMU, ultrasonic, camera, LIDAR
- Motors: DC, servo, stepper

---

## 🛠️ Technology Stack

- **GUI**: PySide6 (Qt6)
- **ROS2**: Humble (Python)
- **Simulation**: MuJoCo 3.0+
- **Build**: colcon
- **Hardware**: micro-ROS, Arduino, ESP32
- **Visualization**: RViz2, Qt OpenGL

---

## 🤝 Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

- 🐛 [Report bugs](https://github.com/AdershM-2/RoboShireGUI/issues)
- 💡 [Suggest features](https://github.com/AdershM-2/RoboShireGUI/discussions)
- 📖 Improve documentation
- 🔧 Submit pull requests

---

## 🗺️ Roadmap

### Future Plans
- AI-powered code suggestions
- Cloud workspace sync
- Mobile monitoring app
- Multi-robot coordination
- Gazebo integration
- Web-based interface

See [CHANGELOG.md](CHANGELOG.md) for version history.

---

## 📄 License

RoboShireGUI is open source under the [MIT License](LICENSE).

Generated ROS2 code uses **Apache 2.0 License** for maximum compatibility.

---

## 🙏 Acknowledgments

- **ROS2 Team** for the incredible robotics framework
- **Qt/PySide6** for the GUI toolkit
- **MuJoCo Team** for physics simulation
- **micro-ROS** for microcontroller integration
- **All contributors** who made this possible

---

## 🚀 Get Started Now!

```bash
# Install
curl -fsSL https://raw.githubusercontent.com/AdershM-2/RoboShireGUI/main/install_roboshire_ubuntu.sh | bash

# Launch
roboshire
```

**New to ROS2?** Start with the [Weather Station Tutorial](docs/tutorials/WEATHER_STATION_TUTORIAL.md).

**Experienced?** Check out the [Command Palette](docs/ROBOSHIRE_V2.5.0_QUICK_REFERENCE.md) and power-user features!

---

<div align="center">

**Made with ❤️ for the robotics community**

⭐ Star us on GitHub if RoboShireGUI helped you!

[Documentation](docs/README.md) • [Tutorials](docs/tutorials/README.md) • [Hardware Guide](docs/hardware/README.md)

</div>
