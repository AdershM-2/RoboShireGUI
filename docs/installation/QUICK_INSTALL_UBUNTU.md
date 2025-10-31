# RoboShire Quick Install Guide (Ubuntu)

**Version**: 2.3.0

Fast-track installation for Ubuntu 22.04+

---

## One-Line Install

```bash
curl -fsSL https://raw.githubusercontent.com/roboshire/roboshire/main/install_ubuntu.sh | bash
```

**Installation time**: 3-5 minutes

**What this installs**:
- RoboShire GUI application
- Python dependencies (PySide6, etc.)
- Desktop application entry
- `roboshire` command-line tool
- Default workspace at `~/roboshire_workspace`

---

## Launch RoboShire

After installation, launch using any method:

### Method 1: Command Line
```bash
roboshire
```

### Method 2: Applications Menu
Press Super key → Search "RoboShire" → Click icon

### Method 3: Direct Launch
```bash
~/roboshire/roboshire-ubuntu.sh
```

---

## First Launch Setup

On first launch, choose your execution mode:

### Local Mode (Recommended for Development)
- ✅ Run ROS2 on your Ubuntu machine
- ✅ No network required
- ✅ Fastest performance
- **Requires**: ROS2 installed locally

### Remote Mode (For Robot Deployment)
- ✅ Deploy to Jetson, Raspberry Pi, or remote Ubuntu
- ✅ Develop locally, run on robot hardware
- **Requires**: SSH access to device with ROS2

---

## Install ROS2 (for Local Mode)

If you chose Local Mode and don't have ROS2:

```bash
# Set up ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Switch Execution Mode Anytime

You can switch between Local and Remote modes anytime:

1. **Settings** → **Execution Mode...**
2. Select **Local** or **Remote**
3. Configure settings
4. Click **Apply**

Changes take effect immediately!

---

## Quick Troubleshooting

**RoboShire command not found?**
```bash
source ~/.bashrc
# Or use: ~/roboshire/roboshire-ubuntu.sh
```

**ROS2 not found (Local Mode)?**
```bash
source /opt/ros/humble/setup.bash
ros2 --version
# If error, install ROS2 (see above)
```

**SSH connection failed (Remote Mode)?**
- Test: `ssh user@device-ip` from terminal
- Check SSH service: `sudo systemctl status ssh` on remote device
- Use Settings → Execution Mode → "Test SSH Connection"

---

## Next Steps

1. **Try Examples**: Getting Started tab → Browse Examples
2. **Create Project**: New Project → Select template
3. **Build & Run**: Build menu → Build Workspace → Run

---

## Full Documentation

- **Complete Installation Guide**: [docs/installation/UBUNTU_INSTALLATION.md](docs/installation/UBUNTU_INSTALLATION.md)
- **Execution Modes Guide**: [docs/guides/EXECUTION_MODES.md](docs/guides/EXECUTION_MODES.md)
- **Jetson/RPi Deployment**: [docs/guides/JETSON_RPI_DEPLOYMENT.md](docs/guides/JETSON_RPI_DEPLOYMENT.md)

---

## System Requirements

- **OS**: Ubuntu 22.04 LTS or later
- **RAM**: 4 GB minimum (8 GB recommended)
- **Python**: 3.8+ (included in Ubuntu 22.04+)
- **ROS2**: Humble/Iron/Jazzy/Rolling (optional for Remote Mode)

---

**Happy robot building! 🤖**
