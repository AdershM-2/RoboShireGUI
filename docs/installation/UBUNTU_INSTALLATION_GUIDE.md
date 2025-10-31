# RoboShire Ubuntu Installation Guide

**Version**: 2.3.0
**Supported**: Ubuntu 22.04 LTS (Recommended), Ubuntu 24.04 LTS
**Installation Time**: 20-30 minutes
**Disk Space**: ~5-7 GB

---

## One-Command Installation (Recommended)

### Quick Install

```bash
curl -fsSL https://roboshire.com/install.sh | bash
```

**OR** if you have the script locally:

```bash
cd /path/to/ROS2_PROJECT/docs/installation
chmod +x install_roboshire_ubuntu.sh
./install_roboshire_ubuntu.sh
```

This automated script installs:
- ✅ ROS 2 (Humble for Ubuntu 22.04, Jazzy for Ubuntu 24.04)
- ✅ Python dependencies (PyQt5, PySerial, etc.)
- ✅ MuJoCo physics simulator
- ✅ Arduino IDE and tools
- ✅ RoboShire GUI application
- ✅ Desktop launcher
- ✅ Serial port permissions

---

## System Requirements

### Minimum Requirements
- **OS**: Ubuntu 22.04 LTS (64-bit)
- **RAM**: 4 GB (8 GB recommended)
- **Storage**: 10 GB free space
- **Display**: 1280x720 resolution
- **Internet**: Required for installation

### Recommended Requirements
- **OS**: Ubuntu 22.04 LTS (latest updates)
- **RAM**: 8+ GB
- **Storage**: 20+ GB SSD
- **Display**: 1920x1080 resolution
- **CPU**: 4+ cores (Intel i5/Ryzen 5 or better)

### Supported Ubuntu Versions

| Ubuntu Version | ROS 2 Distro | Status | Notes |
|---------------|--------------|--------|-------|
| 22.04 LTS | Humble | ✅ Recommended | Best support |
| 24.04 LTS | Jazzy | ✅ Supported | Newer, fewer packages |
| 20.04 LTS | Foxy | ⚠️ EOL | End-of-life, not recommended |

---

## What Gets Installed

### 1. ROS 2 Desktop
- **Size**: ~3 GB
- **Includes**: Core libraries, rqt tools, RViz2, Gazebo
- **Repository**: packages.ros.org

### 2. Python Dependencies
- **PyQt5**: GUI framework
- **PySerial**: Arduino/ESP32 communication
- **Jinja2**: Code template rendering
- **lxml**: URDF/XML parsing
- **matplotlib**, **numpy**: Data visualization
- **paramiko**: SSH/remote deployment

### 3. MuJoCo Simulator
- **Version**: 3.1.0
- **Size**: ~200 MB
- **Purpose**: 3D robot simulation and visualization

### 4. Development Tools
- **Arduino IDE**: Microcontroller programming
- **colcon**: ROS 2 build tool
- **rosdep**: Dependency management
- **Git**: Version control

### 5. RoboShire Application
- **Location**: `~/roboshire/`
- **Size**: ~100 MB
- **Components**: GUI, backend, examples, documentation

---

## Installation Steps Explained

### Step 1: System Check (2 minutes)
The installer verifies:
- Ubuntu version and architecture
- Available RAM and disk space
- Internet connectivity
- Existing ROS 2 installation

### Step 2: ROS 2 Installation (10-15 minutes)
- Adds ROS 2 apt repository
- Installs `ros-humble-desktop` (or `jazzy` for Ubuntu 24.04)
- Initializes `rosdep` for dependency management
- Configures environment variables

**Output example**:
```
==> Installing ROS 2 humble...
Reading package lists... Done
Building dependency tree... Done
The following NEW packages will be installed:
  ros-humble-desktop (2400+ packages)
...
✓ ROS 2 humble installed successfully
```

### Step 3: Python Dependencies (3-5 minutes)
- Installs pip3 if not present
- Installs RoboShire Python requirements
- Configures user-level packages (no sudo needed)

### Step 4: MuJoCo Download (2-3 minutes)
- Downloads MuJoCo 3.1.0 from GitHub releases
- Extracts to `~/.mujoco/`
- Installs Python bindings (`mujoco` package)
- Configures library paths

### Step 5: Clone RoboShire (1-2 minutes)
- Clones repository to `~/roboshire/`
- Creates workspace structure
- Copies example projects

### Step 6: Build Workspace (2-3 minutes)
- Runs `colcon build` for any pre-existing packages
- Creates install/ and build/ directories
- Sources workspace automatically

### Step 7: Environment Configuration (< 1 minute)
Adds to `~/.bashrc`:
```bash
export ROBOSHIRE_HOME="$HOME/roboshire"
source /opt/ros/humble/setup.bash
source $ROBOSHIRE_HOME/workspace/install/setup.bash
alias roboshire="python3 $ROBOSHIRE_HOME/roboshire"
```

### Step 8: Desktop Launcher (< 1 minute)
Creates `~/.local/share/applications/roboshire.desktop`:
- Appears in application menu as "RoboShire"
- Automatically sources ROS 2 environment
- Sets correct working directory

### Step 9: Serial Permissions (< 1 minute)
- Adds user to `dialout` group
- Creates udev rules for Arduino/ESP32
- **Requires logout/login to take effect**

### Step 10: Post-Install Tests (1 minute)
Verifies:
- ROS 2 commands work (`ros2 --version`)
- Python imports succeed (`import PyQt5`)
- RoboShire module loads

---

## After Installation

### 1. Log Out and Log In
**IMPORTANT**: Required for serial port access (dialout group)

```bash
# Check if you're in dialout group
groups | grep dialout
# If not found, log out and log in
```

### 2. Launch RoboShire

**Method A**: Application Menu
- Open app launcher
- Search for "RoboShire"
- Click icon

**Method B**: Command Line
```bash
cd ~/roboshire
python3 -m roboshire
```

**Method C**: Desktop Icon (if created)
- Double-click RoboShire icon on desktop

### 3. Verify Installation

```bash
# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 --version
# Expected: ros2 cli version 0.xx.x

# Check Python dependencies
python3 -c "import PyQt5, serial, yaml, jinja2, lxml"
# Expected: No output (success)

# Check MuJoCo
python3 -c "import mujoco; print(mujoco.__version__)"
# Expected: 3.1.0

# Test RoboShire import
cd ~/roboshire
python3 -c "import roboshire; print('RoboShire OK')"
```

### 4. Run First Project

Follow the [Hardware Quick Start](../hardware/HARDWARE_QUICK_START.md):
```bash
cd ~/roboshire
python3 -m roboshire
# Tools → Workflow Wizard → Weather Monitor Station
```

---

## Troubleshooting Installation

### Issue 1: ROS 2 Repository Key Error

**Error**: `GPG error: ... NO_PUBKEY`

**Fix**:
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <KEY_ID>
sudo apt update
```

### Issue 2: Insufficient Disk Space

**Error**: `No space left on device`

**Check space**:
```bash
df -h /
```

**Free up space**:
```bash
# Clean apt cache
sudo apt clean
sudo apt autoremove

# Remove old kernels
sudo apt remove --purge $(dpkg -l 'linux-*' | sed '/^ii/!d;/'"$(uname -r | sed "s/\(.*\)-\([^0-9]\+\)/\1/")"'/d;s/^[^ ]* [^ ]* \([^ ]*\).*/\1/;/[0-9]/!d')
```

### Issue 3: Python Import Errors

**Error**: `ModuleNotFoundError: No module named 'PyQt5'`

**Fix**:
```bash
# Reinstall dependencies
pip3 install --user --force-reinstall PyQt5 pyserial pyyaml
```

### Issue 4: Serial Port Permission Denied

**Error**: `PermissionError: [Errno 13] Permission denied: '/dev/ttyUSB0'`

**Fix**:
```bash
# Add to dialout group
sudo usermod -aG dialout $USER

# Logout and login, then verify
groups | grep dialout

# Or temporary fix (reset on reboot)
sudo chmod 666 /dev/ttyUSB0
```

### Issue 5: MuJoCo Library Not Found

**Error**: `OSError: libmujoco.so.3.1.0: cannot open shared object file`

**Fix**:
```bash
# Add to LD_LIBRARY_PATH in ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco-3.1.0/lib' >> ~/.bashrc
source ~/.bashrc
```

---

## Manual Installation (Advanced)

If the automated installer fails, follow manual steps:

### 1. Install ROS 2 Manually

```bash
# Add repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 2. Install System Dependencies

```bash
sudo apt install -y \
    git wget curl build-essential cmake \
    python3-pip python3-venv \
    qtbase5-dev python3-pyqt5 \
    arduino arduino-core
```

### 3. Install Python Dependencies

```bash
pip3 install --user \
    PyQt5 pyserial pyyaml lxml jinja2 \
    psutil paramiko requests matplotlib numpy
```

### 4. Install MuJoCo

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.1.0/mujoco-3.1.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.1.0-linux-x86_64.tar.gz
pip3 install --user mujoco
```

### 5. Clone and Setup RoboShire

```bash
git clone https://github.com/YOUR_ORG/roboshire.git ~/roboshire
cd ~/roboshire
mkdir -p workspace/src
source /opt/ros/humble/setup.bash
cd workspace
colcon build
```

### 6. Configure Environment

```bash
cat >> ~/.bashrc << 'EOF'
export ROBOSHIRE_HOME="$HOME/roboshire"
source /opt/ros/humble/setup.bash
source $ROBOSHIRE_HOME/workspace/install/setup.bash
export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.1.0
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
alias roboshire="python3 $ROBOSHIRE_HOME/roboshire"
EOF

source ~/.bashrc
```

---

## Uninstallation

To remove RoboShire:

```bash
# Remove RoboShire directory
rm -rf ~/roboshire

# Remove desktop launcher
rm ~/.local/share/applications/roboshire.desktop

# Remove environment config from ~/.bashrc
# (manually edit ~/.bashrc and remove RoboShire section)

# Optional: Remove ROS 2 (if not needed for other projects)
sudo apt remove ros-humble-* python3-colcon-common-extensions
sudo apt autoremove

# Optional: Remove MuJoCo
rm -rf ~/.mujoco
```

---

## Updating RoboShire

```bash
cd ~/roboshire
git pull origin main

# Rebuild workspace if needed
cd workspace
colcon build --packages-select <changed_packages>
```

---

## Docker Installation (Alternative)

For containerized installation:

```bash
# Pull Docker image (when available)
docker pull roboshire/roboshire:2.3.0

# Run with X11 forwarding (GUI support)
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/roboshire_workspace:/workspace \
    --device=/dev/ttyUSB0 \
    roboshire/roboshire:2.3.0
```

---

## Getting Help

### Documentation
- **Quick Start**: [/docs/hardware/HARDWARE_QUICK_START.md](../hardware/HARDWARE_QUICK_START.md)
- **Troubleshooting**: [/docs/hardware/TROUBLESHOOTING_HARDWARE.md](../hardware/TROUBLESHOOTING_HARDWARE.md)
- **All Docs**: [/docs/hardware/README.md](../hardware/README.md)

### Support Channels
- **GitHub Issues**: https://github.com/YOUR_ORG/roboshire/issues
- **Discord**: https://discord.gg/roboshire
- **ROS Discourse**: https://discourse.ros.org/ (tag: roboshire)

### Installation Logs
Check logs if installation fails:
```bash
# Installation script output (if redirected)
cat ~/roboshire_install.log

# ROS 2 installation logs
cat /var/log/apt/term.log

# Python pip logs
cat ~/.pip/pip.log
```

---

## Next Steps

1. ✅ **Complete installation** (you are here!)
2. 📖 **Read Quick Start**: Build first project in 15 minutes
3. 🔧 **Connect hardware**: Weather station or robot
4. 🚀 **Build and deploy**: Create ROS2 package
5. 🎓 **Learn more**: Explore tutorials and examples

**Start here**: [Hardware Quick Start Guide](../hardware/HARDWARE_QUICK_START.md)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-10-29
**Tested On**: Ubuntu 22.04.3 LTS (Fresh install)
**Installation Script Version**: 2.3.0
