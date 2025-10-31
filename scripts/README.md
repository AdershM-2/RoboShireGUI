# RoboShireGUI Utility Scripts

Essential utility scripts for RoboShireGUI deployment and maintenance.

---

## 📁 Available Scripts

### 🧹 `cleanup_all_ros2.sh`

**Purpose**: Complete ROS2 workspace cleanup - kills all nodes and removes build artifacts

**Usage** (on Ubuntu):
```bash
cd RoboShireGUI/scripts
./cleanup_all_ros2.sh
```

**What it does**:
1. Kills all running ROS2 nodes
2. Kills ROS2 daemon and DDS processes
3. Deletes `workspace/build/`, `install/`, `log/`
4. Removes Python cache files
5. Cleans temporary log files

**When to use**:
- Before regenerating code
- When experiencing node conflicts
- For a fresh start

---

### 🗑️ `cleanup_workspace.sh`

**Purpose**: Clean ROS2 workspace build artifacts only

**Usage** (on Ubuntu):
```bash
cd RoboShireGUI/scripts
./cleanup_workspace.sh
```

**What it deletes**:
- `workspace/build/`
- `workspace/install/`
- `workspace/log/`

---

### 🔐 `setup_ubuntu_ssh.sh`

**Purpose**: Configure SSH server for remote deployment (Jetson/RPi)

**Usage** (on remote device):
```bash
cd RoboShireGUI/scripts
./setup_ubuntu_ssh.sh
```

**What it does**:
- Installs OpenSSH server
- Configures SSH for key-based authentication
- Sets up authorized_keys
- Restarts SSH service

**When to use**:
- Setting up Jetson/Raspberry Pi for remote deployment
- Configuring Ubuntu server for SSH access

---

## 🚀 Quick Reference

| Script | Purpose | Platform |
|--------|---------|----------|
| `cleanup_all_ros2.sh` | Full ROS2 cleanup | Ubuntu |
| `cleanup_workspace.sh` | Build artifacts only | Ubuntu |
| `setup_ubuntu_ssh.sh` | Configure SSH server | Ubuntu/Jetson/RPi |

---

## 💡 Tips

### Making Scripts Executable

```bash
chmod +x *.sh
```

### Running Scripts

Always run from the scripts directory or use full path:

```bash
# From scripts directory
cd RoboShireGUI/scripts
./cleanup_all_ros2.sh

# Or use full path
/path/to/RoboShireGUI/scripts/cleanup_all_ros2.sh
```

---

## 📚 Related Documentation

- [Installation Guide](../docs/installation/UBUNTU_INSTALLATION_GUIDE.md)
- [Deployment Guide](../docs/guides/JETSON_RPI_DEPLOYMENT.md)
- [Troubleshooting](../docs/troubleshooting/COMMON_ISSUES.md)

---

**RoboShireGUI Version**: 2.5.0
