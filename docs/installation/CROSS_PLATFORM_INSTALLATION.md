# Cross-Platform RoboShire Installation Guide

Version 2.3.0

This guide covers installation of RoboShire on **Windows**, **macOS**, and **Linux** systems.

## Quick Start

### Windows 11/10 (WSL2)

```powershell
# Run as Administrator in PowerShell
powershell -ExecutionPolicy Bypass -File install_windows.ps1
```

### macOS (Docker)

```bash
curl -fsSL https://raw.githubusercontent.com/roboshire/roboshire/main/scripts/install_macos.sh | bash
# OR
bash ./scripts/install_macos.sh
```

### Ubuntu/Linux

```bash
curl -fsSL https://raw.githubusercontent.com/roboshire/roboshire/main/docs/installation/install_ubuntu.sh | bash
# OR
bash ./docs/installation/install_ubuntu.sh
```

---

## Windows 11/10 Installation (WSL2)

RoboShire on Windows runs in a lightweight Ubuntu Linux environment via WSL2 (Windows Subsystem for Linux), providing full ROS2 compatibility without virtualizing the entire OS.

### System Requirements

- **OS**: Windows 11 (22000+) or Windows 10 21H2 (19045+)
- **RAM**: 8 GB minimum, 16 GB recommended
- **Disk Space**: 15 GB free (for Windows + Ubuntu + RoboShire)
- **Processor**: Intel VT-x or AMD-V virtualization support (enabled in BIOS)
- **Administrator Access**: Required for first-time setup

### What the Installer Does

1. **Checks Windows Version** - Ensures Windows 11 or Windows 10 21H2+
2. **Enables WSL2** - Activates Virtual Machine Platform and Windows Subsystem for Linux
3. **Installs Ubuntu 22.04** - Downloads and installs Ubuntu LTS from Microsoft Store
4. **Configures X11 Forwarding**:
   - WSLg (built-in for Windows 11)
   - VcXsrv (optional for Windows 10)
5. **Sets up USB Passthrough** - Installs usbipd-win for Arduino/ESP32 access
6. **Creates Shortcuts** - Desktop and command-line access
7. **Installs RoboShire** - Full setup in Ubuntu environment

### Installation Steps

#### 1. Download Installer

Download `install_windows.ps1` from the scripts folder or fetch directly:

```powershell
$ProgressPreference = 'SilentlyContinue'
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/roboshire/roboshire/main/scripts/install_windows.ps1" -OutFile "install_windows.ps1"
```

#### 2. Run as Administrator

```powershell
# Right-click PowerShell > Run as Administrator
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process -Force
.\install_windows.ps1
```

#### 3. Follow Prompts

The installer will:
- Check system requirements
- Enable WSL2 (may require restart)
- Open Microsoft Store for Ubuntu installation
- Configure X11 and USB access
- Install RoboShire in Ubuntu

### Post-Installation

#### Launch RoboShire

Choose one method:

1. **Desktop Shortcut** - Double-click `RoboShire.lnk` on desktop
2. **Command Line**:
   ```powershell
   wsl -d Ubuntu-22.04 ~/roboshire/roboshire-ubuntu.sh
   ```
3. **From Windows Terminal**:
   ```bash
   roboshire
   ```

#### USB Device Setup (Arduino/ESP32)

```powershell
# List USB devices
usbipd wsl list

# Attach device (replace BUS-ID)
usbipd wsl attach --busid 2-1

# In Ubuntu, verify:
lsusb
ls /dev/ttyUSB*
```

### Troubleshooting

#### WSL2 Won't Start

**Problem**: "The Virtual Machine Platform is not available"

**Solutions**:
1. Enable virtualization in BIOS (VT-x for Intel, AMD-V for AMD)
2. Update Windows to latest version
3. Update WSL2:
   ```powershell
   wsl --update
   ```

#### Ubuntu Installation Fails

**Problem**: Microsoft Store download fails or hangs

**Solutions**:
1. Restart PowerShell and try again
2. Use alternative installation:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```
3. Install from Store manually:
   - Search "Ubuntu 22.04" in Microsoft Store
   - Click "Get" or "Install"

#### GUI Applications Won't Launch

**Windows 11**:
- WSLg should work automatically
- If issues: `wsl --update`

**Windows 10**:
1. Install VcXsrv when prompted
2. Before launching GUI apps:
   ```powershell
   # Start VcXsrv with default settings
   # Then launch RoboShire
   ```

#### Network Issues in WSL2

**Problem**: Can't ping hosts from Ubuntu

**Solutions**:
1. Check Windows firewall:
   ```powershell
   Get-NetFirewallProfile | Format-Table
   ```
2. Reset network:
   ```powershell
   wsl --shutdown
   wsl
   ```

#### Arduino/ESP32 Not Appearing in /dev/ttyUSB*

**Problem**: Device not visible in Ubuntu

**Solutions**:
1. Verify device is recognized in Windows:
   ```powershell
   usbipd wsl list
   ```
2. Detach and reattach:
   ```powershell
   usbipd wsl detach --busid 2-1
   usbipd wsl attach --busid 2-1
   ```
3. Install USB drivers in Windows if needed

---

## macOS Installation (Docker)

RoboShire on macOS runs in a Docker container with full ROS2 support, X11 GUI forwarding, and USB device access.

### System Requirements

- **OS**: macOS 10.15 Catalina or newer
- **Processor**: Intel or Apple Silicon (M1/M2/M3)
- **RAM**: 8 GB minimum, 16 GB recommended
- **Disk Space**: 20 GB free
- **Administrator Access**: For Docker Desktop installation

### What the Installer Does

1. **Checks macOS Version** - Ensures 10.15+
2. **Installs Homebrew** - Package manager (if needed)
3. **Installs Docker Desktop** - Container runtime
4. **Sets up XQuartz** - X11 server for GUI applications
5. **Creates Docker Image** - ROS2 + RoboShire + dependencies
6. **Creates Launcher** - Application entry point
7. **Configures USB Access** - For Arduino/ESP32 devices

### Installation Steps

#### 1. Download and Run Installer

```bash
# Download
curl -fsSL https://raw.githubusercontent.com/roboshire/roboshire/main/scripts/install_macos.sh -o install_macos.sh
chmod +x install_macos.sh

# Run
./install_macos.sh
```

#### 2. Follow Prompts

The installer will:
- Check system requirements
- Install Homebrew (if needed)
- Install Docker Desktop
- Install XQuartz
- Build RoboShire Docker image
- Create application launcher

#### 3. Start Docker Desktop

Docker Desktop may open automatically. If not:
```bash
open /Applications/Docker.app
```

Wait for Docker to fully start (icon in menu bar shows "connected").

### Post-Installation

#### Launch RoboShire

Choose one method:

1. **Applications Folder**:
   - Go to: `~/Applications/RoboShire.app`
   - Double-click to launch

2. **Command Line**:
   ```bash
   roboshire
   ```

3. **Direct Docker Command**:
   ```bash
   docker run -it roboshire:latest
   ```

#### Workspace

ROS2 workspace is at `~/roboshire_workspace`:

```bash
# Build packages
cd ~/roboshire_workspace
colcon build

# Source workspace
source install/setup.bash
```

#### USB Device Setup (Arduino/ESP32)

List devices:
```bash
ioreg -p IOUSB -l -w 0
```

Mount device in Docker (update launcher):
```bash
docker run -v /dev:/dev --device=/dev/ttyUSB0 -it roboshire:latest
```

### Troubleshooting

#### Docker Desktop Won't Start

**Problem**: Installation or startup fails

**Solutions**:
1. Check system requirements (Apple Silicon vs Intel)
2. Uninstall and reinstall:
   ```bash
   brew uninstall --cask docker
   brew install --cask docker
   ```
3. Check logs:
   ```bash
   cat ~/.docker/daemon.json
   ```

#### XQuartz GUI Not Working

**Problem**: Windows appear black or don't display

**Solutions**:
1. Restart XQuartz:
   ```bash
   killall Xquartz
   open -a Xquartz
   ```
2. Update macOS and Docker Desktop to latest versions
3. Enable hardware acceleration in Docker preferences

#### Docker Image Build Fails

**Problem**: "Failed to build image"

**Solutions**:
1. Check Docker is running:
   ```bash
   docker ps
   ```
2. Clean up Docker:
   ```bash
   docker system prune
   ```
3. Rebuild image:
   ```bash
   docker build -t roboshire:latest .
   ```

#### USB Device Not Accessible

**Problem**: "Permission denied" when accessing /dev/ttyUSB0

**Solutions**:
1. List available devices:
   ```bash
   ls -la /dev/tty.usb*
   ```
2. Use serial number in mount:
   ```bash
   docker run --device=/dev/tty.usbserial-XXXXX ...
   ```
3. Run container with privilege escalation:
   ```bash
   docker run --privileged -it roboshire:latest
   ```

#### Disk Space Issues

**Problem**: Docker fills disk with images/containers

**Solutions**:
```bash
# Clean unused images
docker image prune -a

# Clean containers
docker container prune

# Full cleanup (WARNING: removes all unused Docker data)
docker system prune -a
```

---

## Ubuntu/Linux Installation

See: [Ubuntu Installation Guide](UBUNTU_INSTALLATION.md)

```bash
bash ./docs/installation/install_ubuntu.sh
```

---

## Feature Comparison

| Feature | Windows WSL2 | macOS Docker | Ubuntu |
|---------|-------------|--------------|--------|
| Native ROS2 | Emulated in WSL | Containerized | Native |
| Performance | Excellent | Very Good | Best |
| GUI Support | WSLg + VcXsrv | XQuartz | Direct |
| USB Access | usbipd-win | Docker mount | Direct |
| File Speed | Fast | Good | Best |
| Startup Time | ~1 sec | ~2 sec | ~1 sec |
| Resource Usage | Lightweight | Medium | Lightweight |
| Hardware Access | Via WSL | Via Docker | Direct |

---

## Advanced Configuration

### Windows WSL2

#### Increase Memory Allocation

Edit `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```

#### File Sharing Performance

For faster file access with Windows projects:

```bash
# Use /mnt/c/ location (auto-mounted Windows drive)
# Or enable WSL socket for Docker:
# Settings > Resources > WSL Integration
```

### macOS Docker

#### Increase Memory/CPU

Docker > Preferences > Resources:
- Memory: 8-12 GB recommended
- CPUs: 4+ cores recommended

#### Enable Kubernetes (Optional)

Docker > Preferences > Kubernetes:
- Enable Kubernetes
- Useful for testing deployment

### Linux Performance Tuning

#### Enable Real-Time Priority (Jetson/RPi)

```bash
# Add user to realtime group
sudo usermod -a -G realtime $(whoami)

# Set scheduling priorities
ulimit -r 99
```

---

## Next Steps

After installation:

1. **Launch RoboShire** - Use appropriate launcher for your platform
2. **Execution Mode** - Choose Local (development) or Remote (Jetson/RPi)
3. **Create Workspace** - Build your first ROS2 project
4. **Check Examples** - Browse example packages in the app
5. **Connect Devices** - Set up Arduino/ESP32 serial access
6. **Consult Documentation** - Full guides at https://github.com/roboshire/roboshire

---

## Platform-Specific Tips

### Windows Users

- **Path Separators**: Use `/` in WSL2 (Unix-style)
- **Windows Interop**: Access Windows files via `/mnt/c/Users/YourName/`
- **Clipboard**: Copy/paste works between Windows and WSL2
- **Performance**: Store large projects in Ubuntu filesystem (`/home/roboshire/`) for speed

### macOS Users

- **Dock Icon**: Right-click Docker icon > Show/Hide for Kubernetes
- **Resource Limits**: Monitor Docker stats via Activity Monitor
- **M1/M2 Support**: Docker Desktop now fully supports Apple Silicon
- **Homebrew**: Separate installations for Intel vs Apple Silicon

### Linux Users

- **Package Manager**: Use native `apt` for additional packages
- **Permissions**: May need `sudo` for hardware access
- **Container Engine**: Can use Podman instead of Docker if preferred
- **X11 Forwarding**: SSH to remote machines with `-X` flag

---

## Support

For issues:

1. Check troubleshooting sections above
2. Review logs:
   - Windows WSL2: `wsl --status`
   - macOS: `docker logs roboshire`
   - Ubuntu: `~/.roboshire/logs/`
3. Open issue: https://github.com/roboshire/roboshire/issues
4. Community Discord: https://discord.gg/roboshire

---

## See Also

- [Advanced Performance Monitor Guide](../guides/PERFORMANCE_MONITORING.md)
- [USB Device Setup](../hardware/USB_DEVICE_SETUP.md)
- [Remote Deployment Guide](../guides/JETSON_RPI_DEPLOYMENT.md)
