# Quick Reference - Cross-Platform Installation

## One-Liner Installation

### Windows 11/10 (Admin PowerShell)
```powershell
powershell -ExecutionPolicy Bypass -File scripts/install_windows.ps1
```

### macOS
```bash
curl -fsSL https://raw.githubusercontent.com/roboshire/roboshire/main/scripts/install_macos.sh | bash
```

### Ubuntu/Linux
```bash
curl -fsSL https://raw.githubusercontent.com/roboshire/roboshire/main/docs/installation/install_ubuntu.sh | bash
```

---

## Post-Installation Launch

### Windows
```powershell
# Desktop shortcut: RoboShire.lnk
# Command line:
wsl -d Ubuntu-22.04 ~/roboshire/roboshire-ubuntu.sh
# Or (after .bashrc sourced):
roboshire
```

### macOS
```bash
# Applications folder: ~/Applications/RoboShire.app
# Command line:
roboshire
# Direct Docker:
docker run -it roboshire:latest
```

### Ubuntu
```bash
# Command line:
roboshire
# Direct:
~/roboshire/roboshire-ubuntu.sh
```

---

## USB Device Setup

### Windows (Arduino/ESP32)
```powershell
# List devices
usbipd wsl list

# Attach to WSL
usbipd wsl attach --busid 2-1

# In Ubuntu, verify:
lsusb
ls /dev/ttyUSB*
```

### macOS
```bash
# List devices
ioreg -p IOUSB -l -w 0

# Mount in Docker (edit launcher):
docker run -v /dev:/dev --device=/dev/ttyUSB0 ...
```

### Ubuntu
```bash
# Direct access
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

---

## Advanced Performance Monitor

### Access
- **GUI**: Tools > Performance > Advanced Monitor
- **Standalone**:
  ```bash
  python3 -c "from roboshire.gui.performance_monitor_advanced import AdvancedPerformanceMonitor; ..."
  ```

### Key Features

| Tab | Purpose |
|-----|---------|
| Node Performance | CPU, memory, callbacks |
| Message Throughput | Topic rates, bandwidth |
| Latency Analysis | P50, P95, P99 percentiles |
| Memory Analysis | Leak detection, growth rate |
| Callback Timing | Execution times |
| DDS Discovery | Network overhead |

### Export
```bash
# Monitor exports to:
# - JSON (full metrics)
# - CSV (spreadsheet-friendly)

# Open File > Export Report in monitor
```

---

## Troubleshooting Quick Fixes

### Windows WSL2 Not Starting
```powershell
# Enable virtualization in BIOS first (VT-x or AMD-V)
wsl --update
wsl --shutdown
wsl
```

### macOS Docker Fails
```bash
# Restart Docker
killall Docker
open /Applications/Docker.app

# Clean up
docker system prune -a
```

### GUI Applications Not Showing
```bash
# Windows 11: Auto (WSLg)
# Windows 10: Start VcXsrv first, then launch
# macOS: Check XQuartz: killall Xquartz && open -a Xquartz
```

### No ROS2 Nodes/Topics
```bash
# Check ROS2 is running
ros2 node list
ros2 topic list

# Or use Demo Mode in performance monitor
```

---

## Configuration

### Monitor Update Interval
In performance monitor UI: Interval spinner (100-5000 ms)

### Memory Tracking
In performance monitor UI: "Track Memory" checkbox (tracemalloc)

### Windows Memory Allocation
Edit `%USERPROFILE%\.wslconfig`:
```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
```

### macOS Docker Resources
Docker > Preferences > Resources:
- Memory: 8-12 GB
- CPUs: 4+ cores

---

## Performance Tips

### Real-Time Systems
1. Use BEST_EFFORT QoS
2. Keep callbacks < 10ms
3. Monitor latency P95 < 50ms
4. Check callback time variance

### Memory Optimization
1. Monitor growth rate
2. Alert threshold: > 1 MB/min
3. Export reports regularly
4. Profile with tracemalloc

### Network Tuning
1. Check DDS discovery rate
2. Reduce message size
3. Lower publication frequency
4. Batch messages when possible

---

## File Locations

| Component | Location |
|-----------|----------|
| Windows installer | `/scripts/install_windows.ps1` |
| macOS installer | `/scripts/install_macos.sh` |
| Ubuntu installer | `/docs/installation/install_ubuntu.sh` |
| Performance monitor | `/roboshire/gui/performance_monitor_advanced.py` |
| Installation guide | `/docs/installation/CROSS_PLATFORM_INSTALLATION.md` |
| Monitor guide | `/docs/guides/ADVANCED_PERFORMANCE_MONITORING.md` |
| Full summary | `/CROSS_PLATFORM_INSTALLATION_COMPLETE.md` |

---

## Key Metrics Explained

### CPU %
Processor utilization. Alert if > 80%

### Memory MB
RAM usage. Alert if > 500 MB

### Growth Rate (MB/min)
Memory leak detector. Alert if > 1.0

### Latency P95 (ms)
95% of messages arrive faster. Target < 100ms

### Callback Time (ms)
Function execution duration. Target < 10ms

### DDS Discovery (discoveries/min)
Network overhead. Typical 0.5-2 per minute

---

## Common Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list -t

# Get node info
ros2 node info /node_name

# Echo topic
ros2 topic echo /topic_name

# ROS2 doctor check
ros2 doctor

# ROS2 environment
printenv | grep ROS_

# Activate environment (Ubuntu)
source ~/.bashrc
```

---

## Getting Help

1. **Check docs**: `/docs/installation/CROSS_PLATFORM_INSTALLATION.md`
2. **Check troubleshooting**: Same document, scroll down
3. **GitHub Issues**: https://github.com/roboshire/roboshire/issues
4. **Discord**: https://discord.gg/roboshire

---

## Version Info

- **RoboShire**: v2.3.0
- **ROS2**: Humble (Ubuntu 22.04)
- **Python**: 3.10+
- **Docker**: Latest
- **WSL2**: Latest

---

## Checklist: Installation Complete?

- [ ] Installer ran without errors
- [ ] RoboShire launches successfully
- [ ] Can list nodes: `ros2 node list`
- [ ] Can list topics: `ros2 topic list`
- [ ] Performance monitor opens (Tools menu)
- [ ] Can enable Demo Mode in monitor
- [ ] Export JSON/CSV works
- [ ] Desktop shortcut working (Windows/macOS)

---

**Last Updated**: October 30, 2024 | **Version**: 2.3.0
