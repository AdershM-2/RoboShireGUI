# Integration Notes - Cross-Platform Installation & Performance Monitor

## Code Integration Points

### 1. Performance Monitor Integration with Main Window

**Location**: `roboshire/gui/main_window.py`

Add to Tools menu:

```python
from roboshire.gui.performance_monitor_advanced import AdvancedPerformanceMonitor

class MainWindow(QMainWindow):
    def __init__(self):
        # ... existing code ...

        # Add to Tools menu
        tools_menu = self.menuBar().addMenu("Tools")

        performance_action = QAction("Advanced Performance Monitor", self)
        performance_action.triggered.connect(self.show_advanced_monitor)
        tools_menu.addAction(performance_action)

    def show_advanced_monitor(self):
        """Open advanced performance monitor"""
        if not hasattr(self, 'advanced_monitor'):
            self.advanced_monitor = AdvancedPerformanceMonitor(
                ssh_bridge=self.ssh_bridge
            )
        self.advanced_monitor.show()
        self.advanced_monitor.raise_()
        self.advanced_monitor.activateWindow()
```

### 2. SSH Bridge Integration

The performance monitor already accepts an `ssh_bridge` parameter:

```python
from roboshire.integrations.ros2_launcher import SSHBridge

# In main window
ssh_bridge = SSHBridge(host, port, username, password)
monitor = AdvancedPerformanceMonitor(ssh_bridge=ssh_bridge)
```

### 3. Demo Mode Auto-Enable

Detect when ROS2 is unavailable and auto-enable demo mode:

```python
def on_startup(self):
    monitor = AdvancedPerformanceMonitor()

    # Check if ROS2 available
    if not self.check_ros2_available():
        print("ROS2 not available, enabling demo mode...")
        monitor.mode_btn.setChecked(True)
        monitor._toggle_demo_mode(True)
```

### 4. Settings Persistence

Add to RoboShire settings:

```yaml
# ~/.roboshire/config.yaml
performance_monitor:
  demo_mode: false
  update_interval_ms: 1000
  history_size: 300

  memory_tracking: false
  tracemalloc_frames: 10

  # Alert thresholds
  cpu_threshold: 80
  memory_threshold: 500
  growth_threshold: 1.0
  latency_threshold: 100

  # Export
  auto_export: false
  export_interval_sec: 300
  export_path: ~/.roboshire/performance_reports/
```

### 5. Tab Widget Integration

If performance monitor should be embedded as a tab in main window:

```python
class MainWindow(QMainWindow):
    def __init__(self):
        # ... existing tabs ...

        # Add performance tab
        from roboshire.gui.performance_monitor_advanced import AdvancedPerformanceMonitor
        self.perf_monitor = AdvancedPerformanceMonitor()
        self.main_tabs.addTab(self.perf_monitor, "Performance")
```

---

## Cross-Platform Installer Integration

### 1. GitHub Actions Workflow

For automated testing (`.github/workflows/cross-platform-test.yml`):

```yaml
name: Test Cross-Platform Installers

on: [push, pull_request]

jobs:
  test-windows:
    runs-on: windows-latest
    steps:
      - uses: actions/checkout@v3
      - name: Test Windows installer syntax
        run: |
          powershell -NoProfile -Command "
            . { param($Path) [ScriptBlock]::Create([IO.File]::ReadAllText($Path)).Invoke()
            } -Path 'scripts/install_windows.ps1' -Verbose
          "

  test-macos:
    runs-on: macos-latest
    steps:
      - uses: actions/checkout@v3
      - name: Test macOS installer syntax
        run: bash -n scripts/install_macos.sh

  test-linux:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Test Ubuntu installer syntax
        run: bash -n docs/installation/install_ubuntu.sh
```

### 2. CI/CD Performance Testing

Automated performance regression detection:

```python
# tests/test_performance_baseline.py
import json
from roboshire.gui.performance_monitor_advanced import AdvancedPerformanceMonitor

def test_performance_baseline():
    """Test that performance hasn't regressed"""
    monitor = AdvancedPerformanceMonitor()

    # Load baseline
    with open('tests/performance_baseline.json') as f:
        baseline = json.load(f)

    # Run current system and capture
    monitor._toggle_demo_mode(True)
    monitor._toggle_monitoring(True)
    time.sleep(60)  # Run for 1 minute

    # Compare metrics
    for node_name in baseline['nodes']:
        base_memory = baseline['nodes'][node_name]['current_memory']
        current_memory = monitor.nodes[node_name].current_memory

        # Allow 10% regression
        assert current_memory <= base_memory * 1.1, \
            f"Memory regression in {node_name}"
```

### 3. Distribution Package Integration

For PyPI/package managers:

```bash
# setup.py
setup(
    name='roboshire',
    version='2.3.0',
    scripts=['scripts/install_windows.ps1', 'scripts/install_macos.sh'],
    entry_points={
        'console_scripts': [
            'roboshire=roboshire.main:main',
            'roboshire-monitor=roboshire.gui.performance_monitor_advanced:main',
        ],
    },
    install_requires=[
        'pyside6>=6.5.0',
        'pyqtgraph>=0.13.0',
        'numpy>=1.24.0',
        # ... other deps ...
    ],
)
```

---

## Documentation Integration

### 1. Main README Update

Add to root README.md:

```markdown
## Quick Installation

### Windows 11/10 (WSL2)
```powershell
powershell -ExecutionPolicy Bypass -File scripts/install_windows.ps1
```

### macOS (Docker)
```bash
bash scripts/install_macos.sh
```

### Ubuntu/Linux
```bash
bash docs/installation/install_ubuntu.sh
```

See [Cross-Platform Installation Guide](docs/installation/CROSS_PLATFORM_INSTALLATION.md) for details.

## Performance Monitoring

Access **Tools > Advanced Performance Monitor** for:
- Real-time CPU/memory graphs
- Memory leak detection
- Latency analysis (P50, P95, P99)
- Callback timing profiler
- DDS metrics
- JSON/CSV export

See [Advanced Performance Monitoring Guide](docs/guides/ADVANCED_PERFORMANCE_MONITORING.md).
```

### 2. Troubleshooting Wiki

Add to docs/troubleshooting/:

```markdown
# Platform-Specific Issues

## Windows WSL2
- [WSL2 won't start](WINDOWS.md#wsl2-wont-start)
- [GUI applications don't appear](WINDOWS.md#gui-applications-dont-appear)
- [USB device access](WINDOWS.md#usb-device-access)

## macOS
- [Docker Desktop fails](MACOS.md#docker-desktop-fails)
- [XQuartz GUI issues](MACOS.md#xquartz-gui-issues)
- [Performance monitor slow](MACOS.md#performance-monitor-slow)

## Ubuntu
- [ROS2 installation issues](UBUNTU.md#ros2-installation)
- [Permission denied errors](UBUNTU.md#permission-denied)
```

---

## Performance Testing

### 1. Baseline Establishment

Create initial performance baseline:

```bash
#!/bin/bash
# scripts/establish_baseline.sh

echo "Establishing performance baseline..."

python3 << 'EOF'
import json
from roboshire.gui.performance_monitor_advanced import AdvancedPerformanceMonitor

monitor = AdvancedPerformanceMonitor()
monitor._toggle_demo_mode(True)
monitor._toggle_monitoring(True)

# Run for 2 minutes
import time
time.sleep(120)

# Export baseline
report = {
    'timestamp': __import__('datetime').datetime.now().isoformat(),
    'nodes': {}
}

for node_name, node in monitor.nodes.items():
    report['nodes'][node_name] = {
        'cpu': node.current_cpu,
        'memory': node.current_memory,
        'callback_time': node.get_average_callback_time(),
    }

with open('tests/performance_baseline.json', 'w') as f:
    json.dump(report, f, indent=2)

print("Baseline saved to tests/performance_baseline.json")
EOF
```

### 2. Regression Detection

```python
# tests/test_performance_regression.py
import json
import subprocess

def test_no_performance_regression():
    """Ensure no performance regression"""

    # Load baseline
    with open('tests/performance_baseline.json') as f:
        baseline = json.load(f)

    # Run performance monitor
    result = subprocess.run([
        'python3', '-m', 'pytest',
        'tests/perf_test.py', '--benchmark'
    ])

    # Compare with baseline
    # Alert if regression > 10%
```

---

## Version Management

### 1. Versioning Strategy

Keep versions aligned:

```
VERSION = "2.3.0"  # roboshire/__init__.py

# Windows installer (line 7)
# Version: 2.3.0

# macOS installer (line 7)
# Version: 2.3.0

# Performance monitor (line 10)
# Version: 2.3.0

# Documentation (top of each file)
# Version: 2.3.0
```

### 2. Update Script

```bash
#!/bin/bash
# scripts/update_version.sh
NEW_VERSION=$1

echo "Updating version to $NEW_VERSION..."

sed -i "s/VERSION = \".*\"/VERSION = \"$NEW_VERSION\"/" roboshire/__init__.py
sed -i "s/Version: .*/Version: $NEW_VERSION/" scripts/install_windows.ps1
sed -i "s/Version: .*/Version: $NEW_VERSION/" scripts/install_macos.sh
sed -i "s/Version: .*/Version: $NEW_VERSION/" roboshire/gui/performance_monitor_advanced.py
sed -i "s/Version 2\..*/Version $NEW_VERSION/" docs/**/*.md

echo "Version updated to $NEW_VERSION"
```

---

## Future Enhancements

### 1. Real-Time Alerting

```python
class PerformanceAlerter:
    def __init__(self, monitor):
        self.monitor = monitor
        self.monitor.alert_triggered.connect(self.on_alert)

    def on_alert(self, node_name, message):
        # Send to monitoring service
        # Slack/Teams integration
        # Email notification
        pass
```

### 2. Historical Analytics

```python
class PerformanceHistory:
    def save_report(self, monitor):
        """Save timestamped report"""
        timestamp = datetime.now().isoformat()
        path = f"~/.roboshire/performance_reports/{timestamp}.json"
        self._export_json(monitor, path)

    def analyze_trends(self, days=7):
        """Analyze performance over time"""
        # Load all reports from last N days
        # Calculate trends
        # Predict issues
```

### 3. Remote Dashboard

```python
# Future: Web-based dashboard
# Use FastAPI to expose metrics
# Real-time WebSocket updates
# Grafana integration
```

### 4. Machine Learning Anomaly Detection

```python
from sklearn.ensemble import IsolationForest

class AnomalyDetector:
    def detect_anomalies(self, monitor):
        """Detect unusual performance patterns"""
        # Train on historical data
        # Detect outliers in real-time
        # Alert on anomalies
```

---

## Support and Maintenance

### 1. Regular Updates

- **Monthly**: Update base Docker image (macOS)
- **Quarterly**: Test with new ROS2 distributions
- **Yearly**: Major version refresh

### 2. Testing Checklist

Before each release:

- [ ] Windows WSL2 installation (Win11 and Win10)
- [ ] macOS installation (Intel and Apple Silicon)
- [ ] Ubuntu installation
- [ ] Performance monitor demo mode
- [ ] Performance monitor runtime mode
- [ ] Export JSON/CSV
- [ ] Memory leak detection
- [ ] Latency tracking
- [ ] Cross-platform documentation

### 3. Support Issues

Common issues to track:

1. Windows WSL2 virtualization disabled
2. macOS Docker memory limits
3. Ubuntu ROS2 not installed
4. GUI forwarding issues
5. USB device permissions
6. Performance monitor pyqtgraph missing

---

## Architecture Diagram

```
RoboShire 2.3.0
├── Installation Layer
│   ├── Windows: install_windows.ps1 → WSL2 → Ubuntu
│   ├── macOS: install_macos.sh → Docker → ROS2
│   └── Linux: install_ubuntu.sh → Native ROS2
│
├── GUI Layer
│   ├── Main Window
│   │   └── Tools Menu
│   │       └── Advanced Performance Monitor
│   │
│   └── Advanced Performance Monitor
│       ├── 6 Analysis Tabs
│       ├── Real-time Graphs (pyqtgraph)
│       ├── Data Models (NodePerformanceData)
│       └── Export System (JSON/CSV)
│
├── Backend Layer
│   ├── SSH Bridge (for remote systems)
│   ├── ROS2 Integration
│   │   ├── Node discovery
│   │   ├── Topic monitoring
│   │   └── DDS introspection
│   │
│   └── Analytics Engine
│       ├── Memory leak detection
│       ├── Latency percentiles
│       └── Callback profiling
│
└── Storage Layer
    ├── Performance baseline
    ├── Historical reports
    └── Configuration (YAML)
```

---

## Checklist: Ready for Production?

- [ ] All installers tested on target platforms
- [ ] Documentation complete and accurate
- [ ] Performance monitor integrated into main window
- [ ] Demo mode functional without ROS2
- [ ] Export functionality working
- [ ] Error handling comprehensive
- [ ] No crashes under load
- [ ] Cross-platform shortcuts working
- [ ] USB device setup documented
- [ ] Troubleshooting guide complete

---

## Quick Integration Checklist

For developers integrating these components:

1. **Copy Files**:
   - [ ] Copy `scripts/install_*.ps1` and `scripts/install_*.sh`
   - [ ] Copy `roboshire/gui/performance_monitor_advanced.py`
   - [ ] Copy documentation to `docs/`

2. **Update Main Window**:
   - [ ] Import `AdvancedPerformanceMonitor`
   - [ ] Add to Tools menu
   - [ ] Pass `ssh_bridge` parameter

3. **Update Requirements**:
   - [ ] Ensure `pyqtgraph>=0.13.0` in `requirements.txt`
   - [ ] Mark as optional for Qt Charts fallback

4. **Test**:
   - [ ] Run performance monitor (Tools menu)
   - [ ] Test Demo Mode
   - [ ] Export JSON/CSV
   - [ ] Check for errors in console

5. **Document**:
   - [ ] Update README with installation links
   - [ ] Add performance monitor to features list
   - [ ] Link to guides from main docs

---

**Last Updated**: October 30, 2024
**Integration Version**: 2.3.0
