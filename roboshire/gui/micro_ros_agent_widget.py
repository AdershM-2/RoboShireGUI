"""
micro-ROS Agent Manager Widget

Provides GUI for managing micro-ROS agent connections to embedded devices
(Arduino, ESP32, etc.) without terminal commands.
"""

import subprocess
import threading
import time
from pathlib import Path
from typing import Optional, List, Dict
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QTextEdit, QGroupBox, QRadioButton, QLineEdit,
    QCheckBox, QMessageBox, QFileDialog
)
from PySide6.QtCore import Qt, Signal, QTimer, Slot
from PySide6.QtGui import QFont
import logging
import json

try:
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class AgentManager:
    """Manages micro-ROS agent subprocess"""

    def __init__(self):
        self.process: Optional[subprocess.Popen] = None
        self.logger = logging.getLogger(__name__)

    def start_serial_agent(self, port: str, baud_rate: int = 115200) -> bool:
        """Start micro-ROS agent for serial connection"""
        try:
            cmd = [
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', port, '-b', str(baud_rate)
            ]

            self.logger.info(f"Starting micro-ROS agent: {' '.join(cmd)}")

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )

            return True

        except Exception as e:
            self.logger.error(f"Failed to start serial agent: {e}")
            return False

    def start_udp_agent(self, port: int = 8888) -> bool:
        """Start micro-ROS agent for UDP connection"""
        try:
            cmd = [
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'udp4', '--port', str(port)
            ]

            self.logger.info(f"Starting micro-ROS agent: {' '.join(cmd)}")

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )

            return True

        except Exception as e:
            self.logger.error(f"Failed to start UDP agent: {e}")
            return False

    def stop_agent(self) -> bool:
        """Stop micro-ROS agent"""
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
                self.logger.info("micro-ROS agent stopped")
                return True
            except subprocess.TimeoutExpired:
                self.logger.warning("Agent did not stop gracefully, killing...")
                self.process.kill()
                self.process.wait()
                return True
            except Exception as e:
                self.logger.error(f"Failed to stop agent: {e}")
                return False
        return True

    def is_running(self) -> bool:
        """Check if agent is running"""
        return self.process is not None and self.process.poll() is None

    def read_output(self) -> Optional[str]:
        """Read agent output (non-blocking)"""
        if self.process and self.process.stdout:
            try:
                return self.process.stdout.readline()
            except:
                return None
        return None


class MicroROSAgentWidget(QWidget):
    """
    GUI widget for managing micro-ROS agent

    Features:
    - Auto-detect serial ports (Arduino, ESP32)
    - Start/Stop agent button
    - Connection status indicator
    - Agent log viewer
    - WiFi/UDP agent support
    - Save/load configurations
    - Auto-reconnect on disconnect
    """

    # Signals
    status_changed = Signal(bool, str)  # (connected, status_text)
    log_message = Signal(str)  # log message

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)
        self.agent_manager = AgentManager()
        self.auto_reconnect = False
        self.last_config = {}

        # Timer for monitoring agent output
        self.output_timer = QTimer()
        self.output_timer.timeout.connect(self._read_agent_output)

        # Timer for connection monitoring
        self.monitor_timer = QTimer()
        self.monitor_timer.timeout.connect(self._monitor_connection)

        self._setup_ui()
        self._load_config()

        # Initial port refresh
        if SERIAL_AVAILABLE:
            self._refresh_ports()

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("micro-ROS Agent Manager")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Connection Type Selection
        conn_group = QGroupBox("Connection Type")
        conn_layout = QHBoxLayout()

        self.serial_radio = QRadioButton("Serial (USB)")
        self.serial_radio.setChecked(True)
        self.serial_radio.toggled.connect(self._on_connection_type_changed)

        self.udp_radio = QRadioButton("WiFi (UDP)")
        self.udp_radio.toggled.connect(self._on_connection_type_changed)

        conn_layout.addWidget(self.serial_radio)
        conn_layout.addWidget(self.udp_radio)
        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        # Serial Configuration
        self.serial_group = QGroupBox("Serial Configuration")
        serial_layout = QVBoxLayout()

        # Port selection
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Serial Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        port_layout.addWidget(self.port_combo, 1)

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        port_layout.addWidget(self.refresh_btn)
        serial_layout.addLayout(port_layout)

        # Baud rate
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '115200', '230400', '460800', '921600'])
        self.baud_combo.setCurrentText('115200')
        baud_layout.addWidget(self.baud_combo)
        baud_layout.addStretch()
        serial_layout.addLayout(baud_layout)

        self.serial_group.setLayout(serial_layout)
        layout.addWidget(self.serial_group)

        # UDP Configuration
        self.udp_group = QGroupBox("WiFi/UDP Configuration")
        udp_layout = QHBoxLayout()
        udp_layout.addWidget(QLabel("UDP Port:"))
        self.udp_port_edit = QLineEdit("8888")
        self.udp_port_edit.setMaximumWidth(100)
        udp_layout.addWidget(self.udp_port_edit)
        udp_layout.addStretch()
        self.udp_group.setLayout(udp_layout)
        self.udp_group.setVisible(False)
        layout.addWidget(self.udp_group)

        # Status and Control
        control_layout = QHBoxLayout()

        self.status_label = QLabel("Status:")
        control_layout.addWidget(self.status_label)

        self.status_indicator = QLabel("● Disconnected")
        self.status_indicator.setStyleSheet("QLabel { color: red; font-weight: bold; }")
        control_layout.addWidget(self.status_indicator)

        control_layout.addStretch()

        self.start_btn = QPushButton("Start Agent")
        self.start_btn.clicked.connect(self._on_start)
        self.start_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 5px 15px; }")
        control_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop Agent")
        self.stop_btn.clicked.connect(self._on_stop)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 5px 15px; }")
        control_layout.addWidget(self.stop_btn)

        layout.addLayout(control_layout)

        # Agent Log Viewer
        log_group = QGroupBox("Agent Log")
        log_layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        self.log_text.setFont(QFont("Courier", 9))
        log_layout.addWidget(self.log_text)

        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        # Options
        options_layout = QHBoxLayout()

        self.auto_reconnect_check = QCheckBox("Auto-reconnect on disconnect")
        self.auto_reconnect_check.toggled.connect(self._on_auto_reconnect_toggled)
        options_layout.addWidget(self.auto_reconnect_check)

        options_layout.addStretch()

        self.save_config_btn = QPushButton("Save Configuration")
        self.save_config_btn.clicked.connect(self._save_config)
        options_layout.addWidget(self.save_config_btn)

        layout.addLayout(options_layout)

        layout.addStretch()

        if not SERIAL_AVAILABLE:
            self._append_log("[WARNING] pyserial not installed. Install with: pip install pyserial")
            self.serial_radio.setEnabled(False)
            self.refresh_btn.setEnabled(False)

    def _on_connection_type_changed(self):
        """Handle connection type radio button change"""
        is_serial = self.serial_radio.isChecked()
        self.serial_group.setVisible(is_serial)
        self.udp_group.setVisible(not is_serial)

    def _refresh_ports(self):
        """Refresh available serial ports"""
        if not SERIAL_AVAILABLE:
            return

        self.port_combo.clear()
        ports = self._detect_serial_ports()

        if not ports:
            self.port_combo.addItem("No devices found")
            self._append_log("[INFO] No Arduino/ESP32 devices detected")
        else:
            for port in ports:
                label = f"{port['device']} - {port['description']}"
                self.port_combo.addItem(label, port['device'])
            self._append_log(f"[INFO] Found {len(ports)} device(s)")

    def _detect_serial_ports(self) -> List[Dict]:
        """Auto-detect Arduino/ESP32 serial ports"""
        # Common VIDs for Arduino and ESP32 boards
        arduino_vids = [
            0x2341,  # Arduino
            0x1A86,  # CH340 (clone boards)
            0x0403,  # FTDI
            0x10C4,  # CP210x (ESP32)
            0x067B,  # Prolific
        ]

        ports = []
        try:
            for port in serial.tools.list_ports.comports():
                if port.vid in arduino_vids or 'Arduino' in port.description or 'CH340' in port.description or 'CP210' in port.description:
                    ports.append({
                        'device': port.device,
                        'description': port.description,
                        'vid': port.vid,
                        'pid': port.pid
                    })
        except Exception as e:
            self.logger.error(f"Error detecting ports: {e}")

        return ports

    def _on_start(self):
        """Start micro-ROS agent"""
        success = False

        if self.serial_radio.isChecked():
            # Serial mode
            if self.port_combo.currentData():
                port = self.port_combo.currentData()
                baud_rate = int(self.baud_combo.currentText())
                success = self.agent_manager.start_serial_agent(port, baud_rate)

                if success:
                    self.last_config = {
                        'type': 'serial',
                        'port': port,
                        'baud_rate': baud_rate
                    }
                    self._append_log(f"[INFO] Starting serial agent on {port} @ {baud_rate} baud")
            else:
                QMessageBox.warning(self, "No Device", "No serial port selected")
                return
        else:
            # UDP mode
            try:
                udp_port = int(self.udp_port_edit.text())
                success = self.agent_manager.start_udp_agent(udp_port)

                if success:
                    self.last_config = {
                        'type': 'udp',
                        'port': udp_port
                    }
                    self._append_log(f"[INFO] Starting UDP agent on port {udp_port}")
            except ValueError:
                QMessageBox.warning(self, "Invalid Port", "UDP port must be a number")
                return

        if success:
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.serial_radio.setEnabled(False)
            self.udp_radio.setEnabled(False)
            self._update_status(True, "Starting...")

            # Start monitoring
            self.output_timer.start(100)  # Read output every 100ms
            self.monitor_timer.start(1000)  # Check connection every second
        else:
            QMessageBox.critical(self, "Start Failed",
                               "Failed to start micro-ROS agent. Make sure micro_ros_agent is installed:\n\n"
                               "sudo apt install ros-$ROS_DISTRO-micro-ros-agent")

    def _on_stop(self):
        """Stop micro-ROS agent"""
        self.output_timer.stop()
        self.monitor_timer.stop()

        if self.agent_manager.stop_agent():
            self._append_log("[INFO] Agent stopped")

        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.serial_radio.setEnabled(True)
        self.udp_radio.setEnabled(True)
        self._update_status(False, "Disconnected")

    def _read_agent_output(self):
        """Read and display agent output"""
        output = self.agent_manager.read_output()
        if output:
            self._append_log(output.strip())

            # Check for connection indicators in output
            if "Created client" in output or "session established" in output.lower():
                self._update_status(True, "Connected")

    def _monitor_connection(self):
        """Monitor agent process and connection status"""
        if not self.agent_manager.is_running():
            # Agent died
            self._append_log("[ERROR] Agent process terminated unexpectedly")
            self._update_status(False, "Disconnected")

            self.output_timer.stop()
            self.monitor_timer.stop()

            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.serial_radio.setEnabled(True)
            self.udp_radio.setEnabled(True)

            # Auto-reconnect if enabled
            if self.auto_reconnect and self.last_config:
                self._append_log("[INFO] Auto-reconnect in 3 seconds...")
                QTimer.singleShot(3000, self._auto_reconnect_attempt)

    def _auto_reconnect_attempt(self):
        """Attempt to reconnect automatically"""
        if not self.agent_manager.is_running() and self.auto_reconnect:
            self._append_log("[INFO] Attempting to reconnect...")
            self._on_start()

    def _update_status(self, connected: bool, status_text: str):
        """Update status indicator"""
        if connected:
            self.status_indicator.setText(f"● {status_text}")
            self.status_indicator.setStyleSheet("QLabel { color: green; font-weight: bold; }")
        else:
            self.status_indicator.setText(f"● {status_text}")
            self.status_indicator.setStyleSheet("QLabel { color: red; font-weight: bold; }")

        self.status_changed.emit(connected, status_text)

    def _append_log(self, message: str):
        """Append message to log viewer"""
        self.log_text.append(message)
        # Auto-scroll to bottom
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
        self.log_message.emit(message)

    def _on_auto_reconnect_toggled(self, checked: bool):
        """Handle auto-reconnect checkbox"""
        self.auto_reconnect = checked
        if checked:
            self._append_log("[INFO] Auto-reconnect enabled")
        else:
            self._append_log("[INFO] Auto-reconnect disabled")

    def _save_config(self):
        """Save current configuration"""
        config_dir = Path.home() / '.roboshire'
        config_dir.mkdir(exist_ok=True)
        config_file = config_dir / 'micro_ros_agent_config.json'

        config = {
            'connection_type': 'serial' if self.serial_radio.isChecked() else 'udp',
            'serial_port': self.port_combo.currentData() if self.port_combo.currentData() else '',
            'baud_rate': self.baud_combo.currentText(),
            'udp_port': self.udp_port_edit.text(),
            'auto_reconnect': self.auto_reconnect
        }

        try:
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)
            self._append_log(f"[INFO] Configuration saved to {config_file}")
            QMessageBox.information(self, "Saved", "Configuration saved successfully")
        except Exception as e:
            self.logger.error(f"Failed to save config: {e}")
            QMessageBox.warning(self, "Save Failed", f"Failed to save configuration: {e}")

    def _load_config(self):
        """Load saved configuration"""
        config_file = Path.home() / '.roboshire' / 'micro_ros_agent_config.json'

        if not config_file.exists():
            return

        try:
            with open(config_file, 'r') as f:
                config = json.load(f)

            # Apply configuration
            if config.get('connection_type') == 'udp':
                self.udp_radio.setChecked(True)
            else:
                self.serial_radio.setChecked(True)

            self.baud_combo.setCurrentText(config.get('baud_rate', '115200'))
            self.udp_port_edit.setText(config.get('udp_port', '8888'))
            self.auto_reconnect_check.setChecked(config.get('auto_reconnect', False))

            self.logger.info(f"Loaded configuration from {config_file}")

        except Exception as e:
            self.logger.warning(f"Failed to load config: {e}")

    def get_status(self) -> tuple:
        """Get current connection status"""
        is_running = self.agent_manager.is_running()
        status_text = self.status_indicator.text()
        return (is_running, status_text)

    def cleanup(self):
        """Cleanup when widget is closed"""
        self.output_timer.stop()
        self.monitor_timer.stop()
        self.agent_manager.stop_agent()
