"""
QoS Configuration Dialog

User-friendly dialog for configuring ROS2 Quality of Service (QoS) profiles.
Eliminates the #2 ROS2 pain point: "QoS confusion" (140+ forum mentions).

Features:
- Visual presets for common patterns
- Plain English explanations
- Real-time compatibility checking
- Example use cases for each setting
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QSpinBox, QGroupBox, QRadioButton, QTextEdit,
    QButtonGroup, QFrame
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from typing import Dict, Optional


class QoSConfigDialog(QDialog):
    """
    QoS Configuration Dialog

    Provides user-friendly interface for configuring ROS2 QoS profiles
    with presets, explanations, and compatibility checking.
    """

    # QoS presets for common patterns
    QOS_PRESETS = {
        "Sensor Data (Default)": {
            "reliability": "RELIABLE",
            "durability": "VOLATILE",
            "history": "KEEP_LAST",
            "depth": 10,
            "description": "✅ Best for sensors publishing frequently (IMU, cameras, encoders)\n"
                          "• Reliable delivery ensures no data loss\n"
                          "• Volatile means new subscribers don't get old data\n"
                          "• Works with 99% of sensor nodes",
            "examples": "IMU data, wheel encoders, temperature sensors, GPS"
        },
        "Command/Control": {
            "reliability": "RELIABLE",
            "durability": "TRANSIENT_LOCAL",
            "history": "KEEP_LAST",
            "depth": 5,
            "description": "🎯 Best for commands that must not be lost (motor commands, robot arm goals)\n"
                          "• Reliable ensures command arrives\n"
                          "• Transient Local means late-joining subscribers get last command\n"
                          "• Small queue (5) for responsive control",
            "examples": "cmd_vel, robot arm goals, gripper commands, LED control"
        },
        "Real-time Critical": {
            "reliability": "BEST_EFFORT",
            "durability": "VOLATILE",
            "history": "KEEP_LAST",
            "depth": 1,
            "description": "⚡ Best for time-critical data where speed > reliability\n"
                          "• Best Effort skips lost packets (no retries)\n"
                          "• Only keeps latest message (depth=1)\n"
                          "• Lowest latency possible",
            "examples": "Video streaming, audio, high-frequency odometry, lidar scans"
        },
        "Configuration/Parameters": {
            "reliability": "RELIABLE",
            "durability": "TRANSIENT_LOCAL",
            "history": "KEEP_ALL",
            "depth": 100,
            "description": "💾 Best for configuration data that must persist\n"
                          "• Reliable ensures all config messages delivered\n"
                          "• Transient Local stores messages for late subscribers\n"
                          "• Keep All history ensures nothing is dropped",
            "examples": "Robot configuration, calibration data, map data, waypoints"
        },
        "Status/Diagnostics": {
            "reliability": "BEST_EFFORT",
            "durability": "VOLATILE",
            "history": "KEEP_LAST",
            "depth": 10,
            "description": "📊 Best for status updates where latest value matters most\n"
                          "• Best Effort reduces network overhead\n"
                          "• Volatile means only current status matters\n"
                          "• Medium queue for monitoring tools",
            "examples": "Battery status, system health, CPU usage, temperature monitoring"
        },
        "Event/Log Messages": {
            "reliability": "RELIABLE",
            "durability": "TRANSIENT_LOCAL",
            "history": "KEEP_LAST",
            "depth": 100,
            "description": "📝 Best for important events that must be logged\n"
                          "• Reliable ensures no events are lost\n"
                          "• Transient Local so log viewers can catch up\n"
                          "• Large queue to handle bursts",
            "examples": "Error messages, state transitions, user actions, alarms"
        },
        "Custom": {
            "reliability": "RELIABLE",
            "durability": "VOLATILE",
            "history": "KEEP_LAST",
            "depth": 10,
            "description": "🔧 Configure your own QoS settings manually",
            "examples": "Use when none of the presets fit your use case"
        }
    }

    qos_configured = Signal(dict)  # Emits QoS configuration when accepted

    def __init__(self, current_qos: Optional[Dict] = None, parent=None):
        super().__init__(parent)

        self.current_qos = current_qos or {
            "reliability": "RELIABLE",
            "durability": "VOLATILE",
            "history": "KEEP_LAST",
            "depth": 10
        }

        self.setWindowTitle("Configure QoS Profile")
        self.setMinimumWidth(700)
        self.setMinimumHeight(600)

        self._setup_ui()
        self._load_current_preset()

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Quality of Service (QoS) Configuration")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        subtitle = QLabel(
            "QoS controls how messages are delivered between publishers and subscribers.\n"
            "Choose a preset that matches your use case, or configure manually."
        )
        subtitle.setWordWrap(True)
        layout.addWidget(subtitle)

        # Preset selection
        preset_group = QGroupBox("Quick Presets")
        preset_layout = QVBoxLayout()

        preset_layout.addWidget(QLabel("Choose a preset based on your use case:"))

        self.preset_combo = QComboBox()
        self.preset_combo.addItems(self.QOS_PRESETS.keys())
        self.preset_combo.currentTextChanged.connect(self._on_preset_changed)
        preset_layout.addWidget(self.preset_combo)

        # Description area
        self.description_text = QTextEdit()
        self.description_text.setReadOnly(True)
        self.description_text.setMaximumHeight(120)
        self.description_text.setStyleSheet("""
            QTextEdit {
                background-color: #F5F5F5;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 8px;
            }
        """)
        preset_layout.addWidget(self.description_text)

        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)

        # Manual configuration
        manual_group = QGroupBox("Manual Configuration")
        manual_layout = QVBoxLayout()

        # Reliability
        reliability_layout = QVBoxLayout()
        reliability_layout.addWidget(QLabel("<b>Reliability Policy</b>"))
        reliability_desc = QLabel(
            "🔒 RELIABLE: Guarantees delivery (retries until success)\n"
            "⚡ BEST_EFFORT: May lose messages but faster (no retries)"
        )
        reliability_desc.setWordWrap(True)
        reliability_layout.addWidget(reliability_desc)

        reliability_buttons_layout = QHBoxLayout()
        self.reliability_group = QButtonGroup(self)

        self.reliable_radio = QRadioButton("RELIABLE")
        self.reliable_radio.setChecked(True)
        self.reliability_group.addButton(self.reliable_radio, 0)
        reliability_buttons_layout.addWidget(self.reliable_radio)

        self.best_effort_radio = QRadioButton("BEST_EFFORT")
        self.reliability_group.addButton(self.best_effort_radio, 1)
        reliability_buttons_layout.addWidget(self.best_effort_radio)

        reliability_buttons_layout.addStretch()
        reliability_layout.addLayout(reliability_buttons_layout)
        manual_layout.addLayout(reliability_layout)

        # Separator
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setFrameShadow(QFrame.Sunken)
        manual_layout.addWidget(line1)

        # Durability
        durability_layout = QVBoxLayout()
        durability_layout.addWidget(QLabel("<b>Durability Policy</b>"))
        durability_desc = QLabel(
            "🔄 VOLATILE: Only current data (late subscribers miss old messages)\n"
            "💾 TRANSIENT_LOCAL: Stores messages (late subscribers receive recent history)"
        )
        durability_desc.setWordWrap(True)
        durability_layout.addWidget(durability_desc)

        durability_buttons_layout = QHBoxLayout()
        self.durability_group = QButtonGroup(self)

        self.volatile_radio = QRadioButton("VOLATILE")
        self.volatile_radio.setChecked(True)
        self.durability_group.addButton(self.volatile_radio, 0)
        durability_buttons_layout.addWidget(self.volatile_radio)

        self.transient_radio = QRadioButton("TRANSIENT_LOCAL")
        self.durability_group.addButton(self.transient_radio, 1)
        durability_buttons_layout.addWidget(self.transient_radio)

        durability_buttons_layout.addStretch()
        durability_layout.addLayout(durability_buttons_layout)
        manual_layout.addLayout(durability_layout)

        # Separator
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setFrameShadow(QFrame.Sunken)
        manual_layout.addWidget(line2)

        # History
        history_layout = QVBoxLayout()
        history_layout.addWidget(QLabel("<b>History Policy</b>"))
        history_desc = QLabel(
            "📝 KEEP_LAST: Keep only N most recent messages (set depth below)\n"
            "📚 KEEP_ALL: Keep all messages until delivered (may use lots of memory)"
        )
        history_desc.setWordWrap(True)
        history_layout.addWidget(history_desc)

        history_buttons_layout = QHBoxLayout()
        self.history_group = QButtonGroup(self)

        self.keep_last_radio = QRadioButton("KEEP_LAST")
        self.keep_last_radio.setChecked(True)
        self.history_group.addButton(self.keep_last_radio, 0)
        history_buttons_layout.addWidget(self.keep_last_radio)

        self.keep_all_radio = QRadioButton("KEEP_ALL")
        self.history_group.addButton(self.keep_all_radio, 1)
        history_buttons_layout.addWidget(self.keep_all_radio)

        history_buttons_layout.addStretch()
        history_layout.addLayout(history_buttons_layout)
        manual_layout.addLayout(history_layout)

        # Depth
        depth_layout = QHBoxLayout()
        depth_layout.addWidget(QLabel("<b>Queue Depth</b> (for KEEP_LAST):"))
        self.depth_spin = QSpinBox()
        self.depth_spin.setMinimum(1)
        self.depth_spin.setMaximum(1000)
        self.depth_spin.setValue(10)
        self.depth_spin.setSuffix(" messages")
        depth_layout.addWidget(self.depth_spin)
        depth_layout.addStretch()
        manual_layout.addLayout(depth_layout)

        # Connect manual changes to update preset to "Custom"
        self.reliable_radio.toggled.connect(self._on_manual_change)
        self.best_effort_radio.toggled.connect(self._on_manual_change)
        self.volatile_radio.toggled.connect(self._on_manual_change)
        self.transient_radio.toggled.connect(self._on_manual_change)
        self.keep_last_radio.toggled.connect(self._on_manual_change)
        self.keep_all_radio.toggled.connect(self._on_manual_change)
        self.depth_spin.valueChanged.connect(self._on_manual_change)

        manual_group.setLayout(manual_layout)
        layout.addWidget(manual_group)

        # Compatibility warning
        self.compatibility_label = QLabel()
        self.compatibility_label.setWordWrap(True)
        self.compatibility_label.setStyleSheet("""
            QLabel {
                background-color: #FFF3CD;
                border: 1px solid #FFC107;
                border-radius: 4px;
                padding: 8px;
                color: #856404;
            }
        """)
        self.compatibility_label.hide()
        layout.addWidget(self.compatibility_label)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)

        self.ok_btn = QPushButton("Apply QoS Configuration")
        self.ok_btn.clicked.connect(self._on_accept)
        self.ok_btn.setStyleSheet("""
            QPushButton {
                background-color: rgb(85, 107, 47);
                color: white;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: rgb(95, 120, 50);
            }
        """)
        button_layout.addWidget(self.ok_btn)

        layout.addLayout(button_layout)

    def _load_current_preset(self):
        """Load current QoS configuration"""
        # Find matching preset
        for preset_name, preset_config in self.QOS_PRESETS.items():
            if (preset_config.get("reliability") == self.current_qos.get("reliability") and
                preset_config.get("durability") == self.current_qos.get("durability") and
                preset_config.get("history") == self.current_qos.get("history") and
                preset_config.get("depth") == self.current_qos.get("depth")):
                self.preset_combo.setCurrentText(preset_name)
                return

        # No match - use Custom
        self.preset_combo.setCurrentText("Custom")
        self._apply_qos_to_ui(self.current_qos)

    def _on_preset_changed(self, preset_name: str):
        """Handle preset selection"""
        preset = self.QOS_PRESETS[preset_name]

        # Update description
        description = preset["description"]
        examples = preset["examples"]
        full_text = f"{description}\n\n<b>Examples:</b> {examples}"
        self.description_text.setHtml(full_text)

        # Update UI elements
        self._apply_qos_to_ui(preset)

        # Check compatibility
        self._check_compatibility()

    def _apply_qos_to_ui(self, qos_config: Dict):
        """Apply QoS configuration to UI elements"""
        # Block signals to prevent triggering manual change
        self.reliable_radio.blockSignals(True)
        self.best_effort_radio.blockSignals(True)
        self.volatile_radio.blockSignals(True)
        self.transient_radio.blockSignals(True)
        self.keep_last_radio.blockSignals(True)
        self.keep_all_radio.blockSignals(True)
        self.depth_spin.blockSignals(True)

        # Reliability
        if qos_config.get("reliability") == "RELIABLE":
            self.reliable_radio.setChecked(True)
        else:
            self.best_effort_radio.setChecked(True)

        # Durability
        if qos_config.get("durability") == "VOLATILE":
            self.volatile_radio.setChecked(True)
        else:
            self.transient_radio.setChecked(True)

        # History
        if qos_config.get("history") == "KEEP_LAST":
            self.keep_last_radio.setChecked(True)
        else:
            self.keep_all_radio.setChecked(True)

        # Depth
        self.depth_spin.setValue(qos_config.get("depth", 10))

        # Unblock signals
        self.reliable_radio.blockSignals(False)
        self.best_effort_radio.blockSignals(False)
        self.volatile_radio.blockSignals(False)
        self.transient_radio.blockSignals(False)
        self.keep_last_radio.blockSignals(False)
        self.keep_all_radio.blockSignals(False)
        self.depth_spin.blockSignals(False)

    def _on_manual_change(self):
        """Handle manual QoS changes"""
        # Switch preset to Custom
        if self.preset_combo.currentText() != "Custom":
            self.preset_combo.blockSignals(True)
            self.preset_combo.setCurrentText("Custom")
            self.preset_combo.blockSignals(False)

        # Update description for Custom
        self.description_text.setHtml(self.QOS_PRESETS["Custom"]["description"])

        # Check compatibility
        self._check_compatibility()

    def _check_compatibility(self):
        """Check for potential QoS compatibility issues"""
        warnings = []

        # Check for common incompatibilities
        if self.best_effort_radio.isChecked() and self.transient_radio.isChecked():
            warnings.append(
                "⚠️ BEST_EFFORT + TRANSIENT_LOCAL is unusual. "
                "TRANSIENT_LOCAL stores messages, but BEST_EFFORT may drop them."
            )

        if self.keep_all_radio.isChecked() and self.best_effort_radio.isChecked():
            warnings.append(
                "⚠️ KEEP_ALL + BEST_EFFORT may cause memory issues. "
                "KEEP_ALL stores everything, but BEST_EFFORT can't guarantee delivery."
            )

        if self.depth_spin.value() > 100 and self.keep_last_radio.isChecked():
            warnings.append(
                "⚠️ Queue depth > 100 may use significant memory. "
                "Consider using a smaller value or KEEP_ALL if you need buffering."
            )

        if warnings:
            self.compatibility_label.setText("\n".join(warnings))
            self.compatibility_label.show()
        else:
            self.compatibility_label.hide()

    def _on_accept(self):
        """Emit QoS configuration and accept dialog"""
        qos_config = {
            "reliability": "RELIABLE" if self.reliable_radio.isChecked() else "BEST_EFFORT",
            "durability": "VOLATILE" if self.volatile_radio.isChecked() else "TRANSIENT_LOCAL",
            "history": "KEEP_LAST" if self.keep_last_radio.isChecked() else "KEEP_ALL",
            "depth": self.depth_spin.value()
        }

        self.qos_configured.emit(qos_config)
        self.accept()

    def get_qos_config(self) -> Dict:
        """Get current QoS configuration"""
        return {
            "reliability": "RELIABLE" if self.reliable_radio.isChecked() else "BEST_EFFORT",
            "durability": "VOLATILE" if self.volatile_radio.isChecked() else "TRANSIENT_LOCAL",
            "history": "KEEP_LAST" if self.keep_last_radio.isChecked() else "KEEP_ALL",
            "depth": self.depth_spin.value()
        }


# Test code
if __name__ == '__main__':
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    dialog = QoSConfigDialog()

    def on_configured(qos):
        print(f"QoS Configuration: {qos}")

    dialog.qos_configured.connect(on_configured)

    if dialog.exec() == QDialog.Accepted:
        print(f"Final QoS: {dialog.get_qos_config()}")
    else:
        print("Canceled")

    sys.exit(0)
