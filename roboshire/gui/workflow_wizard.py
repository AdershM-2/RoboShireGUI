"""
Workflow Wizard - Interactive 5-Minute Robot Creation

Guides users through creating a complete robot project from scratch,
including hardware configuration, code generation, and ROS2 integration.

Author: RoboShire Team
Phase: 10 (Advanced UX)
"""

from PySide6.QtWidgets import (
    QWizard, QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QRadioButton, QCheckBox, QLineEdit, QTextEdit,
    QComboBox, QGroupBox, QListWidget, QListWidgetItem, QMessageBox,
    QFileDialog, QButtonGroup, QScrollArea, QWidget, QProgressBar
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QPixmap, QPainter, QColor
from pathlib import Path
from typing import Dict, List, Optional
import json


class WorkflowWizard(QWizard):
    """
    Interactive wizard for creating robot projects

    7-Step Process:
    1. URDF Selection (with/without)
    2. Hardware Configuration (MCU + Sensors)
    3. Project Goals & Naming
    4. Microcontroller Code Templates
    5. ROS2 Data Flow Setup
    5a. Lifecycle Nodes (Advanced, Optional)
    6. Build & Test Guide
    """

    # Wizard completed signal with project configuration
    project_created = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("New Robot Wizard - Create Your Robot in 5 Minutes")
        self.setWizardStyle(QWizard.ModernStyle)
        self.setOption(QWizard.HaveHelpButton, True)
        self.setOption(QWizard.HaveCustomButton1, False)

        # Make window resizable
        self.setWindowFlags(self.windowFlags() | Qt.WindowMaximizeButtonHint)
        self.setSizeGripEnabled(True)

        # Wizard configuration storage
        self.config = {
            'use_urdf': True,
            'urdf_path': None,
            'microcontroller': None,
            'sensors': [],
            'actuators': [],
            'communication': [],
            'project_name': '',
            'project_goal': '',
            'use_micro_ros': False,
            'arduino_code_path': None,
            'ros2_nodes': [],
            'use_lifecycle_nodes': False,
            'lifecycle_node_ids': []
        }

        # Add wizard pages
        self.addPage(Step1_URDFSelection(self.config))
        self.addPage(Step2_HardwareConfig(self.config))
        self.addPage(Step3_ProjectGoals(self.config))
        self.addPage(Step4_MicrocontrollerCode(self.config))
        self.addPage(Step5_ROS2DataFlow(self.config))
        self.addPage(Step5a_LifecycleNodes(self.config))
        self.addPage(Step6_BuildGuide(self.config))

        # Set reasonable initial size and constraints
        self.setMinimumSize(900, 650)  # Larger minimum size to prevent cramping
        self.resize(1000, 700)  # Comfortable initial size
        # No maximum size - user can resize as needed

        # Connect finish
        self.finished.connect(self._on_wizard_finished)

        # Help button handler
        self.helpRequested.connect(self._show_help)

    def _on_wizard_finished(self, result):
        """Handle wizard completion"""
        if result == QWizard.Accepted:
            # Emit project configuration
            self.project_created.emit(self.config)

    def _show_help(self):
        """Show context-sensitive help"""
        current_page = self.currentPage()
        if hasattr(current_page, 'get_help_text'):
            help_text = current_page.get_help_text()
            QMessageBox.information(
                self,
                "Help - " + current_page.title(),
                help_text
            )


def create_scrollable_page_layout(page: QWizardPage) -> QVBoxLayout:
    """
    Create a scrollable layout for wizard pages.
    Returns the content layout that should be populated with widgets.
    """
    # Main layout for the page
    main_layout = QVBoxLayout(page)
    main_layout.setContentsMargins(0, 0, 0, 0)

    # Scroll area
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QScrollArea.NoFrame)
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

    # Content widget inside scroll area
    content_widget = QWidget()
    content_layout = QVBoxLayout(content_widget)
    content_layout.setContentsMargins(10, 10, 10, 10)

    scroll.setWidget(content_widget)
    main_layout.addWidget(scroll)

    return content_layout


class Step1_URDFSelection(QWizardPage):
    """Step 1: Choose to start with or without URDF"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 1: Robot Starting Point")
        self.setSubTitle(
            "Choose how you want to start your robot project. "
            "You can create a physical robot (with URDF) or a software-only project like a weather station."
        )

        self._setup_ui()

    def _setup_ui(self):
        # Use scrollable layout
        layout = create_scrollable_page_layout(self)

        # Preset examples section
        presets_group = QGroupBox("🚀 Quick Start: Example Robot Presets")
        presets_layout = QVBoxLayout()

        presets_desc = QLabel(
            "New to RoboShire? Try these preset examples to learn the workflow:"
        )
        presets_desc.setWordWrap(True)
        presets_layout.addWidget(presets_desc)

        presets_buttons_layout = QHBoxLayout()

        self.weather_preset_btn = QPushButton("📡 Weather Monitor Station")
        self.weather_preset_btn.setToolTip("IoT project: ESP32 + sensors (no URDF)")
        self.weather_preset_btn.clicked.connect(self._load_weather_preset)
        self.weather_preset_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 10px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        presets_buttons_layout.addWidget(self.weather_preset_btn)

        self.diff_drive_preset_btn = QPushButton("🤖 Differential Drive Robot")
        self.diff_drive_preset_btn.setToolTip("Mobile robot: Arduino + motors + sensors (with URDF)")
        self.diff_drive_preset_btn.clicked.connect(self._load_diff_drive_preset)
        self.diff_drive_preset_btn.setStyleSheet("""
            QPushButton {
                background-color: #556B2F;
                color: white;
                padding: 10px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
        """)
        presets_buttons_layout.addWidget(self.diff_drive_preset_btn)

        presets_layout.addLayout(presets_buttons_layout)

        preset_note = QLabel(
            "💡 <b>Presets will auto-fill all wizard steps.</b> You can still modify any settings!"
        )
        preset_note.setWordWrap(True)
        preset_note.setStyleSheet("color: #666; font-size: 10px; margin-top: 5px;")
        presets_layout.addWidget(preset_note)

        presets_group.setLayout(presets_layout)
        layout.addWidget(presets_group)

        layout.addSpacing(15)

        # Divider
        divider = QLabel("─" * 80)
        divider.setStyleSheet("color: #ccc;")
        layout.addWidget(divider)

        layout.addSpacing(10)

        # Explanation
        explanation = QLabel(
            "🤖 <b>Or create a custom robot from scratch:</b><br>"
            "URDF (Unified Robot Description Format) describes the physical structure of a robot: "
            "links (body parts), joints (connections), sensors, and geometry.<br><br>"
            "<b>Choose:</b>"
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)

        # Radio button group
        self.button_group = QButtonGroup(self)

        # Option 1: With URDF
        with_urdf_option = QGroupBox()
        with_urdf_layout = QVBoxLayout()

        self.with_urdf_radio = QRadioButton("Start with URDF (Physical Robot)")
        self.with_urdf_radio.setFont(QFont("", 10, QFont.Bold))
        self.with_urdf_radio.setChecked(True)
        self.button_group.addButton(self.with_urdf_radio, 1)
        with_urdf_layout.addWidget(self.with_urdf_radio)

        with_urdf_desc = QLabel(
            "   ✓ For robots with motors, wheels, arms, legs\n"
            "   ✓ Need 3D visualization in MuJoCo\n"
            "   ✓ Examples: Mobile robot, robot arm, humanoid"
        )
        with_urdf_desc.setStyleSheet("color: #666; margin-left: 20px;")
        with_urdf_layout.addWidget(with_urdf_desc)

        # URDF file selection
        urdf_file_layout = QHBoxLayout()
        urdf_file_layout.addSpacing(30)

        self.urdf_source_combo = QComboBox()
        self.urdf_source_combo.addItems([
            "Load from file...",
            "Use example: Differential Drive Robot",
            "Use example: 6-DOF Robot Arm",
            "Use example: Quadruped Robot",
            "Create new URDF (advanced)"
        ])
        self.urdf_source_combo.setEnabled(True)
        self.urdf_source_combo.currentIndexChanged.connect(self._on_urdf_source_changed)
        urdf_file_layout.addWidget(QLabel("URDF Source:"))
        urdf_file_layout.addWidget(self.urdf_source_combo, 1)

        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self._browse_urdf)
        self.browse_button.setEnabled(False)
        urdf_file_layout.addWidget(self.browse_button)

        with_urdf_layout.addLayout(urdf_file_layout)

        self.urdf_path_label = QLabel("")
        self.urdf_path_label.setStyleSheet("color: #556B2F; margin-left: 30px;")
        with_urdf_layout.addWidget(self.urdf_path_label)

        with_urdf_option.setLayout(with_urdf_layout)
        layout.addWidget(with_urdf_option)

        layout.addSpacing(20)

        # Option 2: Without URDF
        without_urdf_option = QGroupBox()
        without_urdf_layout = QVBoxLayout()

        self.without_urdf_radio = QRadioButton("Start without URDF (Software/Sensor Project)")
        self.without_urdf_radio.setFont(QFont("", 10, QFont.Bold))
        self.button_group.addButton(self.without_urdf_radio, 2)
        without_urdf_layout.addWidget(self.without_urdf_radio)

        without_urdf_desc = QLabel(
            "   ✓ For stationary projects (weather station, sensor hub)\n"
            "   ✓ Focus on data collection and processing\n"
            "   ✓ Examples: Weather monitor, smart home hub, data logger"
        )
        without_urdf_desc.setStyleSheet("color: #666; margin-left: 20px;")
        without_urdf_layout.addWidget(without_urdf_desc)

        without_urdf_option.setLayout(without_urdf_layout)
        layout.addWidget(without_urdf_option)

        # Connect radio button changes
        self.with_urdf_radio.toggled.connect(self._on_urdf_choice_changed)

        layout.addStretch()

        # Info box
        info_box = QLabel(
            "💡 <b>Tip:</b> You can always change this later by modifying your project configuration."
        )
        info_box.setWordWrap(True)
        info_box.setStyleSheet(
            "background-color: #F0F5E6; padding: 10px; border-radius: 5px; color: #556B2F;"
        )
        layout.addWidget(info_box)

        # Note: Layout already set by create_scrollable_page_layout()

        # Register fields for validation
        self.registerField("use_urdf", self.with_urdf_radio)

    def _on_urdf_choice_changed(self, checked):
        """Handle URDF choice radio button change"""
        self.urdf_source_combo.setEnabled(checked)
        self.config['use_urdf'] = checked

        if not checked:
            self.urdf_path_label.setText("")
            self.config['urdf_path'] = None

    def _on_urdf_source_changed(self, index):
        """Handle URDF source selection"""
        self.browse_button.setEnabled(index == 0)

        if index == 0:
            # Load from file
            self.urdf_path_label.setText("Click 'Browse...' to select URDF file")
        elif index == 1:
            # Differential drive
            path = "roboshire/examples/differential_drive/robot.urdf"
            self.urdf_path_label.setText(f"✓ Using: {path}")
            self.config['urdf_path'] = path
        elif index == 2:
            # Robot arm
            path = "roboshire/examples/robotic_arm_6dof/robot.urdf"
            self.urdf_path_label.setText(f"✓ Using: {path}")
            self.config['urdf_path'] = path
        elif index == 3:
            # Quadruped
            path = "roboshire/examples/quadruped_robot/robot.urdf"
            self.urdf_path_label.setText(f"✓ Using: {path}")
            self.config['urdf_path'] = path
        elif index == 4:
            # Create new
            self.urdf_path_label.setText("⚠ You'll create URDF after wizard (advanced)")
            self.config['urdf_path'] = "create_new"

    def _browse_urdf(self):
        """Browse for URDF file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select URDF File",
            "",
            "URDF Files (*.urdf);;All Files (*)"
        )

        if file_path:
            self.urdf_path_label.setText(f"✓ Selected: {file_path}")
            self.config['urdf_path'] = file_path

    def validatePage(self):
        """Validate page before moving to next"""
        if self.with_urdf_radio.isChecked():
            if not self.config.get('urdf_path'):
                QMessageBox.warning(
                    self,
                    "URDF Required",
                    "Please select a URDF source before continuing."
                )
                return False

        return True

    def _load_weather_preset(self):
        """Load weather monitor preset configuration"""
        # Step 1: No URDF
        self.without_urdf_radio.setChecked(True)
        self.config['use_urdf'] = False
        self.config['urdf_path'] = None

        # Notify user
        QMessageBox.information(
            self,
            "Weather Monitor Preset Loaded",
            "✅ Weather Monitor configuration loaded!\n\n"
            "This preset includes:\n"
            "• ESP32 microcontroller\n"
            "• Temperature/Humidity sensor (DHT22)\n"
            "• Pressure sensor (BMP280)\n"
            "• WiFi communication\n"
            "• Project name: 'weather_station'\n"
            "• Complete node graph\n"
            "• Arduino code ready to upload\n\n"
            "Click 'Next' to see auto-filled settings. You can modify them!\n\n"
            "After the wizard, node graph will be auto-loaded!"
        )

        # Pre-fill config for other steps
        self.config['microcontroller'] = "ESP32 (WiFi + Bluetooth)"
        self.config['sensors'] = [
            "Temperature (DHT11/22, DS18B20)",
            "Pressure (BMP280/BME280)",
            "Light Sensor (LDR/BH1750)"
        ]
        self.config['actuators'] = []
        self.config['communication'] = ["WiFi", "Serial (UART)"]
        self.config['project_name'] = 'weather_station'
        self.config['project_goal'] = 'Weather monitoring station'
        self.config['project_description'] = (
            'Monitor temperature, humidity, pressure, and light levels. '
            'Publish sensor data to ROS2 topics for logging and visualization.'
        )
        self.config['use_micro_ros'] = False
        self.config['code_template_type'] = 'Standard Arduino (.ino) - Recommended'
        self.config['ai_assistance_enabled'] = False

        # Add example paths for complete project
        self.config['example_type'] = 'weather_station'
        self.config['node_graph_path'] = 'roboshire/examples/weather_station/node_graph.json'
        self.config['arduino_code_path'] = 'roboshire/examples/weather_station/arduino_code.ino'
        self.config['is_preset_example'] = True

    def _load_diff_drive_preset(self):
        """Load differential drive robot preset configuration"""
        # Step 1: With URDF
        self.with_urdf_radio.setChecked(True)
        self.urdf_source_combo.setCurrentIndex(1)  # Diff drive example
        self.config['use_urdf'] = True
        self.config['urdf_path'] = "roboshire/examples/differential_drive/robot.urdf"
        self.urdf_path_label.setText(f"✓ Using: {self.config['urdf_path']}")

        # Notify user
        QMessageBox.information(
            self,
            "Differential Drive Robot Preset Loaded",
            "✅ Differential Drive Robot configuration loaded!\n\n"
            "This preset includes:\n"
            "• Arduino Mega microcontroller\n"
            "• 2x DC Motors (wheels)\n"
            "• IMU sensor (MPU6050)\n"
            "• Ultrasonic sensor (HC-SR04)\n"
            "• Serial communication\n"
            "• Project name: 'diff_drive_robot'\n"
            "• Complete node graph with obstacle avoidance\n"
            "• Arduino motor control code ready to upload\n\n"
            "Click 'Next' to see auto-filled settings. You can modify them!\n\n"
            "After the wizard, node graph will be auto-loaded!"
        )

        # Pre-fill config for other steps
        self.config['microcontroller'] = "Arduino Mega 2560 (ATmega2560)"
        self.config['sensors'] = [
            "IMU (MPU6050/BNO055)",
            "Ultrasonic (HC-SR04)"
        ]
        self.config['actuators'] = [
            "DC Motor"
        ]
        self.config['communication'] = ["Serial (UART)", "I2C"]
        self.config['project_name'] = 'diff_drive_robot'
        self.config['project_goal'] = 'Obstacle avoidance robot'
        self.config['project_description'] = (
            'Two-wheeled mobile robot with obstacle detection. '
            'Uses ultrasonic sensor for distance measurement and IMU for orientation tracking.'
        )
        self.config['use_micro_ros'] = False
        self.config['code_template_type'] = 'Standard Arduino (.ino) - Recommended'
        self.config['ai_assistance_enabled'] = False

        # Add example paths for complete project
        self.config['example_type'] = 'diff_drive_robot'
        self.config['node_graph_path'] = 'roboshire/examples/diff_drive_robot/node_graph.json'
        self.config['arduino_code_path'] = 'roboshire/examples/diff_drive_robot/arduino_code.ino'
        self.config['is_preset_example'] = True

    def get_help_text(self):
        return (
            "<h3>Step 1: Robot Starting Point</h3>"
            "<p><b>Preset Examples:</b> Try the Weather Monitor or Differential Drive presets "
            "to learn the complete workflow. All settings will be auto-filled!</p>"
            "<p><b>With URDF:</b> Choose this if your robot has physical structure (motors, links, joints). "
            "URDF files describe the robot's geometry and help visualize it in 3D.</p>"
            "<p><b>Without URDF:</b> Choose this for software-only projects like sensor hubs, "
            "data loggers, or stationary monitoring systems.</p>"
            "<p><b>Examples without URDF:</b> Weather station, smart home controller, "
            "environmental monitor, data aggregator.</p>"
        )


class Step2_HardwareConfig(QWizardPage):
    """Step 2: Select microcontroller and hardware components"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 2: Hardware Configuration")
        self.setSubTitle(
            "Select your microcontroller and the sensors/devices you'll use. "
            "Don't worry, you can add or remove these later!"
        )

        self._setup_ui()

    def _setup_ui(self):
        # Use scrollable layout
        layout = create_scrollable_page_layout(self)

        # Microcontroller selection
        mcu_group = QGroupBox("Microcontroller / Computer")
        mcu_layout = QVBoxLayout()

        mcu_label = QLabel(
            "<b>What microcontroller are you using?</b><br>"
            "This is the 'brain' that controls your sensors and motors."
        )
        mcu_label.setWordWrap(True)
        mcu_layout.addWidget(mcu_label)

        self.mcu_combo = QComboBox()
        self.mcu_combo.addItems([
            "Select microcontroller...",
            "Arduino Uno (ATmega328P)",
            "Arduino Mega 2560 (ATmega2560)",
            "Arduino Nano (ATmega328P - Small)",
            "ESP32 (WiFi + Bluetooth)",
            "ESP8266 (WiFi)",
            "Raspberry Pi Pico (RP2040)",
            "Teensy 4.0/4.1 (ARM Cortex-M7)",
            "STM32 (ARM Cortex-M series)",
            "Raspberry Pi 3/4 (Linux SBC)",
            "Jetson Nano (NVIDIA AI)",
            "Other / Custom"
        ])
        self.mcu_combo.currentIndexChanged.connect(self._on_mcu_changed)
        mcu_layout.addWidget(self.mcu_combo)

        # Micro-ROS option
        self.micro_ros_checkbox = QCheckBox(
            "Enable micro-ROS (Run ROS2 directly on microcontroller)"
        )
        self.micro_ros_checkbox.setToolTip(
            "micro-ROS allows your microcontroller to be a full ROS2 node. "
            "Recommended for ESP32, Teensy, or STM32. Requires more setup."
        )
        self.micro_ros_checkbox.stateChanged.connect(self._on_micro_ros_changed)
        mcu_layout.addWidget(self.micro_ros_checkbox)

        self.micro_ros_info = QLabel(
            "ℹ️ micro-ROS lets your Arduino/ESP32 communicate natively with ROS2. "
            "Without it, we'll use serial communication."
        )
        self.micro_ros_info.setWordWrap(True)
        self.micro_ros_info.setStyleSheet("color: #666; font-size: 10px; margin-left: 20px;")
        self.micro_ros_info.setVisible(False)
        mcu_layout.addWidget(self.micro_ros_info)

        mcu_group.setLayout(mcu_layout)
        layout.addWidget(mcu_group)

        # Hardware components (scrollable)
        hardware_group = QGroupBox("Sensors & Devices")
        hardware_layout = QVBoxLayout()

        hardware_desc = QLabel(
            "<b>What hardware do you have?</b> (Select all that apply)<br>"
            "✓ Check everything you plan to use - you can modify this later"
        )
        hardware_desc.setWordWrap(True)
        hardware_layout.addWidget(hardware_desc)

        # Scrollable area for components
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMaximumHeight(250)

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)

        # Motors & Actuators
        self.motors_group = self._create_checkbox_group(
            "🔧 Motors & Actuators",
            [
                ("DC Motor", "Standard DC motor for wheels"),
                ("Servo Motor", "Position-controlled motor (0-180°)"),
                ("Stepper Motor", "Precise step-by-step motor"),
                ("Brushless Motor (ESC)", "High-power motor with ESC"),
                ("Linear Actuator", "Push/pull motion"),
            ]
        )
        scroll_layout.addWidget(self.motors_group)

        # Sensors
        self.sensors_group = self._create_checkbox_group(
            "📡 Sensors",
            [
                ("IMU (MPU6050/BNO055)", "Accelerometer + Gyroscope"),
                ("Ultrasonic (HC-SR04)", "Distance sensor (2-400cm)"),
                ("LiDAR", "Laser distance scanner"),
                ("Camera (USB/CSI)", "Visual sensing"),
                ("GPS (NEO-6M)", "Location tracking"),
                ("Temperature (DHT11/22, DS18B20)", "Temperature/humidity"),
                ("Pressure (BMP280/BME280)", "Barometric pressure"),
                ("Gas Sensor (MQ-series)", "Gas/smoke detection"),
                ("PIR Motion Sensor", "Detect movement"),
                ("Light Sensor (LDR/BH1750)", "Ambient light"),
                ("Soil Moisture", "For plant monitoring"),
                ("Current Sensor (ACS712)", "Measure current draw"),
            ]
        )
        scroll_layout.addWidget(self.sensors_group)

        # Communication
        self.communication_group = self._create_checkbox_group(
            "📶 Communication",
            [
                ("Serial (UART)", "Standard serial communication"),
                ("I2C", "Two-wire protocol (sensors)"),
                ("SPI", "High-speed serial (displays, SD)"),
                ("CAN Bus", "Automotive communication"),
                ("Bluetooth", "Wireless (BLE/Classic)"),
                ("WiFi", "Wireless network"),
                ("LoRa", "Long-range wireless"),
            ]
        )
        scroll_layout.addWidget(self.communication_group)

        # Display & Output
        self.output_group = self._create_checkbox_group(
            "🖥️ Display & Output",
            [
                ("OLED Display (128x64)", "Small screen"),
                ("LCD Display (16x2 / 20x4)", "Character display"),
                ("LED Strip (WS2812B)", "RGB LEDs"),
                ("Buzzer/Speaker", "Audio output"),
                ("Relay Module", "Switch high-power devices"),
            ]
        )
        scroll_layout.addWidget(self.output_group)

        scroll.setWidget(scroll_widget)
        hardware_layout.addWidget(scroll)

        hardware_group.setLayout(hardware_layout)
        layout.addWidget(hardware_group)

        # Note: Layout already set by create_scrollable_page_layout()

        # Register field
        self.registerField("microcontroller*", self.mcu_combo)

    def initializePage(self):
        """Apply preset values when page is initialized"""
        # Apply microcontroller from config
        if self.config.get('microcontroller'):
            mcu = self.config['microcontroller']
            for i in range(self.mcu_combo.count()):
                if mcu in self.mcu_combo.itemText(i):
                    self.mcu_combo.setCurrentIndex(i)
                    break

        # Apply sensors from config
        if self.config.get('sensors'):
            for sensor_name in self.config['sensors']:
                for checkbox in self.sensors_group.findChildren(QCheckBox):
                    if sensor_name in checkbox.text():
                        checkbox.setChecked(True)

        # Apply actuators from config
        if self.config.get('actuators'):
            for actuator_name in self.config['actuators']:
                for checkbox in self.motors_group.findChildren(QCheckBox):
                    if actuator_name in checkbox.text():
                        checkbox.setChecked(True)

        # Apply communication from config
        if self.config.get('communication'):
            for comm_name in self.config['communication']:
                for checkbox in self.communication_group.findChildren(QCheckBox):
                    if comm_name in checkbox.text():
                        checkbox.setChecked(True)

    def _create_checkbox_group(self, title: str, items: List[tuple]) -> QGroupBox:
        """Create a checkbox group with tooltips"""
        group = QGroupBox(title)
        group_layout = QVBoxLayout()
        group_layout.setSpacing(5)

        for item_name, item_tooltip in items:
            checkbox = QCheckBox(item_name)
            checkbox.setToolTip(item_tooltip)
            checkbox.stateChanged.connect(self._on_hardware_changed)
            group_layout.addWidget(checkbox)

        group.setLayout(group_layout)
        return group

    def _on_mcu_changed(self, index):
        """Handle microcontroller selection"""
        if index > 0:
            self.config['microcontroller'] = self.mcu_combo.currentText()

            # Show micro-ROS option for compatible MCUs
            compatible_mcus = ['ESP32', 'Teensy', 'STM32', 'Raspberry Pi Pico']
            is_compatible = any(mcu in self.mcu_combo.currentText() for mcu in compatible_mcus)
            self.micro_ros_checkbox.setEnabled(is_compatible)

            if is_compatible:
                self.micro_ros_info.setVisible(True)
            else:
                self.micro_ros_checkbox.setChecked(False)
                self.micro_ros_info.setVisible(False)

    def _on_micro_ros_changed(self, state):
        """Handle micro-ROS checkbox"""
        self.config['use_micro_ros'] = (state == Qt.Checked)

    def _on_hardware_changed(self):
        """Collect all selected hardware"""
        sensors = []
        actuators = []
        communication = []

        # Collect from each group
        for checkbox in self.motors_group.findChildren(QCheckBox):
            if checkbox.isChecked():
                actuators.append(checkbox.text())

        for checkbox in self.sensors_group.findChildren(QCheckBox):
            if checkbox.isChecked():
                sensors.append(checkbox.text())

        for checkbox in self.communication_group.findChildren(QCheckBox):
            if checkbox.isChecked():
                communication.append(checkbox.text())

        for checkbox in self.output_group.findChildren(QCheckBox):
            if checkbox.isChecked():
                sensors.append(checkbox.text())  # Outputs treated as sensors for ROS

        self.config['sensors'] = sensors
        self.config['actuators'] = actuators
        self.config['communication'] = communication

    def validatePage(self):
        """Validate page"""
        if self.mcu_combo.currentIndex() == 0:
            QMessageBox.warning(
                self,
                "Microcontroller Required",
                "Please select a microcontroller before continuing."
            )
            return False

        # Update config one last time
        self._on_hardware_changed()

        return True

    def get_help_text(self):
        return (
            "<h3>Step 2: Hardware Configuration</h3>"
            "<p><b>Microcontroller:</b> The main brain that runs your Arduino code. "
            "Different boards have different capabilities (memory, speed, pins).</p>"
            "<p><b>micro-ROS:</b> Enables your microcontroller to run ROS2 natively. "
            "Recommended for ESP32/Teensy. Otherwise, we'll use serial communication.</p>"
            "<p><b>Sensors:</b> Select everything you have or plan to add. "
            "This helps us generate the correct Arduino and ROS2 code templates.</p>"
            "<p><b>Note:</b> You can always add/remove hardware later!</p>"
        )


class Step3_ProjectGoals(QWizardPage):
    """Step 3: Define project name and goals"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 3: Project Goals & Naming")
        self.setSubTitle(
            "Give your project a name and describe what you want to achieve."
        )

        self._setup_ui()

    def _setup_ui(self):
        # Use scrollable layout
        layout = create_scrollable_page_layout(self)

        # Project name
        name_group = QGroupBox("Project Name")
        name_layout = QVBoxLayout()

        name_label = QLabel(
            "<b>What do you want to call your project?</b><br>"
            "Use lowercase with underscores (e.g., 'weather_station', 'line_follower_bot')"
        )
        name_label.setWordWrap(True)
        name_layout.addWidget(name_label)

        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("my_robot_project")
        self.name_input.textChanged.connect(self._on_name_changed)
        name_layout.addWidget(self.name_input)

        self.name_validation = QLabel("")
        self.name_validation.setStyleSheet("color: #666; font-size: 10px;")
        name_layout.addWidget(self.name_validation)

        name_group.setLayout(name_layout)
        layout.addWidget(name_group)

        # Workspace folder selection
        workspace_group = QGroupBox("📁 Workspace & Build Folder")
        workspace_layout = QVBoxLayout()

        workspace_label = QLabel(
            "<b>Where should we create your ROS2 workspace?</b><br>"
            "A workspace is a folder containing your ROS2 packages, code, and build files."
        )
        workspace_label.setWordWrap(True)
        workspace_layout.addWidget(workspace_label)

        # Workspace path selection
        workspace_path_layout = QHBoxLayout()

        self.workspace_input = QLineEdit()
        self.workspace_input.setPlaceholderText("~/ros2_ws  (Default workspace location)")
        self.workspace_input.setText("~/ros2_ws")
        self.workspace_input.textChanged.connect(self._on_workspace_changed)
        workspace_path_layout.addWidget(self.workspace_input, 1)

        self.browse_workspace_btn = QPushButton("Browse...")
        self.browse_workspace_btn.clicked.connect(self._browse_workspace)
        workspace_path_layout.addWidget(self.browse_workspace_btn)

        workspace_layout.addLayout(workspace_path_layout)

        # Beginner-friendly explanation
        workspace_info = QLabel(
            "<b>💡 What's happening inside the workspace?</b><br><br>"
            "<b>src/</b> - Your Python source code (ROS2 nodes) goes here<br>"
            "<b>build/</b> - Temporary files created during compilation (auto-generated)<br>"
            "<b>install/</b> - Final installed packages ready to run (auto-generated)<br>"
            "<b>log/</b> - Build and runtime logs (auto-generated)<br><br>"
            "<b>Note:</b> The workspace is located on your local Ubuntu system for optimal performance."
        )
        workspace_info.setWordWrap(True)
        workspace_info.setStyleSheet(
            "background-color: #FFF9E6; padding: 10px; border-radius: 5px; "
            "color: #7D6608; font-size: 10px; margin-top: 5px;"
        )
        workspace_layout.addWidget(workspace_info)

        # Build folder explanation
        build_info = QLabel(
            "<b>📦 About the Build Folder:</b><br>"
            "• Created automatically when you press 'Build' (Ctrl+B)<br>"
            "• Contains compiled code and intermediate files<br>"
            "• Don't edit files in build/ directly - edit your source code instead<br>"
            "• If build fails, you can delete build/ folder and rebuild from scratch<br>"
            "• Location: <code>~/ros2_ws/build/[your_package_name]</code>"
        )
        build_info.setWordWrap(True)
        build_info.setStyleSheet("color: #666; font-size: 10px; margin-top: 10px;")
        workspace_layout.addWidget(build_info)

        workspace_group.setLayout(workspace_layout)
        layout.addWidget(workspace_group)

        # Project goal/purpose
        goal_group = QGroupBox("What are you trying to achieve?")
        goal_layout = QVBoxLayout()

        goal_label = QLabel(
            "<b>Select a goal or describe your own:</b><br>"
            "This helps us generate better code and documentation for your project."
        )
        goal_label.setWordWrap(True)
        goal_layout.addWidget(goal_label)

        self.goal_combo = QComboBox()
        self.goal_combo.addItems([
            "Custom goal (describe below)",
            "Line follower robot",
            "Obstacle avoidance robot",
            "Delivery/transport robot",
            "Surveillance/patrol robot",
            "Weather monitoring station",
            "Environmental data logger",
            "Smart home controller",
            "Plant watering system",
            "Security/alarm system",
            "Remote controlled vehicle",
            "Autonomous navigation",
            "Object detection/tracking",
            "Data collection platform"
        ])
        self.goal_combo.currentIndexChanged.connect(self._on_goal_changed)
        goal_layout.addWidget(self.goal_combo)

        goal_desc_label = QLabel("Describe your goal (optional but helpful):")
        goal_layout.addWidget(goal_desc_label)

        self.goal_description = QTextEdit()
        self.goal_description.setPlaceholderText(
            "Example: Create a weather station that monitors temperature, humidity, "
            "and pressure, then publishes data to ROS2 topics for logging and visualization."
        )
        self.goal_description.setMaximumHeight(100)
        self.goal_description.textChanged.connect(self._on_description_changed)
        goal_layout.addWidget(self.goal_description)

        goal_group.setLayout(goal_layout)
        layout.addWidget(goal_group)

        layout.addStretch()

        # Info box
        info_box = QLabel(
            "💡 <b>Why does this matter?</b><br>"
            "Based on your goal, we'll generate appropriate ROS2 nodes, topics, and test procedures. "
            "For example, a line follower needs different sensors and logic than a weather station!"
        )
        info_box.setWordWrap(True)
        info_box.setStyleSheet(
            "background-color: #E8F5E9; padding: 10px; border-radius: 5px; color: #2E7D32;"
        )
        layout.addWidget(info_box)

        # Note: Layout already set by create_scrollable_page_layout()

        # Register fields
        self.registerField("project_name*", self.name_input)

    def initializePage(self):
        """Apply preset values when page is initialized"""
        # Apply project name from config
        if self.config.get('project_name'):
            self.name_input.setText(self.config['project_name'])

        # Apply project goal from config
        if self.config.get('project_goal'):
            goal_text = self.config['project_goal']
            for i in range(self.goal_combo.count()):
                if goal_text in self.goal_combo.itemText(i):
                    self.goal_combo.setCurrentIndex(i)
                    break

        # Apply description from config
        if self.config.get('project_description'):
            self.goal_description.setPlainText(self.config['project_description'])

    def _on_name_changed(self, text):
        """Validate project name"""
        # Check if valid
        if not text:
            self.name_validation.setText("❌ Name cannot be empty")
            self.name_validation.setStyleSheet("color: #f44336;")
            return

        # Check format
        import re
        if not re.match(r'^[a-z][a-z0-9_]*$', text):
            self.name_validation.setText(
                "❌ Use lowercase letters, numbers, and underscores only. Must start with a letter."
            )
            self.name_validation.setStyleSheet("color: #f44336;")
            return

        # Valid
        self.name_validation.setText(f"✓ Valid project name: '{text}'")
        self.name_validation.setStyleSheet("color: #4CAF50;")
        self.config['project_name'] = text

    def _on_goal_changed(self, index):
        """Handle goal selection"""
        if index > 0:
            self.config['project_goal'] = self.goal_combo.currentText()

    def _on_description_changed(self):
        """Handle description change"""
        self.config['project_description'] = self.goal_description.toPlainText()

    def _on_workspace_changed(self, text):
        """Handle workspace path change"""
        self.config['workspace_path'] = text

    def _browse_workspace(self):
        """Browse for workspace directory"""
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Workspace Directory",
            self.workspace_input.text() or "~"
        )
        if directory:
            self.workspace_input.setText(directory)
            self.config['workspace_path'] = directory

    def validatePage(self):
        """Validate page"""
        import re

        # Check name
        name = self.name_input.text()
        if not name:
            QMessageBox.warning(self, "Name Required", "Please enter a project name.")
            return False

        if not re.match(r'^[a-z][a-z0-9_]*$', name):
            QMessageBox.warning(
                self,
                "Invalid Name",
                "Project name must:\n"
                "• Start with a lowercase letter\n"
                "• Contain only lowercase letters, numbers, and underscores\n"
                "• Example: weather_station_v1"
            )
            return False

        return True

    def get_help_text(self):
        return (
            "<h3>Step 3: Project Goals</h3>"
            "<p><b>Project Name:</b> This will be used for ROS2 package names, "
            "Arduino sketches, and file names. Use snake_case (lowercase with underscores).</p>"
            "<p><b>Project Goal:</b> Helps us understand what you're building. "
            "We'll generate appropriate node graphs, topics, and code templates.</p>"
            "<p><b>Examples:</b></p>"
            "<ul>"
            "<li>Weather Station: Temperature/humidity/pressure sensors → ROS2 topics</li>"
            "<li>Line Follower: IR sensors → Motor control logic</li>"
            "<li>Obstacle Avoider: Ultrasonic sensors → Navigation decisions</li>"
            "</ul>"
        )


# Continued in next file due to length...
"""
Workflow Wizard Steps 4, 5, 6

Continuation of workflow_wizard.py

Author: RoboShire Team
Phase: 10
"""

from PySide6.QtWidgets import (
    QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QTextEdit, QGroupBox, QMessageBox,
    QCheckBox, QScrollArea, QWidget, QComboBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QTextCharFormat, QColor, QSyntaxHighlighter
from typing import Dict


class Step4_MicrocontrollerCode(QWizardPage):
    """Step 4: Arduino/Microcontroller code generation and AI integration"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 4: Microcontroller Code")
        self.setSubTitle(
            "We'll generate Arduino code templates for your hardware. "
            "You can edit them manually or use AI assistance (GitHub Copilot, Claude, GPT)."
        )

        self._setup_ui()

    def _setup_ui(self):
        # Use scrollable layout
        layout = create_scrollable_page_layout(self)

        # Explanation
        explanation = QLabel(
            "<b>📝 What is this code for?</b><br>"
            "Your microcontroller (Arduino/ESP32) needs code to:<br>"
            "• Read sensors<br>"
            "• Control motors<br>"
            "• Send data to your computer via serial/WiFi<br><br>"
            "We'll generate standard templates based on your hardware selection."
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)

        # Code generation options
        options_group = QGroupBox("Code Generation Options")
        options_layout = QVBoxLayout()

        # Template type
        template_layout = QHBoxLayout()
        template_layout.addWidget(QLabel("Code Template:"))

        self.template_combo = QComboBox()
        self.template_combo.addItems([
            "Standard Arduino (.ino) - Recommended",
            "PlatformIO Project - Advanced",
            "micro-ROS Template - For ROS2 on MCU"
        ])
        self.template_combo.currentIndexChanged.connect(self._on_template_changed)
        template_layout.addWidget(self.template_combo, 1)

        options_layout.addLayout(template_layout)

        # AI Integration option
        self.ai_integration_group = QGroupBox("✨ AI Code Assistant Integration (Optional)")
        ai_layout = QVBoxLayout()

        ai_info = QLabel(
            "Connect an AI assistant to help you write and improve your Arduino code.<br>"
            "This is <b>optional</b> - you can always edit code manually."
        )
        ai_info.setWordWrap(True)
        ai_layout.addWidget(ai_info)

        self.enable_ai_checkbox = QCheckBox("Enable AI code assistance")
        self.enable_ai_checkbox.stateChanged.connect(self._on_ai_enabled_changed)
        ai_layout.addWidget(self.enable_ai_checkbox)

        # AI service selection
        self.ai_service_layout = QHBoxLayout()
        self.ai_service_layout.addWidget(QLabel("AI Service:"))

        self.ai_service_combo = QComboBox()
        self.ai_service_combo.addItems([
            "Select AI service...",
            "GitHub Copilot (VS Code Extension)",
            "Claude API (Anthropic)",
            "OpenAI Codex (GPT-4/3.5)",
            "Local LLM (Ollama/LM Studio)"
        ])
        self.ai_service_combo.setEnabled(False)
        self.ai_service_layout.addWidget(self.ai_service_combo, 1)

        ai_layout.addLayout(self.ai_service_layout)

        # Note about configuring AI later
        self.ai_config_note = QLabel(
            "ℹ️ <b>AI assistance is optional and can be configured later.</b><br>"
            "For now, we'll generate standard Arduino templates. You can integrate "
            "GitHub Copilot or other AI tools in your IDE after the wizard completes."
        )
        self.ai_config_note.setWordWrap(True)
        self.ai_config_note.setStyleSheet(
            "color: #666; font-size: 10px; padding: 10px; "
            "background-color: #f5f5f5; border-radius: 5px; margin-top: 5px;"
        )
        self.ai_config_note.setVisible(False)
        ai_layout.addWidget(self.ai_config_note)

        self.ai_integration_group.setLayout(ai_layout)
        options_layout.addWidget(self.ai_integration_group)

        options_group.setLayout(options_layout)
        layout.addWidget(options_group)

        # Code preview
        preview_group = QGroupBox("Generated Code Preview")
        preview_layout = QVBoxLayout()

        preview_desc = QLabel(
            "Here's what we'll generate for you (example for selected hardware):"
        )
        preview_layout.addWidget(preview_desc)

        self.code_preview = QTextEdit()
        self.code_preview.setReadOnly(True)
        self.code_preview.setFont(QFont("Consolas", 9))
        self.code_preview.setMaximumHeight(200)
        self.code_preview.setStyleSheet(
            "QTextEdit { background-color: #1E1E1E; color: #D4D4D4; }"
        )
        preview_layout.addWidget(self.code_preview)

        preview_group.setLayout(preview_layout)
        layout.addWidget(preview_group)

        # Upload instructions
        upload_group = QGroupBox("📤 How to Upload Code to Your Microcontroller")
        upload_layout = QVBoxLayout()

        upload_steps = QLabel(
            "<b>After the wizard completes:</b><br>"
            "1. Open the generated <code>.ino</code> file in Arduino IDE<br>"
            "2. Connect your microcontroller via USB<br>"
            "3. Select the correct board: <b>Tools → Board → [Your Board]</b><br>"
            "4. Select the correct port: <b>Tools → Port → COM# or /dev/ttyUSB#</b><br>"
            "5. Click the <b>Upload</b> button (→ arrow icon)<br>"
            "6. Wait for \"Done uploading\" message<br><br>"
            "💡 <b>Tip:</b> If upload fails, check USB cable and drivers!"
        )
        upload_steps.setWordWrap(True)
        upload_layout.addWidget(upload_steps)

        upload_group.setLayout(upload_layout)
        layout.addWidget(upload_group)

        layout.addStretch()

        # Note: Layout already set by create_scrollable_page_layout()

        # Initial code preview
        self._update_code_preview()

    def _on_template_changed(self, index):
        """Handle template type change"""
        self.config['code_template_type'] = self.template_combo.currentText()

        # Enable micro-ROS checkbox if selected
        if index == 2:  # micro-ROS template
            self.config['use_micro_ros'] = True

        self._update_code_preview()

    def _on_ai_enabled_changed(self, state):
        """Handle AI enable checkbox"""
        enabled = (state == Qt.Checked)
        self.ai_service_combo.setEnabled(enabled)
        self.ai_config_note.setVisible(enabled)
        self.config['ai_assistance_enabled'] = enabled

    def _update_code_preview(self):
        """Update code preview based on configuration"""
        # Generate example code based on hardware
        code = self._generate_example_code()
        self.code_preview.setPlainText(code)

    def _generate_example_code(self) -> str:
        """Generate example Arduino code"""
        mcu = self.config.get('microcontroller', 'Arduino')
        sensors = self.config.get('sensors', [])
        actuators = self.config.get('actuators', [])
        project_name = self.config.get('project_name', 'my_robot')

        # Basic template
        code = f"""/*
 * {project_name} - Arduino Code
 * Generated by RoboShire Workflow Wizard
 *
 * Microcontroller: {mcu}
 * Sensors: {', '.join(sensors[:3]) if sensors else 'None'}
 * Actuators: {', '.join(actuators[:2]) if actuators else 'None'}
 */

// Pin definitions
"""

        # Add sensor pins
        if 'Ultrasonic' in str(sensors):
            code += "#define TRIG_PIN 9\n#define ECHO_PIN 10\n"

        if 'Temperature' in str(sensors):
            code += "#define DHT_PIN 2\n"

        # Add motor pins
        if actuators:
            code += "#define MOTOR_PIN 5\n"

        code += """
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize pins
"""

        if 'Ultrasonic' in str(sensors):
            code += "  pinMode(TRIG_PIN, OUTPUT);\n  pinMode(ECHO_PIN, INPUT);\n"

        code += """
  Serial.println("System initialized!");
}

void loop() {
  // Read sensors and publish data
"""

        if 'Ultrasonic' in str(sensors):
            code += """
  // Read ultrasonic sensor
  long distance = readUltrasonic();
  Serial.print("Distance: ");
  Serial.println(distance);
"""

        if 'Temperature' in str(sensors):
            code += """
  // Read temperature sensor
  float temp = readTemperature();
  Serial.print("Temperature: ");
  Serial.println(temp);
"""

        code += """
  delay(100); // 10Hz update rate
}

// Sensor reading functions would go here...
"""

        return code

    def get_help_text(self):
        return (
            "<h3>Step 4: Microcontroller Code</h3>"
            "<p><b>Arduino Code:</b> The program that runs on your microcontroller. "
            "It reads sensors, controls motors, and communicates with your computer.</p>"
            "<p><b>Templates:</b> We provide starter code with your sensors/motors already configured. "
            "You can modify it to add your specific logic.</p>"
            "<p><b>AI Assistance (Optional):</b> Connect GitHub Copilot or other AI tools to help write code faster. "
            "The AI can suggest sensor reading code, motor control logic, and more.</p>"
            "<p><b>Upload Process:</b> Use Arduino IDE to compile and upload the code to your board via USB.</p>"
        )


class Step5_ROS2DataFlow(QWizardPage):
    """Step 5: ROS2 data flow explanation and visualization"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 5: ROS2 Data Flow & Nodes")
        self.setSubTitle(
            "Understand how data flows between your robot and ROS2. "
            "We'll create ROS2 nodes to handle communication."
        )

        self._setup_ui()

    def _setup_ui(self):
        # Main layout
        main_layout = QVBoxLayout()

        # Scrollable area for all content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Content widget
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)

        # Explanation section
        explanation_group = QGroupBox("🔄 What is Data Flow?")
        explanation_layout = QVBoxLayout()

        explanation_text = QLabel(
            "<b>In robotics, data flows like this:</b><br><br>"
            "1. <b>Sensors</b> → Collect data (temperature, distance, etc.)<br>"
            "2. <b>Microcontroller</b> → Processes and sends data via serial/WiFi<br>"
            "3. <b>ROS2 Node</b> → Receives data and publishes to ROS2 topics<br>"
            "4. <b>Other Nodes</b> → Subscribe to topics and make decisions<br>"
            "5. <b>Commands</b> → Sent back to microcontroller to control motors<br><br>"
            "<b>Why ROS2?</b><br>"
            "ROS2 helps organize this data flow with <b>topics</b>, <b>nodes</b>, and <b>services</b>. "
            "It's like a messaging system between different parts of your robot."
        )
        explanation_text.setWordWrap(True)
        explanation_layout.addWidget(explanation_text)

        explanation_group.setLayout(explanation_layout)
        layout.addWidget(explanation_group)

        # Visual diagram
        diagram_group = QGroupBox("📊 Your Project's Data Flow")
        diagram_layout = QVBoxLayout()

        diagram_label = QLabel("Here's how data will flow in your project:")
        diagram_layout.addWidget(diagram_label)

        # Create visual diagram
        self.diagram_widget = DataFlowDiagram(self.config)
        diagram_layout.addWidget(self.diagram_widget)

        diagram_group.setLayout(diagram_layout)
        layout.addWidget(diagram_group)

        # Nodes explanation
        nodes_group = QGroupBox("🔵 ROS2 Nodes We'll Create")
        nodes_layout = QVBoxLayout()

        nodes_desc = QLabel(
            "Based on your hardware, we'll generate these ROS2 nodes:"
        )
        nodes_layout.addWidget(nodes_desc)

        # Scrollable nodes list
        nodes_scroll = QScrollArea()
        nodes_scroll.setWidgetResizable(True)
        nodes_scroll.setMaximumHeight(150)

        nodes_scroll_widget = QWidget()
        nodes_scroll_layout = QVBoxLayout(nodes_scroll_widget)

        # Generate node descriptions
        self.node_descriptions = self._generate_node_descriptions()
        for node_desc in self.node_descriptions:
            node_label = QLabel(node_desc)
            node_label.setWordWrap(True)
            node_label.setStyleSheet("padding: 5px; border-left: 3px solid #556B2F; margin: 2px;")
            nodes_scroll_layout.addWidget(node_label)

        nodes_scroll.setWidget(nodes_scroll_widget)
        nodes_layout.addWidget(nodes_scroll)

        nodes_group.setLayout(nodes_layout)
        layout.addWidget(nodes_group)

        # Topics explanation
        topics_group = QGroupBox("📡 ROS2 Topics (Data Streams)")
        topics_layout = QVBoxLayout()

        topics_text = QLabel(
            "<b>What are Topics?</b><br>"
            "Topics are like TV channels - nodes can <b>publish</b> data to a topic, "
            "and other nodes can <b>subscribe</b> to read that data.<br><br>"
            "<b>Example Topics for your project:</b>"
        )
        topics_text.setWordWrap(True)
        topics_layout.addWidget(topics_text)

        self.topics_list = self._generate_topics_list()
        topics_layout.addWidget(self.topics_list)

        topics_group.setLayout(topics_layout)
        layout.addWidget(topics_group)

        layout.addStretch()

        # Info box
        info_box = QLabel(
            "💡 <b>Don't worry if this seems complex!</b><br>"
            "We'll generate all the code automatically. You can modify the data flow later using the Node Graph Editor."
        )
        info_box.setWordWrap(True)
        info_box.setStyleSheet(
            "background-color: #FFF3E0; padding: 10px; border-radius: 5px; color: #E65100;"
        )
        layout.addWidget(info_box)

        # Set content widget and scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def _generate_node_descriptions(self) -> list:
        """Generate descriptions of ROS2 nodes to be created"""
        nodes = []

        project_name = self.config.get('project_name', 'robot')
        sensors = self.config.get('sensors', [])
        actuators = self.config.get('actuators', [])
        use_micro_ros = self.config.get('use_micro_ros', False)

        # Serial bridge node (if not using micro-ROS)
        if not use_micro_ros:
            nodes.append(
                f"<b>1. Serial Bridge Node</b> (<code>{project_name}_serial_bridge</code>)<br>"
                f"   → Reads data from your microcontroller via USB serial<br>"
                f"   → Publishes sensor data to ROS2 topics<br>"
                f"   → Sends motor commands back to microcontroller"
            )

        # Sensor nodes
        sensor_count = 1 if use_micro_ros else 2
        for i, sensor in enumerate(sensors[:3], start=sensor_count):
            sensor_short = sensor.split('(')[0].strip().lower().replace(' ', '_')
            nodes.append(
                f"<b>{i}. {sensor.split('(')[0].strip()} Node</b> (<code>{sensor_short}_node</code>)<br>"
                f"   → Publishes {sensor.split('(')[0].strip()} data to <code>/sensors/{sensor_short}</code>"
            )

        # Control node (if actuators)
        if actuators:
            nodes.append(
                f"<b>{len(nodes) + 1}. Control Node</b> (<code>{project_name}_controller</code>)<br>"
                f"   → Subscribes to sensor data<br>"
                f"   → Makes decisions<br>"
                f"   → Publishes motor commands to <code>/cmd_vel</code> or <code>/motor_control</code>"
            )

        return nodes

    def _generate_topics_list(self) -> QLabel:
        """Generate list of ROS2 topics"""
        sensors = self.config.get('sensors', [])
        topics_text = "<ul>"

        # Sensor topics
        for sensor in sensors[:4]:
            sensor_short = sensor.split('(')[0].strip().lower().replace(' ', '_')
            topics_text += f"<li><code>/sensors/{sensor_short}</code> - {sensor.split('(')[0].strip()} readings</li>"

        # Command topic
        topics_text += "<li><code>/cmd_vel</code> - Motor velocity commands</li>"

        # Status topic
        project_name = self.config.get('project_name', 'robot')
        topics_text += f"<li><code>/{project_name}/status</code> - System status</li>"

        topics_text += "</ul>"

        label = QLabel(topics_text)
        label.setWordWrap(True)
        return label

    def get_help_text(self):
        return (
            "<h3>Step 5: ROS2 Data Flow</h3>"
            "<p><b>Nodes:</b> Independent programs that perform specific tasks (read sensor, control motor, etc.)</p>"
            "<p><b>Topics:</b> Named channels for data. Nodes publish and subscribe to topics.</p>"
            "<p><b>Data Flow Example:</b></p>"
            "<ol>"
            "<li>Ultrasonic sensor reads distance → Arduino</li>"
            "<li>Arduino sends distance via serial → Computer</li>"
            "<li>Serial bridge node receives data → Publishes to /sensors/ultrasonic topic</li>"
            "<li>Control node subscribes to topic → Makes decision (turn if obstacle)</li>"
            "<li>Control node publishes to /cmd_vel → Motor commands sent to Arduino</li>"
            "</ol>"
            "<p><b>We generate all this automatically!</b> You can visualize and modify it later.</p>"
        )


class DataFlowDiagram(QWidget):
    """Visual data flow diagram"""

    def __init__(self, config: Dict):
        super().__init__()
        self.config = config
        self.setMinimumHeight(200)

    def paintEvent(self, event):
        """Draw data flow diagram"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Simple flow diagram
        width = self.width()
        height = self.height()

        # Colors
        sensor_color = QColor(76, 175, 80)  # Green
        mcu_color = QColor(33, 150, 243)    # Blue
        ros_color = QColor(255, 152, 0)     # Orange

        # Draw boxes
        box_width = 150
        box_height = 60
        y_center = height // 2

        # Sensor box
        x1 = 50
        painter.fillRect(x1, y_center - box_height//2, box_width, box_height, sensor_color)
        painter.drawText(x1, y_center - box_height//2, box_width, box_height,
                        Qt.AlignCenter, "Sensors\n(Arduino)")

        # MCU box
        x2 = x1 + box_width + 80
        painter.fillRect(x2, y_center - box_height//2, box_width, box_height, mcu_color)
        painter.drawText(x2, y_center - box_height//2, box_width, box_height,
                        Qt.AlignCenter, "Serial/WiFi\nCommunication")

        # ROS box
        x3 = x2 + box_width + 80
        painter.fillRect(x3, y_center - box_height//2, box_width, box_height, ros_color)
        painter.drawText(x3, y_center - box_height//2, box_width, box_height,
                        Qt.AlignCenter, "ROS2 Nodes\n(Computer)")

        # Draw arrows
        painter.setPen(QColor(100, 100, 100))
        # Arrow 1
        painter.drawLine(x1 + box_width, y_center, x2, y_center)
        painter.drawText(x1 + box_width + 5, y_center - 10, "data →")

        # Arrow 2
        painter.drawLine(x2 + box_width, y_center, x3, y_center)
        painter.drawText(x2 + box_width + 5, y_center - 10, "topics →")


class Step5a_LifecycleNodes(QWizardPage):
    """Step 5a: Lifecycle Nodes - Advanced node management (Optional)"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 5a: Lifecycle Nodes (Advanced - Optional)")
        self.setSubTitle(
            "Learn about ROS2 lifecycle nodes and choose which nodes should be lifecycle-managed. "
            "This is optional and recommended for advanced users."
        )

        self._setup_ui()

    def _setup_ui(self):
        # Main layout
        main_layout = QVBoxLayout()

        # Scrollable area for all content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Content widget
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)

        # Introduction
        intro_group = QGroupBox("🔄 What are Lifecycle Nodes?")
        intro_layout = QVBoxLayout()

        intro_text = QLabel(
            "<b>Lifecycle nodes are a special type of ROS2 node with managed states.</b><br><br>"
            "Instead of just \"running\" or \"stopped\", lifecycle nodes can be in different states "
            "that you control. This gives you better control over initialization, shutdown, and error recovery.<br><br>"
            "<b>Regular Node vs Lifecycle Node:</b><br>"
            "• <b>Regular Node:</b> Starts immediately when launched. Always active until killed.<br>"
            "• <b>Lifecycle Node:</b> Starts in Unconfigured state. You manually transition through states: "
            "Unconfigured → Inactive → Active → (back to Inactive or shutdown)."
        )
        intro_text.setWordWrap(True)
        intro_layout.addWidget(intro_text)

        intro_group.setLayout(intro_layout)
        layout.addWidget(intro_group)

        # State diagram
        diagram_group = QGroupBox("📊 Lifecycle State Diagram")
        diagram_layout = QVBoxLayout()

        # Create visual state diagram
        self.state_diagram = LifecycleStateDiagram(self.config)
        diagram_layout.addWidget(self.state_diagram)

        # State explanations
        states_text = QLabel(
            "<b>States Explained:</b><br><br>"
            "🔵 <b>Unconfigured:</b> Node exists but hasn't loaded configuration. Not ready to run.<br>"
            "   <i>Example: Temperature sensor node created but doesn't know which sensor pin to read.</i><br><br>"
            "🟡 <b>Inactive:</b> Node is configured and ready, but not processing data yet.<br>"
            "   <i>Example: Temperature sensor knows it's on pin 2, but not reading yet.</i><br><br>"
            "🟢 <b>Active:</b> Node is fully operational and processing data.<br>"
            "   <i>Example: Temperature sensor actively reading and publishing data.</i><br><br>"
            "⚫ <b>Finalized:</b> Node has been cleanly shut down. Resources released.<br>"
            "   <i>Example: Temperature sensor stopped, connections closed, ready to exit.</i>"
        )
        states_text.setWordWrap(True)
        diagram_layout.addWidget(states_text)

        diagram_group.setLayout(diagram_layout)
        layout.addWidget(diagram_group)

        # State transitions
        transitions_group = QGroupBox("🔀 State Transitions")
        transitions_layout = QVBoxLayout()

        transitions_text = QLabel(
            "<b>How to change states (CLI commands):</b><br><br>"
            "• <code>ros2 lifecycle set /node_name configure</code> - Unconfigured → Inactive<br>"
            "• <code>ros2 lifecycle set /node_name activate</code> - Inactive → Active<br>"
            "• <code>ros2 lifecycle set /node_name deactivate</code> - Active → Inactive<br>"
            "• <code>ros2 lifecycle set /node_name cleanup</code> - Inactive → Unconfigured<br>"
            "• <code>ros2 lifecycle set /node_name shutdown</code> - Any state → Finalized<br><br>"
            "💡 <b>Tip:</b> You can also use RoboShire's Node Status panel to control lifecycle nodes!"
        )
        transitions_text.setWordWrap(True)
        transitions_layout.addWidget(transitions_text)

        transitions_group.setLayout(transitions_layout)
        layout.addWidget(transitions_group)

        # When to use
        when_group = QGroupBox("❓ When Should You Use Lifecycle Nodes?")
        when_layout = QVBoxLayout()

        when_text = QLabel(
            "<b>✅ Use lifecycle nodes when:</b><br>"
            "• Your node needs complex initialization (sensor calibration, loading config files)<br>"
            "• You want to reconfigure nodes without restarting them<br>"
            "• You need graceful shutdown (save data, close connections properly)<br>"
            "• You're building fault-tolerant systems (restart nodes without killing process)<br>"
            "• Multiple nodes need to start in a coordinated sequence<br><br>"
            "<b>❌ Don't use lifecycle nodes when:</b><br>"
            "• Your node is simple (just publishes/subscribes data)<br>"
            "• You're a beginner and want to keep things simple<br>"
            "• You don't need explicit state control<br><br>"
            "<b>Examples from your project:</b><br>"
            "• <b>Serial Bridge:</b> Good candidate! Needs to open serial port (configure), "
            "verify connection (activate), close cleanly (cleanup).<br>"
            "• <b>Simple Sensor Publisher:</b> Probably not needed. Just publishes data, no complex setup."
        )
        when_text.setWordWrap(True)
        when_layout.addWidget(when_text)

        when_group.setLayout(when_layout)
        layout.addWidget(when_group)

        # Enable lifecycle nodes checkbox
        enable_group = QGroupBox("⚙️ Enable Lifecycle Nodes (Optional)")
        enable_layout = QVBoxLayout()

        self.enable_lifecycle_checkbox = QCheckBox(
            "Enable lifecycle node management for this project"
        )
        self.enable_lifecycle_checkbox.setToolTip(
            "Check this if you want some of your nodes to be lifecycle-managed"
        )
        self.enable_lifecycle_checkbox.stateChanged.connect(self._on_lifecycle_enabled_changed)
        enable_layout.addWidget(self.enable_lifecycle_checkbox)

        # Node selection (initially hidden)
        self.node_selection_widget = QWidget()
        node_selection_layout = QVBoxLayout(self.node_selection_widget)

        node_selection_label = QLabel(
            "<b>Select which nodes should be lifecycle-managed:</b><br>"
            "We recommend choosing nodes that need complex initialization or graceful shutdown."
        )
        node_selection_label.setWordWrap(True)
        node_selection_layout.addWidget(node_selection_label)

        # Checkboxes for each node (will be populated dynamically)
        self.node_checkboxes = []
        self.node_list_layout = QVBoxLayout()
        node_selection_layout.addLayout(self.node_list_layout)

        self.node_selection_widget.setVisible(False)
        enable_layout.addWidget(self.node_selection_widget)

        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)

        # Code example
        example_group = QGroupBox("📝 Code Comparison: Regular vs Lifecycle Node")
        example_layout = QVBoxLayout()

        example_text = QTextEdit()
        example_text.setReadOnly(True)
        example_text.setFont(QFont("Consolas", 9))
        example_text.setMaximumHeight(250)
        example_text.setStyleSheet("QTextEdit { background-color: #1E1E1E; color: #D4D4D4; }")
        example_text.setPlainText("""
# Regular ROS2 Node (Simple)
class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, '/sensors/temp', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        # Sensor starts immediately!

    def publish_temperature(self):
        msg = Float32()
        msg.data = self.read_sensor()
        self.publisher.publish(msg)


# Lifecycle Node (Advanced Control)
class TemperatureSensor(LifecycleNode):
    def __init__(self):
        super().__init__('temperature_sensor')
        # Node created but NOT running yet!

    def on_configure(self, state):
        # Called when: ros2 lifecycle set /temperature_sensor configure
        self.get_logger().info('Configuring sensor on pin 2...')
        self.init_sensor_hardware()  # Setup hardware
        self.publisher = self.create_lifecycle_publisher(Float32, '/sensors/temp', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # Called when: ros2 lifecycle set /temperature_sensor activate
        self.get_logger().info('Activating sensor - starting reads')
        self.publisher.on_activate()  # Enable publishing
        self.timer = self.create_timer(1.0, self.publish_temperature)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        # Called when: ros2 lifecycle set /temperature_sensor deactivate
        self.get_logger().info('Deactivating sensor - pausing reads')
        self.timer.cancel()  # Stop reading
        self.publisher.on_deactivate()  # Disable publishing
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        # Called when: ros2 lifecycle set /temperature_sensor cleanup
        self.get_logger().info('Cleaning up sensor resources')
        self.close_sensor_hardware()  # Release hardware
        return TransitionCallbackReturn.SUCCESS
""")
        example_layout.addWidget(example_text)

        example_note = QLabel(
            "💡 <b>See the difference?</b> Lifecycle nodes give you hooks to control exactly "
            "when things happen: configure, activate, deactivate, cleanup. Regular nodes start immediately."
        )
        example_note.setWordWrap(True)
        example_note.setStyleSheet("color: #666; font-size: 10px; margin-top: 5px;")
        example_layout.addWidget(example_note)

        example_group.setLayout(example_layout)
        layout.addWidget(example_group)

        layout.addStretch()

        # Skip button info
        skip_info = QLabel(
            "ℹ️ <b>This step is completely optional!</b><br>"
            "If you're new to ROS2, you can skip lifecycle nodes for now and use regular nodes. "
            "You can always convert nodes to lifecycle later. Click 'Next' to continue."
        )
        skip_info.setWordWrap(True)
        skip_info.setStyleSheet(
            "background-color: #E3F2FD; padding: 15px; border-radius: 5px; "
            "color: #556B2F; font-size: 11px;"
        )
        layout.addWidget(skip_info)

        # Set content widget and scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def initializePage(self):
        """Initialize page when shown - populate node checkboxes"""
        # Clear existing checkboxes
        for checkbox in self.node_checkboxes:
            checkbox.setParent(None)
            checkbox.deleteLater()
        self.node_checkboxes.clear()

        # Get sensors and actuators from config
        sensors = self.config.get('sensors', [])
        actuators = self.config.get('actuators', [])
        project_name = self.config.get('project_name', 'robot')
        use_micro_ros = self.config.get('use_micro_ros', False)

        # Create checkboxes for potential lifecycle nodes
        if not use_micro_ros:
            # Serial bridge is a good lifecycle candidate
            checkbox = QCheckBox(
                f"🔌 {project_name}_serial_bridge (Serial communication - RECOMMENDED)"
            )
            checkbox.setToolTip(
                "Serial bridge benefits from lifecycle management: "
                "configure = open port, activate = start reading, deactivate = pause, cleanup = close port"
            )
            checkbox.setProperty('node_id', f'{project_name}_serial_bridge')
            self.node_checkboxes.append(checkbox)
            self.node_list_layout.addWidget(checkbox)

        # Add sensor nodes
        for i, sensor in enumerate(sensors[:3], 1):
            sensor_short = sensor.split('(')[0].strip().lower().replace(' ', '_')
            checkbox = QCheckBox(
                f"📡 {sensor_short}_node ({sensor.split('(')[0].strip()})"
            )
            checkbox.setToolTip(
                f"Enable lifecycle management for {sensor.split('(')[0].strip()} sensor node"
            )
            checkbox.setProperty('node_id', f'{sensor_short}_node')
            self.node_checkboxes.append(checkbox)
            self.node_list_layout.addWidget(checkbox)

        # Add control node if actuators exist
        if actuators:
            checkbox = QCheckBox(
                f"🎮 {project_name}_controller (Control logic - RECOMMENDED)"
            )
            checkbox.setToolTip(
                "Control nodes benefit from lifecycle: "
                "configure = load parameters, activate = start control loop, deactivate = safe stop"
            )
            checkbox.setProperty('node_id', f'{project_name}_controller')
            self.node_checkboxes.append(checkbox)
            self.node_list_layout.addWidget(checkbox)

        # Apply saved config if any
        if self.config.get('use_lifecycle_nodes'):
            self.enable_lifecycle_checkbox.setChecked(True)
            lifecycle_node_ids = self.config.get('lifecycle_node_ids', [])
            for checkbox in self.node_checkboxes:
                if checkbox.property('node_id') in lifecycle_node_ids:
                    checkbox.setChecked(True)

    def _on_lifecycle_enabled_changed(self, state):
        """Handle lifecycle enable checkbox"""
        enabled = (state == Qt.Checked)
        self.node_selection_widget.setVisible(enabled)
        self.config['use_lifecycle_nodes'] = enabled

        if enabled:
            # Auto-check recommended nodes (serial_bridge and controller)
            for checkbox in self.node_checkboxes:
                node_id = checkbox.property('node_id')
                if 'serial_bridge' in node_id or 'controller' in node_id:
                    checkbox.setChecked(True)

    def validatePage(self):
        """Save selected lifecycle nodes to config"""
        if self.enable_lifecycle_checkbox.isChecked():
            # Collect selected node IDs
            lifecycle_node_ids = []
            for checkbox in self.node_checkboxes:
                if checkbox.isChecked():
                    lifecycle_node_ids.append(checkbox.property('node_id'))

            self.config['lifecycle_node_ids'] = lifecycle_node_ids
        else:
            self.config['use_lifecycle_nodes'] = False
            self.config['lifecycle_node_ids'] = []

        return True

    def get_help_text(self):
        return (
            "<h3>Step 5a: Lifecycle Nodes (Advanced)</h3>"
            "<p><b>What are they?</b> ROS2 nodes with managed states: Unconfigured, Inactive, Active, Finalized.</p>"
            "<p><b>Why use them?</b> Better control over initialization, reconfiguration, and shutdown.</p>"
            "<p><b>When to use:</b></p>"
            "<ul>"
            "<li>Complex initialization (sensor calibration, hardware setup)</li>"
            "<li>Graceful shutdown (save data, close connections)</li>"
            "<li>Reconfiguration without restart</li>"
            "<li>Coordinated startup of multiple nodes</li>"
            "</ul>"
            "<p><b>When NOT to use:</b> Simple nodes that just publish/subscribe data.</p>"
            "<p><b>Recommendation:</b> If you're new to ROS2, skip this for now. You can always add lifecycle later!</p>"
        )


class LifecycleStateDiagram(QWidget):
    """Visual lifecycle state diagram"""

    def __init__(self, config: Dict):
        super().__init__()
        self.config = config
        self.setMinimumHeight(180)
        self.setMaximumHeight(180)

    def paintEvent(self, event):
        """Draw lifecycle state diagram"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()

        # Colors for states
        unconfigured_color = QColor(100, 100, 100)  # Gray
        inactive_color = QColor(255, 193, 7)        # Yellow
        active_color = QColor(76, 175, 80)          # Green
        finalized_color = QColor(33, 33, 33)        # Dark gray

        # State positions
        box_width = 120
        box_height = 40
        y_top = 20
        y_bottom = height - 60

        # Draw states
        # Unconfigured (top left)
        x1 = 50
        painter.fillRect(x1, y_top, box_width, box_height, unconfigured_color)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x1, y_top, box_width, box_height, Qt.AlignCenter, "Unconfigured")

        # Inactive (top right)
        x2 = x1 + box_width + 100
        painter.fillRect(x2, y_top, box_width, box_height, inactive_color)
        painter.setPen(QColor(0, 0, 0))
        painter.drawText(x2, y_top, box_width, box_height, Qt.AlignCenter, "Inactive")

        # Active (bottom right)
        painter.fillRect(x2, y_bottom, box_width, box_height, active_color)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x2, y_bottom, box_width, box_height, Qt.AlignCenter, "Active")

        # Finalized (bottom left)
        painter.fillRect(x1, y_bottom, box_width, box_height, finalized_color)
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x1, y_bottom, box_width, box_height, Qt.AlignCenter, "Finalized")

        # Draw arrows
        painter.setPen(QColor(100, 100, 100))
        painter.setBrush(QColor(100, 100, 100))

        # Unconfigured -> Inactive (configure)
        painter.drawLine(x1 + box_width, y_top + box_height//2, x2, y_top + box_height//2)
        painter.drawText(x1 + box_width + 10, y_top + box_height//2 - 10, "configure")

        # Inactive -> Active (activate)
        painter.drawLine(x2 + box_width//2, y_top + box_height, x2 + box_width//2, y_bottom)
        painter.drawText(x2 + box_width//2 + 10, y_top + box_height + 30, "activate")

        # Active -> Inactive (deactivate)
        painter.drawLine(x2 + box_width//2 + 15, y_bottom, x2 + box_width//2 + 15, y_top + box_height)
        painter.drawText(x2 + box_width//2 - 70, y_top + box_height + 30, "deactivate")

        # Inactive -> Unconfigured (cleanup)
        painter.drawLine(x2, y_top + box_height//2 + 10, x1 + box_width, y_top + box_height//2 + 10)
        painter.drawText(x1 + box_width + 10, y_top + box_height//2 + 25, "cleanup")

        # Any -> Finalized (shutdown)
        painter.drawLine(x1 + box_width//2, y_top + box_height, x1 + box_width//2, y_bottom)
        painter.drawText(x1 - 30, y_top + box_height + 30, "shutdown")


class Step6_BuildGuide(QWizardPage):
    """Step 6: Build explanation and testing guide"""

    def __init__(self, config: Dict):
        super().__init__()

        self.config = config
        self.setTitle("Step 6: Build & Test Your Robot")
        self.setSubTitle(
            "Learn what 'building' means, how to test your robot, and what to do next."
        )

        self._setup_ui()

    def _setup_ui(self):
        # Main layout
        main_layout = QVBoxLayout()

        # Scrollable area for all content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Content widget
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)

        # What is Build?
        build_explanation = QGroupBox("🔨 What Does 'Build' Mean?")
        build_layout = QVBoxLayout()

        build_text = QLabel(
            "<b>'Building' in ROS2 means:</b><br><br>"
            "<b>1. Compile:</b> Check your Python code for errors<br>"
            "<b>2. Package:</b> Organize code into a ROS2 package<br>"
            "<b>3. Install:</b> Make the package available to run<br><br>"
            "<b>Why do we need to build?</b><br>"
            "• ROS2 needs to know about your new nodes and topics<br>"
            "• Dependencies need to be resolved<br>"
            "• Multiple packages need to work together<br><br>"
            "<b>What tool does the building?</b><br>"
            "<code>colcon</code> - The official ROS2 build tool"
        )
        build_text.setWordWrap(True)
        build_layout.addWidget(build_text)

        build_explanation.setLayout(build_layout)
        layout.addWidget(build_explanation)

        # Build process
        process_group = QGroupBox("⚙️ Build Process (Automated!)")
        process_layout = QVBoxLayout()

        process_text = QLabel(
            "<b>When you click 'Build' in RoboShire:</b><br><br>"
            "1. ✓ ROS2 workspace environment is sourced<br>"
            "2. ✓ <code>colcon build</code> command runs on Ubuntu<br>"
            "3. ✓ Dependencies are checked<br>"
            "4. ✓ Python code is validated<br>"
            "5. ✓ Packages are installed to workspace<br>"
            "6. ✓ You see build output in real-time<br><br>"
            "<b>If build succeeds:</b> ✅ Your code is ready to run!<br>"
            "<b>If build fails:</b> ❌ Error messages tell you what to fix"
        )
        process_text.setWordWrap(True)
        process_layout.addWidget(process_text)

        process_group.setLayout(process_layout)
        layout.addWidget(process_group)

        # Executable explanation
        executable_group = QGroupBox("🚀 What is an Executable? What Runs When You Click 'Run'?")
        executable_layout = QVBoxLayout()

        executable_text = QLabel(
            "<b>Understanding Executables (Beginner-Friendly Explanation):</b><br><br>"
            "<b>What is an executable?</b><br>"
            "An 'executable' is a runnable program - like a Python script that does a specific job. "
            "In ROS2, each node (serial_bridge, temperature_sensor, etc.) is an executable.<br><br>"
            "<b>Example from your project:</b><br>"
            "• <code>temperature_publisher</code> - An executable that reads temperature and publishes it<br>"
            "• <code>serial_bridge_node</code> - An executable that converts serial data to ROS2<br>"
            "• <code>data_logger_node</code> - An executable that logs data to a file<br><br>"
            "<b>What happens when you click 'Run' in RoboShire?</b><br>"
            "1. <b>RoboShire executes command</b> on Ubuntu: "
            "<code>ros2 run your_package temperature_publisher</code><br>"
            "2. <b>ROS2 finds the executable</b> in "
            "<code>~/ros2_ws/install/your_package/lib/your_package/</code><br>"
            "3. <b>Python script starts running</b> in the background on Ubuntu<br>"
            "4. <b>The node connects to ROS2</b> network and starts publishing/subscribing<br>"
            "5. <b>You see output</b> in the Node Status panel (green = running, red = crashed)<br><br>"
            "<b>What's actually running?</b><br>"
            "Your Python script! The executable is just a wrapper that calls:<br>"
            "<code>#!/usr/bin/env python3<br>"
            "import rclpy<br>"
            "from your_package.temperature_publisher import TemperaturePublisher<br>"
            "rclpy.init()<br>"
            "node = TemperaturePublisher()  # YOUR CODE RUNS HERE<br>"
            "rclpy.spin(node)  # Keeps node alive</code><br><br>"
            "<b>How to stop a running executable:</b><br>"
            "• Click the Stop button in Node Status panel<br>"
            "• Or send Ctrl+C signal via terminal<br>"
            "• Or use: <code>ros2 node kill /node_name</code><br><br>"
            "<b>Common beginner questions:</b><br>"
            "❓ <i>\"Where is the executable file?\"</i><br>"
            "→ After build: <code>~/ros2_ws/install/[package]/lib/[package]/[executable_name]</code><br><br>"
            "❓ <i>\"Can I run it directly from command line?\"</i><br>"
            "→ Yes! SSH to Ubuntu and run: <code>source ~/ros2_ws/install/setup.bash && "
            "ros2 run [package] [executable]</code><br><br>"
            "❓ <i>\"What if the executable crashes?\"</i><br>"
            "→ Check logs in the Build Output / Log Viewer tabs for error messages<br><br>"
            "❓ <i>\"Can I run multiple executables at once?\"</i><br>"
            "→ Yes! ROS2 is designed for this. Run each node separately and they'll all communicate."
        )
        executable_text.setWordWrap(True)
        executable_layout.addWidget(executable_text)

        executable_group.setLayout(executable_layout)
        layout.addWidget(executable_group)

        # Testing guide
        test_group = QGroupBox("🧪 How to Test Your Robot")
        test_layout = QVBoxLayout()

        test_intro = QLabel("<b>Follow these steps to test everything works:</b>")
        test_layout.addWidget(test_intro)

        # Generate test steps
        test_steps = self._generate_test_steps()
        test_steps_label = QLabel(test_steps)
        test_steps_label.setWordWrap(True)
        test_layout.addWidget(test_steps_label)

        test_group.setLayout(test_layout)
        layout.addWidget(test_group)

        # Next steps
        next_group = QGroupBox("🎯 After Wizard Completes")
        next_layout = QVBoxLayout()

        next_text = QLabel(
            "<b>You'll be able to:</b><br><br>"
            "1. View generated Arduino code (Step 4)<br>"
            "2. Upload code to your microcontroller<br>"
            "3. See ROS2 node graph (Visual editor)<br>"
            "4. Build the project (Ctrl+B)<br>"
            "5. Run and test your nodes<br>"
            "6. Monitor data in real-time<br>"
            "7. Modify and improve your robot!<br><br>"
            "<b>All code is generated - you just need to test and refine!</b>"
        )
        next_text.setWordWrap(True)
        next_layout.addWidget(next_text)

        next_group.setLayout(next_layout)
        layout.addWidget(next_group)

        layout.addStretch()

        # Finish button info
        finish_info = QLabel(
            "🎉 <b>Click 'Finish' to generate your complete robot project!</b><br>"
            "This will create all code, configuration files, and documentation."
        )
        finish_info.setWordWrap(True)
        finish_info.setStyleSheet(
            "background-color: #E8F5E9; padding: 15px; border-radius: 5px; "
            "color: #2E7D32; font-size: 12px; font-weight: bold;"
        )
        layout.addWidget(finish_info)

        # Set content widget and scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

    def _generate_test_steps(self) -> str:
        """Generate testing steps based on configuration"""
        project_name = self.config.get('project_name', 'robot')
        sensors = self.config.get('sensors', [])

        steps = "<ol>"

        # Step 1: Upload Arduino code
        steps += (
            "<li><b>Upload Arduino Code</b><br>"
            "   → Open <code>" + project_name + ".ino</code> in Arduino IDE<br>"
            "   → Upload to your microcontroller<br>"
            "   → Check for 'Done uploading' message</li><br>"
        )

        # Step 2: Test serial connection
        steps += (
            "<li><b>Test Serial Communication</b><br>"
            "   → Open Arduino Serial Monitor (Tools → Serial Monitor)<br>"
            "   → Set baud rate to 115200<br>"
            "   → You should see sensor data printing</li><br>"
        )

        # Step 3: Build ROS2
        steps += (
            "<li><b>Build ROS2 Package</b><br>"
            "   → In RoboShire: Press Ctrl+B or click Build<br>"
            "   → Wait for 'Build successful' message<br>"
            "   → Check Build Output tab for details</li><br>"
        )

        # Step 4: Run nodes
        steps += (
            "<li><b>Run ROS2 Nodes</b><br>"
            "   → Start serial bridge node first<br>"
            "   → Check Node Status tab - nodes should be green<br>"
            "   → Use <code>ros2 topic list</code> to see topics</li><br>"
        )

        # Step 5: Monitor data
        if sensors:
            sensor_name = sensors[0].split('(')[0].strip().lower().replace(' ', '_')
            steps += (
                "<li><b>Monitor Data</b><br>"
                "   → Open Topic Inspector tab (Ctrl+6)<br>"
                "   → Subscribe to <code>/sensors/" + sensor_name + "</code><br>"
                "   → You should see live sensor data!</li><br>"
            )

        # Step 6: Test full workflow
        steps += (
            "<li><b>Test Complete Workflow</b><br>"
            "   → Move sensors/robot and watch data change<br>"
            "   → Use rqt_graph to visualize connections<br>"
            "   → Everything working? Congratulations! 🎉</li>"
        )

        steps += "</ol>"

        return steps

    def get_help_text(self):
        return (
            "<h3>Step 6: Build & Test</h3>"
            "<p><b>Build Process:</b> colcon compiles and packages your ROS2 code. "
            "Think of it like 'making the code ready to run'.</p>"
            "<p><b>Why Build?</b> ROS2 needs to index your nodes, topics, and dependencies. "
            "Building creates the necessary files.</p>"
            "<p><b>Testing:</b> Follow the checklist to verify each component works:</p>"
            "<ul>"
            "<li>Arduino code uploads successfully</li>"
            "<li>Serial data appears (sensor readings)</li>"
            "<li>ROS2 build completes without errors</li>"
            "<li>Nodes start and stay green</li>"
            "<li>Topics show live data</li>"
            "</ul>"
            "<p><b>Troubleshooting:</b> If something fails, error messages will guide you. "
            "Common issues: USB connection, wrong baud rate, missing dependencies.</p>"
        )
