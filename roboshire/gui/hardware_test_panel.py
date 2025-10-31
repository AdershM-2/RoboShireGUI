"""
Hardware Testing Panel Widget

Interactive hardware testing and debugging for sensors and actuators.
No separate tools needed - test everything in one place!
"""

import time
from typing import Dict, List, Optional, Any
from collections import deque
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSlider, QGroupBox, QScrollArea, QProgressBar, QTextEdit,
    QGridLayout, QSplitter, QMessageBox, QCheckBox
)
from PySide6.QtCore import Qt, Signal, QTimer, Slot
from PySide6.QtGui import QFont
import logging

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Float32, Int32, Bool, String
    from sensor_msgs.msg import Imu, Temperature as TemperatureMsg
    from geometry_msgs.msg import Twist, Vector3
    from roboshire.integrations.ros2_context import ROS2ContextManager
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Import pyqtgraph for mini graphs
try:
    import pyqtgraph as pg
    import numpy as np
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False


class SensorDisplay(QWidget):
    """Individual sensor display with real-time value and mini graph"""

    def __init__(self, sensor_name: str, topic: str, unit: str = "", parent=None):
        super().__init__(parent)

        self.sensor_name = sensor_name
        self.topic = topic
        self.unit = unit
        self.data_buffer = deque(maxlen=100)  # Last 100 samples
        self.last_update_time = time.time()

        self._setup_ui()

    def _setup_ui(self):
        """Setup sensor display UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Sensor name and value
        header_layout = QHBoxLayout()

        name_label = QLabel(self.sensor_name)
        name_font = QFont()
        name_font.setBold(True)
        name_label.setFont(name_font)
        header_layout.addWidget(name_label)

        header_layout.addStretch()

        self.value_label = QLabel("--")
        value_font = QFont()
        value_font.setPointSize(14)
        value_font.setBold(True)
        self.value_label.setFont(value_font)
        header_layout.addWidget(self.value_label)

        self.status_label = QLabel("✗")
        self.status_label.setStyleSheet("QLabel { color: red; font-size: 16px; }")
        header_layout.addWidget(self.status_label)

        layout.addLayout(header_layout)

        # Signal strength bar
        signal_layout = QHBoxLayout()
        signal_layout.addWidget(QLabel("Signal:"))

        self.signal_bar = QProgressBar()
        self.signal_bar.setMaximum(100)
        self.signal_bar.setValue(0)
        self.signal_bar.setMaximumHeight(15)
        signal_layout.addWidget(self.signal_bar)

        layout.addLayout(signal_layout)

        # Mini graph
        if PYQTGRAPH_AVAILABLE:
            self.graph_widget = pg.PlotWidget()
            self.graph_widget.setMaximumHeight(80)
            self.graph_widget.setBackground('w')
            self.graph_widget.setLabel('left', self.unit)
            self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
            self.graph_widget.setYRange(0, 100)  # Will auto-adjust
            self.plot_curve = self.graph_widget.plot(pen='b')
            layout.addWidget(self.graph_widget)
        else:
            self.graph_widget = None

    def update_value(self, value: float):
        """Update sensor value and graph"""
        # Update value display
        if self.unit:
            self.value_label.setText(f"{value:.2f} {self.unit}")
        else:
            self.value_label.setText(f"{value:.2f}")

        # Update status indicator
        self.status_label.setText("✓")
        self.status_label.setStyleSheet("QLabel { color: green; font-size: 16px; }")

        # Update signal strength based on update frequency
        current_time = time.time()
        time_since_last = current_time - self.last_update_time
        self.last_update_time = current_time

        # Calculate signal strength (assume 10 Hz is 100%)
        if time_since_last > 0:
            frequency = 1.0 / time_since_last
            signal_strength = min(100, int((frequency / 10.0) * 100))
            self.signal_bar.setValue(signal_strength)

        # Update graph
        self.data_buffer.append(value)
        if self.graph_widget and PYQTGRAPH_AVAILABLE:
            data = np.array(self.data_buffer)
            self.plot_curve.setData(data)

            # Auto-scale Y axis
            if len(data) > 0:
                y_min, y_max = data.min(), data.max()
                margin = (y_max - y_min) * 0.1 if y_max != y_min else 1
                self.graph_widget.setYRange(y_min - margin, y_max + margin)

    def set_disconnected(self):
        """Mark sensor as disconnected"""
        self.value_label.setText("--")
        self.status_label.setText("✗")
        self.status_label.setStyleSheet("QLabel { color: red; font-size: 16px; }")
        self.signal_bar.setValue(0)


class MotorController(QWidget):
    """Motor control widget with slider"""

    motor_command = Signal(str, int)  # (motor_name, speed)

    def __init__(self, motor_name: str, parent=None):
        super().__init__(parent)

        self.motor_name = motor_name
        self.current_speed = 0

        self._setup_ui()

    def _setup_ui(self):
        """Setup motor control UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Motor name
        name_label = QLabel(self.motor_name)
        name_font = QFont()
        name_font.setBold(True)
        name_label.setFont(name_font)
        layout.addWidget(name_label)

        # Speed slider
        slider_layout = QHBoxLayout()

        slider_layout.addWidget(QLabel("-255"))

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(-255)
        self.speed_slider.setMaximum(255)
        self.speed_slider.setValue(0)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(50)
        self.speed_slider.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.speed_slider)

        slider_layout.addWidget(QLabel("255"))

        layout.addLayout(slider_layout)

        # Speed value display
        self.speed_label = QLabel("Speed: 0")
        self.speed_label.setAlignment(Qt.AlignCenter)
        speed_font = QFont()
        speed_font.setPointSize(12)
        self.speed_label.setFont(speed_font)
        layout.addWidget(self.speed_label)

        # Control buttons
        button_layout = QHBoxLayout()

        self.test_btn = QPushButton("▶ Test")
        self.test_btn.clicked.connect(self._on_test)
        button_layout.addWidget(self.test_btn)

        self.stop_btn = QPushButton("⏹ Stop")
        self.stop_btn.clicked.connect(self._on_stop)
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; }")
        button_layout.addWidget(self.stop_btn)

        layout.addLayout(button_layout)

    def _on_slider_changed(self, value: int):
        """Handle slider value change"""
        self.current_speed = value
        self.speed_label.setText(f"Speed: {value}")
        self.motor_command.emit(self.motor_name, value)

    def _on_test(self):
        """Run motor test sequence"""
        # Simple test: ramp up to 100, then back to 0
        self.speed_slider.setValue(100)
        QTimer.singleShot(2000, lambda: self.speed_slider.setValue(0))

    def _on_stop(self):
        """Emergency stop"""
        self.speed_slider.setValue(0)

    def get_speed(self) -> int:
        """Get current motor speed"""
        return self.current_speed


class HardwareTestPanel(QWidget):
    """
    Interactive hardware testing and debugging widget

    Features:
    - Live sensor value display with units
    - Motor/servo manual control (sliders)
    - Connection testing
    - Signal quality indicators
    - Quick graphs (last 100 samples)
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)
        self.sensors: Dict[str, SensorDisplay] = {}
        self.motors: Dict[str, MotorController] = {}

        # ROS2 node (created lazily)
        self.ros_node: Optional[Node] = None
        self.subscriptions = []
        self.publishers = {}

        self._setup_ui()

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Hardware Testing Panel")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Main content splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left side: Sensors
        sensors_widget = QWidget()
        sensors_layout = QVBoxLayout(sensors_widget)

        sensors_header = QLabel("Sensors")
        sensors_header_font = QFont()
        sensors_header_font.setBold(True)
        sensors_header.setFont(sensors_header_font)
        sensors_layout.addWidget(sensors_header)

        # Scrollable sensor area
        sensor_scroll = QScrollArea()
        sensor_scroll.setWidgetResizable(True)
        self.sensors_container = QWidget()
        self.sensors_layout = QVBoxLayout(self.sensors_container)
        self.sensors_layout.addStretch()
        sensor_scroll.setWidget(self.sensors_container)
        sensors_layout.addWidget(sensor_scroll)

        splitter.addWidget(sensors_widget)

        # Right side: Actuators
        actuators_widget = QWidget()
        actuators_layout = QVBoxLayout(actuators_widget)

        actuators_header = QLabel("Actuators")
        actuators_header_font = QFont()
        actuators_header_font.setBold(True)
        actuators_header.setFont(actuators_header_font)
        actuators_layout.addWidget(actuators_header)

        # Scrollable actuator area
        actuator_scroll = QScrollArea()
        actuator_scroll.setWidgetResizable(True)
        self.actuators_container = QWidget()
        self.actuators_layout = QVBoxLayout(self.actuators_container)
        self.actuators_layout.addStretch()
        actuator_scroll.setWidget(self.actuators_container)
        actuators_layout.addWidget(actuator_scroll)

        splitter.addWidget(actuators_widget)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter)

        # Bottom: Control buttons and status
        control_layout = QHBoxLayout()

        self.auto_detect_btn = QPushButton("🔍 Auto-Detect Hardware")
        self.auto_detect_btn.clicked.connect(self._auto_detect_hardware)
        control_layout.addWidget(self.auto_detect_btn)

        self.test_connection_btn = QPushButton("🔌 Test Connection")
        self.test_connection_btn.clicked.connect(self._test_connection)
        control_layout.addWidget(self.test_connection_btn)

        self.emergency_stop_btn = QPushButton("🛑 EMERGENCY STOP")
        self.emergency_stop_btn.clicked.connect(self._emergency_stop)
        self.emergency_stop_btn.setStyleSheet(
            "QPushButton { background-color: #d32f2f; color: white; font-weight: bold; padding: 8px; }"
        )
        control_layout.addWidget(self.emergency_stop_btn)

        control_layout.addStretch()

        layout.addLayout(control_layout)

        # Status bar
        status_layout = QHBoxLayout()

        self.status_label = QLabel("Status: Ready")
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()

        self.last_update_label = QLabel("Last update: --")
        status_layout.addWidget(self.last_update_label)

        layout.addLayout(status_layout)

        # ROS2 availability warning
        if not ROS2_AVAILABLE:
            warning = QLabel("⚠️  ROS2 Python libraries not available. Please source ROS2 workspace.")
            warning.setStyleSheet("QLabel { color: orange; font-weight: bold; padding: 5px; }")
            layout.insertWidget(1, warning)

    def _auto_detect_hardware(self):
        """Auto-detect available sensors and actuators from ROS2 topics"""
        if not ROS2_AVAILABLE:
            QMessageBox.warning(self, "ROS2 Not Available",
                              "ROS2 libraries not found. Please source your ROS2 workspace.")
            return

        try:
            # Initialize ROS2 if not already done
            if not self.ros_node:
                self._init_ros()

            # Get all topics
            topic_list = self.ros_node.get_topic_names_and_types()

            sensors_found = []
            actuators_found = []

            for topic, types in topic_list:
                # Detect sensors
                if '/sensors/' in topic or '/imu' in topic or '/temperature' in topic or '/humidity' in topic:
                    sensor_name = topic.split('/')[-1]
                    sensors_found.append((sensor_name, topic, types[0]))

                # Detect actuators
                elif '/cmd_' in topic or '/control/' in topic or '/motor' in topic:
                    actuator_name = topic.split('/')[-1]
                    actuators_found.append((actuator_name, topic, types[0]))

            # Add detected sensors
            for name, topic, msg_type in sensors_found:
                self.add_sensor(name, topic)

            # Add detected actuators (motors)
            for name, topic, msg_type in actuators_found:
                if 'twist' in msg_type.lower() or 'velocity' in msg_type.lower():
                    self.add_motor(name, topic)

            self.status_label.setText(
                f"Status: Found {len(sensors_found)} sensors, {len(actuators_found)} actuators"
            )

            if not sensors_found and not actuators_found:
                QMessageBox.information(self, "No Hardware Found",
                                      "No sensors or actuators detected. Make sure your hardware nodes are running.")

        except Exception as e:
            self.logger.error(f"Auto-detect failed: {e}")
            QMessageBox.critical(self, "Auto-Detect Failed", f"Failed to detect hardware:\n{e}")

    def _init_ros(self):
        """Initialize ROS2 node using shared context manager"""
        if not ROS2_AVAILABLE:
            return

        try:
            # Use shared ROS2 context manager (singleton pattern)
            context_mgr = ROS2ContextManager()
            self.ros_node = context_mgr.create_node('hardware_test_panel')

            # Note: No need to start spinning timer - context manager handles this
            # in background thread with shared executor

            self.logger.info("ROS2 node initialized with shared context")

        except Exception as e:
            self.logger.error(f"Failed to initialize ROS2: {e}")
            QMessageBox.warning(self, "ROS2 Initialization Failed",
                              f"Could not initialize ROS2 node:\n\n{e}\n\n"
                              "Make sure ROS2 is sourced and no other ROS2 issues exist.")


    def add_sensor(self, name: str, topic: str, unit: str = ""):
        """Add a sensor display"""
        if name in self.sensors:
            return  # Already added

        sensor_display = SensorDisplay(name, topic, unit)
        self.sensors[name] = sensor_display

        # Insert before stretch
        count = self.sensors_layout.count()
        self.sensors_layout.insertWidget(count - 1, sensor_display)

        # Create ROS2 subscription
        if self.ros_node:
            try:
                # Create QoS profile for sensor data (reliable, volatile)
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )

                # Use Float32 as default message type
                sub = self.ros_node.create_subscription(
                    Float32,
                    topic,
                    lambda msg, n=name: self._on_sensor_data(n, msg.data),
                    qos_profile
                )
                self.subscriptions.append(sub)
                self.logger.info(f"Subscribed to sensor: {topic}")

            except Exception as e:
                self.logger.error(f"Failed to subscribe to {topic}: {e}")

    def add_motor(self, name: str, topic: str = ""):
        """Add a motor controller"""
        if name in self.motors:
            return  # Already added

        motor_controller = MotorController(name)
        motor_controller.motor_command.connect(self._on_motor_command)
        self.motors[name] = motor_controller

        # Insert before stretch
        count = self.actuators_layout.count()
        self.actuators_layout.insertWidget(count - 1, motor_controller)

        # Create ROS2 publisher for motor commands
        if self.ros_node and topic:
            try:
                # Create QoS profile for command data (reliable, volatile)
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )

                pub = self.ros_node.create_publisher(Twist, topic, qos_profile)
                self.publishers[name] = pub
                self.logger.info(f"Created publisher for motor: {topic}")

            except Exception as e:
                self.logger.error(f"Failed to create publisher for {topic}: {e}")

    def _on_sensor_data(self, sensor_name: str, value: float):
        """Handle sensor data update"""
        if sensor_name in self.sensors:
            self.sensors[sensor_name].update_value(value)
            self.last_update_label.setText(f"Last update: {time.strftime('%H:%M:%S')}")

    def _on_motor_command(self, motor_name: str, speed: int):
        """Handle motor command"""
        if motor_name in self.publishers:
            try:
                # Convert speed (-255 to 255) to velocity (-1.0 to 1.0)
                velocity = speed / 255.0

                msg = Twist()
                msg.linear.x = velocity
                self.publishers[motor_name].publish(msg)

                self.logger.debug(f"Motor {motor_name} command: {speed}")

            except Exception as e:
                self.logger.error(f"Failed to publish motor command: {e}")

    def _test_connection(self):
        """Test all hardware connections"""
        results = []

        # Check ROS2 node
        if not self.ros_node:
            results.append("✗ ROS2 node not initialized")
        else:
            results.append("✓ ROS2 node running")

        # Check sensors
        sensor_count = len(self.sensors)
        results.append(f"Sensors detected: {sensor_count}")

        # Check actuators
        actuator_count = len(self.motors)
        results.append(f"Actuators detected: {actuator_count}")

        # Display results
        result_text = "\n".join(results)
        QMessageBox.information(self, "Connection Test Results", result_text)

        self.status_label.setText("Status: Connection test complete")

    def _emergency_stop(self):
        """Stop all actuators immediately"""
        for motor_name, motor in self.motors.items():
            motor.speed_slider.setValue(0)

        self.status_label.setText("Status: EMERGENCY STOP activated")
        self.logger.warning("Emergency stop activated")

    def cleanup(self):
        """Cleanup when widget is closed"""
        if self.ros_node:
            try:
                # Use context manager to properly destroy node
                context_mgr = ROS2ContextManager()
                context_mgr.destroy_node(self.ros_node)
                self.ros_node = None
                self.logger.info("Hardware test panel ROS2 node destroyed")
            except Exception as e:
                self.logger.error(f"Error during cleanup: {e}")

        # Clear subscriptions and publishers
        self.subscriptions.clear()
        self.publishers.clear()

        # NOTE: Do NOT call rclpy.shutdown() here!
        # The context manager handles ROS2 lifecycle.
        # Other widgets may still need ROS2 to be running.
