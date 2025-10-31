"""
Gazebo Plugin Manager Widget

GUI for managing Gazebo plugins without XML editing
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QListWidget, QListWidgetItem, QGroupBox, QLineEdit, QSpinBox,
    QCheckBox, QComboBox, QTextEdit, QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
import logging
from pathlib import Path

from roboshire.backend.gazebo_plugin_generator import GazeboPluginWizard


class GazeboPluginManagerWidget(QWidget):
    """
    GUI for Gazebo plugin configuration

    Features:
    - Select from common plugin types
    - Configure plugin parameters
    - Preview generated XML
    - Add to URDF automatically
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)
        self.wizard = GazeboPluginWizard()
        self.urdf_path = None

        self._setup_ui()
        self._detect_gazebo()

    def _setup_ui(self):
        """Setup user interface"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Gazebo Plugin Manager")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # URDF file selection
        urdf_layout = QHBoxLayout()
        urdf_layout.addWidget(QLabel("URDF File:"))
        self.urdf_path_edit = QLineEdit()
        self.urdf_path_edit.setPlaceholderText("Select URDF file...")
        urdf_layout.addWidget(self.urdf_path_edit)

        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self._browse_urdf)
        urdf_layout.addWidget(browse_btn)
        layout.addLayout(urdf_layout)

        # Main content
        content_layout = QHBoxLayout()

        # Left: Available plugins list
        plugins_group = QGroupBox("Available Plugins")
        plugins_layout = QVBoxLayout()

        self.plugin_list = QListWidget()
        self.plugin_list.addItem("Camera Sensor")
        self.plugin_list.addItem("Lidar Sensor")
        self.plugin_list.addItem("IMU Sensor")
        self.plugin_list.addItem("Differential Drive Controller")
        self.plugin_list.addItem("Depth Camera")
        self.plugin_list.addItem("GPS Sensor")
        self.plugin_list.currentItemChanged.connect(self._on_plugin_selected)
        plugins_layout.addWidget(self.plugin_list)

        plugins_group.setLayout(plugins_layout)
        content_layout.addWidget(plugins_group)

        # Right: Plugin configuration
        config_group = QGroupBox("Plugin Configuration")
        self.config_layout = QVBoxLayout()

        self.config_widget = QWidget()
        self.config_widget_layout = QVBoxLayout(self.config_widget)
        self.config_layout.addWidget(self.config_widget)

        config_group.setLayout(self.config_layout)
        content_layout.addWidget(config_group)

        layout.addLayout(content_layout)

        # Buttons
        button_layout = QHBoxLayout()

        self.add_btn = QPushButton("Add to URDF")
        self.add_btn.clicked.connect(self._add_plugin)
        self.add_btn.setEnabled(False)
        button_layout.addWidget(self.add_btn)

        preview_btn = QPushButton("Preview XML")
        preview_btn.clicked.connect(self._preview_xml)
        button_layout.addWidget(preview_btn)

        clear_btn = QPushButton("Clear All")
        clear_btn.clicked.connect(self._clear_plugins)
        button_layout.addWidget(clear_btn)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # Status
        self.status_label = QLabel("Ready")
        layout.addWidget(self.status_label)

    def _detect_gazebo(self):
        """Detect Gazebo version"""
        version = self.wizard.detect_gazebo_version()
        if version:
            self.status_label.setText(f"Detected: {version}")
        else:
            self.status_label.setText("Gazebo not detected (plugins will still work)")

    def _browse_urdf(self):
        """Browse for URDF file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Select URDF File", "", "URDF Files (*.urdf);;All Files (*)"
        )
        if filename:
            self.urdf_path = filename
            self.urdf_path_edit.setText(filename)
            self.add_btn.setEnabled(True)

    def _on_plugin_selected(self, current, previous):
        """Handle plugin selection"""
        if not current:
            return

        plugin_name = current.text()

        # Clear current configuration
        while self.config_widget_layout.count():
            child = self.config_widget_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        # Add configuration for selected plugin
        if plugin_name == "Camera Sensor":
            self._setup_camera_config()
        elif plugin_name == "Lidar Sensor":
            self._setup_lidar_config()
        elif plugin_name == "IMU Sensor":
            self._setup_imu_config()
        elif plugin_name == "Differential Drive Controller":
            self._setup_diff_drive_config()
        elif plugin_name == "Depth Camera":
            self._setup_depth_camera_config()
        elif plugin_name == "GPS Sensor":
            self._setup_gps_config()

    def _setup_camera_config(self):
        """Setup camera plugin configuration"""
        self.config_widget_layout.addWidget(QLabel("Link Name:"))
        self.camera_link = QLineEdit("camera_link")
        self.config_widget_layout.addWidget(self.camera_link)

        self.config_widget_layout.addWidget(QLabel("Camera Name:"))
        self.camera_name = QLineEdit("camera")
        self.config_widget_layout.addWidget(self.camera_name)

        self.config_widget_layout.addWidget(QLabel("Topic:"))
        self.camera_topic = QLineEdit("/camera/image")
        self.config_widget_layout.addWidget(self.camera_topic)

        self.config_widget_layout.addWidget(QLabel("Update Rate (Hz):"))
        self.camera_rate = QSpinBox()
        self.camera_rate.setRange(1, 120)
        self.camera_rate.setValue(30)
        self.config_widget_layout.addWidget(self.camera_rate)

        self.config_widget_layout.addStretch()

    def _setup_lidar_config(self):
        """Setup lidar plugin configuration"""
        self.config_widget_layout.addWidget(QLabel("Link Name:"))
        self.lidar_link = QLineEdit("lidar_link")
        self.config_widget_layout.addWidget(self.lidar_link)

        self.config_widget_layout.addWidget(QLabel("Sensor Name:"))
        self.lidar_name = QLineEdit("lidar")
        self.config_widget_layout.addWidget(self.lidar_name)

        self.config_widget_layout.addWidget(QLabel("Topic:"))
        self.lidar_topic = QLineEdit("/scan")
        self.config_widget_layout.addWidget(self.lidar_topic)

        self.config_widget_layout.addWidget(QLabel("Update Rate (Hz):"))
        self.lidar_rate = QSpinBox()
        self.lidar_rate.setRange(1, 100)
        self.lidar_rate.setValue(10)
        self.config_widget_layout.addWidget(self.lidar_rate)

        self.config_widget_layout.addStretch()

    def _setup_imu_config(self):
        """Setup IMU plugin configuration"""
        self.config_widget_layout.addWidget(QLabel("Link Name:"))
        self.imu_link = QLineEdit("imu_link")
        self.config_widget_layout.addWidget(self.imu_link)

        self.config_widget_layout.addWidget(QLabel("Sensor Name:"))
        self.imu_name = QLineEdit("imu")
        self.config_widget_layout.addWidget(self.imu_name)

        self.config_widget_layout.addWidget(QLabel("Topic:"))
        self.imu_topic = QLineEdit("/imu/data")
        self.config_widget_layout.addWidget(self.imu_topic)

        self.config_widget_layout.addWidget(QLabel("Update Rate (Hz):"))
        self.imu_rate = QSpinBox()
        self.imu_rate.setRange(1, 200)
        self.imu_rate.setValue(100)
        self.config_widget_layout.addWidget(self.imu_rate)

        self.imu_noise = QCheckBox("Enable Noise")
        self.imu_noise.setChecked(True)
        self.config_widget_layout.addWidget(self.imu_noise)

        self.config_widget_layout.addStretch()

    def _setup_diff_drive_config(self):
        """Setup differential drive plugin configuration"""
        self.config_widget_layout.addWidget(QLabel("Left Joint:"))
        self.diff_left = QLineEdit("left_wheel_joint")
        self.config_widget_layout.addWidget(self.diff_left)

        self.config_widget_layout.addWidget(QLabel("Right Joint:"))
        self.diff_right = QLineEdit("right_wheel_joint")
        self.config_widget_layout.addWidget(self.diff_right)

        self.config_widget_layout.addWidget(QLabel("Command Topic:"))
        self.diff_cmd_topic = QLineEdit("/cmd_vel")
        self.config_widget_layout.addWidget(self.diff_cmd_topic)

        self.config_widget_layout.addWidget(QLabel("Odometry Topic:"))
        self.diff_odom_topic = QLineEdit("/odom")
        self.config_widget_layout.addWidget(self.diff_odom_topic)

        self.config_widget_layout.addStretch()

    def _setup_depth_camera_config(self):
        """Setup depth camera plugin configuration"""
        self.config_widget_layout.addWidget(QLabel("Link Name:"))
        self.depth_link = QLineEdit("depth_camera_link")
        self.config_widget_layout.addWidget(self.depth_link)

        self.config_widget_layout.addWidget(QLabel("Camera Name:"))
        self.depth_name = QLineEdit("depth_camera")
        self.config_widget_layout.addWidget(self.depth_name)

        self.config_widget_layout.addWidget(QLabel("Topic:"))
        self.depth_topic = QLineEdit("/depth_camera")
        self.config_widget_layout.addWidget(self.depth_topic)

        self.config_widget_layout.addStretch()

    def _setup_gps_config(self):
        """Setup GPS plugin configuration"""
        self.config_widget_layout.addWidget(QLabel("Link Name:"))
        self.gps_link = QLineEdit("gps_link")
        self.config_widget_layout.addWidget(self.gps_link)

        self.config_widget_layout.addWidget(QLabel("Sensor Name:"))
        self.gps_name = QLineEdit("gps")
        self.config_widget_layout.addWidget(self.gps_name)

        self.config_widget_layout.addWidget(QLabel("Topic:"))
        self.gps_topic = QLineEdit("/gps/fix")
        self.config_widget_layout.addWidget(self.gps_topic)

        self.config_widget_layout.addStretch()

    def _add_plugin(self):
        """Add plugin to URDF"""
        if not self.urdf_path:
            QMessageBox.warning(self, "No URDF", "Please select a URDF file first")
            return

        current_item = self.plugin_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "No Plugin", "Please select a plugin type")
            return

        plugin_name = current_item.text()

        try:
            # Generate plugin XML based on type
            if plugin_name == "Camera Sensor":
                self.wizard.add_camera_plugin(
                    self.camera_link.text(),
                    self.camera_name.text(),
                    self.camera_topic.text(),
                    self.camera_rate.value()
                )
            elif plugin_name == "Lidar Sensor":
                self.wizard.add_lidar_plugin(
                    self.lidar_link.text(),
                    self.lidar_name.text(),
                    self.lidar_topic.text(),
                    self.lidar_rate.value()
                )
            elif plugin_name == "IMU Sensor":
                self.wizard.add_imu_plugin(
                    self.imu_link.text(),
                    self.imu_name.text(),
                    self.imu_topic.text(),
                    self.imu_rate.value(),
                    self.imu_noise.isChecked()
                )
            elif plugin_name == "Differential Drive Controller":
                self.wizard.add_differential_drive_plugin(
                    self.diff_left.text(),
                    self.diff_right.text(),
                    self.diff_cmd_topic.text(),
                    self.diff_odom_topic.text()
                )
            elif plugin_name == "Depth Camera":
                self.wizard.add_depth_camera_plugin(
                    self.depth_link.text(),
                    self.depth_name.text(),
                    self.depth_topic.text()
                )
            elif plugin_name == "GPS Sensor":
                self.wizard.add_gps_plugin(
                    self.gps_link.text(),
                    self.gps_name.text(),
                    self.gps_topic.text()
                )

            # Read URDF
            with open(self.urdf_path, 'r') as f:
                urdf_content = f.read()

            # Add plugins
            urdf_with_plugins = self.wizard.export_to_urdf(urdf_content)

            # Save
            with open(self.urdf_path, 'w') as f:
                f.write(urdf_with_plugins)

            self.status_label.setText(f"✓ Added {plugin_name} to URDF")
            QMessageBox.information(self, "Success", f"{plugin_name} added to URDF successfully!")

        except Exception as e:
            self.logger.error(f"Failed to add plugin: {e}")
            QMessageBox.critical(self, "Error", f"Failed to add plugin:\n{e}")

    def _preview_xml(self):
        """Preview generated XML"""
        plugins = self.wizard.get_all_plugins()

        if not plugins:
            QMessageBox.information(self, "No Plugins", "No plugins have been added yet")
            return

        # Create preview dialog
        from PySide6.QtWidgets import QDialog, QVBoxLayout
        dialog = QDialog(self)
        dialog.setWindowTitle("Plugin XML Preview")
        dialog.resize(700, 500)

        layout = QVBoxLayout(dialog)

        text_edit = QTextEdit()
        text_edit.setReadOnly(True)
        text_edit.setPlainText('\n'.join([p['xml'] for p in plugins]))
        layout.addWidget(text_edit)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dialog.accept)
        layout.addWidget(close_btn)

        dialog.exec()

    def _clear_plugins(self):
        """Clear all plugins"""
        self.wizard.clear_plugins()
        self.status_label.setText("All plugins cleared")
