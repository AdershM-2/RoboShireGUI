"""
Execution Mode Dialog - Switch between Local and Remote execution

Allows users to change execution mode (Local/Remote) anytime via GUI.
Provides configuration for both local Ubuntu execution and remote SSH execution.

Author: RoboShire Team
Version: 2.3.0 (Ubuntu Standalone)
"""

import os
import logging
from pathlib import Path
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QRadioButton, QGroupBox, QComboBox,
    QFileDialog, QMessageBox, QTextEdit, QFormLayout, QSpinBox
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont

from roboshire.backend.settings import SettingsManager
from roboshire.backend.execution_manager import ExecutionManager, ExecutionMode


class ExecutionModeDialog(QDialog):
    """
    Dialog for configuring and switching execution modes

    Allows switching between:
    - Local execution (run ROS2 on this Ubuntu machine)
    - Remote execution (run ROS2 via SSH on Jetson/RPi/remote Ubuntu)
    """

    mode_changed = Signal(str)  # Emits new mode when changed

    def __init__(self, settings: SettingsManager, parent=None):
        super().__init__(parent)

        self.settings = settings
        self.execution_manager = ExecutionManager.get_instance()

        self.setWindowTitle("Execution Mode Configuration")
        self.setModal(True)
        self.resize(700, 600)

        self._init_ui()
        self._load_current_settings()

    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()

        # Title
        title = QLabel("🔧 Execution Mode Configuration")
        title_font = QFont("Montserrat", 16, QFont.Bold)
        title.setFont(title_font)
        layout.addWidget(title)

        # Description
        desc = QLabel(
            "Choose where RoboShire executes ROS2 commands:\n"
            "• Local - Run on this Ubuntu machine (for development)\n"
            "• Remote - Run via SSH on Jetson, Raspberry Pi, or remote Ubuntu (for deployment)"
        )
        desc.setWordWrap(True)
        layout.addWidget(desc)

        layout.addSpacing(10)

        # Mode selection
        mode_group = QGroupBox("Execution Mode")
        mode_layout = QVBoxLayout()

        self.radio_local = QRadioButton("⚙️ Local Execution (Ubuntu native)")
        self.radio_remote = QRadioButton("🌐 Remote Execution (SSH)")

        mode_layout.addWidget(self.radio_local)
        mode_layout.addWidget(self.radio_remote)

        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # Local execution settings
        self.local_group = QGroupBox("Local Execution Settings")
        local_layout = QFormLayout()

        self.local_workspace_edit = QLineEdit()
        self.local_workspace_browse = QPushButton("Browse...")
        self.local_workspace_browse.clicked.connect(self._browse_local_workspace)

        workspace_layout = QHBoxLayout()
        workspace_layout.addWidget(self.local_workspace_edit)
        workspace_layout.addWidget(self.local_workspace_browse)

        self.local_ros_distro = QComboBox()
        self.local_ros_distro.addItems(["humble", "iron", "jazzy", "rolling"])

        self.local_test_btn = QPushButton("🔍 Test Local ROS2")
        self.local_test_btn.clicked.connect(self._test_local)

        local_layout.addRow("Workspace Path:", workspace_layout)
        local_layout.addRow("ROS2 Distro:", self.local_ros_distro)
        local_layout.addRow("", self.local_test_btn)

        self.local_group.setLayout(local_layout)
        layout.addWidget(self.local_group)

        # Remote execution settings
        self.remote_group = QGroupBox("Remote Execution Settings")
        remote_layout = QFormLayout()

        self.device_type_combo = QComboBox()
        self.device_type_combo.addItems([
            "Ubuntu Server",
            "Jetson Nano",
            "Jetson Xavier",
            "Jetson Orin",
            "Raspberry Pi 4 (Ubuntu)",
            "Raspberry Pi 5 (Ubuntu)"
        ])

        self.ssh_host_edit = QLineEdit()
        self.ssh_user_edit = QLineEdit()
        self.ssh_port_spin = QSpinBox()
        self.ssh_port_spin.setMinimum(1)
        self.ssh_port_spin.setMaximum(65535)
        self.ssh_port_spin.setValue(22)

        self.remote_workspace_edit = QLineEdit()

        self.remote_ros_distro = QComboBox()
        self.remote_ros_distro.addItems(["humble", "iron", "jazzy", "rolling"])

        self.ssh_config_btn = QPushButton("🔐 Configure SSH Authentication...")
        self.ssh_config_btn.clicked.connect(self._configure_ssh)

        self.remote_test_btn = QPushButton("🔍 Test SSH Connection")
        self.remote_test_btn.clicked.connect(self._test_remote)

        remote_layout.addRow("Device Type:", self.device_type_combo)
        remote_layout.addRow("SSH Host (IP):", self.ssh_host_edit)
        remote_layout.addRow("SSH User:", self.ssh_user_edit)
        remote_layout.addRow("SSH Port:", self.ssh_port_spin)
        remote_layout.addRow("Remote Workspace:", self.remote_workspace_edit)
        remote_layout.addRow("ROS2 Distro:", self.remote_ros_distro)
        remote_layout.addRow("", self.ssh_config_btn)
        remote_layout.addRow("", self.remote_test_btn)

        self.remote_group.setLayout(remote_layout)
        layout.addWidget(self.remote_group)

        # Status text
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(100)
        self.status_text.setReadOnly(True)
        layout.addWidget(QLabel("Status:"))
        layout.addWidget(self.status_text)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        self.apply_btn = QPushButton("Apply")
        self.apply_btn.clicked.connect(self._apply_changes)

        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.clicked.connect(self.reject)

        button_layout.addWidget(self.apply_btn)
        button_layout.addWidget(self.cancel_btn)

        layout.addLayout(button_layout)

        self.setLayout(layout)

        # Connect radio buttons
        self.radio_local.toggled.connect(self._on_mode_changed)
        self.radio_remote.toggled.connect(self._on_mode_changed)

    def _load_current_settings(self):
        """Load current settings into UI"""
        # Set current mode
        current_mode = self.settings.get_execution_mode()
        if current_mode == 'local':
            self.radio_local.setChecked(True)
        else:
            self.radio_remote.setChecked(True)

        # Load local settings
        local_config = self.settings.get_local_config()
        self.local_workspace_edit.setText(local_config.workspace_path)
        self.local_ros_distro.setCurrentText(local_config.ros_distro)

        # Load remote settings
        remote_config = self.settings.get_remote_config()

        # Map device type
        device_map = {
            "ubuntu": "Ubuntu Server",
            "jetson_nano": "Jetson Nano",
            "jetson_xavier": "Jetson Xavier",
            "jetson_orin": "Jetson Orin",
            "rpi4": "Raspberry Pi 4 (Ubuntu)",
            "rpi5": "Raspberry Pi 5 (Ubuntu)"
        }
        device_display = device_map.get(remote_config.device_type, "Ubuntu Server")
        self.device_type_combo.setCurrentText(device_display)

        self.ssh_host_edit.setText(remote_config.ssh_host)
        self.ssh_user_edit.setText(remote_config.ssh_user)
        self.ssh_port_spin.setValue(remote_config.ssh_port)
        self.remote_workspace_edit.setText(remote_config.workspace_path)
        self.remote_ros_distro.setCurrentText(remote_config.ros_distro)

        # Update UI state
        self._on_mode_changed()

    def _on_mode_changed(self):
        """Handle mode selection change"""
        is_local = self.radio_local.isChecked()
        self.local_group.setEnabled(is_local)
        self.remote_group.setEnabled(not is_local)

    def _browse_local_workspace(self):
        """Browse for local workspace directory"""
        current_path = self.local_workspace_edit.text()
        if current_path.startswith("~"):
            current_path = os.path.expanduser(current_path)

        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Workspace Directory",
            current_path,
            QFileDialog.ShowDirsOnly
        )

        if directory:
            self.local_workspace_edit.setText(directory)

    def _test_local(self):
        """Test local ROS2 installation"""
        self.status_text.clear()
        self.status_text.append("Testing local ROS2 installation...")

        ros_distro = self.local_ros_distro.currentText()
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"

        if os.path.exists(ros_setup):
            self.status_text.append(f"✅ Found ROS2 {ros_distro} at {ros_setup}")

            # Test ros2 command
            import subprocess
            try:
                result = subprocess.run(
                    ["bash", "-c", f"source {ros_setup} && ros2 --version"],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode == 0:
                    self.status_text.append(f"✅ ROS2 command works: {result.stdout.strip()}")
                    self.status_text.append("✅ Local execution is ready!")
                else:
                    self.status_text.append(f"❌ ROS2 command failed: {result.stderr}")
            except Exception as e:
                self.status_text.append(f"❌ Error testing ROS2: {e}")
        else:
            self.status_text.append(f"❌ ROS2 {ros_distro} not found at {ros_setup}")
            self.status_text.append("Please install ROS2 or select correct distro.")

    def _test_remote(self):
        """Test remote SSH connection"""
        self.status_text.clear()
        self.status_text.append("Testing SSH connection...")

        # This would require SSH manager - simplified for now
        host = self.ssh_host_edit.text()
        user = self.ssh_user_edit.text()
        port = self.ssh_port_spin.value()

        if not host or not user:
            self.status_text.append("❌ Please enter SSH host and user")
            return

        self.status_text.append(f"SSH configuration: {user}@{host}:{port}")
        self.status_text.append("⚠️ Full SSH testing requires authentication setup")
        self.status_text.append("Click 'Configure SSH Authentication' to set up SSH keys/password")

    def _configure_ssh(self):
        """Open SSH configuration wizard"""
        from roboshire.gui.ssh_setup_wizard import SSHSetupWizard

        wizard = SSHSetupWizard(self)
        if wizard.exec() == QDialog.Accepted:
            # Update SSH settings from wizard
            config = wizard.config
            self.ssh_host_edit.setText(config.get('host', ''))
            self.ssh_user_edit.setText(config.get('user', ''))
            self.ssh_port_spin.setValue(config.get('port', 22))

            self.status_text.append("✅ SSH configuration updated")

    def _apply_changes(self):
        """Apply execution mode changes"""
        try:
            # Determine mode
            new_mode = 'local' if self.radio_local.isChecked() else 'remote'

            # Validate local settings
            if new_mode == 'local':
                workspace = self.local_workspace_edit.text()
                if not workspace:
                    QMessageBox.warning(self, "Invalid Configuration",
                                       "Please enter a workspace path")
                    return

                ros_distro = self.local_ros_distro.currentText()

                # Save local config
                self.settings.save_local_config(workspace, ros_distro)

                # Initialize local executor
                self.execution_manager.initialize_local(ros_distro=ros_distro)

            # Validate remote settings
            else:
                host = self.ssh_host_edit.text()
                user = self.ssh_user_edit.text()
                port = self.ssh_port_spin.value()
                workspace = self.remote_workspace_edit.text()

                if not host or not user or not workspace:
                    QMessageBox.warning(self, "Invalid Configuration",
                                       "Please fill in all SSH fields")
                    return

                ros_distro = self.remote_ros_distro.currentText()

                # Map device type back
                device_reverse_map = {
                    "Ubuntu Server": "ubuntu",
                    "Jetson Nano": "jetson_nano",
                    "Jetson Xavier": "jetson_xavier",
                    "Jetson Orin": "jetson_orin",
                    "Raspberry Pi 4 (Ubuntu)": "rpi4",
                    "Raspberry Pi 5 (Ubuntu)": "rpi5"
                }
                device_type = device_reverse_map[self.device_type_combo.currentText()]

                # Save remote config
                self.settings.save_remote_config(
                    device_type=device_type,
                    workspace_path=workspace,
                    ros_distro=ros_distro,
                    ssh_host=host,
                    ssh_user=user,
                    ssh_port=port
                )

                # Note: SSH manager initialization will happen in main_window
                # when SSH connection is established

            # Set execution mode
            self.settings.set_execution_mode(new_mode)
            self.settings.save()

            # Try to switch mode in execution manager
            success = self.execution_manager.set_mode(
                ExecutionMode.LOCAL if new_mode == 'local' else ExecutionMode.REMOTE
            )

            if success:
                QMessageBox.information(
                    self,
                    "Success",
                    f"Execution mode changed to {new_mode.upper()}\n\n"
                    f"RoboShire will now execute ROS2 commands {'locally on this machine' if new_mode == 'local' else 'remotely via SSH'}."
                )

                # Emit signal
                self.mode_changed.emit(new_mode)

                self.accept()
            else:
                QMessageBox.warning(
                    self,
                    "Mode Switch Failed",
                    f"Could not switch to {new_mode.upper()} mode.\n"
                    f"Please check the configuration and try again."
                )

        except Exception as e:
            logging.error(f"Error applying execution mode: {e}")
            QMessageBox.critical(self, "Error", f"Failed to apply changes: {e}")
