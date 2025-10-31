"""
Deployment Setup Wizard - First-launch configuration for RoboShire

Guides users through initial setup to choose between local and remote execution.
Shown only on first launch when no execution configuration exists.

Author: RoboShire Team
Version: 2.3.0 (Ubuntu Standalone)
"""

import os
import logging
import subprocess
from pathlib import Path
from PySide6.QtWidgets import (
    QWizard, QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QTextEdit, QRadioButton, QGroupBox,
    QFileDialog, QComboBox, QSpinBox, QFormLayout, QScrollArea, QWidget
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from roboshire.backend.settings import SettingsManager


class DeploymentSetupWizard(QWizard):
    """
    First-launch wizard for deployment setup

    Pages:
    1. Welcome - Introduction
    2. Mode Selection - Local vs Remote
    3a. Local Setup - Configure local execution
    3b. Remote Setup - Configure SSH
    4. Summary - Show configuration
    """

    def __init__(self, settings: SettingsManager, parent=None):
        super().__init__(parent)

        self.settings = settings

        self.setWindowTitle("RoboShire Deployment Setup")
        self.setWizardStyle(QWizard.ModernStyle)
        self.resize(800, 600)

        # Configuration storage
        self.config = {
            'mode': 'local',  # 'local' or 'remote'
            'local_workspace': os.path.expanduser('~/roboshire_workspace'),
            'local_ros_distro': 'humble',
            'remote_device_type': 'ubuntu',
            'remote_workspace': '/home/ubuntu/roboshire_workspace',
            'remote_ros_distro': 'humble',
            'ssh_host': '',
            'ssh_user': '',
            'ssh_port': 22
        }

        # Add pages
        self.page_welcome = WelcomePage()
        self.page_mode = ModeSelectionPage(self.config)
        self.page_local = LocalSetupPage(self.config)
        self.page_remote = RemoteSetupPage(self.config)
        self.page_summary = SummaryPage(self.config)

        self.addPage(self.page_welcome)
        self.addPage(self.page_mode)
        self.addPage(self.page_local)
        self.addPage(self.page_remote)
        self.addPage(self.page_summary)

    def accept(self):
        """Override accept to save settings before dialog closes"""
        # Save configuration to settings BEFORE closing
        try:
            mode = self.config['mode']

            if mode == 'local':
                self.settings.save_local_config(
                    workspace_path=self.config['local_workspace'],
                    ros_distro=self.config['local_ros_distro']
                )
            else:
                self.settings.save_remote_config(
                    device_type=self.config['remote_device_type'],
                    workspace_path=self.config['remote_workspace'],
                    ros_distro=self.config['remote_ros_distro'],
                    ssh_host=self.config['ssh_host'],
                    ssh_user=self.config['ssh_user'],
                    ssh_port=self.config['ssh_port']
                )

            self.settings.set_execution_mode(mode)
            self.settings.save()

            logging.info(f"Deployment setup completed: {mode} mode")

        except Exception as e:
            logging.error(f"Error saving deployment configuration: {e}")

        # Call parent accept to close dialog
        super().accept()


class WelcomePage(QWizardPage):
    """Welcome page"""

    def __init__(self):
        super().__init__()

        self.setTitle("Welcome to RoboShire on Ubuntu!")

        layout = QVBoxLayout()

        # Large welcome message
        title = QLabel("🎉 Welcome to RoboShire")
        title_font = QFont("Montserrat", 20, QFont.Bold)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        layout.addSpacing(20)

        # Description
        desc = QLabel(
            "RoboShire is a Visual IDE for ROS2 that makes robotics accessible to everyone.\n\n"
            "This wizard will help you set up how RoboShire executes ROS2 commands:\n\n"
            "• <b>Local Execution</b> - Run ROS2 on this Ubuntu machine (recommended for development)\n"
            "• <b>Remote Execution</b> - Run ROS2 on Jetson, Raspberry Pi, or remote Ubuntu (for deployment)\n\n"
            "You can change this setting anytime in Settings → Execution Mode."
        )
        desc.setWordWrap(True)
        desc.setTextFormat(Qt.RichText)
        layout.addWidget(desc)

        layout.addStretch()

        # Ubuntu logo/info
        ubuntu_info = QLabel(
            "🐧 Running on Ubuntu - RoboShire has been optimized for native Ubuntu execution!"
        )
        ubuntu_info.setStyleSheet("color: #E95420; font-weight: bold;")  # Ubuntu orange
        layout.addWidget(ubuntu_info)

        self.setLayout(layout)


class ModeSelectionPage(QWizardPage):
    """Mode selection page"""

    def __init__(self, config: dict):
        super().__init__()

        self.config = config
        self.setTitle("Choose Execution Mode")
        self.setSubTitle("Where should RoboShire execute ROS2 commands?")

        layout = QVBoxLayout()

        # Local option
        self.radio_local = QRadioButton()
        local_group = QGroupBox()
        local_layout = QVBoxLayout()

        local_title = QLabel("⚙️ Local Execution")
        local_title_font = QFont("Montserrat", 14, QFont.Bold)
        local_title.setFont(local_title_font)
        local_layout.addWidget(local_title)

        local_desc = QLabel(
            "Run ROS2 commands directly on this Ubuntu machine.\n\n"
            "✅ Best for development and testing\n"
            "✅ No network required\n"
            "✅ Fastest performance\n"
            "✅ Recommended for beginners"
        )
        local_desc.setWordWrap(True)
        local_layout.addWidget(local_desc)

        local_group.setLayout(local_layout)
        local_group.setStyleSheet("QGroupBox { border: 2px solid #556B2F; border-radius: 5px; }")

        # Remote option
        self.radio_remote = QRadioButton()
        remote_group = QGroupBox()
        remote_layout = QVBoxLayout()

        remote_title = QLabel("🌐 Remote Execution (SSH)")
        remote_title_font = QFont("Montserrat", 14, QFont.Bold)
        remote_title.setFont(remote_title_font)
        remote_layout.addWidget(remote_title)

        remote_desc = QLabel(
            "Run ROS2 commands on a remote device via SSH.\n\n"
            "✅ Deploy to Jetson Nano, Xavier, Orin\n"
            "✅ Deploy to Raspberry Pi 4/5\n"
            "✅ Use remote Ubuntu servers\n"
            "✅ Ideal for production hardware"
        )
        remote_desc.setWordWrap(True)
        remote_layout.addWidget(remote_desc)

        remote_group.setLayout(remote_layout)
        remote_group.setStyleSheet("QGroupBox { border: 2px solid #4A90E2; border-radius: 5px; }")

        # Add to layout with radio buttons
        local_container = QHBoxLayout()
        local_container.addWidget(self.radio_local)
        local_container.addWidget(local_group, 1)
        layout.addLayout(local_container)

        layout.addSpacing(20)

        remote_container = QHBoxLayout()
        remote_container.addWidget(self.radio_remote)
        remote_container.addWidget(remote_group, 1)
        layout.addLayout(remote_container)

        layout.addStretch()

        self.setLayout(layout)

        # Set default
        self.radio_local.setChecked(True)

        # Connect signals to emit completeChanged when selection changes
        self.radio_local.toggled.connect(self.completeChanged)
        self.radio_remote.toggled.connect(self.completeChanged)

    def isComplete(self):
        """Page is complete if either radio button is checked"""
        return self.radio_local.isChecked() or self.radio_remote.isChecked()

    def nextId(self):
        """Determine next page based on selection"""
        if self.radio_local.isChecked():
            self.config['mode'] = 'local'
            return 2  # Local setup page
        else:
            self.config['mode'] = 'remote'
            return 3  # Remote setup page


class LocalSetupPage(QWizardPage):
    """Local execution setup page"""

    def __init__(self, config: dict):
        super().__init__()

        self.config = config
        self.setTitle("Local Execution Setup")
        self.setSubTitle("Configure ROS2 execution on this Ubuntu machine")

        # Main layout for the page
        main_layout = QVBoxLayout(self)

        # Scrollable content area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Content widget
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)

        # Form layout
        form = QFormLayout()

        # Workspace path
        self.workspace_edit = QLineEdit(config['local_workspace'])
        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self._browse_workspace)

        workspace_layout = QHBoxLayout()
        workspace_layout.addWidget(self.workspace_edit)
        workspace_layout.addWidget(browse_btn)

        form.addRow("Workspace Path:", workspace_layout)

        # ROS2 distro
        self.ros_distro_combo = QComboBox()
        self.ros_distro_combo.addItems(["humble", "iron", "jazzy", "rolling"])
        self.ros_distro_combo.setCurrentText(config['local_ros_distro'])

        form.addRow("ROS2 Distribution:", self.ros_distro_combo)

        layout.addLayout(form)

        layout.addSpacing(20)

        # Test button
        test_btn = QPushButton("🔍 Test Local ROS2 Installation")
        test_btn.clicked.connect(self._test_ros2)
        layout.addWidget(test_btn)

        # Status output
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(150)
        layout.addWidget(QLabel("Test Results:"))
        layout.addWidget(self.status_text)

        layout.addStretch()

        # Set content to scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)

    def _browse_workspace(self):
        """Browse for workspace directory"""
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Workspace Directory",
            self.workspace_edit.text(),
            QFileDialog.ShowDirsOnly
        )
        if directory:
            self.workspace_edit.setText(directory)

    def _test_ros2(self):
        """Test local ROS2 installation"""
        self.status_text.clear()
        self.status_text.append("Testing local ROS2 installation...\n")

        ros_distro = self.ros_distro_combo.currentText()
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"

        if os.path.exists(ros_setup):
            self.status_text.append(f"✅ Found ROS2 {ros_distro} at {ros_setup}")

            try:
                # Test ROS2 installation with a simple command
                result = subprocess.run(
                    ["bash", "-c", f"source {ros_setup} && ros2 pkg list | head -n 5"],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode == 0:
                    self.status_text.append(f"✅ ROS2 is working correctly")
                    self.status_text.append(f"   Found {len(result.stdout.strip().splitlines())} packages (showing first 5)")
                    self.status_text.append("\n✅ Local execution is ready!")
                else:
                    self.status_text.append(f"❌ ROS2 command failed: {result.stderr}")
            except Exception as e:
                self.status_text.append(f"❌ Error: {e}")
        else:
            self.status_text.append(f"❌ ROS2 {ros_distro} not found")
            self.status_text.append(f"\nPlease install ROS2 {ros_distro} or select a different distribution.")

    def validatePage(self):
        """Validate before proceeding"""
        self.config['local_workspace'] = self.workspace_edit.text()
        self.config['local_ros_distro'] = self.ros_distro_combo.currentText()
        return True

    def nextId(self):
        """Skip remote page, go to summary"""
        return 4  # Summary page


class RemoteSetupPage(QWizardPage):
    """Remote execution setup page"""

    def __init__(self, config: dict):
        super().__init__()

        self.config = config
        self.setTitle("Remote Execution Setup")
        self.setSubTitle("Configure SSH connection to remote device")

        # Main layout for the page
        main_layout = QVBoxLayout(self)

        # Scrollable content area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Content widget
        content_widget = QWidget()
        layout = QVBoxLayout(content_widget)

        # Form layout
        form = QFormLayout()

        # Device type
        self.device_combo = QComboBox()
        self.device_combo.addItems([
            "Ubuntu Server",
            "Jetson Nano",
            "Jetson Xavier",
            "Jetson Orin",
            "Raspberry Pi 4 (Ubuntu)",
            "Raspberry Pi 5 (Ubuntu)"
        ])
        form.addRow("Device Type:", self.device_combo)

        # SSH settings
        self.ssh_host_edit = QLineEdit()
        self.ssh_user_edit = QLineEdit("ubuntu")
        self.ssh_port_spin = QSpinBox()
        self.ssh_port_spin.setRange(1, 65535)
        self.ssh_port_spin.setValue(22)

        form.addRow("SSH Host (IP):", self.ssh_host_edit)
        form.addRow("SSH User:", self.ssh_user_edit)
        form.addRow("SSH Port:", self.ssh_port_spin)

        # Workspace path
        self.workspace_edit = QLineEdit(config['remote_workspace'])
        form.addRow("Remote Workspace:", self.workspace_edit)

        # ROS2 distro
        self.ros_distro_combo = QComboBox()
        self.ros_distro_combo.addItems(["humble", "iron", "jazzy", "rolling"])
        self.ros_distro_combo.setCurrentText(config['remote_ros_distro'])
        form.addRow("ROS2 Distribution:", self.ros_distro_combo)

        layout.addLayout(form)

        layout.addSpacing(20)

        # Note about SSH authentication
        note = QLabel(
            "⚠️ <b>Note:</b> After completing this wizard, you'll need to configure SSH authentication "
            "(password or SSH key) via Settings → SSH Configuration."
        )
        note.setWordWrap(True)
        note.setTextFormat(Qt.RichText)
        note.setStyleSheet("background-color: #FFF9E6; padding: 10px; border-radius: 5px;")
        layout.addWidget(note)

        layout.addStretch()

        # Set content to scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)

    def validatePage(self):
        """Validate before proceeding"""
        if not self.ssh_host_edit.text() or not self.ssh_user_edit.text():
            return False

        # Map device type
        device_map = {
            "Ubuntu Server": "ubuntu",
            "Jetson Nano": "jetson_nano",
            "Jetson Xavier": "jetson_xavier",
            "Jetson Orin": "jetson_orin",
            "Raspberry Pi 4 (Ubuntu)": "rpi4",
            "Raspberry Pi 5 (Ubuntu)": "rpi5"
        }

        self.config['remote_device_type'] = device_map[self.device_combo.currentText()]
        self.config['ssh_host'] = self.ssh_host_edit.text()
        self.config['ssh_user'] = self.ssh_user_edit.text()
        self.config['ssh_port'] = self.ssh_port_spin.value()
        self.config['remote_workspace'] = self.workspace_edit.text()
        self.config['remote_ros_distro'] = self.ros_distro_combo.currentText()

        return True

    def nextId(self):
        """Skip local page, go to summary"""
        return 4  # Summary page


class SummaryPage(QWizardPage):
    """Summary page"""

    def __init__(self, config: dict):
        super().__init__()

        self.config = config
        self.setTitle("Setup Complete!")
        self.setSubTitle("Review your configuration")

        layout = QVBoxLayout()

        # Summary text
        self.summary_text = QTextEdit()
        self.summary_text.setReadOnly(True)
        layout.addWidget(self.summary_text)

        # Info
        info = QLabel(
            "✅ You can change these settings anytime in <b>Settings → Execution Mode</b>"
        )
        info.setWordWrap(True)
        info.setTextFormat(Qt.RichText)
        layout.addWidget(info)

        self.setLayout(layout)

    def initializePage(self):
        """Update summary when page is shown"""
        mode = self.config['mode']

        summary = f"<h2>Deployment Configuration</h2>"
        summary += f"<p><b>Execution Mode:</b> {mode.upper()}</p>"

        if mode == 'local':
            summary += f"<h3>Local Execution Settings</h3>"
            summary += f"<p><b>Workspace:</b> {self.config['local_workspace']}</p>"
            summary += f"<p><b>ROS2 Distribution:</b> {self.config['local_ros_distro']}</p>"
            summary += f"<p>RoboShire will execute ROS2 commands directly on this Ubuntu machine.</p>"
        else:
            summary += f"<h3>Remote Execution Settings</h3>"
            summary += f"<p><b>Device Type:</b> {self.config['remote_device_type']}</p>"
            summary += f"<p><b>SSH Host:</b> {self.config['ssh_host']}</p>"
            summary += f"<p><b>SSH User:</b> {self.config['ssh_user']}</p>"
            summary += f"<p><b>SSH Port:</b> {self.config['ssh_port']}</p>"
            summary += f"<p><b>Remote Workspace:</b> {self.config['remote_workspace']}</p>"
            summary += f"<p><b>ROS2 Distribution:</b> {self.config['remote_ros_distro']}</p>"
            summary += f"<p>RoboShire will execute ROS2 commands via SSH on the remote device.</p>"
            summary += f"<p><b>⚠️ Next Step:</b> Configure SSH authentication via Settings → SSH Configuration</p>"

        self.summary_text.setHtml(summary)
