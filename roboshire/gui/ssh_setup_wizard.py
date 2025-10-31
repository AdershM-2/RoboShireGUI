"""
SSH Setup Wizard - Step-by-step SSH configuration

Guides users through SSH setup with testing and diagnostics.

Author: RoboShire Team
Version: 0.14.0
"""

from PySide6.QtWidgets import (
    QWizard, QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QTextEdit, QRadioButton, QGroupBox,
    QFileDialog, QProgressBar, QMessageBox, QCheckBox
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont, QIcon
from pathlib import Path
import logging


class SSHSetupWizard(QWizard):
    """
    Step-by-step SSH configuration wizard

    Pages:
    1. Welcome - Explain SSH and why it's needed
    2. Connection Type - Password vs SSH Key
    3. Host Configuration - IP, username, port
    4. Authentication - Password or key file
    5. Test Connection - Verify setup works
    6. Summary - Show configuration and next steps
    """

    ssh_configured = Signal(dict)  # Emits config when complete

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("SSH Setup Wizard")
        self.setWizardStyle(QWizard.ModernStyle)
        self.setOption(QWizard.HaveHelpButton, True)

        # Wizard configuration storage
        self.config = {
            'host': '',
            'user': '',
            'port': 22,
            'auth_type': 'password',  # or 'key'
            'password': '',
            'key_path': '',
            'test_passed': False
        }

        # Add pages
        self.page_welcome = Page1_Welcome()
        self.page_connection_type = Page2_ConnectionType(self.config)
        self.page_host_config = Page3_HostConfig(self.config)
        self.page_auth = Page4_Authentication(self.config)
        self.page_test = Page5_TestConnection(self.config)
        self.page_summary = Page6_Summary(self.config)

        self.addPage(self.page_welcome)
        self.addPage(self.page_connection_type)
        self.addPage(self.page_host_config)
        self.addPage(self.page_auth)
        self.addPage(self.page_test)
        self.addPage(self.page_summary)

        # Set window size
        self.setMinimumSize(700, 500)
        self.resize(800, 600)

        # Connect finish
        self.finished.connect(self._on_wizard_finished)

        # Help button handler
        self.helpRequested.connect(self._show_help)

    def _on_wizard_finished(self, result):
        """Handle wizard completion"""
        if result == QWizard.Accepted:
            if self.config['test_passed']:
                self.ssh_configured.emit(self.config)
            else:
                QMessageBox.warning(
                    self,
                    "Connection Not Tested",
                    "SSH connection was not successfully tested.\n\n"
                    "You may need to troubleshoot the connection."
                )

    def _show_help(self):
        """Show context-sensitive help"""
        current_page = self.currentPage()
        if hasattr(current_page, 'get_help_text'):
            help_text = current_page.get_help_text()
            QMessageBox.information(self, "Help", help_text)
        else:
            QMessageBox.information(
                self,
                "SSH Setup Help",
                "This wizard helps you configure SSH access to your Ubuntu VM.\n\n"
                "SSH is needed to:\n"
                "• Build ROS2 packages on Ubuntu\n"
                "• Run ROS2 nodes remotely\n"
                "• Access Ubuntu tools (RViz, Gazebo)\n\n"
                "Follow each step and test the connection to ensure it works."
            )


class Page1_Welcome(QWizardPage):
    """Welcome page explaining SSH"""

    def __init__(self):
        super().__init__()

        self.setTitle("Welcome to SSH Setup")
        self.setSubTitle("Let's configure SSH access to your Ubuntu system")

        layout = QVBoxLayout()

        # Explanation
        explanation = QLabel(
            "<h3>What is SSH?</h3>"
            "<p>SSH (Secure Shell) allows RoboShire to connect to your Ubuntu VM "
            "to build and run ROS2 code.</p>"

            "<h3>Why do I need it?</h3>"
            "<ul>"
            "<li><b>Build ROS2 packages</b> - colcon build runs on Ubuntu</li>"
            "<li><b>Run ROS2 nodes</b> - Launch nodes remotely</li>"
            "<li><b>Access visualization tools</b> - RViz2, Gazebo, rqt</li>"
            "</ul>"

            "<h3>What you'll need:</h3>"
            "<ul>"
            "<li>Ubuntu VM running (VMware, VirtualBox, WSL2, or native)</li>"
            "<li>IP address of Ubuntu system</li>"
            "<li>Username and password (or SSH key)</li>"
            "</ul>"

            "<p><b>This wizard will guide you through the setup in 5 minutes.</b></p>"
        )
        explanation.setWordWrap(True)
        layout.addWidget(explanation)

        # Optional: Skip SSH setup checkbox
        self.skip_checkbox = QCheckBox("I'll configure SSH manually later")
        layout.addWidget(self.skip_checkbox)

        layout.addStretch()
        self.setLayout(layout)

    def get_help_text(self):
        return (
            "SSH (Secure Shell) is a network protocol that allows secure "
            "communication between computers.\n\n"
            "RoboShire uses SSH to:\n"
            "1. Run commands on your Ubuntu VM\n"
            "2. Build ROS2 packages using colcon\n"
            "3. Launch ROS2 nodes and tools\n\n"
            "You can use either password authentication or SSH keys."
        )


class Page2_ConnectionType(QWizardPage):
    """Choose authentication method"""

    def __init__(self, config):
        super().__init__()
        self.config = config

        self.setTitle("Authentication Method")
        self.setSubTitle("How would you like to authenticate?")

        layout = QVBoxLayout()

        # Authentication type selection
        auth_group = QGroupBox("Choose Authentication Method")
        auth_layout = QVBoxLayout()

        self.password_radio = QRadioButton("Password Authentication (Easier)")
        self.password_radio.setChecked(True)
        self.password_radio.toggled.connect(self._on_auth_changed)
        auth_layout.addWidget(self.password_radio)

        password_desc = QLabel(
            "  • Simple username and password\n"
            "  • Quick to set up\n"
            "  • Good for local VMs"
        )
        password_desc.setStyleSheet("color: #666; margin-left: 20px;")
        auth_layout.addWidget(password_desc)

        auth_layout.addSpacing(10)

        self.key_radio = QRadioButton("SSH Key Authentication (More Secure)")
        self.key_radio.toggled.connect(self._on_auth_changed)
        auth_layout.addWidget(self.key_radio)

        key_desc = QLabel(
            "  • Uses public/private key pair\n"
            "  • More secure (no password transmission)\n"
            "  • Recommended for remote servers"
        )
        key_desc.setStyleSheet("color: #666; margin-left: 20px;")
        auth_layout.addWidget(key_desc)

        auth_group.setLayout(auth_layout)
        layout.addWidget(auth_group)

        # Info box
        info_label = QLabel(
            "<b>Tip:</b> If you're using a local Ubuntu VM (VMware/VirtualBox), "
            "password authentication is usually easier. For remote servers or "
            "production environments, SSH keys are more secure."
        )
        info_label.setWordWrap(True)
        info_label.setStyleSheet("background-color: #e3f2fd; padding: 10px; border-radius: 5px;")
        layout.addWidget(info_label)

        layout.addStretch()
        self.setLayout(layout)

        # Register field for next page decision
        self.registerField("auth_type", self.password_radio)

    def _on_auth_changed(self):
        if self.password_radio.isChecked():
            self.config['auth_type'] = 'password'
        else:
            self.config['auth_type'] = 'key'

    def get_help_text(self):
        return (
            "Authentication Methods:\n\n"
            "Password Authentication:\n"
            "• You enter your Ubuntu username and password\n"
            "• Simple and straightforward\n"
            "• Password is stored securely in Windows Credential Manager\n\n"
            "SSH Key Authentication:\n"
            "• Uses a key file instead of password\n"
            "• More secure (keys can't be guessed)\n"
            "• Requires generating and copying keys\n\n"
            "For most users with a local VM, password authentication is sufficient."
        )


class Page3_HostConfig(QWizardPage):
    """Configure host, username, and port"""

    def __init__(self, config):
        super().__init__()
        self.config = config

        self.setTitle("Ubuntu Connection Details")
        self.setSubTitle("Enter your Ubuntu system's connection information")

        layout = QVBoxLayout()

        # Host input
        host_label = QLabel("<b>Host (IP Address or Hostname):</b>")
        layout.addWidget(host_label)

        self.host_input = QLineEdit()
        self.host_input.setPlaceholderText("e.g., 192.168.1.100 or ubuntu")
        self.host_input.textChanged.connect(self._on_host_changed)
        layout.addWidget(self.host_input)

        host_help = QLabel(
            "💡 To find your Ubuntu IP address, open a terminal and run: <code>hostname -I</code>"
        )
        host_help.setWordWrap(True)
        host_help.setStyleSheet("color: #666; font-size: 10px; margin-bottom: 15px;")
        layout.addWidget(host_help)

        # Username input
        user_label = QLabel("<b>Username:</b>")
        layout.addWidget(user_label)

        self.user_input = QLineEdit()
        self.user_input.setPlaceholderText("e.g., adm20 or ubuntu")
        self.user_input.textChanged.connect(self._on_user_changed)
        layout.addWidget(self.user_input)

        user_help = QLabel(
            "💡 Your Ubuntu username (the name you use to log in)"
        )
        user_help.setWordWrap(True)
        user_help.setStyleSheet("color: #666; font-size: 10px; margin-bottom: 15px;")
        layout.addWidget(user_help)

        # Port input (advanced)
        port_label = QLabel("<b>Port (Advanced):</b>")
        layout.addWidget(port_label)

        self.port_input = QLineEdit("22")
        self.port_input.setPlaceholderText("22 (default)")
        self.port_input.setMaximumWidth(100)
        self.port_input.textChanged.connect(self._on_port_changed)
        layout.addWidget(self.port_input)

        port_help = QLabel(
            "💡 SSH uses port 22 by default. Only change if you configured a different port."
        )
        port_help.setWordWrap(True)
        port_help.setStyleSheet("color: #666; font-size: 10px;")
        layout.addWidget(port_help)

        layout.addStretch()
        self.setLayout(layout)

        # Register mandatory fields
        self.registerField("host*", self.host_input)
        self.registerField("user*", self.user_input)

    def _on_host_changed(self, text):
        self.config['host'] = text

    def _on_user_changed(self, text):
        self.config['user'] = text

    def _on_port_changed(self, text):
        try:
            self.config['port'] = int(text) if text else 22
        except ValueError:
            self.config['port'] = 22

    def get_help_text(self):
        return (
            "Connection Details:\n\n"
            "Host:\n"
            "• IP address (e.g., 192.168.1.100)\n"
            "• Or hostname (e.g., ubuntu, localhost)\n"
            "• For WSL2, use 'localhost'\n\n"
            "Username:\n"
            "• Your Ubuntu username\n"
            "• Find it by running 'whoami' in Ubuntu terminal\n\n"
            "Port:\n"
            "• Default is 22\n"
            "• Only change if you configured SSH on a different port"
        )


class Page4_Authentication(QWizardPage):
    """Password or SSH key input"""

    def __init__(self, config):
        super().__init__()
        self.config = config

        self.setTitle("Authentication Credentials")
        self.setSubTitle("Enter your authentication details")

        self.layout = QVBoxLayout()

        # Password section (shown if password auth selected)
        self.password_group = QGroupBox("Password")
        password_layout = QVBoxLayout()

        password_label = QLabel("<b>Ubuntu Password:</b>")
        password_layout.addWidget(password_label)

        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setPlaceholderText("Enter your Ubuntu password")
        self.password_input.textChanged.connect(self._on_password_changed)
        password_layout.addWidget(self.password_input)

        self.show_password_check = QCheckBox("Show password")
        self.show_password_check.toggled.connect(self._toggle_password_visibility)
        password_layout.addWidget(self.show_password_check)

        password_help = QLabel(
            "🔒 Your password is stored securely using Windows Credential Manager."
        )
        password_help.setWordWrap(True)
        password_help.setStyleSheet("color: #666; font-size: 10px;")
        password_layout.addWidget(password_help)

        self.password_group.setLayout(password_layout)
        self.layout.addWidget(self.password_group)

        # SSH Key section (shown if key auth selected)
        self.key_group = QGroupBox("SSH Key")
        key_layout = QVBoxLayout()

        key_label = QLabel("<b>Private Key File:</b>")
        key_layout.addWidget(key_label)

        key_input_layout = QHBoxLayout()
        self.key_input = QLineEdit()
        self.key_input.setPlaceholderText("Path to private key file (e.g., ~/.ssh/id_rsa)")
        self.key_input.textChanged.connect(self._on_key_changed)
        key_input_layout.addWidget(self.key_input)

        browse_button = QPushButton("Browse...")
        browse_button.clicked.connect(self._browse_key_file)
        key_input_layout.addWidget(browse_button)

        key_layout.addLayout(key_input_layout)

        key_help = QLabel(
            "💡 Don't have an SSH key? Run this in Ubuntu terminal:\n"
            "<code>ssh-keygen -t rsa -b 4096</code>\n"
            "Then copy the public key to authorized_keys:\n"
            "<code>ssh-copy-id user@host</code>"
        )
        key_help.setWordWrap(True)
        key_help.setStyleSheet("color: #666; font-size: 10px;")
        key_layout.addWidget(key_help)

        self.key_group.setLayout(key_layout)
        self.layout.addWidget(self.key_group)

        self.layout.addStretch()
        self.setLayout(self.layout)

        # Initially hide one group based on auth type
        self._update_visibility()

    def initializePage(self):
        """Called when page is shown"""
        self._update_visibility()

    def _update_visibility(self):
        """Show/hide groups based on auth type"""
        if self.config['auth_type'] == 'password':
            self.password_group.show()
            self.key_group.hide()
        else:
            self.password_group.hide()
            self.key_group.show()

    def _on_password_changed(self, text):
        self.config['password'] = text

    def _on_key_changed(self, text):
        self.config['key_path'] = text

    def _toggle_password_visibility(self, checked):
        if checked:
            self.password_input.setEchoMode(QLineEdit.Normal)
        else:
            self.password_input.setEchoMode(QLineEdit.Password)

    def _browse_key_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select SSH Private Key",
            str(Path.home() / ".ssh"),
            "All Files (*)"
        )
        if file_path:
            self.key_input.setText(file_path)

    def validatePage(self):
        """Validate before moving to next page"""
        if self.config['auth_type'] == 'password':
            if not self.config['password']:
                QMessageBox.warning(self, "Missing Password", "Please enter your password.")
                return False
        else:
            if not self.config['key_path']:
                QMessageBox.warning(self, "Missing Key File", "Please select your SSH key file.")
                return False
            if not Path(self.config['key_path']).exists():
                QMessageBox.warning(self, "Key Not Found", f"Key file not found:\n{self.config['key_path']}")
                return False
        return True

    def get_help_text(self):
        if self.config['auth_type'] == 'password':
            return (
                "Password Authentication:\n\n"
                "• Enter the password you use to log into Ubuntu\n"
                "• The password is encrypted and stored securely\n"
                "• You can change it later in Settings\n\n"
                "Security Note:\n"
                "RoboShire stores your password using Windows Credential Manager, "
                "which is encrypted and protected by your Windows user account."
            )
        else:
            return (
                "SSH Key Authentication:\n\n"
                "1. Generate a key pair (if you don't have one):\n"
                "   ssh-keygen -t rsa -b 4096\n\n"
                "2. Copy public key to Ubuntu:\n"
                "   ssh-copy-id user@host\n\n"
                "3. Select your private key file (usually ~/.ssh/id_rsa)\n\n"
                "The private key stays on your computer. Only the public key "
                "is on the Ubuntu system."
            )


class Page5_TestConnection(QWizardPage):
    """Test the SSH connection"""

    def __init__(self, config):
        super().__init__()
        self.config = config

        self.setTitle("Test Connection")
        self.setSubTitle("Let's verify the SSH connection works")

        layout = QVBoxLayout()

        # Instructions
        instructions = QLabel(
            "Click 'Test Connection' to verify your SSH configuration.\n\n"
            "This will check:\n"
            "• Network connectivity (can we reach the host?)\n"
            "• SSH service (is SSH server running?)\n"
            "• Authentication (are credentials correct?)\n"
            "• Command execution (can we run commands?)"
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)

        # Test button
        self.test_button = QPushButton("Test Connection")
        self.test_button.clicked.connect(self._test_connection)
        self.test_button.setMinimumHeight(40)
        layout.addWidget(self.test_button)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("Not tested")
        layout.addWidget(self.progress_bar)

        # Results text
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setMaximumHeight(200)
        self.results_text.setPlaceholderText("Test results will appear here...")
        layout.addWidget(self.results_text)

        layout.addStretch()
        self.setLayout(layout)

        self.test_passed = False

    def _test_connection(self):
        """Test the SSH connection"""
        from roboshire.backend.ssh_subprocess import SSHManagerSubprocess

        self.test_button.setEnabled(False)
        self.results_text.clear()
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Testing...")

        # Create SSH manager
        ssh = SSHManagerSubprocess(
            host=self.config['host'],
            user=self.config['user'],
            port=self.config['port']
        )

        # Run diagnostics
        self._log("🔍 Running SSH diagnostics...\n")
        QTimer.singleShot(100, lambda: self._run_diagnostics(ssh))

    def _run_diagnostics(self, ssh):
        """Run detailed connection diagnostics"""
        diagnostics = ssh.test_connection()

        total_steps = 4
        current_step = 0

        # Step 1: Host reachable
        current_step += 1
        self.progress_bar.setValue(int(current_step / total_steps * 100))

        if diagnostics['host_reachable']:
            self._log("✅ Step 1/4: Host is reachable (ping successful)\n")
        else:
            self._log("❌ Step 1/4: Host is NOT reachable\n")
            self._log(f"   Problem: Cannot ping {self.config['host']}\n")
            self._log("   Solution: Check if Ubuntu VM is running and network is configured\n")
            self._test_failed()
            return

        QTimer.singleShot(500, lambda: self._continue_diagnostics(ssh, diagnostics, current_step, total_steps))

    def _continue_diagnostics(self, ssh, diagnostics, current_step, total_steps):
        """Continue diagnostics - step 2"""
        current_step += 1
        self.progress_bar.setValue(int(current_step / total_steps * 100))

        # Step 2: SSH service
        if diagnostics['ssh_service_running']:
            self._log("✅ Step 2/4: SSH service is running\n")
        else:
            self._log("❌ Step 2/4: SSH service is NOT running\n")
            self._log(f"   Problem: Cannot connect to port {self.config['port']}\n")
            self._log("   Solution: Install SSH server on Ubuntu:\n")
            self._log("   sudo apt install openssh-server\n")
            self._log("   sudo systemctl start ssh\n")
            self._test_failed()
            return

        QTimer.singleShot(500, lambda: self._test_auth(ssh, diagnostics, current_step, total_steps))

    def _test_auth(self, ssh, diagnostics, current_step, total_steps):
        """Test authentication - step 3"""
        current_step += 1
        self.progress_bar.setValue(int(current_step / total_steps * 100))

        # Step 3: Authentication
        if diagnostics['authentication_ok']:
            self._log("✅ Step 3/4: Authentication successful\n")
        else:
            self._log("❌ Step 3/4: Authentication FAILED\n")
            self._log(f"   Problem: {diagnostics.get('error', 'Unknown error')}\n")

            if self.config['auth_type'] == 'password':
                self._log("   Solution: Check username and password\n")
            else:
                self._log("   Solution: Check SSH key file and permissions\n")

            self._test_failed()
            return

        QTimer.singleShot(500, lambda: self._test_command(ssh, current_step, total_steps))

    def _test_command(self, ssh, current_step, total_steps):
        """Test command execution - step 4"""
        current_step += 1
        self.progress_bar.setValue(int(current_step / total_steps * 100))

        # Step 4: Command execution
        try:
            result = ssh.execute_command("echo 'RoboShire Test'", timeout=5.0)
            if result.exit_code == 0 and "RoboShire Test" in result.stdout:
                self._log("✅ Step 4/4: Command execution successful\n")
                if diagnostics.get('latency_ms'):
                    self._log(f"   Latency: {diagnostics['latency_ms']:.0f} ms\n")

                self._log("\n🎉 SUCCESS! SSH connection is working correctly.\n")
                self._test_passed()
            else:
                self._log("❌ Step 4/4: Command execution FAILED\n")
                self._log(f"   Exit code: {result.exit_code}\n")
                self._test_failed()
        except Exception as e:
            self._log("❌ Step 4/4: Command execution FAILED\n")
            self._log(f"   Error: {str(e)}\n")
            self._test_failed()

    def _log(self, text):
        """Append to results text"""
        self.results_text.insertPlainText(text)
        self.results_text.verticalScrollBar().setValue(
            self.results_text.verticalScrollBar().maximum()
        )
        self.results_text.repaint()

    def _test_passed(self):
        """Mark test as passed"""
        self.test_passed = True
        self.config['test_passed'] = True
        self.progress_bar.setValue(100)
        self.progress_bar.setFormat("✅ Test Passed")
        self.progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #4caf50; }")
        self.test_button.setEnabled(True)
        self.test_button.setText("Test Again")

    def _test_failed(self):
        """Mark test as failed"""
        self.test_passed = False
        self.config['test_passed'] = False
        self.progress_bar.setFormat("❌ Test Failed")
        self.progress_bar.setStyleSheet("QProgressBar::chunk { background-color: #f44336; }")
        self.test_button.setEnabled(True)
        self.test_button.setText("Test Again")

    def validatePage(self):
        """Allow proceeding even if test failed (user may want to configure manually)"""
        if not self.test_passed:
            reply = QMessageBox.question(
                self,
                "Connection Not Tested",
                "SSH connection test did not pass.\n\n"
                "Do you want to continue anyway?\n"
                "(You can reconfigure SSH later in Settings)",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            return reply == QMessageBox.Yes
        return True

    def get_help_text(self):
        return (
            "Connection Test:\n\n"
            "The wizard performs these checks:\n\n"
            "1. Ping Test - Can we reach the host?\n"
            "2. SSH Service - Is SSH server running?\n"
            "3. Authentication - Are credentials correct?\n"
            "4. Command Execution - Can we run commands?\n\n"
            "If any step fails, read the error message for solutions.\n\n"
            "Common Issues:\n"
            "• VM not running - Start your Ubuntu VM\n"
            "• Wrong IP - Check with 'hostname -I'\n"
            "• Wrong password - Double-check credentials\n"
            "• Firewall - Ensure port 22 is open"
        )


class Page6_Summary(QWizardPage):
    """Summary and next steps"""

    def __init__(self, config):
        super().__init__()
        self.config = config

        self.setTitle("Setup Complete!")
        self.setSubTitle("SSH configuration summary")

        self.layout = QVBoxLayout()

        # Summary text (will be populated on page show)
        self.summary_text = QTextEdit()
        self.summary_text.setReadOnly(True)
        self.summary_text.setMaximumHeight(250)
        self.layout.addWidget(self.summary_text)

        # Next steps
        next_steps = QLabel(
            "<h3>Next Steps:</h3>"
            "<ol>"
            "<li>Click 'Finish' to save your SSH configuration</li>"
            "<li>Try building a ROS2 package (Build → Build Package)</li>"
            "<li>Run ROS2 nodes remotely (Build → Run)</li>"
            "<li>Launch RViz2 on Ubuntu (Robot → View in RViz2)</li>"
            "</ol>"

            "<p><b>Need to change settings?</b><br>"
            "Go to Tools → Settings → SSH Configuration</p>"
        )
        next_steps.setWordWrap(True)
        self.layout.addWidget(next_steps)

        self.layout.addStretch()
        self.setLayout(self.layout)

    def initializePage(self):
        """Populate summary when page is shown"""
        summary = "<h3>SSH Configuration</h3>"
        summary += "<table cellpadding='5'>"
        summary += f"<tr><td><b>Host:</b></td><td>{self.config['host']}</td></tr>"
        summary += f"<tr><td><b>User:</b></td><td>{self.config['user']}</td></tr>"
        summary += f"<tr><td><b>Port:</b></td><td>{self.config['port']}</td></tr>"
        summary += f"<tr><td><b>Auth Type:</b></td><td>{self.config['auth_type'].capitalize()}</td></tr>"

        if self.config['auth_type'] == 'password':
            summary += f"<tr><td><b>Password:</b></td><td>{'*' * len(self.config['password'])}</td></tr>"
        else:
            summary += f"<tr><td><b>Key File:</b></td><td>{self.config['key_path']}</td></tr>"

        summary += f"<tr><td><b>Test Status:</b></td><td>"
        if self.config['test_passed']:
            summary += "<span style='color: green;'>✅ Passed</span>"
        else:
            summary += "<span style='color: orange;'>⚠️ Not tested or failed</span>"
        summary += "</td></tr>"
        summary += "</table>"

        self.summary_text.setHtml(summary)

    def get_help_text(self):
        return (
            "Configuration Summary:\n\n"
            "Your SSH settings are shown above.\n\n"
            "Click 'Finish' to save the configuration. "
            "RoboShire will use these settings to connect to Ubuntu.\n\n"
            "You can change these settings anytime in:\n"
            "Tools → Settings → SSH Configuration"
        )


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    wizard = SSHSetupWizard()
    wizard.ssh_configured.connect(lambda config: print(f"SSH configured: {config}"))
    wizard.show()
    sys.exit(app.exec())
