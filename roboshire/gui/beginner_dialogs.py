"""
Beginner-Friendly Dialog Boxes with Clear Explanations

Provides enhanced dialog boxes that explain concepts like:
- Package names
- Build folders
- Output directories
- Executables

Author: RoboShire Team
Phase: 10 (UX Polish)
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QTextEdit, QComboBox, QGroupBox, QFileDialog,
    QDialogButtonBox, QListWidget, QListWidgetItem, QScrollArea, QWidget
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from typing import Optional, List
from pathlib import Path
import os


def scan_package_executables(package_name: str, workspace_path: str = "workspace") -> List[str]:
    """
    Scan a ROS2 package for available executables

    Args:
        package_name: Name of the package to scan
        workspace_path: Path to workspace (default: "workspace")

    Returns:
        List of executable names found in the package
    """
    executables = []

    # Try workspace/install/package_name/lib/package_name/
    install_path = Path(workspace_path) / "install" / package_name / "lib" / package_name

    if install_path.exists() and install_path.is_dir():
        for item in install_path.iterdir():
            if item.is_file():
                # Check if file is executable (on Unix) or any file on Windows
                if os.name == 'posix':
                    if os.access(item, os.X_OK):
                        executables.append(item.name)
                else:
                    # On Windows, just list all files (they'll be executables after transfer to VM)
                    if not item.name.startswith('.') and not item.name.endswith('.pyc'):
                        executables.append(item.name)

    # Also check setup.py for entry_points if install/ doesn't exist yet
    if not executables:
        setup_py_path = Path(workspace_path) / "src" / package_name / "setup.py"
        if setup_py_path.exists():
            try:
                with open(setup_py_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    # Parse entry_points console_scripts
                    if "'console_scripts':" in content or '"console_scripts":' in content:
                        import re
                        # Extract only the console_scripts section
                        console_scripts_match = re.search(
                            r"['\"]console_scripts['\"]\s*:\s*\[(.*?)\]",
                            content,
                            re.DOTALL
                        )
                        if console_scripts_match:
                            scripts_section = console_scripts_match.group(1)
                            # Extract executable names (before '=' in each entry)
                            matches = re.findall(r"['\"]([^'\"]+)\s*=\s*", scripts_section)
                            executables.extend([m.strip() for m in matches if m and not m.startswith('_')])
            except Exception:
                pass  # Ignore parsing errors

    # Remove duplicates and sort
    return sorted(list(set(executables)))


class PackageNameDialog(QDialog):
    """
    Dialog for entering ROS2 package name with beginner-friendly explanation
    """

    def __init__(self, parent=None, default_name: str = "my_robot_pkg"):
        super().__init__(parent)

        self.setWindowTitle("Enter Package Name")
        self.resize(600, 500)

        self.package_name = default_name

        self._setup_ui(default_name)

    def _setup_ui(self, default_name: str):
        layout = QVBoxLayout()

        # Title
        title = QLabel("<h2>📦 What is a ROS2 Package?</h2>")
        layout.addWidget(title)

        # Explanation
        explanation_group = QGroupBox("🎓 Beginner Explanation")
        explanation_layout = QVBoxLayout()

        explanation = QLabel(
            "<b>A ROS2 package is like a folder that contains all your robot code.</b><br><br>"
            "Think of it like a <b>project folder</b> that holds:<br>"
            "• Python files (your node code)<br>"
            "• Configuration files (package.xml, setup.py)<br>"
            "• Launch files (to start multiple nodes)<br>"
            "• Resources (models, parameters, etc.)<br><br>"
            "<b>Example packages:</b><br>"
            "• <code>my_robot_control</code> - Controls your robot motors<br>"
            "• <code>weather_station_pkg</code> - Reads and publishes sensor data<br>"
            "• <code>simple_pubsub_pkg</code> - Basic publisher/subscriber example"
        )
        explanation.setWordWrap(True)
        explanation.setStyleSheet(
            "QLabel { background-color: #FFF9E6; padding: 15px; border-radius: 5px; }"
        )
        explanation_layout.addWidget(explanation)

        explanation_group.setLayout(explanation_layout)
        layout.addWidget(explanation_group)

        # Package name input
        input_group = QGroupBox("✏️ Enter Your Package Name")
        input_layout = QVBoxLayout()

        input_label = QLabel(
            "<b>Choose a descriptive name that represents what your robot does.</b>"
        )
        input_label.setWordWrap(True)
        input_layout.addWidget(input_label)

        self.name_input = QLineEdit()
        self.name_input.setText(default_name)
        self.name_input.setPlaceholderText("my_robot_pkg")
        self.name_input.setFont(QFont("Consolas", 11))
        self.name_input.textChanged.connect(self._validate_name)
        input_layout.addWidget(self.name_input)

        # Validation message
        self.validation_label = QLabel("")
        self.validation_label.setWordWrap(True)
        input_layout.addWidget(self.validation_label)

        # Naming rules
        rules = QLabel(
            "<b>📋 Naming rules:</b><br>"
            "✅ Use lowercase letters, numbers, and underscores<br>"
            "✅ Must start with a letter<br>"
            "✅ No spaces or special characters<br>"
            "❌ Avoid: <code>My-Robot</code>, <code>robot pkg</code>, <code>123robot</code><br>"
            "✅ Good: <code>my_robot_pkg</code>, <code>robot_control</code>, <code>sensor_node</code>"
        )
        rules.setWordWrap(True)
        rules.setStyleSheet("color: #666; font-size: 10px; margin-top: 5px;")
        input_layout.addWidget(rules)

        input_group.setLayout(input_layout)
        layout.addWidget(input_group)

        # What happens next
        next_group = QGroupBox("🔜 What Happens Next?")
        next_layout = QVBoxLayout()

        next_text = QLabel(
            "After you click OK, RoboShire will:<br>"
            "1. <b>Create a folder</b> with your package name in <code>workspace/src/</code><br>"
            "2. <b>Generate Python files</b> for each node in your graph<br>"
            "3. <b>Create package.xml</b> (describes your package)<br>"
            "4. <b>Create setup.py</b> (tells ROS2 how to install)<br><br>"
            "💡 You'll be able to see all these files in the output directory!"
        )
        next_text.setWordWrap(True)
        next_text.setStyleSheet(
            "background-color: #F0F5E6; padding: 10px; border-radius: 5px; color: #556B2F;"
        )
        next_layout.addWidget(next_text)

        next_group.setLayout(next_layout)
        layout.addWidget(next_group)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        self.ok_button = QPushButton("OK - Generate Code")
        self.ok_button.clicked.connect(self.accept)
        self.ok_button.setDefault(True)
        button_layout.addWidget(self.ok_button)

        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)

        layout.addLayout(button_layout)

        self.setLayout(layout)

        # Initial validation
        self._validate_name(default_name)

    def _validate_name(self, name: str):
        """Validate package name and show feedback"""
        import re

        if not name:
            self.validation_label.setText("⚠️ Package name cannot be empty")
            self.validation_label.setStyleSheet("color: #FF9800;")
            self.ok_button.setEnabled(False)
            return

        # Check naming rules
        if not re.match(r'^[a-z][a-z0-9_]*$', name):
            self.validation_label.setText(
                "❌ Invalid name! Must start with lowercase letter and contain only "
                "lowercase letters, numbers, and underscores."
            )
            self.validation_label.setStyleSheet("color: #F44336;")
            self.ok_button.setEnabled(False)
            return

        # Valid!
        self.validation_label.setText(f"✅ Valid package name: <code>{name}</code>")
        self.validation_label.setStyleSheet("color: #4CAF50;")
        self.ok_button.setEnabled(True)
        self.package_name = name

    def get_package_name(self) -> str:
        """Get the entered package name"""
        return self.package_name


class OutputDirectoryDialog(QDialog):
    """
    Dialog for selecting output directory with explanation
    """

    def __init__(self, parent=None, default_dir: str = "workspace/src"):
        super().__init__(parent)

        self.setWindowTitle("Select Output Directory")
        self.resize(650, 550)

        self.output_directory = default_dir

        self._setup_ui(default_dir)

    def _setup_ui(self, default_dir: str):
        layout = QVBoxLayout()

        # Title
        title = QLabel("<h2>📁 Where Should We Save the Generated Code?</h2>")
        layout.addWidget(title)

        # Explanation
        explanation_group = QGroupBox("🎓 Understanding the Output Directory")
        explanation_layout = QVBoxLayout()

        explanation = QLabel(
            "<b>The output directory is where RoboShire will create your package folder.</b><br><br>"
            "<b>ROS2 Workspace Structure:</b><br>"
            "<code>workspace/</code> - Your main workspace folder<br>"
            "├── <code>src/</code> - <b>Source code goes here</b> (SELECT THIS!)<br>"
            "│   ├── <code>my_robot_pkg/</code> - Your package folder (created automatically)<br>"
            "│   ├── <code>another_pkg/</code> - Another package<br>"
            "├── <code>build/</code> - Temporary build files (auto-generated, don't edit)<br>"
            "├── <code>install/</code> - Final compiled code (auto-generated)<br>"
            "└── <code>log/</code> - Build logs (auto-generated)<br><br>"
            "<b>⚠️ Important:</b> Always select the <code>src/</code> folder inside your workspace!"
        )
        explanation.setWordWrap(True)
        explanation.setStyleSheet(
            "QLabel { background-color: #FFF9E6; padding: 15px; border-radius: 5px; "
            "font-family: 'Consolas', monospace; }"
        )
        explanation_layout.addWidget(explanation)

        explanation_group.setLayout(explanation_layout)
        layout.addWidget(explanation_group)

        # Directory selection
        select_group = QGroupBox("📂 Select Directory")
        select_layout = QVBoxLayout()

        select_label = QLabel(
            "<b>Current selection:</b><br>"
            "This is where your package folder will be created."
        )
        select_label.setWordWrap(True)
        select_layout.addWidget(select_label)

        # Path display and browse
        path_layout = QHBoxLayout()

        self.path_input = QLineEdit()
        self.path_input.setText(default_dir)
        self.path_input.setReadOnly(True)
        self.path_input.setFont(QFont("Consolas", 10))
        path_layout.addWidget(self.path_input, 1)

        browse_button = QPushButton("Browse...")
        browse_button.clicked.connect(self._browse_directory)
        path_layout.addWidget(browse_button)

        select_layout.addLayout(path_layout)

        # Tips
        tips = QLabel(
            "💡 <b>Tips:</b><br>"
            "• If you don't have a workspace yet, create one first!<br>"
            "• Default location: <code>~/ros2_ws/src</code> (on Ubuntu VM)<br>"
            "• On Windows, RoboShire will transfer files to VM via SSH"
        )
        tips.setWordWrap(True)
        tips.setStyleSheet("color: #666; font-size: 10px; margin-top: 10px;")
        select_layout.addWidget(tips)

        select_group.setLayout(select_layout)
        layout.addWidget(select_group)

        # What gets created
        created_group = QGroupBox("📋 What Will Be Created?")
        created_layout = QVBoxLayout()

        created_text = QLabel(
            "RoboShire will create this structure in your selected directory:<br><br>"
            "<code>your_package_name/</code><br>"
            "├── <code>package.xml</code> - Package metadata<br>"
            "├── <code>setup.py</code> - Installation instructions<br>"
            "├── <code>setup.cfg</code> - Configuration<br>"
            "├── <code>your_package_name/</code> - Python module<br>"
            "│   ├── <code>__init__.py</code><br>"
            "│   ├── <code>node1.py</code> - Your first node<br>"
            "│   ├── <code>node2.py</code> - Your second node<br>"
            "│   └── ... (more nodes)<br>"
            "└── <code>resource/</code> - Resource files"
        )
        created_text.setWordWrap(True)
        created_text.setFont(QFont("Consolas", 9))
        created_text.setStyleSheet(
            "background-color: #E8F5E9; padding: 10px; border-radius: 5px;"
        )
        created_layout.addWidget(created_text)

        created_group.setLayout(created_layout)
        layout.addWidget(created_group)

        # Buttons
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.setLayout(layout)

    def _browse_directory(self):
        """Browse for directory"""
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Output Directory (usually workspace/src)",
            self.path_input.text()
        )

        if directory:
            self.path_input.setText(directory)
            self.output_directory = directory

    def get_directory(self) -> str:
        """Get the selected directory"""
        return self.output_directory


class BuildPackageDialog(QDialog):
    """
    Dialog for selecting which package to build with explanation
    """

    def __init__(self, parent=None, packages: List[str] = None):
        super().__init__(parent)

        self.setWindowTitle("Select Package to Build")
        self.resize(500, 400)  # Smaller, more compact size

        self.packages = packages or []
        self.selected_package = "All Packages" if packages else None

        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout()

        # Title
        title = QLabel("<h3>🔨 Build Package</h3>")
        layout.addWidget(title)

        # Compact explanation
        build_explanation = QLabel(
            "<b>Building</b> compiles your ROS2 code so it can run.<br>"
            "This creates executables in <code>install/</code> from source in <code>src/</code>."
        )
        build_explanation.setWordWrap(True)
        build_explanation.setStyleSheet(
            "background-color: #FFF9E6; padding: 10px; border-radius: 5px; font-size: 10px;"
        )
        layout.addWidget(build_explanation)

        # Package selection
        select_group = QGroupBox("📦 Select Package to Build")
        select_layout = QVBoxLayout()

        select_label = QLabel(
            f"Found <b>{len(self.packages)} package(s)</b> in workspace."
        )
        select_label.setWordWrap(True)
        select_layout.addWidget(select_label)

        # Combo box
        self.package_combo = QComboBox()
        self.package_combo.addItem("All Packages")
        self.package_combo.addItems(self.packages)
        self.package_combo.setCurrentIndex(0)
        self.package_combo.currentTextChanged.connect(self._on_selection_changed)
        select_layout.addWidget(self.package_combo)

        # Selection info
        self.selection_info = QLabel("")
        self.selection_info.setWordWrap(True)
        self.selection_info.setStyleSheet("color: #4CAF50; font-weight: bold; margin-top: 5px;")
        select_layout.addWidget(self.selection_info)

        select_group.setLayout(select_layout)
        layout.addWidget(select_group)

        # Build time estimate (compact)
        time_text = QLabel(
            "⏱️ <b>Build time:</b> Simple pkg: 10-30s • All pkgs: 1-5min"
        )
        time_text.setWordWrap(True)
        time_text.setStyleSheet("color: #666; font-size: 9px; padding: 5px;")
        layout.addWidget(time_text)

        layout.addStretch()  # Push buttons to bottom

        # Buttons
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.button(QDialogButtonBox.Ok).setText("Build Now")
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.setLayout(layout)

        # Initial info
        self._on_selection_changed("All Packages")

    def _on_selection_changed(self, package_name: str):
        """Update selection info"""
        self.selected_package = package_name

        if package_name == "All Packages":
            self.selection_info.setText(
                f"✅ Will build <b>all {len(self.packages)} packages</b> in the workspace. "
                "Recommended for first build."
            )
        else:
            self.selection_info.setText(
                f"✅ Will build only <b>{package_name}</b>. "
                "Faster if you only changed this package."
            )

    def get_selected_package(self) -> str:
        """Get the selected package name"""
        return self.selected_package


class ExecutableSelectionDialog(QDialog):
    """
    Dialog for selecting executable to run with detailed explanation (compact, scrollable)
    """

    def __init__(self, parent=None, package_name: str = "", executables: List[str] = None, workspace_path: str = "workspace"):
        super().__init__(parent)

        self.setWindowTitle(f"Select Executable from '{package_name}'")
        self.resize(600, 500)  # Smaller size

        self.package_name = package_name
        self.workspace_path = workspace_path

        # Auto-detect executables if not provided
        if executables is None:
            self.executables = scan_package_executables(package_name, workspace_path)
        else:
            self.executables = executables

        self.selected_executable = ""

        self._setup_ui()

    def _setup_ui(self):
        # Main layout (no scroll here - just buttons and scroll area)
        main_layout = QVBoxLayout()

        # Title (outside scroll area)
        title = QLabel("<h3>🚀 Select Executable to Run</h3>")
        main_layout.addWidget(title)

        # Scrollable area for all content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Content widget
        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)

        # Compact explanation
        exec_explanation = QLabel(
            "<b>What is an executable?</b> A runnable program that starts your ROS2 node.<br>"
            f"Command: <code>ros2 run {self.package_name} [executable_name]</code>"
        )
        exec_explanation.setWordWrap(True)
        exec_explanation.setStyleSheet(
            "background-color: #FFF9E6; padding: 10px; border-radius: 5px; font-size: 10px;"
        )
        content_layout.addWidget(exec_explanation)

        # Executable selection (PRIMARY FOCUS - USE DROPDOWN)
        select_group = QGroupBox("🎯 Select Executable")
        select_layout = QVBoxLayout()

        if self.executables:
            # Found executables - use dropdown
            found_label = QLabel(
                f"<b>Found {len(self.executables)} executable(s) in '{self.package_name}':</b>"
            )
            select_layout.addWidget(found_label)

            # Dropdown (ComboBox) instead of list
            self.exec_combo = QComboBox()
            self.exec_combo.addItems(self.executables)
            self.exec_combo.setFont(QFont("Consolas", 10))
            self.exec_combo.currentTextChanged.connect(self._on_combo_changed)
            select_layout.addWidget(self.exec_combo)

            # Selection info
            self.selection_info = QLabel("")
            self.selection_info.setWordWrap(True)
            self.selection_info.setStyleSheet("color: #4CAF50; font-weight: bold;")
            select_layout.addWidget(self.selection_info)

            # Set first item as selected
            if self.executables:
                self.exec_combo.setCurrentIndex(0)
                self._on_combo_changed(self.executables[0])
        else:
            # No executables found - manual entry
            warning_label = QLabel(
                "⚠️ <b>No executables found.</b> Enter manually or rebuild package."
            )
            warning_label.setStyleSheet("color: #FF9800;")
            warning_label.setWordWrap(True)
            select_layout.addWidget(warning_label)

            self.exec_input = QLineEdit()
            self.exec_input.setPlaceholderText("e.g., temperature_publisher")
            self.exec_input.setFont(QFont("Consolas", 11))
            self.exec_input.textChanged.connect(lambda t: setattr(self, 'selected_executable', t))
            select_layout.addWidget(self.exec_input)

            hints = QLabel(
                "💡 Common names: temperature_publisher, serial_bridge_node, motor_controller"
            )
            hints.setWordWrap(True)
            hints.setStyleSheet("color: #666; font-size: 9px;")
            select_layout.addWidget(hints)

        select_group.setLayout(select_layout)
        content_layout.addWidget(select_group)

        # Compact "What happens" section (collapsible style)
        happens_group = QGroupBox("▶️ What Happens When You Run?")
        happens_layout = QVBoxLayout()

        happens_text = QLabel(
            "1. Execute on Ubuntu: <code>ros2 run {pkg} {exe}</code><br>"
            "2. ROS2 starts your node in background<br>"
            "3. Node connects and begins publishing/subscribing<br>"
            "4. Monitor: <b>Logs</b> tab (output), <b>Node Status</b> (green=running)"
        .format(pkg=self.package_name, exe="[executable]"))
        happens_text.setWordWrap(True)
        happens_text.setStyleSheet(
            "background-color: #E3F2FD; padding: 8px; border-radius: 5px; "
            "color: #556B2F; font-size: 9px;"
        )
        happens_layout.addWidget(happens_text)

        happens_group.setLayout(happens_layout)
        content_layout.addWidget(happens_group)

        # Troubleshooting (compact)
        trouble_group = QGroupBox("🐛 Troubleshooting")
        trouble_layout = QVBoxLayout()

        trouble_text = QLabel(
            "If node doesn't start:<br>"
            "• Check <b>Logs</b> tab for errors<br>"
            "• Verify package was built (Ctrl+B)<br>"
            "• Check spelling of executable name"
        )
        trouble_text.setWordWrap(True)
        trouble_text.setStyleSheet("color: #666; font-size: 9px;")
        trouble_layout.addWidget(trouble_text)

        trouble_group.setLayout(trouble_layout)
        content_layout.addWidget(trouble_group)

        content_layout.addStretch()

        # Set content widget to scroll area
        scroll.setWidget(content_widget)
        main_layout.addWidget(scroll)

        # Buttons (outside scroll area, always visible)
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.button(QDialogButtonBox.Ok).setText("Run Executable")
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        main_layout.addWidget(button_box)

        self.setLayout(main_layout)

    def _on_combo_changed(self, exe_name: str):
        """Handle executable selection from dropdown"""
        if exe_name:
            self.selected_executable = exe_name
            self.selection_info.setText(
                f"✅ Will run: <code>ros2 run {self.package_name} {exe_name}</code>"
            )

    def get_selected_executable(self) -> str:
        """Get the selected executable name"""
        return self.selected_executable


# Convenience functions for main window

def ask_package_name(parent=None, default_name: str = "my_robot_pkg") -> Optional[str]:
    """
    Show package name dialog and return selected name (or None if cancelled)
    """
    dialog = PackageNameDialog(parent, default_name)
    if dialog.exec() == QDialog.Accepted:
        return dialog.get_package_name()
    return None


def ask_output_directory(parent=None, default_dir: str = "workspace/src") -> Optional[str]:
    """
    Show output directory dialog and return selected path (or None if cancelled)
    """
    dialog = OutputDirectoryDialog(parent, default_dir)
    if dialog.exec() == QDialog.Accepted:
        return dialog.get_directory()
    return None


def ask_build_package(parent=None, packages: List[str] = None) -> Optional[str]:
    """
    Show build package dialog and return selected package (or None if cancelled)
    """
    dialog = BuildPackageDialog(parent, packages)
    if dialog.exec() == QDialog.Accepted:
        return dialog.get_selected_package()
    return None


def ask_executable(parent=None, package_name: str = "", executables: List[str] = None, workspace_path: str = "workspace") -> Optional[str]:
    """
    Show executable selection dialog and return selected executable (or None if cancelled)

    Args:
        parent: Parent widget
        package_name: Name of the package
        executables: List of executables (if None, will auto-detect from workspace)
        workspace_path: Path to workspace for auto-detection (default: "workspace")
    """
    dialog = ExecutableSelectionDialog(parent, package_name, executables, workspace_path)
    if dialog.exec() == QDialog.Accepted:
        return dialog.get_selected_executable()
    return None
