"""
Friendly Error Dialog

Displays error messages with:
- Plain English explanations
- Step-by-step solutions
- Collapsible technical details
- Context-sensitive help

Addresses Priority 2 Task #6: "Improve Error Messages"
Pattern: Reduces "WHAT DOES THIS MEAN?!" moments
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QWidget, QFrame
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QIcon, QPixmap, QPainter, QColor
from typing import Optional, List, Dict
import logging


class FriendlyErrorDialog(QDialog):
    """
    User-friendly error dialog

    Shows:
    - Error title with icon
    - Plain English explanation
    - Step-by-step solution
    - Collapsible technical details
    - Optional "Learn More" link
    """

    def __init__(
        self,
        error_type: str,
        plain_english: str,
        what_to_do: List[str],
        technical_details: Optional[str] = None,
        learn_more_url: Optional[str] = None,
        parent=None
    ):
        """
        Initialize FriendlyErrorDialog

        Args:
            error_type: Type of error ("Build Failed", "Node Crashed", etc.)
            plain_english: Simple explanation of what went wrong
            what_to_do: List of steps to fix the issue
            technical_details: Raw error output (optional, collapsible)
            learn_more_url: URL to documentation (optional)
            parent: Parent widget
        """
        super().__init__(parent)

        self.error_type = error_type
        self.plain_english = plain_english
        self.what_to_do = what_to_do
        self.technical_details = technical_details
        self.learn_more_url = learn_more_url

        self.logger = logging.getLogger(__name__)

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI components"""
        self.setWindowTitle(f"Error: {self.error_type}")
        self.setModal(True)
        self.setMinimumWidth(600)
        self.setMaximumWidth(800)

        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        # Error title with icon
        title_layout = QHBoxLayout()

        # Error icon
        icon_label = QLabel()
        icon_label.setPixmap(self._create_error_icon())
        title_layout.addWidget(icon_label)

        # Error title
        title_label = QLabel(f"<h2 style='color: #D32F2F;'>{self.error_type}</h2>")
        title_layout.addWidget(title_label)
        title_layout.addStretch()

        layout.addLayout(title_layout)

        # Separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator)

        # Plain English explanation
        plain_label = QLabel("<b>Plain English:</b>")
        plain_label.setStyleSheet("font-size: 13px; color: #333;")
        layout.addWidget(plain_label)

        explanation_label = QLabel(self.plain_english)
        explanation_label.setWordWrap(True)
        explanation_label.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #FFF3E0;
                border-left: 4px solid #FF9800;
                border-radius: 4px;
                font-size: 12px;
            }
        """)
        layout.addWidget(explanation_label)

        # What to do section
        what_to_do_label = QLabel("<b>What to do:</b>")
        what_to_do_label.setStyleSheet("font-size: 13px; color: #333; margin-top: 10px;")
        layout.addWidget(what_to_do_label)

        # Steps
        steps_widget = QWidget()
        steps_layout = QVBoxLayout(steps_widget)
        steps_layout.setContentsMargins(0, 0, 0, 0)
        steps_layout.setSpacing(8)

        for i, step in enumerate(self.what_to_do, 1):
            step_label = QLabel(f"{i}. {step}")
            step_label.setWordWrap(True)
            step_label.setStyleSheet("""
                QLabel {
                    padding: 8px;
                    background-color: #E8F5E9;
                    border-left: 3px solid #4CAF50;
                    border-radius: 3px;
                    font-size: 12px;
                }
            """)
            steps_layout.addWidget(step_label)

        layout.addWidget(steps_widget)

        # Technical details (collapsible)
        if self.technical_details:
            # Show/Hide technical details button
            self.details_button = QPushButton("▶ Show Technical Details")
            self.details_button.setStyleSheet("""
                QPushButton {
                    text-align: left;
                    padding: 8px;
                    background-color: transparent;
                    border: none;
                    font-weight: bold;
                    color: #1976D2;
                }
                QPushButton:hover {
                    background-color: #F5F5F5;
                }
            """)
            self.details_button.clicked.connect(self._toggle_details)
            layout.addWidget(self.details_button)

            # Technical details text (hidden by default)
            self.details_text = QTextEdit()
            self.details_text.setPlainText(self.technical_details)
            self.details_text.setReadOnly(True)
            self.details_text.setMaximumHeight(200)
            self.details_text.setStyleSheet("""
                QTextEdit {
                    background-color: #263238;
                    color: #ECEFF1;
                    font-family: 'Courier New', monospace;
                    font-size: 10px;
                    padding: 10px;
                    border: 1px solid #37474F;
                    border-radius: 4px;
                }
            """)
            self.details_text.setVisible(False)
            layout.addWidget(self.details_text)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        # Learn More button (if URL provided)
        if self.learn_more_url:
            learn_more_btn = QPushButton("Learn More")
            learn_more_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                    border: none;
                    padding: 8px 16px;
                    border-radius: 4px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: #1976D2;
                }
            """)
            learn_more_btn.clicked.connect(self._on_learn_more)
            button_layout.addWidget(learn_more_btn)

        # Close button
        close_btn = QPushButton("Close")
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #556B2F;
                color: white;
                border: none;
                padding: 8px 20px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #6B8E3A;
            }
        """)
        close_btn.clicked.connect(self.accept)
        button_layout.addWidget(close_btn)

        layout.addLayout(button_layout)

    def _create_error_icon(self) -> QPixmap:
        """Create error icon pixmap"""
        size = 48
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Red circle
        painter.setBrush(QColor("#D32F2F"))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, size, size)

        # White X
        painter.setPen(QColor("white"))
        painter.setFont(QFont("Arial", 28, QFont.Bold))
        painter.drawText(pixmap.rect(), Qt.AlignCenter, "✖")

        painter.end()
        return pixmap

    def _toggle_details(self):
        """Toggle technical details visibility"""
        if self.details_text.isVisible():
            self.details_text.setVisible(False)
            self.details_button.setText("▶ Show Technical Details")
        else:
            self.details_text.setVisible(True)
            self.details_button.setText("▼ Hide Technical Details")

    def _on_learn_more(self):
        """Open documentation URL"""
        if self.learn_more_url:
            from PySide6.QtGui import QDesktopServices
            from PySide6.QtCore import QUrl
            QDesktopServices.openUrl(QUrl(self.learn_more_url))
            self.logger.info(f"Opened documentation: {self.learn_more_url}")


# Convenience functions for common error types

def show_build_error(
    error_message: str,
    parent=None
) -> None:
    """
    Show build error dialog

    Args:
        error_message: Raw colcon build error
        parent: Parent widget
    """
    # Parse error message for common issues
    plain_english, steps = _parse_build_error(error_message)

    dialog = FriendlyErrorDialog(
        error_type="Build Failed",
        plain_english=plain_english,
        what_to_do=steps,
        technical_details=error_message,
        learn_more_url="https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html",
        parent=parent
    )
    dialog.exec()


def show_node_crash_error(
    node_name: str,
    error_message: str,
    parent=None
) -> None:
    """
    Show node crash error dialog

    Args:
        node_name: Name of crashed node
        error_message: Crash error message
        parent: Parent widget
    """
    dialog = FriendlyErrorDialog(
        error_type=f"Node Crashed: {node_name}",
        plain_english=f"The ROS2 node '{node_name}' stopped unexpectedly. This usually means there's a bug in the node code or a missing dependency.",
        what_to_do=[
            "Check the Logs tab for the full error traceback",
            "Look for Python exceptions or segmentation faults",
            "Verify all dependencies are installed (pip install, apt install)",
            "Check node parameters in Parameters tab",
            "Try rebuilding the workspace (Project → Clean Build)"
        ],
        technical_details=error_message,
        learn_more_url="https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html",
        parent=parent
    )
    dialog.exec()


def show_qos_incompatible_error(
    topic_name: str,
    parent=None
) -> None:
    """
    Show QoS incompatibility error dialog

    Args:
        topic_name: Topic with QoS mismatch
        parent: Parent widget
    """
    dialog = FriendlyErrorDialog(
        error_type="QoS Incompatible",
        plain_english=f"The publisher and subscriber for topic '{topic_name}' have incompatible Quality of Service (QoS) settings. They can't communicate because one requires RELIABLE delivery and the other uses BEST_EFFORT, or similar mismatches.",
        what_to_do=[
            "Open Node Graph tab and select the publisher node",
            "Click 'Configure QoS' for the topic",
            "Choose a preset (usually 'Sensor Data' works)",
            "Do the same for the subscriber node",
            "Regenerate code (Project → Generate Code)",
            "Rebuild (Project → Build)"
        ],
        technical_details=f"QoS policy mismatch detected on topic: {topic_name}\n\nCommon causes:\n- Reliability mismatch (RELIABLE vs BEST_EFFORT)\n- Durability mismatch (VOLATILE vs TRANSIENT_LOCAL)\n\nBoth publisher and subscriber must use compatible QoS settings.",
        learn_more_url="https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html",
        parent=parent
    )
    dialog.exec()


def show_workspace_not_found_error(
    workspace_path: str,
    parent=None
) -> None:
    """
    Show workspace not found error

    Args:
        workspace_path: Path that was not found
        parent: Parent widget
    """
    dialog = FriendlyErrorDialog(
        error_type="Workspace Not Found",
        plain_english=f"The ROS2 workspace at '{workspace_path}' doesn't exist or isn't a valid ROS2 workspace. This usually means the workspace hasn't been created yet or the path is incorrect.",
        what_to_do=[
            "Check if the path exists on your system",
            "Create a new project (File → New Project)",
            "Or open an existing project (File → Open Project)",
            "Make sure the workspace has a 'src' folder",
            "Try building the workspace (Project → Build) to initialize it"
        ],
        technical_details=f"Workspace path not found: {workspace_path}\n\nExpected structure:\n{workspace_path}/\n  src/\n  build/\n  install/\n  log/",
        learn_more_url="https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html",
        parent=parent
    )
    dialog.exec()


def show_ssh_connection_error(
    host: str,
    error_message: str,
    parent=None
) -> None:
    """
    Show SSH connection error

    Args:
        host: SSH host that failed to connect
        error_message: Connection error message
        parent: Parent widget
    """
    dialog = FriendlyErrorDialog(
        error_type="SSH Connection Failed",
        plain_english=f"Could not connect to '{host}' via SSH. This usually means the SSH server isn't running, the credentials are wrong, or there's a network issue.",
        what_to_do=[
            "Check that the Ubuntu VM is running",
            "Verify the IP address is correct",
            "Run SSH Setup Wizard (Tools → SSH Setup Wizard)",
            "Test connection: ssh your_username@{host}",
            "Check firewall settings (allow port 22)",
            "Ensure SSH server is installed: sudo apt install openssh-server"
        ],
        technical_details=error_message,
        learn_more_url="https://ubuntu.com/server/docs/service-openssh",
        parent=parent
    )
    dialog.exec()


def _parse_build_error(error_message: str) -> tuple:
    """
    Parse build error to extract plain English and steps

    Args:
        error_message: Raw colcon build error

    Returns:
        Tuple of (plain_english, steps)
    """
    # Check for common error patterns
    if "ModuleNotFoundError" in error_message or "ImportError" in error_message:
        return (
            "Your code is trying to import a Python module that isn't installed. This is like trying to use a tool you don't have.",
            [
                "Look at the error to see which module is missing",
                "Install it with: pip install <module_name>",
                "Or add it to requirements.txt and run: pip install -r requirements.txt",
                "Then rebuild (Project → Build)"
            ]
        )

    elif "SyntaxError" in error_message:
        return (
            "Your Python code has syntax errors (like missing colons, parentheses, or indentation issues). Python can't understand the code.",
            [
                "Check the Build Output tab for the line number",
                "Open Code Editor and go to that line",
                "Look for common mistakes: missing colons (:), unmatched parentheses, wrong indentation",
                "Fix the syntax error",
                "Try building again (Project → Build)"
            ]
        )

    elif "CMake Error" in error_message:
        return (
            "The build system (CMake) encountered an error. This usually means a package.xml or setup.py file is misconfigured.",
            [
                "Check Build Output for which package failed",
                "Verify package.xml has correct dependencies",
                "Verify setup.py lists all Python files",
                "Make sure all dependencies are installed",
                "Try cleaning and rebuilding (Project → Clean Build)"
            ]
        )

    elif "No such file or directory" in error_message:
        return (
            "The build system is looking for a file that doesn't exist. This could be a missing source file or incorrect path.",
            [
                "Check which file is missing in Build Output",
                "Verify the file exists in your workspace/src folder",
                "Check for typos in file names",
                "If the file should be generated, check earlier build steps",
                "Try regenerating code (Project → Generate Code)"
            ]
        )

    else:
        # Generic build error
        return (
            "The build process failed. This could be due to syntax errors, missing dependencies, or misconfigured files.",
            [
                "Check the Build Output tab for red error lines",
                "Look for the first error (later errors are often cascading)",
                "Common causes: Python syntax errors, missing imports, wrong dependencies",
                "Try cleaning and rebuilding (Project → Clean Build)",
                "If stuck, expand Technical Details below for the full error"
            ]
        )


# Example usage and testing
if __name__ == '__main__':
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    # Test build error
    show_build_error(
        error_message="""
--- stderr: my_robot_pkg
Traceback (most recent call last):
  File "/home/user/workspace/src/my_robot_pkg/my_robot_pkg/temperature_sensor.py", line 5, in <module>
    import fake_module
ModuleNotFoundError: No module named 'fake_module'
---
Failed   <<< my_robot_pkg [2.3s, exited with code 1]
        """,
        parent=None
    )

    sys.exit(app.exec())
