"""
Getting Started Widget - Interactive onboarding panel

Provides step-by-step guidance for new users.

Author: RoboShire Team
Version: 0.14.0
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QFrame, QScrollArea, QCheckBox, QProgressBar
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QPixmap
from typing import List, Dict
import json
from pathlib import Path


class GettingStartedWidget(QWidget):
    """
    Interactive getting started panel with step-by-step guidance

    Features:
    - Step-by-step checklist
    - Quick action buttons
    - Progress tracking
    - Collapsible sections
    - Help links
    """

    action_requested = Signal(str)  # Emits action name when button clicked

    def __init__(self, parent=None):
        super().__init__(parent)

        self.steps_completed = self._load_progress()
        self._init_ui()

    def _init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)

        # Scroll area for steps
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        steps_widget = QWidget()
        steps_layout = QGridLayout(steps_widget)
        steps_layout.setSpacing(15)  # Space between grid items
        steps_layout.setContentsMargins(10, 10, 10, 10)

        # Define steps
        self.steps = [
            {
                'id': 'welcome',
                'title': '1. Welcome',
                'description': 'Learn what RoboShire can do',
                'action_text': 'Watch Quick Tour',
                'action': 'show_welcome'
            },
            {
                'id': 'configure_ssh',
                'title': '2. Configure SSH Connection',
                'description': 'Set up connection to Ubuntu VM for building ROS2 packages',
                'action_text': 'Run SSH Setup Wizard',
                'action': 'configure_ssh',
                'help': 'SSH lets RoboShire build and run ROS2 code on your Ubuntu system.'
            },
            {
                'id': 'create_project',
                'title': '3. Create Your First Project',
                'description': 'Use the wizard to create a robot in 5 minutes',
                'action_text': 'Start Robot Wizard',
                'action': 'new_robot_wizard',
                'help': 'The wizard guides you through URDF, sensors, and code generation.'
            },
            {
                'id': 'import_example',
                'title': '   OR: Try an Example Robot',
                'description': 'Start with a pre-built robot (Weather Station, Robotic Arm, etc.)',
                'action_text': 'Browse Examples',
                'action': 'open_examples',
                'help': '6 example robots with complete URDF and code ready to run.'
            },
            {
                'id': 'build_package',
                'title': '4. Build Your ROS2 Package',
                'description': 'Compile your robot code using colcon',
                'action_text': 'Build Now',
                'action': 'build_package',
                'help': 'Building compiles your Python/C++ code into a ROS2 package.'
            },
            {
                'id': 'run_simulation',
                'title': '5. Simulate in MuJoCo',
                'description': 'See your robot in 3D physics simulation',
                'action_text': 'Open MuJoCo Viewer',
                'action': 'open_mujoco',
                'help': 'MuJoCo provides real-time 60 FPS physics simulation.'
            },
            {
                'id': 'run_nodes',
                'title': '6. Run ROS2 Nodes',
                'description': 'Launch your robot nodes on Ubuntu',
                'action_text': 'Run Nodes',
                'action': 'run_nodes',
                'help': 'Nodes are the programs that control your robot.'
            },
            {
                'id': 'visualize_rviz',
                'title': '7. Visualize in RViz2',
                'description': 'View robot in RViz2 3D visualization',
                'action_text': 'Launch RViz2',
                'action': 'launch_rviz',
                'help': 'RViz2 shows robot model, sensors, and transforms.'
            },
            {
                'id': 'explore_features',
                'title': '8. Explore Advanced Features',
                'description': 'Node Graph Editor, Topic Inspector, Parameter Editor, etc.',
                'action_text': 'Show Feature Guide',
                'action': 'show_features',
                'help': 'RoboShire has 12 tabs with powerful ROS2 tools.'
            },
            {
                'id': 'keyboard_shortcuts',
                'title': '9. Learn Keyboard Shortcuts',
                'description': 'Speed up your workflow with 40+ shortcuts',
                'action_text': 'View Shortcuts',
                'action': 'show_shortcuts',
                'help': 'Press F1 anytime to see all keyboard shortcuts.'
            },
        ]

        # Create step widgets in grid layout (3 columns)
        self.step_widgets = []
        num_columns = 3  # 3 cards per row
        for i, step in enumerate(self.steps):
            step_widget = self._create_step_widget(step)
            self.step_widgets.append(step_widget)

            # Calculate row and column
            row = i // num_columns
            col = i % num_columns
            steps_layout.addWidget(step_widget, row, col)

        # Add stretch to bottom row to push cards to top
        last_row = (len(self.steps) // num_columns) + 1
        steps_layout.setRowStretch(last_row, 1)

        scroll.setWidget(steps_widget)
        layout.addWidget(scroll)

        # Footer buttons
        footer = QHBoxLayout()

        self.hide_button = QPushButton("Hide This Panel")
        self.hide_button.clicked.connect(self._hide_getting_started)
        footer.addWidget(self.hide_button)

        self.reset_button = QPushButton("Reset Progress")
        self.reset_button.clicked.connect(self._reset_progress)
        footer.addWidget(self.reset_button)

        footer.addStretch()

        help_button = QPushButton("Need Help?")
        help_button.clicked.connect(lambda: self.action_requested.emit('show_help'))
        footer.addWidget(help_button)

        layout.addLayout(footer)

        # Update progress
        self._update_progress()

    def _create_step_widget(self, step: Dict) -> QWidget:
        """Create a single step widget"""
        widget = QFrame()
        widget.setFrameShape(QFrame.StyledPanel)
        widget.setStyleSheet("""
            QFrame {
                background-color: #f9f9f9;
                border: 1px solid #ddd;
                border-radius: 5px;
                padding: 10px;
                margin: 5px 0;
            }
        """)

        layout = QVBoxLayout(widget)

        # Title (no checkbox)
        title = QLabel(step['title'])
        title_font = QFont()
        title_font.setBold(True)
        title_font.setPointSize(11)
        title.setFont(title_font)
        layout.addWidget(title)

        # Description (no left margin since no checkbox)
        desc = QLabel(step['description'])
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #555;")
        layout.addWidget(desc)

        # Help text (if provided)
        if 'help' in step:
            help_label = QLabel(f"💡 {step['help']}")
            help_label.setWordWrap(True)
            help_label.setStyleSheet("color: #888; font-size: 9px; font-style: italic;")
            layout.addWidget(help_label)

        # Action button (no left spacing since no checkbox)
        button_layout = QHBoxLayout()

        action_button = QPushButton(step['action_text'])
        action_button.setStyleSheet("""
            QPushButton {
                background-color: #556B2F;
                color: white;
                border: none;
                padding: 8px 15px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #6B8E23;
            }
            QPushButton:pressed {
                background-color: #3D5016;
            }
        """)
        action_button.clicked.connect(lambda: self._on_action_clicked(step))
        button_layout.addWidget(action_button)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        return widget

    def _on_step_toggled(self, step: Dict, state: int):
        """Handle step checkbox toggle"""
        if state == Qt.Checked:
            if step['id'] not in self.steps_completed:
                self.steps_completed.append(step['id'])
        else:
            if step['id'] in self.steps_completed:
                self.steps_completed.remove(step['id'])

        self._save_progress()
        self._update_progress()

    def _on_action_clicked(self, step: Dict):
        """Handle action button click"""
        # Mark step as completed (progress tracked internally)
        if step['id'] not in self.steps_completed:
            self.steps_completed.append(step['id'])
            self._save_progress()
            self._update_progress()

        # Emit action signal
        self.action_requested.emit(step['action'])

    def _update_progress(self):
        """Update progress (tracking only, no UI update)"""
        # Progress is tracked internally but no longer displayed
        pass

    def _load_progress(self) -> List[str]:
        """Load progress from file"""
        progress_file = Path.home() / ".roboshire" / "getting_started_progress.json"

        if progress_file.exists():
            try:
                with open(progress_file, 'r') as f:
                    data = json.load(f)
                    return data.get('completed_steps', [])
            except:
                pass

        return []

    def _save_progress(self):
        """Save progress to file"""
        progress_file = Path.home() / ".roboshire" / "getting_started_progress.json"
        progress_file.parent.mkdir(parents=True, exist_ok=True)

        try:
            with open(progress_file, 'w') as f:
                json.dump({'completed_steps': self.steps_completed}, f)
        except:
            pass

    def _reset_progress(self):
        """Reset all progress"""
        from PySide6.QtWidgets import QMessageBox

        reply = QMessageBox.question(
            self,
            "Reset Progress",
            "Are you sure you want to reset all getting started progress?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.steps_completed = []
            self._save_progress()
            self._update_progress()

    def _hide_getting_started(self):
        """Hide the getting started panel"""
        from PySide6.QtWidgets import QMessageBox

        reply = QMessageBox.question(
            self,
            "Hide Getting Started",
            "Hide the Getting Started panel?\n\n"
            "You can show it again from: View → Show Getting Started",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.hide()


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    widget = GettingStartedWidget()
    widget.action_requested.connect(lambda action: print(f"Action requested: {action}"))
    widget.show()

    sys.exit(app.exec())
