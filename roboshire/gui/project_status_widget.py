"""
Project Status Widget - Tracks complete workflow from start to deployment

Shows project progress through all stages:
1. Project Created (Wizard completed)
2. URDF Loaded (if applicable)
3. Node Graph Designed
4. Code Generated
5. Build Completed
6. Nodes Running
7. Testing Complete
8. Deployment Ready

Author: RoboShire Team
Phase: 9 (UX Polish)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QGroupBox, QProgressBar, QTextEdit, QScrollArea, QFrame
)
from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtGui import QFont, QIcon
from typing import Dict, Optional
import logging
from datetime import datetime
from roboshire.gui.brand_theme import BrandColors, BrandFonts


class ProjectStatusWidget(QWidget):
    """
    Displays project workflow status and guides user through deployment
    """

    # Signals for actions
    workflow_wizard_requested = Signal()  # New signal for starting wizard
    design_node_graph_requested = Signal()
    generate_code_requested = Signal()
    build_requested = Signal()
    run_requested = Signal()
    open_testing_guide_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)

        # Project state
        self.project_name = "No Project"
        self.project_created_time = None
        self.workflow_config = {}

        # Workflow status flags
        self.status = {
            'project_created': False,
            'urdf_loaded': False,
            'node_graph_designed': False,
            'code_generated': False,
            'build_completed': False,
            'nodes_running': False,
            'testing_complete': False,
            'deployment_ready': False
        }

        self._setup_ui()

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout()
        layout.setSpacing(10)

        # Header
        header = self._create_header()
        layout.addWidget(header)

        # Scrollable area for content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)

        # Progress overview
        self.progress_section = self._create_progress_section()
        content_layout.addWidget(self.progress_section)

        # Current step guide
        self.current_step_section = self._create_current_step_section()
        content_layout.addWidget(self.current_step_section)

        # Workflow checklist
        self.checklist_section = self._create_checklist_section()
        content_layout.addWidget(self.checklist_section)

        # Project info
        self.info_section = self._create_info_section()
        content_layout.addWidget(self.info_section)

        content_layout.addStretch()

        scroll.setWidget(content_widget)
        layout.addWidget(scroll)

        self.setLayout(layout)

        # Initial update
        self._update_display()

    def _create_header(self) -> QWidget:
        """Create header with project name and overview"""
        header = QFrame()
        header.setStyleSheet(f"""
            QFrame {{
                background-color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});
                border-radius: 8px;
                padding: 15px;
            }}
        """)

        layout = QVBoxLayout(header)

        # Title
        self.project_title = QLabel("📊 Project Status Dashboard")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        self.project_title.setFont(title_font)
        self.project_title.setStyleSheet("color: white;")
        layout.addWidget(self.project_title)

        # Subtitle
        self.project_subtitle = QLabel("Track your robot development from start to deployment")
        self.project_subtitle.setStyleSheet("color: rgba(255, 255, 255, 0.9); font-size: 11px;")
        layout.addWidget(self.project_subtitle)

        return header

    def _create_progress_section(self) -> QGroupBox:
        """Create overall progress section"""
        group = QGroupBox("🎯 Overall Progress")
        layout = QVBoxLayout()

        # Progress bar
        self.overall_progress_bar = QProgressBar()
        self.overall_progress_bar.setMaximum(8)  # 8 stages
        self.overall_progress_bar.setValue(0)
        self.overall_progress_bar.setTextVisible(True)
        self.overall_progress_bar.setFormat("%v / %m stages complete (%p%)")
        self.overall_progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #ccc;
                border-radius: 5px;
                text-align: center;
                height: 30px;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background-color: rgb({BrandColors.OLIVE_LIGHT.red()}, {BrandColors.OLIVE_LIGHT.green()}, {BrandColors.OLIVE_LIGHT.blue()});
                border-radius: 3px;
            }}
        """)
        layout.addWidget(self.overall_progress_bar)

        # Progress label
        self.progress_label = QLabel("No project created yet")
        self.progress_label.setAlignment(Qt.AlignCenter)
        self.progress_label.setStyleSheet("font-size: 12px; color: #666; margin-top: 5px;")
        layout.addWidget(self.progress_label)

        group.setLayout(layout)
        return group

    def _create_current_step_section(self) -> QGroupBox:
        """Create current step guidance section"""
        group = QGroupBox("👉 Next Step")
        layout = QVBoxLayout()

        # Current step title
        self.current_step_title = QLabel("Get Started")
        step_font = QFont()
        step_font.setPointSize(14)
        step_font.setBold(True)
        self.current_step_title.setFont(step_font)
        self.current_step_title.setStyleSheet(f"color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});")
        layout.addWidget(self.current_step_title)

        # Current step description
        self.current_step_desc = QLabel(
            "Use the Workflow Wizard (File → New Robot Wizard or Ctrl+Shift+N) "
            "to create your first robot project in 5 minutes!"
        )
        self.current_step_desc.setWordWrap(True)
        self.current_step_desc.setStyleSheet("font-size: 11px; margin-top: 5px;")
        layout.addWidget(self.current_step_desc)

        # Action button
        self.action_button = QPushButton("🚀 Start Workflow Wizard")
        self.action_button.setStyleSheet(f"""
            QPushButton {{
                background-color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});
                color: white;
                padding: 12px;
                font-size: 13px;
                font-weight: bold;
                border-radius: 5px;
                margin-top: 10px;
            }}
            QPushButton:hover {{
                background-color: rgb({BrandColors.OLIVE_LIGHT.red()}, {BrandColors.OLIVE_LIGHT.green()}, {BrandColors.OLIVE_LIGHT.blue()});
            }}
            QPushButton:disabled {{
                background-color: #ccc;
            }}
        """)
        self.action_button.clicked.connect(self._on_action_button_clicked)
        layout.addWidget(self.action_button)

        group.setLayout(layout)
        return group

    def _create_checklist_section(self) -> QGroupBox:
        """Create workflow checklist in grid layout"""
        group = QGroupBox("✓ Workflow Checklist")
        layout = QGridLayout()
        layout.setSpacing(10)  # Space between cards
        layout.setContentsMargins(10, 10, 10, 10)

        # Checklist items
        self.checklist_items = {}

        steps = [
            ('project_created', '1. Create Project', 'Use Workflow Wizard to set up your robot'),
            ('urdf_loaded', '2. Load URDF (Optional)', 'Load robot model for 3D visualization'),
            ('node_graph_designed', '3. Design Node Graph', 'Create ROS2 nodes and connections'),
            ('code_generated', '4. Generate Code', 'Convert node graph to Python code'),
            ('build_completed', '5. Build on Ubuntu', 'Compile and package with colcon'),
            ('nodes_running', '6. Run Nodes', 'Launch ROS2 nodes and verify operation'),
            ('testing_complete', '7. Test & Validate', 'Verify sensors, data flow, and behavior'),
            ('deployment_ready', '8. Deploy', 'Ready for production deployment')
        ]

        # Arrange in 2 columns for better readability
        num_columns = 2
        for i, (key, title, desc) in enumerate(steps):
            item_widget = self._create_checklist_item(key, title, desc)
            self.checklist_items[key] = item_widget

            # Calculate row and column
            row = i // num_columns
            col = i % num_columns
            layout.addWidget(item_widget, row, col)

        # Add stretch to bottom to push items to top
        last_row = (len(steps) // num_columns) + 1
        layout.setRowStretch(last_row, 1)

        group.setLayout(layout)
        return group

    def _create_checklist_item(self, key: str, title: str, description: str) -> QFrame:
        """Create a single checklist item"""
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #f5f5f5;
                border-left: 4px solid #ccc;
                border-radius: 3px;
                padding: 8px;
                margin: 2px;
            }
        """)

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(10, 5, 10, 5)

        # Status icon
        status_label = QLabel("○")
        status_label.setObjectName("status_icon")
        status_label.setStyleSheet("font-size: 16px; color: #ccc; font-weight: bold;")
        layout.addWidget(status_label)

        # Text content
        text_layout = QVBoxLayout()
        text_layout.setSpacing(2)

        title_label = QLabel(title)
        title_label.setObjectName("title")
        title_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        text_layout.addWidget(title_label)

        desc_label = QLabel(description)
        desc_label.setObjectName("description")
        desc_label.setStyleSheet("font-size: 10px; color: #666;")
        desc_label.setWordWrap(True)
        text_layout.addWidget(desc_label)

        layout.addLayout(text_layout, 1)

        return frame

    def _create_info_section(self) -> QGroupBox:
        """Create project information section"""
        group = QGroupBox("ℹ️ Project Information")
        layout = QVBoxLayout()

        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(150)
        self.info_text.setStyleSheet("""
            QTextEdit {
                background-color: #f9f9f9;
                border: 1px solid #ddd;
                border-radius: 5px;
                padding: 10px;
                font-family: 'Consolas', monospace;
                font-size: 10px;
            }
        """)
        self.info_text.setPlainText("No project loaded yet.")
        layout.addWidget(self.info_text)

        group.setLayout(layout)
        return group

    @Slot(dict)
    def load_wizard_config(self, config: Dict):
        """Load configuration from wizard completion"""
        self.workflow_config = config
        self.project_name = config.get('project_name', 'Unknown Project')
        self.project_created_time = datetime.now()

        # Mark project as created
        self.update_status('project_created', True)

        # If URDF was loaded
        if config.get('use_urdf') and config.get('urdf_path'):
            self.update_status('urdf_loaded', True)

        self._update_display()
        self.logger.info(f"Loaded wizard config for project: {self.project_name}")

    def update_status(self, stage: str, completed: bool = True):
        """Update a specific workflow stage status"""
        if stage in self.status:
            self.status[stage] = completed
            self._update_display()
            self.logger.info(f"Status updated: {stage} = {completed}")

    def _update_display(self):
        """Update all UI elements based on current status"""
        # Count completed stages
        completed_count = sum(1 for v in self.status.values() if v)

        # Update progress bar
        self.overall_progress_bar.setValue(completed_count)

        # Update progress label
        percentage = int((completed_count / 8) * 100)
        self.progress_label.setText(f"{percentage}% Complete - {self._get_stage_name()}")

        # Update project title
        if self.project_name != "No Project":
            self.project_title.setText(f"📊 {self.project_name} - Status Dashboard")

        # Update checklist items
        for key, frame in self.checklist_items.items():
            status_icon = frame.findChild(QLabel, "status_icon")
            if status_icon:
                if self.status[key]:
                    status_icon.setText("✓")
                    status_icon.setStyleSheet(f"font-size: 16px; color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()}); font-weight: bold;")
                    frame.setStyleSheet(f"""
                        QFrame {{
                            background-color: rgb({BrandColors.OLIVE_VERY_LIGHT.red()}, {BrandColors.OLIVE_VERY_LIGHT.green()}, {BrandColors.OLIVE_VERY_LIGHT.blue()});
                            border-left: 4px solid rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});
                            border-radius: 3px;
                            padding: 8px;
                            margin: 2px;
                        }}
                    """)
                else:
                    status_icon.setText("○")
                    status_icon.setStyleSheet("font-size: 16px; color: #ccc; font-weight: bold;")
                    frame.setStyleSheet("""
                        QFrame {
                            background-color: #f5f5f5;
                            border-left: 4px solid #ccc;
                            border-radius: 3px;
                            padding: 8px;
                            margin: 2px;
                        }
                    """)

        # Update current step section
        self._update_current_step()

        # Update info section
        self._update_info_text()

    def _get_stage_name(self) -> str:
        """Get current stage name"""
        if not self.status['project_created']:
            return "Getting Started"
        elif not self.status['node_graph_designed']:
            return "Design Phase"
        elif not self.status['code_generated']:
            return "Ready to Generate"
        elif not self.status['build_completed']:
            return "Ready to Build"
        elif not self.status['nodes_running']:
            return "Ready to Run"
        elif not self.status['testing_complete']:
            return "Testing Phase"
        elif not self.status['deployment_ready']:
            return "Deployment Prep"
        else:
            return "Deployment Ready!"

    def _update_current_step(self):
        """Update the current step guidance"""
        if not self.status['project_created']:
            self.current_step_title.setText("1. Create Your Project")
            self.current_step_desc.setText(
                "Use the Workflow Wizard to create your robot project. "
                "Click the button below or use File → New Robot Wizard (Ctrl+Shift+N)."
            )
            self.action_button.setText("🚀 Start Workflow Wizard")
            self.action_button.setEnabled(True)

        elif not self.status['node_graph_designed']:
            self.current_step_title.setText("2. Design Node Graph")
            self.current_step_desc.setText(
                "Switch to the Node Graph tab and design your ROS2 nodes. "
                "Drag nodes from the library, connect them, and configure their properties. "
                "This defines your robot's behavior!"
            )
            self.action_button.setText("📐 Open Node Graph Editor")
            self.action_button.setEnabled(True)

        elif not self.status['code_generated']:
            self.current_step_title.setText("3. Generate ROS2 Code")
            self.current_step_desc.setText(
                "Convert your node graph into Python code. "
                "Click the button below or use Build → Generate Code (Ctrl+G). "
                "This creates all ROS2 packages, nodes, launch files, and configuration."
            )
            self.action_button.setText("⚡ Generate Code (Ctrl+G)")
            self.action_button.setEnabled(True)

        elif not self.status['build_completed']:
            self.current_step_title.setText("4. Build on VM")
            self.current_step_desc.setText(
                "Compile and package your ROS2 code using colcon. "
                "Make sure your Ubuntu VM is running and SSH is configured. "
                "Click below or use Build → Build (Ctrl+B)."
            )
            self.action_button.setText("🔨 Build Project (Ctrl+B)")
            self.action_button.setEnabled(True)

        elif not self.status['nodes_running']:
            self.current_step_title.setText("5. Run Your Nodes")
            self.current_step_desc.setText(
                "Launch your ROS2 nodes and start your robot! "
                "Click below or use Build → Run (Ctrl+R). "
                "Monitor node status in the Node Status tab and logs in the Logs tab."
            )
            self.action_button.setText("▶️ Run Nodes (Ctrl+R)")
            self.action_button.setEnabled(True)

        elif not self.status['testing_complete']:
            self.current_step_title.setText("6. Test & Validate")
            self.current_step_desc.setText(
                "Test your robot's functionality! "
                "Use the Topic Inspector to monitor data, check node status, "
                "and verify sensors and actuators work correctly. "
                "Run through the testing checklist from the wizard."
            )
            self.action_button.setText("📋 Open Testing Guide")
            self.action_button.setEnabled(True)

        elif not self.status['deployment_ready']:
            self.current_step_title.setText("7. Prepare for Deployment")
            self.current_step_desc.setText(
                "Your robot is tested and working! "
                "Final steps: Create documentation, save configuration, "
                "and prepare deployment scripts. Mark as deployment-ready when complete."
            )
            self.action_button.setText("✅ Mark Deployment Ready")
            self.action_button.setEnabled(True)

        else:
            self.current_step_title.setText("🎉 Deployment Ready!")
            self.current_step_desc.setText(
                "Congratulations! Your robot project is complete and ready for deployment. "
                "You can now deploy to your target platform (Raspberry Pi, Jetson, etc.). "
                "Start a new project or continue improving this one!"
            )
            self.action_button.setText("🆕 Start New Project")
            self.action_button.setEnabled(True)

    def _update_info_text(self):
        """Update project information display"""
        if not self.status['project_created']:
            info = "No project loaded yet.\n\nUse the Workflow Wizard to create your first robot project!"
        else:
            info = f"Project Name: {self.project_name}\n"
            if self.project_created_time:
                info += f"Created: {self.project_created_time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            info += f"\n--- Configuration ---\n"

            if self.workflow_config:
                info += f"Microcontroller: {self.workflow_config.get('microcontroller', 'Not set')}\n"
                info += f"URDF: {'Yes' if self.workflow_config.get('use_urdf') else 'No'}\n"

                sensors = self.workflow_config.get('sensors', [])
                if sensors:
                    info += f"Sensors: {', '.join(sensors[:3])}"
                    if len(sensors) > 3:
                        info += f" (+{len(sensors)-3} more)"
                    info += "\n"

                actuators = self.workflow_config.get('actuators', [])
                if actuators:
                    info += f"Actuators: {', '.join(actuators[:3])}"
                    if len(actuators) > 3:
                        info += f" (+{len(actuators)-3} more)"
                    info += "\n"

                goal = self.workflow_config.get('project_goal')
                if goal:
                    info += f"Goal: {goal}\n"

                info += f"micro-ROS: {'Enabled' if self.workflow_config.get('use_micro_ros') else 'Disabled'}\n"

            info += f"\n--- Progress ---\n"
            info += f"Completed: {sum(1 for v in self.status.values() if v)}/8 stages\n"

            next_step = self._get_next_incomplete_step()
            if next_step:
                info += f"Next: {next_step}\n"

        self.info_text.setPlainText(info)

    def _get_next_incomplete_step(self) -> Optional[str]:
        """Get the next incomplete step name"""
        steps = [
            ('project_created', 'Create Project'),
            ('urdf_loaded', 'Load URDF'),
            ('node_graph_designed', 'Design Node Graph'),
            ('code_generated', 'Generate Code'),
            ('build_completed', 'Build'),
            ('nodes_running', 'Run Nodes'),
            ('testing_complete', 'Testing'),
            ('deployment_ready', 'Deployment')
        ]

        for key, name in steps:
            if not self.status[key]:
                return name
        return None

    def _on_action_button_clicked(self):
        """Handle action button click based on current stage"""
        if not self.status['project_created']:
            # Trigger workflow wizard
            self.workflow_wizard_requested.emit()
        elif not self.status['node_graph_designed']:
            self.design_node_graph_requested.emit()
        elif not self.status['code_generated']:
            self.generate_code_requested.emit()
        elif not self.status['build_completed']:
            self.build_requested.emit()
        elif not self.status['nodes_running']:
            self.run_requested.emit()
        elif not self.status['testing_complete']:
            self.open_testing_guide_requested.emit()
        elif not self.status['deployment_ready']:
            # Mark as deployment ready
            self.update_status('deployment_ready', True)
        else:
            # All stages complete - offer to start new project
            from PySide6.QtWidgets import QMessageBox
            reply = QMessageBox.question(
                self,
                "Start New Project?",
                "All workflow stages complete! Congratulations!\n\n"
                "Would you like to start a new project?\n\n"
                "(This will reset the Project Status tracker and open the Workflow Wizard)",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )

            if reply == QMessageBox.Yes:
                # Reset status and start wizard
                self.reset_status()
                self.workflow_wizard_requested.emit()

    def reset_status(self):
        """Reset all status to initial state"""
        for key in self.status:
            self.status[key] = False
        self.project_name = "No Project"
        self.workflow_config = {}
        self.project_created_time = None
        self._update_display()
