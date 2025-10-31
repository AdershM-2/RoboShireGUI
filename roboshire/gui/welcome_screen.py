"""
Welcome Screen Widget

First-time user onboarding screen with:
- Quick start tutorial
- Recent projects
- Example browser
- Getting started guide
- Video tutorials (links)

Author: RoboShire Team
Phase: 10 (UX Enhancement)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QFrame, QGridLayout, QSpacerItem, QSizePolicy
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QPixmap, QPalette, QColor
from pathlib import Path
import json


class WelcomeScreen(QWidget):
    """
    Welcome screen shown on first launch or via Help menu
    """

    # Signals for user actions
    new_project_requested = Signal()
    new_from_example_requested = Signal()
    new_robot_wizard_requested = Signal()
    open_project_requested = Signal(str)  # project_path
    open_documentation_requested = Signal(str)  # doc_url
    close_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.recent_projects = self._load_recent_projects()
        self._setup_ui()

    def _setup_ui(self):
        """Setup UI"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Create scroll area for content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        # Content widget
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(40, 40, 40, 40)
        content_layout.setSpacing(30)

        # Header
        header = self._create_header()
        content_layout.addWidget(header)

        # Quick Actions Section
        quick_actions = self._create_quick_actions()
        content_layout.addWidget(quick_actions)

        # Recent Projects Section
        if self.recent_projects:
            recent = self._create_recent_projects()
            content_layout.addWidget(recent)

        # Learning Resources Section
        learning = self._create_learning_resources()
        content_layout.addWidget(learning)

        # Getting Started Guide
        getting_started = self._create_getting_started()
        content_layout.addWidget(getting_started)

        # Add stretch to push content to top
        content_layout.addStretch()

        # Close button at bottom
        close_layout = QHBoxLayout()
        close_layout.addStretch()
        close_btn = QPushButton("Close Welcome Screen")
        close_btn.clicked.connect(self.close_requested.emit)
        close_layout.addWidget(close_btn)
        content_layout.addLayout(close_layout)

        scroll_area.setWidget(content)
        main_layout.addWidget(scroll_area)

    def _create_header(self) -> QWidget:
        """Create welcome header with logo and brand colors"""
        from roboshire.resources import get_logo_path, BRAND_COLORS

        header = QFrame()
        header.setStyleSheet(f"""
            QFrame {{
                background-color: {BRAND_COLORS['olive_green']};
                border-radius: 8px;
                padding: 20px;
            }}
        """)

        layout = QVBoxLayout(header)

        # Logo (centered)
        try:
            logo_label = QLabel()
            pixmap = QPixmap(str(get_logo_path(with_text=True)))
            # Scale logo to fit header
            pixmap = pixmap.scaledToWidth(400, Qt.TransformationMode.SmoothTransformation)
            logo_label.setPixmap(pixmap)
            logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(logo_label)
        except Exception as e:
            # Fallback to text title if logo fails
            title = QLabel("Welcome to RoboShire!")
            title_font = QFont()
            title_font.setPointSize(32)
            title_font.setBold(True)
            title.setFont(title_font)
            title.setStyleSheet("color: white;")
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(title)

        # Subtitle
        subtitle = QLabel(
            "Your beginner-friendly visual IDE for ROS2 robotics\n"
            "Build robots as easily as programming an Arduino!"
        )
        subtitle_font = QFont()
        subtitle_font.setPointSize(14)
        subtitle.setFont(subtitle_font)
        subtitle.setStyleSheet("color: white; margin-top: 15px;")
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(subtitle)

        # Version
        version = QLabel("v0.12.0 - Discovery & Search Complete")
        version.setStyleSheet("color: rgba(255, 255, 255, 0.9); margin-top: 5px;")
        version.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(version)

        return header

    def _create_quick_actions(self) -> QWidget:
        """Create quick action buttons"""
        group = QFrame()
        group.setStyleSheet("""
            QFrame {
                background-color: #f5f5f5;
                border-radius: 8px;
                padding: 20px;
            }
        """)

        layout = QVBoxLayout(group)

        # Section title
        title = QLabel("🚀 Quick Start")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Buttons grid
        buttons_layout = QGridLayout()
        buttons_layout.setSpacing(15)

        # Button 1: New Robot Wizard (RECOMMENDED)
        wizard_btn = self._create_action_button(
            "🎯 New Robot Wizard",
            "5-minute guided setup\n(RECOMMENDED FOR BEGINNERS)",
            "#4CAF50"
        )
        wizard_btn.clicked.connect(self.new_robot_wizard_requested.emit)
        buttons_layout.addWidget(wizard_btn, 0, 0)

        # Button 2: New from Example
        example_btn = self._create_action_button(
            "📚 Open Example",
            "Start from a pre-built robot\n(6 examples included)",
            "#2196F3"
        )
        example_btn.clicked.connect(self.new_from_example_requested.emit)
        buttons_layout.addWidget(example_btn, 0, 1)

        # Button 3: New Project
        new_btn = self._create_action_button(
            "📄 New Project",
            "Start from scratch\n(For advanced users)",
            "#FF9800"
        )
        new_btn.clicked.connect(self.new_project_requested.emit)
        buttons_layout.addWidget(new_btn, 1, 0)

        # Button 4: Open Recent
        open_btn = self._create_action_button(
            "📂 Open Project",
            "Continue previous work\n(Load .rsproj file)",
            "#9C27B0"
        )
        open_btn.clicked.connect(lambda: self.open_project_requested.emit(""))
        buttons_layout.addWidget(open_btn, 1, 1)

        layout.addLayout(buttons_layout)

        return group

    def _create_action_button(self, title: str, description: str, color: str) -> QPushButton:
        """Create styled action button"""
        btn = QPushButton()
        btn.setMinimumHeight(120)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                border-radius: 8px;
                padding: 15px;
                text-align: left;
                font-size: 14px;
            }}
            QPushButton:hover {{
                background-color: {self._darken_color(color)};
            }}
        """)

        layout = QVBoxLayout(btn)
        layout.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        # Title
        title_label = QLabel(title)
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("color: white;")
        layout.addWidget(title_label)

        # Description
        desc_label = QLabel(description)
        desc_label.setStyleSheet("color: rgba(255, 255, 255, 0.9); margin-top: 5px;")
        desc_label.setWordWrap(True)
        layout.addWidget(desc_label)

        return btn

    def _create_recent_projects(self) -> QWidget:
        """Create recent projects section"""
        group = QFrame()
        group.setStyleSheet("""
            QFrame {
                background-color: #f5f5f5;
                border-radius: 8px;
                padding: 20px;
            }
        """)

        layout = QVBoxLayout(group)

        # Section title
        title = QLabel("⏱️ Recent Projects")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Recent project list
        for project in self.recent_projects[:5]:  # Show max 5
            project_btn = QPushButton()
            project_btn.setMinimumHeight(50)
            project_btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    border: 1px solid #ddd;
                    border-radius: 4px;
                    padding: 10px;
                    text-align: left;
                }
                QPushButton:hover {
                    background-color: #e3f2fd;
                }
            """)

            project_path = Path(project["path"])
            project_btn.setText(f"📁 {project_path.name}\n   {project_path.parent}")
            project_btn.clicked.connect(
                lambda checked, p=project["path"]: self.open_project_requested.emit(p)
            )
            layout.addWidget(project_btn)

        return group

    def _create_learning_resources(self) -> QWidget:
        """Create learning resources section"""
        group = QFrame()
        group.setStyleSheet("""
            QFrame {
                background-color: #f5f5f5;
                border-radius: 8px;
                padding: 20px;
            }
        """)

        layout = QVBoxLayout(group)

        # Section title
        title = QLabel("📖 Learning Resources")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Resources grid
        resources_layout = QGridLayout()
        resources_layout.setSpacing(10)

        resources = [
            ("Quick Start Guide", "10-minute tutorial", "docs/QUICK_START.md"),
            ("URDF to Robot Tutorial", "Complete workflow", "docs/guides/urdf_to_robot.md"),
            ("MuJoCo Integration", "3D visualization", "docs/guides/mujoco_integration.md"),
            ("Keyboard Shortcuts", "Speed up workflow", "help://shortcuts"),
            ("SSH Setup Guide", "Configure VM", "docs/guides/ssh_setup.md"),
            ("Testing Guide", "Validate your robot", "docs/testing/TESTING_GUIDE.md"),
        ]

        for i, (name, desc, link) in enumerate(resources):
            resource_btn = self._create_resource_button(name, desc, link)
            resources_layout.addWidget(resource_btn, i // 2, i % 2)

        layout.addLayout(resources_layout)

        return group

    def _create_resource_button(self, name: str, description: str, link: str) -> QPushButton:
        """Create resource button"""
        btn = QPushButton()
        btn.setMinimumHeight(70)
        btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: 1px solid #2196F3;
                border-radius: 4px;
                padding: 10px;
                text-align: left;
            }
            QPushButton:hover {
                background-color: #e3f2fd;
            }
        """)

        layout = QVBoxLayout(btn)
        layout.setAlignment(Qt.AlignLeft)

        # Name
        name_label = QLabel(name)
        name_font = QFont()
        name_font.setBold(True)
        name_label.setFont(name_font)
        layout.addWidget(name_label)

        # Description
        desc_label = QLabel(description)
        desc_label.setStyleSheet("color: #666; font-size: 11px;")
        layout.addWidget(desc_label)

        btn.clicked.connect(lambda: self.open_documentation_requested.emit(link))

        return btn

    def _create_getting_started(self) -> QWidget:
        """Create getting started guide"""
        group = QFrame()
        group.setStyleSheet("""
            QFrame {
                background-color: #fff3e0;
                border-left: 4px solid #FF9800;
                border-radius: 4px;
                padding: 20px;
            }
        """)

        layout = QVBoxLayout(group)

        # Title
        title = QLabel("💡 First Time Using RoboShire?")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Step-by-step guide
        steps = [
            ("1️⃣", "Click 'New Robot Wizard' above", "Follow the 5-minute guided setup"),
            ("2️⃣", "Import a URDF or start from scratch", "Try one of our 6 example robots first!"),
            ("3️⃣", "Design your node graph", "Drag-and-drop nodes to create robot behavior"),
            ("4️⃣", "Generate ROS2 code", "Click Generate Code (Ctrl+G) - we handle the complexity!"),
            ("5️⃣", "Build on Ubuntu VM", "Click Build (Ctrl+B) - watch live output"),
            ("6️⃣", "Run and test your robot", "Click Run (Ctrl+R) and monitor with built-in tools"),
        ]

        for emoji, step, detail in steps:
            step_layout = QHBoxLayout()

            emoji_label = QLabel(emoji)
            emoji_label.setStyleSheet("font-size: 20px;")
            step_layout.addWidget(emoji_label)

            text_layout = QVBoxLayout()
            step_label = QLabel(step)
            step_font = QFont()
            step_font.setBold(True)
            step_label.setFont(step_font)
            text_layout.addWidget(step_label)

            detail_label = QLabel(detail)
            detail_label.setStyleSheet("color: #666; font-size: 11px;")
            text_layout.addWidget(detail_label)

            step_layout.addLayout(text_layout, 1)
            layout.addLayout(step_layout)

        # Pro tip
        tip = QLabel(
            "💡 <b>Pro Tip:</b> Press <b>F1</b> or <b>Ctrl+/</b> anytime to see all keyboard shortcuts!"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("margin-top: 15px; padding: 10px; background-color: white; border-radius: 4px;")
        layout.addWidget(tip)

        return group

    def _darken_color(self, hex_color: str) -> str:
        """Darken a hex color by 20% for hover effects"""
        # Remove # if present
        hex_color = hex_color.lstrip('#')

        # Convert to RGB
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)

        # Darken by 20%
        r = int(r * 0.8)
        g = int(g * 0.8)
        b = int(b * 0.8)

        return f"#{r:02x}{g:02x}{b:02x}"

    def _load_recent_projects(self) -> list:
        """Load recent projects from settings"""
        # TODO: Integrate with SettingsManager
        recent_file = Path.home() / ".roboshire" / "recent_projects.json"

        if recent_file.exists():
            try:
                with open(recent_file, 'r') as f:
                    return json.load(f)
            except Exception:
                return []

        return []

    @staticmethod
    def _save_recent_project(project_path: str):
        """Save project to recent list"""
        recent_file = Path.home() / ".roboshire" / "recent_projects.json"
        recent_file.parent.mkdir(parents=True, exist_ok=True)

        # Load existing
        recent = []
        if recent_file.exists():
            try:
                with open(recent_file, 'r') as f:
                    recent = json.load(f)
            except Exception:
                recent = []

        # Add new project (remove duplicates, keep newest)
        project_data = {"path": str(project_path), "name": Path(project_path).name}
        recent = [p for p in recent if p["path"] != str(project_path)]
        recent.insert(0, project_data)

        # Keep only 10 most recent
        recent = recent[:10]

        # Save
        try:
            with open(recent_file, 'w') as f:
                json.dump(recent, f, indent=2)
        except Exception:
            pass
