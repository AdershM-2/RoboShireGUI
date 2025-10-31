"""
Example Browser Widget for RoboShire Phase 6

Provides a visual browser for discovering and loading robot examples.
Each example includes a URDF model, node graph configuration, and documentation.

Architecture:
- Uses ExampleManager backend for example discovery
- Displays examples in filterable list with preview
- Shows detailed metadata and requirements
- One-click loading into main editor

Author: RoboShire Team
Phase: 6 (Advanced Features)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QListWidget, QListWidgetItem, QTextEdit, QComboBox,
    QLineEdit, QSplitter, QGroupBox, QMessageBox, QScrollArea
)
from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QPixmap, QFont, QColor, QPalette
from typing import Optional, List
import os

from roboshire.backend.example_manager import ExampleManager, RobotExample


class ExampleBrowserWidget(QWidget):
    """
    Widget for browsing and loading robot examples.

    Signals:
        example_loaded: Emitted when user loads an example (urdf_path, node_graph_data)
    """

    example_loaded = Signal(str, dict)  # (urdf_content, node_graph_dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        # Backend
        self.example_manager = ExampleManager()
        self.all_examples: List[RobotExample] = []
        self.filtered_examples: List[RobotExample] = []
        self.current_example: Optional[RobotExample] = None

        # UI Setup
        self._init_ui()
        self._connect_signals()

        # Load examples
        self.refresh_examples()

    def _init_ui(self):
        """Initialize the user interface."""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Title
        title_label = QLabel("Robot Examples Library")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        main_layout.addWidget(title_label)

        # Subtitle
        subtitle_label = QLabel("Browse and load pre-built robot examples with URDF models and node graphs")
        subtitle_label.setStyleSheet("color: #666; margin-bottom: 10px;")
        main_layout.addWidget(subtitle_label)

        # Search and Filter Bar
        filter_layout = QHBoxLayout()

        # Search box
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Search examples...")
        self.search_box.setClearButtonEnabled(True)
        filter_layout.addWidget(QLabel("Search:"))
        filter_layout.addWidget(self.search_box, stretch=2)

        # Category filter
        self.category_combo = QComboBox()
        self.category_combo.addItem("All Categories")
        filter_layout.addWidget(QLabel("Category:"))
        filter_layout.addWidget(self.category_combo, stretch=1)

        # Difficulty filter
        self.difficulty_combo = QComboBox()
        self.difficulty_combo.addItem("All Levels")
        self.difficulty_combo.addItem("Beginner")
        self.difficulty_combo.addItem("Intermediate")
        self.difficulty_combo.addItem("Advanced")
        filter_layout.addWidget(QLabel("Difficulty:"))
        filter_layout.addWidget(self.difficulty_combo, stretch=1)

        # Refresh button
        self.refresh_btn = QPushButton("Refresh")
        filter_layout.addWidget(self.refresh_btn)

        main_layout.addLayout(filter_layout)

        # Splitter for list and details
        splitter = QSplitter(Qt.Horizontal)

        # Left: Example list
        list_widget = QWidget()
        list_layout = QVBoxLayout(list_widget)
        list_layout.setContentsMargins(0, 0, 0, 0)

        list_label = QLabel("Available Examples")
        list_label.setStyleSheet("font-weight: bold; margin-top: 5px;")
        list_layout.addWidget(list_label)

        self.example_list = QListWidget()
        self.example_list.setMinimumWidth(250)
        list_layout.addWidget(self.example_list)

        splitter.addWidget(list_widget)

        # Right: Example details
        details_widget = QWidget()
        details_layout = QVBoxLayout(details_widget)
        details_layout.setContentsMargins(0, 0, 0, 0)

        # Scrollable details area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        scroll_content = QWidget()
        self.details_layout = QVBoxLayout(scroll_content)
        self.details_layout.setContentsMargins(10, 10, 10, 10)

        # Preview image
        self.preview_group = QGroupBox("Preview")
        preview_layout = QVBoxLayout(self.preview_group)
        self.preview_label = QLabel()
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumHeight(200)
        self.preview_label.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc;")
        self.preview_label.setText("No preview available")
        preview_layout.addWidget(self.preview_label)
        self.details_layout.addWidget(self.preview_group)

        # Info group
        self.info_group = QGroupBox("Information")
        info_layout = QVBoxLayout(self.info_group)

        self.name_label = QLabel("<b>Name:</b> Select an example")
        self.name_label.setWordWrap(True)
        info_layout.addWidget(self.name_label)

        self.version_label = QLabel("<b>Version:</b> -")
        info_layout.addWidget(self.version_label)

        self.author_label = QLabel("<b>Author:</b> -")
        info_layout.addWidget(self.author_label)

        self.category_label = QLabel("<b>Category:</b> -")
        info_layout.addWidget(self.category_label)

        self.difficulty_label = QLabel("<b>Difficulty:</b> -")
        info_layout.addWidget(self.difficulty_label)

        self.license_label = QLabel("<b>License:</b> -")
        info_layout.addWidget(self.license_label)

        self.details_layout.addWidget(self.info_group)

        # Description group
        self.desc_group = QGroupBox("Description")
        desc_layout = QVBoxLayout(self.desc_group)
        self.desc_text = QTextEdit()
        self.desc_text.setReadOnly(True)
        self.desc_text.setMaximumHeight(100)
        desc_layout.addWidget(self.desc_text)
        self.details_layout.addWidget(self.desc_group)

        # Features group
        self.features_group = QGroupBox("Features")
        features_layout = QVBoxLayout(self.features_group)
        self.features_text = QTextEdit()
        self.features_text.setReadOnly(True)
        self.features_text.setMaximumHeight(120)
        features_layout.addWidget(self.features_text)
        self.details_layout.addWidget(self.features_group)

        # Requirements group
        self.req_group = QGroupBox("Requirements")
        req_layout = QVBoxLayout(self.req_group)
        self.req_text = QTextEdit()
        self.req_text.setReadOnly(True)
        self.req_text.setMaximumHeight(100)
        req_layout.addWidget(self.req_text)
        self.details_layout.addWidget(self.req_group)

        # Use cases group
        self.usecase_group = QGroupBox("Use Cases")
        usecase_layout = QVBoxLayout(self.usecase_group)
        self.usecase_text = QTextEdit()
        self.usecase_text.setReadOnly(True)
        self.usecase_text.setMaximumHeight(100)
        usecase_layout.addWidget(self.usecase_text)
        self.details_layout.addWidget(self.usecase_group)

        # Tags
        self.tags_label = QLabel("<b>Tags:</b> -")
        self.tags_label.setWordWrap(True)
        self.details_layout.addWidget(self.tags_label)

        self.details_layout.addStretch()

        scroll_area.setWidget(scroll_content)
        details_layout.addWidget(scroll_area)

        # Load button at bottom of details
        self.load_btn = QPushButton("Load Example")
        self.load_btn.setEnabled(False)
        self.load_btn.setMinimumHeight(40)
        self.load_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        details_layout.addWidget(self.load_btn)

        splitter.addWidget(details_widget)

        # Set splitter sizes
        splitter.setSizes([300, 500])

        main_layout.addWidget(splitter)

        # Status bar
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: #666; font-size: 10px;")
        main_layout.addWidget(self.status_label)

    def _connect_signals(self):
        """Connect UI signals to handlers."""
        self.search_box.textChanged.connect(self._apply_filters)
        self.category_combo.currentTextChanged.connect(self._apply_filters)
        self.difficulty_combo.currentTextChanged.connect(self._apply_filters)
        self.refresh_btn.clicked.connect(self.refresh_examples)
        self.example_list.currentItemChanged.connect(self._on_example_selected)
        self.load_btn.clicked.connect(self._on_load_example)

    def refresh_examples(self):
        """Discover and load all available examples."""
        self.status_label.setText("Discovering examples...")

        # Discover examples
        self.all_examples = self.example_manager.discover_examples()

        # Update category filter
        categories = set()
        for example in self.all_examples:
            if example.category:
                categories.add(example.category)

        current_category = self.category_combo.currentText()
        self.category_combo.clear()
        self.category_combo.addItem("All Categories")
        for category in sorted(categories):
            self.category_combo.addItem(category)

        # Restore previous selection if possible
        index = self.category_combo.findText(current_category)
        if index >= 0:
            self.category_combo.setCurrentIndex(index)

        # Apply filters
        self._apply_filters()

        self.status_label.setText(f"Found {len(self.all_examples)} examples")

    def _apply_filters(self):
        """Apply search and filter criteria to example list."""
        search_text = self.search_box.text().lower()
        category = self.category_combo.currentText()
        difficulty = self.difficulty_combo.currentText()

        # Filter examples
        self.filtered_examples = []
        for example in self.all_examples:
            # Search filter
            if search_text:
                searchable = (
                    example.name.lower() + " " +
                    example.description.lower() + " " +
                    " ".join(example.tags).lower()
                )
                if search_text not in searchable:
                    continue

            # Category filter
            if category != "All Categories" and example.category != category:
                continue

            # Difficulty filter
            if difficulty != "All Levels" and example.difficulty.title() != difficulty:
                continue

            self.filtered_examples.append(example)

        # Update list
        self._populate_list()

    def _populate_list(self):
        """Populate the example list with filtered examples."""
        self.example_list.clear()

        for example in self.filtered_examples:
            item = QListWidgetItem(example.name)

            # Set tooltip with description
            item.setToolTip(example.description)

            # Color code by difficulty
            if example.difficulty == "beginner":
                item.setForeground(QColor("#4CAF50"))  # Green
            elif example.difficulty == "intermediate":
                item.setForeground(QColor("#FF9800"))  # Orange
            elif example.difficulty == "advanced":
                item.setForeground(QColor("#F44336"))  # Red

            # Store example reference
            item.setData(Qt.UserRole, example)

            self.example_list.addItem(item)

        # Update status
        self.status_label.setText(f"Showing {len(self.filtered_examples)} of {len(self.all_examples)} examples")

    def _on_example_selected(self, current: QListWidgetItem, previous: QListWidgetItem):
        """Handle example selection."""
        if not current:
            self.current_example = None
            self.load_btn.setEnabled(False)
            return

        self.current_example = current.data(Qt.UserRole)
        self._display_example_details(self.current_example)
        self.load_btn.setEnabled(True)

    def _display_example_details(self, example: RobotExample):
        """Display detailed information for selected example."""
        # Update info labels
        self.name_label.setText(f"<b>Name:</b> {example.name}")
        self.version_label.setText(f"<b>Version:</b> {example.version}")
        self.author_label.setText(f"<b>Author:</b> {example.author}")
        self.category_label.setText(f"<b>Category:</b> {example.category or 'N/A'}")

        # Color-coded difficulty
        difficulty_colors = {
            "beginner": "#4CAF50",
            "intermediate": "#FF9800",
            "advanced": "#F44336"
        }
        color = difficulty_colors.get(example.difficulty, "#666")
        self.difficulty_label.setText(
            f"<b>Difficulty:</b> <span style='color: {color};'>{example.difficulty.title()}</span>"
        )

        self.license_label.setText(f"<b>License:</b> {example.license or 'N/A'}")

        # Description
        self.desc_text.setPlainText(example.description)

        # Features
        if example.features:
            features_text = "\n".join(f"- {feature}" for feature in example.features)
            self.features_text.setPlainText(features_text)
        else:
            self.features_text.setPlainText("No features listed")

        # Requirements
        req_parts = []
        if example.requirements:
            if example.requirements.get("ros2_packages"):
                req_parts.append("ROS2 Packages:")
                for pkg in example.requirements["ros2_packages"]:
                    req_parts.append(f"  - {pkg}")
            if example.requirements.get("optional_packages"):
                req_parts.append("\nOptional Packages:")
                for pkg in example.requirements["optional_packages"]:
                    req_parts.append(f"  - {pkg}")

        self.req_text.setPlainText("\n".join(req_parts) if req_parts else "No requirements listed")

        # Use cases
        if example.use_cases:
            usecases_text = "\n".join(f"- {uc}" for uc in example.use_cases)
            self.usecase_text.setPlainText(usecases_text)
        else:
            self.usecase_text.setPlainText("No use cases listed")

        # Tags
        if example.tags:
            tags_html = ", ".join(f"<span style='background-color: #e0e0e0; padding: 2px 6px; border-radius: 3px;'>{tag}</span>" for tag in example.tags)
            self.tags_label.setText(f"<b>Tags:</b> {tags_html}")
        else:
            self.tags_label.setText("<b>Tags:</b> None")

        # Preview image
        if example.files.get("preview"):
            preview_path = os.path.join(example.directory, example.files["preview"])
            if os.path.exists(preview_path):
                pixmap = QPixmap(preview_path)
                if not pixmap.isNull():
                    scaled_pixmap = pixmap.scaled(
                        400, 300,
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation
                    )
                    self.preview_label.setPixmap(scaled_pixmap)
                else:
                    self.preview_label.setText("Preview image failed to load")
            else:
                self.preview_label.setText("Preview image not found")
        else:
            self.preview_label.setText("No preview available")

    def _on_load_example(self):
        """Load the selected example."""
        if not self.current_example:
            return

        # Confirm with user
        reply = QMessageBox.question(
            self,
            "Load Example",
            f"Load '{self.current_example.name}'?\n\n"
            "This will replace your current URDF and node graph.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply != QMessageBox.Yes:
            return

        # Load URDF
        urdf_content = self.example_manager.load_example_urdf(self.current_example.id)
        if not urdf_content:
            QMessageBox.warning(
                self,
                "Load Failed",
                f"Failed to load URDF file for '{self.current_example.name}'"
            )
            return

        # Load node graph
        node_graph = self.example_manager.load_example_node_graph(self.current_example.id)
        if not node_graph:
            QMessageBox.warning(
                self,
                "Load Failed",
                f"Failed to load node graph for '{self.current_example.name}'"
            )
            return

        # Emit signal
        self.example_loaded.emit(urdf_content, node_graph)

        # Show success message
        QMessageBox.information(
            self,
            "Example Loaded",
            f"Successfully loaded '{self.current_example.name}'!\n\n"
            "The URDF model and node graph are now ready to use."
        )

        self.status_label.setText(f"Loaded: {self.current_example.name}")
