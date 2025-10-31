"""
Package File Tree Widget - GUI for viewing package file structure

Features:
- Package selection dropdown
- File tree view of selected package
- Double-click to open files
- Icons for different file types
"""

import logging
from pathlib import Path
from typing import Optional, List
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QLabel, QComboBox, QPushButton
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor


class PackageFileTreeWidget(QWidget):
    """
    Widget showing file tree of selected package

    Features:
    - Package selector dropdown
    - Hierarchical file tree
    - File type icons
    - Double-click to open
    """

    # Signals
    file_selected = Signal(str)  # file_path

    def __init__(self, parent=None):
        """
        Initialize package file tree widget

        Args:
            parent: Parent widget
        """
        super().__init__(parent)

        self.workspace_path = Path("workspace")
        self.current_package: Optional[str] = None

        self._setup_ui()

        logging.info("PackageFileTreeWidget initialized")

    def _setup_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout(self)

        # Title
        title_label = QLabel("<h3>Package Files</h3>")
        layout.addWidget(title_label)

        # Package selector
        selector_layout = QHBoxLayout()
        selector_layout.addWidget(QLabel("<b>Package:</b>"))

        self.package_combo = QComboBox()
        self.package_combo.currentTextChanged.connect(self._on_package_selected)
        selector_layout.addWidget(self.package_combo, 1)

        # Refresh button (v2.5.0)
        self.refresh_button = QPushButton("🔄 Refresh")
        self.refresh_button.setToolTip("Refresh package files and Arduino folder")
        self.refresh_button.clicked.connect(self._on_refresh)
        self.refresh_button.setMaximumWidth(100)
        selector_layout.addWidget(self.refresh_button)

        layout.addLayout(selector_layout)

        # File tree
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Files"])
        self.tree.itemDoubleClicked.connect(self._on_item_double_clicked)
        self.tree.itemClicked.connect(self._on_item_clicked)  # Single click support
        self.tree.setAlternatingRowColors(True)
        layout.addWidget(self.tree)

        # Info label
        self.info_label = QLabel("Select a package to view its files")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setStyleSheet("color: #666; font-style: italic; padding: 10px;")
        layout.addWidget(self.info_label)

    def update_packages(self, packages: List[str]):
        """
        Update package list

        Args:
            packages: List of package names
        """
        self.package_combo.clear()
        self.package_combo.addItems(packages)

        if packages:
            self.package_combo.setCurrentIndex(0)
            self.info_label.setText(f"{len(packages)} package(s) available")
        else:
            self.info_label.setText("No packages found - generate code first")

    def _on_package_selected(self, package_name: str):
        """
        Load file tree for selected package

        Args:
            package_name: Selected package name
        """
        if not package_name:
            return

        self.current_package = package_name
        self.tree.clear()

        # Build file tree
        package_path = self.workspace_path / "src" / package_name

        if not package_path.exists():
            self.info_label.setText(f"Package not found: {package_path}")
            logging.warning(f"Package path does not exist: {package_path}")
            return

        # Add root item
        root = QTreeWidgetItem(self.tree)
        root.setText(0, f"📦 {package_name}")
        root.setData(0, Qt.UserRole, str(package_path))

        # Recursively add files
        self._add_directory_items(root, package_path)

        # Add Arduino/Firmware folder (v2.5.0)
        self._add_arduino_folder(root, package_name)

        root.setExpanded(True)
        self.info_label.setText(f"Showing files for: {package_name}")

        logging.info(f"Loaded file tree for package: {package_name}")

    def _add_directory_items(self, parent_item: QTreeWidgetItem, directory: Path):
        """
        Recursively add directory contents to tree

        Args:
            parent_item: Parent tree item
            directory: Directory path
        """
        try:
            for item in sorted(directory.iterdir()):
                # Skip hidden files and __pycache__
                if item.name.startswith('.') or item.name == '__pycache__':
                    continue

                tree_item = QTreeWidgetItem(parent_item)
                tree_item.setData(0, Qt.UserRole, str(item))

                if item.is_dir():
                    # Directory icon and name
                    tree_item.setText(0, f"📁 {item.name}")
                    tree_item.setExpanded(False)
                    # Recursively add subdirectory contents
                    self._add_directory_items(tree_item, item)
                else:
                    # File icon based on extension
                    icon = self._get_file_icon(item.suffix)
                    tree_item.setText(0, f"{icon} {item.name}")

        except PermissionError as e:
            logging.warning(f"Permission denied accessing {directory}: {e}")
        except Exception as e:
            logging.error(f"Error adding directory items for {directory}: {e}")

    def _get_file_icon(self, extension: str) -> str:
        """
        Get icon emoji for file extension

        Args:
            extension: File extension (e.g., '.py')

        Returns:
            Icon emoji string
        """
        icon_map = {
            '.py': '🐍',      # Python files
            '.xml': '📄',     # XML files (package.xml)
            '.yaml': '⚙️',    # YAML files
            '.yml': '⚙️',     # YAML files
            '.launch': '🚀',  # Launch files
            '.urdf': '🤖',    # URDF files
            '.sdf': '🤖',     # SDF files
            '.msg': '📨',     # Message files
            '.srv': '🔧',     # Service files
            '.action': '⚡',  # Action files
            '.md': '📝',      # Markdown files
            '.txt': '📄',     # Text files
            '.json': '📋',    # JSON files
            '.cfg': '⚙️',     # Config files
        }
        return icon_map.get(extension.lower(), '📄')

    def _on_item_clicked(self, item: QTreeWidgetItem, column: int):
        """
        Handle item single click - open file if code editor is active

        Args:
            item: Clicked tree item
            column: Column index
        """
        file_path = item.data(0, Qt.UserRole)
        if file_path and Path(file_path).is_file():
            # Check if code editor is currently active/visible
            # Emit signal - let main window decide if it should open
            logging.info(f"File clicked: {file_path}")
            self.file_selected.emit(file_path)

    def _on_item_double_clicked(self, item: QTreeWidgetItem, column: int):
        """
        Handle item double click - open file

        Args:
            item: Double-clicked tree item
            column: Column index
        """
        file_path = item.data(0, Qt.UserRole)
        if file_path and Path(file_path).is_file():
            logging.info(f"Opening file: {file_path}")
            self.file_selected.emit(file_path)

    def _add_arduino_folder(self, parent_item: QTreeWidgetItem, package_name: str):
        """
        Add Arduino/Firmware folder to package tree (v2.5.0)

        Looks for Arduino code in workspace/projects/<project_name>/arduino/

        Args:
            parent_item: Parent tree item (package root)
            package_name: Package name to find associated project
        """
        # Try to find project with this package
        projects_path = Path("workspace/projects")

        if not projects_path.exists():
            return

        # Look for arduino folder in any project
        arduino_folders = []
        for project_dir in projects_path.iterdir():
            if project_dir.is_dir():
                arduino_path = project_dir / "arduino"
                if arduino_path.exists() and arduino_path.is_dir():
                    arduino_folders.append((project_dir.name, arduino_path))

        if not arduino_folders:
            # No Arduino code found - create placeholder
            arduino_item = QTreeWidgetItem(parent_item)
            arduino_item.setText(0, "🔧 arduino (empty - generate firmware code)")
            arduino_item.setForeground(0, QColor(128, 128, 128))
            return

        # Add Arduino folder(s)
        for project_name, arduino_path in arduino_folders:
            arduino_item = QTreeWidgetItem(parent_item)
            arduino_item.setText(0, f"🔧 arduino ({project_name})")
            arduino_item.setData(0, Qt.UserRole, str(arduino_path))
            arduino_item.setExpanded(True)

            # Add Arduino files
            self._add_arduino_files(arduino_item, arduino_path)

    def _add_arduino_files(self, parent_item: QTreeWidgetItem, arduino_path: Path):
        """
        Add Arduino files to tree

        Args:
            parent_item: Parent tree item
            arduino_path: Path to arduino folder
        """
        try:
            for item in sorted(arduino_path.iterdir()):
                # Skip hidden files
                if item.name.startswith('.'):
                    continue

                tree_item = QTreeWidgetItem(parent_item)
                tree_item.setData(0, Qt.UserRole, str(item))

                if item.is_dir():
                    tree_item.setText(0, f"📁 {item.name}")
                    # Recursively add subdirectory
                    self._add_arduino_files(tree_item, item)
                else:
                    # Arduino file icons
                    icon = self._get_arduino_file_icon(item.suffix)
                    tree_item.setText(0, f"{icon} {item.name}")

        except PermissionError as e:
            logging.warning(f"Permission denied accessing {arduino_path}: {e}")
        except Exception as e:
            logging.error(f"Error adding Arduino files for {arduino_path}: {e}")

    def _get_arduino_file_icon(self, extension: str) -> str:
        """
        Get icon for Arduino file types

        Args:
            extension: File extension

        Returns:
            Icon emoji
        """
        icon_map = {
            '.ino': '⚡',      # Arduino sketch
            '.cpp': '💻',     # C++ source
            '.h': '📋',       # Header file
            '.c': '💻',       # C source
            '.hpp': '📋',     # C++ header
            '.txt': '📄',     # Text/docs
            '.md': '📝',      # Markdown
            '.hex': '🔢',     # Compiled hex file
        }
        return icon_map.get(extension.lower(), '📄')

    def _on_refresh(self):
        """
        Refresh package file tree (v2.5.0)

        Reloads the current package to show updated files and Arduino folder
        """
        current_package = self.package_combo.currentText()
        if current_package:
            logging.info(f"Refreshing package file tree for: {current_package}")
            self._on_package_selected(current_package)
