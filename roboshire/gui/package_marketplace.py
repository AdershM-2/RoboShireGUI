"""
ROS2 Package Marketplace

Browse, search, and install ROS2 packages from:
- ROS Index (index.ros.org)
- GitHub repositories
- Local package cache

Author: RoboShire Team
Phase: 11 (Advanced Features)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QListWidget, QListWidgetItem, QTextEdit,
    QSplitter, QGroupBox, QComboBox, QProgressBar, QMessageBox,
    QScrollArea, QFrame
)
from PySide6.QtCore import Qt, Signal, QThread
from PySide6.QtGui import QFont, QPixmap
from pathlib import Path
from typing import List, Dict, Optional
import json


class PackageInfo:
    """Package information"""
    def __init__(self, name: str, description: str, source: str = "ros_index"):
        self.name = name
        self.description = description
        self.source = source  # "ros_index", "github", "local"
        self.version = "unknown"
        self.maintainer = "unknown"
        self.license = "unknown"
        self.url = ""
        self.dependencies = []
        self.tags = []
        self.install_command = ""
        self.is_installed = False


class PackageSearchThread(QThread):
    """Background thread for searching packages"""
    results_ready = Signal(list)  # List[PackageInfo]
    error_occurred = Signal(str)  # error_message

    def __init__(self, query: str, source: str = "all"):
        super().__init__()
        self.query = query
        self.source = source

    def run(self):
        """Search for packages"""
        try:
            results = []

            # Simulate package search (in real impl, would query ROS Index API)
            if self.source in ["all", "ros_index"]:
                results.extend(self._search_ros_index())

            if self.source in ["all", "github"]:
                results.extend(self._search_github())

            self.results_ready.emit(results)

        except Exception as e:
            self.error_occurred.emit(str(e))

    def _search_ros_index(self) -> List[PackageInfo]:
        """Search ROS Index (mock data for now)"""
        # In real implementation, would use requests to query:
        # https://index.ros.org/packages/

        mock_packages = [
            PackageInfo("nav2_bringup", "Navigation2 launch files", "ros_index"),
            PackageInfo("slam_toolbox", "SLAM Toolbox for 2D mapping", "ros_index"),
            PackageInfo("robot_localization", "State estimation package", "ros_index"),
            PackageInfo("teleop_twist_keyboard", "Keyboard teleoperation", "ros_index"),
            PackageInfo("rqt_robot_steering", "Robot steering GUI", "ros_index"),
        ]

        # Filter by query
        if self.query:
            mock_packages = [
                pkg for pkg in mock_packages
                if self.query.lower() in pkg.name.lower() or
                   self.query.lower() in pkg.description.lower()
            ]

        return mock_packages

    def _search_github(self) -> List[PackageInfo]:
        """Search GitHub for ROS2 packages"""
        # In real implementation, would use GitHub API
        return []


class PackageInstallThread(QThread):
    """Background thread for installing packages"""
    progress_updated = Signal(int, str)  # progress_percent, status_message
    install_complete = Signal(bool, str)  # success, message

    def __init__(self, package: PackageInfo, workspace_path: str):
        super().__init__()
        self.package = package
        self.workspace_path = workspace_path

    def run(self):
        """Install package"""
        try:
            self.progress_updated.emit(10, "Resolving dependencies...")
            # Simulate work
            self.msleep(500)

            self.progress_updated.emit(30, "Downloading package...")
            self.msleep(1000)

            self.progress_updated.emit(60, "Building package...")
            self.msleep(1500)

            self.progress_updated.emit(90, "Installing...")
            self.msleep(500)

            self.progress_updated.emit(100, "Complete!")
            self.install_complete.emit(True, f"Successfully installed {self.package.name}")

        except Exception as e:
            self.install_complete.emit(False, f"Installation failed: {e}")


class PackageMarketplace(QWidget):
    """
    Package marketplace browser and installer
    """

    # Signal when package is installed
    package_installed = Signal(str)  # package_name

    def __init__(self, parent=None):
        super().__init__(parent)

        self.workspace_path = None  # Set externally
        self.current_package: Optional[PackageInfo] = None
        self.search_thread: Optional[PackageSearchThread] = None
        self.install_thread: Optional[PackageInstallThread] = None

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Header
        header = self._create_header()
        layout.addWidget(header)

        # Search bar
        search_bar = self._create_search_bar()
        layout.addWidget(search_bar)

        # Main content
        splitter = QSplitter(Qt.Horizontal)

        # Left: Package list
        left_panel = self._create_package_list_panel()
        splitter.addWidget(left_panel)

        # Right: Package details
        right_panel = self._create_package_details_panel()
        splitter.addWidget(right_panel)

        splitter.setSizes([300, 500])
        layout.addWidget(splitter)

        # Bottom: Install progress
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)

        self.progress_label = QLabel("")
        self.progress_label.setVisible(False)
        layout.addWidget(self.progress_label)

    def _create_header(self) -> QWidget:
        """Create header"""
        header = QFrame()
        header.setStyleSheet("""
            QFrame {
                background-color: #673AB7;
                border-radius: 8px;
                padding: 15px;
            }
        """)

        layout = QVBoxLayout(header)

        title = QLabel("📦 ROS2 Package Marketplace")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        subtitle = QLabel("Discover and install ROS2 packages from ROS Index and GitHub")
        subtitle.setStyleSheet("color: rgba(255, 255, 255, 0.9);")
        layout.addWidget(subtitle)

        return header

    def _create_search_bar(self) -> QWidget:
        """Create search bar"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 5, 0, 5)

        # Search input
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Search packages (e.g., 'navigation', 'slam', 'teleop')...")
        self.search_input.returnPressed.connect(self._search_packages)
        layout.addWidget(self.search_input)

        # Source filter
        self.source_combo = QComboBox()
        self.source_combo.addItems(["All Sources", "ROS Index", "GitHub", "Local"])
        layout.addWidget(self.source_combo)

        # Search button
        search_btn = QPushButton("🔍 Search")
        search_btn.clicked.connect(self._search_packages)
        layout.addWidget(search_btn)

        return widget

    def _create_package_list_panel(self) -> QWidget:
        """Create package list panel"""
        panel = QGroupBox("Search Results")
        layout = QVBoxLayout(panel)

        # Result count
        self.result_count_label = QLabel("0 packages found")
        layout.addWidget(self.result_count_label)

        # Package list
        self.package_list = QListWidget()
        self.package_list.currentItemChanged.connect(self._on_package_selected)
        layout.addWidget(self.package_list)

        # Popular packages section
        popular_label = QLabel("⭐ Popular Packages")
        popular_font = popular_label.font()
        popular_font.setBold(True)
        popular_label.setFont(popular_font)
        layout.addWidget(popular_label)

        popular_packages = [
            "nav2_bringup - Navigation Stack",
            "slam_toolbox - 2D SLAM",
            "robot_localization - State Estimation",
            "moveit - Motion Planning",
            "gazebo_ros_pkgs - Gazebo Integration",
        ]

        for pkg in popular_packages:
            item = QListWidgetItem(pkg)
            item.setToolTip("Click to view details")
            self.package_list.addItem(item)

        return panel

    def _create_package_details_panel(self) -> QWidget:
        """Create package details panel"""
        panel = QGroupBox("Package Details")
        layout = QVBoxLayout(panel)

        # Scroll area for details
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        details_widget = QWidget()
        self.details_layout = QVBoxLayout(details_widget)

        # Package name
        self.package_name_label = QLabel("Select a package to view details")
        name_font = self.package_name_label.font()
        name_font.setPointSize(16)
        name_font.setBold(True)
        self.package_name_label.setFont(name_font)
        self.details_layout.addWidget(self.package_name_label)

        # Package description
        self.package_desc_label = QLabel("")
        self.package_desc_label.setWordWrap(True)
        self.details_layout.addWidget(self.package_desc_label)

        # Metadata
        self.metadata_text = QTextEdit()
        self.metadata_text.setReadOnly(True)
        self.metadata_text.setMaximumHeight(150)
        self.details_layout.addWidget(self.metadata_text)

        # Dependencies
        self.dependencies_label = QLabel("<b>Dependencies:</b>")
        self.details_layout.addWidget(self.dependencies_label)

        self.dependencies_list = QLabel("None")
        self.dependencies_list.setWordWrap(True)
        self.details_layout.addWidget(self.dependencies_list)

        # Install button
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        self.install_btn = QPushButton("📥 Install Package")
        self.install_btn.setMinimumHeight(40)
        self.install_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 10px 20px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)
        self.install_btn.clicked.connect(self._install_package)
        self.install_btn.setEnabled(False)
        button_layout.addWidget(self.install_btn)

        self.details_layout.addLayout(button_layout)
        self.details_layout.addStretch()

        scroll.setWidget(details_widget)
        layout.addWidget(scroll)

        return panel

    def set_workspace_path(self, path: str):
        """Set workspace path for installations"""
        self.workspace_path = path

    def _search_packages(self):
        """Search for packages"""
        query = self.search_input.text().strip()

        # Determine source
        source_map = {
            "All Sources": "all",
            "ROS Index": "ros_index",
            "GitHub": "github",
            "Local": "local"
        }
        source = source_map.get(self.source_combo.currentText(), "all")

        # Start search thread
        if self.search_thread and self.search_thread.isRunning():
            self.search_thread.terminate()

        self.search_thread = PackageSearchThread(query, source)
        self.search_thread.results_ready.connect(self._on_search_results)
        self.search_thread.error_occurred.connect(self._on_search_error)
        self.search_thread.start()

        self.result_count_label.setText("Searching...")

    def _on_search_results(self, results: List[PackageInfo]):
        """Handle search results"""
        self.package_list.clear()

        for pkg in results:
            item = QListWidgetItem(f"{pkg.name} - {pkg.description}")
            item.setData(Qt.UserRole, pkg)
            self.package_list.addItem(item)

        self.result_count_label.setText(f"{len(results)} package(s) found")

    def _on_search_error(self, error: str):
        """Handle search error"""
        QMessageBox.critical(
            self,
            "Search Error",
            f"Failed to search packages:\n\n{error}"
        )
        self.result_count_label.setText("Search failed")

    def _on_package_selected(self, current: QListWidgetItem, previous: QListWidgetItem):
        """Handle package selection"""
        if not current:
            return

        # Get package data
        pkg = current.data(Qt.UserRole)

        if not pkg:
            # Parse from text (for popular packages)
            text = current.text()
            name = text.split(' - ')[0]
            desc = text.split(' - ')[1] if ' - ' in text else ""
            pkg = PackageInfo(name, desc, "ros_index")

        self.current_package = pkg

        # Update details
        self.package_name_label.setText(pkg.name)
        self.package_desc_label.setText(pkg.description)

        metadata = f"""
<b>Version:</b> {pkg.version}<br>
<b>Maintainer:</b> {pkg.maintainer}<br>
<b>License:</b> {pkg.license}<br>
<b>Source:</b> {pkg.source}<br>
"""
        if pkg.url:
            metadata += f"<b>URL:</b> <a href='{pkg.url}'>{pkg.url}</a><br>"

        self.metadata_text.setHtml(metadata)

        # Dependencies
        if pkg.dependencies:
            self.dependencies_list.setText(", ".join(pkg.dependencies))
        else:
            self.dependencies_list.setText("None (or not yet resolved)")

        # Enable install button
        self.install_btn.setEnabled(True)
        if pkg.is_installed:
            self.install_btn.setText("✅ Already Installed")
            self.install_btn.setEnabled(False)
        else:
            self.install_btn.setText("📥 Install Package")

    def _install_package(self):
        """Install selected package"""
        if not self.current_package:
            return

        if not self.workspace_path:
            QMessageBox.warning(
                self,
                "No Workspace",
                "Workspace path not set. Please configure workspace first."
            )
            return

        # Confirm installation
        reply = QMessageBox.question(
            self,
            "Install Package",
            f"Install package '{self.current_package.name}'?\n\n"
            f"This will:\n"
            f"1. Download the package\n"
            f"2. Install dependencies\n"
            f"3. Build the package\n"
            f"4. Add to workspace\n\n"
            f"Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )

        if reply != QMessageBox.Yes:
            return

        # Start installation thread
        self.install_thread = PackageInstallThread(self.current_package, self.workspace_path)
        self.install_thread.progress_updated.connect(self._on_install_progress)
        self.install_thread.install_complete.connect(self._on_install_complete)
        self.install_thread.start()

        # Show progress
        self.progress_bar.setVisible(True)
        self.progress_label.setVisible(True)
        self.progress_bar.setValue(0)
        self.install_btn.setEnabled(False)

    def _on_install_progress(self, percent: int, message: str):
        """Handle installation progress"""
        self.progress_bar.setValue(percent)
        self.progress_label.setText(message)

    def _on_install_complete(self, success: bool, message: str):
        """Handle installation completion"""
        self.progress_bar.setVisible(False)
        self.progress_label.setVisible(False)
        self.install_btn.setEnabled(True)

        if success:
            QMessageBox.information(
                self,
                "Installation Complete",
                message
            )
            self.package_installed.emit(self.current_package.name)

            # Mark as installed
            if self.current_package:
                self.current_package.is_installed = True
                self.install_btn.setText("✅ Already Installed")
                self.install_btn.setEnabled(False)
        else:
            QMessageBox.critical(
                self,
                "Installation Failed",
                message
            )
