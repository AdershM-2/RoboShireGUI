"""
Package Browser Widget for RoboShire

Provides a searchable interface for discovering ROS2 packages from rosdistro.
"""

import logging
from typing import Optional, List
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QComboBox, QListWidget, QListWidgetItem,
    QProgressBar, QMessageBox, QFrame
)
from PySide6.QtCore import Qt, Signal, QThread
from PySide6.QtGui import QFont, QIcon

from roboshire.backend.package_repository import (
    PackageRepository,
    PackageInfo,
    PackageCategory
)


class PackageFetchThread(QThread):
    """Background thread for fetching packages from rosdistro"""

    finished = Signal(list)  # Emits list of packages
    error = Signal(str)  # Emits error message
    progress = Signal(str)  # Emits progress message

    def __init__(self, repository: PackageRepository, force_refresh: bool = False):
        super().__init__()
        self.repository = repository
        self.force_refresh = force_refresh

    def run(self):
        """Fetch packages in background"""
        try:
            self.progress.emit("Fetching packages from rosdistro...")
            packages = self.repository.get_packages(force_refresh=self.force_refresh)
            self.finished.emit(packages)
        except Exception as e:
            self.error.emit(str(e))


class PackageListItem(QFrame):
    """Custom widget for displaying a package in the list"""

    details_clicked = Signal(str)  # Emits package name
    install_clicked = Signal(str)  # Emits package name

    def __init__(self, package: PackageInfo, parent=None):
        super().__init__(parent)
        self.package = package
        self._init_ui()

    def _init_ui(self):
        """Initialize the UI"""
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setLineWidth(1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(5)

        # Top row: Name and popularity
        top_layout = QHBoxLayout()

        # Package name (bold)
        name_label = QLabel(self.package.name)
        name_font = QFont()
        name_font.setBold(True)
        name_font.setPointSize(10)
        name_label.setFont(name_font)
        top_layout.addWidget(name_label)

        # Spacer
        top_layout.addStretch()

        # Popularity (stars)
        stars = "★" * self.package.popularity + "☆" * (5 - self.package.popularity)
        stars_label = QLabel(stars)
        stars_label.setStyleSheet("color: #FFD700;")  # Gold color
        top_layout.addWidget(stars_label)

        layout.addLayout(top_layout)

        # Description
        if self.package.description:
            desc_label = QLabel(self.package.description)
            desc_label.setWordWrap(True)
            desc_label.setStyleSheet("color: #666;")
            layout.addWidget(desc_label)

        # Bottom row: Version, License, and buttons
        bottom_layout = QHBoxLayout()

        # Version
        if self.package.version:
            version_label = QLabel(f"v{self.package.version}")
            version_label.setStyleSheet("color: #888; font-size: 9pt;")
            bottom_layout.addWidget(version_label)

        # Category
        category_label = QLabel(f"[{self.package.category.value}]")
        category_label.setStyleSheet("color: #0066CC; font-size: 9pt;")
        bottom_layout.addWidget(category_label)

        # Spacer
        bottom_layout.addStretch()

        # Details button
        details_btn = QPushButton("Details")
        details_btn.setMaximumWidth(80)
        details_btn.clicked.connect(lambda: self.details_clicked.emit(self.package.name))
        bottom_layout.addWidget(details_btn)

        # Install button
        install_btn = QPushButton("Install")
        install_btn.setMaximumWidth(80)
        install_btn.clicked.connect(lambda: self.install_clicked.emit(self.package.name))
        install_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
        bottom_layout.addWidget(install_btn)

        layout.addLayout(bottom_layout)

        # Hover effect
        self.setStyleSheet("""
            PackageListItem {
                background-color: #f9f9f9;
                border: 1px solid #ddd;
                border-radius: 4px;
            }
            PackageListItem:hover {
                background-color: #e8f4f8;
                border: 1px solid #0066CC;
            }
        """)


class PackageBrowserWidget(QWidget):
    """
    Widget for browsing and searching ROS2 packages

    Features:
    - Search by name/description
    - Filter by category
    - Display package list with details
    - Refresh from internet
    - View package details
    - Copy install commands
    """

    package_selected = Signal(str)  # Emits package name when selected

    def __init__(self, ros_distro: str = "humble", parent=None):
        super().__init__(parent)

        self.ros_distro = ros_distro
        self.logger = logging.getLogger(__name__)

        # Create repository
        self.repository = PackageRepository(ros_distro)

        # State
        self.all_packages: List[PackageInfo] = []
        self.filtered_packages: List[PackageInfo] = []
        self.fetch_thread: Optional[PackageFetchThread] = None

        self._init_ui()
        self._load_packages()

    def _init_ui(self):
        """Initialize the user interface"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # Title and refresh button
        title_layout = QHBoxLayout()

        title_label = QLabel("Package Browser")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_layout.addWidget(title_label)

        title_layout.addStretch()

        # Help button
        help_btn = QPushButton("?")
        help_btn.setMaximumWidth(30)
        help_btn.setToolTip("Help: How to use Package Browser")
        help_btn.clicked.connect(self._show_help)
        title_layout.addWidget(help_btn)

        # Refresh button
        self.refresh_btn = QPushButton("⟳ Refresh")
        self.refresh_btn.setToolTip("Refresh package list from rosdistro")
        self.refresh_btn.clicked.connect(self._refresh_packages)
        title_layout.addWidget(self.refresh_btn)

        layout.addLayout(title_layout)

        # Search bar
        search_layout = QHBoxLayout()

        search_label = QLabel("Search:")
        search_layout.addWidget(search_label)

        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Enter package name or description...")
        self.search_input.textChanged.connect(self._on_search_changed)
        self.search_input.returnPressed.connect(self._apply_filters)
        search_layout.addWidget(self.search_input)

        search_btn = QPushButton("🔍 Search")
        search_btn.clicked.connect(self._apply_filters)
        search_layout.addWidget(search_btn)

        layout.addLayout(search_layout)

        # Category filter
        filter_layout = QHBoxLayout()

        filter_label = QLabel("Category:")
        filter_layout.addWidget(filter_label)

        self.category_combo = QComboBox()
        self.category_combo.addItem("All", PackageCategory.ALL)
        for category in PackageCategory:
            if category != PackageCategory.ALL:
                self.category_combo.addItem(category.value, category)
        self.category_combo.currentIndexChanged.connect(self._apply_filters)
        filter_layout.addWidget(self.category_combo)

        filter_layout.addStretch()

        layout.addLayout(filter_layout)

        # Progress bar (hidden by default)
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setRange(0, 0)  # Indeterminate
        self.progress_bar.hide()
        layout.addWidget(self.progress_bar)

        # Package list
        self.package_list = QListWidget()
        self.package_list.setSpacing(5)
        self.package_list.setVerticalScrollMode(QListWidget.ScrollPerPixel)
        layout.addWidget(self.package_list)

        # Status label
        self.status_label = QLabel("Loading packages...")
        self.status_label.setStyleSheet("color: #666; font-style: italic;")
        layout.addWidget(self.status_label)

    def _load_packages(self):
        """Load packages (from cache or internet)"""
        self.logger.info("Loading packages...")
        self.status_label.setText("Loading packages from cache...")

        # Start fetch in background thread
        self.fetch_thread = PackageFetchThread(self.repository, force_refresh=False)
        self.fetch_thread.finished.connect(self._on_packages_loaded)
        self.fetch_thread.error.connect(self._on_fetch_error)
        self.fetch_thread.progress.connect(self._on_fetch_progress)
        self.fetch_thread.start()

        # Show progress bar
        self.progress_bar.show()
        self.refresh_btn.setEnabled(False)

    def _refresh_packages(self):
        """Force refresh packages from internet"""
        reply = QMessageBox.question(
            self,
            "Refresh Packages",
            "This will fetch the latest package list from rosdistro.\n\n"
            "This may take a moment. Continue?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.logger.info("Refreshing packages from rosdistro...")
            self.status_label.setText("Fetching latest package list...")

            # Clear current list
            self.package_list.clear()

            # Start fetch in background thread
            self.fetch_thread = PackageFetchThread(self.repository, force_refresh=True)
            self.fetch_thread.finished.connect(self._on_packages_loaded)
            self.fetch_thread.error.connect(self._on_fetch_error)
            self.fetch_thread.progress.connect(self._on_fetch_progress)
            self.fetch_thread.start()

            # Show progress bar
            self.progress_bar.show()
            self.refresh_btn.setEnabled(False)

    def _on_packages_loaded(self, packages: List[PackageInfo]):
        """Handle packages loaded successfully"""
        self.all_packages = packages
        self.filtered_packages = packages

        self.logger.info(f"Loaded {len(packages)} packages")

        # Hide progress bar
        self.progress_bar.hide()
        self.refresh_btn.setEnabled(True)

        # Update display
        self._update_package_list()

        # Show cache info
        cache_info = self.repository.get_cache_info()
        if cache_info['exists']:
            age_str = f"{cache_info['age_days']} days ago" if cache_info['age_days'] > 0 else "today"
            self.status_label.setText(
                f"Showing {len(self.filtered_packages)} of {len(self.all_packages)} packages "
                f"(cached {age_str})"
            )
        else:
            self.status_label.setText(f"Showing {len(self.filtered_packages)} of {len(self.all_packages)} packages")

    def _on_fetch_error(self, error: str):
        """Handle fetch error"""
        self.logger.error(f"Failed to fetch packages: {error}")

        # Hide progress bar
        self.progress_bar.hide()
        self.refresh_btn.setEnabled(True)

        # Show error message
        self.status_label.setText(f"Error: {error}")

        QMessageBox.warning(
            self,
            "Fetch Error",
            f"Failed to fetch packages:\n\n{error}\n\n"
            "Using cached data if available."
        )

    def _on_fetch_progress(self, message: str):
        """Handle fetch progress update"""
        self.status_label.setText(message)

    def _on_search_changed(self, text: str):
        """Handle search text changed (real-time filtering)"""
        # Only apply if text is empty or user pressed enter
        if not text:
            self._apply_filters()

    def _apply_filters(self):
        """Apply search and category filters"""
        query = self.search_input.text().strip()
        category_data = self.category_combo.currentData()

        # Search packages
        self.filtered_packages = self.repository.search_packages(query, category_data)

        # Update display
        self._update_package_list()

        # Update status
        if query or category_data != PackageCategory.ALL:
            self.status_label.setText(
                f"Showing {len(self.filtered_packages)} of {len(self.all_packages)} packages "
                f"(filtered)"
            )
        else:
            self.status_label.setText(f"Showing {len(self.filtered_packages)} packages")

    def _update_package_list(self):
        """Update the package list display"""
        self.package_list.clear()

        if not self.filtered_packages:
            # Show "no packages" message
            item = QListWidgetItem("No packages found")
            item.setFlags(Qt.NoItemFlags)
            self.package_list.addItem(item)
            return

        # Add packages to list
        for package in self.filtered_packages[:100]:  # Limit to first 100 for performance
            # Create custom widget
            widget = PackageListItem(package)
            widget.details_clicked.connect(self._on_details_clicked)
            widget.install_clicked.connect(self._on_install_clicked)

            # Add to list
            item = QListWidgetItem(self.package_list)
            item.setSizeHint(widget.sizeHint())
            self.package_list.addItem(item)
            self.package_list.setItemWidget(item, widget)

        # Show truncation message if needed
        if len(self.filtered_packages) > 100:
            info_item = QListWidgetItem(
                f"Showing first 100 of {len(self.filtered_packages)} packages. "
                "Use search to narrow results."
            )
            info_item.setFlags(Qt.NoItemFlags)
            info_item.setForeground(Qt.gray)
            self.package_list.addItem(info_item)

    def _on_details_clicked(self, package_name: str):
        """Handle details button clicked"""
        self.logger.info(f"Details clicked for: {package_name}")

        package = self.repository.get_package_details(package_name)
        if package:
            self._show_package_details(package)

    def _on_install_clicked(self, package_name: str):
        """Handle install button clicked"""
        self.logger.info(f"Install clicked for: {package_name}")

        # Generate install command
        install_cmd = f"sudo apt install ros-{self.ros_distro}-{package_name.replace('_', '-')}"

        # Copy to clipboard
        from PySide6.QtGui import QGuiApplication
        clipboard = QGuiApplication.clipboard()
        clipboard.setText(install_cmd)

        # Show message
        QMessageBox.information(
            self,
            "Install Command Copied",
            f"The following command has been copied to your clipboard:\n\n"
            f"{install_cmd}\n\n"
            "Run this command on your Ubuntu VM to install the package."
        )

    def _show_package_details(self, package: PackageInfo):
        """Show detailed package information dialog"""
        # Import here to avoid circular dependency
        from roboshire.gui.package_detail_dialog import PackageDetailDialog

        dialog = PackageDetailDialog(package, self.ros_distro, self)
        dialog.exec()

    def _show_help(self):
        """Show help dialog"""
        help_text = (
            "<h3>Package Browser Help</h3>"
            "<p>The Package Browser helps you discover and install ROS2 packages.</p>"

            "<h4>Search</h4>"
            "<ul>"
            "<li>Type in the search box to find packages by name or description</li>"
            "<li>Press Enter or click 'Search' to apply the filter</li>"
            "</ul>"

            "<h4>Categories</h4>"
            "<ul>"
            "<li>Use the dropdown to filter packages by category</li>"
            "<li>Categories: Navigation, Vision, Control, Sensors, etc.</li>"
            "</ul>"

            "<h4>Actions</h4>"
            "<ul>"
            "<li><b>Details:</b> View full package information</li>"
            "<li><b>Install:</b> Copy install command to clipboard</li>"
            "<li><b>Refresh:</b> Fetch latest package list from rosdistro</li>"
            "</ul>"

            "<h4>Installing Packages</h4>"
            "<p>Click 'Install' to copy the install command. Then run it on your Ubuntu VM:</p>"
            "<pre>sudo apt install ros-humble-&lt;package-name&gt;</pre>"

            "<h4>Cache</h4>"
            "<p>Package lists are cached for 7 days. Click 'Refresh' to update.</p>"
        )

        QMessageBox.information(
            self,
            "Package Browser Help",
            help_text
        )


# Example usage
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    logging.basicConfig(level=logging.INFO)

    app = QApplication(sys.argv)

    widget = PackageBrowserWidget("humble")
    widget.setWindowTitle("ROS2 Package Browser")
    widget.resize(800, 600)
    widget.show()

    sys.exit(app.exec())
