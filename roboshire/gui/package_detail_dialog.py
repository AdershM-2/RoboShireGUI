"""
Package Detail Dialog for RoboShire

Shows detailed information about a ROS2 package.
"""

import logging
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QTextEdit, QGroupBox, QMessageBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QGuiApplication

from roboshire.backend.package_repository import PackageInfo


class PackageDetailDialog(QDialog):
    """
    Dialog showing detailed package information

    Features:
    - Package name, version, license
    - Full description
    - Repository URL
    - Dependencies list
    - Install command
    - Copy to clipboard
    - View documentation link
    """

    def __init__(self, package: PackageInfo, ros_distro: str = "humble", parent=None):
        super().__init__(parent)

        self.package = package
        self.ros_distro = ros_distro
        self.logger = logging.getLogger(__name__)

        self._init_ui()

    def _init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle(f"Package Details - {self.package.name}")
        self.setMinimumWidth(600)
        self.setMinimumHeight(500)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(15)

        # Package name (title)
        name_label = QLabel(self.package.name)
        name_font = QFont()
        name_font.setPointSize(16)
        name_font.setBold(True)
        name_label.setFont(name_font)
        layout.addWidget(name_label)

        # Category and popularity
        meta_layout = QHBoxLayout()

        category_label = QLabel(f"Category: {self.package.category.value}")
        category_label.setStyleSheet("color: #0066CC; font-weight: bold;")
        meta_layout.addWidget(category_label)

        meta_layout.addStretch()

        stars = "★" * self.package.popularity + "☆" * (5 - self.package.popularity)
        popularity_label = QLabel(f"Popularity: {stars}")
        popularity_label.setStyleSheet("color: #FFD700;")
        meta_layout.addWidget(popularity_label)

        layout.addLayout(meta_layout)

        # Description
        desc_group = QGroupBox("Description")
        desc_layout = QVBoxLayout(desc_group)

        if self.package.description:
            desc_text = QLabel(self.package.description)
            desc_text.setWordWrap(True)
        else:
            desc_text = QLabel("<i>No description available</i>")
            desc_text.setStyleSheet("color: #888;")

        desc_layout.addWidget(desc_text)
        layout.addWidget(desc_group)

        # Details group
        details_group = QGroupBox("Details")
        details_layout = QVBoxLayout(details_group)

        # Version
        if self.package.version:
            version_label = QLabel(f"<b>Version:</b> {self.package.version}")
            details_layout.addWidget(version_label)

        # License
        if self.package.license:
            license_label = QLabel(f"<b>License:</b> {self.package.license}")
            details_layout.addWidget(license_label)
        else:
            license_label = QLabel("<b>License:</b> <i>Not specified</i>")
            license_label.setStyleSheet("color: #888;")
            details_layout.addWidget(license_label)

        # Maintainer
        if self.package.maintainer:
            maintainer_label = QLabel(f"<b>Maintainer:</b> {self.package.maintainer}")
            details_layout.addWidget(maintainer_label)

        # Repository URL
        if self.package.repository_url:
            repo_layout = QHBoxLayout()
            repo_layout.addWidget(QLabel("<b>Repository:</b>"))

            repo_link = QLabel(f'<a href="{self.package.repository_url}">{self.package.repository_url}</a>')
            repo_link.setOpenExternalLinks(True)
            repo_link.setTextInteractionFlags(Qt.TextBrowserInteraction)
            repo_layout.addWidget(repo_link)
            repo_layout.addStretch()

            details_layout.addLayout(repo_layout)

        layout.addWidget(details_group)

        # Dependencies (if any)
        if self.package.dependencies:
            deps_group = QGroupBox("Dependencies")
            deps_layout = QVBoxLayout(deps_group)

            deps_text = QTextEdit()
            deps_text.setReadOnly(True)
            deps_text.setMaximumHeight(100)
            deps_text.setPlainText("\n".join(f"- {dep}" for dep in self.package.dependencies))
            deps_layout.addWidget(deps_text)

            layout.addWidget(deps_group)

        # Install command group
        install_group = QGroupBox("Installation")
        install_layout = QVBoxLayout(install_group)

        # Generate install command
        install_cmd = f"sudo apt install ros-{self.ros_distro}-{self.package.name.replace('_', '-')}"

        install_label = QLabel("Run this command on your Ubuntu VM:")
        install_layout.addWidget(install_label)

        install_text = QTextEdit()
        install_text.setReadOnly(True)
        install_text.setMaximumHeight(60)
        install_text.setPlainText(install_cmd)
        install_text.setStyleSheet(
            "background-color: #f5f5f5; "
            "border: 1px solid #ccc; "
            "font-family: monospace; "
            "padding: 5px;"
        )
        install_layout.addWidget(install_text)

        # Copy button
        copy_btn = QPushButton("📋 Copy Install Command")
        copy_btn.clicked.connect(lambda: self._copy_install_command(install_cmd))
        install_layout.addWidget(copy_btn)

        layout.addWidget(install_group)

        # Buttons at bottom
        button_layout = QHBoxLayout()

        # View docs button
        if self.package.repository_url:
            docs_btn = QPushButton("View Documentation")
            docs_btn.clicked.connect(self._open_documentation)
            button_layout.addWidget(docs_btn)

        button_layout.addStretch()

        # Close button
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        close_btn.setDefault(True)
        button_layout.addWidget(close_btn)

        layout.addLayout(button_layout)

    def _copy_install_command(self, command: str):
        """Copy install command to clipboard"""
        clipboard = QGuiApplication.clipboard()
        clipboard.setText(command)

        self.logger.info(f"Copied install command: {command}")

        # Show brief confirmation
        QMessageBox.information(
            self,
            "Copied",
            "Install command copied to clipboard!"
        )

    def _open_documentation(self):
        """Open package documentation in browser"""
        if self.package.repository_url:
            from PySide6.QtGui import QDesktopServices
            from PySide6.QtCore import QUrl

            url = QUrl(self.package.repository_url)
            QDesktopServices.openUrl(url)

            self.logger.info(f"Opening documentation: {self.package.repository_url}")


# Example usage
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication
    from roboshire.backend.package_repository import PackageCategory

    logging.basicConfig(level=logging.INFO)

    app = QApplication(sys.argv)

    # Create sample package
    sample_package = PackageInfo(
        name="nav2_bringup",
        description="Navigation 2 stack bringup package. Provides launch files and configuration for bringing up the Nav2 navigation stack on a robot.",
        version="1.1.9",
        license="Apache 2.0",
        maintainer="Steve Macenski",
        repository_url="https://github.com/ros-planning/navigation2",
        dependencies=["nav2_common", "nav2_msgs", "tf2_ros", "rclpy"],
        category=PackageCategory.NAVIGATION,
        popularity=5
    )

    dialog = PackageDetailDialog(sample_package, "humble")
    dialog.exec()

    sys.exit(app.exec())
