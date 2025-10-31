#!/usr/bin/env python3
"""
RoboShire Main Application Entry Point
"""

import sys
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from roboshire.gui.main_window import MainWindow


def main():
    """Main application entry point"""

    # Enable high DPI scaling
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("RoboShire")
    app.setApplicationVersion("0.1.0")
    app.setOrganizationName("RoboShire")

    # Create and show main window
    window = MainWindow()
    window.show()

    # Run event loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
