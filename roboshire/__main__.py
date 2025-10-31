"""
RoboShire entry point for running as a module

Usage: python -m roboshire
"""

import sys
import logging
from PySide6.QtWidgets import QApplication
from roboshire.gui.main_window import MainWindow

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def main():
    """Main entry point for RoboShire"""
    app = QApplication(sys.argv)

    # Set application metadata
    app.setApplicationName("RoboShire")
    app.setOrganizationName("RoboShire")
    app.setApplicationVersion("2.2.0")  # v2.2.0 with AI features

    # Apply RoboShire brand theme
    from roboshire.gui.brand_theme import apply_brand_theme_to_app
    apply_brand_theme_to_app(app)

    # Create and show main window
    window = MainWindow()
    window.show()

    # Run application
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
