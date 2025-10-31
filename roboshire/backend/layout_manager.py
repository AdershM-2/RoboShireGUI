"""
Layout Manager

Saves and restores window layout using QSettings.
Addresses "Layout Doesn't Save" issue from UX evaluation (Priority 2).

Features:
- Save/restore tab positions
- Save/restore widget sizes
- Save/restore splitter positions
- Per-project layouts (optional)
- Reset to default layout
"""

from PySide6.QtCore import QSettings, QByteArray
from PySide6.QtWidgets import QMainWindow, QTabWidget, QSplitter
from typing import Optional, Dict, Any
import logging


class LayoutManager:
    """
    Manages window layout persistence using QSettings

    Saves and restores:
    - Window geometry (size, position)
    - Tab widget current tab
    - Splitter positions
    - Dock widget states
    - Toolbar visibility
    """

    def __init__(self, organization: str = "RoboShire", application: str = "RoboShire"):
        """
        Initialize LayoutManager

        Args:
            organization: Organization name for QSettings
            application: Application name for QSettings
        """
        self.settings = QSettings(organization, application)
        self.logger = logging.getLogger(__name__)

    def save_layout(self, main_window: QMainWindow, layout_name: str = "default") -> bool:
        """
        Save complete window layout

        Args:
            main_window: QMainWindow to save layout from
            layout_name: Name for this layout (default, custom, project-specific, etc.)

        Returns:
            True if save succeeded
        """
        try:
            prefix = f"layouts/{layout_name}"

            # Save window geometry
            self.settings.setValue(f"{prefix}/geometry", main_window.saveGeometry())
            self.settings.setValue(f"{prefix}/windowState", main_window.saveState())

            # Save tab widget state
            self._save_tab_widgets(main_window, prefix)

            # Save splitter positions
            self._save_splitters(main_window, prefix)

            # Sync to disk
            self.settings.sync()

            self.logger.info(f"Layout '{layout_name}' saved successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save layout: {e}")
            return False

    def restore_layout(self, main_window: QMainWindow, layout_name: str = "default") -> bool:
        """
        Restore window layout

        Args:
            main_window: QMainWindow to restore layout to
            layout_name: Name of layout to restore

        Returns:
            True if restore succeeded
        """
        try:
            prefix = f"layouts/{layout_name}"

            # Check if layout exists
            if not self.settings.contains(f"{prefix}/geometry"):
                self.logger.warning(f"Layout '{layout_name}' not found")
                return False

            # Restore window geometry
            geometry = self.settings.value(f"{prefix}/geometry")
            if geometry:
                main_window.restoreGeometry(geometry)

            window_state = self.settings.value(f"{prefix}/windowState")
            if window_state:
                main_window.restoreState(window_state)

            # Restore tab widget state
            self._restore_tab_widgets(main_window, prefix)

            # Restore splitter positions
            self._restore_splitters(main_window, prefix)

            self.logger.info(f"Layout '{layout_name}' restored successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to restore layout: {e}")
            return False

    def _save_tab_widgets(self, parent, prefix: str):
        """Recursively save all QTabWidget states"""
        for tab_widget in parent.findChildren(QTabWidget):
            # Get unique identifier for this tab widget
            object_name = tab_widget.objectName()
            if not object_name:
                continue  # Skip widgets without names

            # Save current tab index
            current_index = tab_widget.currentIndex()
            self.settings.setValue(f"{prefix}/tabs/{object_name}/currentIndex", current_index)

            # Save tab order (tab names in current order)
            tab_names = []
            for i in range(tab_widget.count()):
                tab_names.append(tab_widget.tabText(i))
            self.settings.setValue(f"{prefix}/tabs/{object_name}/tabOrder", tab_names)

    def _restore_tab_widgets(self, parent, prefix: str):
        """Recursively restore all QTabWidget states"""
        for tab_widget in parent.findChildren(QTabWidget):
            object_name = tab_widget.objectName()
            if not object_name:
                continue

            # Restore current tab index
            current_index = self.settings.value(f"{prefix}/tabs/{object_name}/currentIndex", type=int)
            if current_index is not None and 0 <= current_index < tab_widget.count():
                tab_widget.setCurrentIndex(current_index)

            # Note: Tab order restoration would require more complex logic
            # (moving tabs, which may not be supported by all tab widgets)
            # For now, we just restore the active tab

    def _save_splitters(self, parent, prefix: str):
        """Recursively save all QSplitter states"""
        for splitter in parent.findChildren(QSplitter):
            object_name = splitter.objectName()
            if not object_name:
                continue

            # Save splitter sizes
            sizes = splitter.sizes()
            self.settings.setValue(f"{prefix}/splitters/{object_name}/sizes", sizes)

            # Save splitter state (includes handle positions)
            self.settings.setValue(f"{prefix}/splitters/{object_name}/state", splitter.saveState())

    def _restore_splitters(self, parent, prefix: str):
        """Recursively restore all QSplitter states"""
        for splitter in parent.findChildren(QSplitter):
            object_name = splitter.objectName()
            if not object_name:
                continue

            # Restore splitter state
            state = self.settings.value(f"{prefix}/splitters/{object_name}/state")
            if state:
                splitter.restoreState(state)

            # Restore splitter sizes (fallback if state doesn't work)
            sizes = self.settings.value(f"{prefix}/splitters/{object_name}/sizes", type=list)
            if sizes and len(sizes) == splitter.count():
                # Convert to integers
                sizes = [int(s) for s in sizes]
                splitter.setSizes(sizes)

    def reset_layout(self, layout_name: str = "default"):
        """
        Delete saved layout (forces reset to default)

        Args:
            layout_name: Name of layout to delete
        """
        try:
            prefix = f"layouts/{layout_name}"
            self.settings.remove(prefix)
            self.settings.sync()
            self.logger.info(f"Layout '{layout_name}' reset")

        except Exception as e:
            self.logger.error(f"Failed to reset layout: {e}")

    def list_layouts(self) -> list:
        """
        List all saved layouts

        Returns:
            List of layout names
        """
        try:
            self.settings.beginGroup("layouts")
            layouts = self.settings.childGroups()
            self.settings.endGroup()
            return layouts

        except Exception as e:
            self.logger.error(f"Failed to list layouts: {e}")
            return []

    def save_preference(self, key: str, value: Any):
        """
        Save a user preference

        Args:
            key: Preference key
            value: Preference value
        """
        self.settings.setValue(f"preferences/{key}", value)
        self.settings.sync()

    def get_preference(self, key: str, default: Any = None) -> Any:
        """
        Get a user preference

        Args:
            key: Preference key
            default: Default value if key not found

        Returns:
            Preference value or default
        """
        return self.settings.value(f"preferences/{key}", default)

    def save_recent_projects(self, project_paths: list, max_recent: int = 10):
        """
        Save list of recent projects

        Args:
            project_paths: List of project paths
            max_recent: Maximum number of recent projects to keep
        """
        # Keep only the most recent
        recent = project_paths[:max_recent]
        self.settings.setValue("recentProjects", recent)
        self.settings.sync()

    def get_recent_projects(self, max_recent: int = 10) -> list:
        """
        Get list of recent projects

        Args:
            max_recent: Maximum number to return

        Returns:
            List of recent project paths
        """
        recent = self.settings.value("recentProjects", [])
        if not isinstance(recent, list):
            recent = []
        return recent[:max_recent]

    def clear_all_settings(self):
        """Clear all saved settings (use with caution!)"""
        self.settings.clear()
        self.settings.sync()
        self.logger.warning("All settings cleared")


# Convenience functions for common operations

def save_window_layout(main_window: QMainWindow, layout_name: str = "default") -> bool:
    """
    Convenience function to save window layout

    Args:
        main_window: QMainWindow to save
        layout_name: Name for this layout

    Returns:
        True if successful
    """
    manager = LayoutManager()
    return manager.save_layout(main_window, layout_name)


def restore_window_layout(main_window: QMainWindow, layout_name: str = "default") -> bool:
    """
    Convenience function to restore window layout

    Args:
        main_window: QMainWindow to restore
        layout_name: Name of layout to restore

    Returns:
        True if successful
    """
    manager = LayoutManager()
    return manager.restore_layout(main_window, layout_name)


def reset_window_layout(layout_name: str = "default"):
    """
    Convenience function to reset layout to default

    Args:
        layout_name: Name of layout to reset
    """
    manager = LayoutManager()
    manager.reset_layout(layout_name)


# Example usage
if __name__ == '__main__':
    import sys
    from PySide6.QtWidgets import QApplication, QMainWindow, QTabWidget, QSplitter, QLabel

    app = QApplication(sys.argv)

    # Create test window
    window = QMainWindow()
    window.setWindowTitle("Layout Manager Test")

    # Create tab widget
    tabs = QTabWidget()
    tabs.setObjectName("mainTabs")  # Important: Set object name for saving
    tabs.addTab(QLabel("Tab 1"), "Tab 1")
    tabs.addTab(QLabel("Tab 2"), "Tab 2")
    tabs.addTab(QLabel("Tab 3"), "Tab 3")

    # Create splitter
    splitter = QSplitter()
    splitter.setObjectName("mainSplitter")  # Important: Set object name
    splitter.addWidget(QLabel("Left"))
    splitter.addWidget(tabs)
    splitter.addWidget(QLabel("Right"))

    window.setCentralWidget(splitter)
    window.resize(800, 600)

    # Test layout saving
    manager = LayoutManager()

    # Try to restore previous layout
    if not manager.restore_layout(window):
        print("No saved layout found, using defaults")

    window.show()

    # Save layout on close
    def on_close():
        manager.save_layout(window)
        print("Layout saved")

    window.closeEvent = lambda event: (on_close(), event.accept())

    sys.exit(app.exec())
