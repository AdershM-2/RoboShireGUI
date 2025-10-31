"""
Icon Manager

Centralized icon management system using Unicode emoji and symbols.
Provides consistent iconography across the application.

Note: Using Unicode emoji/symbols instead of icon fonts for:
- No external dependencies
- Cross-platform compatibility
- Easy to implement
- Professional appearance

Future: Can be extended to support icon fonts (Material Design, Font Awesome, etc.)
"""

from PySide6.QtGui import QIcon, QPixmap, QPainter, QColor, QFont
from PySide6.QtCore import Qt, QSize
from typing import Optional
import logging


class IconManager:
    """
    Manages icons for the application using Unicode symbols

    Provides consistent icon set with semantic naming.
    Icons are rendered as text using Unicode characters.
    """

    # File operations
    FILE_NEW = "📄"
    FILE_OPEN = "📂"
    FILE_SAVE = "💾"
    FILE_CLOSE = "✖"
    FILE_IMPORT = "📥"
    FILE_EXPORT = "📤"

    # Project/Build operations
    PROJECT_NEW = "🆕"
    PROJECT_BUILD = "🔨"
    PROJECT_RUN = "▶"
    PROJECT_STOP = "⏹"
    PROJECT_CLEAN = "🧹"
    GENERATE_CODE = "⚙"

    # Robot/Hardware
    ROBOT = "🤖"
    ROBOT_ARM = "🦾"
    SENSOR = "📡"
    HARDWARE = "🔧"
    MICROCONTROLLER = "💻"
    CONNECTION = "🔌"

    # Simulation/Visualization
    SIMULATION = "🎮"
    VIEWER_3D = "🎨"
    CAMERA = "📹"
    GAZEBO = "🌐"
    RVIZ = "👁"

    # Navigation
    NAVIGATION = "🧭"
    MAP = "🗺"
    WAYPOINT = "📍"
    PATH = "🛤"

    # Monitoring/Status
    STATUS_OK = "✅"
    STATUS_ERROR = "❌"
    STATUS_WARNING = "⚠"
    STATUS_INFO = "ℹ"
    MONITOR = "📊"
    LOG = "📋"
    GRAPH = "📈"

    # Node/Graph operations
    NODE = "⬢"
    NODE_ADD = "➕"
    NODE_DELETE = "➖"
    CONNECTION_LINE = "🔗"
    LAYOUT = "⚡"

    # Tools
    TOOLS = "🛠"
    SETTINGS = "⚙"
    SEARCH = "🔍"
    FILTER = "🔬"
    DEBUG = "🐛"
    TEST = "🧪"

    # Communication
    TOPIC = "📢"
    SERVICE = "🔔"
    ACTION = "⚡"
    PARAMETER = "🎚"

    # Help/Documentation
    HELP = "❓"
    DOCUMENTATION = "📖"
    TUTORIAL = "🎓"
    KEYBOARD = "⌨"
    ABOUT = "ℹ"

    # Window/Layout
    WINDOW = "🪟"
    TAB = "📑"
    SPLIT = "⬌"
    RESIZE = "↔"
    RESET = "🔄"

    # Actions
    PLAY = "▶"
    PAUSE = "⏸"
    STOP = "⏹"
    REFRESH = "🔄"
    UNDO = "↶"
    REDO = "↷"
    COPY = "📋"
    PASTE = "📄"
    CUT = "✂"
    DELETE = "🗑"

    # Status indicators
    INDICATOR_RUNNING = "🟢"
    INDICATOR_STOPPED = "🔴"
    INDICATOR_PAUSED = "🟡"
    INDICATOR_UNKNOWN = "⚫"

    # Special
    WIZARD = "🪄"
    EXAMPLE = "📚"
    PACKAGE = "📦"
    FOLDER = "📁"
    CODE = "💻"
    SSH = "🔐"
    NETWORK = "🌐"

    def __init__(self):
        """Initialize IconManager"""
        self.logger = logging.getLogger(__name__)
        self.icon_cache = {}

    def get_icon(self, icon_char: str, size: int = 16, color: Optional[QColor] = None) -> QIcon:
        """
        Get QIcon from Unicode character

        Args:
            icon_char: Unicode character (e.g., "📄", "🔨")
            size: Icon size in pixels
            color: Optional color (default: black)

        Returns:
            QIcon with rendered character
        """
        # Create cache key
        cache_key = f"{icon_char}_{size}_{color.name() if color else 'default'}"

        # Return cached icon if available
        if cache_key in self.icon_cache:
            return self.icon_cache[cache_key]

        # Create pixmap
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.transparent)

        # Draw character
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Set font
        font = QFont()
        font.setPixelSize(int(size * 0.8))  # 80% of size for padding
        painter.setFont(font)

        # Set color
        if color:
            painter.setPen(color)

        # Draw centered text
        painter.drawText(pixmap.rect(), Qt.AlignCenter, icon_char)
        painter.end()

        # Create icon
        icon = QIcon(pixmap)

        # Cache it
        self.icon_cache[cache_key] = icon

        return icon

    def get_menu_icon(self, icon_char: str) -> QIcon:
        """Get icon sized for menu items (16px)"""
        return self.get_icon(icon_char, size=16)

    def get_toolbar_icon(self, icon_char: str) -> QIcon:
        """Get icon sized for toolbar buttons (24px)"""
        return self.get_icon(icon_char, size=24)

    def get_button_icon(self, icon_char: str) -> QIcon:
        """Get icon sized for buttons (20px)"""
        return self.get_icon(icon_char, size=20)

    def get_tab_icon(self, icon_char: str) -> QIcon:
        """Get icon sized for tabs (16px)"""
        return self.get_icon(icon_char, size=16)

    def clear_cache(self):
        """Clear icon cache"""
        self.icon_cache.clear()


# Singleton instance
_icon_manager = None


def get_icon_manager() -> IconManager:
    """Get singleton IconManager instance"""
    global _icon_manager
    if _icon_manager is None:
        _icon_manager = IconManager()
    return _icon_manager


# Convenience functions
def get_icon(icon_char: str, size: int = 16, color: Optional[QColor] = None) -> QIcon:
    """Convenience function to get icon"""
    return get_icon_manager().get_icon(icon_char, size, color)


def get_menu_icon(icon_char: str) -> QIcon:
    """Convenience function to get menu icon"""
    return get_icon_manager().get_menu_icon(icon_char)


def get_toolbar_icon(icon_char: str) -> QIcon:
    """Convenience function to get toolbar icon"""
    return get_icon_manager().get_toolbar_icon(icon_char)


# Icon mapping for common actions
ACTION_ICONS = {
    # File operations
    "new_project": IconManager.PROJECT_NEW,
    "open_project": IconManager.FILE_OPEN,
    "save_project": IconManager.FILE_SAVE,
    "import_urdf": IconManager.FILE_IMPORT,
    "export_urdf": IconManager.FILE_EXPORT,

    # Build/Run
    "generate_code": IconManager.GENERATE_CODE,
    "build": IconManager.PROJECT_BUILD,
    "run": IconManager.PROJECT_RUN,
    "stop": IconManager.PROJECT_STOP,
    "clean": IconManager.PROJECT_CLEAN,

    # Robot/Simulation
    "robot": IconManager.ROBOT,
    "rviz": IconManager.RVIZ,
    "gazebo": IconManager.GAZEBO,
    "mujoco": IconManager.VIEWER_3D,
    "nav2": IconManager.NAVIGATION,

    # Hardware
    "hardware_test": IconManager.HARDWARE,
    "serial_monitor": IconManager.MICROCONTROLLER,
    "micro_ros": IconManager.CONNECTION,
    "ssh": IconManager.SSH,

    # Tools
    "rqt_graph": IconManager.GRAPH,
    "rqt_console": IconManager.LOG,
    "urdf_validator": IconManager.TEST,

    # Window/Layout
    "reset_layout": IconManager.RESET,
    "save_layout": IconManager.FILE_SAVE,

    # Help
    "shortcuts": IconManager.KEYBOARD,
    "help": IconManager.HELP,
    "about": IconManager.ABOUT,

    # Monitoring
    "node_status": IconManager.MONITOR,
    "topics": IconManager.TOPIC,
    "parameters": IconManager.PARAMETER,
    "logs": IconManager.LOG,

    # Wizard
    "wizard": IconManager.WIZARD,
    "examples": IconManager.EXAMPLE,
    "packages": IconManager.PACKAGE,
}


def get_action_icon(action_name: str, size: int = 16) -> Optional[QIcon]:
    """
    Get icon for common action by name

    Args:
        action_name: Action name (e.g., "new_project", "build", "run")
        size: Icon size in pixels

    Returns:
        QIcon or None if action not found
    """
    icon_char = ACTION_ICONS.get(action_name)
    if icon_char:
        return get_icon(icon_char, size)
    return None


# Status indicator helper
def get_status_icon(status: str) -> QIcon:
    """
    Get status indicator icon

    Args:
        status: "running", "stopped", "paused", "unknown", "ok", "error", "warning"

    Returns:
        QIcon with appropriate status indicator
    """
    status_icons = {
        "running": IconManager.INDICATOR_RUNNING,
        "stopped": IconManager.INDICATOR_STOPPED,
        "paused": IconManager.INDICATOR_PAUSED,
        "unknown": IconManager.INDICATOR_UNKNOWN,
        "ok": IconManager.STATUS_OK,
        "error": IconManager.STATUS_ERROR,
        "warning": IconManager.STATUS_WARNING,
        "info": IconManager.STATUS_INFO,
    }

    icon_char = status_icons.get(status.lower(), IconManager.INDICATOR_UNKNOWN)
    return get_icon(icon_char, size=16)


# Example usage and testing
if __name__ == '__main__':
    import sys
    from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel

    app = QApplication(sys.argv)

    window = QMainWindow()
    window.setWindowTitle("Icon Manager Test")

    central = QWidget()
    layout = QVBoxLayout(central)

    # Test different icon types
    manager = IconManager()

    layout.addWidget(QLabel("<h2>Icon Manager Examples</h2>"))

    # File operations
    layout.addWidget(QLabel("<b>File Operations:</b>"))
    for name in ["new_project", "open_project", "save_project", "import_urdf", "export_urdf"]:
        btn = QPushButton(name.replace("_", " ").title())
        btn.setIcon(get_action_icon(name, size=20))
        layout.addWidget(btn)

    # Build operations
    layout.addWidget(QLabel("<b>Build Operations:</b>"))
    for name in ["generate_code", "build", "run", "stop", "clean"]:
        btn = QPushButton(name.replace("_", " ").title())
        btn.setIcon(get_action_icon(name, size=20))
        layout.addWidget(btn)

    # Status indicators
    layout.addWidget(QLabel("<b>Status Indicators:</b>"))
    for status in ["running", "stopped", "paused", "ok", "error", "warning"]:
        btn = QPushButton(status.title())
        btn.setIcon(get_status_icon(status))
        layout.addWidget(btn)

    window.setCentralWidget(central)
    window.show()

    sys.exit(app.exec())
