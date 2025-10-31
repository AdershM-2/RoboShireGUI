"""
Keyboard Shortcuts Manager for RoboShire

Centralized management of all keyboard shortcuts across the application.
Provides consistent shortcut definitions, tooltip generation, and documentation.

Author: RoboShire Team
Phase: 7 (Polish & UX)
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeySequence, QAction
from typing import Dict, List, Tuple


class ShortcutCategory:
    """Shortcut category constants"""
    FILE = "File"
    EDIT = "Edit"
    BUILD = "Build"
    VIEW = "View"
    ROBOT = "Robot"
    GRAPH = "Node Graph"
    TOOLS = "Tools"
    HELP = "Help"


class KeyboardShortcuts:
    """
    Centralized keyboard shortcut management for RoboShire

    Features:
    - Consistent shortcut definitions
    - Automatic tooltip generation
    - Shortcut documentation
    - Conflict detection
    """

    # ========== FILE OPERATIONS ==========
    NEW_PROJECT = QKeySequence.New  # Ctrl+N
    OPEN_PROJECT = QKeySequence.Open  # Ctrl+O
    SAVE_PROJECT = QKeySequence.Save  # Ctrl+S
    SAVE_PROJECT_AS = QKeySequence.SaveAs  # Ctrl+Shift+S
    IMPORT_URDF = QKeySequence("Ctrl+I")
    EXPORT_URDF = QKeySequence("Ctrl+E")
    QUIT = QKeySequence.Quit  # Ctrl+Q
    CLOSE_TAB = QKeySequence.Close  # Ctrl+W

    # ========== EDIT OPERATIONS ==========
    UNDO = QKeySequence.Undo  # Ctrl+Z
    REDO = QKeySequence.Redo  # Ctrl+Y or Ctrl+Shift+Z
    CUT = QKeySequence.Cut  # Ctrl+X
    COPY = QKeySequence.Copy  # Ctrl+C
    PASTE = QKeySequence.Paste  # Ctrl+V
    DELETE = QKeySequence.Delete  # Delete key
    SELECT_ALL = QKeySequence.SelectAll  # Ctrl+A
    FIND = QKeySequence.Find  # Ctrl+F

    # ========== BUILD & RUN OPERATIONS ==========
    GENERATE_CODE = QKeySequence("Ctrl+G")
    BUILD = QKeySequence("Ctrl+B")
    CLEAN_BUILD = QKeySequence("Ctrl+Shift+B")
    RUN = QKeySequence("Ctrl+R")
    STOP = QKeySequence("Ctrl+Shift+R")
    QUICK_RUN = QKeySequence("F5")
    STOP_ALL = QKeySequence("Shift+F5")

    # ========== VIEW OPERATIONS ==========
    TAB_NODE_GRAPH = QKeySequence("Ctrl+1")
    TAB_BUILD = QKeySequence("Ctrl+2")
    TAB_LOGS = QKeySequence("Ctrl+3")
    TAB_NODE_STATUS = QKeySequence("Ctrl+4")
    TAB_TOPICS = QKeySequence("Ctrl+5")
    TAB_PARAMETERS = QKeySequence("Ctrl+6")
    TAB_LIFECYCLE = QKeySequence("Ctrl+7")
    TAB_PACKAGES = QKeySequence("Ctrl+9")  # Phase 12: Package Browser
    RESET_VIEW = QKeySequence("Ctrl+0")
    ZOOM_IN = QKeySequence.ZoomIn  # Ctrl++
    ZOOM_OUT = QKeySequence.ZoomOut  # Ctrl+-
    FULLSCREEN = QKeySequence("F11")

    # ========== ROBOT OPERATIONS ==========
    VALIDATE_URDF = QKeySequence("Ctrl+Shift+V")
    VIEW_RVIZ = QKeySequence("Ctrl+Shift+P")
    SIMULATE_GAZEBO = QKeySequence("Ctrl+Shift+G")
    OPEN_EXAMPLES = QKeySequence("Ctrl+Shift+E")

    # ========== NODE GRAPH OPERATIONS ==========
    ADD_NODE = QKeySequence("Ctrl+Shift+N")
    DUPLICATE_NODE = QKeySequence("Ctrl+D")
    DELETE_NODE = QKeySequence.Delete
    AUTO_LAYOUT = QKeySequence("Ctrl+L")

    # ========== TOOLS ==========
    OPEN_RQT_GRAPH = QKeySequence("Ctrl+Shift+T")
    OPEN_RQT_CONSOLE = QKeySequence("Ctrl+Shift+C")
    CLEAR_LOGS = QKeySequence("Ctrl+Shift+L")
    DEVELOPER_CONSOLE = QKeySequence("F12")

    # ========== HARDWARE (v2.4.0) ==========
    MICRO_ROS_AGENT = QKeySequence("Ctrl+Shift+M")
    HARDWARE_TEST_PANEL = QKeySequence("Ctrl+T")
    SERIAL_MONITOR = QKeySequence("Ctrl+Shift+O")  # v2.4.1: Serial Output Monitor

    # ========== HELP ==========
    HELP = QKeySequence.HelpContents  # F1
    SHORTCUT_CHEATSHEET = QKeySequence("Ctrl+Shift+K")
    ABOUT = QKeySequence("Ctrl+Shift+A")

    @staticmethod
    def apply_to_action(action: QAction, shortcut: QKeySequence,
                       description: str = "", show_in_tooltip: bool = True):
        """
        Apply shortcut to action and optionally update tooltip

        Args:
            action: The QAction to configure
            shortcut: The keyboard shortcut
            description: Description for tooltip
            show_in_tooltip: Whether to append shortcut to tooltip
        """
        action.setShortcut(shortcut)

        if show_in_tooltip and description:
            # Handle both QKeySequence and StandardKey
            if isinstance(shortcut, QKeySequence):
                shortcut_text = shortcut.toString()
            else:
                # Convert StandardKey to QKeySequence first
                shortcut_text = QKeySequence(shortcut).toString()

            tooltip = f"{description} ({shortcut_text})"
            action.setToolTip(tooltip)
            action.setStatusTip(description)
        elif description:
            action.setStatusTip(description)

    @staticmethod
    def get_all_shortcuts() -> Dict[str, List[Tuple[str, str, str]]]:
        """
        Get all shortcuts organized by category

        Returns:
            Dictionary mapping category name to list of (name, shortcut, description) tuples
        """
        return {
            ShortcutCategory.FILE: [
                ("New Project", "Ctrl+N", "Create a new RoboShire project"),
                ("Open Project", "Ctrl+O", "Open an existing project"),
                ("Save Project", "Ctrl+S", "Save the current project"),
                ("Save Project As", "Ctrl+Shift+S", "Save project with a new name"),
                ("Import URDF", "Ctrl+I", "Import a URDF file"),
                ("Export URDF", "Ctrl+E", "Export URDF to file"),
                ("Close Tab", "Ctrl+W", "Close current tab"),
                ("Quit", "Ctrl+Q", "Exit RoboShire"),
            ],

            ShortcutCategory.EDIT: [
                ("Undo", "Ctrl+Z", "Undo last action"),
                ("Redo", "Ctrl+Y", "Redo previously undone action"),
                ("Cut", "Ctrl+X", "Cut selection to clipboard"),
                ("Copy", "Ctrl+C", "Copy selection to clipboard"),
                ("Paste", "Ctrl+V", "Paste from clipboard"),
                ("Delete", "Delete", "Delete selected item"),
                ("Select All", "Ctrl+A", "Select all items"),
                ("Find", "Ctrl+F", "Find in current view"),
            ],

            ShortcutCategory.BUILD: [
                ("Generate Code", "Ctrl+G", "Generate ROS2 code from node graph"),
                ("Build", "Ctrl+B", "Build workspace with colcon"),
                ("Clean Build", "Ctrl+Shift+B", "Clean and rebuild workspace"),
                ("Run", "Ctrl+R", "Launch ROS2 nodes"),
                ("Stop", "Ctrl+Shift+R", "Stop running nodes"),
                ("Quick Build & Run", "F5", "Build and run in one step"),
                ("Stop All", "Shift+F5", "Stop all running processes"),
            ],

            ShortcutCategory.VIEW: [
                ("Node Graph", "Ctrl+1", "Switch to Node Graph tab"),
                ("Build Output", "Ctrl+2", "Switch to Build tab"),
                ("Logs", "Ctrl+3", "Switch to Logs tab"),
                ("Node Status", "Ctrl+4", "Switch to Node Status tab"),
                ("Topics", "Ctrl+5", "Switch to Topics tab"),
                ("Parameters", "Ctrl+6", "Switch to Parameters tab"),
                ("Lifecycle", "Ctrl+7", "Switch to Lifecycle tab"),
                ("Packages", "Ctrl+9", "Open Package Browser"),
                ("Reset View", "Ctrl+0", "Reset zoom and view"),
                ("Zoom In", "Ctrl++", "Zoom in"),
                ("Zoom Out", "Ctrl+-", "Zoom out"),
                ("Fullscreen", "F11", "Toggle fullscreen mode"),
            ],

            ShortcutCategory.ROBOT: [
                ("Validate URDF", "Ctrl+Shift+V", "Validate current URDF file"),
                ("View in RViz2", "Ctrl+Shift+P", "Launch RViz2 with robot model"),
                ("Simulate in Gazebo", "Ctrl+Shift+G", "Launch Gazebo simulation"),
                ("Open Examples", "Ctrl+Shift+E", "Open robot examples browser"),
            ],

            ShortcutCategory.GRAPH: [
                ("Add Node", "Ctrl+Shift+N", "Add a new node to graph"),
                ("Duplicate Node", "Ctrl+D", "Duplicate selected node"),
                ("Delete Node", "Delete", "Delete selected node"),
                ("Auto Layout", "Ctrl+L", "Automatically arrange nodes"),
            ],

            ShortcutCategory.TOOLS: [
                ("rqt_graph", "Ctrl+Shift+T", "Launch rqt_graph tool"),
                ("rqt_console", "Ctrl+Shift+C", "Launch rqt_console tool"),
                ("Hardware Test Panel", "Ctrl+T", "Open hardware testing & debugging panel"),
                ("Serial Monitor", "Ctrl+Shift+O", "Open Arduino/ESP32 serial monitor"),
                ("micro-ROS Agent", "Ctrl+Shift+M", "Launch micro-ROS agent for embedded devices"),
                ("Clear Logs", "Ctrl+Shift+L", "Clear all log messages"),
                ("Developer Console", "F12", "Open developer console"),
            ],

            ShortcutCategory.HELP: [
                ("Help", "F1", "Open context-sensitive help"),
                ("Keyboard Shortcuts", "Ctrl+Shift+K", "Show keyboard shortcut cheat sheet"),
                ("About", "Ctrl+Shift+A", "About RoboShire"),
            ],
        }

    @staticmethod
    def detect_conflicts() -> List[Tuple[str, str, List[str]]]:
        """
        Detect shortcut conflicts

        Returns:
            List of (shortcut, category, [conflicting_actions]) tuples
        """
        shortcuts = KeyboardShortcuts.get_all_shortcuts()
        shortcut_map: Dict[str, List[Tuple[str, str]]] = {}

        # Build map of shortcuts to actions
        for category, actions in shortcuts.items():
            for name, shortcut, _ in actions:
                if shortcut not in shortcut_map:
                    shortcut_map[shortcut] = []
                shortcut_map[shortcut].append((category, name))

        # Find conflicts (same shortcut used multiple times)
        conflicts = []
        for shortcut, usages in shortcut_map.items():
            if len(usages) > 1:
                categories = [cat for cat, _ in usages]
                actions = [name for _, name in usages]
                conflicts.append((shortcut, ", ".join(categories), actions))

        return conflicts

    @staticmethod
    def generate_cheatsheet_html() -> str:
        """
        Generate HTML cheat sheet for all keyboard shortcuts

        Returns:
            HTML string with formatted shortcut reference
        """
        shortcuts = KeyboardShortcuts.get_all_shortcuts()

        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <style>
                body {
                    font-family: 'Segoe UI', Arial, sans-serif;
                    margin: 20px;
                    background-color: #f5f5f5;
                }
                h1 {
                    color: #2196F3;
                    text-align: center;
                    margin-bottom: 30px;
                }
                .category {
                    background-color: white;
                    margin-bottom: 20px;
                    padding: 15px;
                    border-radius: 8px;
                    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                }
                .category h2 {
                    color: #333;
                    margin-top: 0;
                    margin-bottom: 15px;
                    border-bottom: 2px solid #2196F3;
                    padding-bottom: 10px;
                }
                table {
                    width: 100%;
                    border-collapse: collapse;
                }
                tr {
                    border-bottom: 1px solid #eee;
                }
                tr:last-child {
                    border-bottom: none;
                }
                td {
                    padding: 10px 5px;
                }
                .action-name {
                    font-weight: 500;
                    color: #333;
                    width: 30%;
                }
                .shortcut {
                    font-family: 'Courier New', monospace;
                    background-color: #f0f0f0;
                    padding: 4px 8px;
                    border-radius: 4px;
                    color: #2196F3;
                    font-weight: bold;
                    width: 25%;
                }
                .description {
                    color: #666;
                    width: 45%;
                }
                .footer {
                    text-align: center;
                    margin-top: 30px;
                    color: #999;
                    font-size: 12px;
                }
            </style>
        </head>
        <body>
            <h1>RoboShire Keyboard Shortcuts</h1>
        """

        for category, actions in shortcuts.items():
            html += f'<div class="category">\n'
            html += f'<h2>{category}</h2>\n'
            html += '<table>\n'

            for name, shortcut, description in actions:
                html += '<tr>\n'
                html += f'<td class="action-name">{name}</td>\n'
                html += f'<td class="shortcut">{shortcut}</td>\n'
                html += f'<td class="description">{description}</td>\n'
                html += '</tr>\n'

            html += '</table>\n'
            html += '</div>\n'

        html += """
            <div class="footer">
                <p>RoboShire - Visual IDE for ROS2 Robotics</p>
                <p>Press Ctrl+Shift+K to toggle this cheat sheet</p>
            </div>
        </body>
        </html>
        """

        return html


# Convenience function for quick shortcut application
def apply_shortcut(action: QAction, shortcut: QKeySequence, tooltip: str = ""):
    """
    Convenience function to apply shortcut to action

    Args:
        action: QAction to configure
        shortcut: Keyboard shortcut
        tooltip: Optional tooltip description
    """
    KeyboardShortcuts.apply_to_action(action, shortcut, tooltip, show_in_tooltip=bool(tooltip))
