"""
Command Palette

Modern command palette with fuzzy search (VS Code/Sublime Text pattern).
Addresses Priority 2 Task #5 from UX evaluation.

Features:
- Ctrl+Shift+P shortcut
- Fuzzy search across all commands
- Recent commands at top
- Keyboard navigation
- Shows keyboard shortcuts
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QLineEdit, QListWidget, QListWidgetItem,
    QLabel, QWidget, QHBoxLayout
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QAction, QKeyEvent, QFont
from typing import List, Optional, Dict
import logging
import re


class CommandPaletteItem(QWidget):
    """
    Custom widget for command palette list items

    Shows:
    - Command name
    - Keyboard shortcut (if available)
    - Icon (if available)
    """

    def __init__(self, action: QAction, parent=None):
        super().__init__(parent)
        self.action = action
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)

        # Command name
        name_label = QLabel(self.action.text().replace("&", ""))
        name_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(name_label)

        layout.addStretch()

        # Keyboard shortcut
        shortcut = self.action.shortcut()
        if not shortcut.isEmpty():
            shortcut_label = QLabel(shortcut.toString())
            shortcut_label.setStyleSheet("""
                QLabel {
                    background-color: #E0E0E0;
                    border: 1px solid #999;
                    border-radius: 3px;
                    padding: 2px 6px;
                    font-family: monospace;
                    font-size: 10px;
                    color: #333;
                }
            """)
            layout.addWidget(shortcut_label)


class CommandPalette(QDialog):
    """
    Command Palette with fuzzy search

    Usage:
        palette = CommandPalette(all_actions, parent=main_window)
        palette.exec()
    """

    command_executed = Signal(QAction)

    def __init__(self, actions: List[QAction], parent=None):
        super().__init__(parent)
        self.all_actions = [a for a in actions if a.text()]  # Filter out separators
        self.recent_actions: List[QAction] = []
        self.max_recent = 10

        self.logger = logging.getLogger(__name__)

        self._setup_ui()
        self._populate_commands()

        # Focus on search immediately
        QTimer.singleShot(0, self.search_input.setFocus)

    def _setup_ui(self):
        """Setup UI components"""
        self.setWindowTitle("Command Palette")
        self.setModal(True)
        self.resize(600, 400)

        # Remove window decorations for more modern look
        self.setWindowFlags(Qt.Popup | Qt.FramelessWindowHint)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Search input
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("Type command name... (fuzzy search)")
        self.search_input.textChanged.connect(self._on_search_changed)
        self.search_input.setStyleSheet("""
            QLineEdit {
                padding: 12px;
                font-size: 14px;
                border: none;
                border-bottom: 2px solid #0078D4;
                background-color: #F5F5F5;
            }
        """)
        layout.addWidget(self.search_input)

        # Help text
        help_label = QLabel("↑↓ Navigate  Enter Execute  Esc Close")
        help_label.setStyleSheet("""
            QLabel {
                padding: 6px 12px;
                font-size: 10px;
                color: #666;
                background-color: #FAFAFA;
                border-bottom: 1px solid #E0E0E0;
            }
        """)
        layout.addWidget(help_label)

        # Command list
        self.command_list = QListWidget()
        self.command_list.setStyleSheet("""
            QListWidget {
                border: none;
                outline: none;
                background-color: white;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #F0F0F0;
            }
            QListWidget::item:selected {
                background-color: #E3F2FD;
                color: black;
            }
            QListWidget::item:hover {
                background-color: #F5F5F5;
            }
        """)
        self.command_list.itemActivated.connect(self._on_item_activated)
        layout.addWidget(self.command_list)

    def _populate_commands(self, filter_text: str = ""):
        """
        Populate command list with optional fuzzy filter

        Args:
            filter_text: Search query (empty shows all)
        """
        self.command_list.clear()

        # Get filtered actions
        if filter_text:
            actions = self._fuzzy_filter(filter_text)
        else:
            # Show recent commands first, then all others
            recent_set = set(self.recent_actions)
            other_actions = [a for a in self.all_actions if a not in recent_set]
            actions = self.recent_actions + other_actions

        # Add to list
        for i, action in enumerate(actions):
            item = QListWidgetItem(self.command_list)

            # Create custom widget
            widget = CommandPaletteItem(action)
            item.setSizeHint(widget.sizeHint())

            self.command_list.addItem(item)
            self.command_list.setItemWidget(item, widget)

            # Store action reference
            item.setData(Qt.UserRole, action)

            # Mark recent commands
            if i < len(self.recent_actions) and not filter_text:
                widget.layout().itemAt(0).widget().setText(
                    "⏱ " + widget.layout().itemAt(0).widget().text()
                )

        # Select first item
        if self.command_list.count() > 0:
            self.command_list.setCurrentRow(0)

    def _fuzzy_filter(self, query: str) -> List[QAction]:
        """
        Fuzzy filter actions by query

        Matches if all characters in query appear in order in action text.
        Example: "bp" matches "Build Project", "run" matches "Run"

        Args:
            query: Search query

        Returns:
            List of matching actions, sorted by relevance
        """
        query = query.lower()
        matches = []

        for action in self.all_actions:
            text = action.text().replace("&", "").lower()

            # Check if all query characters appear in order
            score = self._fuzzy_match_score(query, text)
            if score > 0:
                matches.append((score, action))

        # Sort by score (higher = better match)
        matches.sort(key=lambda x: x[0], reverse=True)

        return [action for score, action in matches]

    def _fuzzy_match_score(self, query: str, text: str) -> int:
        """
        Calculate fuzzy match score

        Returns:
            Score (higher = better match), 0 = no match
        """
        if not query:
            return 1

        # Check if exact match
        if query in text:
            return 1000

        # Check if all characters appear in order
        text_index = 0
        matched_chars = 0

        for char in query:
            found = text.find(char, text_index)
            if found == -1:
                return 0  # Character not found

            matched_chars += 1
            text_index = found + 1

        # Score based on:
        # 1. How many characters matched
        # 2. How early in the string they appeared
        score = matched_chars * 10
        score += (100 - text_index)  # Earlier matches are better

        return score

    def _on_search_changed(self, text: str):
        """Handle search input change"""
        self._populate_commands(text)

    def _on_item_activated(self, item: QListWidgetItem):
        """Handle command execution"""
        action = item.data(Qt.UserRole)
        if action:
            self._execute_action(action)

    def _execute_action(self, action: QAction):
        """
        Execute action and close palette

        Args:
            action: QAction to execute
        """
        # Add to recent commands
        if action in self.recent_actions:
            self.recent_actions.remove(action)
        self.recent_actions.insert(0, action)
        self.recent_actions = self.recent_actions[:self.max_recent]

        # Emit signal
        self.command_executed.emit(action)

        # Trigger action
        action.trigger()

        # Close palette
        self.accept()

        self.logger.info(f"Executed command: {action.text()}")

    def keyPressEvent(self, event: QKeyEvent):
        """Handle keyboard events"""
        key = event.key()

        if key == Qt.Key_Escape:
            # Close palette
            self.reject()

        elif key == Qt.Key_Return or key == Qt.Key_Enter:
            # Execute selected command
            current_item = self.command_list.currentItem()
            if current_item:
                self._on_item_activated(current_item)

        elif key == Qt.Key_Down:
            # Navigate down
            current_row = self.command_list.currentRow()
            if current_row < self.command_list.count() - 1:
                self.command_list.setCurrentRow(current_row + 1)

        elif key == Qt.Key_Up:
            # Navigate up
            current_row = self.command_list.currentRow()
            if current_row > 0:
                self.command_list.setCurrentRow(current_row - 1)

        else:
            # Pass to search input
            if not self.search_input.hasFocus():
                self.search_input.setFocus()
                self.search_input.event(event)


# Convenience function
def show_command_palette(main_window, actions: List[QAction]) -> Optional[QAction]:
    """
    Show command palette and return executed action

    Args:
        main_window: Parent window
        actions: List of all available actions

    Returns:
        Executed action or None if cancelled
    """
    palette = CommandPalette(actions, parent=main_window)

    executed_action = None

    def on_command_executed(action):
        nonlocal executed_action
        executed_action = action

    palette.command_executed.connect(on_command_executed)
    palette.exec()

    return executed_action


# Example usage and testing
if __name__ == '__main__':
    import sys
    from PySide6.QtWidgets import QApplication, QMainWindow
    from PySide6.QtGui import QKeySequence

    app = QApplication(sys.argv)

    # Create test window with sample actions
    window = QMainWindow()
    window.setWindowTitle("Command Palette Test")

    # Create sample actions
    test_actions = [
        QAction("New Project", window),
        QAction("Open Project", window),
        QAction("Save Project", window),
        QAction("Build Project", window),
        QAction("Run", window),
        QAction("Stop", window),
        QAction("Clean Build", window),
        QAction("Generate Code", window),
        QAction("Import URDF", window),
        QAction("Export URDF", window),
        QAction("Open RViz", window),
        QAction("Open Gazebo", window),
        QAction("Start MuJoCo Viewer", window),
        QAction("Node Graph", window),
        QAction("Parameters", window),
        QAction("Topics", window),
        QAction("Services", window),
        QAction("Settings", window),
        QAction("Help", window),
        QAction("About", window),
    ]

    # Add shortcuts to some actions
    test_actions[0].setShortcut(QKeySequence("Ctrl+N"))
    test_actions[1].setShortcut(QKeySequence("Ctrl+O"))
    test_actions[2].setShortcut(QKeySequence("Ctrl+S"))
    test_actions[3].setShortcut(QKeySequence("Ctrl+B"))
    test_actions[4].setShortcut(QKeySequence("Ctrl+R"))

    # Create command palette button
    def show_palette():
        action = show_command_palette(window, test_actions)
        if action:
            print(f"Executed: {action.text()}")

    # Add toolbar with button to open palette
    from PySide6.QtWidgets import QPushButton
    button = QPushButton("Open Command Palette (Ctrl+Shift+P)")
    button.clicked.connect(show_palette)
    window.setCentralWidget(button)

    # Add shortcut
    palette_action = QAction(window)
    palette_action.setShortcut(QKeySequence("Ctrl+Shift+P"))
    palette_action.triggered.connect(show_palette)
    window.addAction(palette_action)

    window.resize(800, 600)
    window.show()

    print("Press Ctrl+Shift+P to open command palette")
    print("Try fuzzy search: 'bp' → Build Project, 'ng' → Node Graph")

    sys.exit(app.exec())
