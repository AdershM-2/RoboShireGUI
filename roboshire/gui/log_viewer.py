"""
Log Viewer - GUI widget for displaying ROS2 node logs in real-time

Displays real-time node output with syntax highlighting and filtering.
Pattern: Similar to BuildOutputViewer from Phase 3
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit,
    QPushButton, QComboBox, QLabel, QFrame, QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QTextCharFormat, QColor, QFont, QTextCursor
import re
import logging


class LogViewer(QWidget):
    """
    Widget for displaying ROS2 node logs with filtering.

    Features:
    - Real-time log display
    - Filter by node name
    - Filter by severity (ALL, DEBUG, INFO, WARN, ERROR)
    - Color-coded output
    - Clear and save functionality
    """

    # Signals
    clear_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.all_logs = []  # Store all logs: (line, node_name, severity)
        self.current_node_filter = "All Nodes"
        self.current_severity_filter = "ALL"

        # Regex patterns for detecting severity
        self.pattern_error = re.compile(r'\[ERROR\]|\berror\b|failed|exception|traceback', re.IGNORECASE)
        self.pattern_warn = re.compile(r'\[WARN\]|\[WARNING\]|\bwarn\b', re.IGNORECASE)
        self.pattern_info = re.compile(r'\[INFO\]', re.IGNORECASE)
        self.pattern_debug = re.compile(r'\[DEBUG\]', re.IGNORECASE)

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Filter bar at top
        self.filter_frame = self._create_filter_bar()
        layout.addWidget(self.filter_frame)

        # Output text area
        self.output_text = QPlainTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setMaximumBlockCount(10000)  # Limit to 10k lines
        self.output_text.setLineWrapMode(QPlainTextEdit.NoWrap)

        # Use monospace font
        font = QFont("Consolas", 9)
        if not font.exactMatch():
            font = QFont("Courier New", 9)
        self.output_text.setFont(font)

        layout.addWidget(self.output_text)

        # Button bar at bottom
        button_layout = QHBoxLayout()

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_logs)

        self.save_button = QPushButton("Save Logs...")
        self.save_button.clicked.connect(self._on_save_clicked)

        button_layout.addWidget(self.clear_button)
        button_layout.addWidget(self.save_button)
        button_layout.addStretch()

        layout.addLayout(button_layout)

    def _create_filter_bar(self) -> QFrame:
        """Create filter bar with node and severity selection"""
        frame = QFrame()
        frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(5, 5, 5, 5)

        # Node filter
        layout.addWidget(QLabel("Node:"))
        self.node_filter_combo = QComboBox()
        self.node_filter_combo.addItem("All Nodes")
        self.node_filter_combo.currentTextChanged.connect(self._on_node_filter_changed)
        layout.addWidget(self.node_filter_combo)

        layout.addSpacing(20)

        # Severity filter
        layout.addWidget(QLabel("Severity:"))
        self.severity_filter_combo = QComboBox()
        self.severity_filter_combo.addItems(["ALL", "DEBUG", "INFO", "WARN", "ERROR"])
        self.severity_filter_combo.currentTextChanged.connect(self._on_severity_filter_changed)
        layout.addWidget(self.severity_filter_combo)

        layout.addStretch()

        # Stats label
        self.stats_label = QLabel("0 log entries")
        layout.addWidget(self.stats_label)

        return frame

    def append_log(self, line: str, node_name: str = "", severity: str = "INFO"):
        """
        Append a log line

        Args:
            line: Log line text
            node_name: Source node name
            severity: Log severity (DEBUG, INFO, WARN, ERROR)
        """
        # Auto-detect severity if not provided or is generic
        if severity == "INFO" or not severity:
            severity = self._detect_severity(line)

        # Store log entry
        self.all_logs.append((line, node_name, severity))

        # Update node filter combo if this is a new node
        if node_name and node_name not in [
            self.node_filter_combo.itemText(i)
            for i in range(self.node_filter_combo.count())
        ]:
            self.node_filter_combo.addItem(node_name)

        # Update stats
        self._update_stats()

        # Check if this log should be displayed based on filters
        if not self._should_display_log(node_name, severity):
            return

        # Apply syntax highlighting and display
        self._display_log(line, severity)

    def _detect_severity(self, line: str) -> str:
        """
        Auto-detect severity from log line

        Args:
            line: Log line

        Returns:
            Detected severity (ERROR, WARN, INFO, DEBUG)
        """
        if self.pattern_error.search(line):
            return "ERROR"
        elif self.pattern_warn.search(line):
            return "WARN"
        elif self.pattern_debug.search(line):
            return "DEBUG"
        else:
            return "INFO"

    def _should_display_log(self, node_name: str, severity: str) -> bool:
        """
        Check if log should be displayed based on current filters

        Args:
            node_name: Node name
            severity: Log severity

        Returns:
            True if should be displayed
        """
        # Check node filter
        if self.current_node_filter != "All Nodes" and node_name != self.current_node_filter:
            return False

        # Check severity filter
        if self.current_severity_filter != "ALL" and severity != self.current_severity_filter:
            return False

        return True

    def _display_log(self, line: str, severity: str):
        """
        Display a log line with color coding

        Args:
            line: Log line
            severity: Severity level
        """
        # Format line based on severity
        formatted = self._format_line(line, severity)

        # Append to text area
        cursor = self.output_text.textCursor()
        cursor.movePosition(QTextCursor.End)

        # Apply formatting
        if formatted['color']:
            fmt = QTextCharFormat()
            fmt.setForeground(QColor(formatted['color']))
            if formatted['bold']:
                fmt.setFontWeight(QFont.Bold)
            cursor.setCharFormat(fmt)
        else:
            cursor.setCharFormat(QTextCharFormat())  # Reset format

        cursor.insertText(formatted['text'] + '\n')

        # Auto-scroll to bottom
        scrollbar = self.output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _format_line(self, line: str, severity: str) -> dict:
        """
        Format a line with color and style based on severity

        Args:
            line: Line to format
            severity: Severity level

        Returns:
            Dict with 'text', 'color', 'bold'
        """
        result = {
            'text': line,
            'color': None,
            'bold': False
        }

        # Color by severity
        if severity == "ERROR":
            result['color'] = '#cc0000'  # Red
            result['bold'] = True
        elif severity == "WARN":
            result['color'] = '#ff8800'  # Orange
            result['bold'] = True
        elif severity == "INFO":
            result['color'] = '#0066cc'  # Blue
            result['bold'] = False
        elif severity == "DEBUG":
            result['color'] = '#666666'  # Gray
            result['bold'] = False

        return result

    def _on_node_filter_changed(self, node_name: str):
        """Handle node filter change"""
        self.current_node_filter = node_name
        self._refresh_display()

    def _on_severity_filter_changed(self, severity: str):
        """Handle severity filter change"""
        self.current_severity_filter = severity
        self._refresh_display()

    def _refresh_display(self):
        """Refresh display based on current filters"""
        # Clear display
        self.output_text.clear()

        # Re-display filtered logs
        for line, node_name, severity in self.all_logs:
            if self._should_display_log(node_name, severity):
                self._display_log(line, severity)

    def clear_logs(self):
        """Clear all logs"""
        self.all_logs.clear()
        self.output_text.clear()
        self._update_stats()

        # Keep only "All Nodes" in filter
        while self.node_filter_combo.count() > 1:
            self.node_filter_combo.removeItem(1)

        self.clear_requested.emit()

    def _update_stats(self):
        """Update statistics label"""
        total = len(self.all_logs)
        self.stats_label.setText(f"{total} log entries")

    def _on_save_clicked(self):
        """Handle save button click"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Logs",
            "ros2_logs.txt",
            "Text Files (*.txt);;All Files (*.*)"
        )

        if file_path:
            if self.save_logs(file_path):
                QMessageBox.information(
                    self,
                    "Success",
                    f"Logs saved to:\n{file_path}"
                )
            else:
                QMessageBox.warning(
                    self,
                    "Error",
                    "Failed to save logs"
                )

    def save_logs(self, file_path: str) -> bool:
        """
        Save logs to file

        Args:
            file_path: Path to save file

        Returns:
            True if successful
        """
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                # Write header
                f.write("=" * 60 + "\n")
                f.write("RoboShire - ROS2 Node Logs\n")
                f.write("=" * 60 + "\n\n")

                # Write filtered logs
                for line, node_name, severity in self.all_logs:
                    if self._should_display_log(node_name, severity):
                        if node_name:
                            f.write(f"[{severity}] [{node_name}] {line}\n")
                        else:
                            f.write(f"[{severity}] {line}\n")

            logging.info(f"Logs saved to {file_path}")
            return True
        except Exception as e:
            logging.error(f"Failed to save logs: {e}")
            return False

    def get_log_count(self) -> int:
        """
        Get total number of logs

        Returns:
            Number of log entries
        """
        return len(self.all_logs)

    def get_filtered_log_count(self) -> int:
        """
        Get number of logs matching current filters

        Returns:
            Number of filtered log entries
        """
        count = 0
        for line, node_name, severity in self.all_logs:
            if self._should_display_log(node_name, severity):
                count += 1
        return count
