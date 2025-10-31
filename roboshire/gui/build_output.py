"""
Build Output Viewer - GUI widget for displaying colcon build output

Displays real-time build output with syntax highlighting and progress tracking.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit,
    QPushButton, QProgressBar, QLabel, QFrame
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QTextCharFormat, QColor, QFont, QTextCursor
import re
import logging


class BuildOutputViewer(QWidget):
    """
    Widget for displaying build output with syntax highlighting.

    Features:
    - Real-time output display
    - Syntax highlighting (errors in red, warnings in yellow)
    - Progress bar tracking
    - Build status display
    - Clear and save functionality
    """

    # Signals
    build_started = Signal()
    build_finished = Signal(bool)  # True if successful, False if failed
    cancel_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.is_building = False
        self.build_start_time = None
        self.packages_total = 0
        self.packages_finished = 0

        # Regex patterns for parsing
        self.pattern_starting = re.compile(r"Starting\s+>>>\s+(\w+)")
        self.pattern_finished = re.compile(r"Finished\s+<<<\s+(\w+)\s+\[([0-9.]+)s\]")
        self.pattern_failed = re.compile(r"Failed\s+<<<\s+(\w+)")
        self.pattern_summary = re.compile(r"Summary:\s+(\d+)\s+package")

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # Status bar at top
        self.status_frame = self._create_status_bar()
        layout.addWidget(self.status_frame)

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
        self.clear_button.clicked.connect(self.clear_output)

        self.cancel_button = QPushButton("Cancel Build")
        self.cancel_button.clicked.connect(self._on_cancel_clicked)
        self.cancel_button.setEnabled(False)

        button_layout.addWidget(self.clear_button)
        button_layout.addWidget(self.cancel_button)
        button_layout.addStretch()

        layout.addLayout(button_layout)

    def _create_status_bar(self) -> QFrame:
        """Create status bar with progress info"""
        frame = QFrame()
        frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(5, 5, 5, 5)

        # Status label
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.status_label)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("%v/%m packages")
        layout.addWidget(self.progress_bar)

        # Time label
        self.time_label = QLabel("--:--")
        layout.addWidget(self.time_label)

        return frame

    def start_build(self, package_count: int = 0):
        """
        Signal that a build has started

        Args:
            package_count: Number of packages to build (0 if unknown)
        """
        self.is_building = True
        self.packages_total = package_count
        self.packages_finished = 0

        if package_count > 0:
            self.progress_bar.setMaximum(package_count)
            self.progress_bar.setValue(0)
        else:
            self.progress_bar.setMaximum(0)  # Indeterminate progress

        self.status_label.setText("Building...")
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        self.cancel_button.setEnabled(True)

        # Start timer for elapsed time
        import time
        self.build_start_time = time.time()
        self._start_time_update()

        self.build_started.emit()

    def finish_build(self, success: bool):
        """
        Signal that a build has finished

        Args:
            success: True if build succeeded, False if failed
        """
        self.is_building = False
        self.cancel_button.setEnabled(False)

        if success:
            self.status_label.setText("Build Successful")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.progress_bar.setValue(self.progress_bar.maximum())
        else:
            self.status_label.setText("Build Failed")
            self.status_label.setStyleSheet("font-weight: bold; color: red;")

        self._stop_time_update()
        self.build_finished.emit(success)

    def append_output(self, line: str, stream: str = "stdout"):
        """
        Append a line of output

        Args:
            line: Output line
            stream: Stream name ('stdout' or 'stderr')
        """
        # Parse line for special patterns and update progress
        self._parse_build_line(line)

        # Apply syntax highlighting
        formatted_line = self._format_line(line, stream)

        # Append to text area
        cursor = self.output_text.textCursor()
        cursor.movePosition(QTextCursor.End)

        # Apply formatting
        if formatted_line['color']:
            fmt = QTextCharFormat()
            fmt.setForeground(QColor(formatted_line['color']))
            if formatted_line['bold']:
                fmt.setFontWeight(QFont.Bold)
            cursor.setCharFormat(fmt)
        else:
            cursor.setCharFormat(QTextCharFormat())  # Reset format

        cursor.insertText(formatted_line['text'] + '\n')

        # Auto-scroll to bottom
        scrollbar = self.output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _format_line(self, line: str, stream: str) -> dict:
        """
        Format a line with color and style based on content

        Args:
            line: Line to format
            stream: Stream name

        Returns:
            Dict with 'text', 'color', 'bold'
        """
        result = {
            'text': line,
            'color': None,
            'bold': False
        }

        # Error patterns (red)
        if any(keyword in line.lower() for keyword in ['error', 'failed', 'traceback']):
            result['color'] = '#cc0000'  # Red
            result['bold'] = True

        # Warning patterns (yellow/orange)
        elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
            result['color'] = '#ff8800'  # Orange
            result['bold'] = True

        # Success patterns (green)
        elif 'finished <<<' in line.lower() or 'summary:' in line.lower():
            result['color'] = '#008800'  # Green
            result['bold'] = True

        # Info patterns (blue)
        elif 'starting >>>' in line.lower():
            result['color'] = '#0066cc'  # Blue
            result['bold'] = True

        # stderr is generally red unless already colored
        elif stream == 'stderr' and not result['color']:
            result['color'] = '#880000'  # Dark red

        return result

    def _parse_build_line(self, line: str):
        """
        Parse build output to update progress

        Args:
            line: Output line
        """
        # Check for "Starting >>> package"
        match = self.pattern_starting.match(line)
        if match:
            logging.info(f"Building package: {match.group(1)}")
            return

        # Check for "Finished <<< package [time]"
        match = self.pattern_finished.match(line)
        if match:
            self.packages_finished += 1
            self.progress_bar.setValue(self.packages_finished)
            logging.info(f"Finished package: {match.group(1)} in {match.group(2)}s")
            return

        # Check for "Failed <<< package"
        match = self.pattern_failed.match(line)
        if match:
            self.packages_finished += 1
            self.progress_bar.setValue(self.packages_finished)
            logging.error(f"Failed package: {match.group(1)}")
            return

        # Check for "Summary: N packages..."
        match = self.pattern_summary.match(line)
        if match:
            total = int(match.group(1))
            if self.packages_total == 0:
                self.packages_total = total
                self.progress_bar.setMaximum(total)

    def clear_output(self):
        """Clear all output"""
        self.output_text.clear()
        self.packages_finished = 0
        self.progress_bar.setValue(0)

    def _on_cancel_clicked(self):
        """Handle cancel button click"""
        self.cancel_requested.emit()
        self.status_label.setText("Cancelling...")
        self.status_label.setStyleSheet("font-weight: bold; color: orange;")
        self.cancel_button.setEnabled(False)

    def _start_time_update(self):
        """Start timer to update elapsed time"""
        self.time_timer = QTimer(self)
        self.time_timer.timeout.connect(self._update_elapsed_time)
        self.time_timer.start(1000)  # Update every second

    def _stop_time_update(self):
        """Stop time update timer"""
        if hasattr(self, 'time_timer'):
            self.time_timer.stop()

    def _update_elapsed_time(self):
        """Update elapsed time display"""
        if self.build_start_time:
            import time
            elapsed = time.time() - self.build_start_time
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            self.time_label.setText(f"{minutes:02d}:{seconds:02d}")

    def get_output_text(self) -> str:
        """
        Get all output text

        Returns:
            Full output as string
        """
        return self.output_text.toPlainText()

    def save_output(self, file_path: str) -> bool:
        """
        Save output to file

        Args:
            file_path: Path to save file

        Returns:
            True if successful
        """
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(self.get_output_text())
            logging.info(f"Build output saved to {file_path}")
            return True
        except Exception as e:
            logging.error(f"Failed to save output: {e}")
            return False
