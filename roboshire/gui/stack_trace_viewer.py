"""
Stack Trace Viewer

Enhanced error visualization with:
- Syntax-highlighted stack traces
- Clickable file/line links
- Error categorization
- Common fixes suggestions
- Integration with Log Viewer

Author: RoboShire Team
Phase: 11 (Advanced Features)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTextEdit, QListWidget, QListWidgetItem, QSplitter,
    QGroupBox, QMessageBox
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QTextCharFormat, QColor, QSyntaxHighlighter, QTextCursor
from typing import List, Optional, Dict
import re
from pathlib import Path


class StackFrame:
    """Single stack frame"""
    def __init__(self, file_path: str, line_number: int, function_name: str, code_line: str = ""):
        self.file_path = file_path
        self.line_number = line_number
        self.function_name = function_name
        self.code_line = code_line


class ErrorInfo:
    """Error information with stack trace"""
    def __init__(self, error_type: str, message: str, timestamp: str = ""):
        self.error_type = error_type  # e.g., "ImportError", "RuntimeError"
        self.message = message
        self.timestamp = timestamp
        self.stack_frames: List[StackFrame] = []
        self.severity = "error"  # "error", "warning", "critical"
        self.node_name = ""


class PythonStackTraceHighlighter(QSyntaxHighlighter):
    """Syntax highlighter for Python stack traces"""

    def __init__(self, parent=None):
        super().__init__(parent)

        # Define formats
        self.file_format = QTextCharFormat()
        self.file_format.setForeground(QColor(33, 150, 243))  # Blue
        self.file_format.setFontWeight(QFont.Bold)

        self.line_format = QTextCharFormat()
        self.line_format.setForeground(QColor(156, 39, 176))  # Purple

        self.function_format = QTextCharFormat()
        self.function_format.setForeground(QColor(0, 150, 136))  # Teal

        self.error_format = QTextCharFormat()
        self.error_format.setForeground(QColor(244, 67, 54))  # Red
        self.error_format.setFontWeight(QFont.Bold)

        self.code_format = QTextCharFormat()
        self.code_format.setForeground(QColor(96, 125, 139))  # Grey

    def highlightBlock(self, text):
        """Highlight a block of text"""
        # File paths: File "/path/to/file.py", line 123
        file_pattern = r'File "([^"]+)"'
        for match in re.finditer(file_pattern, text):
            self.setFormat(match.start(), match.end() - match.start(), self.file_format)

        # Line numbers
        line_pattern = r'line \d+'
        for match in re.finditer(line_pattern, text):
            self.setFormat(match.start(), match.end() - match.start(), self.line_format)

        # Function names
        func_pattern = r'in (\w+)'
        for match in re.finditer(func_pattern, text):
            self.setFormat(match.start(1), match.end(1) - match.start(1), self.function_format)

        # Error types: SomeError: message
        error_pattern = r'^\w+Error:'
        for match in re.finditer(error_pattern, text):
            self.setFormat(match.start(), match.end() - match.start(), self.error_format)

        # Code lines (indented)
        if text.startswith('    ') and not text.strip().startswith('File'):
            self.setFormat(0, len(text), self.code_format)


class StackTraceViewer(QWidget):
    """
    Enhanced stack trace viewer with analysis and suggestions
    """

    # Signal to open file at line
    open_file_requested = Signal(str, int)  # file_path, line_number

    def __init__(self, parent=None):
        super().__init__(parent)

        self.current_error: Optional[ErrorInfo] = None
        self.error_history: List[ErrorInfo] = []

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Header
        header = self._create_header()
        layout.addWidget(header)

        # Main splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left: Error history
        left_panel = self._create_history_panel()
        splitter.addWidget(left_panel)

        # Right: Stack trace and suggestions
        right_panel = self._create_details_panel()
        splitter.addWidget(right_panel)

        splitter.setSizes([250, 750])
        layout.addWidget(splitter)

    def _create_header(self) -> QWidget:
        """Create header"""
        header = QWidget()
        header.setStyleSheet("""
            QWidget {
                background-color: #F44336;
                border-radius: 8px;
                padding: 15px;
            }
        """)

        layout = QVBoxLayout(header)

        title = QLabel("🐛 Error & Stack Trace Viewer")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        subtitle = QLabel("Analyze errors with syntax highlighting and helpful suggestions")
        subtitle.setStyleSheet("color: rgba(255, 255, 255, 0.9);")
        layout.addWidget(subtitle)

        return header

    def _create_history_panel(self) -> QWidget:
        """Create error history panel"""
        panel = QGroupBox("Error History")
        layout = QVBoxLayout(panel)

        # Clear button
        clear_btn = QPushButton("🗑 Clear History")
        clear_btn.clicked.connect(self._clear_history)
        layout.addWidget(clear_btn)

        # Error list
        self.error_list = QListWidget()
        self.error_list.currentItemChanged.connect(self._on_error_selected)
        layout.addWidget(self.error_list)

        # Stats
        self.stats_label = QLabel("No errors recorded")
        layout.addWidget(self.stats_label)

        return panel

    def _create_details_panel(self) -> QWidget:
        """Create details panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # Error summary
        summary_group = QGroupBox("Error Summary")
        summary_layout = QVBoxLayout(summary_group)

        self.error_title_label = QLabel("No error selected")
        error_font = self.error_title_label.font()
        error_font.setPointSize(14)
        error_font.setBold(True)
        self.error_title_label.setFont(error_font)
        summary_layout.addWidget(self.error_title_label)

        self.error_message_label = QLabel("")
        self.error_message_label.setWordWrap(True)
        summary_layout.addWidget(self.error_message_label)

        layout.addWidget(summary_group)

        # Stack trace viewer
        trace_group = QGroupBox("Stack Trace (Click to Jump to Code)")
        trace_layout = QVBoxLayout(trace_group)

        self.stack_trace_viewer = QTextEdit()
        self.stack_trace_viewer.setReadOnly(True)
        self.stack_trace_viewer.setFontFamily("Consolas, Courier New, monospace")
        self.stack_trace_viewer.cursorPositionChanged.connect(self._on_cursor_changed)

        # Apply syntax highlighter
        self.highlighter = PythonStackTraceHighlighter(self.stack_trace_viewer.document())

        trace_layout.addWidget(self.stack_trace_viewer)

        jump_btn = QPushButton("📂 Jump to Selected Line")
        jump_btn.clicked.connect(self._jump_to_code)
        trace_layout.addWidget(jump_btn)

        layout.addWidget(trace_group)

        # Suggestions
        suggestions_group = QGroupBox("💡 Suggested Fixes")
        suggestions_layout = QVBoxLayout(suggestions_group)

        self.suggestions_text = QTextEdit()
        self.suggestions_text.setReadOnly(True)
        self.suggestions_text.setMaximumHeight(150)
        suggestions_layout.addWidget(self.suggestions_text)

        layout.addWidget(suggestions_group)

        return panel

    def add_error(self, error_log: str, node_name: str = ""):
        """
        Add error from log output

        Args:
            error_log: Full error log including stack trace
            node_name: Name of node that produced error
        """
        # Parse error
        error = self._parse_error_log(error_log, node_name)

        if not error:
            return

        # Add to history
        self.error_history.append(error)

        # Add to list
        severity_emoji = {"critical": "🔴", "error": "🟠", "warning": "🟡"}.get(error.severity, "🟠")
        item = QListWidgetItem(f"{severity_emoji} {error.error_type}: {error.message[:50]}...")
        item.setData(Qt.UserRole, error)
        self.error_list.addItem(item)

        # Update stats
        self._update_stats()

        # Select newest error
        self.error_list.setCurrentItem(item)

    def _parse_error_log(self, log: str, node_name: str) -> Optional[ErrorInfo]:
        """Parse error from log output"""
        # Look for Python traceback
        if "Traceback (most recent call last):" not in log:
            return None

        lines = log.split('\n')

        # Find error type and message (last non-empty line)
        error_line = ""
        for line in reversed(lines):
            if line.strip() and not line.strip().startswith('File'):
                error_line = line.strip()
                break

        if not error_line:
            return None

        # Parse error type
        error_type = "Error"
        message = error_line

        if ':' in error_line:
            parts = error_line.split(':', 1)
            error_type = parts[0].strip()
            message = parts[1].strip()

        error = ErrorInfo(error_type, message)
        error.node_name = node_name

        # Parse stack frames
        i = 0
        while i < len(lines):
            line = lines[i]

            # Look for File line
            file_match = re.match(r'\s*File "([^"]+)", line (\d+), in (\w+)', line)
            if file_match:
                file_path = file_match.group(1)
                line_num = int(file_match.group(2))
                func_name = file_match.group(3)

                # Next line might be code
                code_line = ""
                if i + 1 < len(lines) and lines[i + 1].strip():
                    code_line = lines[i + 1].strip()

                frame = StackFrame(file_path, line_num, func_name, code_line)
                error.stack_frames.append(frame)

            i += 1

        return error

    def _on_error_selected(self, current: QListWidgetItem, previous: QListWidgetItem):
        """Handle error selection"""
        if not current:
            return

        error = current.data(Qt.UserRole)
        if not error:
            return

        self.current_error = error

        # Update summary
        self.error_title_label.setText(f"{error.error_type}")
        self.error_message_label.setText(error.message)

        # Update stack trace
        stack_text = self._format_stack_trace(error)
        self.stack_trace_viewer.setText(stack_text)

        # Update suggestions
        suggestions = self._generate_suggestions(error)
        self.suggestions_text.setMarkdown(suggestions)

    def _format_stack_trace(self, error: ErrorInfo) -> str:
        """Format stack trace for display"""
        lines = ["Traceback (most recent call last):\n"]

        for frame in error.stack_frames:
            lines.append(f'  File "{frame.file_path}", line {frame.line_number}, in {frame.function_name}')
            if frame.code_line:
                lines.append(f'    {frame.code_line}')

        lines.append(f'\n{error.error_type}: {error.message}')

        return '\n'.join(lines)

    def _generate_suggestions(self, error: ErrorInfo) -> str:
        """Generate helpful suggestions based on error type"""
        suggestions = []

        # Common error patterns
        if "ImportError" in error.error_type or "ModuleNotFoundError" in error.error_type:
            suggestions.append("**Import Error Detected**")
            suggestions.append("- Install missing package: `pip install <package-name>` or `sudo apt install ros-humble-<package>`")
            suggestions.append("- Check if package is in workspace and built")
            suggestions.append("- Verify Python path includes workspace")

        elif "AttributeError" in error.error_type:
            suggestions.append("**Attribute Error Detected**")
            suggestions.append("- Check if object is None before accessing attributes")
            suggestions.append("- Verify object type matches expected type")
            suggestions.append("- Check for typos in attribute names")

        elif "TypeError" in error.error_type:
            suggestions.append("**Type Error Detected**")
            suggestions.append("- Check function arguments match expected types")
            suggestions.append("- Verify ROS message types are correct")
            suggestions.append("- Check if variables are initialized")

        elif "KeyError" in error.error_type:
            suggestions.append("**Key Error Detected**")
            suggestions.append("- Dictionary key does not exist")
            suggestions.append("- Use `.get()` method for safe access")
            suggestions.append("- Check parameter names in launch files")

        elif "FileNotFoundError" in error.error_type:
            suggestions.append("**File Not Found**")
            suggestions.append("- Verify file path is correct")
            suggestions.append("- Check if file exists in workspace")
            suggestions.append("- Use absolute paths or Path.resolve()")

        elif "ConnectionRefusedError" in error.error_type or "socket" in error.message.lower():
            suggestions.append("**Connection Error Detected**")
            suggestions.append("- Check if ROS2 daemon is running")
            suggestions.append("- Verify network configuration")
            suggestions.append("- Restart ROS2 nodes")

        else:
            suggestions.append("**General Debugging Tips**")
            suggestions.append("- Check the stack trace to find the source file")
            suggestions.append("- Add print statements or logging")
            suggestions.append("- Verify all dependencies are installed")
            suggestions.append("- Check ROS2 topic/service names")

        # Add common next steps
        suggestions.append("\n**Next Steps:**")
        suggestions.append("1. Click a stack frame to jump to code")
        suggestions.append("2. Check Logs tab for more context")
        suggestions.append("3. Review Node Status tab")

        return '\n'.join(suggestions)

    def _on_cursor_changed(self):
        """Handle cursor position change"""
        # Could highlight current stack frame
        pass

    def _jump_to_code(self):
        """Jump to code at selected stack frame"""
        if not self.current_error:
            return

        # Get current line in stack trace
        cursor = self.stack_trace_viewer.textCursor()
        text = cursor.block().text()

        # Parse file and line from current line
        file_match = re.match(r'\s*File "([^"]+)", line (\d+)', text)
        if file_match:
            file_path = file_match.group(1)
            line_num = int(file_match.group(2))

            self.open_file_requested.emit(file_path, line_num)
        else:
            QMessageBox.information(
                self,
                "Jump to Code",
                "Click on a stack frame line (File \"...\", line N) to jump to code."
            )

    def _clear_history(self):
        """Clear error history"""
        self.error_history.clear()
        self.error_list.clear()
        self.current_error = None
        self._update_stats()

        # Clear details
        self.error_title_label.setText("No error selected")
        self.error_message_label.setText("")
        self.stack_trace_viewer.clear()
        self.suggestions_text.clear()

    def _update_stats(self):
        """Update statistics"""
        if not self.error_history:
            self.stats_label.setText("No errors recorded")
        else:
            self.stats_label.setText(f"Total errors: {len(self.error_history)}")
