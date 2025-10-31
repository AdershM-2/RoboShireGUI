"""
Code Editor Widget - Simple embedded code editor for viewing/editing generated files

Basic editor with syntax highlighting (preview version for v0.14.0).
Full QScintilla integration planned for v1.0.0.

Author: RoboShire Team
Version: 0.14.0
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton,
    QLabel, QFileDialog, QMessageBox, QComboBox, QToolBar
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import (
    QSyntaxHighlighter, QTextCharFormat, QFont, QColor,
    QTextCursor, QKeySequence
)
from pathlib import Path
import re


class PythonSyntaxHighlighter(QSyntaxHighlighter):
    """Basic Python syntax highlighter"""

    def __init__(self, document):
        super().__init__(document)

        # Define formats
        self.formats = {}

        # Keywords
        keyword_format = QTextCharFormat()
        keyword_format.setForeground(QColor(86, 156, 214))  # Blue
        keyword_format.setFontWeight(QFont.Bold)
        keywords = [
            'and', 'as', 'assert', 'break', 'class', 'continue', 'def',
            'del', 'elif', 'else', 'except', 'False', 'finally', 'for',
            'from', 'global', 'if', 'import', 'in', 'is', 'lambda', 'None',
            'nonlocal', 'not', 'or', 'pass', 'raise', 'return', 'True',
            'try', 'while', 'with', 'yield', 'async', 'await'
        ]
        self.formats['keyword'] = [(r'\b' + kw + r'\b', keyword_format) for kw in keywords]

        # Strings
        string_format = QTextCharFormat()
        string_format.setForeground(QColor(206, 145, 120))  # Orange-brown
        self.formats['string'] = [
            (r'"[^"\\]*(\\.[^"\\]*)*"', string_format),
            (r"'[^'\\]*(\\.[^'\\]*)*'", string_format),
        ]

        # Comments
        comment_format = QTextCharFormat()
        comment_format.setForeground(QColor(106, 153, 85))  # Green
        self.formats['comment'] = [(r'#[^\n]*', comment_format)]

        # Functions
        function_format = QTextCharFormat()
        function_format.setForeground(QColor(220, 220, 170))  # Yellow
        self.formats['function'] = [(r'\bdef\s+(\w+)', function_format)]

        # Classes
        class_format = QTextCharFormat()
        class_format.setForeground(QColor(78, 201, 176))  # Cyan
        class_format.setFontWeight(QFont.Bold)
        self.formats['class'] = [(r'\bclass\s+(\w+)', class_format)]

        # Numbers
        number_format = QTextCharFormat()
        number_format.setForeground(QColor(181, 206, 168))  # Light green
        self.formats['number'] = [(r'\b\d+(\.\d+)?\b', number_format)]

    def highlightBlock(self, text):
        """Apply syntax highlighting to a block of text"""
        for category, patterns in self.formats.items():
            for pattern, format in patterns:
                for match in re.finditer(pattern, text):
                    start, end = match.span()
                    self.setFormat(start, end - start, format)


class XMLSyntaxHighlighter(QSyntaxHighlighter):
    """Basic XML syntax highlighter"""

    def __init__(self, document):
        super().__init__(document)

        # Tag format
        self.tag_format = QTextCharFormat()
        self.tag_format.setForeground(QColor(86, 156, 214))  # Blue
        self.tag_format.setFontWeight(QFont.Bold)

        # Attribute format
        self.attr_format = QTextCharFormat()
        self.attr_format.setForeground(QColor(156, 220, 254))  # Light blue

        # Value format
        self.value_format = QTextCharFormat()
        self.value_format.setForeground(QColor(206, 145, 120))  # Orange

        # Comment format
        self.comment_format = QTextCharFormat()
        self.comment_format.setForeground(QColor(106, 153, 85))  # Green

    def highlightBlock(self, text):
        # Comments
        for match in re.finditer(r'<!--.*?-->', text):
            start, end = match.span()
            self.setFormat(start, end - start, self.comment_format)

        # Tags
        for match in re.finditer(r'</?[\w:]+', text):
            start, end = match.span()
            self.setFormat(start, end - start, self.tag_format)

        # Attributes
        for match in re.finditer(r'\b[\w:]+(?==)', text):
            start, end = match.span()
            self.setFormat(start, end - start, self.attr_format)

        # Values
        for match in re.finditer(r'"[^"]*"', text):
            start, end = match.span()
            self.setFormat(start, end - start, self.value_format)


class CodeEditorWidget(QWidget):
    """
    Simple code editor for viewing/editing generated files

    Features (Preview):
    - Syntax highlighting (Python, XML, C++)
    - Line numbers (via font)
    - Save/Save As
    - Find text
    - Basic editing

    Note: This is a preview version. Full QScintilla integration
    with advanced features planned for v1.0.0.
    """

    file_saved = Signal(str)  # Emits file path when saved

    def __init__(self, parent=None):
        super().__init__(parent)

        self.current_file = None
        self.is_modified = False
        self.highlighter = None

        self._init_ui()

    def _init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Toolbar
        toolbar = QToolBar()

        self.file_label = QLabel("No file loaded")
        self.file_label.setStyleSheet("padding: 5px; font-weight: bold;")
        toolbar.addWidget(self.file_label)

        toolbar.addSeparator()

        # File operations
        open_button = QPushButton("Open...")
        open_button.clicked.connect(self.open_file)
        toolbar.addWidget(open_button)

        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save_file)
        self.save_button.setEnabled(False)
        self.save_button.setShortcut(QKeySequence.Save)
        toolbar.addWidget(self.save_button)

        save_as_button = QPushButton("Save As...")
        save_as_button.clicked.connect(self.save_file_as)
        toolbar.addWidget(save_as_button)

        toolbar.addSeparator()

        # Syntax selector
        syntax_label = QLabel("Syntax:")
        toolbar.addWidget(syntax_label)

        self.syntax_combo = QComboBox()
        self.syntax_combo.addItems(["Auto", "Python", "XML", "C++", "Plain Text"])
        self.syntax_combo.currentTextChanged.connect(self._on_syntax_changed)
        toolbar.addWidget(self.syntax_combo)

        layout.addWidget(toolbar)

        # Editor
        self.editor = QTextEdit()
        self.editor.setAcceptRichText(False)
        self.editor.textChanged.connect(self._on_text_changed)

        # Use monospace font
        font = QFont("Consolas", 10)
        if not font.exactMatch():
            font = QFont("Courier New", 10)
        self.editor.setFont(font)

        # Tab width = 4 spaces
        self.editor.setTabStopDistance(self.editor.fontMetrics().horizontalAdvance(' ') * 4)

        # Dark theme
        self.editor.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: 1px solid #3c3c3c;
            }
        """)

        layout.addWidget(self.editor)

        # Status bar
        status_layout = QHBoxLayout()

        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("padding: 5px;")
        status_layout.addWidget(self.status_label, 1)

        self.line_col_label = QLabel("Line: 1, Col: 1")
        self.line_col_label.setStyleSheet("padding: 5px;")
        status_layout.addWidget(self.line_col_label)

        layout.addLayout(status_layout)

        # Connect cursor position
        self.editor.cursorPositionChanged.connect(self._update_cursor_position)

    def open_file(self, file_path: str = None):
        """Open a file in the editor"""
        if not file_path:
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Open File",
                "",
                "All Files (*);;Python Files (*.py);;XML Files (*.xml *.urdf *.xacro);;C++ Files (*.cpp *.h *.hpp)"
            )

        if not file_path:
            return

        try:
            path = Path(file_path)

            if not path.exists():
                QMessageBox.warning(self, "File Not Found", f"File not found:\n{file_path}")
                return

            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()

            self.editor.setPlainText(content)
            self.current_file = str(path)
            self.is_modified = False

            # Update UI
            self.file_label.setText(path.name)
            self.file_label.setToolTip(str(path))
            self.save_button.setEnabled(False)
            self.status_label.setText(f"Loaded: {path.name}")

            # Auto-detect syntax
            self._auto_detect_syntax(path)

        except Exception as e:
            QMessageBox.critical(self, "Error Opening File", f"Failed to open file:\n{str(e)}")

    def save_file(self):
        """Save current file"""
        if not self.current_file:
            self.save_file_as()
            return

        try:
            with open(self.current_file, 'w', encoding='utf-8') as f:
                f.write(self.editor.toPlainText())

            self.is_modified = False
            self.save_button.setEnabled(False)
            self.status_label.setText(f"Saved: {Path(self.current_file).name}")
            self.file_saved.emit(self.current_file)

        except Exception as e:
            QMessageBox.critical(self, "Error Saving File", f"Failed to save file:\n{str(e)}")

    def save_file_as(self):
        """Save file with new name"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save File As",
            self.current_file or "",
            "All Files (*);;Python Files (*.py);;XML Files (*.xml);;C++ Files (*.cpp)"
        )

        if file_path:
            self.current_file = file_path
            self.save_file()

            # Update UI
            path = Path(file_path)
            self.file_label.setText(path.name)
            self.file_label.setToolTip(str(path))

    def _on_text_changed(self):
        """Handle text changes"""
        if self.current_file and not self.is_modified:
            self.is_modified = True
            self.save_button.setEnabled(True)
            self.status_label.setText("Modified")

    def _on_syntax_changed(self, syntax: str):
        """Change syntax highlighting"""
        # Remove old highlighter
        if self.highlighter:
            self.highlighter.setDocument(None)
            self.highlighter = None

        # Apply new highlighter
        if syntax == "Python":
            self.highlighter = PythonSyntaxHighlighter(self.editor.document())
        elif syntax == "XML":
            self.highlighter = XMLSyntaxHighlighter(self.editor.document())
        elif syntax == "Plain Text":
            pass  # No highlighting
        elif syntax == "Auto":
            if self.current_file:
                self._auto_detect_syntax(Path(self.current_file))
        elif syntax == "C++":
            # Basic C++ highlighting (similar to Python)
            self.highlighter = PythonSyntaxHighlighter(self.editor.document())

    def _auto_detect_syntax(self, path: Path):
        """Auto-detect syntax from file extension"""
        ext = path.suffix.lower()

        if ext in ['.py']:
            self.syntax_combo.setCurrentText("Python")
        elif ext in ['.xml', '.urdf', '.xacro', '.launch']:
            self.syntax_combo.setCurrentText("XML")
        elif ext in ['.cpp', '.h', '.hpp', '.c', '.cc']:
            self.syntax_combo.setCurrentText("C++")
        else:
            self.syntax_combo.setCurrentText("Plain Text")

    def _update_cursor_position(self):
        """Update cursor position in status bar"""
        cursor = self.editor.textCursor()
        line = cursor.blockNumber() + 1
        col = cursor.columnNumber() + 1
        self.line_col_label.setText(f"Line: {line}, Col: {col}")

    def load_file_content(self, file_path: str):
        """Load file content (public method)"""
        self.open_file(file_path)

    def get_content(self) -> str:
        """Get current editor content"""
        return self.editor.toPlainText()

    def set_content(self, content: str):
        """Set editor content"""
        self.editor.setPlainText(content)
        self.is_modified = False
        self.save_button.setEnabled(False)


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    editor = CodeEditorWidget()
    editor.file_saved.connect(lambda path: print(f"File saved: {path}"))
    editor.resize(800, 600)
    editor.show()

    # Load sample Python code
    sample_code = '''#!/usr/bin/env python3
"""
ROS2 Node Example
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExampleNode(Node):
    """Example ROS2 Node"""

    def __init__(self):
        super().__init__('example_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Publish message"""
        msg = String()
        msg.data = f'Hello World {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

    editor.set_content(sample_code)
    editor.syntax_combo.setCurrentText("Python")

    sys.exit(app.exec())
