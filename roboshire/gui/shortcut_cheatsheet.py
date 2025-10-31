"""
Keyboard Shortcut Cheat Sheet Dialog

Displays a comprehensive list of all keyboard shortcuts in RoboShire.
Can be opened with Ctrl+Shift+K or from Help menu.

Author: RoboShire Team
Phase: 7 (Polish & UX)
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
    QTextBrowser, QLabel, QTabWidget, QWidget, QTableWidget,
    QTableWidgetItem, QHeaderView
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont, QKeySequence

from roboshire.gui.keyboard_shortcuts import KeyboardShortcuts, ShortcutCategory


class ShortcutCheatSheetDialog(QDialog):
    """
    Dialog displaying all keyboard shortcuts

    Features:
    - Organized by category (tabs)
    - Searchable table view
    - HTML rendered version
    - Print capability
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Keyboard Shortcuts - RoboShire")
        self.setMinimumSize(900, 600)
        self.resize(1000, 700)

        self._init_ui()

    def _init_ui(self):
        """Initialize the user interface"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)

        # Title
        title_label = QLabel("Keyboard Shortcuts Reference")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Subtitle
        subtitle = QLabel("Press Ctrl+Shift+K to toggle this cheat sheet anytime")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet("color: #666; margin-bottom: 15px;")
        layout.addWidget(subtitle)

        # Tab widget for different views
        self.tab_widget = QTabWidget()

        # View 1: Table view (organized by category)
        self.table_view = self._create_table_view()
        self.tab_widget.addTab(self.table_view, "Table View")

        # View 2: HTML view (formatted cheat sheet)
        self.html_view = self._create_html_view()
        self.tab_widget.addTab(self.html_view, "Formatted View")

        layout.addWidget(self.tab_widget)

        # Buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()

        print_btn = QPushButton("Print")
        print_btn.clicked.connect(self._on_print)
        button_layout.addWidget(print_btn)

        export_btn = QPushButton("Export to PDF")
        export_btn.clicked.connect(self._on_export_pdf)
        button_layout.addWidget(export_btn)

        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        close_btn.setDefault(True)
        button_layout.addWidget(close_btn)

        layout.addLayout(button_layout)

        # Check for conflicts
        conflicts = KeyboardShortcuts.detect_conflicts()
        if conflicts:
            self._show_conflicts_warning(conflicts)

    def _create_table_view(self) -> QWidget:
        """Create table view of shortcuts"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(10, 10, 10, 10)

        # Get all shortcuts
        shortcuts = KeyboardShortcuts.get_all_shortcuts()

        # Create tabs for each category
        category_tabs = QTabWidget()

        for category, actions in shortcuts.items():
            # Create table for this category
            table = QTableWidget()
            table.setColumnCount(3)
            table.setHorizontalHeaderLabels(["Action", "Shortcut", "Description"])
            table.setRowCount(len(actions))

            # Populate table
            for row, (name, shortcut, description) in enumerate(actions):
                # Action name
                name_item = QTableWidgetItem(name)
                name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
                table.setItem(row, 0, name_item)

                # Shortcut
                shortcut_item = QTableWidgetItem(shortcut)
                shortcut_item.setFlags(shortcut_item.flags() & ~Qt.ItemIsEditable)
                shortcut_font = QFont("Courier New", 10, QFont.Bold)
                shortcut_item.setFont(shortcut_font)
                shortcut_item.setForeground(Qt.blue)
                table.setItem(row, 1, shortcut_item)

                # Description
                desc_item = QTableWidgetItem(description)
                desc_item.setFlags(desc_item.flags() & ~Qt.ItemIsEditable)
                table.setItem(row, 2, desc_item)

            # Configure table
            table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
            table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
            table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
            table.verticalHeader().setVisible(False)
            table.setAlternatingRowColors(True)
            table.setSelectionBehavior(QTableWidget.SelectRows)

            # Add to category tabs
            category_tabs.addTab(table, category)

        layout.addWidget(category_tabs)
        return widget

    def _create_html_view(self) -> QWidget:
        """Create HTML-formatted view of shortcuts"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Text browser for HTML
        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)

        # Generate and display HTML
        html_content = KeyboardShortcuts.generate_cheatsheet_html()
        browser.setHtml(html_content)

        layout.addWidget(browser)
        return widget

    def _show_conflicts_warning(self, conflicts):
        """Show warning if shortcut conflicts detected"""
        warning_text = "⚠️ Warning: Shortcut conflicts detected:\n\n"

        for shortcut, categories, actions in conflicts:
            warning_text += f"• {shortcut}: {', '.join(actions)}\n"

        warning_label = QLabel(warning_text)
        warning_label.setStyleSheet("""
            QLabel {
                background-color: #fff3cd;
                color: #856404;
                padding: 10px;
                border: 1px solid #ffc107;
                border-radius: 4px;
                margin: 10px 0;
            }
        """)
        warning_label.setWordWrap(True)

        # Insert at top of layout
        self.layout().insertWidget(2, warning_label)

    def _on_print(self):
        """Print the cheat sheet"""
        from PySide6.QtPrintSupport import QPrinter, QPrintDialog

        # Get current view
        if self.tab_widget.currentIndex() == 1:  # HTML view
            html_widget = self.html_view.findChild(QTextBrowser)
            if html_widget:
                printer = QPrinter(QPrinter.HighResolution)
                dialog = QPrintDialog(printer, self)

                if dialog.exec() == QPrintDialog.Accepted:
                    html_widget.print(printer)

    def _on_export_pdf(self):
        """Export cheat sheet to PDF"""
        from PySide6.QtWidgets import QFileDialog
        from PySide6.QtPrintSupport import QPrinter

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Shortcuts to PDF",
            "roboshire_shortcuts.pdf",
            "PDF Files (*.pdf)"
        )

        if file_path:
            if not file_path.endswith('.pdf'):
                file_path += '.pdf'

            # Get HTML view
            html_widget = self.html_view.findChild(QTextBrowser)
            if html_widget:
                printer = QPrinter(QPrinter.HighResolution)
                printer.setOutputFormat(QPrinter.PdfFormat)
                printer.setOutputFileName(file_path)
                printer.setPageSize(QPrinter.A4)

                html_widget.print(printer)

                from PySide6.QtWidgets import QMessageBox
                QMessageBox.information(
                    self,
                    "Export Complete",
                    f"Shortcut reference exported to:\n{file_path}"
                )


def show_shortcut_cheatsheet(parent=None):
    """
    Convenience function to show shortcut cheat sheet

    Args:
        parent: Parent widget (optional)
    """
    dialog = ShortcutCheatSheetDialog(parent)
    dialog.exec()
