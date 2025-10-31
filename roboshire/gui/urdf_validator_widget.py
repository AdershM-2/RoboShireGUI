"""
URDF Validator Widget

GUI widget for displaying URDF validation results with detailed error messages,
fix suggestions, and export functionality.

Features:
- Tabbed view (Errors/Warnings/Info)
- Color-coded severity
- Detailed issue information
- Fix suggestions
- Click to jump to element
- Export validation report

Author: RoboShire Team
Phase: 6.3 (Advanced URDF Tools)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QTableWidget, QTableWidgetItem, QTextEdit, QTabWidget,
    QHeaderView, QGroupBox, QFileDialog, QMessageBox, QSplitter
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QFont, QBrush
from typing import Optional, List
from pathlib import Path

from roboshire.backend.urdf_validator import (
    URDFValidator, ValidationResult, ValidationIssue, Severity
)


class URDFValidatorWidget(QWidget):
    """
    Widget for displaying URDF validation results

    Signals:
        element_selected: Emitted when user clicks on an issue (element_name)
    """

    element_selected = Signal(str)  # element_name

    def __init__(self, parent=None):
        super().__init__(parent)

        self.validator = URDFValidator()
        self.current_result: Optional[ValidationResult] = None
        self.current_urdf_path: Optional[str] = None

        self._init_ui()

    def _init_ui(self):
        """Initialize the user interface"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Title
        title_label = QLabel("URDF Validation")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        main_layout.addWidget(title_label)

        # Summary panel
        self.summary_group = QGroupBox("Validation Summary")
        summary_layout = QVBoxLayout(self.summary_group)

        self.status_label = QLabel("No validation performed")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        summary_layout.addWidget(self.status_label)

        stats_layout = QHBoxLayout()
        self.errors_label = QLabel("Errors: -")
        self.warnings_label = QLabel("Warnings: -")
        self.info_label = QLabel("Info: -")
        self.time_label = QLabel("Time: -")

        stats_layout.addWidget(self.errors_label)
        stats_layout.addWidget(QLabel("|"))
        stats_layout.addWidget(self.warnings_label)
        stats_layout.addWidget(QLabel("|"))
        stats_layout.addWidget(self.info_label)
        stats_layout.addWidget(QLabel("|"))
        stats_layout.addWidget(self.time_label)
        stats_layout.addStretch()

        summary_layout.addLayout(stats_layout)

        # URDF summary
        self.urdf_summary_label = QLabel("")
        summary_layout.addWidget(self.urdf_summary_label)

        main_layout.addWidget(self.summary_group)

        # Splitter for issues and details
        splitter = QSplitter(Qt.Vertical)

        # Tab widget for issues by severity
        self.tab_widget = QTabWidget()

        # Errors tab
        self.errors_table = self._create_issues_table()
        self.tab_widget.addTab(self.errors_table, "Errors (0)")

        # Warnings tab
        self.warnings_table = self._create_issues_table()
        self.tab_widget.addTab(self.warnings_table, "Warnings (0)")

        # Info tab
        self.info_table = self._create_issues_table()
        self.tab_widget.addTab(self.info_table, "Info (0)")

        # All issues tab
        self.all_table = self._create_issues_table()
        self.tab_widget.addTab(self.all_table, "All (0)")

        splitter.addWidget(self.tab_widget)

        # Details panel
        details_widget = QWidget()
        details_layout = QVBoxLayout(details_widget)
        details_layout.setContentsMargins(0, 0, 0, 0)

        details_label = QLabel("Issue Details")
        details_label.setStyleSheet("font-weight: bold;")
        details_layout.addWidget(details_label)

        self.details_text = QTextEdit()
        self.details_text.setReadOnly(True)
        self.details_text.setMaximumHeight(200)
        details_layout.addWidget(self.details_text)

        splitter.addWidget(details_widget)

        # Set splitter sizes
        splitter.setSizes([400, 200])

        main_layout.addWidget(splitter)

        # Button bar
        button_layout = QHBoxLayout()

        self.validate_btn = QPushButton("Validate URDF")
        self.validate_btn.clicked.connect(self._on_validate_clicked)
        button_layout.addWidget(self.validate_btn)

        self.export_btn = QPushButton("Export Report")
        self.export_btn.setEnabled(False)
        self.export_btn.clicked.connect(self._on_export_report)
        button_layout.addWidget(self.export_btn)

        button_layout.addStretch()

        self.jump_btn = QPushButton("Jump to Element")
        self.jump_btn.setEnabled(False)
        self.jump_btn.clicked.connect(self._on_jump_to_element)
        button_layout.addWidget(self.jump_btn)

        main_layout.addLayout(button_layout)

        # Connect table selection signals
        self.errors_table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.warnings_table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.info_table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.all_table.itemSelectionChanged.connect(self._on_table_selection_changed)

    def _create_issues_table(self) -> QTableWidget:
        """Create a table for displaying issues"""
        table = QTableWidget()
        table.setColumnCount(5)
        table.setHorizontalHeaderLabels(["Severity", "Category", "Element", "Message", "Line"])

        # Set column widths
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)

        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        table.setAlternatingRowColors(True)

        return table

    def validate_urdf(self, urdf_path: str):
        """
        Validate a URDF file and display results

        Args:
            urdf_path: Path to URDF file
        """
        self.current_urdf_path = urdf_path
        self.status_label.setText("Validating...")

        # Run validation
        self.current_result = self.validator.validate(urdf_path)

        # Update display
        self._display_result(self.current_result)

    def _display_result(self, result: ValidationResult):
        """Display validation result"""
        # Update summary
        if result.is_valid:
            self.status_label.setText("✓ VALID - No errors found")
            self.status_label.setStyleSheet("font-weight: bold; font-size: 12px; color: green;")
        else:
            self.status_label.setText("✗ INVALID - Must fix errors")
            self.status_label.setStyleSheet("font-weight: bold; font-size: 12px; color: red;")

        # Update stats
        self.errors_label.setText(f"Errors: {result.error_count}")
        self.errors_label.setStyleSheet("color: red; font-weight: bold;" if result.error_count > 0 else "")

        self.warnings_label.setText(f"Warnings: {result.warning_count}")
        self.warnings_label.setStyleSheet("color: orange; font-weight: bold;" if result.warning_count > 0 else "")

        self.info_label.setText(f"Info: {result.info_count}")
        self.info_label.setStyleSheet("color: blue;" if result.info_count > 0 else "")

        self.time_label.setText(f"Time: {result.validation_time:.3f}s")

        # URDF summary
        if result.urdf_summary:
            summary_text = (
                f"Robot: {result.urdf_summary.get('robot_name', 'N/A')} | "
                f"Links: {result.urdf_summary.get('link_count', 0)} | "
                f"Joints: {result.urdf_summary.get('joint_count', 0)} | "
                f"Total Mass: {result.urdf_summary.get('total_mass', 0):.2f} kg"
            )
            self.urdf_summary_label.setText(summary_text)

        # Populate tables
        self._populate_tables(result.issues)

        # Update tab titles with counts
        self.tab_widget.setTabText(0, f"Errors ({result.error_count})")
        self.tab_widget.setTabText(1, f"Warnings ({result.warning_count})")
        self.tab_widget.setTabText(2, f"Info ({result.info_count})")
        self.tab_widget.setTabText(3, f"All ({len(result.issues)})")

        # Enable export if we have issues
        self.export_btn.setEnabled(len(result.issues) > 0)

        # Switch to appropriate tab
        if result.error_count > 0:
            self.tab_widget.setCurrentIndex(0)  # Errors tab
        elif result.warning_count > 0:
            self.tab_widget.setCurrentIndex(1)  # Warnings tab
        elif result.info_count > 0:
            self.tab_widget.setCurrentIndex(2)  # Info tab

    def _populate_tables(self, issues: List[ValidationIssue]):
        """Populate issue tables"""
        # Clear all tables
        self.errors_table.setRowCount(0)
        self.warnings_table.setRowCount(0)
        self.info_table.setRowCount(0)
        self.all_table.setRowCount(0)

        # Separate issues by severity
        errors = [i for i in issues if i.severity == Severity.ERROR]
        warnings = [i for i in issues if i.severity == Severity.WARNING]
        infos = [i for i in issues if i.severity == Severity.INFO]

        # Populate each table
        self._fill_table(self.errors_table, errors)
        self._fill_table(self.warnings_table, warnings)
        self._fill_table(self.info_table, infos)
        self._fill_table(self.all_table, issues)

    def _fill_table(self, table: QTableWidget, issues: List[ValidationIssue]):
        """Fill a table with issues"""
        table.setRowCount(len(issues))

        for row, issue in enumerate(issues):
            # Severity
            severity_item = QTableWidgetItem(issue.severity.value.upper())
            severity_item.setData(Qt.UserRole, issue)  # Store issue object

            # Color code by severity
            if issue.severity == Severity.ERROR:
                color = QColor("#F44336")  # Red
            elif issue.severity == Severity.WARNING:
                color = QColor("#FF9800")  # Orange
            else:
                color = QColor("#2196F3")  # Blue

            severity_item.setForeground(QBrush(color))
            severity_item.setFlags(severity_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 0, severity_item)

            # Category
            category_item = QTableWidgetItem(issue.category)
            category_item.setFlags(category_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 1, category_item)

            # Element
            element_item = QTableWidgetItem(issue.element)
            element_item.setFlags(element_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 2, element_item)

            # Message
            message_item = QTableWidgetItem(issue.message)
            message_item.setFlags(message_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 3, message_item)

            # Line number
            line_text = str(issue.line_number) if issue.line_number else "-"
            line_item = QTableWidgetItem(line_text)
            line_item.setFlags(line_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 4, line_item)

    def _on_table_selection_changed(self):
        """Handle table selection change"""
        # Get current table
        current_table = self.tab_widget.currentWidget()
        if not isinstance(current_table, QTableWidget):
            return

        selected_items = current_table.selectedItems()
        if not selected_items:
            self.details_text.clear()
            self.jump_btn.setEnabled(False)
            return

        # Get issue from first column of selected row
        row = selected_items[0].row()
        severity_item = current_table.item(row, 0)
        issue = severity_item.data(Qt.UserRole)

        if issue:
            self._display_issue_details(issue)
            self.jump_btn.setEnabled(True)

    def _display_issue_details(self, issue: ValidationIssue):
        """Display detailed information for an issue"""
        details = []

        # Header
        severity_color = {
            Severity.ERROR: "red",
            Severity.WARNING: "orange",
            Severity.INFO: "blue"
        }
        color = severity_color.get(issue.severity, "black")

        details.append(f"<h3 style='color: {color};'>{issue.severity.value.upper()}</h3>")
        details.append(f"<p><b>Rule:</b> {issue.rule_id} - {issue.category}</p>")

        # Element and location
        details.append(f"<p><b>Element:</b> {issue.element}</p>")
        if issue.line_number:
            location = f"Line {issue.line_number}"
            if issue.column:
                location += f", Column {issue.column}"
            details.append(f"<p><b>Location:</b> {location}</p>")

        # Message
        details.append(f"<p><b>Issue:</b> {issue.message}</p>")

        # Fix suggestion
        if issue.fix_suggestion:
            details.append(f"<p><b>Fix Suggestion:</b></p>")
            details.append(f"<p style='background-color: #f0f0f0; padding: 10px; font-family: monospace;'>{issue.fix_suggestion}</p>")

        # Documentation
        if issue.documentation_url:
            details.append(f"<p><b>Documentation:</b> <a href='{issue.documentation_url}'>{issue.documentation_url}</a></p>")

        self.details_text.setHtml("".join(details))

    def _on_validate_clicked(self):
        """Handle validate button click"""
        if not self.current_urdf_path:
            QMessageBox.warning(
                self,
                "No URDF",
                "Please load a URDF file first"
            )
            return

        self.validate_urdf(self.current_urdf_path)

    def _on_jump_to_element(self):
        """Handle jump to element button click"""
        # Get current table
        current_table = self.tab_widget.currentWidget()
        if not isinstance(current_table, QTableWidget):
            return

        selected_items = current_table.selectedItems()
        if not selected_items:
            return

        # Get issue
        row = selected_items[0].row()
        severity_item = current_table.item(row, 0)
        issue = severity_item.data(Qt.UserRole)

        if issue and issue.element:
            self.element_selected.emit(issue.element)

    def _on_export_report(self):
        """Export validation report"""
        if not self.current_result:
            return

        # Get save location
        default_name = "urdf_validation_report.html"
        if self.current_urdf_path:
            urdf_name = Path(self.current_urdf_path).stem
            default_name = f"{urdf_name}_validation.html"

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Validation Report",
            default_name,
            "HTML Files (*.html);;Text Files (*.txt);;All Files (*)"
        )

        if not file_path:
            return

        try:
            # Generate HTML report
            html = self._generate_html_report(self.current_result)

            # Write to file
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(html)

            QMessageBox.information(
                self,
                "Report Exported",
                f"Validation report exported to:\n{file_path}"
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Export Failed",
                f"Failed to export report:\n\n{e}"
            )

    def _generate_html_report(self, result: ValidationResult) -> str:
        """Generate HTML validation report"""
        html = []

        # Header
        html.append("<!DOCTYPE html>")
        html.append("<html>")
        html.append("<head>")
        html.append("<meta charset='utf-8'>")
        html.append("<title>URDF Validation Report</title>")
        html.append("<style>")
        html.append("body { font-family: Arial, sans-serif; margin: 20px; }")
        html.append("h1 { color: #333; }")
        html.append("h2 { color: #666; margin-top: 30px; }")
        html.append(".summary { background-color: #f5f5f5; padding: 15px; border-radius: 5px; }")
        html.append(".valid { color: green; font-weight: bold; }")
        html.append(".invalid { color: red; font-weight: bold; }")
        html.append("table { border-collapse: collapse; width: 100%; margin-top: 20px; }")
        html.append("th { background-color: #4CAF50; color: white; padding: 10px; text-align: left; }")
        html.append("td { border: 1px solid #ddd; padding: 8px; }")
        html.append("tr:nth-child(even) { background-color: #f2f2f2; }")
        html.append(".error { color: #F44336; font-weight: bold; }")
        html.append(".warning { color: #FF9800; font-weight: bold; }")
        html.append(".info { color: #2196F3; }")
        html.append(".fix { background-color: #fffacd; padding: 10px; margin: 10px 0; font-family: monospace; }")
        html.append("</style>")
        html.append("</head>")
        html.append("<body>")

        # Title
        html.append("<h1>URDF Validation Report</h1>")

        # Summary
        html.append("<div class='summary'>")
        status_class = "valid" if result.is_valid else "invalid"
        status_text = "VALID" if result.is_valid else "INVALID"
        html.append(f"<p class='{status_class}'>Status: {status_text}</p>")
        html.append(f"<p>Errors: {result.error_count} | Warnings: {result.warning_count} | Info: {result.info_count}</p>")
        html.append(f"<p>Validation Time: {result.validation_time:.3f}s</p>")

        if result.urdf_summary:
            html.append(f"<p>Robot: {result.urdf_summary.get('robot_name', 'N/A')}</p>")
            html.append(f"<p>Links: {result.urdf_summary.get('link_count', 0)} | Joints: {result.urdf_summary.get('joint_count', 0)}</p>")
            html.append(f"<p>Total Mass: {result.urdf_summary.get('total_mass', 0):.2f} kg</p>")

        html.append("</div>")

        # Issues by severity
        if result.error_count > 0:
            html.append("<h2>Errors</h2>")
            html.append(self._generate_issues_table([i for i in result.issues if i.severity == Severity.ERROR]))

        if result.warning_count > 0:
            html.append("<h2>Warnings</h2>")
            html.append(self._generate_issues_table([i for i in result.issues if i.severity == Severity.WARNING]))

        if result.info_count > 0:
            html.append("<h2>Info</h2>")
            html.append(self._generate_issues_table([i for i in result.issues if i.severity == Severity.INFO]))

        # Footer
        html.append("<p style='margin-top: 50px; color: #999; font-size: 12px;'>")
        html.append("Generated by RoboShire URDF Validator")
        html.append("</p>")

        html.append("</body>")
        html.append("</html>")

        return "\n".join(html)

    def _generate_issues_table(self, issues: List[ValidationIssue]) -> str:
        """Generate HTML table for issues"""
        html = []

        html.append("<table>")
        html.append("<tr>")
        html.append("<th>Severity</th>")
        html.append("<th>Rule</th>")
        html.append("<th>Element</th>")
        html.append("<th>Message</th>")
        html.append("<th>Line</th>")
        html.append("<th>Fix</th>")
        html.append("</tr>")

        for issue in issues:
            severity_class = issue.severity.value
            html.append("<tr>")
            html.append(f"<td class='{severity_class}'>{issue.severity.value.upper()}</td>")
            html.append(f"<td>{issue.rule_id}</td>")
            html.append(f"<td>{issue.element}</td>")
            html.append(f"<td>{issue.message}</td>")
            html.append(f"<td>{issue.line_number if issue.line_number else '-'}</td>")
            html.append(f"<td>{issue.fix_suggestion if issue.fix_suggestion else '-'}</td>")
            html.append("</tr>")

        html.append("</table>")

        return "\n".join(html)

    def clear(self):
        """Clear validation results"""
        self.current_result = None
        self.current_urdf_path = None

        self.status_label.setText("No validation performed")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 12px;")

        self.errors_label.setText("Errors: -")
        self.errors_label.setStyleSheet("")
        self.warnings_label.setText("Warnings: -")
        self.warnings_label.setStyleSheet("")
        self.info_label.setText("Info: -")
        self.info_label.setStyleSheet("")
        self.time_label.setText("Time: -")

        self.urdf_summary_label.setText("")

        self.errors_table.setRowCount(0)
        self.warnings_table.setRowCount(0)
        self.info_table.setRowCount(0)
        self.all_table.setRowCount(0)

        self.details_text.clear()

        self.tab_widget.setTabText(0, "Errors (0)")
        self.tab_widget.setTabText(1, "Warnings (0)")
        self.tab_widget.setTabText(2, "Info (0)")
        self.tab_widget.setTabText(3, "All (0)")

        self.export_btn.setEnabled(False)
        self.jump_btn.setEnabled(False)
