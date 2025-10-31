"""
Launch File Visual Editor - Create ROS2 launch files visually

Provides a visual interface for creating and editing ROS2 launch files.

Author: RoboShire Team
Version: 1.0.0
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QLabel, QComboBox, QLineEdit, QTextEdit, QSplitter,
    QGroupBox, QFormLayout, QSpinBox, QCheckBox, QMessageBox, QFileDialog
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from pathlib import Path
from typing import List, Dict, Optional
import logging


class LaunchFileEditor(QWidget):
    """
    Visual editor for ROS2 launch files

    Features:
    - Add nodes with parameters
    - Add include statements
    - Add arguments and substitutions
    - Live Python/XML preview
    - Save/Load launch files
    """

    launch_file_saved = Signal(str)  # Emits file path

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)
        self.current_file = None
        self.launch_items = []  # List of launch items (nodes, includes, etc.)

        self._init_ui()

    def _init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout(self)

        # Toolbar
        toolbar = self._create_toolbar()
        layout.addLayout(toolbar)

        # Main splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left: Launch items tree
        left_panel = self._create_items_panel()
        splitter.addWidget(left_panel)

        # Right: Item editor and preview
        right_panel = self._create_editor_panel()
        splitter.addWidget(right_panel)

        splitter.setSizes([400, 600])
        layout.addWidget(splitter)

    def _create_toolbar(self) -> QHBoxLayout:
        """Create toolbar with file operations"""
        toolbar = QHBoxLayout()

        self.new_button = QPushButton("New")
        self.new_button.clicked.connect(self._on_new)
        toolbar.addWidget(self.new_button)

        self.open_button = QPushButton("Open...")
        self.open_button.clicked.connect(self._on_open)
        toolbar.addWidget(self.open_button)

        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self._on_save)
        toolbar.addWidget(self.save_button)

        self.save_as_button = QPushButton("Save As...")
        self.save_as_button.clicked.connect(self._on_save_as)
        toolbar.addWidget(self.save_as_button)

        toolbar.addStretch()

        # Format selector
        toolbar.addWidget(QLabel("Format:"))
        self.format_combo = QComboBox()
        self.format_combo.addItems(["Python", "XML"])
        self.format_combo.currentTextChanged.connect(self._update_preview)
        toolbar.addWidget(self.format_combo)

        return toolbar

    def _create_items_panel(self) -> QWidget:
        """Create left panel with launch items tree"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Add buttons
        add_layout = QHBoxLayout()

        self.add_node_button = QPushButton("+ Node")
        self.add_node_button.clicked.connect(self._on_add_node)
        add_layout.addWidget(self.add_node_button)

        self.add_include_button = QPushButton("+ Include")
        self.add_include_button.clicked.connect(self._on_add_include)
        add_layout.addWidget(self.add_include_button)

        self.add_arg_button = QPushButton("+ Argument")
        self.add_arg_button.clicked.connect(self._on_add_argument)
        add_layout.addWidget(self.add_arg_button)

        layout.addLayout(add_layout)

        # Tree widget for launch items
        self.items_tree = QTreeWidget()
        self.items_tree.setHeaderLabel("Launch Items")
        self.items_tree.itemClicked.connect(self._on_item_selected)
        layout.addWidget(self.items_tree)

        # Remove button
        self.remove_button = QPushButton("Remove Selected")
        self.remove_button.clicked.connect(self._on_remove_item)
        layout.addWidget(self.remove_button)

        return panel

    def _create_editor_panel(self) -> QWidget:
        """Create right panel with item editor and preview"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Item editor
        self.editor_group = QGroupBox("Item Editor")
        self.editor_layout = QVBoxLayout(self.editor_group)

        self.editor_label = QLabel("Select an item to edit")
        self.editor_label.setAlignment(Qt.AlignCenter)
        self.editor_label.setStyleSheet("color: #666; font-size: 12px;")
        self.editor_layout.addWidget(self.editor_label)

        layout.addWidget(self.editor_group)

        # Preview
        preview_group = QGroupBox("Launch File Preview")
        preview_layout = QVBoxLayout(preview_group)

        self.preview_text = QTextEdit()
        self.preview_text.setReadOnly(True)
        font = QFont("Consolas", 9)
        self.preview_text.setFont(font)
        preview_layout.addWidget(self.preview_text)

        layout.addWidget(preview_group)

        return panel

    def _on_new(self):
        """Create new launch file"""
        if self.launch_items:
            reply = QMessageBox.question(
                self,
                "New Launch File",
                "Discard current launch file?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if reply == QMessageBox.No:
                return

        self.launch_items = []
        self.current_file = None
        self.items_tree.clear()
        self._update_preview()

    def _on_open(self):
        """Open existing launch file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Launch File",
            "",
            "Launch Files (*.launch.py *.launch.xml);;All Files (*)"
        )

        if file_path:
            # TODO: Parse launch file and populate tree
            self.current_file = file_path
            QMessageBox.information(
                self,
                "Not Implemented",
                "Launch file parsing is not yet implemented.\n"
                "Use this editor to create new launch files for now."
            )

    def _on_save(self):
        """Save current launch file"""
        if not self.current_file:
            self._on_save_as()
        else:
            self._save_to_file(self.current_file)

    def _on_save_as(self):
        """Save current launch file with new name"""
        format_ext = ".launch.py" if self.format_combo.currentText() == "Python" else ".launch.xml"

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Launch File",
            f"robot{format_ext}",
            "Launch Files (*.launch.py *.launch.xml);;All Files (*)"
        )

        if file_path:
            self.current_file = file_path
            self._save_to_file(file_path)

    def _save_to_file(self, file_path: str):
        """Save launch file to disk"""
        try:
            content = self._generate_launch_file()

            with open(file_path, 'w') as f:
                f.write(content)

            self.logger.info(f"Saved launch file: {file_path}")
            self.launch_file_saved.emit(file_path)
            QMessageBox.information(
                self,
                "Success",
                f"Launch file saved:\n{file_path}"
            )

        except Exception as e:
            self.logger.error(f"Failed to save launch file: {e}")
            QMessageBox.critical(
                self,
                "Save Error",
                f"Failed to save launch file:\n{e}"
            )

    def _on_add_node(self):
        """Add a new node to launch file"""
        item = QTreeWidgetItem(self.items_tree)
        item.setText(0, "Node: my_node")
        item.setData(0, Qt.UserRole, {
            'type': 'node',
            'package': 'my_package',
            'executable': 'my_node',
            'name': 'my_node',
            'namespace': '',
            'parameters': {},
            'remappings': {},
            'arguments': []
        })
        self.launch_items.append(item.data(0, Qt.UserRole))
        self._update_preview()

    def _on_add_include(self):
        """Add an include statement"""
        item = QTreeWidgetItem(self.items_tree)
        item.setText(0, "Include: other.launch.py")
        item.setData(0, Qt.UserRole, {
            'type': 'include',
            'package': 'my_package',
            'file': 'other.launch.py',
            'launch_arguments': {}
        })
        self.launch_items.append(item.data(0, Qt.UserRole))
        self._update_preview()

    def _on_add_argument(self):
        """Add a launch argument"""
        item = QTreeWidgetItem(self.items_tree)
        item.setText(0, "Argument: my_arg")
        item.setData(0, Qt.UserRole, {
            'type': 'argument',
            'name': 'my_arg',
            'default': '',
            'description': ''
        })
        self.launch_items.append(item.data(0, Qt.UserRole))
        self._update_preview()

    def _on_remove_item(self):
        """Remove selected item"""
        selected = self.items_tree.selectedItems()
        if not selected:
            return

        item = selected[0]
        item_data = item.data(0, Qt.UserRole)

        # Remove from items list
        if item_data in self.launch_items:
            self.launch_items.remove(item_data)

        # Remove from tree
        index = self.items_tree.indexOfTopLevelItem(item)
        self.items_tree.takeTopLevelItem(index)

        self._update_preview()

    def _on_item_selected(self, item: QTreeWidgetItem):
        """Handle item selection"""
        item_data = item.data(0, Qt.UserRole)

        # Clear previous editor
        for i in reversed(range(self.editor_layout.count())):
            self.editor_layout.itemAt(i).widget().setParent(None)

        # Create editor for selected item type
        if item_data['type'] == 'node':
            self._create_node_editor(item, item_data)
        elif item_data['type'] == 'include':
            self._create_include_editor(item, item_data)
        elif item_data['type'] == 'argument':
            self._create_argument_editor(item, item_data)

    def _create_node_editor(self, item: QTreeWidgetItem, data: dict):
        """Create editor for node item"""
        form = QFormLayout()

        package_edit = QLineEdit(data.get('package', ''))
        package_edit.textChanged.connect(lambda text: self._update_item_data(item, 'package', text))
        form.addRow("Package:", package_edit)

        executable_edit = QLineEdit(data.get('executable', ''))
        executable_edit.textChanged.connect(lambda text: self._update_item_data(item, 'executable', text))
        form.addRow("Executable:", executable_edit)

        name_edit = QLineEdit(data.get('name', ''))
        name_edit.textChanged.connect(lambda text: self._update_item_data(item, 'name', text))
        form.addRow("Name:", name_edit)

        namespace_edit = QLineEdit(data.get('namespace', ''))
        namespace_edit.textChanged.connect(lambda text: self._update_item_data(item, 'namespace', text))
        form.addRow("Namespace:", namespace_edit)

        self.editor_layout.addLayout(form)

    def _create_include_editor(self, item: QTreeWidgetItem, data: dict):
        """Create editor for include item"""
        form = QFormLayout()

        package_edit = QLineEdit(data.get('package', ''))
        package_edit.textChanged.connect(lambda text: self._update_item_data(item, 'package', text))
        form.addRow("Package:", package_edit)

        file_edit = QLineEdit(data.get('file', ''))
        file_edit.textChanged.connect(lambda text: self._update_item_data(item, 'file', text))
        form.addRow("File:", file_edit)

        self.editor_layout.addLayout(form)

    def _create_argument_editor(self, item: QTreeWidgetItem, data: dict):
        """Create editor for argument item"""
        form = QFormLayout()

        name_edit = QLineEdit(data.get('name', ''))
        name_edit.textChanged.connect(lambda text: self._update_item_data(item, 'name', text))
        form.addRow("Name:", name_edit)

        default_edit = QLineEdit(data.get('default', ''))
        default_edit.textChanged.connect(lambda text: self._update_item_data(item, 'default', text))
        form.addRow("Default:", default_edit)

        desc_edit = QLineEdit(data.get('description', ''))
        desc_edit.textChanged.connect(lambda text: self._update_item_data(item, 'description', text))
        form.addRow("Description:", desc_edit)

        self.editor_layout.addLayout(form)

    def _update_item_data(self, item: QTreeWidgetItem, key: str, value: str):
        """Update item data and refresh"""
        data = item.data(0, Qt.UserRole)
        data[key] = value
        item.setData(0, Qt.UserRole, data)

        # Update item text
        if data['type'] == 'node':
            item.setText(0, f"Node: {data.get('name', 'unnamed')}")
        elif data['type'] == 'include':
            item.setText(0, f"Include: {data.get('file', 'unnamed')}")
        elif data['type'] == 'argument':
            item.setText(0, f"Argument: {data.get('name', 'unnamed')}")

        self._update_preview()

    def _update_preview(self):
        """Update launch file preview"""
        content = self._generate_launch_file()
        self.preview_text.setPlainText(content)

    def _generate_launch_file(self) -> str:
        """Generate launch file content"""
        if self.format_combo.currentText() == "Python":
            return self._generate_python_launch()
        else:
            return self._generate_xml_launch()

    def _generate_python_launch(self) -> str:
        """Generate Python launch file"""
        lines = []
        lines.append("from launch import LaunchDescription")
        lines.append("from launch_ros.actions import Node")
        lines.append("from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription")
        lines.append("from launch.substitutions import LaunchConfiguration")
        lines.append("from launch_ros.substitutions import FindPackageShare")
        lines.append("from launch.launch_description_sources import PythonLaunchDescriptionSource")
        lines.append("import os")
        lines.append("")
        lines.append("")
        lines.append("def generate_launch_description():")
        lines.append("    \"\"\"Generate launch description\"\"\"")
        lines.append("")

        # Arguments
        for item_data in self.launch_items:
            if item_data['type'] == 'argument':
                lines.append(f"    # Argument: {item_data['name']}")
                lines.append(f"    {item_data['name']}_arg = DeclareLaunchArgument(")
                lines.append(f"        '{item_data['name']}',")
                if item_data.get('default'):
                    lines.append(f"        default_value='{item_data['default']}',")
                if item_data.get('description'):
                    lines.append(f"        description='{item_data['description']}'")
                lines.append("    )")
                lines.append("")

        # Nodes and Includes
        for item_data in self.launch_items:
            if item_data['type'] == 'node':
                lines.append(f"    # Node: {item_data['name']}")
                lines.append(f"    {item_data['name']}_node = Node(")
                lines.append(f"        package='{item_data['package']}',")
                lines.append(f"        executable='{item_data['executable']}',")
                lines.append(f"        name='{item_data['name']}'")
                if item_data.get('namespace'):
                    lines.append(f"        namespace='{item_data['namespace']}',")
                lines.append("    )")
                lines.append("")
            elif item_data['type'] == 'include':
                lines.append(f"    # Include: {item_data['file']}")
                lines.append(f"    include_{item_data['file'].replace('.', '_')} = IncludeLaunchDescription(")
                lines.append(f"        PythonLaunchDescriptionSource([")
                lines.append(f"            FindPackageShare('{item_data['package']}'),")
                lines.append(f"            '/launch/{item_data['file']}'")
                lines.append("        ])")
                lines.append("    )")
                lines.append("")

        # Return LaunchDescription
        lines.append("    return LaunchDescription([")
        for item_data in self.launch_items:
            if item_data['type'] == 'argument':
                lines.append(f"        {item_data['name']}_arg,")
            elif item_data['type'] == 'node':
                lines.append(f"        {item_data['name']}_node,")
            elif item_data['type'] == 'include':
                lines.append(f"        include_{item_data['file'].replace('.', '_')},")
        lines.append("    ])")
        lines.append("")

        return "\n".join(lines)

    def _generate_xml_launch(self) -> str:
        """Generate XML launch file"""
        lines = []
        lines.append('<?xml version="1.0"?>')
        lines.append('<launch>')
        lines.append('')

        # Arguments
        for item_data in self.launch_items:
            if item_data['type'] == 'argument':
                line = f'  <arg name="{item_data["name"]}"'
                if item_data.get('default'):
                    line += f' default="{item_data["default"]}"'
                if item_data.get('description'):
                    line += f' description="{item_data["description"]}"'
                line += ' />'
                lines.append(line)

        if any(item['type'] == 'argument' for item in self.launch_items):
            lines.append('')

        # Nodes and Includes
        for item_data in self.launch_items:
            if item_data['type'] == 'node':
                lines.append(f'  <!-- Node: {item_data["name"]} -->')
                line = f'  <node pkg="{item_data["package"]}" exec="{item_data["executable"]}" name="{item_data["name"]}"'
                if item_data.get('namespace'):
                    line += f' namespace="{item_data["namespace"]}"'
                line += ' />'
                lines.append(line)
                lines.append('')
            elif item_data['type'] == 'include':
                lines.append(f'  <!-- Include: {item_data["file"]} -->')
                lines.append(f'  <include file="$(find-pkg-share {item_data["package"]})/launch/{item_data["file"]}" />')
                lines.append('')

        lines.append('</launch>')
        lines.append('')

        return "\n".join(lines)


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    editor = LaunchFileEditor()
    editor.setWindowTitle("Launch File Editor")
    editor.resize(1000, 700)
    editor.show()

    sys.exit(app.exec())
