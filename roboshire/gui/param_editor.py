"""
Parameter Editor Widget - GUI for editing ROS2 node parameters

Features:
- Tree view of parameters by node
- Type-aware editors (int, float, string, bool, array)
- Apply changes
- Save/load configurations
- Reset to defaults
"""

import logging
from typing import Optional, Dict, List
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QLabel, QLineEdit, QCheckBox, QDoubleSpinBox,
    QSpinBox, QMessageBox, QFileDialog, QGroupBox
)
from PySide6.QtCore import Qt, Signal

from roboshire.integrations.param_manager import ParameterManager, Parameter


class ParameterEditorWidget(QWidget):
    """
    Widget for editing ROS2 node parameters

    Features:
    - Tree view of parameters
    - Inline editing
    - Apply/revert changes
    - Save/load configs
    """

    # Signals
    parameter_changed = Signal(str, str, object)  # node_name, param_name, value

    def __init__(self, param_manager: Optional[ParameterManager] = None, parent=None):
        """
        Initialize parameter editor widget

        Args:
            param_manager: ParameterManager instance
            parent: Parent widget
        """
        super().__init__(parent)

        self.param_manager = param_manager
        self.modified_params: Dict[str, Dict[str, any]] = {}  # node -> {param: value}

        self._init_ui()

        logging.info("ParameterEditorWidget initialized")

    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()

        # Title
        title_label = QLabel("<h3>Parameter Editor</h3>")
        layout.addWidget(title_label)

        # Control buttons
        control_layout = QHBoxLayout()

        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self._refresh_parameters)
        control_layout.addWidget(refresh_button)

        self.apply_button = QPushButton("Apply Changes")
        self.apply_button.clicked.connect(self._apply_changes)
        self.apply_button.setEnabled(False)
        control_layout.addWidget(self.apply_button)

        self.revert_button = QPushButton("Revert")
        self.revert_button.clicked.connect(self._revert_changes)
        self.revert_button.setEnabled(False)
        control_layout.addWidget(self.revert_button)

        control_layout.addStretch()

        save_button = QPushButton("Save Config")
        save_button.clicked.connect(self._save_config)
        control_layout.addWidget(save_button)

        load_button = QPushButton("Load Config")
        load_button.clicked.connect(self._load_config)
        control_layout.addWidget(load_button)

        layout.addLayout(control_layout)

        # Parameter tree
        self.param_tree = QTreeWidget()
        self.param_tree.setHeaderLabels(["Parameter", "Value", "Type"])
        self.param_tree.setColumnWidth(0, 300)
        self.param_tree.setColumnWidth(1, 200)
        self.param_tree.setAlternatingRowColors(True)
        self.param_tree.itemChanged.connect(self._on_item_changed)
        layout.addWidget(self.param_tree)

        # Status
        self.status_label = QLabel("No parameters loaded")
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    def set_param_manager(self, param_manager: ParameterManager):
        """
        Set the parameter manager instance

        Args:
            param_manager: ParameterManager instance
        """
        self.param_manager = param_manager
        self._refresh_parameters()
        logging.info("ParameterManager attached to ParameterEditorWidget")

    def refresh_parameters(self):
        """Public method to refresh the parameter tree"""
        self._refresh_parameters()

    def _refresh_parameters(self):
        """Refresh the parameter tree"""
        if not self.param_manager:
            return

        try:
            self.param_tree.clear()
            self.modified_params.clear()
            self._update_buttons()

            self.status_label.setText("Loading parameters...")

            # Get all parameters
            all_params = self.param_manager.get_all_params()

            if not all_params:
                self.status_label.setText("No parameters found")
                return

            # Build tree
            for node_name, params in sorted(all_params.items()):
                # Create node item
                node_item = QTreeWidgetItem(self.param_tree)
                node_item.setText(0, node_name)
                node_item.setExpanded(True)
                node_item.setFlags(node_item.flags() & ~Qt.ItemIsEditable)

                # Add parameters
                for param in params:
                    param_item = QTreeWidgetItem(node_item)
                    param_item.setText(0, param.param_name)
                    param_item.setText(1, str(param.value))
                    param_item.setText(2, param.param_type)

                    # Make value editable
                    param_item.setFlags(param_item.flags() | Qt.ItemIsEditable)

                    # Store original value
                    param_item.setData(0, Qt.UserRole, {
                        'node': node_name,
                        'param': param.param_name,
                        'original_value': param.value,
                        'type': param.param_type
                    })

            total_params = sum(len(p) for p in all_params.values())
            self.status_label.setText(f"Loaded {total_params} parameters from {len(all_params)} nodes")

            logging.info(f"Refreshed {total_params} parameters from {len(all_params)} nodes")

        except Exception as e:
            logging.error(f"Error refreshing parameters: {e}")
            self.status_label.setText(f"Error: {e}")
            QMessageBox.critical(self, "Error", f"Failed to refresh parameters:\n\n{e}")

    def _on_item_changed(self, item: QTreeWidgetItem, column: int):
        """
        Handle parameter value change

        Args:
            item: Changed item
            column: Changed column
        """
        if column != 1:  # Only care about value column
            return

        data = item.data(0, Qt.UserRole)
        if not data:
            return

        node_name = data['node']
        param_name = data['param']
        original_value = data['original_value']
        new_value_str = item.text(1)

        # Convert value to appropriate type
        param_type = data['type']
        try:
            new_value = self._convert_value(new_value_str, param_type)

            # Check if actually changed
            if new_value != original_value:
                # Mark as modified
                if node_name not in self.modified_params:
                    self.modified_params[node_name] = {}

                self.modified_params[node_name][param_name] = new_value

                # Highlight changed item
                item.setBackground(1, Qt.yellow)

                self._update_buttons()
                logging.debug(f"Parameter modified: {node_name}::{param_name} = {new_value}")
            else:
                # Remove from modified if reverted to original
                if node_name in self.modified_params:
                    self.modified_params[node_name].pop(param_name, None)
                    if not self.modified_params[node_name]:
                        del self.modified_params[node_name]

                item.setBackground(1, Qt.transparent)
                self._update_buttons()

        except ValueError as e:
            logging.warning(f"Invalid value for {param_type}: {new_value_str}")
            QMessageBox.warning(self, "Invalid Value", f"Invalid value for {param_type}:\n\n{e}")
            # Revert to original
            item.setText(1, str(original_value))

    def _convert_value(self, value_str: str, param_type: str):
        """
        Convert value string to appropriate Python type

        Args:
            value_str: Value string
            param_type: Parameter type

        Returns:
            Converted value

        Raises:
            ValueError: If conversion fails
        """
        value_str = value_str.strip()

        if param_type == 'boolean':
            if value_str.lower() in ('true', '1', 'yes', 'on'):
                return True
            elif value_str.lower() in ('false', '0', 'no', 'off'):
                return False
            else:
                raise ValueError(f"Invalid boolean value: {value_str}")

        elif param_type == 'integer':
            return int(value_str)

        elif param_type in ('double', 'float'):
            return float(value_str)

        elif param_type == 'string':
            return value_str

        elif 'array' in param_type:
            # Try to parse as list
            import ast
            return ast.literal_eval(value_str)

        else:
            return value_str

    def _update_buttons(self):
        """Update button states based on modifications"""
        has_changes = bool(self.modified_params)
        self.apply_button.setEnabled(has_changes)
        self.revert_button.setEnabled(has_changes)

        if has_changes:
            total_changes = sum(len(params) for params in self.modified_params.values())
            self.status_label.setText(f"{total_changes} parameter(s) modified (not applied)")

    def _apply_changes(self):
        """Apply modified parameters to nodes"""
        if not self.param_manager or not self.modified_params:
            return

        try:
            success_count = 0
            fail_count = 0

            for node_name, params in self.modified_params.items():
                for param_name, value in params.items():
                    success = self.param_manager.set_param(node_name, param_name, value)

                    if success:
                        success_count += 1
                        self.parameter_changed.emit(node_name, param_name, value)
                    else:
                        fail_count += 1

            # Show result
            if fail_count == 0:
                QMessageBox.information(
                    self,
                    "Success",
                    f"Applied {success_count} parameter change(s)"
                )

                # Clear modifications and refresh
                self.modified_params.clear()
                self._refresh_parameters()

            else:
                QMessageBox.warning(
                    self,
                    "Partial Success",
                    f"Applied {success_count} parameter(s), {fail_count} failed"
                )

        except Exception as e:
            logging.error(f"Error applying changes: {e}")
            QMessageBox.critical(self, "Error", f"Failed to apply changes:\n\n{e}")

    def _revert_changes(self):
        """Revert all modifications"""
        reply = QMessageBox.question(
            self,
            "Revert Changes",
            "Are you sure you want to revert all changes?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.modified_params.clear()
            self._refresh_parameters()

    def _save_config(self):
        """Save current parameters to a file"""
        if not self.param_manager:
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Parameter Configuration",
            "",
            "YAML Files (*.yaml *.yml);;All Files (*)"
        )

        if not file_path:
            return

        try:
            # Get all current parameters
            all_params = self.param_manager.get_all_params()

            # Convert to config format
            config_data = {}
            for node_name, params in all_params.items():
                config_data[node_name] = {p.param_name: p.value for p in params}

            # Save to file
            success = self.param_manager.save_config(config_data, file_path)

            if success:
                QMessageBox.information(
                    self,
                    "Success",
                    f"Saved parameter configuration to:\n{file_path}"
                )
            else:
                QMessageBox.warning(self, "Error", "Failed to save configuration")

        except Exception as e:
            logging.error(f"Error saving config: {e}")
            QMessageBox.critical(self, "Error", f"Failed to save config:\n\n{e}")

    def _load_config(self):
        """Load parameters from a file"""
        if not self.param_manager:
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Parameter Configuration",
            "",
            "YAML Files (*.yaml *.yml);;All Files (*)"
        )

        if not file_path:
            return

        try:
            # Load config
            config_data = self.param_manager.load_config(file_path)

            if not config_data:
                QMessageBox.warning(self, "Error", "Failed to load configuration file")
                return

            # Apply parameters
            success_count = 0
            fail_count = 0

            for node_name, params in config_data.items():
                for param_name, value in params.items():
                    success = self.param_manager.set_param(node_name, param_name, value)
                    if success:
                        success_count += 1
                    else:
                        fail_count += 1

            # Show result
            if fail_count == 0:
                QMessageBox.information(
                    self,
                    "Success",
                    f"Loaded and applied {success_count} parameter(s)"
                )
            else:
                QMessageBox.warning(
                    self,
                    "Partial Success",
                    f"Applied {success_count} parameter(s), {fail_count} failed"
                )

            # Refresh display
            self._refresh_parameters()

        except Exception as e:
            logging.error(f"Error loading config: {e}")
            QMessageBox.critical(self, "Error", f"Failed to load config:\n\n{e}")
