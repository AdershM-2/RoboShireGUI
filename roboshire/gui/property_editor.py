"""
Property Editor - Widget for editing URDF link/joint properties
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QLabel,
    QLineEdit, QDoubleSpinBox, QGroupBox, QPushButton,
    QScrollArea, QMessageBox
)
from PySide6.QtCore import Qt, Signal
from typing import Optional, Dict, Any
import logging


class PropertyEditor(QWidget):
    """
    Property editor for URDF components (links and joints).

    Displays editable properties in form layout.
    """

    # Signals
    property_changed = Signal(str, str, str, object)  # (item_type, item_name, property_name, value)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.urdf_manager = None
        self.current_item_type: Optional[str] = None
        self.current_item_name: Optional[str] = None

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Title label
        self.title_label = QLabel("No Selection")
        self.title_label.setStyleSheet("QLabel { font-weight: bold; font-size: 14px; }")
        layout.addWidget(self.title_label)

        # Scroll area for properties
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Container widget for scroll area
        self.container = QWidget()
        self.container_layout = QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)

        scroll.setWidget(self.container)
        layout.addWidget(scroll)

        # Placeholder label
        self.placeholder_label = QLabel("Select a link or joint to view properties")
        self.placeholder_label.setAlignment(Qt.AlignCenter)
        self.placeholder_label.setStyleSheet("QLabel { color: gray; }")
        self.container_layout.addWidget(self.placeholder_label)
        self.container_layout.addStretch()

    def set_urdf_manager(self, urdf_manager):
        """Set URDF manager"""
        self.urdf_manager = urdf_manager

    def clear(self):
        """Clear property editor"""
        # Remove all widgets from container
        while self.container_layout.count():
            item = self.container_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        # Add placeholder back
        self.placeholder_label = QLabel("Select a link or joint to view properties")
        self.placeholder_label.setAlignment(Qt.AlignCenter)
        self.placeholder_label.setStyleSheet("QLabel { color: gray; }")
        self.container_layout.addWidget(self.placeholder_label)
        self.container_layout.addStretch()

        self.title_label.setText("No Selection")
        self.current_item_type = None
        self.current_item_name = None

    def show_properties(self, item_type: str, item_name: str):
        """
        Show properties for selected item

        Args:
            item_type: 'link' or 'joint'
            item_name: Name of the item
        """
        if self.urdf_manager is None or not self.urdf_manager.is_loaded():
            self.clear()
            return

        self.current_item_type = item_type
        self.current_item_name = item_name

        # Clear existing widgets
        while self.container_layout.count():
            item = self.container_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        # Update title
        self.title_label.setText(f"{item_type.title()}: {item_name}")

        # Show appropriate properties
        if item_type == 'link':
            self._show_link_properties(item_name)
        elif item_type == 'joint':
            self._show_joint_properties(item_name)

        self.container_layout.addStretch()

    def _show_link_properties(self, link_name: str):
        """Show properties for a link"""
        info = self.urdf_manager.get_link_info(link_name)

        if not info:
            error_label = QLabel(f"Failed to load link info for: {link_name}")
            error_label.setStyleSheet("QLabel { color: red; }")
            self.container_layout.addWidget(error_label)
            return

        # Basic info group
        basic_group = QGroupBox("Basic Information")
        basic_layout = QFormLayout()

        name_edit = QLineEdit(info.get('name', ''))
        name_edit.setReadOnly(True)
        basic_layout.addRow("Name:", name_edit)

        has_visual = QLineEdit("Yes" if info.get('has_visual') else "No")
        has_visual.setReadOnly(True)
        basic_layout.addRow("Visual:", has_visual)

        has_collision = QLineEdit("Yes" if info.get('has_collision') else "No")
        has_collision.setReadOnly(True)
        basic_layout.addRow("Collision:", has_collision)

        has_inertial = QLineEdit("Yes" if info.get('has_inertial') else "No")
        has_inertial.setReadOnly(True)
        basic_layout.addRow("Inertial:", has_inertial)

        basic_group.setLayout(basic_layout)
        self.container_layout.addWidget(basic_group)

        # Inertial properties group (if available)
        if info.get('has_inertial'):
            inertial_group = QGroupBox("Inertial Properties")
            inertial_layout = QFormLayout()

            mass = info.get('mass', 0.0)
            mass_spin = QDoubleSpinBox()
            mass_spin.setRange(0.0, 10000.0)
            mass_spin.setDecimals(6)
            mass_spin.setValue(mass)
            mass_spin.setSuffix(" kg")
            mass_spin.setReadOnly(True)  # TODO: Make editable when save is implemented
            inertial_layout.addRow("Mass:", mass_spin)

            inertial_group.setLayout(inertial_layout)
            self.container_layout.addWidget(inertial_group)

        # Info label
        info_label = QLabel("Note: Full link editing coming soon")
        info_label.setStyleSheet("QLabel { color: gray; font-style: italic; }")
        self.container_layout.addWidget(info_label)

    def _show_joint_properties(self, joint_name: str):
        """Show properties for a joint"""
        info = self.urdf_manager.get_joint_info(joint_name)

        if not info:
            error_label = QLabel(f"Failed to load joint info for: {joint_name}")
            error_label.setStyleSheet("QLabel { color: red; }")
            self.container_layout.addWidget(error_label)
            return

        # Basic info group
        basic_group = QGroupBox("Basic Information")
        basic_layout = QFormLayout()

        name_edit = QLineEdit(info.get('name', ''))
        name_edit.setReadOnly(True)
        basic_layout.addRow("Name:", name_edit)

        type_edit = QLineEdit(info.get('type', 'unknown'))
        type_edit.setReadOnly(True)
        basic_layout.addRow("Type:", type_edit)

        parent_edit = QLineEdit(info.get('parent', ''))
        parent_edit.setReadOnly(True)
        basic_layout.addRow("Parent Link:", parent_edit)

        child_edit = QLineEdit(info.get('child', ''))
        child_edit.setReadOnly(True)
        basic_layout.addRow("Child Link:", child_edit)

        basic_group.setLayout(basic_layout)
        self.container_layout.addWidget(basic_group)

        # Joint limits (if available)
        if 'lower_limit' in info:
            limits_group = QGroupBox("Joint Limits")
            limits_layout = QFormLayout()

            # Lower limit
            self.lower_limit_spin = QDoubleSpinBox()
            self.lower_limit_spin.setRange(-1000.0, 1000.0)
            self.lower_limit_spin.setDecimals(6)
            self.lower_limit_spin.setValue(info.get('lower_limit', 0.0))
            limits_layout.addRow("Lower Limit:", self.lower_limit_spin)

            # Upper limit
            self.upper_limit_spin = QDoubleSpinBox()
            self.upper_limit_spin.setRange(-1000.0, 1000.0)
            self.upper_limit_spin.setDecimals(6)
            self.upper_limit_spin.setValue(info.get('upper_limit', 0.0))
            limits_layout.addRow("Upper Limit:", self.upper_limit_spin)

            # Effort
            effort_spin = QDoubleSpinBox()
            effort_spin.setRange(0.0, 10000.0)
            effort_spin.setDecimals(3)
            effort_spin.setValue(info.get('effort', 0.0))
            effort_spin.setReadOnly(True)  # TODO: Make editable
            limits_layout.addRow("Effort:", effort_spin)

            # Velocity
            velocity_spin = QDoubleSpinBox()
            velocity_spin.setRange(0.0, 1000.0)
            velocity_spin.setDecimals(3)
            velocity_spin.setValue(info.get('velocity', 0.0))
            velocity_spin.setReadOnly(True)  # TODO: Make editable
            limits_layout.addRow("Velocity:", velocity_spin)

            limits_group.setLayout(limits_layout)
            self.container_layout.addWidget(limits_group)

            # Apply button for limits
            apply_btn = QPushButton("Apply Joint Limits")
            apply_btn.clicked.connect(self._apply_joint_limits)
            self.container_layout.addWidget(apply_btn)

        # Info label
        info_label = QLabel("Note: Full joint editing coming soon")
        info_label.setStyleSheet("QLabel { color: gray; font-style: italic; }")
        self.container_layout.addWidget(info_label)

    def _apply_joint_limits(self):
        """Apply modified joint limits"""
        if self.urdf_manager is None or self.current_item_name is None:
            return

        try:
            lower = self.lower_limit_spin.value()
            upper = self.upper_limit_spin.value()

            if lower > upper:
                QMessageBox.warning(
                    self,
                    "Invalid Limits",
                    "Lower limit cannot be greater than upper limit"
                )
                return

            success = self.urdf_manager.update_joint_limits(
                self.current_item_name,
                lower,
                upper
            )

            if success:
                QMessageBox.information(
                    self,
                    "Success",
                    f"Updated joint limits for {self.current_item_name}"
                )
                self.property_changed.emit(
                    'joint',
                    self.current_item_name,
                    'limits',
                    {'lower': lower, 'upper': upper}
                )
            else:
                QMessageBox.warning(
                    self,
                    "Failed",
                    f"Failed to update joint limits for {self.current_item_name}"
                )

        except Exception as e:
            logging.error(f"Error applying joint limits: {e}")
            QMessageBox.critical(
                self,
                "Error",
                f"Error applying joint limits: {e}"
            )
