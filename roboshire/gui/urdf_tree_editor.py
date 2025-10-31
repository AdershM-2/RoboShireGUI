"""
URDF Tree Editor - Visual tree widget for URDF structure
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QHBoxLayout, QMenu, QMessageBox
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QIcon
from typing import Optional, Dict, Any
import logging


class URDFTreeEditor(QWidget):
    """
    Visual tree editor for URDF robot structure.

    Displays links and joints in a hierarchical tree view.
    Allows selection and basic editing operations.
    """

    # Signals
    item_selected = Signal(str, str)  # (type, name) - 'link' or 'joint'
    item_double_clicked = Signal(str, str)  # (type, name)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.urdf_manager = None
        self._setup_ui()

    def _setup_ui(self):
        """Setup UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Tree widget
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Component", "Type", "Details"])
        self.tree.setColumnWidth(0, 200)
        self.tree.setColumnWidth(1, 80)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.customContextMenuRequested.connect(self._show_context_menu)
        self.tree.itemSelectionChanged.connect(self._on_selection_changed)
        self.tree.itemDoubleClicked.connect(self._on_item_double_clicked)

        layout.addWidget(self.tree)

        # Button bar
        button_layout = QHBoxLayout()

        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh)

        self.expand_btn = QPushButton("Expand All")
        self.expand_btn.clicked.connect(self.tree.expandAll)

        self.collapse_btn = QPushButton("Collapse All")
        self.collapse_btn.clicked.connect(self.tree.collapseAll)

        button_layout.addWidget(self.refresh_btn)
        button_layout.addWidget(self.expand_btn)
        button_layout.addWidget(self.collapse_btn)
        button_layout.addStretch()

        layout.addLayout(button_layout)

    def set_urdf_manager(self, urdf_manager):
        """
        Set the URDF manager to display

        Args:
            urdf_manager: URDFManager instance
        """
        self.urdf_manager = urdf_manager
        self.refresh()

    def refresh(self):
        """Refresh tree from URDF manager"""
        self.tree.clear()

        if self.urdf_manager is None or not self.urdf_manager.is_loaded():
            # Show placeholder
            placeholder = QTreeWidgetItem(self.tree)
            placeholder.setText(0, "No URDF loaded")
            placeholder.setForeground(0, Qt.gray)
            return

        # Build tree structure
        self._build_tree()

    def _build_tree(self):
        """Build tree structure from URDF"""
        try:
            # Get robot name
            robot_name = self.urdf_manager.get_robot_name()

            # Create root item
            root_item = QTreeWidgetItem(self.tree)
            root_item.setText(0, robot_name)
            root_item.setText(1, "Robot")
            root_item.setExpanded(True)

            # Add links section
            links_item = QTreeWidgetItem(root_item)
            links_item.setText(0, "Links")
            links_item.setText(1, f"({len(self.urdf_manager.get_links())})")
            links_item.setExpanded(True)

            # Add each link
            for link_name in self.urdf_manager.get_links():
                link_info = self.urdf_manager.get_link_info(link_name)
                link_item = QTreeWidgetItem(links_item)
                link_item.setText(0, link_name)
                link_item.setText(1, "Link")

                # Add details
                details = []
                if link_info:
                    if link_info.get('has_visual'):
                        details.append("Visual")
                    if link_info.get('has_collision'):
                        details.append("Collision")
                    if link_info.get('has_inertial'):
                        details.append(f"Mass: {link_info.get('mass', 0):.3f}")

                link_item.setText(2, ", ".join(details) if details else "Empty")

                # Store data for later retrieval
                link_item.setData(0, Qt.UserRole, {'type': 'link', 'name': link_name})

            # Add joints section
            joints_item = QTreeWidgetItem(root_item)
            joints_item.setText(0, "Joints")
            joints_item.setText(1, f"({len(self.urdf_manager.get_joints())})")
            joints_item.setExpanded(True)

            # Add each joint
            for joint_name in self.urdf_manager.get_joints():
                joint_info = self.urdf_manager.get_joint_info(joint_name)
                joint_item = QTreeWidgetItem(joints_item)
                joint_item.setText(0, joint_name)
                joint_item.setText(1, "Joint")

                # Add details
                if joint_info:
                    joint_type = joint_info.get('type', 'unknown')
                    parent = joint_info.get('parent', '?')
                    child = joint_info.get('child', '?')
                    details = f"{joint_type}: {parent} → {child}"
                    joint_item.setText(2, details)

                # Store data
                joint_item.setData(0, Qt.UserRole, {'type': 'joint', 'name': joint_name})

            logging.info("URDF tree refreshed")

        except Exception as e:
            logging.error(f"Failed to build URDF tree: {e}")
            error_item = QTreeWidgetItem(self.tree)
            error_item.setText(0, f"Error: {e}")
            error_item.setForeground(0, Qt.red)

    def _on_selection_changed(self):
        """Handle tree selection change"""
        selected_items = self.tree.selectedItems()

        if not selected_items:
            return

        item = selected_items[0]
        data = item.data(0, Qt.UserRole)

        if data and isinstance(data, dict):
            item_type = data.get('type')
            item_name = data.get('name')

            if item_type and item_name:
                self.item_selected.emit(item_type, item_name)

    def _on_item_double_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle item double click"""
        data = item.data(0, Qt.UserRole)

        if data and isinstance(data, dict):
            item_type = data.get('type')
            item_name = data.get('name')

            if item_type and item_name:
                self.item_double_clicked.emit(item_type, item_name)

    def _show_context_menu(self, position):
        """Show context menu for tree items"""
        item = self.tree.itemAt(position)

        if item is None:
            return

        data = item.data(0, Qt.UserRole)

        if not data or not isinstance(data, dict):
            return

        item_type = data.get('type')
        item_name = data.get('name')

        # Create context menu
        menu = QMenu(self)

        if item_type == 'link':
            view_action = menu.addAction(f"View Link: {item_name}")
            view_action.triggered.connect(lambda: self._view_link(item_name))

            edit_action = menu.addAction(f"Edit Link: {item_name}")
            edit_action.triggered.connect(lambda: self._edit_link(item_name))

        elif item_type == 'joint':
            view_action = menu.addAction(f"View Joint: {item_name}")
            view_action.triggered.connect(lambda: self._view_joint(item_name))

            edit_action = menu.addAction(f"Edit Joint: {item_name}")
            edit_action.triggered.connect(lambda: self._edit_joint(item_name))

        menu.exec(self.tree.viewport().mapToGlobal(position))

    def _view_link(self, link_name: str):
        """View link details"""
        if self.urdf_manager is None:
            return

        info = self.urdf_manager.get_link_info(link_name)

        if info:
            details = f"Link: {link_name}\n\n"
            for key, value in info.items():
                details += f"{key}: {value}\n"

            QMessageBox.information(self, "Link Details", details)

    def _view_joint(self, joint_name: str):
        """View joint details"""
        if self.urdf_manager is None:
            return

        info = self.urdf_manager.get_joint_info(joint_name)

        if info:
            details = f"Joint: {joint_name}\n\n"
            for key, value in info.items():
                details += f"{key}: {value}\n"

            QMessageBox.information(self, "Joint Details", details)

    def _edit_link(self, link_name: str):
        """Edit link (to be implemented with property editor)"""
        logging.info(f"Edit link: {link_name} (property editor integration needed)")
        self.item_double_clicked.emit('link', link_name)

    def _edit_joint(self, joint_name: str):
        """Edit joint (to be implemented with property editor)"""
        logging.info(f"Edit joint: {joint_name} (property editor integration needed)")
        self.item_double_clicked.emit('joint', joint_name)

    def get_selected_item(self) -> Optional[tuple[str, str]]:
        """
        Get currently selected item

        Returns:
            Tuple of (type, name) or None if nothing selected
        """
        selected_items = self.tree.selectedItems()

        if not selected_items:
            return None

        item = selected_items[0]
        data = item.data(0, Qt.UserRole)

        if data and isinstance(data, dict):
            return (data.get('type'), data.get('name'))

        return None
