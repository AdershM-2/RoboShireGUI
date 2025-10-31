"""
Behavior Tree Editor - Visual behavior tree creation for Nav2 and robotics

This widget provides a drag-and-drop interface for creating behavior trees
compatible with BehaviorTree.CPP and Nav2's BT Navigator.

Author: RoboShire Team
Version: 2.2.0 (v2.2.0 Advanced: Natural Language BT Generation)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QToolBar, QGraphicsView,
    QGraphicsScene, QGraphicsItem, QGraphicsRectItem, QGraphicsTextItem,
    QListWidget, QListWidgetItem, QSplitter, QTextEdit, QPushButton,
    QComboBox, QLabel, QMenu, QFileDialog, QMessageBox, QGraphicsEllipseItem,
    QGraphicsLineItem
)
from PySide6.QtCore import Qt, QRectF, QPointF, Signal, QLineF
from PySide6.QtGui import (
    QPen, QBrush, QColor, QPainter, QFont, QAction, QPainterPath
)
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom


class BTNodeType:
    """Behavior Tree node types"""
    # Control nodes
    SEQUENCE = "Sequence"
    FALLBACK = "Fallback"  # Also called Selector
    PARALLEL = "Parallel"
    REACTIVE_SEQUENCE = "ReactiveSequence"
    REACTIVE_FALLBACK = "ReactiveFallback"

    # Decorator nodes
    INVERTER = "Inverter"
    FORCE_SUCCESS = "ForceSuccess"
    FORCE_FAILURE = "ForceFailure"
    REPEAT = "Repeat"
    RETRY = "Retry"

    # Action nodes (Nav2 specific)
    NAVIGATE_TO_POSE = "NavigateToPose"
    FOLLOW_PATH = "FollowPath"
    COMPUTE_PATH_TO_POSE = "ComputePathToPose"
    SMOOTH_PATH = "SmoothPath"
    CLEAR_COSTMAP = "ClearCostmap"
    SPIN = "Spin"
    WAIT = "Wait"
    BACKUP = "BackUp"

    # Condition nodes (Nav2 specific)
    GOAL_REACHED = "GoalReached"
    IS_STUCK = "IsStuck"
    IS_PATH_VALID = "IsPathValid"
    GOAL_UPDATED = "GoalUpdated"
    INITIAL_POSE_RECEIVED = "InitialPoseReceived"


class BTNode(QGraphicsRectItem):
    """Visual representation of a behavior tree node"""

    def __init__(self, node_type: str, name: str = None):
        super().__init__()

        self.node_type = node_type
        self.node_name = name or node_type
        self.children: List['BTNode'] = []
        self.parent: Optional['BTNode'] = None
        self.properties: Dict = {}

        # Visual properties
        self.setRect(0, 0, 150, 60)
        self.setFlags(
            QGraphicsItem.ItemIsMovable |
            QGraphicsItem.ItemIsSelectable |
            QGraphicsItem.ItemSendsGeometryChanges
        )

        # Set colors based on node type
        self._set_colors()

        # Add text label
        self.text_item = QGraphicsTextItem(self.node_name, self)
        self.text_item.setPos(10, 20)
        font = QFont("Arial", 9, QFont.Bold)
        self.text_item.setFont(font)

    def _set_colors(self):
        """Set node colors based on type"""
        # Control nodes - blue
        if self.node_type in [BTNodeType.SEQUENCE, BTNodeType.FALLBACK,
                               BTNodeType.PARALLEL, BTNodeType.REACTIVE_SEQUENCE,
                               BTNodeType.REACTIVE_FALLBACK]:
            self.setBrush(QBrush(QColor(100, 150, 255)))

        # Decorator nodes - purple
        elif self.node_type in [BTNodeType.INVERTER, BTNodeType.FORCE_SUCCESS,
                                 BTNodeType.FORCE_FAILURE, BTNodeType.REPEAT,
                                 BTNodeType.RETRY]:
            self.setBrush(QBrush(QColor(180, 100, 255)))

        # Action nodes - green
        elif self.node_type in [BTNodeType.NAVIGATE_TO_POSE, BTNodeType.FOLLOW_PATH,
                                 BTNodeType.COMPUTE_PATH_TO_POSE, BTNodeType.SMOOTH_PATH,
                                 BTNodeType.CLEAR_COSTMAP, BTNodeType.SPIN,
                                 BTNodeType.WAIT, BTNodeType.BACKUP]:
            self.setBrush(QBrush(QColor(100, 200, 100)))

        # Condition nodes - orange
        elif self.node_type in [BTNodeType.GOAL_REACHED, BTNodeType.IS_STUCK,
                                 BTNodeType.IS_PATH_VALID, BTNodeType.GOAL_UPDATED,
                                 BTNodeType.INITIAL_POSE_RECEIVED]:
            self.setBrush(QBrush(QColor(255, 180, 80)))

        # Default - gray
        else:
            self.setBrush(QBrush(QColor(150, 150, 150)))

        self.setPen(QPen(QColor(50, 50, 50), 2))

    def add_child(self, child: 'BTNode'):
        """Add a child node"""
        if child not in self.children:
            self.children.append(child)
            child.parent = self

    def remove_child(self, child: 'BTNode'):
        """Remove a child node"""
        if child in self.children:
            self.children.remove(child)
            child.parent = None


class BTConnection(QGraphicsLineItem):
    """Visual connection between behavior tree nodes"""

    def __init__(self, parent_node: BTNode, child_node: BTNode):
        super().__init__()

        self.parent_node = parent_node
        self.child_node = child_node

        self.setPen(QPen(QColor(50, 50, 50), 2))
        self.setZValue(-1)  # Draw behind nodes

        self.update_position()

    def update_position(self):
        """Update line position based on node positions"""
        # Get node centers
        parent_rect = self.parent_node.rect()
        child_rect = self.child_node.rect()

        parent_center = self.parent_node.pos() + QPointF(
            parent_rect.width() / 2, parent_rect.height()
        )
        child_center = self.child_node.pos() + QPointF(
            child_rect.width() / 2, 0
        )

        self.setLine(QLineF(parent_center, child_center))


class BehaviorTreeEditor(QWidget):
    """
    Visual behavior tree editor for robotics applications

    Features:
    - Drag-and-drop node creation
    - Nav2 action/condition nodes
    - XML export (BehaviorTree.CPP format)
    - Python export (py_trees format)
    """

    tree_changed = Signal()
    tree_saved = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.root_node: Optional[BTNode] = None
        self.connections: List[BTConnection] = []
        self.current_file: Optional[Path] = None

        self._init_ui()

    def _init_ui(self):
        """Initialize UI"""
        layout = QHBoxLayout(self)

        # Left panel - Node library
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        left_layout.addWidget(QLabel("<b>Behavior Tree Nodes</b>"))

        # Node categories
        self.category_combo = QComboBox()
        self.category_combo.addItems(["All", "Control", "Decorator", "Action", "Condition"])
        self.category_combo.currentTextChanged.connect(self._filter_nodes)
        left_layout.addWidget(self.category_combo)

        # Node list
        self.node_list = QListWidget()
        self.node_list.itemDoubleClicked.connect(self._on_node_double_clicked)
        left_layout.addWidget(self.node_list)

        # Populate node library
        self._populate_node_library()

        left_panel.setMaximumWidth(250)

        # Center panel - Tree editor
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)

        # Toolbar
        toolbar = QToolBar()

        new_action = QAction("New Tree", self)
        new_action.triggered.connect(self._new_tree)
        toolbar.addAction(new_action)

        # v2.1.0: Templates button
        templates_action = QAction("Load Template", self)
        templates_action.setToolTip("Load a pre-built behavior tree template")
        templates_action.triggered.connect(self._show_templates)
        toolbar.addAction(templates_action)

        open_action = QAction("Open", self)
        open_action.triggered.connect(self._open_tree)
        toolbar.addAction(open_action)

        save_action = QAction("Save", self)
        save_action.triggered.connect(self._save_tree)
        toolbar.addAction(save_action)

        toolbar.addSeparator()

        # v2.2.0: Natural Language BT Generation
        nl_action = QAction("🤖 Generate from Text", self)
        nl_action.setToolTip("Generate behavior tree from natural language description")
        nl_action.triggered.connect(self._generate_from_natural_language)
        toolbar.addAction(nl_action)

        toolbar.addSeparator()

        export_xml_action = QAction("Export XML", self)
        export_xml_action.triggered.connect(self._export_xml)
        toolbar.addAction(export_xml_action)

        export_python_action = QAction("Export Python", self)
        export_python_action.triggered.connect(self._export_python)
        toolbar.addAction(export_python_action)

        center_layout.addWidget(toolbar)

        # Graphics view
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(0, 0, 2000, 2000)

        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        center_layout.addWidget(self.view)

        # Right panel - Code preview
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        right_layout.addWidget(QLabel("<b>XML Preview</b>"))

        self.code_preview = QTextEdit()
        self.code_preview.setReadOnly(True)
        self.code_preview.setFont(QFont("Consolas", 9))
        right_layout.addWidget(self.code_preview)

        right_panel.setMaximumWidth(350)

        # Add panels to splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(1, 1)  # Center panel gets most space

        layout.addWidget(splitter)

    def _populate_node_library(self):
        """Populate node library with available nodes"""
        self.all_nodes = {
            "Control": [
                BTNodeType.SEQUENCE,
                BTNodeType.FALLBACK,
                BTNodeType.PARALLEL,
                BTNodeType.REACTIVE_SEQUENCE,
                BTNodeType.REACTIVE_FALLBACK
            ],
            "Decorator": [
                BTNodeType.INVERTER,
                BTNodeType.FORCE_SUCCESS,
                BTNodeType.FORCE_FAILURE,
                BTNodeType.REPEAT,
                BTNodeType.RETRY
            ],
            "Action": [
                BTNodeType.NAVIGATE_TO_POSE,
                BTNodeType.FOLLOW_PATH,
                BTNodeType.COMPUTE_PATH_TO_POSE,
                BTNodeType.SMOOTH_PATH,
                BTNodeType.CLEAR_COSTMAP,
                BTNodeType.SPIN,
                BTNodeType.WAIT,
                BTNodeType.BACKUP
            ],
            "Condition": [
                BTNodeType.GOAL_REACHED,
                BTNodeType.IS_STUCK,
                BTNodeType.IS_PATH_VALID,
                BTNodeType.GOAL_UPDATED,
                BTNodeType.INITIAL_POSE_RECEIVED
            ]
        }

        self._filter_nodes("All")

    def _filter_nodes(self, category: str):
        """Filter nodes by category"""
        self.node_list.clear()

        if category == "All":
            for cat_nodes in self.all_nodes.values():
                for node_type in cat_nodes:
                    item = QListWidgetItem(node_type)
                    self.node_list.addItem(item)
        else:
            for node_type in self.all_nodes.get(category, []):
                item = QListWidgetItem(node_type)
                self.node_list.addItem(item)

    def _on_node_double_clicked(self, item: QListWidgetItem):
        """Handle node double-click - add to scene"""
        node_type = item.text()
        node = BTNode(node_type)
        node.setPos(100, 100)
        self.scene.addItem(node)

        if self.root_node is None:
            self.root_node = node

        self._update_preview()
        self.tree_changed.emit()

    def _new_tree(self):
        """Create a new behavior tree"""
        reply = QMessageBox.question(
            self,
            "New Tree",
            "Create new behavior tree? Unsaved changes will be lost.",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.scene.clear()
            self.root_node = None
            self.connections.clear()
            self.current_file = None
            self.code_preview.clear()
            self.tree_changed.emit()

    def _open_tree(self):
        """Open existing behavior tree from XML"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open Behavior Tree",
            "",
            "XML Files (*.xml);;All Files (*)"
        )

        if file_path:
            try:
                # TODO: Implement XML import
                self.current_file = Path(file_path)
                QMessageBox.information(self, "Open Tree", "XML import not yet implemented")
            except Exception as e:
                QMessageBox.critical(self, "Open Error", f"Failed to open file:\n{e}")

    def _save_tree(self):
        """Save current behavior tree"""
        if self.current_file:
            self._export_xml_to_file(self.current_file)
        else:
            self._export_xml()

    def _export_xml(self):
        """Export behavior tree to XML file"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Behavior Tree XML",
            "",
            "XML Files (*.xml);;All Files (*)"
        )

        if file_path:
            self._export_xml_to_file(Path(file_path))

    def _export_xml_to_file(self, file_path: Path):
        """Export tree to XML file"""
        try:
            xml_content = self._generate_xml()

            with open(file_path, 'w') as f:
                f.write(xml_content)

            self.current_file = file_path
            self.tree_saved.emit(str(file_path))
            QMessageBox.information(
                self,
                "Export Successful",
                f"Behavior tree saved to:\n{file_path}"
            )

        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export:\n{e}")

    def _export_python(self):
        """Export behavior tree to Python (py_trees)"""
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export Behavior Tree Python",
            "",
            "Python Files (*.py);;All Files (*)"
        )

        if file_path:
            try:
                python_content = self._generate_python()

                with open(file_path, 'w') as f:
                    f.write(python_content)

                QMessageBox.information(
                    self,
                    "Export Successful",
                    f"Python behavior tree saved to:\n{file_path}"
                )

            except Exception as e:
                QMessageBox.critical(self, "Export Error", f"Failed to export:\n{e}")

    def _generate_xml(self) -> str:
        """Generate BehaviorTree.CPP XML format"""
        if not self.root_node:
            return '<?xml version="1.0"?>\n<root/>'

        root = ET.Element('root')
        root.set('main_tree_to_execute', 'MainTree')

        behavior_tree = ET.SubElement(root, 'BehaviorTree')
        behavior_tree.set('ID', 'MainTree')

        # Recursively build tree
        self._build_xml_node(self.root_node, behavior_tree)

        # Pretty print
        xml_str = ET.tostring(root, encoding='unicode')
        dom = minidom.parseString(xml_str)
        return dom.toprettyxml(indent='  ')

    def _build_xml_node(self, node: BTNode, parent_element: ET.Element):
        """Recursively build XML tree"""
        element = ET.SubElement(parent_element, node.node_type)

        if node.node_name != node.node_type:
            element.set('name', node.node_name)

        # Add properties as attributes
        for key, value in node.properties.items():
            element.set(key, str(value))

        # Add children
        for child in node.children:
            self._build_xml_node(child, element)

    def _generate_python(self) -> str:
        """Generate py_trees Python code"""
        lines = []
        lines.append("#!/usr/bin/env python3")
        lines.append('"""')
        lines.append("Behavior Tree - Generated by RoboShire v2.0.0")
        lines.append('"""')
        lines.append("")
        lines.append("import py_trees")
        lines.append("from py_trees.composites import Sequence, Selector, Parallel")
        lines.append("")
        lines.append("")
        lines.append("def create_tree():")
        lines.append("    \"\"\"Create and return behavior tree\"\"\"")

        if self.root_node:
            lines.append(f"    root = {self._python_node_code(self.root_node, 1)}")
        else:
            lines.append("    root = Sequence('Root')")

        lines.append("")
        lines.append("    return root")
        lines.append("")
        lines.append("")
        lines.append("if __name__ == '__main__':")
        lines.append("    tree = create_tree()")
        lines.append("    print(py_trees.display.unicode_tree(tree))")

        return "\n".join(lines)

    def _python_node_code(self, node: BTNode, indent: int) -> str:
        """Generate Python code for a node"""
        ind = "    " * indent

        if node.node_type == BTNodeType.SEQUENCE:
            code = f"Sequence('{node.node_name}')"
        elif node.node_type == BTNodeType.FALLBACK:
            code = f"Selector('{node.node_name}')"
        else:
            code = f"py_trees.behaviours.Success('{node.node_name}')"

        # TODO: Add children support

        return code

    def _update_preview(self):
        """Update XML code preview"""
        xml_content = self._generate_xml()
        self.code_preview.setPlainText(xml_content)
    def _show_templates(self):
        """v2.1.0: Show behavior tree templates dialog"""
        from PySide6.QtWidgets import QDialog, QDialogButtonBox, QListWidget

        dialog = QDialog(self)
        dialog.setWindowTitle("Behavior Tree Templates")
        dialog.resize(700, 500)

        layout = QVBoxLayout(dialog)

        layout.addWidget(QLabel("<b>Pre-built Behavior Tree Templates</b>"))
        layout.addWidget(QLabel("Select a template to load into the editor:"))

        # Templates list
        templates_list = QListWidget()

        templates_data = self._get_templates()

        for template_name, template_info in templates_data.items():
            item = QListWidgetItem(f"{template_name} - {template_info['description']}")
            item.setData(Qt.UserRole, template_name)
            templates_list.addItem(item)

        templates_list.itemDoubleClicked.connect(lambda: dialog.accept())
        layout.addWidget(templates_list)

        # Preview area
        preview_group = QGroupBox("Template Description")
        preview_layout = QVBoxLayout(preview_group)

        preview_text = QTextEdit()
        preview_text.setReadOnly(True)
        preview_text.setMaximumHeight(150)
        preview_layout.addWidget(preview_text)

        layout.addWidget(preview_group)

        def update_preview():
            selected_items = templates_list.selectedItems()
            if selected_items:
                template_name = selected_items[0].data(Qt.UserRole)
                template_info = templates_data[template_name]
                preview_text.setPlainText(
                    f"Template: {template_name}\n\n"
                    f"Description: {template_info['description']}\n\n"
                    f"Use Case: {template_info['use_case']}\n\n"
                    f"Nodes: {template_info['node_count']}"
                )

        templates_list.itemSelectionChanged.connect(update_preview)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        # Show dialog
        if dialog.exec() == QDialog.Accepted:
            selected_items = templates_list.selectedItems()
            if selected_items:
                template_name = selected_items[0].data(Qt.UserRole)
                self._load_template(template_name)

    def _get_templates(self) -> Dict:
        """v2.1.0: Get available behavior tree templates"""
        return {
            "Simple Navigation": {
                "description": "Basic point-to-point navigation",
                "use_case": "Navigate from current position to a goal pose",
                "node_count": 3,
                "structure": {
                    "root": {
                        "type": BTNodeType.SEQUENCE,
                        "children": [
                            {"type": BTNodeType.COMPUTE_PATH_TO_POSE},
                            {"type": BTNodeType.FOLLOW_PATH}
                        ]
                    }
                }
            },
            "Navigation with Recovery": {
                "description": "Navigation with error handling and recovery behaviors",
                "use_case": "Robust navigation that handles getting stuck",
                "node_count": 7,
                "structure": {
                    "root": {
                        "type": BTNodeType.FALLBACK,
                        "children": [
                            {
                                "type": BTNodeType.SEQUENCE,
                                "children": [
                                    {"type": BTNodeType.COMPUTE_PATH_TO_POSE},
                                    {"type": BTNodeType.FOLLOW_PATH},
                                    {"type": BTNodeType.GOAL_REACHED}
                                ]
                            },
                            {
                                "type": BTNodeType.SEQUENCE,
                                "children": [
                                    {"type": BTNodeType.CLEAR_COSTMAP},
                                    {"type": BTNodeType.SPIN},
                                    {"type": BTNodeType.BACKUP}
                                ]
                            }
                        ]
                    }
                }
            },
            "Patrol Route": {
                "description": "Continuous patrol between waypoints",
                "use_case": "Security or monitoring patrol pattern",
                "node_count": 5,
                "structure": {
                    "root": {
                        "type": BTNodeType.REACTIVE_SEQUENCE,
                        "children": [
                            {"type": BTNodeType.GOAL_UPDATED},
                            {"type": BTNodeType.COMPUTE_PATH_TO_POSE},
                            {"type": BTNodeType.FOLLOW_PATH},
                            {"type": BTNodeType.WAIT},  # Wait at waypoint
                            {"type": BTNodeType.GOAL_REACHED}
                        ]
                    }
                }
            },
            "Inspection Task": {
                "description": "Navigate to inspection points and perform checks",
                "use_case": "Industrial inspection or data collection",
                "node_count": 8,
                "structure": {
                    "root": {
                        "type": BTNodeType.SEQUENCE,
                        "children": [
                            {
                                "type": BTNodeType.FALLBACK,
                                "children": [
                                    {
                                        "type": BTNodeType.SEQUENCE,
                                        "children": [
                                            {"type": BTNodeType.IS_PATH_VALID},
                                            {"type": BTNodeType.NAVIGATE_TO_POSE}
                                        ]
                                    },
                                    {"type": BTNodeType.CLEAR_COSTMAP}
                                ]
                            },
                            {"type": BTNodeType.WAIT},  # Perform inspection
                            {"type": BTNodeType.GOAL_REACHED}
                        ]
                    }
                }
            },
            "Adaptive Behavior": {
                "description": "Behavior that adapts based on conditions",
                "use_case": "Dynamic behavior switching based on robot state",
                "node_count": 10,
                "structure": {
                    "root": {
                        "type": BTNodeType.FALLBACK,
                        "children": [
                            {
                                "type": BTNodeType.SEQUENCE,
                                "children": [
                                    {"type": BTNodeType.GOAL_REACHED},
                                    {"type": BTNodeType.FORCE_SUCCESS, "children": [{"type": BTNodeType.WAIT}]}
                                ]
                            },
                            {
                                "type": BTNodeType.SEQUENCE,
                                "children": [
                                    {"type": BTNodeType.IS_STUCK},
                                    {
                                        "type": BTNodeType.FALLBACK,
                                        "children": [
                                            {"type": BTNodeType.BACKUP},
                                            {"type": BTNodeType.SPIN},
                                            {"type": BTNodeType.CLEAR_COSTMAP}
                                        ]
                                    }
                                ]
                            },
                            {
                                "type": BTNodeType.SEQUENCE,
                                "children": [
                                    {"type": BTNodeType.COMPUTE_PATH_TO_POSE},
                                    {"type": BTNodeType.FOLLOW_PATH}
                                ]
                            }
                        ]
                    }
                }
            }
        }

    def _load_template(self, template_name: str):
        """v2.1.0: Load a behavior tree template"""
        templates = self._get_templates()

        if template_name not in templates:
            QMessageBox.warning(self, "Template Not Found", f"Template '{template_name}' not found.")
            return

        # Clear current tree
        self.scene.clear()
        self.root_node = None
        self.connections.clear()
        self.current_file = None

        # Build tree from template
        template = templates[template_name]
        self.root_node = self._build_tree_from_structure(template["structure"]["root"], 400, 50)

        # Layout nodes
        self._auto_layout_tree()

        self._update_preview()
        self.tree_changed.emit()

        QMessageBox.information(
            self,
            "Template Loaded",
            f"Template '{template_name}' loaded successfully!\n\n"
            f"You can now customize the tree by:\n"
            f"• Moving nodes to adjust layout\n"
            f"• Adding/removing nodes\n"
            f"• Connecting nodes with parent-child relationships\n"
            f"• Exporting to XML or Python"
        )

    def _build_tree_from_structure(self, structure: Dict, x: float, y: float) -> BTNode:
        """v2.1.0: Recursively build tree from template structure"""
        node_type = structure["type"]
        node = BTNode(node_type)
        node.setPos(x, y)
        self.scene.addItem(node)

        # Recursively add children
        if "children" in structure:
            child_x = x - 100 * (len(structure["children"]) - 1) / 2
            child_y = y + 150

            for i, child_structure in enumerate(structure["children"]):
                child_node = self._build_tree_from_structure(
                    child_structure,
                    child_x + i * 200,
                    child_y
                )
                node.add_child(child_node)

                # Add visual connection
                connection = BTConnection(node, child_node)
                self.scene.addItem(connection)
                self.connections.append(connection)

        return node

    def _auto_layout_tree(self):
        """v2.1.0: Automatically layout tree nodes in a hierarchical structure"""
        if not self.root_node:
            return

        # Simple tree layout algorithm
        def layout_node(node: BTNode, x: float, y: float, h_spacing: float = 200):
            node.setPos(x, y)

            if node.children:
                child_y = y + 150
                total_width = (len(node.children) - 1) * h_spacing
                start_x = x - total_width / 2

                for i, child in enumerate(node.children):
                    child_x = start_x + i * h_spacing
                    layout_node(child, child_x, child_y, h_spacing * 0.8)

        layout_node(self.root_node, 400, 50)

        # Update all connections
        for connection in self.connections:
            connection.update_position()

    def _generate_from_natural_language(self):
        """
        v2.2.0: Generate behavior tree from natural language description

        Uses keyword matching and pattern recognition to create BT structure
        """
        from PySide6.QtWidgets import QDialog, QDialogButtonBox, QTextEdit

        dialog = QDialog(self)
        dialog.setWindowTitle("Natural Language BT Generator")
        dialog.resize(600, 400)

        layout = QVBoxLayout(dialog)

        layout.addWidget(QLabel("<b>🤖 Natural Language BT Generator</b>"))
        layout.addWidget(QLabel(
            "Describe your robot behavior in plain English. Examples:\n"
            "• \"Navigate to kitchen, if stuck, clear costmap and retry\"\n"
            "• \"Follow path to goal, if blocked, spin and backup\"\n"
            "• \"Try to reach goal, if failed, clear map then retry\""
        ))

        # Text input
        text_input = QTextEdit()
        text_input.setPlaceholderText(
            "Example: Navigate to the goal. If the robot gets stuck, "
            "clear the costmap, spin around, and try again..."
        )
        layout.addWidget(text_input)

        # Example button
        example_btn = QPushButton("Load Example")
        example_btn.clicked.connect(
            lambda: text_input.setPlainText(
                "Navigate to the goal. If the robot gets stuck, clear the costmap, "
                "spin around, and backup. Then try navigating again."
            )
        )
        layout.addWidget(example_btn)

        # Buttons
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec() == QDialog.Accepted:
            text = text_input.toPlainText()
            if text.strip():
                try:
                    self._parse_and_generate_bt(text)
                except Exception as e:
                    QMessageBox.warning(
                        self,
                        "Generation Error",
                        f"Failed to generate behavior tree from text:\n\n{str(e)}\n\n"
                        f"Try using clearer keywords like 'navigate', 'if stuck', 'retry', etc."
                    )

    def _parse_and_generate_bt(self, text: str):
        """
        v2.2.0: Parse natural language and generate BT structure

        Simple keyword-based parser for common navigation patterns
        """
        text_lower = text.lower()

        # Detect control structure keywords
        has_if = 'if' in text_lower
        has_retry = 'retry' in text_lower or 'try again' in text_lower
        has_loop = 'loop' in text_lower or 'repeatedly' in text_lower or 'patrol' in text_lower

        # Detect actions
        actions = []

        # Navigation actions
        if 'navigate' in text_lower or 'go to' in text_lower or 'reach' in text_lower:
            actions.append(('navigate', BTNodeType.NAVIGATE_TO_POSE))

        if 'compute path' in text_lower or 'plan path' in text_lower:
            actions.append(('compute_path', BTNodeType.COMPUTE_PATH_TO_POSE))

        if 'follow path' in text_lower or 'follow' in text_lower:
            actions.append(('follow', BTNodeType.FOLLOW_PATH))

        # Recovery actions
        recovery_actions = []

        if 'clear costmap' in text_lower or 'clear map' in text_lower:
            recovery_actions.append(('clear_costmap', BTNodeType.CLEAR_COSTMAP))

        if 'spin' in text_lower or 'rotate' in text_lower or 'turn around' in text_lower:
            recovery_actions.append(('spin', BTNodeType.SPIN))

        if 'backup' in text_lower or 'back up' in text_lower or 'reverse' in text_lower:
            recovery_actions.append(('backup', BTNodeType.BACKUP))

        if 'wait' in text_lower:
            recovery_actions.append(('wait', BTNodeType.WAIT))

        # Detect conditions
        conditions = []

        if 'stuck' in text_lower or 'blocked' in text_lower:
            conditions.append(('stuck', BTNodeType.IS_STUCK))

        if 'goal reached' in text_lower or 'reached goal' in text_lower:
            conditions.append(('goal_reached', BTNodeType.GOAL_REACHED))

        if 'path valid' in text_lower or 'valid path' in text_lower:
            conditions.append(('path_valid', BTNodeType.IS_PATH_VALID))

        # Generate structure based on detected patterns
        structure = self._infer_bt_structure(
            actions, recovery_actions, conditions, has_if, has_retry, has_loop
        )

        # Clear current tree
        self.scene.clear()
        self.root_node = None
        self.connections.clear()

        # Build tree from inferred structure
        self.root_node = self._build_tree_from_structure(structure, 400, 50)

        # Layout nodes
        self._auto_layout_tree()

        self._update_preview()
        self.tree_changed.emit()

        # Show success message with what was generated
        action_names = [a[0] for a in actions]
        recovery_names = [r[0] for r in recovery_actions]

        QMessageBox.information(
            self,
            "BT Generated Successfully",
            f"✅ Behavior tree generated from your description!\n\n"
            f"Detected Actions: {', '.join(action_names) if action_names else 'None'}\n"
            f"Detected Recovery: {', '.join(recovery_names) if recovery_names else 'None'}\n"
            f"Control Structure: {'Fallback (if/else)' if has_if else 'Sequence'}\n\n"
            f"You can now edit the tree visually or export to XML/Python."
        )

    def _infer_bt_structure(
        self,
        actions: List[Tuple[str, str]],
        recovery_actions: List[Tuple[str, str]],
        conditions: List[Tuple[str, str]],
        has_if: bool,
        has_retry: bool,
        has_loop: bool
    ) -> Dict:
        """
        v2.2.0: Infer behavior tree structure from parsed elements

        Applies heuristics to determine appropriate BT control flow
        """
        # Default: simple sequence of actions
        if not actions:
            # No actions detected, use basic navigation
            actions = [('navigate', BTNodeType.NAVIGATE_TO_POSE)]

        # Pattern 1: If/else structure (Fallback)
        if has_if and recovery_actions:
            # Main sequence (try to navigate)
            main_sequence = {
                'type': BTNodeType.SEQUENCE,
                'children': [{'type': node_type} for _, node_type in actions]
            }

            # Recovery sequence (handle failures)
            recovery_sequence = {
                'type': BTNodeType.SEQUENCE,
                'children': [{'type': node_type} for _, node_type in recovery_actions]
            }

            # Add retry if mentioned
            if has_retry:
                recovery_sequence['children'].append({'type': BTNodeType.COMPUTE_PATH_TO_POSE})
                recovery_sequence['children'].append({'type': BTNodeType.FOLLOW_PATH})

            return {
                'type': BTNodeType.FALLBACK,
                'children': [main_sequence, recovery_sequence]
            }

        # Pattern 2: Loop/Patrol (Reactive Sequence)
        elif has_loop:
            return {
                'type': BTNodeType.REACTIVE_SEQUENCE,
                'children': [{'type': node_type} for _, node_type in actions] + [
                    {'type': BTNodeType.GOAL_REACHED},
                    {'type': BTNodeType.WAIT}
                ]
            }

        # Pattern 3: Simple sequence
        else:
            return {
                'type': BTNodeType.SEQUENCE,
                'children': [{'type': node_type} for _, node_type in actions]
            }


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    editor = BehaviorTreeEditor()
    editor.setWindowTitle("Behavior Tree Editor")
    editor.resize(1200, 700)
    editor.show()

    sys.exit(app.exec())
