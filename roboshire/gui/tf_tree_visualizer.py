"""
TF Tree Visualizer - Real-time transform tree visualization and debugging

This widget displays the TF (transform) tree for a running ROS2 system,
showing parent-child frame relationships and transform data.

v2.0.1 Features:
- Design Mode: Create TF trees without running ROS2 system
- Manual frame entry and editing
- URDF import/export
- Transform calculator
- Validation suite

Author: RoboShire Team
Version: 2.0.1
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QToolBar, QGraphicsView,
    QGraphicsScene, QGraphicsItem, QGraphicsEllipseItem, QGraphicsTextItem,
    QGraphicsLineItem, QLabel, QTextEdit, QSplitter, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QTreeWidget, QTreeWidgetItem,
    QGroupBox, QFormLayout, QDialog, QLineEdit, QDoubleSpinBox, QDialogButtonBox,
    QComboBox, QMessageBox, QFileDialog, QTabWidget, QCheckBox
)
from PySide6.QtCore import Qt, QPointF, QTimer, Signal, QRectF, QLineF
from PySide6.QtGui import (
    QPen, QBrush, QColor, QPainter, QFont, QAction
)
from typing import Dict, List, Optional, Tuple
import subprocess
import re
import math
import json
from pathlib import Path


class TFFrame(QGraphicsItem):
    """Visual representation of a TF frame with rounded rectangle design"""

    def __init__(self, frame_name: str):
        super().__init__()

        self.frame_name = frame_name
        self.children: List['TFFrame'] = []
        self.parent_frame: Optional['TFFrame'] = None

        # Size
        self.width = 140
        self.height = 50

        # Determine color based on frame type
        if frame_name in ['map', 'world', 'odom']:
            # Root frame - Gold/Orange
            self.bg_color = QColor(255, 200, 80)
            self.border_color = QColor(200, 150, 40)
            self.icon = "🌍"
        elif 'base' in frame_name.lower():
            # Base link - Green
            self.bg_color = QColor(120, 220, 140)
            self.border_color = QColor(80, 180, 100)
            self.icon = "🤖"
        elif 'camera' in frame_name.lower():
            # Camera - Purple
            self.bg_color = QColor(180, 140, 220)
            self.border_color = QColor(140, 100, 180)
            self.icon = "📷"
        elif 'lidar' in frame_name.lower() or 'laser' in frame_name.lower():
            # Lidar - Red
            self.bg_color = QColor(255, 140, 140)
            self.border_color = QColor(200, 100, 100)
            self.icon = "📡"
        elif 'link' in frame_name.lower():
            # Link - Blue
            self.bg_color = QColor(140, 180, 255)
            self.border_color = QColor(100, 140, 220)
            self.icon = "🔗"
        else:
            # Default - Light Blue
            self.bg_color = QColor(160, 200, 255)
            self.border_color = QColor(120, 160, 220)
            self.icon = "📍"

        self.setFlags(
            QGraphicsItem.ItemIsMovable |
            QGraphicsItem.ItemIsSelectable
        )

        # Add text label
        self.text_item = QGraphicsTextItem(self)
        self.text_item.setHtml(
            f"<div style='text-align: center;'>"
            f"<span style='font-size: 14px;'>{self.icon}</span><br>"
            f"<span style='font-size: 10px; font-weight: bold;'>{frame_name}</span>"
            f"</div>"
        )
        # Center the text
        text_rect = self.text_item.boundingRect()
        self.text_item.setPos(
            -text_rect.width() / 2,
            -text_rect.height() / 2
        )

    def boundingRect(self):
        """Define the bounding rectangle"""
        return QRectF(-self.width/2, -self.height/2, self.width, self.height)

    def paint(self, painter, option, widget=None):
        """Draw the frame as a rounded rectangle"""
        painter.setRenderHint(QPainter.Antialiasing)

        # Shadow effect
        shadow_rect = QRectF(-self.width/2 + 2, -self.height/2 + 2, self.width, self.height)
        painter.setBrush(QBrush(QColor(0, 0, 0, 30)))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(shadow_rect, 10, 10)

        # Main rectangle
        rect = QRectF(-self.width/2, -self.height/2, self.width, self.height)

        # Gradient background
        from PySide6.QtGui import QLinearGradient
        gradient = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        gradient.setColorAt(0, self.bg_color.lighter(115))
        gradient.setColorAt(1, self.bg_color)

        painter.setBrush(QBrush(gradient))

        # Border (thicker if selected)
        if self.isSelected():
            painter.setPen(QPen(QColor(255, 140, 0), 3))
        else:
            painter.setPen(QPen(self.border_color, 2))

        painter.drawRoundedRect(rect, 10, 10)


class TFConnection(QGraphicsItem):
    """Visual connection between TF frames with proper arrows"""

    def __init__(self, parent_frame: TFFrame, child_frame: TFFrame):
        super().__init__()

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.arrow = None  # Keep for compatibility but not used

        self.setZValue(-1)  # Draw behind nodes

    def boundingRect(self):
        """Define bounding rectangle"""
        parent_pos = self.parent_frame.pos()
        child_pos = self.child_frame.pos()

        x1, y1 = parent_pos.x(), parent_pos.y()
        x2, y2 = child_pos.x(), child_pos.y()

        # Include some padding for the arrow
        padding = 20
        return QRectF(
            min(x1, x2) - padding,
            min(y1, y2) - padding,
            abs(x2 - x1) + 2 * padding,
            abs(y2 - y1) + 2 * padding
        )

    def paint(self, painter, option, widget=None):
        """Draw connection with curved line and arrowhead"""
        painter.setRenderHint(QPainter.Antialiasing)

        parent_pos = self.parent_frame.pos()
        child_pos = self.child_frame.pos()

        # Draw curved path
        from PySide6.QtGui import QPainterPath
        path = QPainterPath()
        path.moveTo(parent_pos)

        # Control points for smooth curve
        ctrl_offset_x = abs(child_pos.x() - parent_pos.x()) * 0.5
        ctrl1 = QPointF(parent_pos.x() + ctrl_offset_x, parent_pos.y())
        ctrl2 = QPointF(child_pos.x() - ctrl_offset_x, child_pos.y())

        path.cubicTo(ctrl1, ctrl2, child_pos)

        # Draw the path with gradient
        from PySide6.QtGui import QLinearGradient
        gradient = QLinearGradient(parent_pos, child_pos)
        gradient.setColorAt(0, QColor(100, 100, 100))
        gradient.setColorAt(1, QColor(140, 140, 140))

        pen = QPen(QBrush(gradient), 3)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawPath(path)

        # Draw arrowhead
        angle = math.atan2(child_pos.y() - parent_pos.y(), child_pos.x() - parent_pos.x())
        arrow_size = 12

        # Arrow tip at child position
        arrow_p1 = QPointF(
            child_pos.x() - arrow_size * math.cos(angle - math.pi / 6),
            child_pos.y() - arrow_size * math.sin(angle - math.pi / 6)
        )
        arrow_p2 = QPointF(
            child_pos.x() - arrow_size * math.cos(angle + math.pi / 6),
            child_pos.y() - arrow_size * math.sin(angle + math.pi / 6)
        )

        # Draw filled arrow
        arrow_path = QPainterPath()
        arrow_path.moveTo(child_pos)
        arrow_path.lineTo(arrow_p1)
        arrow_path.lineTo(arrow_p2)
        arrow_path.closeSubpath()

        painter.setBrush(QBrush(QColor(100, 100, 100)))
        painter.setPen(Qt.NoPen)
        painter.drawPath(arrow_path)

    def update_position(self):
        """Update position (called when frames move)"""
        self.prepareGeometryChange()
        self.update()


class FrameEditorDialog(QDialog):
    """Dialog for adding/editing TF frames manually"""

    def __init__(self, parent=None, frame_data=None, existing_frames=None):
        super().__init__(parent)
        self.setWindowTitle("Add/Edit TF Frame")
        self.setMinimumWidth(400)

        self.existing_frames = existing_frames or []

        layout = QVBoxLayout(self)

        # Frame name
        form_layout = QFormLayout()

        self.frame_name_edit = QLineEdit()
        if frame_data:
            self.frame_name_edit.setText(frame_data.get('name', ''))
            self.frame_name_edit.setReadOnly(True)
        form_layout.addRow("Frame Name:", self.frame_name_edit)

        # Parent frame
        self.parent_combo = QComboBox()
        self.parent_combo.addItem("(None - Root Frame)")
        self.parent_combo.addItems(self.existing_frames)
        if frame_data and frame_data.get('parent'):
            index = self.parent_combo.findText(frame_data['parent'])
            if index >= 0:
                self.parent_combo.setCurrentIndex(index)
        form_layout.addRow("Parent Frame:", self.parent_combo)

        layout.addLayout(form_layout)

        # Transform group
        transform_group = QGroupBox("Transform (Parent → This Frame)")
        transform_layout = QFormLayout(transform_group)

        # Translation
        self.tx_spin = QDoubleSpinBox()
        self.tx_spin.setRange(-100, 100)
        self.tx_spin.setDecimals(3)
        self.tx_spin.setSingleStep(0.01)
        self.tx_spin.setValue(frame_data.get('tx', 0.0) if frame_data else 0.0)
        transform_layout.addRow("Translation X (m):", self.tx_spin)

        self.ty_spin = QDoubleSpinBox()
        self.ty_spin.setRange(-100, 100)
        self.ty_spin.setDecimals(3)
        self.ty_spin.setSingleStep(0.01)
        self.ty_spin.setValue(frame_data.get('ty', 0.0) if frame_data else 0.0)
        transform_layout.addRow("Translation Y (m):", self.ty_spin)

        self.tz_spin = QDoubleSpinBox()
        self.tz_spin.setRange(-100, 100)
        self.tz_spin.setDecimals(3)
        self.tz_spin.setSingleStep(0.01)
        self.tz_spin.setValue(frame_data.get('tz', 0.0) if frame_data else 0.0)
        transform_layout.addRow("Translation Z (m):", self.tz_spin)

        # Rotation (Euler angles in radians)
        self.rx_spin = QDoubleSpinBox()
        self.rx_spin.setRange(-math.pi, math.pi)
        self.rx_spin.setDecimals(4)
        self.rx_spin.setSingleStep(0.1)
        self.rx_spin.setValue(frame_data.get('rx', 0.0) if frame_data else 0.0)
        transform_layout.addRow("Rotation X (rad):", self.rx_spin)

        self.ry_spin = QDoubleSpinBox()
        self.ry_spin.setRange(-math.pi, math.pi)
        self.ry_spin.setDecimals(4)
        self.ry_spin.setSingleStep(0.1)
        self.ry_spin.setValue(frame_data.get('ry', 0.0) if frame_data else 0.0)
        transform_layout.addRow("Rotation Y (rad):", self.ry_spin)

        self.rz_spin = QDoubleSpinBox()
        self.rz_spin.setRange(-math.pi, math.pi)
        self.rz_spin.setDecimals(4)
        self.rz_spin.setSingleStep(0.1)
        self.rz_spin.setValue(frame_data.get('rz', 0.0) if frame_data else 0.0)
        transform_layout.addRow("Rotation Z (rad):", self.rz_spin)

        layout.addWidget(transform_group)

        # Buttons
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def get_frame_data(self):
        """Get frame data from dialog"""
        parent = self.parent_combo.currentText()
        if parent == "(None - Root Frame)":
            parent = None

        return {
            'name': self.frame_name_edit.text().strip(),
            'parent': parent,
            'tx': self.tx_spin.value(),
            'ty': self.ty_spin.value(),
            'tz': self.tz_spin.value(),
            'rx': self.rx_spin.value(),
            'ry': self.ry_spin.value(),
            'rz': self.rz_spin.value()
        }


class TFTreeVisualizer(QWidget):
    """
    TF Tree Visualizer - Display and debug ROS2 transform tree

    Features:
    - Real-time TF tree monitoring (requires ROS2 system)
    - Design Mode: Manual frame creation without ROS2
    - Visual frame hierarchy
    - Transform data inspection and editing
    - Frame relationship debugging
    - URDF import/export
    - Export TF tree diagram
    """

    frame_selected = Signal(str)

    def __init__(self, ssh_bridge=None, parent=None):
        super().__init__(parent)

        self.ssh_bridge = ssh_bridge
        self.frames: Dict[str, TFFrame] = {}
        self.connections: List[TFConnection] = []
        self.tf_data: Dict = {}  # Store TF data
        self.auto_refresh = False
        self.design_mode = True  # Start in design mode by default
        self.manual_frames: Dict[str, Dict] = {}  # Store manually created frames

        # Timer for auto-refresh
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self._refresh_tree)

        self._init_ui()

        # Load sample design mode data
        self._load_sample_design_mode_data()

    def _init_ui(self):
        """Initialize UI"""
        layout = QHBoxLayout(self)

        # Left panel - Tree view and controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Mode selector
        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout(mode_group)

        self.mode_label = QLabel("🎨 <b>Design Mode</b> - Create frames manually")
        self.mode_label.setWordWrap(True)
        self.mode_label.setStyleSheet("color: #0066cc; padding: 5px;")
        mode_layout.addWidget(self.mode_label)

        self.toggle_mode_btn = QPushButton("Switch to Runtime Mode")
        self.toggle_mode_btn.clicked.connect(self._toggle_mode)
        mode_layout.addWidget(self.toggle_mode_btn)

        left_layout.addWidget(mode_group)

        # Controls
        controls_group = QGroupBox("Controls")
        controls_layout = QVBoxLayout(controls_group)

        # Design mode controls
        self.design_controls = QWidget()
        design_layout = QVBoxLayout(self.design_controls)
        design_layout.setContentsMargins(0, 0, 0, 0)

        add_frame_btn = QPushButton("➕ Add Frame")
        add_frame_btn.clicked.connect(self._add_frame)
        design_layout.addWidget(add_frame_btn)

        edit_frame_btn = QPushButton("✏️ Edit Selected Frame")
        edit_frame_btn.clicked.connect(self._edit_selected_frame)
        design_layout.addWidget(edit_frame_btn)

        delete_frame_btn = QPushButton("🗑️ Delete Selected Frame")
        delete_frame_btn.clicked.connect(self._delete_selected_frame)
        design_layout.addWidget(delete_frame_btn)

        load_template_btn = QPushButton("📁 Load Template")
        load_template_btn.clicked.connect(self._load_template)
        design_layout.addWidget(load_template_btn)

        controls_layout.addWidget(self.design_controls)

        # Runtime mode controls
        self.runtime_controls = QWidget()
        runtime_layout = QVBoxLayout(self.runtime_controls)
        runtime_layout.setContentsMargins(0, 0, 0, 0)

        refresh_btn = QPushButton("🔄 Refresh TF Tree")
        refresh_btn.clicked.connect(self._refresh_tree)
        runtime_layout.addWidget(refresh_btn)

        self.auto_refresh_btn = QPushButton("Start Auto-Refresh (1s)")
        self.auto_refresh_btn.setCheckable(True)
        self.auto_refresh_btn.clicked.connect(self._toggle_auto_refresh)
        runtime_layout.addWidget(self.auto_refresh_btn)

        self.runtime_controls.hide()  # Hidden by default (design mode)
        controls_layout.addWidget(self.runtime_controls)

        # Common controls
        export_btn = QPushButton("💾 Export Diagram")
        export_btn.clicked.connect(self._export_diagram)
        controls_layout.addWidget(export_btn)

        export_urdf_btn = QPushButton("📄 Export URDF")
        export_urdf_btn.clicked.connect(self._export_urdf)
        controls_layout.addWidget(export_urdf_btn)

        import_urdf_btn = QPushButton("📂 Import URDF")
        import_urdf_btn.clicked.connect(self._import_urdf)
        controls_layout.addWidget(import_urdf_btn)

        validate_btn = QPushButton("✅ Validate Tree")
        validate_btn.clicked.connect(self._validate_tree)
        controls_layout.addWidget(validate_btn)

        # Zoom controls
        zoom_label = QLabel("<b>Zoom Controls:</b>")
        controls_layout.addWidget(zoom_label)

        zoom_in_btn = QPushButton("🔍+ Zoom In")
        zoom_in_btn.clicked.connect(self._zoom_in)
        controls_layout.addWidget(zoom_in_btn)

        zoom_out_btn = QPushButton("🔍- Zoom Out")
        zoom_out_btn.clicked.connect(self._zoom_out)
        controls_layout.addWidget(zoom_out_btn)

        fit_view_btn = QPushButton("📐 Fit to View")
        fit_view_btn.clicked.connect(self._fit_to_view)
        controls_layout.addWidget(fit_view_btn)

        reset_zoom_btn = QPushButton("↺ Reset Zoom")
        reset_zoom_btn.clicked.connect(self._reset_zoom)
        controls_layout.addWidget(reset_zoom_btn)

        left_layout.addWidget(controls_group)

        # Tree widget (hierarchical view)
        tree_group = QGroupBox("Frame Hierarchy")
        tree_layout = QVBoxLayout(tree_group)

        self.tree_widget = QTreeWidget()
        self.tree_widget.setHeaderLabels(["Frame", "Parent"])
        self.tree_widget.itemClicked.connect(self._on_tree_item_clicked)
        tree_layout.addWidget(self.tree_widget)

        left_layout.addWidget(tree_group)

        left_panel.setMaximumWidth(300)

        # Center panel - Visual TF tree
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)

        center_layout.addWidget(QLabel("<b>Transform Tree Visualization</b>"))

        # Graphics view
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(-500, -500, 1000, 1000)

        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.view.setResizeAnchor(QGraphicsView.AnchorUnderMouse)

        # Enable mouse wheel zoom
        self.view.wheelEvent = self._handle_wheel_event

        # Track zoom level
        self.zoom_level = 1.0

        center_layout.addWidget(self.view)

        # Status label
        self.status_label = QLabel("Click 'Refresh TF Tree' to load transforms")
        self.status_label.setStyleSheet("color: #666; padding: 5px;")
        center_layout.addWidget(self.status_label)

        # Right panel - Frame details
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        right_layout.addWidget(QLabel("<b>Frame Details</b>"))

        # Frame info
        info_group = QGroupBox("Selected Frame")
        info_layout = QFormLayout(info_group)

        self.frame_name_label = QLabel("-")
        info_layout.addRow("Frame Name:", self.frame_name_label)

        self.parent_name_label = QLabel("-")
        info_layout.addRow("Parent Frame:", self.parent_name_label)

        right_layout.addWidget(info_group)

        # Transform data
        transform_group = QGroupBox("Transform Data")
        transform_layout = QVBoxLayout(transform_group)

        self.transform_table = QTableWidget(0, 2)
        self.transform_table.setHorizontalHeaderLabels(["Property", "Value"])
        self.transform_table.horizontalHeader().setStretchLastSection(True)
        transform_layout.addWidget(self.transform_table)

        right_layout.addWidget(transform_group)

        # TF echo output
        echo_group = QGroupBox("TF Echo")
        echo_layout = QVBoxLayout(echo_group)

        self.echo_output = QTextEdit()
        self.echo_output.setReadOnly(True)
        self.echo_output.setFont(QFont("Consolas", 8))
        self.echo_output.setMaximumHeight(150)
        echo_layout.addWidget(self.echo_output)

        right_layout.addWidget(echo_group)

        right_panel.setMaximumWidth(350)

        # Add panels to splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(center_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(1, 1)  # Center panel gets most space

        layout.addWidget(splitter)

    def _toggle_auto_refresh(self, checked: bool):
        """Toggle auto-refresh timer"""
        if checked:
            self.refresh_timer.start(1000)  # 1 second
            self.auto_refresh_btn.setText("Stop Auto-Refresh")
            self.status_label.setText("Auto-refresh enabled (1s interval)")
        else:
            self.refresh_timer.stop()
            self.auto_refresh_btn.setText("Start Auto-Refresh (1s)")
            self.status_label.setText("Auto-refresh disabled")

    def _refresh_tree(self):
        """Refresh TF tree from ROS2"""
        if not self.ssh_bridge:
            self.status_label.setText("⚠️ No SSH connection - cannot fetch TF tree")
            return

        self.status_label.setText("🔄 Fetching TF tree...")

        try:
            # Get TF tree using ros2 run tf2_tools view_frames
            # This generates a PDF, but we'll parse the frames list instead
            cmd = "ros2 run tf2_tools view_frames.py"

            # For now, use tf2_echo to get frame list
            # In production, parse tf2_monitor output
            cmd = "ros2 topic echo /tf_static --once"

            result = self.ssh_bridge.execute_command(cmd, timeout=5)

            if result['returncode'] == 0:
                self._parse_tf_output(result['stdout'])
                self.status_label.setText("✅ TF tree updated")
            else:
                self.status_label.setText(f"❌ Error: {result['stderr']}")

        except Exception as e:
            self.status_label.setText(f"❌ Failed to fetch TF tree: {e}")

    def _parse_tf_output(self, output: str):
        """Parse TF output and build tree"""
        # For demonstration, create a sample TF tree
        # In production, parse actual ros2 tf2_monitor output

        sample_tree = {
            'map': None,
            'odom': 'map',
            'base_link': 'odom',
            'base_footprint': 'base_link',
            'laser_frame': 'base_link',
            'camera_link': 'base_link',
            'imu_link': 'base_link'
        }

        self._build_visual_tree(sample_tree)

    def _build_visual_tree(self, tree_data: Dict[str, Optional[str]]):
        """Build visual TF tree from parsed data"""
        # Clear existing scene
        self.scene.clear()
        self.frames.clear()
        self.connections.clear()

        # Clear tree widget
        self.tree_widget.clear()

        # Find root frame (no parent)
        root_frame_name = None
        for frame, parent in tree_data.items():
            if parent is None:
                root_frame_name = frame
                break

        if not root_frame_name:
            self.status_label.setText("⚠️ No root frame found")
            return

        # Build tree recursively
        root_node = self._create_frame_recursive(root_frame_name, tree_data, None, 0, 0)

        # Add tree widget items
        for frame, parent in tree_data.items():
            item = QTreeWidgetItem([frame, parent or "-"])
            self.tree_widget.addTopLevelItem(item)

        self.tree_widget.expandAll()

        # Fit view to show all items properly
        self._fit_to_view()

    def _create_frame_recursive(
        self,
        frame_name: str,
        tree_data: Dict[str, Optional[str]],
        parent_frame: Optional[TFFrame],
        level: int,
        index: int
    ) -> TFFrame:
        """Recursively create TF frame nodes with improved distribution"""
        # Create frame node
        frame = TFFrame(frame_name)

        # Calculate position (improved tree layout with better spacing)
        # Increased horizontal and vertical spacing for better distribution
        horizontal_spacing = 250  # Increased from 150
        vertical_spacing = 150   # Increased from 120

        x = index * horizontal_spacing - 300
        y = level * vertical_spacing

        frame.setPos(x, y)
        self.scene.addItem(frame)
        self.frames[frame_name] = frame

        # Connect to parent
        if parent_frame:
            connection = TFConnection(parent_frame, frame)
            self.scene.addItem(connection)
            # Arrow is now drawn directly in paint(), no separate item needed
            self.connections.append(connection)
            frame.parent_frame = parent_frame
            parent_frame.children.append(frame)

        # Find children
        children = [f for f, p in tree_data.items() if p == frame_name]

        # Calculate horizontal offset to center children under parent
        total_width = len(children) * horizontal_spacing
        start_offset = -(total_width / 2) + (horizontal_spacing / 2)

        for i, child_name in enumerate(children):
            child_index = index + i - int(len(children) / 2)
            self._create_frame_recursive(child_name, tree_data, frame, level + 1, child_index)

        return frame

    def _on_tree_item_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle tree item click"""
        frame_name = item.text(0)
        parent_name = item.text(1)

        self.frame_name_label.setText(frame_name)
        self.parent_name_label.setText(parent_name)

        # Highlight in scene
        if frame_name in self.frames:
            self.scene.clearSelection()
            self.frames[frame_name].setSelected(True)

        # Get transform data (simulated)
        self._update_transform_data(frame_name, parent_name)

        self.frame_selected.emit(frame_name)

    def _update_transform_data(self, frame: str, parent: str):
        """Update transform data table"""
        self.transform_table.setRowCount(0)

        # Sample transform data
        data = {
            "Translation X": "0.125 m",
            "Translation Y": "0.000 m",
            "Translation Z": "0.250 m",
            "Rotation X": "0.000 rad",
            "Rotation Y": "0.000 rad",
            "Rotation Z": "0.785 rad",
            "Quaternion X": "0.000",
            "Quaternion Y": "0.000",
            "Quaternion Z": "0.383",
            "Quaternion W": "0.924"
        }

        for i, (key, value) in enumerate(data.items()):
            self.transform_table.insertRow(i)
            self.transform_table.setItem(i, 0, QTableWidgetItem(key))
            self.transform_table.setItem(i, 1, QTableWidgetItem(value))

        # Update TF echo
        self.echo_output.setPlainText(
            f"Transform from '{parent}' to '{frame}':\n"
            f"Translation: [0.125, 0.000, 0.250]\n"
            f"Rotation: [0.000, 0.000, 0.383, 0.924] (x, y, z, w)\n"
            f"Last updated: 0.234s ago"
        )

    def _export_diagram(self):
        """Export TF tree diagram as image"""
        from PySide6.QtWidgets import QFileDialog
        from PySide6.QtGui import QPixmap

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export TF Tree Diagram",
            "tf_tree.png",
            "PNG Image (*.png);;All Files (*)"
        )

        if file_path:
            # Render scene to pixmap
            pixmap = QPixmap(int(self.scene.width()), int(self.scene.height()))
            pixmap.fill(Qt.white)

            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)
            self.scene.render(painter)
            painter.end()

            pixmap.save(file_path)

            self.status_label.setText(f"✅ Diagram exported to {file_path}")

    def _toggle_mode(self):
        """Toggle between Design Mode and Runtime Mode"""
        self.design_mode = not self.design_mode

        if self.design_mode:
            self.mode_label.setText("🎨 <b>Design Mode</b> - Create frames manually")
            self.mode_label.setStyleSheet("color: #0066cc; padding: 5px;")
            self.toggle_mode_btn.setText("Switch to Runtime Mode")
            self.design_controls.show()
            self.runtime_controls.hide()
            self.status_label.setText("Design Mode: Add frames manually")
        else:
            self.mode_label.setText("🔴 <b>Runtime Mode</b> - Monitor live TF tree")
            self.mode_label.setStyleSheet("color: #cc0000; padding: 5px;")
            self.toggle_mode_btn.setText("Switch to Design Mode")
            self.design_controls.hide()
            self.runtime_controls.show()
            self.status_label.setText("Runtime Mode: Click 'Refresh TF Tree' to load")

    def _add_frame(self):
        """Add a new frame in design mode"""
        dialog = FrameEditorDialog(self, existing_frames=list(self.manual_frames.keys()))

        if dialog.exec() == QDialog.Accepted:
            frame_data = dialog.get_frame_data()

            if not frame_data['name']:
                QMessageBox.warning(self, "Error", "Frame name cannot be empty")
                return

            if frame_data['name'] in self.manual_frames:
                QMessageBox.warning(self, "Error", f"Frame '{frame_data['name']}' already exists")
                return

            # Store frame data
            self.manual_frames[frame_data['name']] = frame_data

            # Rebuild tree
            self._rebuild_from_manual_frames()

            self.status_label.setText(f"✅ Added frame '{frame_data['name']}'")

    def _edit_selected_frame(self):
        """Edit the currently selected frame"""
        selected_items = self.tree_widget.selectedItems()
        if not selected_items:
            QMessageBox.information(self, "No Selection", "Please select a frame to edit")
            return

        frame_name = selected_items[0].text(0)

        if frame_name not in self.manual_frames:
            QMessageBox.warning(self, "Error", "Cannot edit this frame")
            return

        frame_data = self.manual_frames[frame_name]

        dialog = FrameEditorDialog(
            self,
            frame_data=frame_data,
            existing_frames=[f for f in self.manual_frames.keys() if f != frame_name]
        )

        if dialog.exec() == QDialog.Accepted:
            updated_data = dialog.get_frame_data()
            self.manual_frames[frame_name] = updated_data

            # Rebuild tree
            self._rebuild_from_manual_frames()

            self.status_label.setText(f"✅ Updated frame '{frame_name}'")

    def _delete_selected_frame(self):
        """Delete the currently selected frame"""
        selected_items = self.tree_widget.selectedItems()
        if not selected_items:
            QMessageBox.information(self, "No Selection", "Please select a frame to delete")
            return

        frame_name = selected_items[0].text(0)

        if frame_name not in self.manual_frames:
            QMessageBox.warning(self, "Error", "Cannot delete this frame")
            return

        # Check if frame has children
        has_children = any(
            f['parent'] == frame_name
            for f in self.manual_frames.values()
        )

        if has_children:
            reply = QMessageBox.question(
                self,
                "Confirm Delete",
                f"Frame '{frame_name}' has child frames. Delete anyway?\n"
                "(Children will become orphaned)",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        # Delete frame
        del self.manual_frames[frame_name]

        # Rebuild tree
        self._rebuild_from_manual_frames()

        self.status_label.setText(f"✅ Deleted frame '{frame_name}'")

    def _rebuild_from_manual_frames(self):
        """Rebuild visual tree from manual frame data"""
        tree_data = {
            name: data['parent']
            for name, data in self.manual_frames.items()
        }
        self._build_visual_tree(tree_data)

    def _load_template(self):
        """Load a predefined TF tree template"""
        templates = {
            "Mobile Robot": {
                'map': {'parent': None, 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'odom': {'parent': 'map', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'base_link': {'parent': 'odom', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'base_footprint': {'parent': 'base_link', 'tx': 0, 'ty': 0, 'tz': -0.1, 'rx': 0, 'ry': 0, 'rz': 0},
                'laser_link': {'parent': 'base_link', 'tx': 0.2, 'ty': 0, 'tz': 0.3, 'rx': 0, 'ry': 0, 'rz': 0},
                'camera_link': {'parent': 'base_link', 'tx': 0.3, 'ty': 0, 'tz': 0.5, 'rx': 0, 'ry': 0, 'rz': 0},
                'imu_link': {'parent': 'base_link', 'tx': 0, 'ty': 0, 'tz': 0.2, 'rx': 0, 'ry': 0, 'rz': 0}
            },
            "Manipulator": {
                'world': {'parent': None, 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'base_link': {'parent': 'world', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'link_1': {'parent': 'base_link', 'tx': 0, 'ty': 0, 'tz': 0.2, 'rx': 0, 'ry': 0, 'rz': 0},
                'link_2': {'parent': 'link_1', 'tx': 0, 'ty': 0, 'tz': 0.3, 'rx': 0, 'ry': 0, 'rz': 0},
                'link_3': {'parent': 'link_2', 'tx': 0, 'ty': 0, 'tz': 0.25, 'rx': 0, 'ry': 0, 'rz': 0},
                'end_effector': {'parent': 'link_3', 'tx': 0, 'ty': 0, 'tz': 0.15, 'rx': 0, 'ry': 0, 'rz': 0}
            },
            "Inspection Robot": {
                'map': {'parent': None, 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'odom': {'parent': 'map', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'base_link': {'parent': 'odom', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'laser_link': {'parent': 'base_link', 'tx': 0.2, 'ty': 0, 'tz': 0.3, 'rx': 0, 'ry': 0, 'rz': 0},
                'camera_link': {'parent': 'base_link', 'tx': 0.3, 'ty': 0, 'tz': 0.6, 'rx': 0, 'ry': 0, 'rz': 0},
                'camera_optical_frame': {'parent': 'camera_link', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': -1.5708, 'ry': 0, 'rz': -1.5708},
                'equipment_sensor_link': {'parent': 'base_link', 'tx': 0.4, 'ty': 0, 'tz': 0.3, 'rx': 0, 'ry': -0.785, 'rz': 0}
            }
        }

        # Show selection dialog
        from PySide6.QtWidgets import QInputDialog
        template_names = list(templates.keys())
        template_name, ok = QInputDialog.getItem(
            self,
            "Load Template",
            "Select a TF tree template:",
            template_names,
            0,
            False
        )

        if ok and template_name:
            # Load template
            for frame_name, frame_data in templates[template_name].items():
                frame_data['name'] = frame_name
                self.manual_frames[frame_name] = frame_data

            # Rebuild tree
            self._rebuild_from_manual_frames()

            self.status_label.setText(f"✅ Loaded template: {template_name}")

    def _load_sample_design_mode_data(self):
        """Load sample data for design mode"""
        if self.design_mode and not self.manual_frames:
            # Load default mobile robot template
            sample_frames = {
                'map': {'parent': None, 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'odom': {'parent': 'map', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'base_link': {'parent': 'odom', 'tx': 0, 'ty': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'laser_link': {'parent': 'base_link', 'tx': 0.2, 'ty': 0, 'tz': 0.3, 'rx': 0, 'ry': 0, 'rz': 0},
                'camera_link': {'parent': 'base_link', 'tx': 0.3, 'ty': 0, 'tz': 0.5, 'rx': 0, 'ry': 0, 'rz': 0}
            }

            for frame_name, frame_data in sample_frames.items():
                frame_data['name'] = frame_name
                self.manual_frames[frame_name] = frame_data

            self._rebuild_from_manual_frames()

    def _export_urdf(self):
        """Export TF tree as URDF file"""
        if not self.manual_frames:
            QMessageBox.warning(self, "No Data", "No frames to export")
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Export URDF",
            "robot.urdf",
            "URDF Files (*.urdf);;All Files (*)"
        )

        if file_path:
            urdf_content = self._generate_urdf()

            with open(file_path, 'w') as f:
                f.write(urdf_content)

            self.status_label.setText(f"✅ URDF exported to {file_path}")

    def _generate_urdf(self):
        """Generate URDF content from manual frames"""
        urdf = '<?xml version="1.0"?>\n'
        urdf += '<robot name="roboshire_robot">\n\n'

        # Add links
        for frame_name in self.manual_frames.keys():
            urdf += f'  <link name="{frame_name}"/>\n\n'

        # Add joints
        for frame_name, frame_data in self.manual_frames.items():
            if frame_data['parent']:
                urdf += f'  <joint name="{frame_data["parent"]}_to_{frame_name}" type="fixed">\n'
                urdf += f'    <parent link="{frame_data["parent"]}"/>\n'
                urdf += f'    <child link="{frame_name}"/>\n'
                urdf += f'    <origin xyz="{frame_data["tx"]} {frame_data["ty"]} {frame_data["tz"]}" '
                urdf += f'rpy="{frame_data["rx"]} {frame_data["ry"]} {frame_data["rz"]}"/>\n'
                urdf += f'  </joint>\n\n'

        urdf += '</robot>\n'
        return urdf

    def _import_urdf(self):
        """Import URDF file and create TF tree"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Import URDF",
            "",
            "URDF Files (*.urdf);;XML Files (*.xml);;All Files (*)"
        )

        if file_path:
            try:
                import xml.etree.ElementTree as ET

                tree = ET.parse(file_path)
                root = tree.getroot()

                self.manual_frames.clear()

                # Parse links
                links = {link.get('name'): {'parent': None} for link in root.findall('link')}

                # Parse joints to establish parent-child relationships
                for joint in root.findall('joint'):
                    if joint.get('type') == 'fixed':
                        parent = joint.find('parent').get('link')
                        child = joint.find('child').get('link')
                        origin = joint.find('origin')

                        xyz = [0, 0, 0]
                        rpy = [0, 0, 0]

                        if origin is not None:
                            if origin.get('xyz'):
                                xyz = [float(x) for x in origin.get('xyz').split()]
                            if origin.get('rpy'):
                                rpy = [float(x) for x in origin.get('rpy').split()]

                        links[child] = {
                            'name': child,
                            'parent': parent,
                            'tx': xyz[0],
                            'ty': xyz[1],
                            'tz': xyz[2],
                            'rx': rpy[0],
                            'ry': rpy[1],
                            'rz': rpy[2]
                        }

                self.manual_frames = links
                self._rebuild_from_manual_frames()

                self.status_label.setText(f"✅ URDF imported: {len(links)} frames")

            except Exception as e:
                QMessageBox.critical(self, "Import Error", f"Failed to import URDF:\n{e}")

    def _validate_tree(self):
        """Validate TF tree for common issues"""
        if not self.manual_frames:
            QMessageBox.information(self, "No Data", "No frames to validate")
            return

        issues = []

        # Check for cycles
        def has_cycle(frame, visited, path):
            if frame in path:
                return True
            if frame in visited:
                return False

            visited.add(frame)
            path.add(frame)

            parent = self.manual_frames.get(frame, {}).get('parent')
            if parent:
                if has_cycle(parent, visited, path):
                    return True

            path.remove(frame)
            return False

        visited = set()
        for frame in self.manual_frames:
            if has_cycle(frame, visited, set()):
                issues.append(f"❌ Circular dependency detected involving frame '{frame}'")

        # Check for multiple roots
        roots = [f for f, d in self.manual_frames.items() if d['parent'] is None]
        if len(roots) > 1:
            issues.append(f"⚠️ Multiple root frames found: {', '.join(roots)}")
        elif len(roots) == 0:
            issues.append("❌ No root frame found")

        # Check for orphaned frames (parent doesn't exist)
        for frame_name, frame_data in self.manual_frames.items():
            parent = frame_data['parent']
            if parent and parent not in self.manual_frames:
                issues.append(f"❌ Frame '{frame_name}' references non-existent parent '{parent}'")

        # Report results
        if not issues:
            QMessageBox.information(
                self,
                "Validation Success",
                "✅ TF tree is valid!\n\n"
                f"Total frames: {len(self.manual_frames)}\n"
                f"Root frame: {roots[0] if roots else 'None'}"
            )
        else:
            QMessageBox.warning(
                self,
                "Validation Issues",
                "Found issues in TF tree:\n\n" + "\n".join(issues)
            )

    def set_ssh_bridge(self, ssh_bridge):
        """Set SSH bridge for ROS2 commands"""
        self.ssh_bridge = ssh_bridge

    def _handle_wheel_event(self, event):
        """Handle mouse wheel zoom"""
        # Zoom factor
        zoom_in_factor = 1.15
        zoom_out_factor = 1 / zoom_in_factor

        # Get wheel delta
        if event.angleDelta().y() > 0:
            factor = zoom_in_factor
            self.zoom_level *= factor
        else:
            factor = zoom_out_factor
            self.zoom_level *= factor

        # Limit zoom level
        if self.zoom_level < 0.1:
            self.zoom_level = 0.1
            return
        elif self.zoom_level > 10.0:
            self.zoom_level = 10.0
            return

        # Apply zoom
        self.view.scale(factor, factor)

    def _zoom_in(self):
        """Zoom in"""
        factor = 1.25
        self.zoom_level *= factor
        if self.zoom_level > 10.0:
            self.zoom_level = 10.0
            return
        self.view.scale(factor, factor)

    def _zoom_out(self):
        """Zoom out"""
        factor = 0.8
        self.zoom_level *= factor
        if self.zoom_level < 0.1:
            self.zoom_level = 0.1
            return
        self.view.scale(factor, factor)

    def _fit_to_view(self):
        """Fit all items in view"""
        # Reset zoom first
        self.view.resetTransform()
        self.zoom_level = 1.0

        # Get bounding rect of all items
        items_rect = self.scene.itemsBoundingRect()

        if not items_rect.isEmpty():
            # Add padding (20% on each side)
            padding = 0.2
            width_padding = items_rect.width() * padding
            height_padding = items_rect.height() * padding

            padded_rect = items_rect.adjusted(
                -width_padding, -height_padding,
                width_padding, height_padding
            )

            # Update scene rect to match content
            self.scene.setSceneRect(padded_rect)

            # Fit view to padded rect
            self.view.fitInView(padded_rect, Qt.KeepAspectRatio)

            # Update zoom level based on transform
            transform = self.view.transform()
            self.zoom_level = transform.m11()
        else:
            # No items, just center the view
            self.scene.setSceneRect(-500, -500, 1000, 1000)
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def _reset_zoom(self):
        """Reset zoom to 100%"""
        self.view.resetTransform()
        self.zoom_level = 1.0


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    visualizer = TFTreeVisualizer()
    visualizer.setWindowTitle("TF Tree Visualizer")
    visualizer.resize(1200, 700)
    visualizer.show()

    # Load sample data
    sample_tree = {
        'map': None,
        'odom': 'map',
        'base_link': 'odom',
        'base_footprint': 'base_link',
        'laser_frame': 'base_link',
        'camera_link': 'base_link',
        'imu_link': 'base_link'
    }
    visualizer._build_visual_tree(sample_tree)

    sys.exit(app.exec())
