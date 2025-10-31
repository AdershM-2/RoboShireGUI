"""
TF Visualizer Widget

Simple GUI for viewing TF frames and transforms.

Author: RoboShire Team
Phase: 6.4 (TF Visualization)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QTreeWidget, QTreeWidgetItem, QGroupBox, QTextEdit,
    QComboBox, QSplitter
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from typing import Optional
import logging

from roboshire.integrations.tf_manager import TFManager, FrameInfo, Transform


class TFVisualizerWidget(QWidget):
    """
    Widget for visualizing TF frames and transforms

    Features:
    - Tree view of frame hierarchy
    - Transform inspector
    - Auto-refresh capability
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.tf_manager: Optional[TFManager] = None
        self.frames = {}
        self.logger = logging.getLogger(__name__)

        self._init_ui()

        # Auto-refresh timer
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_frames)

    def _init_ui(self):
        """Initialize UI"""
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Title
        title_label = QLabel("TF Frame Viewer")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        main_layout.addWidget(title_label)

        subtitle_label = QLabel("View coordinate frame relationships and transforms")
        subtitle_label.setStyleSheet("color: #666;")
        main_layout.addWidget(subtitle_label)

        # Control bar
        control_layout = QHBoxLayout()

        self.refresh_btn = QPushButton("Refresh Frames")
        self.refresh_btn.clicked.connect(self.refresh_frames)
        control_layout.addWidget(self.refresh_btn)

        self.auto_refresh_btn = QPushButton("Auto-Refresh: OFF")
        self.auto_refresh_btn.setCheckable(True)
        self.auto_refresh_btn.clicked.connect(self._toggle_auto_refresh)
        control_layout.addWidget(self.auto_refresh_btn)

        control_layout.addStretch()

        self.status_label = QLabel("Not connected")
        self.status_label.setStyleSheet("color: #666;")
        control_layout.addWidget(self.status_label)

        main_layout.addLayout(control_layout)

        # Splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left: Frame tree
        tree_widget = QWidget()
        tree_layout = QVBoxLayout(tree_widget)
        tree_layout.setContentsMargins(0, 0, 0, 0)

        tree_label = QLabel("TF Frame Tree")
        tree_label.setStyleSheet("font-weight: bold;")
        tree_layout.addWidget(tree_label)

        self.frame_tree = QTreeWidget()
        self.frame_tree.setHeaderLabels(["Frame", "Parent"])
        self.frame_tree.setColumnWidth(0, 200)
        self.frame_tree.itemSelectionChanged.connect(self._on_frame_selected)
        tree_layout.addWidget(self.frame_tree)

        splitter.addWidget(tree_widget)

        # Right: Transform inspector
        inspector_widget = QWidget()
        inspector_layout = QVBoxLayout(inspector_widget)
        inspector_layout.setContentsMargins(0, 0, 0, 0)

        # Transform query section
        query_group = QGroupBox("Transform Query")
        query_layout = QVBoxLayout(query_group)

        source_layout = QHBoxLayout()
        source_layout.addWidget(QLabel("Source Frame:"))
        self.source_combo = QComboBox()
        self.source_combo.setEditable(True)
        source_layout.addWidget(self.source_combo)
        query_layout.addLayout(source_layout)

        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target Frame:"))
        self.target_combo = QComboBox()
        self.target_combo.setEditable(True)
        target_layout.addWidget(self.target_combo)
        query_layout.addLayout(target_layout)

        self.query_btn = QPushButton("Get Transform")
        self.query_btn.clicked.connect(self._on_query_transform)
        query_layout.addWidget(self.query_btn)

        inspector_layout.addWidget(query_group)

        # Transform display
        transform_group = QGroupBox("Transform")
        transform_layout = QVBoxLayout(transform_group)

        self.transform_text = QTextEdit()
        self.transform_text.setReadOnly(True)
        self.transform_text.setMaximumHeight(300)
        self.transform_text.setPlaceholderText("Select frames and click 'Get Transform' to view transform data")
        transform_layout.addWidget(self.transform_text)

        inspector_layout.addWidget(transform_group)
        inspector_layout.addStretch()

        splitter.addWidget(inspector_widget)

        splitter.setSizes([400, 400])

        main_layout.addWidget(splitter)

    def set_tf_manager(self, tf_manager: TFManager):
        """Set TF manager"""
        self.tf_manager = tf_manager
        self.status_label.setText("Connected - Click Refresh")
        self.refresh_btn.setEnabled(True)
        self.auto_refresh_btn.setEnabled(True)

    def refresh_frames(self):
        """Refresh frame list"""
        if not self.tf_manager:
            self.status_label.setText("Not connected to ROS2")
            return

        self.status_label.setText("Refreshing...")

        try:
            # Get frame tree
            self.frames = self.tf_manager.get_frame_tree()

            if not self.frames:
                # Fallback: just list frames without relationships
                frame_list = self.tf_manager.list_frames()
                self.frames = {name: FrameInfo(name=name) for name in frame_list}

            # Update tree
            self._populate_tree()

            # Update combos
            frame_names = sorted(self.frames.keys())
            self.source_combo.clear()
            self.source_combo.addItems(frame_names)
            self.target_combo.clear()
            self.target_combo.addItems(frame_names)

            self.status_label.setText(f"Found {len(self.frames)} frames")

        except Exception as e:
            self.logger.error(f"Failed to refresh frames: {e}")
            self.status_label.setText(f"Error: {e}")

    def _populate_tree(self):
        """Populate frame tree"""
        self.frame_tree.clear()

        if not self.frames:
            item = QTreeWidgetItem(["No frames available", ""])
            self.frame_tree.addTopLevelItem(item)
            return

        # Build tree structure
        root_frames = [f for f in self.frames.values() if not f.parent]
        child_map = {}

        for frame in self.frames.values():
            if frame.parent:
                if frame.parent not in child_map:
                    child_map[frame.parent] = []
                child_map[frame.parent].append(frame)

        # Add root frames
        for frame in root_frames:
            self._add_frame_item(frame, child_map, None)

        # Expand all
        self.frame_tree.expandAll()

    def _add_frame_item(self, frame: FrameInfo, child_map: dict, parent_item: Optional[QTreeWidgetItem]):
        """Add frame item to tree recursively"""
        parent_text = frame.parent if frame.parent else "(root)"
        item = QTreeWidgetItem([frame.name, parent_text])

        if parent_item:
            parent_item.addChild(item)
        else:
            self.frame_tree.addTopLevelItem(item)

        # Add children
        if frame.name in child_map:
            for child in child_map[frame.name]:
                self._add_frame_item(child, child_map, item)

    def _on_frame_selected(self):
        """Handle frame selection"""
        selected = self.frame_tree.selectedItems()
        if not selected:
            return

        frame_name = selected[0].text(0)

        # Set as target in combo
        index = self.target_combo.findText(frame_name)
        if index >= 0:
            self.target_combo.setCurrentIndex(index)

    def _on_query_transform(self):
        """Query transform between selected frames"""
        if not self.tf_manager:
            return

        source = self.source_combo.currentText()
        target = self.target_combo.currentText()

        if not source or not target:
            self.transform_text.setPlainText("Please select both source and target frames")
            return

        if source == target:
            self.transform_text.setPlainText("Source and target frames are the same (identity transform)")
            return

        self.transform_text.setPlainText(f"Querying transform {source} -> {target}...")

        try:
            transform = self.tf_manager.get_transform(source, target)

            if transform:
                self._display_transform(transform)
            else:
                self.transform_text.setPlainText(
                    f"Transform not available between:\n"
                    f"  Source: {source}\n"
                    f"  Target: {target}\n\n"
                    f"Possible reasons:\n"
                    f"- Frames not connected in TF tree\n"
                    f"- Transform not being published\n"
                    f"- Nodes not running"
                )

        except Exception as e:
            self.logger.error(f"Failed to query transform: {e}")
            self.transform_text.setPlainText(f"Error querying transform:\n\n{e}")

    def _display_transform(self, transform: Transform):
        """Display transform information"""
        lines = []

        lines.append(f"Transform: {transform.source_frame} -> {transform.target_frame}")
        lines.append("")

        # Translation
        lines.append("Translation:")
        lines.append(f"  x: {transform.translation[0]:.4f} m")
        lines.append(f"  y: {transform.translation[1]:.4f} m")
        lines.append(f"  z: {transform.translation[2]:.4f} m")
        lines.append("")

        # Rotation (quaternion)
        lines.append("Rotation (Quaternion):")
        lines.append(f"  x: {transform.rotation[0]:.4f}")
        lines.append(f"  y: {transform.rotation[1]:.4f}")
        lines.append(f"  z: {transform.rotation[2]:.4f}")
        lines.append(f"  w: {transform.rotation[3]:.4f}")
        lines.append("")

        # Convert to Euler angles (approximate)
        import math
        x, y, z, w = transform.rotation

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        lines.append("Rotation (Euler - RPY):")
        lines.append(f"  Roll:  {math.degrees(roll):.2f}°")
        lines.append(f"  Pitch: {math.degrees(pitch):.2f}°")
        lines.append(f"  Yaw:   {math.degrees(yaw):.2f}°")

        self.transform_text.setPlainText("\n".join(lines))

    def _toggle_auto_refresh(self, checked: bool):
        """Toggle auto-refresh"""
        if checked:
            self.refresh_timer.start(5000)  # 5 seconds
            self.auto_refresh_btn.setText("Auto-Refresh: ON")
            self.refresh_frames()  # Refresh immediately
        else:
            self.refresh_timer.stop()
            self.auto_refresh_btn.setText("Auto-Refresh: OFF")

    def clear(self):
        """Clear display"""
        self.frame_tree.clear()
        self.transform_text.clear()
        self.source_combo.clear()
        self.target_combo.clear()
        self.status_label.setText("Not connected")
        self.refresh_timer.stop()
        self.auto_refresh_btn.setChecked(False)
        self.auto_refresh_btn.setText("Auto-Refresh: OFF")
