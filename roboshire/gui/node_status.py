"""
Node Status Widget - Display node health status with visual indicators

Shows a list of monitored nodes with:
- Color-coded status (Green=Healthy, Yellow=Warning, Red=Crashed)
- Uptime display
- Crash/restart counts
- Auto-restart toggle
- Manual restart button
"""

import logging
from typing import Optional, Dict
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLabel, QHeaderView, QCheckBox, QGroupBox
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QBrush

from roboshire.integrations.node_monitor import NodeMonitor, NodeHealth, NodeHealthStatus


class NodeStatusWidget(QWidget):
    """
    Widget to display node health status

    Features:
    - Table view of all monitored nodes
    - Color-coded health indicators
    - Uptime display
    - Auto-restart toggle per node
    - Manual restart button
    - Statistics summary
    """

    # Signals
    restart_node_requested = Signal(str)  # node_name
    auto_restart_toggled = Signal(str, bool)  # node_name, enabled

    def __init__(self, node_monitor: Optional[NodeMonitor] = None, parent=None):
        """
        Initialize node status widget

        Args:
            node_monitor: NodeMonitor instance (optional, can be set later)
            parent: Parent widget
        """
        super().__init__(parent)

        self.node_monitor = node_monitor
        self.node_rows: Dict[str, int] = {}  # node_name -> row_index

        self._init_ui()

        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_display)
        self.update_timer.start(1000)  # Update every second

        logging.info("NodeStatusWidget initialized")

    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()

        # Title and stats
        header_layout = QHBoxLayout()

        title_label = QLabel("<h3>Node Health Monitor</h3>")
        header_layout.addWidget(title_label)

        header_layout.addStretch()

        self.stats_label = QLabel("No nodes monitored")
        header_layout.addWidget(self.stats_label)

        layout.addLayout(header_layout)

        # Node table
        self.node_table = QTableWidget()
        self.node_table.setColumnCount(7)
        self.node_table.setHorizontalHeaderLabels([
            "Node Name",
            "Status",
            "Uptime",
            "Crashes",
            "Restarts",
            "Auto-Restart",
            "Actions"
        ])

        # Configure table
        header = self.node_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)  # Node name stretches
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(6, QHeaderView.ResizeToContents)

        self.node_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.node_table.setSelectionMode(QTableWidget.SingleSelection)
        self.node_table.setAlternatingRowColors(True)

        layout.addWidget(self.node_table)

        # Legend
        legend_group = QGroupBox("Status Legend")
        legend_layout = QHBoxLayout()

        legend_items = [
            ("Healthy", QColor(0, 200, 0)),
            ("Warning", QColor(255, 165, 0)),
            ("Crashed", QColor(255, 50, 50)),
            ("Restarting", QColor(100, 100, 255)),
            ("Unknown", QColor(128, 128, 128))
        ]

        for status_text, color in legend_items:
            label = QLabel(f"● {status_text}")
            label.setStyleSheet(f"color: rgb({color.red()}, {color.green()}, {color.blue()}); font-weight: bold;")
            legend_layout.addWidget(label)

        legend_layout.addStretch()
        legend_group.setLayout(legend_layout)

        layout.addWidget(legend_group)

        self.setLayout(layout)

    def set_node_monitor(self, node_monitor: NodeMonitor):
        """
        Set the node monitor instance

        Args:
            node_monitor: NodeMonitor instance
        """
        self.node_monitor = node_monitor
        self._update_display()
        logging.info("NodeMonitor attached to NodeStatusWidget")

    def _update_display(self):
        """Update the display with current node statuses"""
        if not self.node_monitor:
            return

        # Get all node statuses
        statuses = self.node_monitor.get_all_statuses()

        # Update stats label
        stats = self.node_monitor.get_stats()
        self.stats_label.setText(
            f"Monitored: {stats.total_nodes_monitored} | "
            f"Healthy: {stats.healthy_nodes} | "
            f"Crashed: {stats.crashed_nodes} | "
            f"Total Crashes: {stats.total_crashes_detected}"
        )

        # Update table
        current_nodes = set(statuses.keys())
        displayed_nodes = set(self.node_rows.keys())

        # Add new nodes
        for node_name in current_nodes - displayed_nodes:
            self._add_node_row(node_name, statuses[node_name])

        # Remove deleted nodes
        for node_name in displayed_nodes - current_nodes:
            self._remove_node_row(node_name)

        # Update existing nodes
        for node_name in current_nodes & displayed_nodes:
            self._update_node_row(node_name, statuses[node_name])

    def _add_node_row(self, node_name: str, status: NodeHealthStatus):
        """
        Add a new node row to the table

        Args:
            node_name: Node name
            status: NodeHealthStatus object
        """
        row = self.node_table.rowCount()
        self.node_table.insertRow(row)
        self.node_rows[node_name] = row

        # Node name
        name_item = QTableWidgetItem(node_name)
        name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
        self.node_table.setItem(row, 0, name_item)

        # Status (will be updated)
        status_item = QTableWidgetItem()
        status_item.setFlags(status_item.flags() & ~Qt.ItemIsEditable)
        self.node_table.setItem(row, 1, status_item)

        # Uptime
        uptime_item = QTableWidgetItem()
        uptime_item.setFlags(uptime_item.flags() & ~Qt.ItemIsEditable)
        uptime_item.setTextAlignment(Qt.AlignCenter)
        self.node_table.setItem(row, 2, uptime_item)

        # Crashes
        crashes_item = QTableWidgetItem()
        crashes_item.setFlags(crashes_item.flags() & ~Qt.ItemIsEditable)
        crashes_item.setTextAlignment(Qt.AlignCenter)
        self.node_table.setItem(row, 3, crashes_item)

        # Restarts
        restarts_item = QTableWidgetItem()
        restarts_item.setFlags(restarts_item.flags() & ~Qt.ItemIsEditable)
        restarts_item.setTextAlignment(Qt.AlignCenter)
        self.node_table.setItem(row, 4, restarts_item)

        # Auto-restart checkbox
        auto_restart_widget = QWidget()
        auto_restart_layout = QHBoxLayout()
        auto_restart_layout.setContentsMargins(0, 0, 0, 0)
        auto_restart_layout.setAlignment(Qt.AlignCenter)

        auto_restart_checkbox = QCheckBox()
        auto_restart_checkbox.setChecked(status.auto_restart_enabled)
        auto_restart_checkbox.stateChanged.connect(
            lambda state, name=node_name: self._on_auto_restart_toggled(name, state == Qt.Checked)
        )

        auto_restart_layout.addWidget(auto_restart_checkbox)
        auto_restart_widget.setLayout(auto_restart_layout)
        self.node_table.setCellWidget(row, 5, auto_restart_widget)

        # Restart button
        button_widget = QWidget()
        button_layout = QHBoxLayout()
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.setAlignment(Qt.AlignCenter)

        restart_button = QPushButton("Restart")
        restart_button.clicked.connect(lambda checked, name=node_name: self._on_restart_clicked(name))

        button_layout.addWidget(restart_button)
        button_widget.setLayout(button_layout)
        self.node_table.setCellWidget(row, 6, button_widget)

        # Update with current status
        self._update_node_row(node_name, status)

    def _update_node_row(self, node_name: str, status: NodeHealthStatus):
        """
        Update an existing node row

        Args:
            node_name: Node name
            status: NodeHealthStatus object
        """
        if node_name not in self.node_rows:
            return

        row = self.node_rows[node_name]

        # Status
        status_item = self.node_table.item(row, 1)
        status_text = status.health.value.upper()
        status_item.setText(status_text)

        # Set color based on health
        color = self._get_status_color(status.health)
        status_item.setForeground(QBrush(color))
        status_item.setBackground(QBrush(QColor(color.red(), color.green(), color.blue(), 30)))

        # Uptime
        uptime_item = self.node_table.item(row, 2)
        uptime_text = self._format_uptime(status.uptime_seconds)
        uptime_item.setText(uptime_text)

        # Crashes
        crashes_item = self.node_table.item(row, 3)
        crashes_item.setText(str(status.crash_count))
        if status.crash_count > 0:
            crashes_item.setForeground(QBrush(QColor(255, 50, 50)))

        # Restarts
        restarts_item = self.node_table.item(row, 4)
        restarts_item.setText(str(status.restart_count))
        if status.restart_count > 0:
            restarts_item.setForeground(QBrush(QColor(100, 100, 255)))

        # Update auto-restart checkbox
        auto_restart_widget = self.node_table.cellWidget(row, 5)
        if auto_restart_widget:
            checkbox = auto_restart_widget.findChild(QCheckBox)
            if checkbox and checkbox.isChecked() != status.auto_restart_enabled:
                checkbox.blockSignals(True)
                checkbox.setChecked(status.auto_restart_enabled)
                checkbox.blockSignals(False)

    def _remove_node_row(self, node_name: str):
        """
        Remove a node row from the table

        Args:
            node_name: Node name
        """
        if node_name not in self.node_rows:
            return

        row = self.node_rows[node_name]
        self.node_table.removeRow(row)

        # Update row indices
        del self.node_rows[node_name]
        for name, idx in list(self.node_rows.items()):
            if idx > row:
                self.node_rows[name] = idx - 1

    def _get_status_color(self, health: NodeHealth) -> QColor:
        """
        Get color for health status

        Args:
            health: NodeHealth enum

        Returns:
            QColor for the status
        """
        color_map = {
            NodeHealth.HEALTHY: QColor(0, 200, 0),
            NodeHealth.WARNING: QColor(255, 165, 0),
            NodeHealth.CRASHED: QColor(255, 50, 50),
            NodeHealth.RESTARTING: QColor(100, 100, 255),
            NodeHealth.UNKNOWN: QColor(128, 128, 128)
        }
        return color_map.get(health, QColor(128, 128, 128))

    def _format_uptime(self, seconds: float) -> str:
        """
        Format uptime in human-readable form

        Args:
            seconds: Uptime in seconds

        Returns:
            Formatted string (e.g., "1h 23m 45s")
        """
        if seconds < 1:
            return "0s"

        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)

        parts = []
        if hours > 0:
            parts.append(f"{hours}h")
        if minutes > 0:
            parts.append(f"{minutes}m")
        if secs > 0 or not parts:
            parts.append(f"{secs}s")

        return " ".join(parts)

    def _on_auto_restart_toggled(self, node_name: str, enabled: bool):
        """
        Handle auto-restart checkbox toggle

        Args:
            node_name: Node name
            enabled: Whether auto-restart is enabled
        """
        logging.info(f"Auto-restart toggled for {node_name}: {enabled}")
        self.auto_restart_toggled.emit(node_name, enabled)

    def _on_restart_clicked(self, node_name: str):
        """
        Handle restart button click

        Args:
            node_name: Node name
        """
        logging.info(f"Manual restart requested for {node_name}")
        self.restart_node_requested.emit(node_name)

    def clear(self):
        """Clear all node rows"""
        self.node_table.setRowCount(0)
        self.node_rows.clear()
        self.stats_label.setText("No nodes monitored")
