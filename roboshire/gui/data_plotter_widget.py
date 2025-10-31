"""
Data Plotter Widget - Embedded Real-Time Visualization

Alternative to external PlotJuggler with embedded pyqtgraph plots.
Shows live topic data with multiple subplots, zoom, pan, and export.

Author: RoboShire Team
Phase: 10 (UX Enhancement)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox,
    QLabel, QSplitter, QListWidget, QGroupBox, QCheckBox,
    QSpinBox, QDoubleSpinBox, QMessageBox, QFileDialog
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor
import pyqtgraph as pg
from typing import Dict, List, Optional, Any
from collections import deque
from datetime import datetime
import csv


class TopicPlot:
    """
    Single plot for one topic field
    """
    def __init__(self, plot_item: pg.PlotItem, topic_name: str, field_name: str, color: QColor):
        self.plot_item = plot_item
        self.topic_name = topic_name
        self.field_name = field_name
        self.color = color

        # Data buffers (circular)
        self.max_points = 1000
        self.timestamps = deque(maxlen=self.max_points)
        self.values = deque(maxlen=self.max_points)

        # Plot curve
        self.curve = self.plot_item.plot(
            pen=pg.mkPen(color=color, width=2),
            name=f"{topic_name}/{field_name}"
        )

    def add_data_point(self, timestamp: float, value: float):
        """Add new data point"""
        self.timestamps.append(timestamp)
        self.values.append(value)
        self.curve.setData(list(self.timestamps), list(self.values))

    def clear(self):
        """Clear all data"""
        self.timestamps.clear()
        self.values.clear()
        self.curve.setData([], [])

    def export_csv(self, file_path: str):
        """Export data to CSV"""
        with open(file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', self.field_name])
            for t, v in zip(self.timestamps, self.values):
                writer.writerow([t, v])


class DataPlotterWidget(QWidget):
    """
    Embedded data plotting widget with real-time visualization
    """

    # Signal when topic subscription is requested
    subscribe_topic_requested = Signal(str)  # topic_name
    unsubscribe_topic_requested = Signal(str)  # topic_name

    def __init__(self, parent=None):
        super().__init__(parent)

        self.topic_inspector = None  # TopicInspector instance (set externally)

        # Active plots {topic_name: {field_name: TopicPlot}}
        self.active_plots: Dict[str, Dict[str, TopicPlot]] = {}

        # Plot colors (cycle through these)
        self.color_palette = [
            QColor(31, 119, 180),   # Blue
            QColor(255, 127, 14),   # Orange
            QColor(44, 160, 44),    # Green
            QColor(214, 39, 40),    # Red
            QColor(148, 103, 189),  # Purple
            QColor(140, 86, 75),    # Brown
            QColor(227, 119, 194),  # Pink
            QColor(127, 127, 127),  # Gray
        ]
        self.color_index = 0

        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_plots)
        self.update_interval_ms = 100  # 10 Hz update rate

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Top toolbar
        toolbar = self._create_toolbar()
        layout.addWidget(toolbar)

        # Main splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left: Available topics list
        left_panel = self._create_topic_list_panel()
        splitter.addWidget(left_panel)

        # Right: Plot area
        right_panel = self._create_plot_area()
        splitter.addWidget(right_panel)

        splitter.setSizes([250, 750])
        layout.addWidget(splitter)

    def _create_toolbar(self) -> QWidget:
        """Create top toolbar"""
        toolbar = QWidget()
        layout = QHBoxLayout(toolbar)
        layout.setContentsMargins(0, 0, 0, 5)

        # Title
        title = QLabel("📊 Real-Time Data Plotter")
        title_font = title.font()
        title_font.setBold(True)
        title_font.setPointSize(12)
        title.setFont(title_font)
        layout.addWidget(title)

        layout.addStretch()

        # Controls
        self.start_btn = QPushButton("▶ Start")
        self.start_btn.clicked.connect(self._start_plotting)
        layout.addWidget(self.start_btn)

        self.pause_btn = QPushButton("⏸ Pause")
        self.pause_btn.clicked.connect(self._pause_plotting)
        self.pause_btn.setEnabled(False)
        layout.addWidget(self.pause_btn)

        self.clear_btn = QPushButton("🗑 Clear")
        self.clear_btn.clicked.connect(self._clear_all_plots)
        layout.addWidget(self.clear_btn)

        self.export_btn = QPushButton("💾 Export CSV")
        self.export_btn.clicked.connect(self._export_data)
        layout.addWidget(self.export_btn)

        return toolbar

    def _create_topic_list_panel(self) -> QWidget:
        """Create topic list panel"""
        panel = QGroupBox("Available Topics")
        layout = QVBoxLayout(panel)

        # Refresh button
        refresh_btn = QPushButton("🔄 Refresh Topics")
        refresh_btn.clicked.connect(self._refresh_topics)
        layout.addWidget(refresh_btn)

        # Topics list
        self.topics_list = QListWidget()
        self.topics_list.itemDoubleClicked.connect(self._on_topic_selected)
        layout.addWidget(self.topics_list)

        # Add topic button
        add_btn = QPushButton("➕ Add Selected Topic")
        add_btn.clicked.connect(self._add_selected_topic)
        layout.addWidget(add_btn)

        # Active plots label
        active_label = QLabel("Active Plots:")
        active_label_font = active_label.font()
        active_label_font.setBold(True)
        active_label.setFont(active_label_font)
        layout.addWidget(active_label)

        # Active plots list
        self.active_plots_list = QListWidget()
        self.active_plots_list.itemDoubleClicked.connect(self._on_active_plot_clicked)
        layout.addWidget(self.active_plots_list)

        # Remove plot button
        remove_btn = QPushButton("➖ Remove Selected")
        remove_btn.clicked.connect(self._remove_selected_plot)
        layout.addWidget(remove_btn)

        # Settings
        settings_group = QGroupBox("Settings")
        settings_layout = QVBoxLayout(settings_group)

        # Buffer size
        buffer_layout = QHBoxLayout()
        buffer_layout.addWidget(QLabel("Buffer Size:"))
        self.buffer_size_spin = QSpinBox()
        self.buffer_size_spin.setRange(100, 10000)
        self.buffer_size_spin.setValue(1000)
        self.buffer_size_spin.valueChanged.connect(self._on_buffer_size_changed)
        buffer_layout.addWidget(self.buffer_size_spin)
        settings_layout.addLayout(buffer_layout)

        # Update rate
        rate_layout = QHBoxLayout()
        rate_layout.addWidget(QLabel("Update Rate (Hz):"))
        self.update_rate_spin = QSpinBox()
        self.update_rate_spin.setRange(1, 60)
        self.update_rate_spin.setValue(10)
        self.update_rate_spin.valueChanged.connect(self._on_update_rate_changed)
        rate_layout.addWidget(self.update_rate_spin)
        settings_layout.addLayout(rate_layout)

        layout.addWidget(settings_group)

        return panel

    def _create_plot_area(self) -> QWidget:
        """Create plot area with pyqtgraph"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        # PyQtGraph widget
        self.graphics_layout = pg.GraphicsLayoutWidget()
        self.graphics_layout.setBackground('w')
        layout.addWidget(self.graphics_layout)

        # Add initial plot
        self.main_plot = self.graphics_layout.addPlot(row=0, col=0)
        self.main_plot.setLabel('left', 'Value')
        self.main_plot.setLabel('bottom', 'Time (s)')
        self.main_plot.showGrid(x=True, y=True, alpha=0.3)
        self.main_plot.addLegend()

        return panel

    def set_topic_inspector(self, inspector):
        """Set topic inspector for fetching topic data"""
        self.topic_inspector = inspector

    def _refresh_topics(self):
        """Refresh available topics list"""
        if not self.topic_inspector:
            QMessageBox.warning(
                self,
                "No Topic Inspector",
                "Topic inspector not initialized. Please launch ROS2 nodes first."
            )
            return

        try:
            topics = self.topic_inspector.list_topics()
            self.topics_list.clear()

            for topic_info in topics:
                topic_name = topic_info.get('name', 'unknown')
                topic_type = topic_info.get('type', 'unknown')
                self.topics_list.addItem(f"{topic_name} ({topic_type})")

        except Exception as e:
            QMessageBox.critical(
                self,
                "Refresh Error",
                f"Failed to refresh topics:\n\n{e}"
            )

    def _on_topic_selected(self, item):
        """Handle topic double-click"""
        # Extract topic name from "topic_name (type)" format
        text = item.text()
        topic_name = text.split(' (')[0] if ' (' in text else text
        self._add_topic_plot(topic_name)

    def _add_selected_topic(self):
        """Add selected topic to plots"""
        current_item = self.topics_list.currentItem()
        if not current_item:
            QMessageBox.information(
                self,
                "No Selection",
                "Please select a topic from the list."
            )
            return

        text = current_item.text()
        topic_name = text.split(' (')[0] if ' (' in text else text
        self._add_topic_plot(topic_name)

    def _add_topic_plot(self, topic_name: str):
        """Add plot for topic"""
        if topic_name in self.active_plots:
            QMessageBox.information(
                self,
                "Already Plotted",
                f"Topic {topic_name} is already being plotted."
            )
            return

        # Get next color
        color = self.color_palette[self.color_index % len(self.color_palette)]
        self.color_index += 1

        # For now, plot a single field (could expand to multiple fields)
        # Assume numeric topics like sensor_msgs/Temperature, std_msgs/Float64, etc.
        field_name = "data"  # Common field name

        # Create plot
        plot = TopicPlot(self.main_plot, topic_name, field_name, color)
        self.active_plots[topic_name] = {field_name: plot}

        # Update active plots list
        self.active_plots_list.addItem(f"📈 {topic_name}/{field_name}")

        # Request subscription
        self.subscribe_topic_requested.emit(topic_name)

        # Start timer if not running
        if not self.update_timer.isActive():
            self._start_plotting()

    def _remove_selected_plot(self):
        """Remove selected plot"""
        current_item = self.active_plots_list.currentItem()
        if not current_item:
            return

        text = current_item.text().replace("📈 ", "")
        topic_name = text.split('/')[0]

        if topic_name in self.active_plots:
            # Clear plot data
            for plot in self.active_plots[topic_name].values():
                plot.clear()

            del self.active_plots[topic_name]

            # Remove from list
            self.active_plots_list.takeItem(self.active_plots_list.currentRow())

            # Unsubscribe
            self.unsubscribe_topic_requested.emit(topic_name)

    def _on_active_plot_clicked(self, item):
        """Handle active plot click (could show properties)"""
        pass

    def _start_plotting(self):
        """Start real-time plotting"""
        self.update_timer.start(self.update_interval_ms)
        self.start_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)

    def _pause_plotting(self):
        """Pause plotting"""
        self.update_timer.stop()
        self.start_btn.setEnabled(True)
        self.pause_btn.setEnabled(False)

    def _update_plots(self):
        """Update all plots with new data"""
        if not self.topic_inspector:
            return

        current_time = datetime.now().timestamp()

        for topic_name, plots in self.active_plots.items():
            try:
                # Echo topic to get latest message
                message = self.topic_inspector.echo_topic(topic_name, count=1)

                if message:
                    # Parse message for numeric fields
                    # Simple heuristic: look for 'data' field or first numeric field
                    value = self._extract_numeric_value(message)

                    if value is not None:
                        for plot in plots.values():
                            plot.add_data_point(current_time, value)

            except Exception:
                # Silently continue if topic not available
                pass

    def _extract_numeric_value(self, message: Any) -> Optional[float]:
        """Extract numeric value from ROS message"""
        # Handle string messages (YAML-like output from ros2 topic echo)
        if isinstance(message, str):
            # Try to parse "data: 123.45" format
            if 'data:' in message:
                try:
                    value_str = message.split('data:')[1].strip().split()[0]
                    return float(value_str)
                except (ValueError, IndexError):
                    pass

        # Handle dict messages
        elif isinstance(message, dict):
            if 'data' in message:
                try:
                    return float(message['data'])
                except (ValueError, TypeError):
                    pass

        return None

    def _clear_all_plots(self):
        """Clear all plot data"""
        for plots in self.active_plots.values():
            for plot in plots.values():
                plot.clear()

    def _export_data(self):
        """Export all plot data to CSV files"""
        if not self.active_plots:
            QMessageBox.information(
                self,
                "No Data",
                "No plots to export. Add some topics first."
            )
            return

        # Ask for directory
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Export Directory",
            "",
            QFileDialog.ShowDirsOnly
        )

        if not directory:
            return

        try:
            exported_count = 0
            for topic_name, plots in self.active_plots.items():
                for field_name, plot in plots.items():
                    # Create safe filename
                    safe_name = topic_name.replace('/', '_').lstrip('_')
                    file_path = f"{directory}/{safe_name}_{field_name}.csv"

                    plot.export_csv(file_path)
                    exported_count += 1

            QMessageBox.information(
                self,
                "Export Complete",
                f"Exported {exported_count} plot(s) to:\n{directory}"
            )

        except Exception as e:
            QMessageBox.critical(
                self,
                "Export Error",
                f"Failed to export data:\n\n{e}"
            )

    def _on_buffer_size_changed(self, value: int):
        """Update buffer size for all plots"""
        for plots in self.active_plots.values():
            for plot in plots.values():
                plot.max_points = value
                plot.timestamps = deque(plot.timestamps, maxlen=value)
                plot.values = deque(plot.values, maxlen=value)

    def _on_update_rate_changed(self, value: int):
        """Update plotting rate"""
        self.update_interval_ms = 1000 // value
        if self.update_timer.isActive():
            self.update_timer.setInterval(self.update_interval_ms)

    def cleanup(self):
        """Cleanup resources"""
        self.update_timer.stop()
        for topic_name in list(self.active_plots.keys()):
            self.unsubscribe_topic_requested.emit(topic_name)
        self.active_plots.clear()
