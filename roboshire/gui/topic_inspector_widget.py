"""
Topic Inspector Widget - GUI for inspecting ROS2 topics

Features:
- List all topics
- View topic info (type, publishers, subscribers)
- Echo messages in real-time
- Display frequency and bandwidth
- Format messages for readability
"""

import logging
from typing import Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QListWidget, QTextEdit, QPushButton, QLabel,
    QGroupBox, QGridLayout, QListWidgetItem, QLineEdit, QComboBox
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont

from roboshire.integrations.topic_inspector import TopicInspector, TopicInfo


class TopicInspectorWidget(QWidget):
    """
    Widget for inspecting ROS2 topics

    Features:
    - Topic list
    - Topic info display
    - Message echo viewer
    - Frequency/bandwidth display
    """

    # Signals
    topic_selected = Signal(str)  # topic_name

    def __init__(self, topic_inspector: Optional[TopicInspector] = None, parent=None):
        """
        Initialize topic inspector widget

        Args:
            topic_inspector: TopicInspector instance
            parent: Parent widget
        """
        super().__init__(parent)

        self.topic_inspector = topic_inspector
        self.current_topic: Optional[str] = None
        self.echo_active = False

        self._init_ui()

        # Refresh timer
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self._refresh_topics)
        self.refresh_timer.start(5000)  # Refresh every 5 seconds

        logging.info("TopicInspectorWidget initialized")

    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()

        # Title
        title_label = QLabel("<h3>Topic Inspector</h3>")
        layout.addWidget(title_label)

        # Main splitter
        splitter = QSplitter(Qt.Horizontal)

        # Left panel - Topic list
        left_panel = QWidget()
        left_layout = QVBoxLayout()

        left_layout.addWidget(QLabel("<b>Available Topics:</b>"))

        # Search/Filter bar
        filter_layout = QHBoxLayout()
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Search topics... (regex supported)")
        self.search_box.textChanged.connect(self._filter_topics)
        filter_layout.addWidget(self.search_box)

        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["All", "Active", "By Type"])
        self.filter_combo.currentTextChanged.connect(self._filter_topics)
        filter_layout.addWidget(self.filter_combo)

        left_layout.addLayout(filter_layout)

        self.topic_list = QListWidget()
        self.topic_list.itemClicked.connect(self._on_topic_selected)
        left_layout.addWidget(self.topic_list)

        # Store all topics for filtering
        self.all_topics = []

        refresh_button = QPushButton("Refresh Topics")
        refresh_button.clicked.connect(self._refresh_topics)
        left_layout.addWidget(refresh_button)

        left_panel.setLayout(left_layout)
        splitter.addWidget(left_panel)

        # Right panel - Topic details
        right_panel = QWidget()
        right_layout = QVBoxLayout()

        # Topic info group
        info_group = QGroupBox("Topic Information")
        info_layout = QGridLayout()

        info_layout.addWidget(QLabel("<b>Name:</b>"), 0, 0)
        self.topic_name_label = QLabel("-")
        info_layout.addWidget(self.topic_name_label, 0, 1)

        info_layout.addWidget(QLabel("<b>Type:</b>"), 1, 0)
        self.topic_type_label = QLabel("-")
        info_layout.addWidget(self.topic_type_label, 1, 1)

        info_layout.addWidget(QLabel("<b>Publishers:</b>"), 2, 0)
        self.publishers_label = QLabel("-")
        info_layout.addWidget(self.publishers_label, 2, 1)

        info_layout.addWidget(QLabel("<b>Subscribers:</b>"), 3, 0)
        self.subscribers_label = QLabel("-")
        info_layout.addWidget(self.subscribers_label, 3, 1)

        info_layout.addWidget(QLabel("<b>Frequency:</b>"), 4, 0)
        self.frequency_label = QLabel("-")
        info_layout.addWidget(self.frequency_label, 4, 1)

        info_layout.addWidget(QLabel("<b>Bandwidth:</b>"), 5, 0)
        self.bandwidth_label = QLabel("-")
        info_layout.addWidget(self.bandwidth_label, 5, 1)

        info_group.setLayout(info_layout)
        right_layout.addWidget(info_group)

        # Message viewer
        viewer_group = QGroupBox("Message Viewer")
        viewer_layout = QVBoxLayout()

        # Control buttons
        button_layout = QHBoxLayout()

        self.echo_button = QPushButton("Start Echo")
        self.echo_button.clicked.connect(self._toggle_echo)
        button_layout.addWidget(self.echo_button)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self._clear_messages)
        button_layout.addWidget(self.clear_button)

        measure_button = QPushButton("Measure Hz/BW")
        measure_button.clicked.connect(self._measure_stats)
        button_layout.addWidget(measure_button)

        button_layout.addStretch()

        viewer_layout.addLayout(button_layout)

        # Message display
        self.message_viewer = QTextEdit()
        self.message_viewer.setReadOnly(True)
        self.message_viewer.setFont(QFont("Courier", 9))
        viewer_layout.addWidget(self.message_viewer)

        viewer_group.setLayout(viewer_layout)
        right_layout.addWidget(viewer_group)

        right_panel.setLayout(right_layout)
        splitter.addWidget(right_panel)

        # Set splitter sizes
        splitter.setSizes([200, 600])

        layout.addWidget(splitter)
        self.setLayout(layout)

    def set_topic_inspector(self, topic_inspector: TopicInspector):
        """
        Set the topic inspector instance

        Args:
            topic_inspector: TopicInspector instance
        """
        self.topic_inspector = topic_inspector
        self._refresh_topics()
        logging.info("TopicInspector attached to TopicInspectorWidget")

    def refresh_topics(self):
        """Public method to refresh the list of topics"""
        self._refresh_topics()

    def _refresh_topics(self):
        """Refresh the list of topics"""
        if not self.topic_inspector:
            return

        try:
            topics = self.topic_inspector.list_topics()
            self.all_topics = sorted(topics)

            # Apply current filter
            self._filter_topics()

            logging.debug(f"Refreshed topic list: {len(topics)} topics")

        except Exception as e:
            logging.error(f"Error refreshing topics: {e}")

    def _filter_topics(self):
        """Filter topics based on search box and filter combo"""
        import re

        # Get search text
        search_text = self.search_box.text().strip()
        filter_mode = self.filter_combo.currentText()

        # Store current selection
        current_selection = self.current_topic

        # Clear list
        self.topic_list.clear()

        # Filter topics
        filtered_topics = []
        for topic in self.all_topics:
            # Apply text filter (regex)
            if search_text:
                try:
                    if not re.search(search_text, topic, re.IGNORECASE):
                        continue
                except re.error:
                    # Invalid regex, fall back to substring search
                    if search_text.lower() not in topic.lower():
                        continue

            # Apply combo filter
            if filter_mode == "Active":
                # Only show topics with publishers (requires topic_inspector enhancement)
                pass  # TODO: Implement when TopicInspector supports checking active topics
            elif filter_mode == "By Type":
                # Future: Filter by message type
                pass

            filtered_topics.append(topic)

        # Repopulate list
        for topic in filtered_topics:
            item = QListWidgetItem(topic)
            self.topic_list.addItem(item)

            # Restore selection
            if topic == current_selection:
                self.topic_list.setCurrentItem(item)

        logging.debug(f"Filtered topics: {len(filtered_topics)}/{len(self.all_topics)}")

    def _on_topic_selected(self, item: QListWidgetItem):
        """
        Handle topic selection

        Args:
            item: Selected list item
        """
        topic_name = item.text()
        self.current_topic = topic_name

        # Stop any active echo
        if self.echo_active:
            self._toggle_echo()

        # Clear message viewer
        self.message_viewer.clear()

        # Update info
        self._update_topic_info(topic_name)

        self.topic_selected.emit(topic_name)
        logging.info(f"Selected topic: {topic_name}")

    def _update_topic_info(self, topic_name: str):
        """
        Update topic information display

        Args:
            topic_name: Topic name
        """
        if not self.topic_inspector:
            return

        try:
            info = self.topic_inspector.get_topic_info(topic_name)

            if info:
                self.topic_name_label.setText(info.name)
                self.topic_type_label.setText(info.msg_type)
                self.publishers_label.setText(f"{info.publisher_count} ({', '.join(info.publishers[:3])})")
                self.subscribers_label.setText(f"{info.subscription_count} ({', '.join(info.subscribers[:3])})")
            else:
                self.topic_name_label.setText(topic_name)
                self.topic_type_label.setText("Unknown")
                self.publishers_label.setText("-")
                self.subscribers_label.setText("-")

            # Reset stats
            self.frequency_label.setText("-")
            self.bandwidth_label.setText("-")

        except Exception as e:
            logging.error(f"Error updating topic info: {e}")

    def _toggle_echo(self):
        """Toggle message echo on/off"""
        if not self.topic_inspector or not self.current_topic:
            return

        if self.echo_active:
            # Stop echo
            self.topic_inspector.stop_echo_stream(self.current_topic)
            self.echo_active = False
            self.echo_button.setText("Start Echo")
            logging.info(f"Stopped echo for {self.current_topic}")
        else:
            # Start echo
            self.message_viewer.clear()
            self.message_viewer.append(f"<b>Echoing messages from {self.current_topic}...</b>\n")

            def on_message(msg):
                try:
                    formatted = self.topic_inspector.format_message(msg)
                    self.message_viewer.append("---")
                    self.message_viewer.append(formatted)
                    self.message_viewer.append("")

                    # Auto-scroll to bottom
                    scrollbar = self.message_viewer.verticalScrollBar()
                    scrollbar.setValue(scrollbar.maximum())

                except Exception as e:
                    logging.error(f"Error displaying message: {e}")

            self.topic_inspector.start_echo_stream(
                self.current_topic,
                callback=on_message,
                max_messages=100  # Limit to prevent overflow
            )

            self.echo_active = True
            self.echo_button.setText("Stop Echo")
            logging.info(f"Started echo for {self.current_topic}")

    def _clear_messages(self):
        """Clear message viewer"""
        self.message_viewer.clear()

    def _measure_stats(self):
        """Measure topic frequency and bandwidth"""
        if not self.topic_inspector or not self.current_topic:
            return

        try:
            self.message_viewer.append("\n<b>Measuring statistics (5 seconds)...</b>")

            # Measure in background to avoid blocking UI
            import threading

            def measure():
                try:
                    stats = self.topic_inspector.get_topic_stats(self.current_topic, duration=5.0)

                    if stats:
                        # Update UI from main thread
                        self.frequency_label.setText(f"{stats.frequency_hz:.2f} Hz")

                        # Format bandwidth
                        if stats.bandwidth_bps < 1024:
                            bw_str = f"{stats.bandwidth_bps:.2f} B/s"
                        elif stats.bandwidth_bps < 1024 * 1024:
                            bw_str = f"{stats.bandwidth_bps / 1024:.2f} KB/s"
                        else:
                            bw_str = f"{stats.bandwidth_bps / (1024 * 1024):.2f} MB/s"

                        self.bandwidth_label.setText(bw_str)

                        self.message_viewer.append(f"<b>Frequency:</b> {stats.frequency_hz:.2f} Hz")
                        self.message_viewer.append(f"<b>Bandwidth:</b> {bw_str}")
                    else:
                        self.message_viewer.append("<span style='color:red;'><b>Failed to measure statistics</b></span>")

                except Exception as e:
                    logging.error(f"Error measuring stats: {e}")
                    self.message_viewer.append(f"<span style='color:red;'><b>Error:</b> {e}</span>")

            thread = threading.Thread(target=measure, daemon=True)
            thread.start()

        except Exception as e:
            logging.error(f"Error starting stats measurement: {e}")

    def closeEvent(self, event):
        """Handle widget close event"""
        # Stop any active echo streams
        if self.topic_inspector:
            self.topic_inspector.stop_all_streams()

        self.refresh_timer.stop()
        super().closeEvent(event)
