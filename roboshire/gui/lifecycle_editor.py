"""
Lifecycle Editor Widget

GUI for controlling ROS2 lifecycle node state transitions.

Features:
- Visual display of lifecycle nodes and their current states
- One-click state transition buttons
- Auto-refresh of node states
- Color-coded state indicators
- State diagram visualization
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QLabel, QGroupBox, QSplitter, QHeaderView
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QFont
import logging

from roboshire.integrations.lifecycle_manager import (
    LifecycleManager, LifecycleState, LifecycleTransition
)


class LifecycleEditorWidget(QWidget):
    """
    Widget for managing lifecycle nodes

    Displays table of lifecycle nodes with state indicators
    and buttons for triggering transitions.
    """

    # Signals
    transition_triggered = Signal(str, str)  # node_name, transition
    state_changed = Signal(str, str)  # node_name, new_state

    def __init__(self, lifecycle_manager: LifecycleManager, parent=None):
        """
        Initialize Lifecycle Editor

        Args:
            lifecycle_manager: LifecycleManager instance
            parent: Parent widget
        """
        super().__init__(parent)

        self.lifecycle_manager = lifecycle_manager
        self.logger = logging.getLogger(__name__)

        # Auto-refresh timer
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self._auto_refresh)
        self.refresh_enabled = False

        # Setup UI
        self._setup_ui()

        # Connect to lifecycle manager callbacks
        self.lifecycle_manager.add_state_change_callback(self._on_state_changed)

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Lifecycle Node Manager")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Control buttons
        control_layout = QHBoxLayout()

        self.refresh_btn = QPushButton("Refresh Nodes")
        self.refresh_btn.clicked.connect(self._refresh_nodes)
        control_layout.addWidget(self.refresh_btn)

        self.auto_refresh_btn = QPushButton("Auto-Refresh: OFF")
        self.auto_refresh_btn.setCheckable(True)
        self.auto_refresh_btn.clicked.connect(self._toggle_auto_refresh)
        control_layout.addWidget(self.auto_refresh_btn)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        # Create splitter for table and state diagram
        splitter = QSplitter(Qt.Vertical)

        # Lifecycle nodes table
        self._setup_nodes_table()
        splitter.addWidget(self.nodes_table)

        # State diagram
        diagram_group = QGroupBox("Lifecycle State Diagram")
        diagram_layout = QVBoxLayout(diagram_group)
        self.state_diagram = self._create_state_diagram()
        diagram_layout.addWidget(self.state_diagram)
        splitter.addWidget(diagram_group)

        splitter.setStretchFactor(0, 3)  # Table gets more space
        splitter.setStretchFactor(1, 1)  # Diagram gets less space

        layout.addWidget(splitter)

        # Status bar
        self.status_label = QLabel("Ready")
        layout.addWidget(self.status_label)

    def _setup_nodes_table(self):
        """Setup the lifecycle nodes table"""
        self.nodes_table = QTableWidget()
        self.nodes_table.setColumnCount(8)
        self.nodes_table.setHorizontalHeaderLabels([
            "Node Name",
            "Current State",
            "Configure",
            "Activate",
            "Deactivate",
            "Cleanup",
            "Shutdown",
            "Auto-Start"
        ])

        # Configure table
        header = self.nodes_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)  # Node name stretches
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        for i in range(2, 8):
            header.setSectionResizeMode(i, QHeaderView.ResizeToContents)

        self.nodes_table.setAlternatingRowColors(True)
        self.nodes_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.nodes_table.setEditTriggers(QTableWidget.NoEditTriggers)

    def _create_state_diagram(self) -> QLabel:
        """Create a visual state diagram label"""
        diagram_text = """
        Lifecycle State Diagram:

        ┌─────────────┐  configure   ┌──────────┐  activate   ┌────────┐
        │ UNCONFIGURED│─────────────>│ INACTIVE │────────────>│ ACTIVE │
        └─────────────┘              └──────────┘             └────────┘
               ^                          ^  │                     │
               │                          │  │                     │
               │       cleanup            │  │    deactivate       │
               └──────────────────────────┘  └─────────────────────┘

        shutdown (from any state) ──> FINALIZED
        """

        diagram = QLabel(diagram_text)
        diagram.setFont(QFont("Courier", 9))
        diagram.setAlignment(Qt.AlignCenter)
        diagram.setStyleSheet("background-color: #f0f0f0; padding: 10px;")

        return diagram

    def _refresh_nodes(self):
        """Refresh the list of lifecycle nodes"""
        try:
            self.status_label.setText("Refreshing lifecycle nodes...")
            self.refresh_btn.setEnabled(False)

            # Refresh nodes from lifecycle manager
            self.lifecycle_manager.refresh_nodes()

            # Update table
            self._update_table()

            self.status_label.setText(f"Found {self.nodes_table.rowCount()} lifecycle nodes")

        except Exception as e:
            self.logger.error(f"Failed to refresh nodes: {e}")
            self.status_label.setText(f"Error: {e}")

        finally:
            self.refresh_btn.setEnabled(True)

    def _update_table(self):
        """Update the nodes table with current data"""
        try:
            nodes = self.lifecycle_manager.get_all_nodes()

            self.nodes_table.setRowCount(len(nodes))

            for row, (node_name, node_info) in enumerate(nodes.items()):
                # Node name
                name_item = QTableWidgetItem(node_name)
                self.nodes_table.setItem(row, 0, name_item)

                # Current state (color-coded)
                state_item = QTableWidgetItem(node_info.state.value.upper())
                state_item.setBackground(self._get_state_color(node_info.state))
                state_item.setForeground(QColor("white"))
                state_font = QFont()
                state_font.setBold(True)
                state_item.setFont(state_font)
                self.nodes_table.setItem(row, 1, state_item)

                # Transition buttons
                self._add_transition_button(row, 2, node_name, LifecycleTransition.CONFIGURE)
                self._add_transition_button(row, 3, node_name, LifecycleTransition.ACTIVATE)
                self._add_transition_button(row, 4, node_name, LifecycleTransition.DEACTIVATE)
                self._add_transition_button(row, 5, node_name, LifecycleTransition.CLEANUP)
                self._add_transition_button(row, 6, node_name, LifecycleTransition.SHUTDOWN)

                # Auto-start button
                auto_start_btn = QPushButton("Auto-Start")
                auto_start_btn.clicked.connect(
                    lambda checked, n=node_name: self._auto_start_node(n)
                )
                self.nodes_table.setCellWidget(row, 7, auto_start_btn)

        except Exception as e:
            self.logger.error(f"Failed to update table: {e}")

    def _add_transition_button(
        self,
        row: int,
        col: int,
        node_name: str,
        transition: LifecycleTransition
    ):
        """Add a transition button to the table"""
        button = QPushButton(transition.value.capitalize())
        button.setStyleSheet("""
            QPushButton {
                padding: 5px 10px;
                font-size: 10pt;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        button.clicked.connect(
            lambda: self._trigger_transition(node_name, transition)
        )
        self.nodes_table.setCellWidget(row, col, button)

    def _get_state_color(self, state: LifecycleState) -> QColor:
        """Get color for a lifecycle state"""
        color_map = {
            LifecycleState.UNKNOWN: QColor("#999999"),      # Gray
            LifecycleState.UNCONFIGURED: QColor("#2196F3"), # Blue
            LifecycleState.INACTIVE: QColor("#FF9800"),     # Orange
            LifecycleState.ACTIVE: QColor("#4CAF50"),       # Green
            LifecycleState.FINALIZED: QColor("#F44336")     # Red
        }
        return color_map.get(state, QColor("#999999"))

    def _trigger_transition(self, node_name: str, transition: LifecycleTransition):
        """Trigger a state transition"""
        try:
            self.status_label.setText(f"Triggering {transition.value} on {node_name}...")

            success = self.lifecycle_manager.transition_node(node_name, transition)

            if success:
                self.status_label.setText(f"Transition {transition.value} successful for {node_name}")
                self.transition_triggered.emit(node_name, transition.value)

                # Refresh table to show new state
                self._update_table()
            else:
                self.status_label.setText(f"Transition {transition.value} failed for {node_name}")

        except Exception as e:
            self.logger.error(f"Failed to trigger transition: {e}")
            self.status_label.setText(f"Error: {e}")

    def _auto_start_node(self, node_name: str):
        """Auto-start a node (configure + activate)"""
        try:
            self.status_label.setText(f"Auto-starting {node_name}...")

            success = self.lifecycle_manager.auto_start_node(node_name)

            if success:
                self.status_label.setText(f"Node {node_name} auto-started successfully")
                # Refresh table
                self._update_table()
            else:
                self.status_label.setText(f"Failed to auto-start {node_name}")

        except Exception as e:
            self.logger.error(f"Failed to auto-start node: {e}")
            self.status_label.setText(f"Error: {e}")

    def _toggle_auto_refresh(self, checked: bool):
        """Toggle auto-refresh on/off"""
        if checked:
            self.refresh_enabled = True
            self.refresh_timer.start(5000)  # Refresh every 5 seconds
            self.auto_refresh_btn.setText("Auto-Refresh: ON")
            self.status_label.setText("Auto-refresh enabled (5s interval)")
        else:
            self.refresh_enabled = False
            self.refresh_timer.stop()
            self.auto_refresh_btn.setText("Auto-Refresh: OFF")
            self.status_label.setText("Auto-refresh disabled")

    def _auto_refresh(self):
        """Auto-refresh callback"""
        if self.refresh_enabled:
            self._refresh_nodes()

    def _on_state_changed(self, node_name: str, new_state: LifecycleState):
        """Callback when a node's state changes"""
        self.logger.info(f"Node {node_name} state changed to {new_state.value}")
        self.state_changed.emit(node_name, new_state.value)

        # Update table if visible
        if self.isVisible():
            self._update_table()
