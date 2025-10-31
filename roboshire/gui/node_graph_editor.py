"""
Node Graph Editor Widget

Visual node-based editor using QGraphicsScene/QGraphicsView.
Allows users to create ROS2 node graphs by dragging and connecting nodes.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGraphicsView, QGraphicsScene,
    QGraphicsItem, QGraphicsEllipseItem, QGraphicsPathItem, QGraphicsTextItem,
    QToolBar, QMenu, QMessageBox, QPushButton, QLabel
)
from PySide6.QtCore import Qt, Signal, QPointF, QRectF, QLineF
from PySide6.QtGui import (
    QPen, QBrush, QColor, QPainter, QPainterPath, QFont, QAction, QPalette, QKeySequence,
    QUndoStack, QUndoCommand
)
from typing import Optional, List, Dict, Any
import logging
import json

from roboshire.backend.node_graph import NodeGraph, Node, Port, Connection, NodeType, PortType, generate_id
from roboshire.backend.node_library import NODE_LIBRARY, get_node_categories


# Color scheme - Clean white theme with RoboShire branding
COLORS = {
    'background': QColor(255, 255, 255),  # White background
    'grid_line': QColor(230, 230, 230),  # Light gray grid
    'node_bg': QColor(250, 250, 245),  # Off-white (beige) for nodes
    'node_selected': QColor(85, 107, 47),  # Olive green (RoboShire brand)
    'node_border': QColor(200, 200, 200),  # Light gray border
    'port_input': QColor(107, 142, 35),  # Olive drab (input)
    'port_output': QColor(212, 175, 55),  # Gold (output)
    'connection': QColor(40, 40, 40),  # Black road connections (v2.5.0)
    'connection_active': QColor(20, 20, 20),  # Black asphalt when active (v2.5.0)
    'connection_data_flow': QColor(255, 255, 0),  # Yellow for animated data (headlights) (v2.5.0)
    'text': QColor(51, 51, 51),  # Dark gray text
    'text_selected': QColor(255, 255, 255),  # White text on selected
}

# Node type colors - for visual grouping by functionality
NODE_TYPE_COLORS = {
    'publisher': QColor(255, 200, 120),      # Orange - output data
    'subscriber': QColor(120, 200, 255),     # Blue - input data
    'service_server': QColor(200, 150, 255), # Purple - provides service
    'service_client': QColor(255, 150, 200), # Pink - requests service
    'action_server': QColor(180, 120, 200),  # Dark purple - provides action
    'action_client': QColor(255, 120, 180),  # Rose - requests action
    'timer': QColor(255, 220, 100),          # Yellow - time-based
    'logic': QColor(150, 220, 150),          # Green - processing
    'parameter': QColor(200, 200, 255),      # Light blue - config
    'lifecycle': QColor(255, 180, 120),      # Peach - managed node
}


class GraphicsPort(QGraphicsEllipseItem):
    """Visual representation of a port"""

    def __init__(self, port: Port, parent_node: 'GraphicsNode'):
        # Port is 12x12 circle
        super().__init__(-6, -6, 12, 12)

        self.port = port
        self.parent_node = parent_node
        self.connections: List['GraphicsConnection'] = []

        # Appearance
        color = COLORS['port_output'] if port.port_type == PortType.OUTPUT else COLORS['port_input']
        self.setBrush(QBrush(color))
        self.setPen(QPen(Qt.black, 1))

        # Make it interactive
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)
        self.setZValue(10)  # Ports on top

        # Tooltip
        self.setToolTip(f"{port.name}\n{port.data_type}")

    def hoverEnterEvent(self, event):
        """Highlight on hover"""
        self.setBrush(QBrush(COLORS['connection_active']))
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        """Remove highlight"""
        color = COLORS['port_output'] if self.port.port_type == PortType.OUTPUT else COLORS['port_input']
        self.setBrush(QBrush(color))
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        """Start connection on click"""
        if event.button() == Qt.LeftButton:
            # Let parent handle connection creation
            self.parent_node.scene().views()[0].start_connection(self)
        super().mousePressEvent(event)


class GraphicsNode(QGraphicsItem):
    """Visual representation of a Node"""

    def __init__(self, node: Node):
        super().__init__()

        self.node = node
        self.width = 240  # Wider nodes for better readability (increased from 200)
        self.height = 130  # Taller for better port spacing (increased from 110)
        self.port_graphics: Dict[str, GraphicsPort] = {}

        # Animation state for active nodes
        self.is_active = False
        self.glow_intensity = 0.0  # 0.0 to 1.0
        self.glow_timer = None

        # Make it interactive
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setZValue(5)

        # Set position
        self.setPos(node.position[0], node.position[1])

        # Create title
        self.title_item = QGraphicsTextItem(self)
        self.title_item.setPlainText(node.name)
        self.title_item.setDefaultTextColor(COLORS['text'])
        font = QFont("Segoe UI", 10, QFont.Bold)  # Professional font
        self.title_item.setFont(font)
        self.title_item.setPos(10, 5)

        # Create type label
        self.type_item = QGraphicsTextItem(self)
        self.type_item.setPlainText(node.node_type.value)
        self.type_item.setDefaultTextColor(QColor(120, 120, 120))  # Medium gray
        font = QFont("Arial", 8)
        self.type_item.setFont(font)
        self.type_item.setPos(10, 25)

        # Create ports
        self._create_ports()

        # Adjust height based on number of ports (more spacing)
        max_ports = max(len(node.get_input_ports()), len(node.get_output_ports()))
        if max_ports > 0:
            self.height = max(110, 50 + max_ports * 30)  # Increased from 25 to 30

    def _create_ports(self):
        """Create visual ports with better spacing"""
        input_ports = self.node.get_input_ports()
        output_ports = self.node.get_output_ports()

        # Position input ports on left (increased spacing)
        for i, port in enumerate(input_ports):
            port_item = GraphicsPort(port, self)
            port_item.setParentItem(self)
            y_pos = 55 + i * 35  # Increased from 30 to 35
            port_item.setPos(0, y_pos)
            self.port_graphics[port.id] = port_item

        # Position output ports on right (increased spacing)
        for i, port in enumerate(output_ports):
            port_item = GraphicsPort(port, self)
            port_item.setParentItem(self)
            y_pos = 55 + i * 35  # Increased from 30 to 35
            port_item.setPos(self.width, y_pos)
            self.port_graphics[port.id] = port_item

    def set_active(self, active: bool):
        """Set node active state (running)"""
        self.is_active = active
        if active and not self.glow_timer:
            # Start pulsing glow animation
            from PySide6.QtCore import QTimer
            import math
            self.glow_timer = QTimer()
            self.glow_animation_step = 0
            self.glow_timer.timeout.connect(self._animate_glow)
            self.glow_timer.start(30)  # 30ms = ~33 FPS
        elif not active and self.glow_timer:
            # Stop animation
            self.glow_timer.stop()
            self.glow_timer = None
            self.glow_intensity = 0.0
        self.update()

    def _animate_glow(self):
        """Animate pulsing glow"""
        import math
        self.glow_animation_step += 1
        # Sine wave: 0.3 to 1.0 over 1 second (33 steps at 30ms)
        t = (self.glow_animation_step % 33) / 33.0  # 0.0 to 1.0
        self.glow_intensity = 0.3 + 0.7 * (0.5 + 0.5 * math.sin(t * 2 * math.pi))
        self.update()

    def boundingRect(self):
        """Define bounding rectangle"""
        return QRectF(0, 0, self.width, self.height)

    def paint(self, painter, option, widget):
        """Paint the node with professional styling and active glow"""
        painter.setRenderHint(QPainter.Antialiasing)

        # Active glow effect (drawn first, behind everything)
        if self.is_active and self.glow_intensity > 0:
            glow_size = 10 * self.glow_intensity
            glow_rect = self.boundingRect().adjusted(-glow_size, -glow_size, glow_size, glow_size)
            glow_color = QColor(154, 205, 50, int(80 * self.glow_intensity))  # Yellow-green
            painter.setBrush(QBrush(glow_color))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(glow_rect, 12, 12)

        # Shadow for depth (only if not selected and not actively glowing)
        if not self.isSelected() and not self.is_active:
            shadow_rect = self.boundingRect().adjusted(3, 3, 3, 3)
            painter.setBrush(QBrush(QColor(0, 0, 0, 30)))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(shadow_rect, 8, 8)

        # Get color based on node type
        node_type_str = self.node.node_type.value if hasattr(self.node.node_type, 'value') else str(self.node.node_type)
        base_color = NODE_TYPE_COLORS.get(node_type_str, COLORS['node_bg'])

        # Node background with gradient
        from PySide6.QtGui import QLinearGradient
        gradient = QLinearGradient(0, 0, 0, self.height)

        if self.isSelected():
            # Selected: Olive green gradient
            gradient.setColorAt(0, COLORS['node_selected'])
            gradient.setColorAt(1, COLORS['node_selected'].darker(110))
            painter.setBrush(QBrush(gradient))
            painter.setPen(QPen(COLORS['node_selected'].lighter(120), 3))
            # Update text colors for selected state
            self.title_item.setDefaultTextColor(COLORS['text_selected'])
            self.type_item.setDefaultTextColor(QColor(240, 240, 240))
        else:
            # Normal: Use node type color with gradient
            gradient.setColorAt(0, base_color.lighter(110))
            gradient.setColorAt(1, base_color)
            painter.setBrush(QBrush(gradient))
            painter.setPen(QPen(base_color.darker(130), 2))
            # Restore normal text colors
            self.title_item.setDefaultTextColor(COLORS['text'])
            self.type_item.setDefaultTextColor(QColor(100, 100, 100))

        painter.drawRoundedRect(self.boundingRect(), 8, 8)

        # Header separator line
        if not self.isSelected():
            painter.setPen(QPen(base_color.darker(120), 1))
            painter.drawLine(5, 40, self.width - 5, 40)

    def itemChange(self, change, value):
        """Handle item changes"""
        if change == QGraphicsItem.ItemPositionHasChanged:
            # Update node position in data model
            self.node.position = (self.pos().x(), self.pos().y())
            # Update connections
            for port_item in self.port_graphics.values():
                for conn in port_item.connections:
                    conn.update_path()

        return super().itemChange(change, value)

    def mouseDoubleClickEvent(self, event):
        """Handle double click - emit signal for editing"""
        # Get the view and emit signal
        view = self.scene().views()[0]
        if hasattr(view, 'node_double_clicked'):
            view.node_double_clicked.emit(self.node.id)
        super().mouseDoubleClickEvent(event)


class GraphicsTopicLane(QGraphicsItem):
    """Visual representation of a ROS2 Topic as a horizontal lane/road that nodes connect to"""

    def __init__(self, topic_name: str, message_type: str = ""):
        super().__init__()

        self.topic_name = topic_name
        self.message_type = message_type
        self.width = 480  # Modern wider proportions (8px grid: 60 * 8)
        self.lane_height = 56  # Taller for better visual weight (8px grid: 7 * 8)
        self.total_height = 96  # More generous spacing (8px grid: 12 * 8)
        self.corner_radius = 12  # Modern rounded corners

        # Track which nodes are connected (for visualization)
        self.connected_publishers = []  # List of node IDs
        self.connected_subscribers = []  # List of node IDs

        # Animation state for active topics
        self.is_active = False
        self.flow_offset = 0.0
        self.flow_timer = None

        # Make it interactive
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setZValue(3)  # Topics between connections and nodes

        # Create title (topic name) above the lane - modern typography
        self.title_item = QGraphicsTextItem(self)
        # Clean up topic name (remove leading slash for display)
        display_name = topic_name.lstrip('/')
        self.title_item.setPlainText(display_name)
        self.title_item.setDefaultTextColor(QColor(45, 45, 55))  # Modern charcoal
        font = QFont("Inter", 11, QFont.DemiBold)  # Modern font
        if not font.exactMatch():
            font = QFont("Segoe UI", 11, QFont.DemiBold)
        self.title_item.setFont(font)

        # Center title above lane with generous spacing
        title_width = self.title_item.boundingRect().width()
        self.title_item.setPos((self.width - title_width) / 2, -32)

        # Create message type label below title - modern subtle style
        if message_type:
            self.type_item = QGraphicsTextItem(self)
            # Simplify message type display
            short_type = message_type.split('/')[-1] if '/' in message_type else message_type
            self.type_item.setPlainText(short_type)
            self.type_item.setDefaultTextColor(QColor(140, 140, 150))  # Subtle cool gray
            font = QFont("Inter", 9)  # Modern font
            if not font.exactMatch():
                font = QFont("Segoe UI", 9)
            self.type_item.setFont(font)
            type_width = self.type_item.boundingRect().width()
            self.type_item.setPos((self.width - type_width) / 2, self.lane_height + 10)

    def set_active(self, active: bool):
        """Set topic active state (data flowing)"""
        self.is_active = active
        if active and not self.flow_timer:
            # Start flowing gradient animation
            from PySide6.QtCore import QTimer
            self.flow_timer = QTimer()
            self.flow_timer.timeout.connect(self._animate_flow)
            self.flow_timer.start(30)  # 30ms = ~33 FPS
        elif not active and self.flow_timer:
            # Stop animation
            self.flow_timer.stop()
            self.flow_timer = None
            self.flow_offset = 0.0
        self.update()

    def _animate_flow(self):
        """Animate flowing gradient"""
        self.flow_offset += 0.05
        if self.flow_offset > 1.0:
            self.flow_offset = 0.0
        self.update()

    def boundingRect(self):
        """Define bounding rectangle (include space for labels and shadows)"""
        return QRectF(-8, -40, self.width + 16, self.total_height + 8)  # Extra space for shadows

    def paint(self, painter, option, widget):
        """Paint the topic as a modern horizontal lane with connection points"""
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        # Topic lane position
        lane_y = 0
        lane_rect = QRectF(0, lane_y, self.width, self.lane_height)

        if self.is_active:
            # Active State: Black asphalt road with flowing dashed white lines (v2.5.0)

            # Draw shadow for depth (road has shadow)
            shadow_rect = lane_rect.adjusted(0, 3, 0, 3)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 0, 0, 40)))
            painter.drawRoundedRect(shadow_rect, self.corner_radius, self.corner_radius)

            # Draw black asphalt road
            painter.setBrush(QBrush(QColor(40, 40, 40)))  # Dark asphalt
            painter.setPen(QPen(QColor(20, 20, 20), 2))   # Darker edge
            painter.drawRoundedRect(lane_rect, self.corner_radius, self.corner_radius)

            # Draw dashed white center lane markings (animated)
            painter.setPen(QPen(QColor(255, 255, 255), 3, Qt.DashLine))
            painter.setDashPattern([15, 10])  # Dash pattern

            # Animate the dashes
            painter.setDashOffset(self.flow_offset * 50)
            painter.drawLine(0, lane_y + self.lane_height/2,
                           self.width, lane_y + self.lane_height/2)

            # Connection points with glow effect
            painter.setPen(Qt.NoPen)
            # Left point glow
            painter.setBrush(QBrush(QColor(100, 180, 140, 80)))
            painter.drawEllipse(QPointF(0, lane_y + self.lane_height/2), 10, 10)
            # Left point
            painter.setBrush(QBrush(QColor(100, 180, 140)))
            painter.drawEllipse(QPointF(0, lane_y + self.lane_height/2), 6, 6)

            # Right point glow
            painter.setBrush(QBrush(QColor(100, 180, 140, 80)))
            painter.drawEllipse(QPointF(self.width, lane_y + self.lane_height/2), 10, 10)
            # Right point
            painter.setBrush(QBrush(QColor(100, 180, 140)))
            painter.drawEllipse(QPointF(self.width, lane_y + self.lane_height/2), 6, 6)

            return

        elif self.isSelected():
            # Selected State: Modern highlight
            from PySide6.QtGui import QLinearGradient
            gradient = QLinearGradient(0, lane_y, 0, lane_y + self.lane_height)
            gradient.setColorAt(0.0, QColor(245, 250, 235))  # Light olive
            gradient.setColorAt(1.0, QColor(235, 245, 225))  # Slightly darker

            # Draw shadow
            shadow_rect = lane_rect.adjusted(0, 2, 0, 2)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 0, 0, 25)))
            painter.drawRoundedRect(shadow_rect, self.corner_radius, self.corner_radius)

            # Draw main lane
            painter.setBrush(QBrush(gradient))
            painter.setPen(QPen(COLORS['node_selected'], 3))
            painter.drawRoundedRect(lane_rect, self.corner_radius, self.corner_radius)

        else:
            # Normal State: Gray asphalt road (inactive) (v2.5.0)

            # Draw subtle shadow for depth
            shadow_rect = lane_rect.adjusted(0, 2, 0, 2)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(0, 0, 0, 20)))
            painter.drawRoundedRect(shadow_rect, self.corner_radius, self.corner_radius)

            # Draw gray inactive road
            painter.setBrush(QBrush(QColor(120, 120, 120)))  # Gray asphalt (inactive)
            painter.setPen(QPen(QColor(100, 100, 100), 1.5))
            painter.drawRoundedRect(lane_rect, self.corner_radius, self.corner_radius)

            # Draw static dashed white center line (no animation)
            painter.setPen(QPen(QColor(200, 200, 200), 2, Qt.DashLine))
            painter.setDashPattern([10, 8])
            painter.drawLine(0, lane_y + self.lane_height/2,
                            self.width, lane_y + self.lane_height/2)

        # Connection points (modern minimal style)
        painter.setPen(QPen(QColor(170, 170, 185), 2))
        painter.setBrush(QBrush(QColor(230, 230, 240)))
        # Left
        painter.drawEllipse(QPointF(0, lane_y + self.lane_height/2), 5, 5)
        # Right
        painter.drawEllipse(QPointF(self.width, lane_y + self.lane_height/2), 5, 5)

        # Modern directional arrow (subtle chevron style)
        arrow_y = lane_y + self.lane_height/2
        arrow_center = self.width/2
        painter.setPen(QPen(QColor(180, 180, 195), 2, Qt.SolidLine, Qt.RoundCap))
        # Chevron shape (>)
        painter.drawLine(QPointF(arrow_center - 8, arrow_y - 5),
                        QPointF(arrow_center, arrow_y))
        painter.drawLine(QPointF(arrow_center, arrow_y),
                        QPointF(arrow_center - 8, arrow_y + 5))

    def itemChange(self, change, value):
        """Handle item changes"""
        if change == QGraphicsItem.ItemPositionHasChanged:
            # Topic lane moved - update all connections that route through this lane
            if self.scene():
                for item in self.scene().items():
                    if isinstance(item, GraphicsConnection):
                        # If this connection uses this topic, update its path
                        if item.connection.topic_name == self.topic_name:
                            item.update_path()

        return super().itemChange(change, value)

    def mouseDoubleClickEvent(self, event):
        """Handle double click on topic lane"""
        # For now, just select the lane
        # Future: Could show topic inspector or topic details dialog
        self.setSelected(True)
        super().mouseDoubleClickEvent(event)


class GraphicsConnection(QGraphicsPathItem):
    """Visual representation of a connection with animated data flow and topic label"""

    def __init__(self, connection: Connection, from_port: GraphicsPort, to_port: GraphicsPort):
        super().__init__()

        self.connection = connection
        self.from_port = from_port
        self.to_port = to_port

        # Register with ports
        from_port.connections.append(self)
        to_port.connections.append(self)

        # Animation state
        self.is_active = False  # Data is flowing
        self.flow_offset = 0.0  # Animation offset
        self.flow_timer = None

        # Topic label
        self.topic_label = QGraphicsTextItem(self)
        self.topic_label.setDefaultTextColor(COLORS['text'])
        topic_name = connection.topic_name or "topic"
        self.topic_label.setPlainText(topic_name)
        font = QFont("Arial", 9)
        font.setBold(True)
        self.topic_label.setFont(font)
        self.topic_label.setZValue(5)  # Above connection line

        # Label background for readability
        self.label_bg = QGraphicsPathItem(self)
        self.label_bg.setBrush(QBrush(QColor(255, 255, 255, 200)))
        self.label_bg.setPen(QPen(COLORS['node_border'], 1))
        self.label_bg.setZValue(4)  # Behind label text

        # Appearance
        self.setPen(QPen(COLORS['connection'], 3))  # Thicker for topic visibility
        self.setZValue(1)  # Behind nodes

        # Make it selectable and deletable
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

        # Update path
        self.update_path()

    def set_active(self, active: bool):
        """Set connection active state (data flowing)"""
        self.is_active = active
        if active and not self.flow_timer:
            # Start animation timer
            from PySide6.QtCore import QTimer
            self.flow_timer = QTimer()
            self.flow_timer.timeout.connect(self._animate_flow)
            self.flow_timer.start(50)  # 50ms = 20 FPS
        elif not active and self.flow_timer:
            # Stop animation
            self.flow_timer.stop()
            self.flow_timer = None
            self.flow_offset = 0.0
        self.update()

    def _animate_flow(self):
        """Animate data flow"""
        self.flow_offset += 0.1
        if self.flow_offset > 1.0:
            self.flow_offset = 0.0
        self.update()

    def update_path(self):
        """Update the connection path and position topic label"""
        # Get port positions in scene coordinates
        from_pos = self.from_port.scenePos()
        to_pos = self.to_port.scenePos()

        # Check if this connection should route through a topic lane
        topic_lane_center = None
        lane_half_width = 240  # Default for 480px wide lanes
        if self.connection.topic_name and self.scene():
            # Find the topic lane in the scene
            for item in self.scene().items():
                if isinstance(item, GraphicsTopicLane) and item.topic_name == self.connection.topic_name:
                    # Get the center point of the topic lane in scene coordinates
                    lane_rect = item.boundingRect()
                    lane_center_local = QPointF(lane_rect.center().x(), lane_rect.top() + item.lane_height / 2)
                    topic_lane_center = item.mapToScene(lane_center_local)
                    lane_half_width = item.width / 2  # Dynamically use actual lane width
                    break

        # Create path
        path = QPainterPath()
        path.moveTo(from_pos)

        if topic_lane_center:
            # Route through topic lane: Publisher → Lane → Subscriber
            # This creates a "hub and spoke" pattern with smooth curves

            # First segment: from_port to left edge of lane
            lane_left = QPointF(topic_lane_center.x() - lane_half_width, topic_lane_center.y())
            ctrl_offset_1 = abs(lane_left.x() - from_pos.x()) * 0.6  # More generous curves
            ctrl1_1 = QPointF(from_pos.x() + ctrl_offset_1, from_pos.y())
            ctrl1_2 = QPointF(lane_left.x() - ctrl_offset_1 * 0.3, lane_left.y())
            path.cubicTo(ctrl1_1, ctrl1_2, lane_left)

            # Second segment: across the lane (straight line through center)
            lane_right = QPointF(topic_lane_center.x() + lane_half_width, topic_lane_center.y())
            path.lineTo(lane_right)

            # Third segment: from right edge of lane to to_port
            ctrl_offset_2 = abs(to_pos.x() - lane_right.x()) * 0.6  # More generous curves
            ctrl2_1 = QPointF(lane_right.x() + ctrl_offset_2 * 0.3, lane_right.y())
            ctrl2_2 = QPointF(to_pos.x() - ctrl_offset_2, to_pos.y())
            path.cubicTo(ctrl2_1, ctrl2_2, to_pos)

        else:
            # No topic lane: direct connection with standard Bezier curve
            ctrl_offset = abs(to_pos.x() - from_pos.x()) * 0.5
            ctrl1 = QPointF(from_pos.x() + ctrl_offset, from_pos.y())
            ctrl2 = QPointF(to_pos.x() - ctrl_offset, to_pos.y())
            path.cubicTo(ctrl1, ctrl2, to_pos)

        self.setPath(path)

        # Position topic label at midpoint of connection
        mid_point = path.pointAtPercent(0.5)
        label_rect = self.topic_label.boundingRect()
        self.topic_label.setPos(mid_point.x() - label_rect.width() / 2,
                                mid_point.y() - label_rect.height() / 2)

        # Position label background
        padding = 4
        bg_rect = label_rect.adjusted(-padding, -padding, padding, padding)
        bg_path = QPainterPath()
        bg_path.addRoundedRect(bg_rect, 4, 4)
        self.label_bg.setPath(bg_path)
        self.label_bg.setPos(self.topic_label.pos())

    def paint(self, painter, option, widget):
        """Paint the connection with optional animated data flow"""
        painter.setRenderHint(QPainter.Antialiasing)

        # Base connection line - thicker and more prominent
        if self.isSelected():
            pen = QPen(COLORS['connection_active'], 5)
        elif self.is_active:
            pen = QPen(COLORS['connection_active'], 4)
        else:
            pen = QPen(COLORS['connection'], 3)

        painter.setPen(pen)
        painter.drawPath(self.path())

        # Draw arrowhead at destination to show direction
        from_pos = self.from_port.scenePos()
        to_pos = self.to_port.scenePos()

        # Calculate arrow direction
        import math
        dx = to_pos.x() - from_pos.x()
        dy = to_pos.y() - from_pos.y()
        angle = math.atan2(dy, dx)

        # Arrowhead size
        arrow_size = 12

        # Calculate arrowhead points
        arrow_p1 = to_pos
        arrow_p2 = QPointF(
            to_pos.x() - arrow_size * math.cos(angle - math.pi / 6),
            to_pos.y() - arrow_size * math.sin(angle - math.pi / 6)
        )
        arrow_p3 = QPointF(
            to_pos.x() - arrow_size * math.cos(angle + math.pi / 6),
            to_pos.y() - arrow_size * math.sin(angle + math.pi / 6)
        )

        # Draw filled arrowhead
        arrow_color = COLORS['connection_active'] if self.isSelected() or self.is_active else COLORS['connection']
        painter.setBrush(QBrush(arrow_color))
        painter.setPen(QPen(arrow_color, 1))
        painter.drawPolygon([arrow_p1, arrow_p2, arrow_p3])

        # Animated data flow particles if active
        if self.is_active:
            # Draw flowing particles
            path_length = self.path().length()
            num_particles = 3

            for i in range(num_particles):
                # Calculate particle position along path
                offset = (self.flow_offset + i / num_particles) % 1.0
                t = offset
                point = self.path().pointAtPercent(t)

                # Draw glowing particle
                painter.setBrush(QBrush(COLORS['connection_data_flow']))
                painter.setPen(QPen(COLORS['connection_data_flow'].lighter(150), 1))

                # Draw particle as small circle
                particle_size = 8
                painter.drawEllipse(point, particle_size, particle_size)

                # Draw glow around particle
                glow_color = QColor(COLORS['connection_data_flow'])
                glow_color.setAlpha(80)
                painter.setBrush(QBrush(glow_color))
                painter.setPen(Qt.NoPen)
                painter.drawEllipse(point, particle_size * 1.5, particle_size * 1.5)


class NodeGraphEditorView(QGraphicsView):
    """Custom QGraphicsView for node graph editing"""

    node_selected = Signal(str)  # node_id
    node_double_clicked = Signal(str)  # node_id
    graph_modified = Signal()

    def __init__(self, node_graph: NodeGraph):
        super().__init__()

        self.node_graph = node_graph
        self.graphics_nodes: Dict[str, GraphicsNode] = {}
        self.graphics_connections: Dict[str, GraphicsConnection] = {}
        self.graphics_topic_lanes: Dict[str, GraphicsTopicLane] = {}  # topic_name -> GraphicsTopicLane

        # Connection creation state
        self.temp_connection_start: Optional[GraphicsPort] = None
        self.temp_connection_line: Optional[QGraphicsPathItem] = None

        # Setup scene
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(-5000, -5000, 10000, 10000)
        self.setScene(self.scene)

        # Appearance
        self.setBackgroundBrush(QBrush(COLORS['background']))
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.TextAntialiasing)
        self.setDragMode(QGraphicsView.RubberBandDrag)

        # Enable mouse tracking for pan
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)

        # Pan with middle mouse button
        self._pan_active = False
        self._pan_start = QPointF()

        self.logger = logging.getLogger(__name__)

    def start_connection(self, from_port: GraphicsPort):
        """Start creating a connection from a port"""
        # Only allow output ports to start connections
        if from_port.port.port_type != PortType.OUTPUT:
            return

        self.temp_connection_start = from_port

        # Create temporary line
        self.temp_connection_line = QGraphicsPathItem()
        self.temp_connection_line.setPen(QPen(COLORS['connection_active'], 2, Qt.DashLine))
        self.scene.addItem(self.temp_connection_line)

    def mouseMoveEvent(self, event):
        """Handle mouse move for connection creation and panning"""
        # Update temporary connection line
        if self.temp_connection_line:
            from_pos = self.temp_connection_start.scenePos()
            to_pos = self.mapToScene(event.pos())

            path = QPainterPath()
            path.moveTo(from_pos)
            ctrl_offset = abs(to_pos.x() - from_pos.x()) * 0.5
            ctrl1 = QPointF(from_pos.x() + ctrl_offset, from_pos.y())
            ctrl2 = QPointF(to_pos.x() - ctrl_offset, to_pos.y())
            path.cubicTo(ctrl1, ctrl2, to_pos)

            self.temp_connection_line.setPath(path)

        # Pan with middle mouse button
        if self._pan_active:
            delta = self.mapToScene(event.pos()) - self._pan_start
            self.setSceneRect(self.sceneRect().translated(-delta.x(), -delta.y()))
            self._pan_start = self.mapToScene(event.pos())

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """Handle mouse release for connection completion"""
        if event.button() == Qt.LeftButton and self.temp_connection_line:
            # Find port under mouse
            item = self.itemAt(event.pos())

            if isinstance(item, GraphicsPort):
                # Try to create connection
                self.create_connection(self.temp_connection_start, item)

            # Clean up temporary line
            self.scene.removeItem(self.temp_connection_line)
            self.temp_connection_line = None
            self.temp_connection_start = None

        if event.button() == Qt.MiddleButton:
            self._pan_active = False

        super().mouseReleaseEvent(event)

    def mousePressEvent(self, event):
        """Handle mouse press for panning"""
        if event.button() == Qt.MiddleButton:
            self._pan_active = True
            self._pan_start = self.mapToScene(event.pos())
            event.accept()
            return

        super().mousePressEvent(event)

    def wheelEvent(self, event):
        """Zoom with mouse wheel"""
        zoom_factor = 1.1
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1 / zoom_factor, 1 / zoom_factor)

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts"""
        if event.key() == Qt.Key_Delete:
            self.delete_selected()
        super().keyPressEvent(event)

    def create_connection(self, from_port: GraphicsPort, to_port: GraphicsPort):
        """Create a connection between two ports"""
        # Validate connection
        can_connect, error = from_port.port.can_connect_to(to_port.port)

        if not can_connect:
            QMessageBox.warning(None, "Invalid Connection", error)
            return

        # Determine topic name from node properties
        topic_name = ""
        message_type = from_port.port.data_type

        # Get topic name from publisher node (output port's node)
        from_node = from_port.parent_node.node
        if hasattr(from_node, 'properties') and 'topic_name' in from_node.properties:
            topic_name = from_node.properties['topic_name']
        else:
            # Fallback: use port name as topic name
            topic_name = f"/{from_port.port.name}"

        # Create connection in data model
        connection = Connection(
            id=generate_id("conn"),
            from_port_id=from_port.port.id,
            to_port_id=to_port.port.id,
            topic_name=topic_name
        )

        success, error = self.node_graph.add_connection(connection)

        if not success:
            QMessageBox.warning(None, "Connection Failed", error)
            return

        # Create or get topic lane for this topic
        if topic_name:
            # Calculate midpoint between the two nodes for lane position
            from_pos = from_port.parent_node.scenePos()
            to_pos = to_port.parent_node.scenePos()
            midpoint = QPointF(
                (from_pos.x() + to_pos.x()) / 2,
                (from_pos.y() + to_pos.y()) / 2 - 80  # More generous spacing (8px grid)
            )

            # Create/get topic lane
            topic_lane = self.get_or_create_topic_lane(topic_name, message_type, midpoint)

            # Update connection color to show it goes through a topic
            self.logger.info(f"Connection routed through topic lane: {topic_name}")

        # Create visual connection
        graphics_conn = GraphicsConnection(connection, from_port, to_port)
        self.scene.addItem(graphics_conn)
        self.graphics_connections[connection.id] = graphics_conn

        self.logger.info(f"Created connection: {from_port.port.name} -> {to_port.port.name} (topic: {topic_name})")
        self.graph_modified.emit()

    def get_or_create_topic_lane(self, topic_name: str, message_type: str = "", position: QPointF = None) -> GraphicsTopicLane:
        """
        Get existing topic lane or create new one for the given topic

        Args:
            topic_name: ROS2 topic name (e.g., "/cmd_vel")
            message_type: Message type (e.g., "geometry_msgs/msg/Twist")
            position: Initial position for new lane (defaults to center of scene)

        Returns:
            GraphicsTopicLane for this topic
        """
        # Check if topic lane already exists
        if topic_name in self.graphics_topic_lanes:
            return self.graphics_topic_lanes[topic_name]

        # Create new topic lane
        lane = GraphicsTopicLane(topic_name, message_type)

        # Set position (default to center if not specified)
        if position is None:
            position = QPointF(0, 0)
        lane.setPos(position)

        # Add to scene and tracking dict
        self.scene.addItem(lane)
        self.graphics_topic_lanes[topic_name] = lane

        self.logger.info(f"Created topic lane: {topic_name} ({message_type})")
        return lane

    def delete_selected(self):
        """Delete selected items"""
        for item in self.scene.selectedItems():
            if isinstance(item, GraphicsConnection):
                # Remove from data model
                self.node_graph.remove_connection(item.connection.id)
                # Remove from graphics
                del self.graphics_connections[item.connection.id]
                self.scene.removeItem(item)
                self.graph_modified.emit()

            elif isinstance(item, GraphicsNode):
                # Remove from data model
                self.node_graph.remove_node(item.node.id)
                # Remove from graphics
                del self.graphics_nodes[item.node.id]
                self.scene.removeItem(item)
                self.graph_modified.emit()

            elif isinstance(item, GraphicsTopicLane):
                # Remove topic lane
                if item.topic_name in self.graphics_topic_lanes:
                    del self.graphics_topic_lanes[item.topic_name]
                self.scene.removeItem(item)
                self.graph_modified.emit()

    def add_node_to_scene(self, node: Node):
        """Add a node to the scene"""
        # Skip topic nodes - topics are represented as lanes, not nodes
        # Check for: NodeType.TOPIC, names starting with '/', or names starting with 'topic_'
        if (node.node_type == NodeType.TOPIC or
            node.name.startswith('/') or
            node.name.startswith('topic_')):
            self.logger.info(f"Skipping topic node (topics are lanes): name={node.name}, type={node.node_type}")
            return

        # Add to data model
        self.node_graph.add_node(node)

        # Create graphics node
        self.logger.debug(f"Adding node to scene: name={node.name}, type={node.node_type}")
        graphics_node = GraphicsNode(node)
        self.scene.addItem(graphics_node)
        self.graphics_nodes[node.id] = graphics_node

        self.logger.info(f"Added node: {node.name} ({node.node_type.value})")
        self.graph_modified.emit()

    def clear_scene(self):
        """Clear all nodes and connections"""
        self.scene.clear()
        self.graphics_nodes.clear()
        self.graphics_connections.clear()
        self.graphics_topic_lanes.clear()
        self.node_graph.clear()
        self.graph_modified.emit()

    def refresh_from_graph(self):
        """Rebuild scene from data model"""
        self.scene.clear()
        self.graphics_nodes.clear()
        self.graphics_connections.clear()
        self.graphics_topic_lanes.clear()

        # Identify topic nodes that will be skipped
        topic_node_ids = set()
        topic_node_names = {}  # node_id -> topic name
        for node in self.node_graph.nodes.values():
            if (node.node_type == NodeType.TOPIC or
                node.name.startswith('/') or
                node.name.startswith('topic_')):
                topic_node_ids.add(node.id)
                topic_node_names[node.id] = node.name
                self.logger.info(f"Identified topic node: name={node.name}, id={node.id}")

        # Add all nodes (skip topics - they are lanes, not nodes)
        for node in self.node_graph.nodes.values():
            # Skip topic nodes - topics are represented as lanes
            if node.id in topic_node_ids:
                continue

            graphics_node = GraphicsNode(node)
            self.scene.addItem(graphics_node)
            self.graphics_nodes[node.id] = graphics_node

        # Build map of connections through topic nodes
        # Maps: topic_node_id -> {publishers: [(node, port)], subscribers: [(node, port)]}
        topic_connections = {}
        for connection in self.node_graph.connections.values():
            from_node = self._get_node_for_port(connection.from_port_id)
            to_node = self._get_node_for_port(connection.to_port_id)

            if not from_node or not to_node:
                continue

            # Connection TO a topic node (publisher → topic)
            if to_node.id in topic_node_ids:
                if to_node.id not in topic_connections:
                    topic_connections[to_node.id] = {'publishers': [], 'subscribers': [], 'message_type': None}
                topic_connections[to_node.id]['publishers'].append((from_node, connection.from_port_id))
                # Get message type from publisher's output port
                if connection.from_port_id in from_node.ports:
                    port = from_node.ports[connection.from_port_id]
                    topic_connections[to_node.id]['message_type'] = port.data_type

            # Connection FROM a topic node (topic → subscriber)
            if from_node.id in topic_node_ids:
                if from_node.id not in topic_connections:
                    topic_connections[from_node.id] = {'publishers': [], 'subscribers': [], 'message_type': None}
                topic_connections[from_node.id]['subscribers'].append((to_node, connection.to_port_id))

        # Create direct connections and topic lanes for reconstructed paths
        for topic_node_id, connections in topic_connections.items():
            topic_name = topic_node_names.get(topic_node_id, f"topic_{topic_node_id[:8]}")
            message_type = connections.get('message_type', 'std_msgs/String')

            # For each publisher, connect to each subscriber through topic lane
            for pub_node, pub_port_id in connections['publishers']:
                for sub_node, sub_port_id in connections['subscribers']:
                    # Find graphics ports
                    from_port = self._find_graphics_port(pub_port_id)
                    to_port = self._find_graphics_port(sub_port_id)

                    if from_port and to_port:
                        # Calculate position between nodes
                        from_pos = from_port.parent_node.scenePos()
                        to_pos = to_port.parent_node.scenePos()
                        midpoint = QPointF(
                            (from_pos.x() + to_pos.x()) / 2,
                            (from_pos.y() + to_pos.y()) / 2 - 80
                        )

                        # Create/get topic lane
                        self.get_or_create_topic_lane(topic_name, message_type, midpoint)

                        # Create visual connection with topic name
                        from roboshire.backend.node_graph import Connection
                        temp_connection = Connection(
                            id=f"{pub_port_id}_{sub_port_id}",
                            from_port_id=pub_port_id,
                            to_port_id=sub_port_id,
                            topic_name=topic_name
                        )
                        graphics_conn = GraphicsConnection(temp_connection, from_port, to_port)
                        self.scene.addItem(graphics_conn)
                        self.graphics_connections[temp_connection.id] = graphics_conn

                        self.logger.info(f"Created direct connection through topic lane: {pub_node.name} → {topic_name} → {sub_node.name}")

        # Add remaining connections that don't involve topic nodes
        for connection in self.node_graph.connections.values():
            from_node = self._get_node_for_port(connection.from_port_id)
            to_node = self._get_node_for_port(connection.to_port_id)

            # Skip if either end is a topic node (already handled above)
            if (from_node and from_node.id in topic_node_ids) or (to_node and to_node.id in topic_node_ids):
                continue

            # Find graphics ports
            from_port = self._find_graphics_port(connection.from_port_id)
            to_port = self._find_graphics_port(connection.to_port_id)

            if from_port and to_port:
                # Create topic lane if connection has a topic name
                if connection.topic_name:
                    # Get message type from port
                    message_type = from_port.port.data_type

                    # Calculate position between nodes
                    from_pos = from_port.parent_node.scenePos()
                    to_pos = to_port.parent_node.scenePos()
                    midpoint = QPointF(
                        (from_pos.x() + to_pos.x()) / 2,
                        (from_pos.y() + to_pos.y()) / 2 - 80
                    )

                    # Create/get topic lane
                    self.get_or_create_topic_lane(connection.topic_name, message_type, midpoint)

                # Create visual connection
                graphics_conn = GraphicsConnection(connection, from_port, to_port)
                self.scene.addItem(graphics_conn)
                self.graphics_connections[connection.id] = graphics_conn

        # Apply auto-layout to arrange nodes cleanly
        self._auto_layout()

    def _auto_layout(self):
        """Automatically arrange nodes in a clean layout"""
        if not self.graphics_nodes:
            return

        # Group nodes by type
        publishers = []
        subscribers = []
        others = []

        for node_id, graphics_node in self.graphics_nodes.items():
            node = graphics_node.node
            if node.node_type == NodeType.PUBLISHER:
                publishers.append(graphics_node)
            elif node.node_type == NodeType.SUBSCRIBER:
                subscribers.append(graphics_node)
            else:
                others.append(graphics_node)

        # Layout parameters (8px grid)
        start_x = 100
        start_y = 100
        vertical_spacing = 160  # 20 * 8
        horizontal_spacing = 480  # 60 * 8

        # Left column: Publishers
        y = start_y
        for graphics_node in publishers:
            graphics_node.setPos(start_x, y)
            y += vertical_spacing

        # Middle column: Others (services, timers, etc.)
        y = start_y
        for graphics_node in others:
            graphics_node.setPos(start_x + horizontal_spacing, y)
            y += vertical_spacing

        # Right column: Subscribers
        y = start_y
        for graphics_node in subscribers:
            graphics_node.setPos(start_x + horizontal_spacing * 2, y)
            y += vertical_spacing

        # Reposition topic lanes to be between their connected nodes
        for topic_name, lane in self.graphics_topic_lanes.items():
            # Find all connections for this topic
            connected_nodes = set()
            for conn in self.graphics_connections.values():
                if conn.connection.topic_name == topic_name:
                    connected_nodes.add(conn.from_port.parent_node)
                    connected_nodes.add(conn.to_port.parent_node)

            if connected_nodes:
                # Calculate average position
                avg_x = sum(node.scenePos().x() for node in connected_nodes) / len(connected_nodes)
                avg_y = sum(node.scenePos().y() for node in connected_nodes) / len(connected_nodes)
                lane.setPos(avg_x - lane.width / 2, avg_y - 80)

    def _get_node_for_port(self, port_id: str) -> Optional[Node]:
        """Find the Node that contains a given port ID"""
        for node in self.node_graph.nodes.values():
            # Check if port_id is in this node's ports dictionary
            if port_id in node.ports:
                return node
        return None

    def _find_graphics_port(self, port_id: str) -> Optional[GraphicsPort]:
        """Find a GraphicsPort by port ID"""
        for graphics_node in self.graphics_nodes.values():
            if port_id in graphics_node.port_graphics:
                return graphics_node.port_graphics[port_id]
        return None


# Undo/Redo Command Classes
class AddNodeCommand(QUndoCommand):
    """Command for adding a node"""
    def __init__(self, editor_widget, node_type: str, position: tuple):
        super().__init__(f"Add {node_type} Node")
        self.editor_widget = editor_widget
        self.node_type = node_type
        self.position = position
        self.node_id = None
        self.node_data = None

    def redo(self):
        """Execute: Add node"""
        if self.node_data is None:
            # First time - create new node
            self.node_id = self.editor_widget.graph_view.add_node_at_position(
                self.node_type, self.position
            )
            # Store node data for potential redo
            node = self.editor_widget.node_graph.get_node(self.node_id)
            if node:
                self.node_data = {
                    'id': node.node_id,
                    'type': node.node_type,
                    'name': node.name,
                    'position': node.position
                }
        else:
            # Redo - restore node
            self.editor_widget.graph_view.restore_node(self.node_data)
            self.node_id = self.node_data['id']

    def undo(self):
        """Undo: Remove node"""
        if self.node_id:
            self.editor_widget.graph_view.remove_node(self.node_id)


class DeleteNodeCommand(QUndoCommand):
    """Command for deleting a node"""
    def __init__(self, editor_widget, node_id: str):
        super().__init__(f"Delete Node")
        self.editor_widget = editor_widget
        self.node_id = node_id
        self.node_data = None
        self.connections_data = []

    def redo(self):
        """Execute: Delete node"""
        if self.node_data is None:
            # Save node data before deletion
            node = self.editor_widget.node_graph.get_node(self.node_id)
            if node:
                self.node_data = {
                    'id': node.node_id,
                    'type': node.node_type,
                    'name': node.name,
                    'position': node.position,
                    'ports': {p.port_id: {'name': p.name, 'type': p.port_type, 'data_type': p.data_type}
                             for p in node.get_all_ports()}
                }
                # Save connections
                for conn in self.editor_widget.node_graph.connections.values():
                    if conn.source_node_id == self.node_id or conn.target_node_id == self.node_id:
                        self.connections_data.append({
                            'id': conn.connection_id,
                            'source_node': conn.source_node_id,
                            'source_port': conn.source_port_id,
                            'target_node': conn.target_node_id,
                            'target_port': conn.target_port_id
                        })
        # Delete node
        self.editor_widget.graph_view.remove_node(self.node_id)

    def undo(self):
        """Undo: Restore node and connections"""
        if self.node_data:
            self.editor_widget.graph_view.restore_node(self.node_data)
            # Restore connections
            for conn_data in self.connections_data:
                self.editor_widget.graph_view.restore_connection(conn_data)


class AddConnectionCommand(QUndoCommand):
    """Command for adding a connection"""
    def __init__(self, editor_widget, source_port_id: str, target_port_id: str):
        super().__init__("Add Connection")
        self.editor_widget = editor_widget
        self.source_port_id = source_port_id
        self.target_port_id = target_port_id
        self.connection_id = None

    def redo(self):
        """Execute: Add connection"""
        self.connection_id = self.editor_widget.graph_view.create_connection(
            self.source_port_id, self.target_port_id
        )

    def undo(self):
        """Undo: Remove connection"""
        if self.connection_id:
            self.editor_widget.graph_view.remove_connection(self.connection_id)


class DeleteConnectionCommand(QUndoCommand):
    """Command for deleting a connection"""
    def __init__(self, editor_widget, connection_id: str):
        super().__init__("Delete Connection")
        self.editor_widget = editor_widget
        self.connection_id = connection_id
        self.connection_data = None

    def redo(self):
        """Execute: Delete connection"""
        if self.connection_data is None:
            # Save connection data
            conn = self.editor_widget.node_graph.connections.get(self.connection_id)
            if conn:
                self.connection_data = {
                    'source_node': conn.source_node_id,
                    'source_port': conn.source_port_id,
                    'target_node': conn.target_node_id,
                    'target_port': conn.target_port_id
                }
        self.editor_widget.graph_view.remove_connection(self.connection_id)

    def undo(self):
        """Undo: Restore connection"""
        if self.connection_data:
            self.editor_widget.graph_view.create_connection(
                self.connection_data['source_port'],
                self.connection_data['target_port']
            )


class MoveNodeCommand(QUndoCommand):
    """Command for moving a node"""
    def __init__(self, editor_widget, node_id: str, old_pos: tuple, new_pos: tuple):
        super().__init__("Move Node")
        self.editor_widget = editor_widget
        self.node_id = node_id
        self.old_pos = old_pos
        self.new_pos = new_pos

    def redo(self):
        """Execute: Move to new position"""
        self.editor_widget.graph_view.set_node_position(self.node_id, self.new_pos)

    def undo(self):
        """Undo: Move to old position"""
        self.editor_widget.graph_view.set_node_position(self.node_id, self.old_pos)


class NodeGraphEditorWidget(QWidget):
    """Main widget for node graph editing with toolbar and undo/redo support"""

    graph_modified = Signal()
    node_selected = Signal(str)  # node_id
    node_double_clicked = Signal(str)  # node_id

    def __init__(self):
        super().__init__()

        self.node_graph = NodeGraph()
        self.logger = logging.getLogger(__name__)

        # Undo/Redo stack
        self.undo_stack = QUndoStack(self)

        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Create toolbar
        self.toolbar = QToolBar()
        self.toolbar.setMovable(False)
        layout.addWidget(self.toolbar)

        # Add node buttons
        self._create_toolbar()

        # Create graph view
        self.graph_view = NodeGraphEditorView(self.node_graph)
        self.graph_view.graph_modified.connect(self.graph_modified)
        self.graph_view.node_selected.connect(self.node_selected)
        self.graph_view.node_double_clicked.connect(self.node_double_clicked)
        layout.addWidget(self.graph_view)

        # Status label
        self.status_label = QLabel("Ready - Right-click to add nodes, drag to connect")
        layout.addWidget(self.status_label)

    def _create_toolbar(self):
        """Create toolbar with node creation buttons and undo/redo"""
        # Undo action
        self.undo_action = self.undo_stack.createUndoAction(self, "Undo")
        self.undo_action.setShortcut(QKeySequence.Undo)  # Ctrl+Z
        self.undo_action.setToolTip("Undo last action (Ctrl+Z)")
        self.toolbar.addAction(self.undo_action)

        # Redo action
        self.redo_action = self.undo_stack.createRedoAction(self, "Redo")
        self.redo_action.setShortcut(QKeySequence.Redo)  # Ctrl+Shift+Z or Ctrl+Y
        self.redo_action.setToolTip("Redo last action (Ctrl+Y)")
        self.toolbar.addAction(self.redo_action)

        self.toolbar.addSeparator()

        # Add node menu
        add_menu = QMenu("Add Node", self)

        # Group by category
        categories = get_node_categories()
        for category in categories:
            category_menu = add_menu.addMenu(category)

            for node_name, node_info in NODE_LIBRARY.items():
                if node_info['category'] == category:
                    icon = node_info.get('icon', '')
                    action = category_menu.addAction(f"{icon} {node_name}")
                    action.triggered.connect(lambda checked, name=node_name: self.add_node(name))

        # Add button to toolbar
        add_button = QPushButton("Add Node")
        add_button.setMenu(add_menu)
        self.toolbar.addWidget(add_button)

        self.toolbar.addSeparator()

        # Clear button
        clear_action = QAction("Clear All", self)
        clear_action.triggered.connect(self.clear_graph)
        self.toolbar.addAction(clear_action)

        # Validate button
        validate_action = QAction("Validate", self)
        validate_action.triggered.connect(self.validate_graph)
        self.toolbar.addAction(validate_action)

    def add_node(self, node_type_name: str):
        """Add a node of the specified type"""
        from roboshire.backend.node_library import create_node_by_type

        try:
            # Create node at center of view
            view_center = self.graph_view.mapToScene(self.graph_view.viewport().rect().center())

            node = create_node_by_type(
                node_type_name,
                name=f"{node_type_name.lower().replace(' ', '_')}_{len(self.node_graph.nodes)}",
                position=(view_center.x(), view_center.y())
            )

            self.graph_view.add_node_to_scene(node)
            self.status_label.setText(f"Added {node_type_name} node")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to add node: {e}")

    def clear_graph(self):
        """Clear all nodes"""
        reply = QMessageBox.question(
            self,
            "Clear Graph",
            "Are you sure you want to clear all nodes?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.graph_view.clear_scene()
            self.status_label.setText("Graph cleared")

    def validate_graph(self):
        """Validate the current graph"""
        is_valid, errors = self.node_graph.validate()

        if is_valid:
            stats = self.node_graph.get_stats()
            QMessageBox.information(
                self,
                "Validation Success",
                f"Graph is valid!\n\nNodes: {stats['num_nodes']}\nConnections: {stats['num_connections']}"
            )
        else:
            error_msg = "Graph has errors:\n\n" + "\n".join(errors[:10])
            if len(errors) > 10:
                error_msg += f"\n\n... and {len(errors) - 10} more"
            QMessageBox.warning(self, "Validation Errors", error_msg)

    def get_node_graph(self) -> NodeGraph:
        """Get the current node graph"""
        return self.node_graph

    def load_graph(self, graph: NodeGraph):
        """Load a graph into the editor"""
        self.node_graph = graph
        self.graph_view.node_graph = graph
        self.graph_view.refresh_from_graph()
        self.status_label.setText(f"Loaded graph with {len(graph.nodes)} nodes")
