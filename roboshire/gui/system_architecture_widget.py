"""
System Architecture Diagram Widget

Visual diagram showing data flow between:
- Windows (RoboShire)
- VMware Ubuntu VM (ROS2 environment)
- Microcontroller (Arduino/ESP32)
- Packages and Nodes
- Build folders

Replaces the Properties/Validation panel with a beginner-friendly architecture view.

Author: RoboShire Team
Phase: 10 (UX Polish)
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QScrollArea
from PySide6.QtCore import Qt, QRect, QPoint
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QBrush, QPainterPath
from typing import List, Dict, Tuple
from roboshire.gui.brand_theme import BrandColors, BrandFonts


class SystemArchitectureWidget(QWidget):
    """
    Visual diagram of system architecture showing data flow
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setMinimumWidth(350)
        self.setMinimumHeight(600)

        # Data for visualization
        self.packages = []
        self.nodes = []
        self.build_status = "Not built"
        self.running_nodes_count = 0

        self._setup_ui()

    def _setup_ui(self):
        """Setup UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Title with RoboShire branding
        title = QLabel("<h3>System Architecture</h3>")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "color: #556B2F; "  # Olive Green
            "font-weight: 600; "
            "padding: 8px; "
            "background-color: #F5F5DC; "  # Beige background
            "border-radius: 5px; "
            "margin-bottom: 5px;"
        )
        layout.addWidget(title)

        # Subtitle
        subtitle = QLabel("Data Flow: RoboShire → ROS2 (Ubuntu) → Robot Hardware")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(
            "color: #556B2F; "  # Olive Green
            "font-size: 10px; "
            "padding: 4px;"
        )
        layout.addWidget(subtitle)

        # Scrollable canvas
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        # Canvas widget
        self.canvas = ArchitectureCanvas(self)
        scroll.setWidget(self.canvas)

        layout.addWidget(scroll)

        # Legend with RoboShire branding
        legend_label = QLabel(
            "<b style='color: #556B2F;'>Legend:</b><br>"
            "<span style='color: #556B2F;'>●</span> Windows / RoboShire<br>"
            "<span style='color: #6B8E23;'>●</span> VMware Ubuntu VM / ROS2<br>"
            "<span style='color: #D4AF37;'>●</span> Microcontroller<br>"
            "<span style='color: #8B7355;'>●</span> ROS2 Packages<br>"
            "<span style='color: #9ACD32;'>●</span> ROS2 Nodes<br>"
            "<span style='color: #A9A9A9;'>●</span> Build/Install Folders"
        )
        legend_label.setWordWrap(True)
        legend_label.setStyleSheet(
            "background-color: #F5F5DC; "  # Beige background
            "padding: 10px; "
            "border-radius: 5px; "
            "border: 1px solid #556B2F; "  # Olive Green border
            "font-size: 9px; "
            "color: #333;"
        )
        layout.addWidget(legend_label)

    def update_packages(self, packages: List[str]):
        """Update list of packages"""
        self.packages = packages
        self.canvas.packages = packages
        self.canvas.update()

    def update_nodes(self, nodes: List[str]):
        """Update list of running nodes"""
        self.nodes = nodes
        self.running_nodes_count = len(nodes)
        self.canvas.nodes = nodes
        self.canvas.update()

    def update_build_status(self, status: str):
        """Update build status"""
        self.build_status = status
        self.canvas.build_status = status
        self.canvas.update()


class ArchitectureCanvas(QWidget):
    """
    Canvas for drawing architecture diagram
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setMinimumHeight(600)  # Reduced from 800 for better proportions

        # Data
        self.packages = []
        self.nodes = []
        self.build_status = "Not built"

    def paintEvent(self, event):
        """Draw professional grid-based architecture diagram"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        width = self.width()
        height = 650  # Reduced for better proportions

        # RoboShire Brand Colors (professional earth tones)
        brand_olive = QColor(85, 107, 47)  # #556B2F - Primary
        brand_light_olive = QColor(107, 142, 35)  # #6B8E23
        brand_gold = QColor(212, 175, 55)  # #D4AF37
        brand_brown = QColor(139, 115, 85)  # #8B7355
        brand_yellow_green = QColor(154, 205, 50)  # #9ACD32
        brand_gray = QColor(169, 169, 169)  # #A9A9A9
        vm_color = QColor(230, 240, 255)  # Light blue for VM background
        folder_color = QColor(255, 215, 0)  # Gold color for folders
        node_color = QColor(144, 238, 144)  # Light green for ROS2 nodes
        mcu_color = QColor(255, 182, 193)  # Light pink for microcontroller
        text_dark = QColor(51, 51, 51)  # #333
        text_light = QColor(255, 255, 255)  # #FFF
        bg_light = QColor(250, 250, 245)  # #FAFAF5

        # Professional Typography using brand fonts
        header_font = BrandFonts.heading(size=11, bold=True)
        title_font = BrandFonts.heading(size=10, bold=True)
        label_font = BrandFonts.body(size=9)
        small_font = BrandFonts.body(size=8)
        tiny_font = BrandFonts.body(size=7)

        # Grid Layout Constants (compact, uniform sizing)
        margin = 20
        y_offset = 20
        box_width = width - (margin * 2)
        row_height = 55  # Uniform row height for main boxes
        small_row_height = 45  # For smaller boxes
        spacing = 12  # Consistent vertical spacing
        h_spacing = 10  # Horizontal spacing for side-by-side boxes

        # =====================================
        # 1. ROBOSHIRE IDE (Top)
        # =====================================
        self._draw_professional_box(
            painter,
            QRect(margin, y_offset, box_width, row_height),
            brand_olive,
            text_light,
            "RoboShire IDE",
            "Visual ROS2 Development Environment",
            header_font,
            small_font
        )
        y_offset += row_height + spacing

        # Arrow down
        self._draw_professional_arrow(
            painter,
            width // 2, y_offset,
            width // 2, y_offset + 25,
            "Generate",
            brand_olive,
            tiny_font
        )
        y_offset += 30

        # === 2. ROS2 WORKSPACE (Ubuntu) - Grid Layout ===
        # Title bar for ROS2 section
        ros2_title_height = 30
        painter.setBrush(QBrush(brand_light_olive))
        painter.setPen(QPen(brand_light_olive.darker(120), 2))
        ros2_title_rect = QRect(margin, y_offset, box_width, ros2_title_height)
        painter.drawRoundedRect(ros2_title_rect, 5, 5)

        painter.setPen(text_light)
        painter.setFont(title_font)
        painter.drawText(ros2_title_rect, Qt.AlignCenter, "ROS2 Environment (Ubuntu)")
        y_offset += ros2_title_height + spacing

        # Row 1: Source Packages (full width)
        src_height = 70
        src_box = QRect(margin, y_offset, box_width, src_height)
        painter.setBrush(QBrush(brand_brown.lighter(180)))
        painter.setPen(QPen(brand_brown, 2))
        painter.drawRoundedRect(src_box, 5, 5)

        painter.setPen(text_dark)
        painter.setFont(label_font)
        painter.drawText(src_box.adjusted(10, 5, -10, -60), Qt.AlignTop | Qt.AlignLeft, "Source Packages")

        # Show packages in grid
        painter.setFont(tiny_font)
        if self.packages:
            pkg_text = ", ".join(self.packages[:5])
            if len(self.packages) > 5:
                pkg_text += f" (+{len(self.packages)-5})"
        else:
            pkg_text = "No packages yet"
        painter.drawText(src_box.adjusted(10, 25, -10, -10), Qt.AlignLeft | Qt.AlignTop, pkg_text)
        y_offset += src_height + spacing

        # Row 2: Build & Install (2 columns)
        col_width = (box_width - h_spacing) // 2

        # Build folder (left)
        build_box = QRect(margin, y_offset, col_width, small_row_height)
        painter.setBrush(QBrush(folder_color.lighter(150)))
        painter.setPen(QPen(folder_color.darker(110), 2))
        painter.drawRoundedRect(build_box, 5, 5)
        painter.setPen(text_dark)
        painter.setFont(label_font)
        painter.drawText(build_box, Qt.AlignCenter, f"Build\n{self.build_status}")

        # Install folder (right)
        install_box = QRect(margin + col_width + h_spacing, y_offset, col_width, small_row_height)
        painter.setBrush(QBrush(folder_color.lighter(150)))
        painter.setPen(QPen(folder_color.darker(110), 2))
        painter.drawRoundedRect(install_box, 5, 5)
        painter.setPen(text_dark)
        painter.setFont(label_font)
        painter.drawText(install_box, Qt.AlignCenter, "Install\nExecutables")
        y_offset += small_row_height + spacing

        # Row 3: Running Nodes (full width)
        nodes_height = 65
        nodes_box = QRect(margin, y_offset, box_width, nodes_height)
        painter.setBrush(QBrush(node_color.lighter(140)))
        painter.setPen(QPen(node_color, 2))
        painter.drawRoundedRect(nodes_box, 5, 5)

        painter.setPen(text_dark)
        painter.setFont(label_font)
        painter.drawText(nodes_box.adjusted(10, 5, -10, -50), Qt.AlignTop | Qt.AlignLeft, f"Running Nodes ({len(self.nodes)})")

        painter.setFont(tiny_font)
        if self.nodes:
            node_text = ", ".join(self.nodes[:4])
            if len(self.nodes) > 4:
                node_text += f" (+{len(self.nodes)-4})"
        else:
            painter.setPen(QColor(150, 150, 150))
            node_text = "No nodes running"
        painter.drawText(nodes_box.adjusted(10, 25, -10, -10), Qt.AlignLeft | Qt.AlignTop, node_text)
        y_offset += nodes_height + spacing

        # Arrow down to hardware
        self._draw_professional_arrow(
            painter,
            width // 2, y_offset,
            width // 2, y_offset + 25,
            "Serial/WiFi",
            brand_olive,
            tiny_font
        )
        y_offset += 30

        # === 3. Hardware Layer (2 columns) ===
        # Microcontroller (left)
        mcu_box = QRect(margin, y_offset, col_width, row_height)
        painter.setBrush(QBrush(mcu_color.lighter(120)))
        painter.setPen(QPen(mcu_color, 2))
        painter.drawRoundedRect(mcu_box, 5, 5)

        painter.setPen(text_dark)
        painter.setFont(label_font)
        painter.drawText(mcu_box, Qt.AlignCenter, "Microcontroller\nArduino/ESP32")

        # Robot Hardware (right)
        hw_box = QRect(margin + col_width + h_spacing, y_offset, col_width, row_height)
        painter.setBrush(QBrush(QColor(255, 178, 102)))
        painter.setPen(QPen(QColor(230, 126, 34), 2))
        painter.drawRoundedRect(hw_box, 5, 5)

        painter.setPen(text_dark)
        painter.setFont(label_font)
        painter.drawText(hw_box, Qt.AlignCenter, "Robot Hardware\nMotors & Sensors")

    def _draw_professional_box(self, painter: QPainter, rect: QRect, bg_color: QColor,
                                text_color: QColor, title: str, subtitle: str,
                                title_font: QFont, subtitle_font: QFont):
        """Draw a professional rounded box with title and subtitle"""
        # Draw shadow
        shadow_offset = 3
        shadow_rect = rect.adjusted(shadow_offset, shadow_offset, shadow_offset, shadow_offset)
        painter.setBrush(QBrush(QColor(0, 0, 0, 30)))
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(shadow_rect, 8, 8)

        # Draw main box with gradient
        from PySide6.QtGui import QLinearGradient
        gradient = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        gradient.setColorAt(0, bg_color)
        gradient.setColorAt(1, bg_color.darker(110))

        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(bg_color.darker(130), 2))
        painter.drawRoundedRect(rect, 8, 8)

        # Draw title
        painter.setPen(text_color)
        painter.setFont(title_font)
        title_rect = rect.adjusted(15, 10, -15, -rect.height() + 30)
        painter.drawText(title_rect, Qt.AlignLeft | Qt.AlignVCenter, title)

        # Draw subtitle
        painter.setFont(subtitle_font)
        painter.setPen(QColor(text_color.red(), text_color.green(), text_color.blue(), 230))
        subtitle_rect = rect.adjusted(15, 32, -15, -10)
        painter.drawText(subtitle_rect, Qt.AlignLeft | Qt.AlignTop, subtitle)

    def _draw_professional_arrow(self, painter: QPainter, x1: int, y1: int, x2: int, y2: int,
                                  label: str, color: QColor, font: QFont):
        """Draw a professional arrow with label"""
        # Draw dashed line
        pen = QPen(color, 2, Qt.DashLine)
        painter.setPen(pen)
        painter.drawLine(x1, y1, x2, y2)

        # Draw arrowhead (solid)
        arrow_size = 10
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(color, 1))
        painter.drawPolygon([
            QPoint(x2, y2),
            QPoint(x2 - arrow_size // 2, y2 - arrow_size),
            QPoint(x2 + arrow_size // 2, y2 - arrow_size)
        ])

        # Draw label with background
        if label:
            painter.setFont(font)
            metrics = painter.fontMetrics()
            label_width = metrics.horizontalAdvance(label)
            label_height = metrics.height()

            label_x = x1 + 10
            label_y = (y1 + y2) // 2 - label_height // 2

            # Label background
            bg_rect = QRect(label_x - 4, label_y - 2, label_width + 8, label_height + 4)
            painter.setBrush(QBrush(QColor(255, 255, 255, 200)))
            painter.setPen(QPen(color.lighter(120), 1))
            painter.drawRoundedRect(bg_rect, 3, 3)

            # Label text
            painter.setPen(color.darker(120))
            painter.drawText(label_x, label_y + metrics.ascent(), label)

    def _draw_arrow(self, painter: QPainter, x1: int, y1: int, x2: int, y2: int, label: str = ""):
        """Draw arrow with label (legacy method for compatibility)"""
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(100, 100, 100)))

        # Draw line
        painter.drawLine(x1, y1, x2, y2)

        # Draw arrowhead
        arrow_size = 8
        painter.drawPolygon([
            QPoint(x2, y2),
            QPoint(x2 - arrow_size // 2, y2 - arrow_size),
            QPoint(x2 + arrow_size // 2, y2 - arrow_size)
        ])

        # Draw label
        if label:
            painter.setPen(QColor(100, 100, 100))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(x1 + 10, (y1 + y2) // 2, label)

    def _draw_bidirectional_arrow(self, painter: QPainter, x1: int, y1: int, x2: int, y2: int, label: str = ""):
        """Draw bidirectional arrow"""
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.setBrush(QBrush(QColor(100, 100, 100)))

        # Draw line
        painter.drawLine(x1, y1, x2, y2)

        # Draw arrowhead pointing down
        arrow_size = 8
        painter.drawPolygon([
            QPoint(x2, y2),
            QPoint(x2 - arrow_size // 2, y2 - arrow_size),
            QPoint(x2 + arrow_size // 2, y2 - arrow_size)
        ])

        # Draw arrowhead pointing up
        painter.drawPolygon([
            QPoint(x1, y1),
            QPoint(x1 - arrow_size // 2, y1 + arrow_size),
            QPoint(x1 + arrow_size // 2, y1 + arrow_size)
        ])

        # Draw label
        if label:
            painter.setPen(QColor(100, 100, 100))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(x1 + 10, (y1 + y2) // 2, label)
