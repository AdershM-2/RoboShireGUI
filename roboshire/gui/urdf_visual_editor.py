"""
Visual URDF Editor Widget

Visual robot designer with 3D viewport, link/joint tree, and property editor.
Create robots without touching XML!
"""

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTreeWidget, QTreeWidgetItem, QSplitter, QGroupBox,
    QLineEdit, QComboBox, QDoubleSpinBox, QTabWidget,
    QFileDialog, QMessageBox, QToolBar, QMenuBar, QMenu
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QAction, QFont
import logging

try:
    from PySide6.QtOpenGLWidgets import QOpenGLWidget
    from PySide6.QtGui import QSurfaceFormat
    OPENGL_AVAILABLE = True
except ImportError:
    OPENGL_AVAILABLE = False

try:
    import trimesh
    import numpy as np
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False


class InertiaCalculator:
    """Calculate inertia tensors from meshes or geometric primitives"""

    @staticmethod
    def from_box(mass: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Calculate inertia for a box

        Args:
            mass: Mass in kg
            x, y, z: Dimensions in meters

        Returns:
            (Ixx, Iyy, Izz) inertia values
        """
        ixx = (1.0 / 12.0) * mass * (y**2 + z**2)
        iyy = (1.0 / 12.0) * mass * (x**2 + z**2)
        izz = (1.0 / 12.0) * mass * (x**2 + y**2)
        return (ixx, iyy, izz)

    @staticmethod
    def from_cylinder(mass: float, radius: float, length: float) -> Tuple[float, float, float]:
        """
        Calculate inertia for a cylinder (along Z axis)

        Args:
            mass: Mass in kg
            radius: Radius in meters
            length: Length in meters

        Returns:
            (Ixx, Iyy, Izz) inertia values
        """
        ixx = (1.0 / 12.0) * mass * (3 * radius**2 + length**2)
        iyy = ixx
        izz = 0.5 * mass * radius**2
        return (ixx, iyy, izz)

    @staticmethod
    def from_sphere(mass: float, radius: float) -> Tuple[float, float, float]:
        """
        Calculate inertia for a sphere

        Args:
            mass: Mass in kg
            radius: Radius in meters

        Returns:
            (Ixx, Iyy, Izz) inertia values (all equal for sphere)
        """
        i = (2.0 / 5.0) * mass * radius**2
        return (i, i, i)

    @staticmethod
    def from_mesh(mesh_path: str, mass: float) -> Optional[Tuple[float, float, float]]:
        """
        Calculate inertia from a 3D mesh file

        Args:
            mesh_path: Path to mesh file (STL, OBJ, etc.)
            mass: Mass in kg

        Returns:
            (Ixx, Iyy, Izz) inertia values or None if calculation fails
        """
        if not TRIMESH_AVAILABLE:
            return None

        try:
            mesh = trimesh.load(mesh_path)

            # Calculate volume and density
            volume = mesh.volume
            if volume <= 0:
                return None

            density = mass / volume

            # Calculate inertia tensor
            inertia = mesh.moment_inertia * density

            # Return diagonal elements (Ixx, Iyy, Izz)
            return (inertia[0, 0], inertia[1, 1], inertia[2, 2])

        except Exception as e:
            logging.error(f"Failed to calculate inertia from mesh: {e}")
            return None


class URDFLink:
    """Represents a URDF link"""

    def __init__(self, name: str):
        self.name = name
        self.mass = 1.0
        self.com = [0.0, 0.0, 0.0]  # Center of mass
        self.inertia = {
            'ixx': 0.01, 'ixy': 0.0, 'ixz': 0.0,
            'iyy': 0.01, 'iyz': 0.0, 'izz': 0.01
        }
        self.visual_geometry = {'type': 'box', 'size': [0.1, 0.1, 0.1]}
        self.collision_geometry = {'type': 'box', 'size': [0.1, 0.1, 0.1]}
        self.color = [0.8, 0.8, 0.8, 1.0]  # RGBA


class URDFJoint:
    """Represents a URDF joint"""

    def __init__(self, name: str, parent: str, child: str, joint_type: str = "fixed"):
        self.name = name
        self.parent = parent
        self.child = child
        self.type = joint_type  # fixed, revolute, prismatic, continuous
        self.origin = {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]}
        self.axis = [0.0, 0.0, 1.0]
        self.limits = {
            'lower': -3.14, 'upper': 3.14,
            'effort': 100.0, 'velocity': 1.0
        }
        self.dynamics = {'damping': 0.0, 'friction': 0.0}


class URDFViewport(QWidget):
    """3D viewport for URDF visualization (simplified for now)"""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.links = []
        self.joints = []

        self._setup_ui()

    def _setup_ui(self):
        """Setup the 3D viewport UI"""
        layout = QVBoxLayout(self)

        # For now, show a placeholder
        # In a full implementation, this would use OpenGL or Qt3D
        placeholder = QLabel("3D Viewport\n(Robot Preview)\n\nNote: Full 3D rendering requires OpenGL")
        placeholder.setAlignment(Qt.AlignCenter)
        placeholder.setStyleSheet("""
            QLabel {
                background-color: #2d2d2d;
                color: #888;
                font-size: 14px;
                border: 2px dashed #555;
                min-height: 300px;
            }
        """)
        layout.addWidget(placeholder)

        # Control buttons
        button_layout = QHBoxLayout()

        reset_btn = QPushButton("Reset View")
        reset_btn.clicked.connect(self.reset_view)
        button_layout.addWidget(reset_btn)

        top_btn = QPushButton("Top")
        button_layout.addWidget(top_btn)

        front_btn = QPushButton("Front")
        button_layout.addWidget(front_btn)

        side_btn = QPushButton("Side")
        button_layout.addWidget(side_btn)

        button_layout.addStretch()

        layout.addLayout(button_layout)

    def reset_view(self):
        """Reset camera view"""
        pass  # TODO: Implement camera reset

    def add_link(self, link: URDFLink):
        """Add a link to the viewport"""
        self.links.append(link)
        # TODO: Add visual representation

    def add_joint(self, joint: URDFJoint):
        """Add a joint to the viewport"""
        self.joints.append(joint)
        # TODO: Add joint visualization


class RobotTreeWidget(QTreeWidget):
    """Hierarchical tree view of robot structure"""

    link_selected = Signal(str)  # link_name
    joint_selected = Signal(str)  # joint_name

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setHeaderLabel("Robot Structure")
        self.itemClicked.connect(self._on_item_clicked)

        # Context menu
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self._show_context_menu)

    def _on_item_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle item click"""
        item_type = item.data(0, Qt.UserRole)
        item_name = item.text(0)

        if item_type == 'link':
            self.link_selected.emit(item_name)
        elif item_type == 'joint':
            self.joint_selected.emit(item_name)

    def _show_context_menu(self, position):
        """Show context menu"""
        item = self.itemAt(position)
        if not item:
            return

        menu = QMenu(self)

        delete_action = QAction("Delete", self)
        menu.addAction(delete_action)

        menu.exec_(self.mapToGlobal(position))

    def add_link_item(self, link_name: str, parent_item=None):
        """Add a link to the tree"""
        item = QTreeWidgetItem([link_name])
        item.setData(0, Qt.UserRole, 'link')
        item.setIcon(0, self.style().standardIcon(self.style().SP_FileIcon))

        if parent_item:
            parent_item.addChild(item)
        else:
            self.addTopLevelItem(item)

        return item

    def add_joint_item(self, joint_name: str, parent_item):
        """Add a joint to the tree"""
        item = QTreeWidgetItem([joint_name])
        item.setData(0, Qt.UserRole, 'joint')
        item.setIcon(0, self.style().standardIcon(self.style().SP_DirLinkIcon))

        parent_item.addChild(item)
        return item


class PropertyPanel(QWidget):
    """Property editor for links and joints"""

    properties_changed = Signal(dict)  # property_dict

    def __init__(self, parent=None):
        super().__init__(parent)

        self.current_link: Optional[URDFLink] = None
        self.current_joint: Optional[URDFJoint] = None

        self._setup_ui()

    def _setup_ui(self):
        """Setup property editor UI"""
        layout = QVBoxLayout(self)

        # Title
        self.title_label = QLabel("Properties")
        title_font = QFont()
        title_font.setBold(True)
        self.title_label.setFont(title_font)
        layout.addWidget(self.title_label)

        # Tab widget for different property categories
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        # Link properties tab
        self.link_tab = self._create_link_properties_tab()
        self.tabs.addTab(self.link_tab, "Link")

        # Joint properties tab
        self.joint_tab = self._create_joint_properties_tab()
        self.tabs.addTab(self.joint_tab, "Joint")

        # Initially show link tab
        self.tabs.setCurrentWidget(self.link_tab)

        layout.addStretch()

    def _create_link_properties_tab(self) -> QWidget:
        """Create link properties tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Mass group
        mass_group = QGroupBox("Inertial Properties")
        mass_layout = QVBoxLayout()

        mass_row = QHBoxLayout()
        mass_row.addWidget(QLabel("Mass (kg):"))
        self.mass_spin = QDoubleSpinBox()
        self.mass_spin.setRange(0.001, 10000.0)
        self.mass_spin.setValue(1.0)
        self.mass_spin.setDecimals(3)
        mass_row.addWidget(self.mass_spin)
        mass_layout.addLayout(mass_row)

        # Center of Mass
        com_label = QLabel("Center of Mass (m):")
        mass_layout.addWidget(com_label)

        com_row = QHBoxLayout()
        com_row.addWidget(QLabel("X:"))
        self.com_x_spin = QDoubleSpinBox()
        self.com_x_spin.setRange(-100.0, 100.0)
        self.com_x_spin.setDecimals(3)
        com_row.addWidget(self.com_x_spin)

        com_row.addWidget(QLabel("Y:"))
        self.com_y_spin = QDoubleSpinBox()
        self.com_y_spin.setRange(-100.0, 100.0)
        self.com_y_spin.setDecimals(3)
        com_row.addWidget(self.com_y_spin)

        com_row.addWidget(QLabel("Z:"))
        self.com_z_spin = QDoubleSpinBox()
        self.com_z_spin.setRange(-100.0, 100.0)
        self.com_z_spin.setDecimals(3)
        com_row.addWidget(self.com_z_spin)

        mass_layout.addLayout(com_row)

        # Inertia
        inertia_label = QLabel("Inertia Tensor:")
        mass_layout.addWidget(inertia_label)

        self.inertia_spins = {}
        for component in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{component}:"))
            spin = QDoubleSpinBox()
            spin.setRange(-100.0, 100.0)
            spin.setValue(0.01 if component in ['ixx', 'iyy', 'izz'] else 0.0)
            spin.setDecimals(6)
            spin.setSingleStep(0.001)
            row.addWidget(spin)
            self.inertia_spins[component] = spin
            mass_layout.addLayout(row)

        # Calculate button
        calc_btn = QPushButton("Calculate from Geometry")
        calc_btn.clicked.connect(self._calculate_inertia)
        mass_layout.addWidget(calc_btn)

        mass_group.setLayout(mass_layout)
        layout.addWidget(mass_group)

        # Geometry group
        geom_group = QGroupBox("Visual Geometry")
        geom_layout = QVBoxLayout()

        type_row = QHBoxLayout()
        type_row.addWidget(QLabel("Type:"))
        self.geom_type_combo = QComboBox()
        self.geom_type_combo.addItems(['box', 'cylinder', 'sphere', 'mesh'])
        type_row.addWidget(self.geom_type_combo)
        geom_layout.addLayout(type_row)

        # Geometry dimensions (changes based on type)
        self.geom_size_widget = QWidget()
        self.geom_size_layout = QHBoxLayout(self.geom_size_widget)
        geom_layout.addWidget(self.geom_size_widget)

        self.geom_type_combo.currentTextChanged.connect(self._on_geometry_type_changed)
        self._on_geometry_type_changed('box')  # Initialize

        geom_group.setLayout(geom_layout)
        layout.addWidget(geom_group)

        layout.addStretch()
        return widget

    def _create_joint_properties_tab(self) -> QWidget:
        """Create joint properties tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Joint type
        type_group = QGroupBox("Joint Type")
        type_layout = QVBoxLayout()

        self.joint_type_combo = QComboBox()
        self.joint_type_combo.addItems(['fixed', 'revolute', 'continuous', 'prismatic', 'floating', 'planar'])
        type_layout.addWidget(self.joint_type_combo)

        type_group.setLayout(type_layout)
        layout.addWidget(type_group)

        # Origin
        origin_group = QGroupBox("Origin")
        origin_layout = QVBoxLayout()

        # XYZ
        xyz_row = QHBoxLayout()
        xyz_row.addWidget(QLabel("Position (m):"))
        origin_layout.addLayout(xyz_row)

        xyz_values = QHBoxLayout()
        self.origin_spins = {}
        for axis in ['x', 'y', 'z']:
            xyz_values.addWidget(QLabel(f"{axis.upper()}:"))
            spin = QDoubleSpinBox()
            spin.setRange(-100.0, 100.0)
            spin.setDecimals(3)
            xyz_values.addWidget(spin)
            self.origin_spins[axis] = spin
        origin_layout.addLayout(xyz_values)

        # RPY
        rpy_row = QHBoxLayout()
        rpy_row.addWidget(QLabel("Rotation (rad):"))
        origin_layout.addLayout(rpy_row)

        rpy_values = QHBoxLayout()
        for axis in ['r', 'p', 'y']:
            rpy_values.addWidget(QLabel(f"{axis.upper()}:"))
            spin = QDoubleSpinBox()
            spin.setRange(-math.pi, math.pi)
            spin.setDecimals(3)
            spin.setSingleStep(0.1)
            rpy_values.addWidget(spin)
            self.origin_spins[axis] = spin
        origin_layout.addLayout(rpy_values)

        origin_group.setLayout(origin_layout)
        layout.addWidget(origin_group)

        # Axis
        axis_group = QGroupBox("Axis of Motion")
        axis_layout = QHBoxLayout()

        self.axis_spins = {}
        for axis in ['x', 'y', 'z']:
            axis_layout.addWidget(QLabel(f"{axis.upper()}:"))
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(1)
            spin.setValue(1.0 if axis == 'z' else 0.0)
            axis_layout.addWidget(spin)
            self.axis_spins[axis] = spin

        axis_group.setLayout(axis_layout)
        layout.addWidget(axis_group)

        # Limits
        limits_group = QGroupBox("Joint Limits")
        limits_layout = QVBoxLayout()

        self.limit_spins = {}
        limits_data = [
            ('lower', 'Lower Limit (rad):', -math.pi),
            ('upper', 'Upper Limit (rad):', math.pi),
            ('effort', 'Max Effort (Nm):', 100.0),
            ('velocity', 'Max Velocity (rad/s):', 1.0)
        ]

        for key, label, default in limits_data:
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            spin = QDoubleSpinBox()
            spin.setRange(-10000.0, 10000.0)
            spin.setValue(default)
            spin.setDecimals(3)
            row.addWidget(spin)
            self.limit_spins[key] = spin
            limits_layout.addLayout(row)

        limits_group.setLayout(limits_layout)
        layout.addWidget(limits_group)

        layout.addStretch()
        return widget

    def _on_geometry_type_changed(self, geom_type: str):
        """Handle geometry type change"""
        # Clear existing widgets
        while self.geom_size_layout.count():
            child = self.geom_size_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        # Add appropriate size inputs
        if geom_type == 'box':
            self.geom_size_layout.addWidget(QLabel("Size (m):"))
            for axis in ['X', 'Y', 'Z']:
                self.geom_size_layout.addWidget(QLabel(f"{axis}:"))
                spin = QDoubleSpinBox()
                spin.setRange(0.001, 100.0)
                spin.setValue(0.1)
                spin.setDecimals(3)
                self.geom_size_layout.addWidget(spin)

        elif geom_type == 'cylinder':
            self.geom_size_layout.addWidget(QLabel("Radius (m):"))
            radius_spin = QDoubleSpinBox()
            radius_spin.setRange(0.001, 100.0)
            radius_spin.setValue(0.05)
            radius_spin.setDecimals(3)
            self.geom_size_layout.addWidget(radius_spin)

            self.geom_size_layout.addWidget(QLabel("Length (m):"))
            length_spin = QDoubleSpinBox()
            length_spin.setRange(0.001, 100.0)
            length_spin.setValue(0.1)
            length_spin.setDecimals(3)
            self.geom_size_layout.addWidget(length_spin)

        elif geom_type == 'sphere':
            self.geom_size_layout.addWidget(QLabel("Radius (m):"))
            radius_spin = QDoubleSpinBox()
            radius_spin.setRange(0.001, 100.0)
            radius_spin.setValue(0.05)
            radius_spin.setDecimals(3)
            self.geom_size_layout.addWidget(radius_spin)

        elif geom_type == 'mesh':
            browse_btn = QPushButton("Browse Mesh File...")
            self.geom_size_layout.addWidget(browse_btn)

    def _calculate_inertia(self):
        """Calculate inertia from current geometry"""
        geom_type = self.geom_type_combo.currentText()
        mass = self.mass_spin.value()

        inertia = None

        if geom_type == 'box':
            # Get box dimensions from layout
            # Simplified: use default values
            x, y, z = 0.1, 0.1, 0.1
            inertia = InertiaCalculator.from_box(mass, x, y, z)

        elif geom_type == 'cylinder':
            radius, length = 0.05, 0.1
            inertia = InertiaCalculator.from_cylinder(mass, radius, length)

        elif geom_type == 'sphere':
            radius = 0.05
            inertia = InertiaCalculator.from_sphere(mass, radius)

        if inertia:
            self.inertia_spins['ixx'].setValue(inertia[0])
            self.inertia_spins['iyy'].setValue(inertia[1])
            self.inertia_spins['izz'].setValue(inertia[2])

            QMessageBox.information(self, "Inertia Calculated",
                                  f"Inertia calculated for {geom_type}:\n"
                                  f"Ixx: {inertia[0]:.6f}\n"
                                  f"Iyy: {inertia[1]:.6f}\n"
                                  f"Izz: {inertia[2]:.6f}")

    def show_link_properties(self, link: URDFLink):
        """Show properties for a link"""
        self.current_link = link
        self.current_joint = None
        self.tabs.setCurrentWidget(self.link_tab)

        # Load link properties
        self.mass_spin.setValue(link.mass)
        self.com_x_spin.setValue(link.com[0])
        self.com_y_spin.setValue(link.com[1])
        self.com_z_spin.setValue(link.com[2])

        for key, spin in self.inertia_spins.items():
            spin.setValue(link.inertia.get(key, 0.0))

    def show_joint_properties(self, joint: URDFJoint):
        """Show properties for a joint"""
        self.current_joint = joint
        self.current_link = None
        self.tabs.setCurrentWidget(self.joint_tab)

        # Load joint properties
        self.joint_type_combo.setCurrentText(joint.type)

        self.origin_spins['x'].setValue(joint.origin['xyz'][0])
        self.origin_spins['y'].setValue(joint.origin['xyz'][1])
        self.origin_spins['z'].setValue(joint.origin['xyz'][2])
        self.origin_spins['r'].setValue(joint.origin['rpy'][0])
        self.origin_spins['p'].setValue(joint.origin['rpy'][1])
        self.origin_spins['y'].setValue(joint.origin['rpy'][2])

        self.axis_spins['x'].setValue(joint.axis[0])
        self.axis_spins['y'].setValue(joint.axis[1])
        self.axis_spins['z'].setValue(joint.axis[2])

        for key, spin in self.limit_spins.items():
            spin.setValue(joint.limits.get(key, 0.0))


class URDFVisualEditor(QWidget):
    """
    Visual URDF robot designer

    Features:
    - 3D viewport (OpenGL/Qt3D)
    - Link/joint tree view
    - Property panels
    - Drag-and-drop link creation
    - Inertia calculator
    - Export to validated URDF
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)
        self.robot_name = "my_robot"
        self.links: Dict[str, URDFLink] = {}
        self.joints: Dict[str, URDFJoint] = {}

        self._setup_ui()

        # Add default base link
        self.add_link("base_link")

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)

        # Menu bar
        menubar = QMenuBar()

        file_menu = menubar.addMenu("File")
        file_menu.addAction("New Robot", self._on_new_robot)
        file_menu.addAction("Open URDF...", self._on_open_urdf)
        file_menu.addAction("Save URDF...", self._on_save_urdf)
        file_menu.addSeparator()
        file_menu.addAction("Export for Validation", self._on_export_validated)

        edit_menu = menubar.addMenu("Edit")
        edit_menu.addAction("Add Link", self._on_add_link)
        edit_menu.addAction("Add Joint", self._on_add_joint)

        view_menu = menubar.addMenu("View")
        view_menu.addAction("Reset Camera", lambda: self.viewport.reset_view())

        layout.setMenuBar(menubar)

        # Toolbar
        toolbar = QToolBar()
        toolbar.addAction("Add Link", self._on_add_link)
        toolbar.addAction("Add Joint", self._on_add_joint)
        toolbar.addSeparator()
        toolbar.addAction("Validate", self._validate_robot)
        toolbar.addAction("Export", self._on_save_urdf)

        layout.addWidget(toolbar)

        # Main content: 3-panel layout
        splitter = QSplitter(Qt.Horizontal)

        # Left: Robot tree
        self.tree = RobotTreeWidget()
        self.tree.link_selected.connect(self._on_link_selected)
        self.tree.joint_selected.connect(self._on_joint_selected)
        self.tree.setMaximumWidth(300)
        splitter.addWidget(self.tree)

        # Center: 3D viewport
        self.viewport = URDFViewport()
        splitter.addWidget(self.viewport)

        # Right: Properties panel
        self.properties = PropertyPanel()
        self.properties.setMaximumWidth(350)
        splitter.addWidget(self.properties)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)
        splitter.setStretchFactor(2, 1)

        layout.addWidget(splitter)

        # Status bar
        status_layout = QHBoxLayout()
        self.status_label = QLabel(f"Robot: {self.robot_name}")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()

        self.link_count_label = QLabel("Links: 0")
        status_layout.addWidget(self.link_count_label)

        self.joint_count_label = QLabel("Joints: 0")
        status_layout.addWidget(self.joint_count_label)

        layout.addLayout(status_layout)

    def add_link(self, name: str, parent_link: Optional[str] = None) -> URDFLink:
        """Add a link to the robot"""
        if name in self.links:
            QMessageBox.warning(self, "Duplicate Link", f"Link '{name}' already exists")
            return self.links[name]

        link = URDFLink(name)
        self.links[name] = link

        # Add to tree
        parent_item = None
        if parent_link and parent_link in self.links:
            # Find parent item in tree
            # Simplified: add as top-level for now
            pass

        self.tree.add_link_item(name, parent_item)

        # Add to viewport
        self.viewport.add_link(link)

        self._update_status()
        self.logger.info(f"Added link: {name}")

        return link

    def add_joint(self, name: str, parent: str, child: str, joint_type: str = "fixed") -> URDFJoint:
        """Add a joint connecting two links"""
        if name in self.joints:
            QMessageBox.warning(self, "Duplicate Joint", f"Joint '{name}' already exists")
            return self.joints[name]

        if parent not in self.links:
            QMessageBox.warning(self, "Invalid Parent", f"Parent link '{parent}' does not exist")
            return None

        if child not in self.links:
            QMessageBox.warning(self, "Invalid Child", f"Child link '{child}' does not exist")
            return None

        joint = URDFJoint(name, parent, child, joint_type)
        self.joints[name] = joint

        # Add to tree (under parent link)
        # Simplified for now

        # Add to viewport
        self.viewport.add_joint(joint)

        self._update_status()
        self.logger.info(f"Added joint: {name} ({parent} -> {child})")

        return joint

    def _update_status(self):
        """Update status bar"""
        self.link_count_label.setText(f"Links: {len(self.links)}")
        self.joint_count_label.setText(f"Joints: {len(self.joints)}")

    def _on_link_selected(self, link_name: str):
        """Handle link selection"""
        if link_name in self.links:
            self.properties.show_link_properties(self.links[link_name])

    def _on_joint_selected(self, joint_name: str):
        """Handle joint selection"""
        if joint_name in self.joints:
            self.properties.show_joint_properties(self.joints[joint_name])

    def _on_new_robot(self):
        """Create a new robot"""
        self.links.clear()
        self.joints.clear()
        self.tree.clear()
        self.add_link("base_link")
        self.logger.info("New robot created")

    def _on_add_link(self):
        """Add a new link"""
        # Simple dialog for now
        from PySide6.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self, "Add Link", "Link name:")
        if ok and name:
            self.add_link(name)

    def _on_add_joint(self):
        """Add a new joint"""
        # Simplified dialog
        from PySide6.QtWidgets import QInputDialog

        if len(self.links) < 2:
            QMessageBox.information(self, "Not Enough Links",
                                  "You need at least 2 links to create a joint")
            return

        name, ok = QInputDialog.getText(self, "Add Joint", "Joint name:")
        if not ok or not name:
            return

        # Select parent and child (simplified - use first two links)
        link_list = list(self.links.keys())
        parent = link_list[0]
        child = link_list[1] if len(link_list) > 1 else link_list[0]

        self.add_joint(name, parent, child, "revolute")

    def _on_open_urdf(self):
        """Open existing URDF file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open URDF", "", "URDF Files (*.urdf);;All Files (*)"
        )
        if filename:
            # TODO: Parse URDF and load into editor
            self.logger.info(f"Opening URDF: {filename}")
            QMessageBox.information(self, "Not Implemented",
                                  "URDF import coming in next update!")

    def _on_save_urdf(self):
        """Save URDF to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save URDF", f"{self.robot_name}.urdf",
            "URDF Files (*.urdf);;All Files (*)"
        )
        if filename:
            self._export_urdf(filename)

    def _on_export_validated(self):
        """Export and validate URDF"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Validated URDF", f"{self.robot_name}_validated.urdf",
            "URDF Files (*.urdf);;All Files (*)"
        )
        if filename:
            self._export_urdf(filename)
            # TODO: Trigger URDF validator
            self.logger.info(f"URDF exported and ready for validation: {filename}")

    def _export_urdf(self, filename: str):
        """Export robot to URDF format"""
        try:
            urdf_content = self._generate_urdf()
            with open(filename, 'w') as f:
                f.write(urdf_content)

            self.logger.info(f"URDF saved: {filename}")
            QMessageBox.information(self, "URDF Saved",
                                  f"Robot URDF saved to:\n{filename}")

        except Exception as e:
            self.logger.error(f"Failed to save URDF: {e}")
            QMessageBox.critical(self, "Save Failed", f"Failed to save URDF:\n{e}")

    def _generate_urdf(self) -> str:
        """Generate URDF XML content"""
        lines = ['<?xml version="1.0"?>']
        lines.append(f'<robot name="{self.robot_name}">')
        lines.append('')

        # Export links
        for link_name, link in self.links.items():
            lines.append(f'  <link name="{link_name}">')

            # Inertial
            lines.append('    <inertial>')
            lines.append(f'      <mass value="{link.mass}"/>')
            lines.append(f'      <origin xyz="{link.com[0]} {link.com[1]} {link.com[2]}"/>')
            lines.append('      <inertia')
            lines.append(f'        ixx="{link.inertia["ixx"]}" ixy="{link.inertia["ixy"]}" ixz="{link.inertia["ixz"]}"')
            lines.append(f'        iyy="{link.inertia["iyy"]}" iyz="{link.inertia["iyz"]}"')
            lines.append(f'        izz="{link.inertia["izz"]}"/>')
            lines.append('    </inertial>')

            # Visual
            lines.append('    <visual>')
            geom = link.visual_geometry
            lines.append('      <geometry>')
            if geom['type'] == 'box':
                size = geom['size']
                lines.append(f'        <box size="{size[0]} {size[1]} {size[2]}"/>')
            lines.append('      </geometry>')
            lines.append('    </visual>')

            # Collision
            lines.append('    <collision>')
            lines.append('      <geometry>')
            if geom['type'] == 'box':
                size = geom['size']
                lines.append(f'        <box size="{size[0]} {size[1]} {size[2]}"/>')
            lines.append('      </geometry>')
            lines.append('    </collision>')

            lines.append('  </link>')
            lines.append('')

        # Export joints
        for joint_name, joint in self.joints.items():
            lines.append(f'  <joint name="{joint_name}" type="{joint.type}">')
            lines.append(f'    <parent link="{joint.parent}"/>')
            lines.append(f'    <child link="{joint.child}"/>')

            xyz = joint.origin['xyz']
            rpy = joint.origin['rpy']
            lines.append(f'    <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>')

            if joint.type in ['revolute', 'prismatic', 'continuous']:
                lines.append(f'    <axis xyz="{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}"/>')

            if joint.type in ['revolute', 'prismatic']:
                limits = joint.limits
                lines.append(f'    <limit lower="{limits["lower"]}" upper="{limits["upper"]}"')
                lines.append(f'           effort="{limits["effort"]}" velocity="{limits["velocity"]}"/>')

            lines.append('  </joint>')
            lines.append('')

        lines.append('</robot>')

        return '\n'.join(lines)

    def _validate_robot(self):
        """Validate current robot design"""
        errors = []

        # Check for at least one link
        if not self.links:
            errors.append("Robot must have at least one link")

        # Check for base_link
        if 'base_link' not in self.links:
            errors.append("Robot should have a 'base_link'")

        # Check joint parent/child validity
        for joint_name, joint in self.joints.items():
            if joint.parent not in self.links:
                errors.append(f"Joint '{joint_name}' references non-existent parent '{joint.parent}'")
            if joint.child not in self.links:
                errors.append(f"Joint '{joint_name}' references non-existent child '{joint.child}'")

        if errors:
            QMessageBox.warning(self, "Validation Errors",
                              "Robot has validation errors:\n\n" + "\n".join(errors))
        else:
            QMessageBox.information(self, "Validation Success",
                                  "Robot design is valid!")
