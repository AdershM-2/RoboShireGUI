"""
Nav2 Integration Wizard - Step-by-step Nav2 setup for mobile robots

This wizard guides users through configuring a complete Nav2 navigation stack
for their mobile robot, including costmap configuration, planners, controllers,
and behavior trees.

Author: RoboShire Team
Version: 2.2.0 (v2.2.0 Advanced: AI-Assisted Configuration)
"""

from PySide6.QtWidgets import (
    QWizard, QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QCheckBox,
    QTextEdit, QPushButton, QGroupBox, QFormLayout, QRadioButton,
    QButtonGroup, QFileDialog, QMessageBox, QProgressBar
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPixmap, QFont
from pathlib import Path
from typing import Dict, Optional
import yaml
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from backend.nav2_generator import Nav2ConfigGenerator


class Nav2IntegrationWizard(QWizard):
    """
    6-page wizard for Nav2 integration

    Pages:
    1. Welcome - Explain Nav2 and requirements
    2. Robot Configuration - Robot dimensions, sensors
    3. Costmap Configuration - Global and local costmaps
    4. Planner Selection - NavFn, SmacPlanner, etc.
    5. Controller Selection - DWB, TEB, RPP
    6. Summary - Review and generate
    """

    nav2_configured = Signal(dict)  # Emits configuration when complete

    # Page IDs
    PAGE_WELCOME = 0
    PAGE_ROBOT_CONFIG = 1
    PAGE_COSTMAP = 2
    PAGE_PLANNER = 3
    PAGE_CONTROLLER = 4
    PAGE_SUMMARY = 5

    def __init__(self, project_dir: Optional[Path] = None, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Nav2 Integration Wizard")
        self.setWizardStyle(QWizard.ModernStyle)
        self.setMinimumSize(800, 600)

        # Configuration storage
        self.config = {}
        self.project_dir = project_dir or Path.cwd()

        # Configuration generator
        self.generator = Nav2ConfigGenerator()

        # Add pages
        self.setPage(self.PAGE_WELCOME, WelcomePage())
        self.setPage(self.PAGE_ROBOT_CONFIG, RobotConfigPage())
        self.setPage(self.PAGE_COSTMAP, CostmapConfigPage())
        self.setPage(self.PAGE_PLANNER, PlannerConfigPage())
        self.setPage(self.PAGE_CONTROLLER, ControllerConfigPage())
        self.setPage(self.PAGE_SUMMARY, SummaryPage())

        # Connect signals
        self.finished.connect(self._on_finished)

    def _on_finished(self, result):
        """Handle wizard completion"""
        if result == QWizard.Accepted:
            # Collect all configuration
            self.config = {
                'robot': self.page(self.PAGE_ROBOT_CONFIG).get_config(),
                'costmap': self.page(self.PAGE_COSTMAP).get_config(),
                'planner': self.page(self.PAGE_PLANNER).get_config(),
                'controller': self.page(self.PAGE_CONTROLLER).get_config()
            }

            # Generate Nav2 configuration files
            try:
                success = self.generator.generate_all(self.config, self.project_dir)

                if success:
                    QMessageBox.information(
                        self,
                        "Nav2 Integration Complete",
                        f"Nav2 configuration generated successfully!\n\n"
                        f"Generated files:\n"
                        f"• config/costmap_common.yaml\n"
                        f"• config/global_costmap.yaml\n"
                        f"• config/local_costmap.yaml\n"
                        f"• config/planner.yaml\n"
                        f"• config/controller.yaml\n"
                        f"• config/nav2_params.yaml\n"
                        f"• launch/nav2_launch.py\n\n"
                        f"Location: {self.project_dir}\n\n"
                        f"Next steps:\n"
                        f"1. Create a map using SLAM\n"
                        f"2. Place map file in {self.project_dir}/maps/my_map.yaml\n"
                        f"3. Launch Nav2: ros2 launch {self.config['robot']['robot_name']} nav2_launch.py"
                    )
                    self.nav2_configured.emit(self.config)
                else:
                    QMessageBox.critical(
                        self,
                        "Nav2 Generation Failed",
                        "Failed to generate Nav2 configuration files.\n"
                        "Check the console for error messages."
                    )

            except Exception as e:
                QMessageBox.critical(
                    self,
                    "Nav2 Generation Error",
                    f"Error generating Nav2 configuration:\n\n{str(e)}"
                )


class WelcomePage(QWizardPage):
    """Welcome page explaining Nav2"""

    def __init__(self):
        super().__init__()

        self.setTitle("Welcome to Nav2 Integration Wizard")
        self.setSubTitle("Configure complete navigation stack for your mobile robot")

        layout = QVBoxLayout(self)

        # Introduction
        intro = QLabel(
            "<h3>What is Nav2?</h3>"
            "<p>Nav2 (Navigation 2) is the ROS2 navigation framework that enables autonomous "
            "navigation for mobile robots. It provides:</p>"
            "<ul>"
            "<li><b>Path Planning</b> - Find optimal paths from A to B</li>"
            "<li><b>Obstacle Avoidance</b> - Detect and avoid obstacles in real-time</li>"
            "<li><b>Localization</b> - Know where the robot is on a map</li>"
            "<li><b>Behavior Trees</b> - Complex navigation behaviors</li>"
            "<li><b>Recovery Behaviors</b> - Handle stuck situations</li>"
            "</ul>"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        # Requirements
        req_group = QGroupBox("Requirements")
        req_layout = QVBoxLayout(req_group)

        requirements = QLabel(
            "Before using this wizard, ensure you have:\n\n"
            "✅ A mobile robot with differential drive or omnidirectional drive\n"
            "✅ Sensor data (LiDAR or depth camera for obstacle detection)\n"
            "✅ Odometry information (wheel encoders or IMU)\n"
            "✅ Basic understanding of your robot's dimensions\n\n"
            "This wizard will generate:\n\n"
            "📦 Nav2 action server nodes (NavigateToPose, FollowPath)\n"
            "📦 Costmap configuration files (global and local)\n"
            "📦 Planner and controller configurations\n"
            "📦 Launch file to start Nav2 stack\n"
            "📦 RViz configuration with Nav2 plugins"
        )
        requirements.setWordWrap(True)
        req_layout.addWidget(requirements)
        layout.addWidget(req_group)

        layout.addStretch()

        # Info label
        info = QLabel(
            "<i>Click 'Next' to begin configuring your Nav2 navigation stack.</i>"
        )
        info.setWordWrap(True)
        layout.addWidget(info)


class RobotConfigPage(QWizardPage):
    """Robot configuration page"""

    def __init__(self):
        super().__init__()

        self.setTitle("Robot Configuration")
        self.setSubTitle("Configure your robot's physical properties")

        layout = QVBoxLayout(self)

        # Robot name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Robot Name:"))
        self.robot_name = QLineEdit("my_robot")
        self.registerField("robot_name*", self.robot_name)
        name_layout.addWidget(self.robot_name)
        layout.addLayout(name_layout)

        # v2.0.1: Namespace support
        namespace_layout = QHBoxLayout()
        namespace_layout.addWidget(QLabel("Namespace (optional):"))
        self.namespace = QLineEdit("")
        self.namespace.setPlaceholderText("Leave empty for global namespace")
        self.namespace.setToolTip("ROS2 namespace for multi-robot systems (e.g., /robot1, /robot2)")
        namespace_layout.addWidget(self.namespace)
        layout.addLayout(namespace_layout)

        # Dimensions group
        dim_group = QGroupBox("Robot Dimensions")
        dim_form = QFormLayout(dim_group)

        self.robot_radius = QDoubleSpinBox()
        self.robot_radius.setRange(0.1, 2.0)
        self.robot_radius.setValue(0.3)
        self.robot_radius.setSuffix(" m")
        self.robot_radius.setDecimals(2)
        dim_form.addRow("Robot Radius (footprint):", self.robot_radius)

        self.max_vel_x = QDoubleSpinBox()
        self.max_vel_x.setRange(0.1, 5.0)
        self.max_vel_x.setValue(0.5)
        self.max_vel_x.setSuffix(" m/s")
        self.max_vel_x.setDecimals(2)
        dim_form.addRow("Max Linear Velocity:", self.max_vel_x)

        self.max_vel_theta = QDoubleSpinBox()
        self.max_vel_theta.setRange(0.1, 5.0)
        self.max_vel_theta.setValue(1.0)
        self.max_vel_theta.setSuffix(" rad/s")
        self.max_vel_theta.setDecimals(2)
        dim_form.addRow("Max Angular Velocity:", self.max_vel_theta)

        self.acc_lim_x = QDoubleSpinBox()
        self.acc_lim_x.setRange(0.1, 5.0)
        self.acc_lim_x.setValue(0.5)
        self.acc_lim_x.setSuffix(" m/s²")
        self.acc_lim_x.setDecimals(2)
        dim_form.addRow("Linear Acceleration Limit:", self.acc_lim_x)

        self.acc_lim_theta = QDoubleSpinBox()
        self.acc_lim_theta.setRange(0.1, 5.0)
        self.acc_lim_theta.setValue(1.0)
        self.acc_lim_theta.setSuffix(" rad/s²")
        self.acc_lim_theta.setDecimals(2)
        dim_form.addRow("Angular Acceleration Limit:", self.acc_lim_theta)

        layout.addWidget(dim_group)

        # Drive type
        drive_group = QGroupBox("Drive Type")
        drive_layout = QVBoxLayout(drive_group)

        self.drive_type_group = QButtonGroup(self)

        self.diff_drive = QRadioButton("Differential Drive (2 wheels)")
        self.diff_drive.setChecked(True)
        self.drive_type_group.addButton(self.diff_drive, 0)
        drive_layout.addWidget(self.diff_drive)

        self.omni_drive = QRadioButton("Omnidirectional Drive (mecanum/omni wheels)")
        self.drive_type_group.addButton(self.omni_drive, 1)
        drive_layout.addWidget(self.omni_drive)

        self.ackermann_drive = QRadioButton("Ackermann Drive (car-like)")
        self.drive_type_group.addButton(self.ackermann_drive, 2)
        drive_layout.addWidget(self.ackermann_drive)

        layout.addWidget(drive_group)

        # Sensor configuration
        sensor_group = QGroupBox("Sensors")
        sensor_layout = QVBoxLayout(sensor_group)

        self.has_lidar = QCheckBox("LiDAR Scanner")
        self.has_lidar.setChecked(True)
        sensor_layout.addWidget(self.has_lidar)

        self.has_camera = QCheckBox("Depth Camera")
        sensor_layout.addWidget(self.has_camera)

        self.has_sonar = QCheckBox("Ultrasonic Sensors")
        sensor_layout.addWidget(self.has_sonar)

        layout.addWidget(sensor_group)

        layout.addStretch()

    def get_config(self) -> Dict:
        """Get robot configuration (v2.0.1: with namespace support)"""
        drive_types = ["differential", "omnidirectional", "ackermann"]

        return {
            'robot_name': self.robot_name.text(),
            'namespace': self.namespace.text().strip(),  # v2.0.1: namespace
            'robot_radius': self.robot_radius.value(),
            'max_vel_x': self.max_vel_x.value(),
            'max_vel_theta': self.max_vel_theta.value(),
            'acc_lim_x': self.acc_lim_x.value(),
            'acc_lim_theta': self.acc_lim_theta.value(),
            'drive_type': drive_types[self.drive_type_group.checkedId()],
            'sensors': {
                'lidar': self.has_lidar.isChecked(),
                'camera': self.has_camera.isChecked(),
                'sonar': self.has_sonar.isChecked()
            }
        }


class CostmapConfigPage(QWizardPage):
    """Costmap configuration page"""

    def __init__(self):
        super().__init__()

        self.setTitle("Costmap Configuration")
        self.setSubTitle("Configure obstacle detection and path planning costmaps")

        layout = QVBoxLayout(self)

        # Explanation
        exp = QLabel(
            "<b>Costmaps</b> are 2D grids representing obstacles and free space:\n"
            "• <b>Global Costmap</b> - Large area, for long-range planning\n"
            "• <b>Local Costmap</b> - Small area around robot, for obstacle avoidance"
        )
        exp.setWordWrap(True)
        layout.addWidget(exp)

        # Global costmap
        global_group = QGroupBox("Global Costmap")
        global_form = QFormLayout(global_group)

        self.global_width = QDoubleSpinBox()
        self.global_width.setRange(10.0, 200.0)
        self.global_width.setValue(50.0)
        self.global_width.setSuffix(" m")
        global_form.addRow("Width:", self.global_width)

        self.global_height = QDoubleSpinBox()
        self.global_height.setRange(10.0, 200.0)
        self.global_height.setValue(50.0)
        self.global_height.setSuffix(" m")
        global_form.addRow("Height:", self.global_height)

        self.global_resolution = QDoubleSpinBox()
        self.global_resolution.setRange(0.01, 0.5)
        self.global_resolution.setValue(0.05)
        self.global_resolution.setSuffix(" m/cell")
        self.global_resolution.setDecimals(3)
        global_form.addRow("Resolution:", self.global_resolution)

        self.global_update_frequency = QDoubleSpinBox()
        self.global_update_frequency.setRange(0.5, 10.0)
        self.global_update_frequency.setValue(1.0)
        self.global_update_frequency.setSuffix(" Hz")
        global_form.addRow("Update Frequency:", self.global_update_frequency)

        layout.addWidget(global_group)

        # Local costmap
        local_group = QGroupBox("Local Costmap")
        local_form = QFormLayout(local_group)

        self.local_width = QDoubleSpinBox()
        self.local_width.setRange(2.0, 20.0)
        self.local_width.setValue(5.0)
        self.local_width.setSuffix(" m")
        local_form.addRow("Width:", self.local_width)

        self.local_height = QDoubleSpinBox()
        self.local_height.setRange(2.0, 20.0)
        self.local_height.setValue(5.0)
        self.local_height.setSuffix(" m")
        local_form.addRow("Height:", self.local_height)

        self.local_resolution = QDoubleSpinBox()
        self.local_resolution.setRange(0.01, 0.2)
        self.local_resolution.setValue(0.05)
        self.local_resolution.setSuffix(" m/cell")
        self.local_resolution.setDecimals(3)
        local_form.addRow("Resolution:", self.local_resolution)

        self.local_update_frequency = QDoubleSpinBox()
        self.local_update_frequency.setRange(1.0, 20.0)
        self.local_update_frequency.setValue(5.0)
        self.local_update_frequency.setSuffix(" Hz")
        local_form.addRow("Update Frequency:", self.local_update_frequency)

        layout.addWidget(local_group)

        # Obstacle settings
        obs_group = QGroupBox("Obstacle Layer")
        obs_form = QFormLayout(obs_group)

        self.obstacle_range = QDoubleSpinBox()
        self.obstacle_range.setRange(1.0, 20.0)
        self.obstacle_range.setValue(6.0)
        self.obstacle_range.setSuffix(" m")
        obs_form.addRow("Obstacle Range:", self.obstacle_range)

        self.raytrace_range = QDoubleSpinBox()
        self.raytrace_range.setRange(1.0, 20.0)
        self.raytrace_range.setValue(8.0)
        self.raytrace_range.setSuffix(" m")
        obs_form.addRow("Raytrace Range:", self.raytrace_range)

        # v2.0.1: Inflation radius for validation
        self.inflation_radius = QDoubleSpinBox()
        self.inflation_radius.setRange(0.0, 1.0)
        self.inflation_radius.setValue(0.25)
        self.inflation_radius.setSuffix(" m")
        self.inflation_radius.setDecimals(2)
        self.inflation_radius.setToolTip("Safety margin around obstacles (typically slightly larger than robot radius)")
        obs_form.addRow("Inflation Radius:", self.inflation_radius)

        layout.addWidget(obs_group)

        # v2.0.1: Validation warnings area
        self.validation_label = QLabel("")
        self.validation_label.setWordWrap(True)
        self.validation_label.setStyleSheet("color: #d9534f; padding: 5px; background-color: #f2dede; border-radius: 3px;")
        self.validation_label.setVisible(False)
        layout.addWidget(self.validation_label)

        layout.addStretch()

    def get_config(self) -> Dict:
        """Get costmap configuration (v2.0.1: with inflation radius)"""
        return {
            'global': {
                'width': self.global_width.value(),
                'height': self.global_height.value(),
                'resolution': self.global_resolution.value(),
                'update_frequency': self.global_update_frequency.value()
            },
            'local': {
                'width': self.local_width.value(),
                'height': self.local_height.value(),
                'resolution': self.local_resolution.value(),
                'update_frequency': self.local_update_frequency.value()
            },
            'obstacle': {
                'obstacle_range': self.obstacle_range.value(),
                'raytrace_range': self.raytrace_range.value(),
                'inflation_radius': self.inflation_radius.value()  # v2.0.1
            }
        }

    def validatePage(self) -> bool:
        """
        v2.0.1: Validate costmap configuration before proceeding

        Checks:
        - Inflation radius vs robot radius
        - Costmap size vs computational load
        - Resolution appropriateness
        """
        # Get robot config from previous page
        robot_page = self.wizard().page(self.wizard().PAGE_ROBOT_CONFIG)
        if not robot_page:
            return True

        robot_config = robot_page.get_config()
        robot_radius = robot_config.get('robot_radius', 0.3)
        inflation_radius = self.inflation_radius.value()

        warnings = []

        # Check 1: Inflation radius should be >= robot radius for safety
        if inflation_radius < robot_radius:
            warnings.append(
                f"⚠️ Inflation radius ({inflation_radius:.2f}m) is smaller than robot radius ({robot_radius:.2f}m). "
                f"This may cause collisions! Recommended: {robot_radius + 0.05:.2f}m"
            )

        # Check 2: Verify aisle clearance (simulate narrow corridor scenario)
        aisle_width = 1.5  # Typical indoor corridor
        total_clearance_needed = 2 * (robot_radius + inflation_radius)
        min_clearance = 0.1  # Minimum safe clearance on each side

        effective_width = aisle_width - total_clearance_needed
        if effective_width < min_clearance:
            warnings.append(
                f"⚠️ Robot may not fit through narrow spaces! "
                f"In a {aisle_width:.1f}m corridor, effective clearance is only {effective_width:.2f}m. "
                f"Consider reducing inflation radius to {(aisle_width/2 - robot_radius - min_clearance):.2f}m max."
            )

        # Check 3: Resolution vs costmap size (memory/CPU warning)
        global_cells = (self.global_width.value() / self.global_resolution.value()) * \
                      (self.global_height.value() / self.global_resolution.value())
        if global_cells > 1_000_000:  # 1 million cells
            warnings.append(
                f"⚠️ Global costmap has {int(global_cells):,} cells. This may be computationally expensive! "
                f"Consider increasing resolution or reducing size."
            )

        # Check 4: Local update frequency (should be higher for real-time obstacle avoidance)
        if self.local_update_frequency.value() < 5.0:
            warnings.append(
                f"⚠️ Local costmap update frequency is low ({self.local_update_frequency.value():.1f} Hz). "
                f"For dynamic obstacle avoidance, consider ≥5 Hz."
            )

        # Display warnings if any
        if warnings:
            self.validation_label.setText("\n\n".join(warnings))
            self.validation_label.setVisible(True)

            # Ask user if they want to proceed despite warnings
            from PySide6.QtWidgets import QMessageBox
            reply = QMessageBox.question(
                self,
                "Configuration Warnings",
                "\n\n".join(warnings) + "\n\nDo you want to proceed anyway?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )

            return reply == QMessageBox.Yes
        else:
            self.validation_label.setVisible(False)
            return True


class PlannerConfigPage(QWizardPage):
    """Planner selection page"""

    def __init__(self):
        super().__init__()

        self.setTitle("Path Planner Selection")
        self.setSubTitle("Choose algorithm for global path planning")

        layout = QVBoxLayout(self)

        # Explanation
        exp = QLabel(
            "<b>Path Planners</b> find optimal paths from start to goal:\n"
            "• <b>NavFn</b> - Classic Dijkstra-based planner (fast, simple)\n"
            "• <b>SmacPlanner</b> - State lattice planner (considers robot kinematics)\n"
            "• <b>ThetaStar</b> - Any-angle planner (smoother paths)"
        )
        exp.setWordWrap(True)
        layout.addWidget(exp)

        # Planner selection
        planner_group = QGroupBox("Select Planner")
        planner_layout = QVBoxLayout(planner_group)

        self.planner_type_group = QButtonGroup(self)

        navfn = QRadioButton("NavFn Planner (Recommended for beginners)")
        navfn.setChecked(True)
        self.planner_type_group.addButton(navfn, 0)
        planner_layout.addWidget(navfn)

        navfn_desc = QLabel("  • Fast and reliable\n  • Works well in most environments\n  • Simple configuration")
        navfn_desc.setStyleSheet("color: #666; margin-left: 20px;")
        planner_layout.addWidget(navfn_desc)

        smac = QRadioButton("SmacPlanner2D (For complex environments)")
        self.planner_type_group.addButton(smac, 1)
        planner_layout.addWidget(smac)

        smac_desc = QLabel("  • Considers robot size and kinematics\n  • Better for tight spaces\n  • Slower but more accurate")
        smac_desc.setStyleSheet("color: #666; margin-left: 20px;")
        planner_layout.addWidget(smac_desc)

        theta = QRadioButton("ThetaStar Planner (For smooth paths)")
        self.planner_type_group.addButton(theta, 2)
        planner_layout.addWidget(theta)

        theta_desc = QLabel("  • Any-angle paths (not grid-aligned)\n  • Smoother trajectories\n  • Good for open spaces")
        theta_desc.setStyleSheet("color: #666; margin-left: 20px;")
        planner_layout.addWidget(theta_desc)

        layout.addWidget(planner_group)

        # v2.2.0: AI Assistant button
        ai_layout = QHBoxLayout()
        ai_button = QPushButton("🤖 Get AI Recommendation")
        ai_button.setToolTip("Get intelligent planner suggestion based on your robot configuration")
        ai_button.clicked.connect(self._show_ai_recommendation)
        ai_layout.addWidget(ai_button)
        ai_layout.addStretch()
        layout.addLayout(ai_layout)

        # v2.2.0: AI recommendation display
        self.ai_recommendation_label = QLabel("")
        self.ai_recommendation_label.setWordWrap(True)
        self.ai_recommendation_label.setStyleSheet(
            "background-color: #e3f2fd; color: #1565c0; padding: 10px; "
            "border-radius: 5px; border: 1px solid #90caf9;"
        )
        self.ai_recommendation_label.setVisible(False)
        layout.addWidget(self.ai_recommendation_label)

        # Planner parameters
        params_group = QGroupBox("Planner Parameters")
        params_form = QFormLayout(params_group)

        self.tolerance = QDoubleSpinBox()
        self.tolerance.setRange(0.01, 1.0)
        self.tolerance.setValue(0.1)
        self.tolerance.setSuffix(" m")
        self.tolerance.setDecimals(2)
        params_form.addRow("Goal Tolerance:", self.tolerance)

        self.use_final_approach_orientation = QCheckBox("Match goal orientation")
        self.use_final_approach_orientation.setChecked(True)
        params_form.addRow("Final Approach:", self.use_final_approach_orientation)

        layout.addWidget(params_group)

        layout.addStretch()

    def _show_ai_recommendation(self):
        """
        v2.2.0: Show AI-powered planner recommendation

        Rule-based expert system that analyzes robot configuration
        and suggests optimal planner based on characteristics
        """
        # Get robot configuration from previous pages
        robot_page = self.wizard().page(self.wizard().PAGE_ROBOT_CONFIG)
        costmap_page = self.wizard().page(self.wizard().PAGE_COSTMAP)

        if not robot_page or not costmap_page:
            self.ai_recommendation_label.setText(
                "⚠️ Unable to get robot configuration. Please complete previous pages first."
            )
            self.ai_recommendation_label.setVisible(True)
            return

        robot_config = robot_page.get_config()
        costmap_config = costmap_page.get_config()

        # Extract key parameters for decision making
        robot_radius = robot_config.get('robot_radius', 0.3)
        drive_type = robot_config.get('drive_type', 'differential')
        max_vel = robot_config.get('max_vel_x', 0.5)
        local_costmap_size = costmap_config['local']['width']
        resolution = costmap_config['global']['resolution']

        # AI Expert System Logic
        recommendation = self._ai_planner_expert_system(
            robot_radius, drive_type, max_vel, local_costmap_size, resolution
        )

        # Display recommendation
        planner_name = recommendation['planner']
        confidence = recommendation['confidence']
        reasoning = recommendation['reasoning']

        # Auto-select the recommended planner
        planner_map = {"NavFn": 0, "SmacPlanner2D": 1, "ThetaStar": 2}
        if planner_name in planner_map:
            self.planner_type_group.button(planner_map[planner_name]).setChecked(True)

        # Show recommendation
        self.ai_recommendation_label.setText(
            f"🤖 <b>AI Recommendation: {planner_name}</b> (Confidence: {confidence*100:.0f}%)\n\n"
            f"<b>Reasoning:</b> {reasoning}\n\n"
            f"<i>The planner has been automatically selected above. You can change it if needed.</i>"
        )
        self.ai_recommendation_label.setVisible(True)

    def _ai_planner_expert_system(
        self,
        robot_radius: float,
        drive_type: str,
        max_vel: float,
        local_costmap_size: float,
        resolution: float
    ) -> Dict:
        """
        v2.2.0: Rule-based expert system for planner selection

        Decision tree based on robotics navigation best practices
        """
        scores = {
            'NavFn': 0.0,
            'SmacPlanner2D': 0.0,
            'ThetaStar': 0.0
        }

        # Rule 1: Robot size vs environment complexity
        if robot_radius > 0.5:  # Large robot
            scores['SmacPlanner2D'] += 0.4  # Needs kinematic awareness
            scores['NavFn'] += 0.2
        else:  # Small robot
            scores['NavFn'] += 0.3  # Fast and simple is fine
            scores['ThetaStar'] += 0.2

        # Rule 2: Drive type kinematics
        if drive_type == 'ackermann':  # Car-like
            scores['SmacPlanner2D'] += 0.5  # MUST consider turning radius
            scores['NavFn'] -= 0.2  # Not suitable
        elif drive_type == 'differential':
            scores['NavFn'] += 0.3  # Good for differential
            scores['ThetaStar'] += 0.2
        else:  # omnidirectional
            scores['ThetaStar'] += 0.3  # Can take advantage of smoothness
            scores['NavFn'] += 0.2

        # Rule 3: Speed requirements
        if max_vel > 1.0:  # Fast robot
            scores['NavFn'] += 0.4  # Fastest planner
            scores['ThetaStar'] += 0.2
            scores['SmacPlanner2D'] -= 0.1  # Slower
        else:
            scores['SmacPlanner2D'] += 0.2  # Can afford computation
            scores['ThetaStar'] += 0.3

        # Rule 4: Environment complexity (inferred from costmap)
        if local_costmap_size < 5.0:  # Tight spaces
            scores['SmacPlanner2D'] += 0.4  # Best for tight spaces
            scores['NavFn'] += 0.1
        elif local_costmap_size > 15.0:  # Open spaces
            scores['ThetaStar'] += 0.4  # Smooth paths in open areas
            scores['NavFn'] += 0.3
        else:  # Medium spaces
            scores['NavFn'] += 0.3  # Balanced
            scores['SmacPlanner2D'] += 0.2

        # Rule 5: Resolution (computational constraints)
        if resolution < 0.03:  # High resolution = lots of computation
            scores['NavFn'] += 0.3  # Lighter weight
            scores['SmacPlanner2D'] -= 0.2
        else:
            scores['SmacPlanner2D'] += 0.2  # Can afford it
            scores['ThetaStar'] += 0.1

        # Normalize scores to 0-1 range
        max_score = max(scores.values())
        if max_score > 0:
            scores = {k: v / max_score for k, v in scores.items()}

        # Find best planner
        best_planner = max(scores, key=scores.get)
        confidence = scores[best_planner]

        # Generate human-readable reasoning
        reasoning_map = {
            'NavFn': f"Your robot (radius: {robot_radius:.2f}m, {drive_type} drive, max velocity: {max_vel:.1f}m/s) will work well with NavFn. It's fast, reliable, and suitable for most environments. The relatively simple kinematics make complex planners unnecessary.",
            'SmacPlanner2D': f"Your robot configuration ({drive_type} drive, radius: {robot_radius:.2f}m) benefits from SmacPlanner2D's kinematic awareness. {'This is essential for Ackermann steering.' if drive_type == 'ackermann' else 'The tight local costmap suggests complex environments where considering robot size is important.'}",
            'ThetaStar': f"Your robot (omnidirectional drive, large costmap size: {local_costmap_size:.1f}m) can take advantage of ThetaStar's smooth, any-angle paths. Good for open environments where straight-line paths aren't always grid-aligned."
        }

        return {
            'planner': best_planner,
            'confidence': confidence,
            'reasoning': reasoning_map[best_planner],
            'scores': scores  # For debugging/transparency
        }

    def get_config(self) -> Dict:
        """Get planner configuration"""
        planners = ["NavFn", "SmacPlanner2D", "ThetaStar"]

        return {
            'planner_type': planners[self.planner_type_group.checkedId()],
            'tolerance': self.tolerance.value(),
            'use_final_approach_orientation': self.use_final_approach_orientation.isChecked()
        }


class ControllerConfigPage(QWizardPage):
    """Controller selection page"""

    def __init__(self):
        super().__init__()

        self.setTitle("Controller Selection")
        self.setSubTitle("Choose algorithm for path following and obstacle avoidance")

        layout = QVBoxLayout(self)

        # Explanation
        exp = QLabel(
            "<b>Controllers</b> follow planned paths while avoiding obstacles:\n"
            "• <b>DWB</b> - Dynamic Window Approach (most popular, reliable)\n"
            "• <b>TEB</b> - Timed Elastic Band (smooth, optimal)\n"
            "• <b>RPP</b> - Regulated Pure Pursuit (simple, fast)"
        )
        exp.setWordWrap(True)
        layout.addWidget(exp)

        # Controller selection
        controller_group = QGroupBox("Select Controller")
        controller_layout = QVBoxLayout(controller_group)

        self.controller_type_group = QButtonGroup(self)

        dwb = QRadioButton("DWB Controller (Recommended)")
        dwb.setChecked(True)
        self.controller_type_group.addButton(dwb, 0)
        controller_layout.addWidget(dwb)

        dwb_desc = QLabel("  • Most widely used\n  • Good balance of speed and accuracy\n  • Works well for differential drive")
        dwb_desc.setStyleSheet("color: #666; margin-left: 20px;")
        controller_layout.addWidget(dwb_desc)

        teb = QRadioButton("TEB Controller (For smooth motion)")
        self.controller_type_group.addButton(teb, 1)
        controller_layout.addWidget(teb)

        teb_desc = QLabel("  • Optimal trajectory generation\n  • Smooth acceleration profiles\n  • Higher computational cost")
        teb_desc.setStyleSheet("color: #666; margin-left: 20px;")
        controller_layout.addWidget(teb_desc)

        rpp = QRadioButton("RPP Controller (For simple paths)")
        self.controller_type_group.addButton(rpp, 2)
        controller_layout.addWidget(rpp)

        rpp_desc = QLabel("  • Fastest controller\n  • Simple pure pursuit algorithm\n  • Best for open environments")
        rpp_desc.setStyleSheet("color: #666; margin-left: 20px;")
        controller_layout.addWidget(rpp_desc)

        layout.addWidget(controller_group)

        # Controller parameters
        params_group = QGroupBox("Controller Parameters")
        params_form = QFormLayout(params_group)

        self.max_vel_x_controller = QDoubleSpinBox()
        self.max_vel_x_controller.setRange(0.1, 5.0)
        self.max_vel_x_controller.setValue(0.5)
        self.max_vel_x_controller.setSuffix(" m/s")
        params_form.addRow("Max Velocity X:", self.max_vel_x_controller)

        self.max_vel_theta_controller = QDoubleSpinBox()
        self.max_vel_theta_controller.setRange(0.1, 5.0)
        self.max_vel_theta_controller.setValue(1.0)
        self.max_vel_theta_controller.setSuffix(" rad/s")
        params_form.addRow("Max Velocity Theta:", self.max_vel_theta_controller)

        self.xy_goal_tolerance = QDoubleSpinBox()
        self.xy_goal_tolerance.setRange(0.01, 1.0)
        self.xy_goal_tolerance.setValue(0.1)
        self.xy_goal_tolerance.setSuffix(" m")
        self.xy_goal_tolerance.setDecimals(2)
        params_form.addRow("XY Goal Tolerance:", self.xy_goal_tolerance)

        self.yaw_goal_tolerance = QDoubleSpinBox()
        self.yaw_goal_tolerance.setRange(0.01, 1.0)
        self.yaw_goal_tolerance.setValue(0.1)
        self.yaw_goal_tolerance.setSuffix(" rad")
        self.yaw_goal_tolerance.setDecimals(2)
        params_form.addRow("Yaw Goal Tolerance:", self.yaw_goal_tolerance)

        layout.addWidget(params_group)

        layout.addStretch()

    def get_config(self) -> Dict:
        """Get controller configuration"""
        controllers = ["DWB", "TEB", "RPP"]

        return {
            'controller_type': controllers[self.controller_type_group.checkedId()],
            'max_vel_x': self.max_vel_x_controller.value(),
            'max_vel_theta': self.max_vel_theta_controller.value(),
            'xy_goal_tolerance': self.xy_goal_tolerance.value(),
            'yaw_goal_tolerance': self.yaw_goal_tolerance.value()
        }


class SummaryPage(QWizardPage):
    """Summary page showing configuration overview"""

    def __init__(self):
        super().__init__()

        self.setTitle("Configuration Summary")
        self.setSubTitle("Review your Nav2 configuration")

        layout = QVBoxLayout(self)

        # Summary text
        self.summary_text = QTextEdit()
        self.summary_text.setReadOnly(True)
        font = QFont("Consolas", 9)
        self.summary_text.setFont(font)
        layout.addWidget(self.summary_text)

        # Info
        info = QLabel(
            "<b>What will be generated:</b>\n"
            "• Action server nodes (NavigateToPose, FollowPath, etc.)\n"
            "• Costmap configuration YAML files\n"
            "• Planner and controller parameter files\n"
            "• Nav2 launch file\n"
            "• RViz configuration with Nav2 plugins\n\n"
            "<i>Click 'Finish' to generate Nav2 integration.</i>"
        )
        info.setWordWrap(True)
        layout.addWidget(info)

    def initializePage(self):
        """Update summary when page is shown"""
        wizard = self.wizard()

        robot_config = wizard.page(Nav2IntegrationWizard.PAGE_ROBOT_CONFIG).get_config()
        costmap_config = wizard.page(Nav2IntegrationWizard.PAGE_COSTMAP).get_config()
        planner_config = wizard.page(Nav2IntegrationWizard.PAGE_PLANNER).get_config()
        controller_config = wizard.page(Nav2IntegrationWizard.PAGE_CONTROLLER).get_config()

        summary = f"""
Nav2 Configuration Summary
==========================

Robot Configuration:
  Name: {robot_config['robot_name']}
  Drive Type: {robot_config['drive_type']}
  Radius: {robot_config['robot_radius']} m
  Max Linear Velocity: {robot_config['max_vel_x']} m/s
  Max Angular Velocity: {robot_config['max_vel_theta']} rad/s
  Sensors: {', '.join([k for k, v in robot_config['sensors'].items() if v])}

Costmap Configuration:
  Global Costmap: {costmap_config['global']['width']} x {costmap_config['global']['height']} m
  Global Resolution: {costmap_config['global']['resolution']} m/cell
  Local Costmap: {costmap_config['local']['width']} x {costmap_config['local']['height']} m
  Local Resolution: {costmap_config['local']['resolution']} m/cell

Planner Configuration:
  Planner: {planner_config['planner_type']}
  Goal Tolerance: {planner_config['tolerance']} m

Controller Configuration:
  Controller: {controller_config['controller_type']}
  Max Velocity: {controller_config['max_vel_x']} m/s
  XY Goal Tolerance: {controller_config['xy_goal_tolerance']} m
  Yaw Goal Tolerance: {controller_config['yaw_goal_tolerance']} rad
"""

        self.summary_text.setPlainText(summary)


# Standalone test
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    wizard = Nav2IntegrationWizard()
    wizard.nav2_configured.connect(lambda cfg: print(f"Nav2 configured: {cfg}"))
    wizard.show()

    sys.exit(app.exec())
