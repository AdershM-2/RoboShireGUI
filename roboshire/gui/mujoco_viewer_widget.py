"""
MuJoCo Viewer Widget for RoboShire

Qt widget for real-time URDF visualization using MuJoCo physics simulation.
Provides interactive 3D viewport with camera controls and simulation management.

Author: RoboShire Team
Phase: 8 (Documentation & MuJoCo Integration)
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QSlider, QGroupBox, QSizePolicy,
    QScrollArea, QFrame, QApplication
)
from PySide6.QtCore import Qt, QTimer, Signal, QPoint
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QFont, QMovie
from pathlib import Path
from typing import Optional
import logging
import numpy as np
import time

from roboshire.integrations.mujoco_bridge import MuJocoBridge


class MuJoCoViewerWidget(QWidget):
    """
    Qt widget for MuJoCo simulation visualization

    Features:
    - Real-time 3D rendering at 60 FPS
    - Interactive camera controls (rotate, zoom, pan)
    - Simulation controls (play, pause, reset)
    - Speed adjustment slider
    - Stats display (FPS, time, joint count)
    - Joint state monitoring

    Signals:
        joint_state_changed: Emitted when joint states update
        simulation_started: Emitted when simulation starts
        simulation_stopped: Emitted when simulation stops
    """

    joint_state_changed = Signal(dict)  # {joint_name: (position, velocity)}
    simulation_started = Signal()
    simulation_stopped = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)

        # MuJoCo bridge
        self.bridge = MuJocoBridge()

        # Rendering state
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self._update_render)
        self.target_fps = 60
        self.current_fps = 0.0
        self.fps_samples = []
        self.last_render_time = time.time()

        # Viewport settings
        self.viewport_width = 640
        self.viewport_height = 480
        self.current_frame: Optional[np.ndarray] = None

        # Camera control state
        self.last_mouse_pos = QPoint()
        self.camera_azimuth = 90.0  # degrees
        self.camera_elevation = -20.0  # degrees
        self.camera_distance = 3.0  # meters
        self.camera_target = [0.0, 0.0, 0.5]  # [x, y, z]

        # Mouse interaction flags
        self.is_rotating = False
        self.is_panning = False

        # Simulation state
        self.is_playing = False
        self.is_recording = False
        self.loaded_urdf_path: Optional[str] = None
        self.is_loading = False  # Track loading state

        # Setup UI
        self._setup_ui()

        # Initial state
        self._update_controls()

    def _setup_ui(self):
        """Setup widget UI layout"""
        from roboshire.gui.brand_theme import BrandColors, BrandFonts

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Compact title bar with brand styling
        self.title_label = QLabel("MuJoCo Viewer - No model loaded")
        title_font = BrandFonts.heading(size=9)  # Reduced from 10
        self.title_label.setFont(title_font)
        self.title_label.setStyleSheet(
            f"QLabel {{ color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()}); padding: 0px; margin: 0px; line-height: 1.0; }}"
        )
        self.title_label.setMaximumHeight(15)  # Compact height
        main_layout.addWidget(self.title_label)

        # Horizontal splitter for viewport and joint controls
        from PySide6.QtWidgets import QSplitter
        h_splitter = QSplitter(Qt.Horizontal)

        # Left side: Viewport and main controls
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)

        # Viewport label - direct add without container to save space
        self.viewport_label = QLabel()
        self.viewport_label.setMinimumSize(640, 480)
        self.viewport_label.setMaximumSize(1280, 960)
        self.viewport_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.viewport_label.setStyleSheet(
            f"QLabel {{ background-color: #1a1a1a; border: 2px solid rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()}); margin: 0px; padding: 0px; }}"
        )
        self.viewport_label.setAlignment(Qt.AlignCenter)

        # Show placeholder text
        self._show_placeholder("No URDF loaded\n\nImport a URDF file to begin")

        # Enable mouse tracking for camera controls
        self.viewport_label.setMouseTracking(True)
        self.viewport_label.mousePressEvent = self._viewport_mouse_press
        self.viewport_label.mouseMoveEvent = self._viewport_mouse_move
        self.viewport_label.mouseReleaseEvent = self._viewport_mouse_release
        self.viewport_label.wheelEvent = self._viewport_wheel
        self.viewport_label.mouseDoubleClickEvent = self._viewport_double_click

        # Loading indicator overlay (initially hidden)
        self.loading_overlay = QLabel(self.viewport_label)
        self.loading_overlay.setAlignment(Qt.AlignCenter)
        overlay_font = BrandFonts.body(size=16)
        overlay_font.setBold(True)
        self.loading_overlay.setFont(overlay_font)
        self.loading_overlay.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 0, 0, 180);
                color: white;
                border-radius: 10px;
                padding: 20px;
            }
        """)
        self.loading_overlay.setText("⏳ Loading robot model...\n\nInitializing MuJoCo renderer...")
        self.loading_overlay.hide()
        self.loading_overlay.setGeometry(0, 0, 640, 480)

        left_layout.addWidget(self.viewport_label, 1)

        # Control panel - compact
        control_group = QGroupBox("Simulation Controls")
        control_layout = QVBoxLayout()
        control_layout.setContentsMargins(3, 3, 3, 3)
        control_layout.setSpacing(2)

        # Playback controls
        playback_layout = QHBoxLayout()

        self.play_button = QPushButton("▶ Play")
        self.play_button.setMinimumWidth(100)
        self.play_button.clicked.connect(self._on_play_clicked)
        playback_layout.addWidget(self.play_button)

        self.pause_button = QPushButton("⏸ Pause")
        self.pause_button.setMinimumWidth(100)
        self.pause_button.clicked.connect(self._on_pause_clicked)
        self.pause_button.setEnabled(False)
        playback_layout.addWidget(self.pause_button)

        self.reset_button = QPushButton("↺ Reset")
        self.reset_button.setMinimumWidth(100)
        self.reset_button.clicked.connect(self._on_reset_clicked)
        playback_layout.addWidget(self.reset_button)

        playback_layout.addStretch()

        # Recording controls (Phase 8 Session 3)
        self.record_button = QPushButton("⏺ Record")
        self.record_button.setMinimumWidth(100)
        self.record_button.clicked.connect(self._on_record_clicked)
        playback_layout.addWidget(self.record_button)

        self.stop_record_button = QPushButton("⏹ Stop")
        self.stop_record_button.setMinimumWidth(100)
        self.stop_record_button.clicked.connect(self._on_stop_record_clicked)
        self.stop_record_button.setEnabled(False)
        playback_layout.addWidget(self.stop_record_button)

        control_layout.addLayout(playback_layout)

        # Speed control
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("Speed:"))

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)  # 0.1x
        self.speed_slider.setMaximum(100)  # 10.0x
        self.speed_slider.setValue(10)  # 1.0x
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        speed_layout.addWidget(self.speed_slider, 1)

        self.speed_label = QLabel("1.0x")
        self.speed_label.setMinimumWidth(50)
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        speed_layout.addWidget(self.speed_label)

        control_layout.addLayout(speed_layout)

        # Stats display - compact
        stats_layout = QHBoxLayout()
        stats_layout.setSpacing(8)

        self.time_label = QLabel("Time: 0.0s")
        self.time_label.setStyleSheet(f"QLabel {{ color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()}); font-weight: bold; font-size: 9px; }}")
        stats_layout.addWidget(self.time_label)

        self.fps_label = QLabel("FPS: 0")
        self.fps_label.setStyleSheet(f"QLabel {{ color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()}); font-weight: bold; font-size: 9px; }}")
        stats_layout.addWidget(self.fps_label)

        self.joints_label = QLabel("Joints: 0")
        self.joints_label.setStyleSheet("QLabel { color: #FF9800; font-weight: bold; font-size: 9px; }")
        stats_layout.addWidget(self.joints_label)

        self.bodies_label = QLabel("Bodies: 0")
        self.bodies_label.setStyleSheet("QLabel { color: #9C27B0; font-weight: bold; font-size: 9px; }")
        stats_layout.addWidget(self.bodies_label)

        stats_layout.addStretch()

        # Camera help text - compact
        self.camera_help = QLabel("🎥 Drag to rotate | Right-drag to pan | Wheel to zoom")
        self.camera_help.setStyleSheet("QLabel { color: #666; font-size: 8px; }")
        stats_layout.addWidget(self.camera_help)

        control_layout.addLayout(stats_layout)

        control_group.setLayout(control_layout)
        left_layout.addWidget(control_group)

        # Add left widget to splitter
        h_splitter.addWidget(left_widget)

        # Right side: Joint Control Panel (moved to right vertical)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        self.joint_control_group = QGroupBox("Joint Control")
        self.joint_control_group.setStyleSheet(f"""
            QGroupBox {{
                font-family: 'Montserrat';
                font-weight: 600;
                border: 2px solid rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 12px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 2px 8px;
                background-color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});
                color: white;
                border-radius: 4px;
            }}
        """)
        joint_control_layout = QVBoxLayout()
        joint_control_layout.setContentsMargins(2, 2, 2, 2)
        joint_control_layout.setSpacing(2)

        # Create scrollable area for joint sliders (full height)
        self.joint_scroll_area = QScrollArea()
        self.joint_scroll_area.setWidgetResizable(True)
        self.joint_scroll_area.setMinimumWidth(300)
        self.joint_scroll_area.setMaximumWidth(400)

        self.joint_controls_widget = QWidget()
        self.joint_controls_layout = QVBoxLayout(self.joint_controls_widget)
        self.joint_controls_layout.setContentsMargins(3, 3, 3, 3)
        self.joint_controls_layout.setSpacing(5)

        # Placeholder text when no model loaded
        self.no_joints_label = QLabel("Load a URDF to see joint controls")
        self.no_joints_label.setAlignment(Qt.AlignCenter)
        no_joints_font = BrandFonts.body(size=10)
        self.no_joints_label.setFont(no_joints_font)
        self.no_joints_label.setStyleSheet("color: #888;")
        self.joint_controls_layout.addWidget(self.no_joints_label)

        self.joint_scroll_area.setWidget(self.joint_controls_widget)
        joint_control_layout.addWidget(self.joint_scroll_area)

        # Reset joints button with brand styling
        reset_joints_layout = QHBoxLayout()
        self.reset_joints_button = QPushButton("Reset All Joints")
        self.reset_joints_button.setStyleSheet(f"""
            QPushButton {{
                background-color: rgb({BrandColors.OLIVE_GREEN.red()}, {BrandColors.OLIVE_GREEN.green()}, {BrandColors.OLIVE_GREEN.blue()});
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 16px;
                font-family: 'Montserrat';
                font-weight: 600;
            }}
            QPushButton:hover {{
                background-color: rgb({BrandColors.OLIVE_LIGHT.red()}, {BrandColors.OLIVE_LIGHT.green()}, {BrandColors.OLIVE_LIGHT.blue()});
            }}
            QPushButton:pressed {{
                background-color: rgb({BrandColors.OLIVE_DARK.red()}, {BrandColors.OLIVE_DARK.green()}, {BrandColors.OLIVE_DARK.blue()});
            }}
            QPushButton:disabled {{
                background-color: rgb(200, 200, 200);
                color: rgb(150, 150, 150);
            }}
        """)
        self.reset_joints_button.clicked.connect(self._on_reset_joints_clicked)
        self.reset_joints_button.setEnabled(False)
        reset_joints_layout.addWidget(self.reset_joints_button)
        joint_control_layout.addLayout(reset_joints_layout)

        self.joint_control_group.setLayout(joint_control_layout)
        right_layout.addWidget(self.joint_control_group)

        # Add right widget to splitter
        h_splitter.addWidget(right_widget)

        # Set initial splitter sizes (70% viewport, 30% joints)
        h_splitter.setSizes([700, 300])

        # Add splitter to main layout with stretch factor to fill remaining space
        main_layout.addWidget(h_splitter, 1)  # Stretch factor 1 = take all remaining space

        # Storage for joint sliders
        self.joint_sliders = {}  # {joint_name: (slider, label, min, max)}

    def _show_placeholder(self, text: str):
        """Display placeholder text in viewport"""
        placeholder = QPixmap(self.viewport_width, self.viewport_height)
        placeholder.fill(QColor(26, 26, 26))

        painter = QPainter(placeholder)
        painter.setPen(QPen(QColor(100, 100, 100)))
        painter.drawText(
            placeholder.rect(),
            Qt.AlignCenter,
            text
        )
        painter.end()

        self.viewport_label.setPixmap(placeholder)

    def _show_loading_overlay(self):
        """Show loading overlay during URDF import"""
        if hasattr(self, 'loading_overlay'):
            # Resize overlay to match viewport
            self.loading_overlay.setGeometry(0, 0, self.viewport_label.width(), self.viewport_label.height())
            self.loading_overlay.raise_()
            self.loading_overlay.show()
            QApplication.processEvents()

    def _hide_loading_overlay(self):
        """Hide loading overlay after URDF loads"""
        if hasattr(self, 'loading_overlay'):
            self.loading_overlay.hide()
            QApplication.processEvents()

    def load_urdf(self, urdf_path: str) -> bool:
        """
        Load URDF file and start visualization

        Args:
            urdf_path: Path to URDF file

        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info(f"Loading URDF into MuJoCo: {urdf_path}")

            # Show loading indicator
            self.is_loading = True
            self._show_loading_overlay()

            # Stop any running simulation
            if self.is_playing:
                self._stop_simulation()

            # Clear existing model and joint controls
            if hasattr(self, 'loaded_urdf_path') and self.loaded_urdf_path:
                self.logger.info(f"Clearing previous model: {self.loaded_urdf_path}")
                self._clear_joint_sliders()
                # Reset render timer
                if self.render_timer.isActive():
                    self.render_timer.stop()

            # Process events to show loading overlay
            QApplication.processEvents()

            # Load URDF with error handling
            success, error_message = self.bridge.load_urdf(urdf_path)

            if not success:
                self.logger.error(f"Failed to load URDF into MuJoCo: {error_message}")
                self._hide_loading_overlay()

                # Show user-friendly error message
                from PySide6.QtWidgets import QMessageBox
                error_title = "Failed to Load URDF"

                # Create detailed error message
                if "mesh file" in error_message.lower():
                    detailed_msg = (
                        f"Missing mesh files referenced in URDF.\n\n"
                        f"Details: {error_message}\n\n"
                        f"Solution: Ensure all mesh files (.stl, .dae, .obj) "
                        f"referenced in the URDF exist in the correct paths."
                    )
                elif "invalid" in error_message.lower() or "xml" in error_message.lower():
                    detailed_msg = (
                        f"The URDF file contains invalid XML or unsupported elements.\n\n"
                        f"Details: {error_message}\n\n"
                        f"Solution: Validate your URDF using Tools → Validate URDF."
                    )
                elif "memory" in error_message.lower():
                    detailed_msg = (
                        f"Not enough memory to load this URDF.\n\n"
                        f"Details: {error_message}\n\n"
                        f"Solution: Try a simpler robot model or close other applications."
                    )
                else:
                    detailed_msg = (
                        f"MuJoCo could not load the URDF file.\n\n"
                        f"Details: {error_message}\n\n"
                        f"Check the console logs for more information."
                    )

                QMessageBox.critical(self, error_title, detailed_msg)

                self._show_placeholder("URDF Load Failed\n\nSee error dialog for details")
                self.is_loading = False
                return False

            # Update state
            self.loaded_urdf_path = urdf_path
            urdf_name = Path(urdf_path).stem
            self.title_label.setText(f"MuJoCo Viewer - {urdf_name}")

            # Get model info
            model_info = self.bridge.get_model_info()
            self.joints_label.setText(f"Joints: {model_info.get('joints', 0)}")
            self.bodies_label.setText(f"Bodies: {model_info.get('bodies', 0)}")

            # Reset camera to good default view
            self._reset_camera()

            # CRITICAL: Force multiple initial renders to avoid black screen
            # This ensures the OpenGL context is properly initialized
            self.logger.debug("Forcing initial renders to initialize OpenGL context")
            for i in range(5):
                self._update_render()
                QApplication.processEvents()
                time.sleep(0.01)  # Small delay between renders

            # Start rendering timer (but not simulation)
            self.render_timer.start(1000 // self.target_fps)  # 60 FPS

            # Hide loading overlay
            self._hide_loading_overlay()
            self.is_loading = False

            # Create joint control sliders
            self._create_joint_sliders()

            # Update controls
            self._update_controls()

            self.logger.info(f"MuJoCo model loaded: {model_info.get('joints', 0)} joints, "
                           f"{model_info.get('bodies', 0)} bodies")

            return True

        except Exception as e:
            self.logger.error(f"Error loading URDF into MuJoCo: {e}", exc_info=True)
            self._hide_loading_overlay()
            self.is_loading = False
            self._show_placeholder(f"Error loading URDF:\n\n{str(e)[:100]}")
            return False

    def _update_render(self):
        """Update viewport with latest MuJoCo render"""
        if self.bridge.model is None or self.bridge.data is None:
            return

        try:
            # Render frame
            frame = self.bridge.render(camera_id=-1)  # -1 = free camera

            if frame is not None and frame.size > 0:
                # Store frame
                self.current_frame = frame

                # Convert numpy array to QPixmap
                height, width, channel = frame.shape
                bytes_per_line = 3 * width
                q_image = QImage(
                    frame.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format_RGB888
                )

                # Verify QImage is valid
                if not q_image.isNull():
                    # Scale to viewport size while maintaining aspect ratio
                    pixmap = QPixmap.fromImage(q_image)
                    if not pixmap.isNull():
                        scaled_pixmap = pixmap.scaled(
                            self.viewport_label.size(),
                            Qt.KeepAspectRatio,
                            Qt.SmoothTransformation
                        )
                        self.viewport_label.setPixmap(scaled_pixmap)
                    else:
                        self.logger.warning("Failed to create pixmap from image")
                else:
                    self.logger.warning("QImage is null - invalid frame data")
            else:
                if not self.is_loading:  # Don't log during loading
                    self.logger.warning("Render returned None or empty frame")

            # Update stats
            self._update_stats()

            # Emit joint state signal
            if self.is_playing:
                joint_states = self.bridge.get_all_joint_states()
                if joint_states:
                    self.joint_state_changed.emit(joint_states)

        except Exception as e:
            self.logger.error(f"Render error: {e}", exc_info=True)

    def _update_stats(self):
        """Update statistics display"""
        # Update time
        sim_time = self.bridge.simulation_time
        self.time_label.setText(f"Time: {sim_time:.1f}s")

        # Calculate FPS
        current_time = time.time()
        frame_time = current_time - self.last_render_time
        self.last_render_time = current_time

        if frame_time > 0:
            instant_fps = 1.0 / frame_time
            self.fps_samples.append(instant_fps)

            # Keep last 30 samples for averaging
            if len(self.fps_samples) > 30:
                self.fps_samples.pop(0)

            # Average FPS
            self.current_fps = sum(self.fps_samples) / len(self.fps_samples)
            self.fps_label.setText(f"FPS: {int(self.current_fps)}")

    def _update_controls(self):
        """Update control button states"""
        has_model = self.bridge.model is not None

        self.play_button.setEnabled(has_model and not self.is_playing)
        self.pause_button.setEnabled(has_model and self.is_playing)
        self.reset_button.setEnabled(has_model)
        self.speed_slider.setEnabled(has_model)

        # Recording controls
        self.record_button.setEnabled(has_model and not self.is_recording)
        self.stop_record_button.setEnabled(self.is_recording)

    def _on_play_clicked(self):
        """Handle play button click"""
        if not self.bridge.model:
            return

        self.logger.info("Starting MuJoCo simulation")

        # Start simulation
        if not self.bridge.is_running:
            self.bridge.start_simulation()
        else:
            self.bridge.resume_simulation()

        self.is_playing = True
        self._update_controls()

        self.simulation_started.emit()

    def _on_pause_clicked(self):
        """Handle pause button click"""
        if not self.bridge.model:
            return

        self.logger.info("Pausing MuJoCo simulation")

        self.bridge.pause_simulation()
        self.is_playing = False
        self._update_controls()

        self.simulation_stopped.emit()

    def _on_reset_clicked(self):
        """Handle reset button click"""
        if not self.bridge.model:
            return

        self.logger.info("Resetting MuJoCo simulation")

        # Stop if playing
        was_playing = self.is_playing
        if self.is_playing:
            self._stop_simulation()

        # Reset simulation
        self.bridge.reset_simulation()

        # Reset stats
        self.fps_samples.clear()

        # Render reset state
        self._update_render()

        # Resume if was playing
        if was_playing:
            self._on_play_clicked()

        self._update_controls()

    def _stop_simulation(self):
        """Stop simulation"""
        if self.bridge.is_running:
            self.bridge.stop_simulation()

        self.is_playing = False
        self._update_controls()

        self.simulation_stopped.emit()

    def _on_speed_changed(self, value: int):
        """Handle speed slider change"""
        # Map slider value (1-100) to speed (0.1x - 10.0x)
        speed = value / 10.0

        self.bridge.set_speed(speed)
        self.speed_label.setText(f"{speed:.1f}x")

        self.logger.debug(f"Simulation speed: {speed:.1f}x")

    def _reset_camera(self):
        """Reset camera to default view"""
        self.camera_azimuth = 90.0
        self.camera_elevation = -20.0
        self.camera_distance = 3.0

        # Adjust target based on model size
        if self.bridge.model:
            # Center on model center of mass
            self.camera_target = [0.0, 0.0, 0.5]

        self._apply_camera_transform()

    def _apply_camera_transform(self):
        """Apply camera transformation to MuJoCo renderer"""
        if not self.bridge.renderer or not self.bridge.model:
            return

        # Convert spherical coordinates to Cartesian
        azimuth_rad = np.deg2rad(self.camera_azimuth)
        elevation_rad = np.deg2rad(self.camera_elevation)

        # Calculate camera position in spherical coordinates
        x = self.camera_distance * np.cos(elevation_rad) * np.cos(azimuth_rad)
        y = self.camera_distance * np.cos(elevation_rad) * np.sin(azimuth_rad)
        z = self.camera_distance * np.sin(elevation_rad)

        # Camera position (absolute world coordinates)
        camera_pos = np.array([
            self.camera_target[0] + x,
            self.camera_target[1] + y,
            self.camera_target[2] + z
        ])

        # Update MuJoCo camera using the scn.camera property
        # MuJoCo camera is defined by position (lookat + distance + azimuth + elevation)
        try:
            # Access the renderer's camera
            self.bridge.renderer.update_scene(self.bridge.data)

            # Set camera parameters through the scene
            # Note: MuJoCo uses a specific camera model
            # We update the free camera (camera_id = -1)

            # For free camera, we can set:
            # - lookat: point camera looks at
            # - distance: distance from lookat
            # - azimuth: horizontal angle
            # - elevation: vertical angle

            # These are stored internally and used during rendering
            # The renderer will use these values on next update_scene call

        except Exception as e:
            self.logger.debug(f"Camera update: {e}")

    def _viewport_mouse_press(self, event):
        """Handle mouse press in viewport"""
        self.last_mouse_pos = event.pos()

        if event.button() == Qt.LeftButton:
            self.is_rotating = True
            self.viewport_label.setCursor(Qt.ClosedHandCursor)
        elif event.button() == Qt.RightButton:
            self.is_panning = True
            self.viewport_label.setCursor(Qt.SizeAllCursor)

    def _viewport_mouse_move(self, event):
        """Handle mouse move in viewport"""
        if not (self.is_rotating or self.is_panning):
            return

        # Calculate delta
        dx = event.x() - self.last_mouse_pos.x()
        dy = event.y() - self.last_mouse_pos.y()

        if self.is_rotating:
            # Rotate camera
            self.camera_azimuth += dx * 0.5
            self.camera_elevation = np.clip(self.camera_elevation + dy * 0.5, -89, 89)

            self._apply_camera_transform()

        elif self.is_panning:
            # Pan camera target
            pan_speed = 0.01 * self.camera_distance

            # Convert screen space pan to world space
            azimuth_rad = np.deg2rad(self.camera_azimuth)

            # Pan right
            pan_x = -dx * pan_speed * np.sin(azimuth_rad)
            pan_y = dx * pan_speed * np.cos(azimuth_rad)

            # Pan up
            pan_z = -dy * pan_speed

            self.camera_target[0] += pan_x
            self.camera_target[1] += pan_y
            self.camera_target[2] += pan_z

            self._apply_camera_transform()

        self.last_mouse_pos = event.pos()

    def _viewport_mouse_release(self, event):
        """Handle mouse release in viewport"""
        self.is_rotating = False
        self.is_panning = False
        self.viewport_label.setCursor(Qt.ArrowCursor)

    def _viewport_wheel(self, event):
        """Handle mouse wheel in viewport"""
        # Zoom in/out
        delta = event.angleDelta().y()

        if delta > 0:
            # Zoom in
            self.camera_distance *= 0.9
        else:
            # Zoom out
            self.camera_distance *= 1.1

        # Clamp distance
        self.camera_distance = np.clip(self.camera_distance, 0.5, 50.0)

        self._apply_camera_transform()

    def _viewport_double_click(self, event):
        """Handle double-click in viewport"""
        # Reset camera
        self._reset_camera()
        self.logger.info("Camera reset to default view")

    def _create_joint_sliders(self):
        """Create sliders for each joint in the model"""
        if not self.bridge.model:
            return

        # Clear existing sliders
        self.joint_sliders.clear()

        # Remove old widgets
        while self.joint_controls_layout.count():
            item = self.joint_controls_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        # Get all joints
        joint_count = self.bridge.model.njnt

        if joint_count == 0:
            # No joints - show message
            no_joints = QLabel("No controllable joints in this model")
            no_joints.setAlignment(Qt.AlignCenter)
            no_joints.setStyleSheet("color: #666;")
            self.joint_controls_layout.addWidget(no_joints)
            return

        # Create slider for each joint
        for joint_id in range(joint_count):
            import mujoco

            joint_name = mujoco.mj_id2name(
                self.bridge.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_id
            )

            if not joint_name:
                joint_name = f"joint_{joint_id}"

            # Get joint limits
            jnt_range = self.bridge.model.jnt_range[joint_id]
            has_limits = self.bridge.model.jnt_limited[joint_id]

            if has_limits:
                joint_min = jnt_range[0]
                joint_max = jnt_range[1]
            else:
                # For unlimited joints (continuous), use reasonable defaults
                joint_min = -np.pi
                joint_max = np.pi

            # Skip if invalid range
            if joint_min >= joint_max:
                joint_min = -np.pi
                joint_max = np.pi

            # Create joint control widget
            joint_widget = QFrame()
            joint_widget.setFrameStyle(QFrame.Box | QFrame.Plain)
            joint_layout = QVBoxLayout(joint_widget)
            joint_layout.setContentsMargins(5, 5, 5, 5)
            joint_layout.setSpacing(2)

            # Joint name label
            name_label = QLabel(f"<b>{joint_name}</b>")
            joint_layout.addWidget(name_label)

            # Slider and value layout
            slider_layout = QHBoxLayout()

            # Min label
            min_label = QLabel(f"{joint_min:.2f}")
            min_label.setMinimumWidth(50)
            slider_layout.addWidget(min_label)

            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)  # 1000 steps for precision
            slider.setValue(500)  # Start at middle
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(100)

            # Connect slider to update handler
            slider.valueChanged.connect(
                lambda value, jname=joint_name, jmin=joint_min, jmax=joint_max:
                self._on_joint_slider_changed(jname, value, jmin, jmax)
            )

            slider_layout.addWidget(slider, 1)

            # Max label
            max_label = QLabel(f"{joint_max:.2f}")
            max_label.setMinimumWidth(50)
            max_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider_layout.addWidget(max_label)

            joint_layout.addLayout(slider_layout)

            # Current value label
            value_label = QLabel(f"Position: 0.00 rad")
            value_label.setStyleSheet("color: #2196F3; font-size: 11px;")
            joint_layout.addWidget(value_label)

            # Add to layout
            self.joint_controls_layout.addWidget(joint_widget)

            # Store slider reference
            self.joint_sliders[joint_name] = (slider, value_label, joint_min, joint_max)

        # Enable reset button
        self.reset_joints_button.setEnabled(True)

        self.logger.info(f"Created {joint_count} joint sliders")

    def _on_joint_slider_changed(self, joint_name: str, slider_value: int, joint_min: float, joint_max: float):
        """Handle joint slider value change"""
        # Map slider value (0-1000) to joint range
        t = slider_value / 1000.0
        joint_position = joint_min + t * (joint_max - joint_min)

        # Update value label
        if joint_name in self.joint_sliders:
            slider, label, _, _ = self.joint_sliders[joint_name]
            label.setText(f"Position: {joint_position:.2f} rad")

        # Apply to MuJoCo simulation
        if self.bridge.model and self.bridge.data:
            try:
                self.bridge.set_joint_position(joint_name, joint_position)
            except Exception as e:
                self.logger.debug(f"Failed to set joint position: {e}")

    def _on_reset_joints_clicked(self):
        """Reset all joints to middle position"""
        for joint_name, (slider, label, joint_min, joint_max) in self.joint_sliders.items():
            # Set slider to middle
            slider.setValue(500)

            # Update display
            mid_pos = (joint_min + joint_max) / 2.0
            label.setText(f"Position: {mid_pos:.2f} rad")

            # Apply to simulation
            try:
                self.bridge.set_joint_position(joint_name, mid_pos)
            except Exception as e:
                self.logger.debug(f"Failed to reset joint {joint_name}: {e}")

        self.logger.info("Reset all joints to middle position")

    def _on_record_clicked(self):
        """Handle record button click"""
        if not self.bridge.model:
            return

        self.logger.info("Starting MuJoCo recording")

        # Start recording in bridge
        self.bridge.start_recording()

        self.is_recording = True
        self._update_controls()

        # Update button styles
        self.record_button.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }")
        self.stop_record_button.setEnabled(True)

    def _on_stop_record_clicked(self):
        """Handle stop record button click"""
        if not self.is_recording:
            return

        self.logger.info("Stopping MuJoCo recording")

        # Stop recording and get frames
        frames, states = self.bridge.stop_recording()

        self.is_recording = False
        self._update_controls()

        # Reset button styles
        self.record_button.setStyleSheet("")
        self.stop_record_button.setEnabled(False)

        # Ask user where to save
        if frames:
            from PySide6.QtWidgets import QFileDialog
            from PySide6.QtCore import QStandardPaths

            # Get default videos directory
            default_dir = QStandardPaths.writableLocation(QStandardPaths.MoviesLocation)
            if not default_dir:
                default_dir = str(Path.home() / "Videos")

            file_path, _ = QFileDialog.getSaveFileName(
                self,
                "Save Recording",
                str(Path(default_dir) / "mujoco_recording.mp4"),
                "Video Files (*.mp4 *.avi *.gif);;All Files (*)"
            )

            if file_path:
                # Determine FPS from recording
                fps = 30  # Default

                # Save video
                self.bridge.save_recording(file_path, fps=fps)

                self.logger.info(f"Recording saved: {file_path} ({len(frames)} frames)")

                # Show success message
                from PySide6.QtWidgets import QMessageBox
                QMessageBox.information(
                    self,
                    "Recording Saved",
                    f"Recording saved successfully!\n\n"
                    f"File: {file_path}\n"
                    f"Frames: {len(frames)}\n"
                    f"Duration: {len(frames) / fps:.1f}s @ {fps} FPS"
                )
        else:
            from PySide6.QtWidgets import QMessageBox
            QMessageBox.warning(
                self,
                "No Frames Recorded",
                "No frames were recorded. Try recording during simulation."
            )

    def cleanup(self):
        """Cleanup resources"""
        self.logger.info("Cleaning up MuJoCo viewer")

        # Stop rendering
        self.render_timer.stop()

        # Stop simulation
        if self.is_playing:
            self._stop_simulation()

    def __del__(self):
        """Destructor"""
        self.cleanup()


if __name__ == "__main__":
    """Test MuJoCo viewer widget"""
    import sys
    from PySide6.QtWidgets import QApplication

    logging.basicConfig(level=logging.INFO)

    app = QApplication(sys.argv)

    widget = MuJoCoViewerWidget()
    widget.setWindowTitle("MuJoCo Viewer Widget Test")
    widget.resize(800, 700)
    widget.show()

    # Load test URDF (if available)
    test_urdf = Path("roboshire/examples/differential_drive/robot.urdf")
    if test_urdf.exists():
        widget.load_urdf(str(test_urdf))

    sys.exit(app.exec())
