"""
MuJoCo Bridge for RoboShire

Integrates MuJoCo physics simulation with ROS2 control interface.
Provides real-time URDF visualization and physics simulation.

Author: RoboShire Team
Phase: 8 (Documentation & MuJoCo Integration)
"""

import mujoco
import numpy as np
from pathlib import Path
from typing import Optional, Dict, Tuple, List, Callable
import logging
import time
from threading import Thread, Lock
import imageio
import os
import subprocess

from roboshire.utils.urdf_to_mjcf import convert_urdf_to_mjcf


class MuJocoBridge:
    """
    Bridge between URDF/ROS2 and MuJoCo simulation

    Features:
    - Load URDF files (auto-converted to MJCF)
    - Physics simulation with accurate dynamics
    - Joint state control (position, velocity, torque)
    - Sensor data (IMU, force/torque, contacts)
    - Recording (trajectories, videos)
    - Real-time rendering
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

        # MuJoCo components
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.renderer: Optional[mujoco.Renderer] = None

        # Simulation state
        self.is_running = False
        self.is_paused = False
        self.simulation_speed = 1.0  # Real-time multiplier
        self.simulation_time = 0.0

        # Threading
        self.sim_thread: Optional[Thread] = None
        self.lock = Lock()

        # Callbacks
        self.step_callback: Optional[Callable] = None
        self.render_callback: Optional[Callable] = None

        # Recording
        self.is_recording = False
        self.recorded_frames: List[np.ndarray] = []
        self.recorded_states: List[Dict] = []

        # Joint control
        self.joint_targets: Dict[str, float] = {}  # Joint name -> target value
        self.control_mode = "position"  # position, velocity, torque

    def _ensure_display(self) -> bool:
        """
        Ensure X11 DISPLAY environment variable is set for OpenGL/EGL rendering

        Handles both X11 and Xwayland environments.

        Returns:
            True if DISPLAY is set or was successfully configured, False otherwise
        """
        # Check if DISPLAY is already set and valid
        if 'DISPLAY' in os.environ and os.environ['DISPLAY']:
            self.logger.info(f"DISPLAY already set: {os.environ['DISPLAY']}")
            # Still configure EGL for X11/Xwayland compatibility
            self._configure_egl_for_xwayland()
            return True

        self.logger.warning("DISPLAY not set - attempting to detect X11/Xwayland display")

        # Try to detect available X11/Xwayland displays
        try:
            # Check for running X servers (including Xwayland)
            result = subprocess.run(
                ['ps', 'aux'],
                capture_output=True,
                text=True,
                timeout=5
            )

            # Look for X server or Xwayland processes
            detected_displays = []
            for line in result.stdout.splitlines():
                if 'Xwayland' in line or 'Xorg' in line or '/X ' in line:
                    # Try to extract display number
                    parts = line.split()
                    for part in parts:
                        if part.startswith(':') and len(part) > 1:
                            # Extract just the display number (e.g., ':0' from ':0')
                            display = part.split()[0].rstrip(',')
                            if display not in detected_displays:
                                detected_displays.append(display)
                                self.logger.info(f"Detected display: {display}")

            # Try detected displays first
            for display in detected_displays:
                try:
                    # Test if display is accessible using xdpyinfo
                    test_result = subprocess.run(
                        ['xdpyinfo', '-display', display],
                        capture_output=True,
                        stderr=subprocess.DEVNULL,
                        timeout=2
                    )
                    if test_result.returncode == 0:
                        os.environ['DISPLAY'] = display
                        self.logger.info(f"Set DISPLAY to working display: {display}")
                        # Configure EGL for X11/Xwayland
                        self._configure_egl_for_xwayland()
                        return True
                except Exception as e:
                    self.logger.debug(f"Display {display} not accessible: {e}")
                    continue

            # Try common display numbers as fallback
            for display_num in [':0', ':1', ':10']:
                try:
                    test_result = subprocess.run(
                        ['xdpyinfo', '-display', display_num],
                        capture_output=True,
                        stderr=subprocess.DEVNULL,
                        timeout=2
                    )
                    if test_result.returncode == 0:
                        os.environ['DISPLAY'] = display_num
                        self.logger.info(f"Set DISPLAY to working display: {display_num}")
                        # Configure EGL for X11/Xwayland
                        self._configure_egl_for_xwayland()
                        return True
                except Exception as e:
                    self.logger.debug(f"Display {display_num} not accessible: {e}")
                    continue

            # Last resort: Set to :0 (most common, especially for Xwayland)
            os.environ['DISPLAY'] = ':0'
            self.logger.warning("Set DISPLAY to :0 as fallback (common for Xwayland)")
            self.logger.warning("If rendering fails, ensure X11/Xwayland is running and accessible")

            # Set EGL platform to X11 for Xwayland compatibility
            self._configure_egl_for_xwayland()
            return True

        except Exception as e:
            self.logger.error(f"Failed to detect X11/Xwayland display: {e}")
            # Set to :0 as last resort
            os.environ['DISPLAY'] = ':0'
            self.logger.warning("Set DISPLAY to :0 as emergency fallback")

            # Set EGL platform to X11 for Xwayland compatibility
            self._configure_egl_for_xwayland()
            return True  # Return True to allow MuJoCo to try

    def _configure_egl_for_xwayland(self):
        """Configure EGL environment variables for Xwayland compatibility"""
        # Force EGL to use X11 platform instead of Wayland
        os.environ['EGL_PLATFORM'] = 'x11'
        os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/50_mesa.json'

        # Force GLFW to use X11 (not Wayland)
        os.environ['SDL_VIDEODRIVER'] = 'x11'

        self.logger.info("Configured EGL for X11/Xwayland compatibility")

    def load_urdf(self, urdf_path: str) -> Tuple[bool, str]:
        """
        Load URDF file into MuJoCo with comprehensive error handling

        Args:
            urdf_path: Path to URDF file

        Returns:
            Tuple of (success, error_message). error_message is empty string if successful
        """
        try:
            urdf_path = Path(urdf_path)

            # Validate file exists
            if not urdf_path.exists():
                error_msg = f"URDF file not found: {urdf_path}"
                self.logger.error(error_msg)
                return (False, error_msg)

            # Validate file is readable
            if not urdf_path.is_file():
                error_msg = f"Path is not a file: {urdf_path}"
                self.logger.error(error_msg)
                return (False, error_msg)

            self.logger.info(f"Loading URDF: {urdf_path}")

            # Convert URDF to MJCF with validation
            try:
                mjcf_path = convert_urdf_to_mjcf(str(urdf_path))
                self.logger.info(f"Converted to MJCF: {mjcf_path}")
            except FileNotFoundError as e:
                error_msg = f"URDF conversion failed - missing mesh file: {str(e)}"
                self.logger.error(error_msg)
                return (False, error_msg)
            except ValueError as e:
                error_msg = f"URDF conversion failed - invalid URDF format: {str(e)}"
                self.logger.error(error_msg)
                return (False, error_msg)
            except Exception as e:
                error_msg = f"URDF conversion failed: {str(e)}"
                self.logger.error(error_msg, exc_info=True)
                return (False, error_msg)

            # Validate MJCF file was created
            if not Path(mjcf_path).exists():
                error_msg = "MJCF file was not created during conversion"
                self.logger.error(error_msg)
                return (False, error_msg)

            # Load MJCF into MuJoCo with error handling
            try:
                self.model = mujoco.MjModel.from_xml_path(mjcf_path)
                self.data = mujoco.MjData(self.model)
            except mujoco.FatalError as e:
                error_msg = f"MuJoCo failed to load model - invalid XML: {str(e)}"
                self.logger.error(error_msg)
                return (False, error_msg)
            except Exception as e:
                error_msg = f"MuJoCo failed to load model: {str(e)}"
                self.logger.error(error_msg, exc_info=True)
                return (False, error_msg)

            # Validate model is valid
            if self.model.nbody == 0:
                error_msg = "Invalid model: No bodies found in URDF"
                self.logger.error(error_msg)
                self.model = None
                self.data = None
                return (False, error_msg)

            # Create renderer with error handling
            try:
                # Ensure DISPLAY environment variable is set
                if not self._ensure_display():
                    error_msg = "Failed to set up X11 display for rendering"
                    self.logger.error(error_msg)
                    self.model = None
                    self.data = None
                    return (False, error_msg)

                self.renderer = mujoco.Renderer(self.model, height=480, width=640)
            except Exception as e:
                error_msg = f"Failed to create MuJoCo renderer: {str(e)}"
                self.logger.error(error_msg, exc_info=True)
                self.model = None
                self.data = None
                return (False, error_msg)

            # Initialize simulation time
            self.simulation_time = 0.0

            # Log model info
            self.logger.info(f"Model loaded successfully: {self.model.nbody} bodies, "
                           f"{self.model.njnt} joints, {self.model.ngeom} geoms")

            return (True, "")

        except MemoryError:
            error_msg = "Out of memory while loading URDF. File may be too large or complex."
            self.logger.error(error_msg)
            return (False, error_msg)

        except Exception as e:
            error_msg = f"Unexpected error loading URDF: {str(e)}"
            self.logger.error(error_msg, exc_info=True)
            return (False, error_msg)

    def start_simulation(self):
        """Start physics simulation in separate thread"""
        if self.model is None:
            self.logger.error("No model loaded")
            return

        if self.is_running:
            self.logger.warning("Simulation already running")
            return

        self.logger.info("Starting simulation")
        self.is_running = True
        self.is_paused = False

        # Start simulation thread
        self.sim_thread = Thread(target=self._simulation_loop, daemon=True)
        self.sim_thread.start()

    def stop_simulation(self):
        """Stop physics simulation"""
        if not self.is_running:
            return

        self.logger.info("Stopping simulation")
        self.is_running = False

        if self.sim_thread is not None:
            self.sim_thread.join(timeout=1.0)
            self.sim_thread = None

    def pause_simulation(self):
        """Pause physics simulation"""
        self.is_paused = True
        self.logger.info("Simulation paused")

    def resume_simulation(self):
        """Resume physics simulation"""
        self.is_paused = False
        self.logger.info("Simulation resumed")

    def reset_simulation(self):
        """Reset simulation to initial state"""
        if self.model is None or self.data is None:
            return

        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self.simulation_time = 0.0
            self.joint_targets.clear()

        self.logger.info("Simulation reset")

    def set_speed(self, speed: float):
        """
        Set simulation speed multiplier

        Args:
            speed: Speed multiplier (0.1 to 10.0, 1.0 = real-time)
        """
        self.simulation_speed = np.clip(speed, 0.1, 10.0)
        self.logger.info(f"Simulation speed: {self.simulation_speed}x")

    def _simulation_loop(self):
        """Main simulation loop (runs in separate thread)"""
        dt = self.model.opt.timestep  # Simulation timestep
        last_time = time.time()

        while self.is_running:
            if self.is_paused:
                time.sleep(0.01)
                last_time = time.time()
                continue

            # Calculate time to next step
            target_dt = dt / self.simulation_speed
            current_time = time.time()
            elapsed = current_time - last_time

            if elapsed < target_dt:
                # Sleep until next step
                time.sleep(target_dt - elapsed)
                continue

            last_time = current_time

            # Apply control
            with self.lock:
                self._apply_control()

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                self.simulation_time += dt

            # Call step callback
            if self.step_callback is not None:
                try:
                    self.step_callback()
                except Exception as e:
                    self.logger.error(f"Step callback error: {e}")

            # Record if active
            if self.is_recording:
                self._record_frame()

    def _apply_control(self):
        """Apply joint control targets to actuators"""
        if self.control_mode == "position":
            # Position control via actuators
            for joint_name, target_pos in self.joint_targets.items():
                actuator_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    f"{joint_name}_motor"
                )

                if actuator_id >= 0:
                    self.data.ctrl[actuator_id] = target_pos

        elif self.control_mode == "velocity":
            # Velocity control
            for joint_name, target_vel in self.joint_targets.items():
                joint_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    joint_name
                )

                if joint_id >= 0:
                    # Apply velocity directly (simplified)
                    qvel_addr = self.model.jnt_dofadr[joint_id]
                    self.data.qvel[qvel_addr] = target_vel

        elif self.control_mode == "torque":
            # Torque control
            for joint_name, target_torque in self.joint_targets.items():
                joint_id = mujoco.mj_name2id(
                    self.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    joint_name
                )

                if joint_id >= 0:
                    qvel_addr = self.model.jnt_dofadr[joint_id]
                    self.data.qfrc_applied[qvel_addr] = target_torque

    def set_joint_position(self, joint_name: str, position: float):
        """Set target position for a joint"""
        with self.lock:
            self.joint_targets[joint_name] = position

    def set_joint_positions(self, positions: Dict[str, float]):
        """Set target positions for multiple joints"""
        with self.lock:
            self.joint_targets.update(positions)

    def get_joint_state(self, joint_name: str) -> Optional[Tuple[float, float]]:
        """
        Get current joint state (position, velocity)

        Returns:
            (position, velocity) or None if joint not found
        """
        if self.model is None or self.data is None:
            return None

        try:
            joint_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                joint_name
            )

            if joint_id < 0:
                return None

            qpos_addr = self.model.jnt_qposadr[joint_id]
            qvel_addr = self.model.jnt_dofadr[joint_id]

            with self.lock:
                position = self.data.qpos[qpos_addr]
                velocity = self.data.qvel[qvel_addr]

            return (float(position), float(velocity))

        except Exception as e:
            self.logger.error(f"Failed to get joint state: {e}")
            return None

    def get_all_joint_states(self) -> Dict[str, Tuple[float, float]]:
        """Get states for all joints"""
        states = {}

        if self.model is None:
            return states

        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                i
            )

            if joint_name:
                state = self.get_joint_state(joint_name)
                if state is not None:
                    states[joint_name] = state

        return states

    def render(self, camera_id: int = -1) -> Optional[np.ndarray]:
        """
        Render current scene

        Args:
            camera_id: Camera ID (-1 for free camera)

        Returns:
            RGB image as numpy array (H, W, 3) or None if failed
        """
        if self.renderer is None or self.data is None:
            return None

        try:
            with self.lock:
                # Update renderer
                self.renderer.update_scene(self.data, camera=camera_id)

                # Render to pixels
                pixels = self.renderer.render()

            # Call render callback
            if self.render_callback is not None:
                try:
                    self.render_callback(pixels)
                except Exception as e:
                    self.logger.error(f"Render callback error: {e}")

            return pixels

        except Exception as e:
            self.logger.error(f"Render failed: {e}")
            return None

    def start_recording(self):
        """Start recording simulation (frames + states)"""
        self.is_recording = True
        self.recorded_frames = []
        self.recorded_states = []
        self.logger.info("Recording started")

    def stop_recording(self) -> Tuple[List[np.ndarray], List[Dict]]:
        """
        Stop recording and return captured data

        Returns:
            (frames, states) - Lists of frames and joint states
        """
        self.is_recording = False
        frames = self.recorded_frames.copy()
        states = self.recorded_states.copy()
        self.recorded_frames.clear()
        self.recorded_states.clear()
        self.logger.info(f"Recording stopped: {len(frames)} frames")
        return frames, states

    def _record_frame(self):
        """Record current frame and state"""
        # Render frame
        frame = self.render()
        if frame is not None:
            self.recorded_frames.append(frame)

        # Record joint states
        states = self.get_all_joint_states()
        self.recorded_states.append({
            "time": self.simulation_time,
            "joints": states
        })

    def save_recording(self, video_path: str, fps: int = 30):
        """
        Save recorded video to file

        Args:
            video_path: Output video file path
            fps: Frames per second
        """
        if not self.recorded_frames:
            self.logger.warning("No frames recorded")
            return

        try:
            self.logger.info(f"Saving video: {video_path}")
            imageio.mimsave(video_path, self.recorded_frames, fps=fps)
            self.logger.info(f"Video saved: {len(self.recorded_frames)} frames @ {fps} fps")

        except Exception as e:
            self.logger.error(f"Failed to save video: {e}")

    def get_model_info(self) -> Dict:
        """Get information about loaded model"""
        if self.model is None:
            return {}

        return {
            "name": self.model.names.decode() if self.model.names else "unknown",
            "bodies": self.model.nbody,
            "joints": self.model.njnt,
            "geoms": self.model.ngeom,
            "actuators": self.model.nu,
            "sensors": self.model.nsensor,
            "timestep": self.model.opt.timestep,
            "gravity": self.model.opt.gravity.tolist()
        }

    def get_contact_forces(self) -> List[Dict]:
        """Get contact forces from simulation"""
        if self.data is None:
            return []

        contacts = []

        with self.lock:
            for i in range(self.data.ncon):
                contact = self.data.contact[i]

                contacts.append({
                    "geom1": contact.geom1,
                    "geom2": contact.geom2,
                    "pos": contact.pos.tolist(),
                    "force": contact.force.tolist(),
                    "dist": contact.dist
                })

        return contacts

    def apply_external_force(self, body_name: str, force: np.ndarray, pos: Optional[np.ndarray] = None):
        """
        Apply external force to a body

        Args:
            body_name: Name of body
            force: Force vector [fx, fy, fz]
            pos: Application point (body frame), default = center of mass
        """
        if self.model is None or self.data is None:
            return

        try:
            body_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_name
            )

            if body_id >= 0:
                with self.lock:
                    if pos is None:
                        # Apply at center of mass
                        self.data.xfrc_applied[body_id, :3] = force
                    else:
                        # Apply at specific point (requires computing torque)
                        self.data.xfrc_applied[body_id, :3] = force
                        # Compute torque: tau = pos x force
                        torque = np.cross(pos, force)
                        self.data.xfrc_applied[body_id, 3:] = torque

        except Exception as e:
            self.logger.error(f"Failed to apply force: {e}")

    def __del__(self):
        """Cleanup on deletion"""
        self.stop_simulation()


if __name__ == "__main__":
    # Test MuJoCo bridge
    logging.basicConfig(level=logging.INFO)

    bridge = MuJocoBridge()

    # Test with simple URDF (create one for testing)
    test_urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.2 0.2 0.1"/>
          </geometry>
          <material name="blue">
            <color rgba="0 0 1 1"/>
          </material>
        </visual>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
        </inertial>
      </link>
    </robot>
    """

    # Save test URDF
    test_path = Path("test_robot.urdf")
    test_path.write_text(test_urdf)

    # Load and test
    if bridge.load_urdf(str(test_path)):
        print("✓ URDF loaded successfully")
        print(f"Model info: {bridge.get_model_info()}")

        # Start simulation
        bridge.start_simulation()
        time.sleep(2)  # Simulate for 2 seconds

        # Render a frame
        frame = bridge.render()
        if frame is not None:
            print(f"✓ Rendered frame: {frame.shape}")

        # Stop simulation
        bridge.stop_simulation()
        print("✓ Simulation stopped")

    # Cleanup
    test_path.unlink()
    print("✓ Test complete")
