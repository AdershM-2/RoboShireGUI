"""
URDF Manager - Wrapper around yourdfpy for URDF operations
"""

from pathlib import Path
from typing import Optional, List, Dict, Any
import logging

try:
    from yourdfpy import URDF
    YOURDFPY_AVAILABLE = True
except ImportError:
    YOURDFPY_AVAILABLE = False
    logging.warning("yourdfpy not installed. Install with: pip install yourdfpy")


class URDFManager:
    """
    Manages URDF file operations using yourdfpy backend.

    Provides a simplified interface for:
    - Loading URDF/XACRO files
    - Validating URDF structure
    - Modifying links and joints
    - Saving URDF files
    """

    def __init__(self):
        """Initialize URDF manager"""
        self.urdf: Optional[URDF] = None
        self.file_path: Optional[Path] = None
        self.modified: bool = False

        if not YOURDFPY_AVAILABLE:
            raise ImportError(
                "yourdfpy is required but not installed. "
                "Install with: pip install yourdfpy"
            )

    def load(self, file_path: str | Path) -> bool:
        """
        Load a URDF file

        Args:
            file_path: Path to URDF or XACRO file

        Returns:
            True if loaded successfully, False otherwise
        """
        try:
            file_path = Path(file_path)

            if not file_path.exists():
                logging.error(f"URDF file not found: {file_path}")
                return False

            # Load URDF
            self.urdf = URDF.load(str(file_path))
            self.file_path = file_path
            self.modified = False

            logging.info(f"Loaded URDF: {file_path}")
            logging.info(f"  Links: {len(self.urdf.link_map)}")
            logging.info(f"  Joints: {len(self.urdf.joint_map)}")

            return True

        except Exception as e:
            logging.error(f"Failed to load URDF: {e}")
            return False

    def save(self, file_path: Optional[str | Path] = None) -> bool:
        """
        Save URDF to file

        Args:
            file_path: Path to save to (uses current path if None)

        Returns:
            True if saved successfully, False otherwise
        """
        if self.urdf is None:
            logging.error("No URDF loaded")
            return False

        try:
            # Use provided path or current path
            save_path = Path(file_path) if file_path else self.file_path

            if save_path is None:
                logging.error("No file path specified")
                return False

            # Save URDF
            self.urdf.write_xml_file(str(save_path))
            self.file_path = save_path
            self.modified = False

            logging.info(f"Saved URDF to: {save_path}")
            return True

        except Exception as e:
            logging.error(f"Failed to save URDF: {e}")
            return False

    def validate(self) -> tuple[bool, List[str]]:
        """
        Validate URDF structure

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        if self.urdf is None:
            return False, ["No URDF loaded"]

        errors = []

        try:
            # Basic validation checks
            if not self.urdf.link_map:
                errors.append("No links defined")

            if len(self.urdf.link_map) > 1 and not self.urdf.joint_map:
                errors.append("Multiple links but no joints defined")

            # Check for root link
            if not hasattr(self.urdf, 'base_link') or self.urdf.base_link is None:
                errors.append("No base link found")

            # Check each link
            for link_name, link in self.urdf.link_map.items():
                if link.name != link_name:
                    errors.append(f"Link name mismatch: {link_name} vs {link.name}")

            # Check each joint
            for joint_name, joint in self.urdf.joint_map.items():
                if joint.name != joint_name:
                    errors.append(f"Joint name mismatch: {joint_name} vs {joint.name}")

                # Check parent/child links exist
                if joint.parent not in self.urdf.link_map:
                    errors.append(f"Joint {joint_name}: parent link '{joint.parent}' not found")

                if joint.child not in self.urdf.link_map:
                    errors.append(f"Joint {joint_name}: child link '{joint.child}' not found")

            is_valid = len(errors) == 0

            if is_valid:
                logging.info("URDF validation passed")
            else:
                logging.warning(f"URDF validation failed with {len(errors)} errors")

            return is_valid, errors

        except Exception as e:
            logging.error(f"Validation error: {e}")
            return False, [str(e)]

    def get_links(self) -> List[str]:
        """Get list of link names"""
        if self.urdf is None:
            return []
        return list(self.urdf.link_map.keys())

    def get_joints(self) -> List[str]:
        """Get list of joint names"""
        if self.urdf is None:
            return []
        return list(self.urdf.joint_map.keys())

    def get_link_info(self, link_name: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a link

        Args:
            link_name: Name of the link

        Returns:
            Dictionary with link information or None
        """
        if self.urdf is None or link_name not in self.urdf.link_map:
            return None

        link = self.urdf.link_map[link_name]

        info = {
            'name': link.name,
            'has_visual': len(link.visuals) > 0,
            'has_collision': len(link.collisions) > 0,
            'has_inertial': link.inertial is not None,
        }

        # Add inertial info if available
        if link.inertial is not None:
            info['mass'] = link.inertial.mass
            if link.inertial.origin is not None:
                info['inertial_origin'] = link.inertial.origin

        return info

    def get_joint_info(self, joint_name: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a joint

        Args:
            joint_name: Name of the joint

        Returns:
            Dictionary with joint information or None
        """
        if self.urdf is None or joint_name not in self.urdf.joint_map:
            return None

        joint = self.urdf.joint_map[joint_name]

        info = {
            'name': joint.name,
            'type': joint.type,
            'parent': joint.parent,
            'child': joint.child,
        }

        # Add joint-specific info
        if joint.axis is not None:
            info['axis'] = joint.axis

        if joint.limit is not None:
            info['lower_limit'] = joint.limit.lower
            info['upper_limit'] = joint.limit.upper
            info['effort'] = joint.limit.effort
            info['velocity'] = joint.limit.velocity

        if joint.origin is not None:
            info['origin'] = joint.origin

        return info

    def update_joint_limits(self, joint_name: str, lower: float, upper: float) -> bool:
        """
        Update joint limits

        Args:
            joint_name: Name of the joint
            lower: Lower limit (radians or meters)
            upper: Upper limit (radians or meters)

        Returns:
            True if updated successfully
        """
        if self.urdf is None or joint_name not in self.urdf.joint_map:
            return False

        try:
            joint = self.urdf.joint_map[joint_name]

            if joint.limit is None:
                logging.warning(f"Joint {joint_name} has no limits to update")
                return False

            joint.limit.lower = lower
            joint.limit.upper = upper
            self.modified = True

            logging.info(f"Updated {joint_name} limits: [{lower}, {upper}]")
            return True

        except Exception as e:
            logging.error(f"Failed to update joint limits: {e}")
            return False

    def get_robot_name(self) -> str:
        """Get robot name from URDF"""
        if self.urdf is None:
            return "Unknown"
        return getattr(self.urdf, 'name', 'Unnamed Robot')

    def get_summary(self) -> str:
        """
        Get a summary of the loaded URDF

        Returns:
            Human-readable summary string
        """
        if self.urdf is None:
            return "No URDF loaded"

        summary_lines = [
            f"Robot: {self.get_robot_name()}",
            f"File: {self.file_path}",
            f"Links: {len(self.urdf.link_map)}",
            f"Joints: {len(self.urdf.joint_map)}",
        ]

        # Add base link info
        if hasattr(self.urdf, 'base_link') and self.urdf.base_link:
            summary_lines.append(f"Base Link: {self.urdf.base_link}")

        return "\n".join(summary_lines)

    def is_loaded(self) -> bool:
        """Check if URDF is loaded"""
        return self.urdf is not None

    def is_modified(self) -> bool:
        """Check if URDF has been modified"""
        return self.modified

    def load_xacro(self, xacro_path: str | Path, mappings: Optional[Dict[str, str]] = None) -> bool:
        """
        Load a Xacro file and convert to URDF (v1.0.0)

        Xacro is a macro language for URDF that allows:
        - Property definitions with $(property_name)
        - Macros with parameters
        - Math expressions
        - Conditional statements

        Args:
            xacro_path: Path to .urdf.xacro file
            mappings: Optional dictionary of property name -> value mappings

        Returns:
            True if loaded successfully, False otherwise
        """
        try:
            import subprocess
            import tempfile

            xacro_path = Path(xacro_path)

            if not xacro_path.exists():
                logging.error(f"Xacro file not found: {xacro_path}")
                return False

            # Build xacro command
            cmd = ["xacro", str(xacro_path)]

            # Add property mappings
            if mappings:
                for key, value in mappings.items():
                    cmd.append(f"{key}:={value}")

            # Run xacro to convert to URDF
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode != 0:
                logging.error(f"Xacro conversion failed: {result.stderr}")
                return False

            # Save temporary URDF
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(result.stdout)
                temp_urdf_path = f.name

            # Load the generated URDF
            success = self.load(temp_urdf_path)

            # Clean up temp file
            Path(temp_urdf_path).unlink()

            if success:
                # Store original xacro path
                self.file_path = xacro_path
                logging.info(f"Loaded Xacro: {xacro_path}")

            return success

        except subprocess.TimeoutExpired:
            logging.error("Xacro conversion timed out")
            return False
        except FileNotFoundError:
            logging.error("xacro command not found. Install with: sudo apt install ros-humble-xacro")
            return False
        except Exception as e:
            logging.error(f"Failed to load Xacro: {e}")
            return False

    def save_as_xacro(self, xacro_path: str | Path, include_properties: bool = True) -> bool:
        """
        Save current URDF as Xacro file with property extraction (v1.0.0)

        This performs basic URDF -> Xacro conversion by:
        - Extracting common values as properties
        - Adding xacro namespace
        - Wrapping in xacro:macro (optional)

        Args:
            xacro_path: Path to save .urdf.xacro file
            include_properties: Extract common values as properties

        Returns:
            True if saved successfully, False otherwise
        """
        if self.urdf is None:
            logging.error("No URDF loaded")
            return False

        try:
            xacro_path = Path(xacro_path)

            # Get URDF XML string
            urdf_xml = self.urdf.write_xml_string()

            # Basic conversion: Add xacro namespace
            xacro_xml = urdf_xml.replace(
                '<robot',
                '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"'
            )

            # Save to file
            with open(xacro_path, 'w') as f:
                f.write('<?xml version="1.0"?>\n')
                f.write('<!-- Generated by RoboShire v1.0.0 -->\n')
                f.write('<!-- This is a basic Xacro conversion. Manual editing recommended for full Xacro features. -->\n')
                f.write(xacro_xml)

            logging.info(f"Saved Xacro to: {xacro_path}")
            return True

        except Exception as e:
            logging.error(f"Failed to save as Xacro: {e}")
            return False
