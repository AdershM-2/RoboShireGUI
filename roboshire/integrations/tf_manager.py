"""
TF Manager

Simple TF (Transform) management via SSH for visualizing coordinate frames.

Author: RoboShire Team
Phase: 6.4 (TF Visualization)
"""

from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import logging
import re


@dataclass
class FrameInfo:
    """Information about a TF frame"""
    name: str
    parent: Optional[str] = None
    is_static: bool = False


@dataclass
class Transform:
    """Transform between two frames"""
    source_frame: str
    target_frame: str
    translation: Tuple[float, float, float]  # x, y, z
    rotation: Tuple[float, float, float, float]  # x, y, z, w (quaternion)
    timestamp: Optional[float] = None


class TFManager:
    """
    Manage TF queries via SSH

    Features:
    - List active frames
    - Get frame relationships
    - Query transforms between frames
    """

    def __init__(self, ssh_manager):
        """
        Initialize TF Manager

        Args:
            ssh_manager: SSHManagerSubprocess instance
        """
        self.ssh_manager = ssh_manager
        self.logger = logging.getLogger(__name__)

    def list_frames(self) -> List[str]:
        """
        List all active TF frames

        Returns:
            List of frame names
        """
        try:
            # Use ros2 run tf2_ros tf2_monitor to list frames
            result = self.ssh_manager.execute_command(
                "timeout 2 ros2 run tf2_ros tf2_monitor --once 2>/dev/null || "
                "echo 'No frames available'"
            )

            if not result or "No frames available" in result:
                return []

            # Parse frame names from output
            frames = []
            for line in result.split('\n'):
                # Look for "Frame: <name>" pattern
                if "Frame:" in line:
                    match = re.search(r'Frame:\s+(\S+)', line)
                    if match:
                        frames.append(match.group(1))

            return frames

        except Exception as e:
            self.logger.error(f"Failed to list TF frames: {e}")
            return []

    def get_frame_tree(self) -> Dict[str, FrameInfo]:
        """
        Get TF frame tree structure

        Returns:
            Dictionary mapping frame names to FrameInfo
        """
        try:
            # Use ros2 run tf2_tools view_frames to get tree
            result = self.ssh_manager.execute_command(
                "cd /tmp && timeout 5 ros2 run tf2_tools view_frames 2>&1 | grep -E '(Frame|Parent)'"
            )

            if not result:
                return {}

            frames = {}
            current_frame = None

            for line in result.split('\n'):
                # Parse "Frame: <name>" lines
                if "Frame:" in line:
                    match = re.search(r'Frame:\s+(\S+)', line)
                    if match:
                        frame_name = match.group(1)
                        current_frame = frame_name
                        frames[frame_name] = FrameInfo(name=frame_name)

                # Parse "Parent: <name>" lines
                elif "Parent:" in line and current_frame:
                    match = re.search(r'Parent:\s+(\S+)', line)
                    if match:
                        parent_name = match.group(1)
                        if parent_name != "NO_PARENT" and parent_name != "world":
                            frames[current_frame].parent = parent_name

            return frames

        except Exception as e:
            self.logger.error(f"Failed to get TF tree: {e}")
            return {}

    def get_transform(self, source_frame: str, target_frame: str) -> Optional[Transform]:
        """
        Get transform between two frames

        Args:
            source_frame: Source frame name
            target_frame: Target frame name

        Returns:
            Transform object or None if unavailable
        """
        try:
            # Use tf2_echo to get transform
            cmd = f"timeout 2 ros2 run tf2_ros tf2_echo {source_frame} {target_frame} 2>/dev/null | head -20"
            result = self.ssh_manager.execute_command(cmd)

            if not result or "Exception" in result:
                return None

            # Parse transform from output
            translation = self._parse_translation(result)
            rotation = self._parse_rotation(result)

            if translation and rotation:
                return Transform(
                    source_frame=source_frame,
                    target_frame=target_frame,
                    translation=translation,
                    rotation=rotation
                )

            return None

        except Exception as e:
            self.logger.error(f"Failed to get transform {source_frame} -> {target_frame}: {e}")
            return None

    def _parse_translation(self, output: str) -> Optional[Tuple[float, float, float]]:
        """Parse translation from tf2_echo output"""
        try:
            # Look for "Translation: [x, y, z]" or similar
            for line in output.split('\n'):
                if "Translation" in line or "translation" in line:
                    # Extract numbers
                    numbers = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', line)
                    if len(numbers) >= 3:
                        return (float(numbers[0]), float(numbers[1]), float(numbers[2]))

                # Also look for "- Translation:" multi-line format
                if "- Translation:" in line:
                    continue
                if line.strip().startswith("x:"):
                    x = float(line.split(':')[1].strip())
                    continue
                if line.strip().startswith("y:"):
                    y = float(line.split(':')[1].strip())
                    continue
                if line.strip().startswith("z:"):
                    z = float(line.split(':')[1].strip())
                    return (x, y, z)

            return None

        except Exception as e:
            self.logger.error(f"Failed to parse translation: {e}")
            return None

    def _parse_rotation(self, output: str) -> Optional[Tuple[float, float, float, float]]:
        """Parse rotation quaternion from tf2_echo output"""
        try:
            # Look for "Rotation: in Quaternion [x, y, z, w]"
            for line in output.split('\n'):
                if "Quaternion" in line or "quaternion" in line:
                    numbers = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', line)
                    if len(numbers) >= 4:
                        return (float(numbers[0]), float(numbers[1]), float(numbers[2]), float(numbers[3]))

                # Multi-line format
                if "- Rotation:" in line or "rotation:" in line:
                    continue
                if line.strip().startswith("x:") and "quat" in output.lower():
                    x = float(line.split(':')[1].strip())
                    continue
                if line.strip().startswith("y:") and "quat" in output.lower():
                    y = float(line.split(':')[1].strip())
                    continue
                if line.strip().startswith("z:") and "quat" in output.lower():
                    z = float(line.split(':')[1].strip())
                    continue
                if line.strip().startswith("w:"):
                    w = float(line.split(':')[1].strip())
                    return (x, y, z, w)

            return None

        except Exception as e:
            self.logger.error(f"Failed to parse rotation: {e}")
            return None

    def check_transform_available(self, source_frame: str, target_frame: str) -> bool:
        """
        Check if transform is available without fetching full data

        Args:
            source_frame: Source frame
            target_frame: Target frame

        Returns:
            True if transform exists
        """
        try:
            cmd = f"timeout 1 ros2 run tf2_ros tf2_echo {source_frame} {target_frame} 2>&1 | head -1"
            result = self.ssh_manager.execute_command(cmd)

            return result and "Exception" not in result and "not found" not in result.lower()

        except Exception:
            return False
