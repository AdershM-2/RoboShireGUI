"""
RViz2 Bridge - Integration for launching and managing RViz2 subprocess
"""

import subprocess
import logging
import tempfile
import yaml
from pathlib import Path
from typing import Optional


class RVizBridge:
    """
    Manages RViz2 subprocess for robot visualization.

    Since RViz2 is Qt5-based and conflicts with Qt6/PySide6,
    we launch it as a separate process via SSH to the Ubuntu VM.
    """

    def __init__(self, ssh_host: str = "adm20@192.168.222.129"):
        """
        Initialize RViz2 bridge

        Args:
            ssh_host: SSH host for Ubuntu VM (e.g., "adm20@192.168.222.129" or "adm20@ubuntu")
        """
        self.ssh_host = ssh_host
        self.process: Optional[subprocess.Popen] = None
        self.is_running = False

    def launch(
        self,
        urdf_path: str | Path,
        rviz_config: Optional[str | Path] = None,
        use_robot_state_publisher: bool = True
    ) -> bool:
        """
        Launch RViz2 to visualize URDF

        Args:
            urdf_path: Path to URDF file (in shared folder)
            rviz_config: Optional RViz config file path
            use_robot_state_publisher: Whether to launch robot_state_publisher

        Returns:
            True if launched successfully, False otherwise
        """
        if self.is_running:
            logging.warning("RViz2 is already running")
            return False

        try:
            urdf_path = Path(urdf_path)

            # Convert Windows path to Ubuntu VM path
            vm_urdf_path = self._convert_to_vm_path(urdf_path)

            # Build command
            if use_robot_state_publisher:
                # Launch with robot_state_publisher
                cmd = self._build_robot_state_publisher_command(vm_urdf_path, rviz_config)
            else:
                # Launch RViz2 only
                cmd = self._build_rviz_only_command(rviz_config)

            logging.info(f"Launching RViz2 with command: {' '.join(cmd)}")

            # Launch process
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            self.is_running = True
            logging.info("RViz2 launched successfully")
            return True

        except Exception as e:
            logging.error(f"Failed to launch RViz2: {e}")
            return False

    def _convert_to_vm_path(self, windows_path: Path) -> str:
        """
        Convert Windows path to Ubuntu VM shared folder path

        Args:
            windows_path: Windows path (e.g., D:\ROS2_PROJECT\...)

        Returns:
            VM path (e.g., /mnt/hgfs/ROS2_PROJECT/...)
        """
        # Get path relative to project root (D:\ROS2_PROJECT)
        project_root = Path("D:/ROS2_PROJECT")

        try:
            rel_path = windows_path.relative_to(project_root)
            vm_path = f"/mnt/hgfs/ROS2_PROJECT/{rel_path.as_posix()}"
            return vm_path
        except ValueError:
            # Path is not relative to project root, use as-is
            logging.warning(f"Path {windows_path} is not in project root, using as-is")
            return str(windows_path)

    def _build_robot_state_publisher_command(
        self,
        urdf_path: str,
        rviz_config: Optional[str | Path] = None
    ) -> list:
        """
        Build command to launch RViz2 with robot_state_publisher

        Args:
            urdf_path: Path to URDF file on VM
            rviz_config: Optional RViz config file

        Returns:
            Command list for subprocess
        """
        # Create a launch command that:
        # 1. Sets DISPLAY for X11
        # 2. Sources ROS2
        # 3. Launches robot_state_publisher with nohup
        # 4. Launches RViz2 with nohup

        rviz_cmd = f'ros2 run rviz2 rviz2 -d {self._convert_to_vm_path(Path(rviz_config))}' if rviz_config else 'ros2 run rviz2 rviz2'

        cmd_str = (
            f"export DISPLAY=${{DISPLAY:-:0}} && "
            f"source /opt/ros/humble/setup.bash && "
            f"export ROBOT_DESCRIPTION=$(cat {urdf_path}) && "
            f"nohup ros2 run robot_state_publisher robot_state_publisher "
            f"--ros-args -p robot_description:=\"$ROBOT_DESCRIPTION\" "
            f"> ~/roboshire_robot_state_publisher.log 2>&1 & "
            f"sleep 2 && "
            f"nohup {rviz_cmd} > ~/roboshire_rviz.log 2>&1 &"
        )

        cmd = ['ssh', self.ssh_host, cmd_str]

        return cmd

    def _build_rviz_only_command(self, rviz_config: Optional[str | Path] = None) -> list:
        """
        Build command to launch RViz2 only (without robot_state_publisher)

        Args:
            rviz_config: Optional RViz config file

        Returns:
            Command list for subprocess
        """
        rviz_cmd = f'ros2 run rviz2 rviz2 -d {self._convert_to_vm_path(Path(rviz_config))}' if rviz_config else 'ros2 run rviz2 rviz2'

        cmd_str = (
            f"export DISPLAY=${{DISPLAY:-:0}} && "
            f"source /opt/ros/humble/setup.bash && "
            f"nohup {rviz_cmd} > ~/roboshire_rviz.log 2>&1 &"
        )

        cmd = ['ssh', self.ssh_host, cmd_str]

        return cmd

    def generate_rviz_config(
        self,
        output_path: str | Path,
        robot_description_topic: str = "/robot_description",
        fixed_frame: str = "base_link"
    ) -> bool:
        """
        Generate a basic RViz config file for robot visualization

        Args:
            output_path: Where to save the config file
            robot_description_topic: Topic for robot description
            fixed_frame: Fixed frame for visualization

        Returns:
            True if generated successfully
        """
        try:
            # Basic RViz config with RobotModel display
            config = {
                'Panels': [
                    {
                        'Class': 'rviz_common/Displays',
                        'Name': 'Displays'
                    },
                    {
                        'Class': 'rviz_common/Views',
                        'Name': 'Views'
                    }
                ],
                'Visualization Manager': {
                    'Class': '',
                    'Displays': [
                        {
                            'Class': 'rviz_default_plugins/Grid',
                            'Name': 'Grid',
                            'Enabled': True
                        },
                        {
                            'Class': 'rviz_default_plugins/RobotModel',
                            'Name': 'RobotModel',
                            'Enabled': True,
                            'Description Topic': {
                                'Value': robot_description_topic
                            }
                        },
                        {
                            'Class': 'rviz_default_plugins/TF',
                            'Name': 'TF',
                            'Enabled': True
                        }
                    ],
                    'Global Options': {
                        'Fixed Frame': fixed_frame
                    }
                }
            }

            # Save to file
            output_path = Path(output_path)
            output_path.parent.mkdir(parents=True, exist_ok=True)

            with open(output_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)

            logging.info(f"Generated RViz config: {output_path}")
            return True

        except Exception as e:
            logging.error(f"Failed to generate RViz config: {e}")
            return False

    def close(self):
        """Close RViz2 process"""
        if self.process is not None:
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
                logging.info("RViz2 process terminated")
            except subprocess.TimeoutExpired:
                self.process.kill()
                logging.warning("RViz2 process killed (didn't terminate gracefully)")
            except Exception as e:
                logging.error(f"Error closing RViz2: {e}")
            finally:
                self.process = None
                self.is_running = False

    def is_alive(self) -> bool:
        """Check if RViz2 process is still running"""
        if self.process is None:
            return False

        poll_result = self.process.poll()
        if poll_result is not None:
            # Process has terminated
            self.is_running = False
            return False

        return True

    def get_output(self) -> tuple[str, str]:
        """
        Get stdout and stderr from RViz2 process

        Returns:
            Tuple of (stdout, stderr)
        """
        if self.process is None:
            return "", ""

        try:
            stdout, stderr = self.process.communicate(timeout=0.1)
            return stdout, stderr
        except subprocess.TimeoutExpired:
            return "", ""
        except Exception as e:
            logging.error(f"Error getting RViz2 output: {e}")
            return "", ""

    def save_config(self, config_path: str | Path, project_name: str = "default") -> bool:
        """
        Save RViz configuration (v1.0.0)

        Creates a default RViz config for the project that can be reused.

        Args:
            config_path: Path to save .rviz config file
            project_name: Name of the project (for display name)

        Returns:
            True if saved successfully, False otherwise
        """
        try:
            config_path = Path(config_path)
            config_path.parent.mkdir(parents=True, exist_ok=True)

            # Create default RViz config
            default_config = {
                'Panels': [
                    {
                        'Class': 'rviz_common/Displays',
                        'Name': 'Displays'
                    },
                    {
                        'Class': 'rviz_common/Views',
                        'Name': 'Views'
                    }
                ],
                'Visualization Manager': {
                    'Displays': [
                        {
                            'Class': 'rviz_default_plugins/Grid',
                            'Name': 'Grid',
                            'Enabled': True
                        },
                        {
                            'Class': 'rviz_default_plugins/RobotModel',
                            'Name': 'RobotModel',
                            'Enabled': True,
                            'Description Topic': {
                                'Value': '/robot_description'
                            }
                        },
                        {
                            'Class': 'rviz_default_plugins/TF',
                            'Name': 'TF',
                            'Enabled': True,
                            'Frame Timeout': 15.0
                        }
                    ],
                    'Global Options': {
                        'Background Color': '48; 48; 48',
                        'Fixed Frame': 'base_link'
                    },
                    'Views': {
                        'Current': {
                            'Class': 'rviz_default_plugins/Orbit',
                            'Distance': 2.0,
                            'Focal Point': {
                                'X': 0.0,
                                'Y': 0.0,
                                'Z': 0.0
                            }
                        }
                    }
                }
            }

            # Save config
            with open(config_path, 'w') as f:
                yaml.dump(default_config, f, default_flow_style=False)

            logging.info(f"Saved RViz config to: {config_path}")
            return True

        except Exception as e:
            logging.error(f"Failed to save RViz config: {e}")
            return False

    def load_config(self, config_path: str | Path) -> Optional[dict]:
        """
        Load RViz configuration (v1.0.0)

        Args:
            config_path: Path to .rviz config file

        Returns:
            Config dictionary if loaded successfully, None otherwise
        """
        try:
            config_path = Path(config_path)

            if not config_path.exists():
                logging.warning(f"RViz config not found: {config_path}")
                return None

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            logging.info(f"Loaded RViz config from: {config_path}")
            return config

        except Exception as e:
            logging.error(f"Failed to load RViz config: {e}")
            return None

    def create_default_config_for_project(self, project_dir: str | Path) -> Path:
        """
        Create a default RViz config for a project (v1.0.0)

        Args:
            project_dir: Project directory

        Returns:
            Path to created config file
        """
        project_dir = Path(project_dir)
        config_dir = project_dir / "rviz"
        config_dir.mkdir(parents=True, exist_ok=True)

        config_path = config_dir / "default.rviz"

        project_name = project_dir.stem
        self.save_config(config_path, project_name)

        return config_path

    def __del__(self):
        """Cleanup on deletion"""
        self.close()
