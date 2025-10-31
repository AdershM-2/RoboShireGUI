"""
Settings Manager - Handle application settings and configuration

Updated for v2.3.0 Ubuntu Standalone - supports execution mode configuration
"""

import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional
from dataclasses import dataclass, asdict


@dataclass
class LocalExecutionSettings:
    """Local execution settings (Ubuntu native)"""
    workspace_path: str = "~/roboshire_workspace"
    ros_distro: str = "humble"


@dataclass
class RemoteExecutionSettings:
    """Remote execution settings (SSH to Jetson/RPi/Ubuntu)"""
    device_type: str = "ubuntu"  # ubuntu, jetson_nano, jetson_xavier, jetson_orin, rpi4, rpi5
    workspace_path: str = "/home/ubuntu/roboshire_workspace"
    ros_distro: str = "humble"
    # SSH settings embedded here
    ssh_host: str = "192.168.145.128"
    ssh_user: str = "adm20"
    ssh_port: int = 22
    ssh_timeout: float = 30.0
    ssh_keepalive: int = 60


@dataclass
class ExecutionSettings:
    """Execution mode settings"""
    mode: str = "local"  # "local" or "remote"
    local: LocalExecutionSettings = None
    remote: RemoteExecutionSettings = None

    def __post_init__(self):
        if self.local is None:
            self.local = LocalExecutionSettings()
        if self.remote is None:
            self.remote = RemoteExecutionSettings()


@dataclass
class SSHSettings:
    """SSH connection settings (legacy - kept for backward compatibility)"""
    host: str = "192.168.145.128"
    user: str = "adm20"
    port: int = 22
    timeout: float = 30.0
    keepalive_interval: int = 60
    # Note: Password/key should NOT be stored in settings for security
    # They should be prompted at runtime or use system SSH agent


@dataclass
class WorkspaceSettings:
    """ROS2 workspace settings"""
    path: str = "/mnt/hgfs/ROS2_PROJECT/workspace"
    ros_distro: str = "humble"
    ros_setup_script: str = "/opt/ros/humble/setup.bash"


@dataclass
class BuildSettings:
    """Build configuration"""
    parallel_workers: int = 4
    symlink_install: bool = False  # VMware shared folders don't support symlinks
    continue_on_error: bool = False
    cmake_args: list = None

    def __post_init__(self):
        if self.cmake_args is None:
            self.cmake_args = []


@dataclass
class LaunchSettings:
    """Launch configuration"""
    default_launch_file: str = "bringup.launch.py"
    auto_source_workspace: bool = True
    log_level: str = "INFO"


class SettingsManager:
    """
    Manages application settings with file persistence.

    Settings are stored in JSON format in the project directory.
    """

    def __init__(self, settings_file: Optional[Path] = None):
        """
        Initialize settings manager

        Args:
            settings_file: Path to settings file (default: roboshire_settings.json)
        """
        if settings_file is None:
            # Store in project root
            self.settings_file = Path("roboshire_settings.json")
        else:
            self.settings_file = Path(settings_file)

        # Default settings
        self.execution = ExecutionSettings()  # NEW: Execution mode settings
        self.ssh = SSHSettings()  # Legacy
        self.workspace = WorkspaceSettings()  # Legacy
        self.build = BuildSettings()
        self.launch = LaunchSettings()

        # Load settings if file exists
        self.load()

    def load(self) -> bool:
        """
        Load settings from file

        Returns:
            True if loaded successfully, False otherwise
        """
        if not self.settings_file.exists():
            logging.info(f"Settings file not found: {self.settings_file}, using defaults")
            return False

        try:
            with open(self.settings_file, 'r') as f:
                data = json.load(f)

            # Load execution settings (NEW)
            if 'execution' in data:
                exec_data = data['execution']
                local_settings = LocalExecutionSettings(**exec_data.get('local', {}))
                remote_settings = RemoteExecutionSettings(**exec_data.get('remote', {}))
                self.execution = ExecutionSettings(
                    mode=exec_data.get('mode', 'local'),
                    local=local_settings,
                    remote=remote_settings
                )

            # Load SSH settings (legacy)
            if 'ssh' in data:
                self.ssh = SSHSettings(**data['ssh'])

            # Load workspace settings (legacy)
            if 'workspace' in data:
                self.workspace = WorkspaceSettings(**data['workspace'])

            # Load build settings
            if 'build' in data:
                self.build = BuildSettings(**data['build'])

            # Load launch settings
            if 'launch' in data:
                self.launch = LaunchSettings(**data['launch'])

            logging.info(f"Settings loaded from {self.settings_file}")
            return True

        except Exception as e:
            logging.error(f"Failed to load settings: {e}")
            return False

    def save(self) -> bool:
        """
        Save settings to file

        Returns:
            True if saved successfully, False otherwise
        """
        try:
            data = {
                'execution': {
                    'mode': self.execution.mode,
                    'local': asdict(self.execution.local),
                    'remote': asdict(self.execution.remote)
                },
                'ssh': asdict(self.ssh),  # Legacy
                'workspace': asdict(self.workspace),  # Legacy
                'build': asdict(self.build),
                'launch': asdict(self.launch),
            }

            with open(self.settings_file, 'w') as f:
                json.dump(data, f, indent=2)

            logging.info(f"Settings saved to {self.settings_file}")
            return True

        except Exception as e:
            logging.error(f"Failed to save settings: {e}")
            return False

    def get(self, section: str, key: str, default: Any = None) -> Any:
        """
        Get a specific setting value

        Args:
            section: Settings section (ssh, workspace, build, launch)
            key: Setting key
            default: Default value if not found

        Returns:
            Setting value or default
        """
        try:
            section_obj = getattr(self, section)
            return getattr(section_obj, key, default)
        except AttributeError:
            return default

    def set(self, section: str, key: str, value: Any):
        """
        Set a specific setting value

        Args:
            section: Settings section (ssh, workspace, build, launch)
            key: Setting key
            value: New value
        """
        try:
            section_obj = getattr(self, section)
            setattr(section_obj, key, value)
        except AttributeError as e:
            logging.error(f"Failed to set setting {section}.{key}: {e}")

    def reset_to_defaults(self):
        """Reset all settings to default values"""
        self.execution = ExecutionSettings()
        self.ssh = SSHSettings()
        self.workspace = WorkspaceSettings()
        self.build = BuildSettings()
        self.launch = LaunchSettings()
        logging.info("Settings reset to defaults")

    # NEW: Execution configuration helpers
    def has_execution_config(self) -> bool:
        """Check if execution mode has been configured"""
        return self.settings_file.exists() and hasattr(self, 'execution')

    def get_execution_mode(self) -> str:
        """Get current execution mode ('local' or 'remote')"""
        return self.execution.mode

    def set_execution_mode(self, mode: str):
        """Set execution mode ('local' or 'remote')"""
        if mode in ['local', 'remote']:
            self.execution.mode = mode
            logging.info(f"Execution mode set to: {mode}")
        else:
            logging.error(f"Invalid execution mode: {mode}")

    def get_local_config(self) -> LocalExecutionSettings:
        """Get local execution configuration"""
        return self.execution.local

    def get_remote_config(self) -> RemoteExecutionSettings:
        """Get remote execution configuration"""
        return self.execution.remote

    def save_local_config(self, workspace_path: str, ros_distro: str):
        """Save local execution configuration"""
        self.execution.local.workspace_path = workspace_path
        self.execution.local.ros_distro = ros_distro
        self.save()

    def save_remote_config(
        self,
        device_type: str,
        workspace_path: str,
        ros_distro: str,
        ssh_host: str,
        ssh_user: str,
        ssh_port: int = 22
    ):
        """Save remote execution configuration"""
        self.execution.remote.device_type = device_type
        self.execution.remote.workspace_path = workspace_path
        self.execution.remote.ros_distro = ros_distro
        self.execution.remote.ssh_host = ssh_host
        self.execution.remote.ssh_user = ssh_user
        self.execution.remote.ssh_port = ssh_port
        self.save()

    def __str__(self) -> str:
        """String representation of settings"""
        exec_mode = self.execution.mode.upper()
        if self.execution.mode == 'local':
            exec_detail = f"Workspace: {self.execution.local.workspace_path}"
        else:
            exec_detail = f"SSH: {self.execution.remote.ssh_user}@{self.execution.remote.ssh_host}"

        return f"""Settings:
  Execution Mode: {exec_mode} ({exec_detail})
  ROS Distro: {self.execution.local.ros_distro if self.execution.mode == 'local' else self.execution.remote.ros_distro}
  Build Workers: {self.build.parallel_workers}
  Launch Log Level: {self.launch.log_level}
"""
