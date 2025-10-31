"""
Parameter Manager - Manage ROS2 node parameters

Features:
- List node parameters
- Get parameter values
- Set parameter values
- Save/load parameter configurations
- Parameter validation
"""

import logging
import yaml
import re
from typing import List, Optional, Dict, Any, Union
from dataclasses import dataclass
from pathlib import Path

from roboshire.backend.ssh_subprocess import SSHManagerSubprocess


@dataclass
class Parameter:
    """ROS2 parameter"""
    node_name: str
    param_name: str
    value: Any
    param_type: str = "unknown"  # bool, int, float, string, array

    def __str__(self):
        return f"{self.node_name}::{self.param_name} = {self.value} ({self.param_type})"


class ParameterManager:
    """
    Manage ROS2 node parameters

    Features:
    - List parameters
    - Get/set parameters
    - Save/load configurations
    - Parameter validation
    """

    def __init__(self, ssh_manager: SSHManagerSubprocess, ros_distro: str = "humble"):
        """
        Initialize parameter manager

        Args:
            ssh_manager: SSH manager for remote execution
            ros_distro: ROS2 distribution name
        """
        self.ssh = ssh_manager
        self.ros_distro = ros_distro

        logging.info("ParameterManager initialized")

    def list_nodes(self) -> List[str]:
        """
        List all nodes (needed to list parameters)

        Returns:
            List of node names
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 node list"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0 and result.stdout:
                nodes = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
                logging.info(f"Found {len(nodes)} nodes")
                return nodes
            else:
                logging.warning(f"Failed to list nodes: {result.stderr}")
                return []

        except Exception as e:
            logging.error(f"Error listing nodes: {e}")
            return []

    def list_params(self, node_name: str) -> List[Parameter]:
        """
        List all parameters for a node

        Args:
            node_name: Node name (e.g., "/velocity_publisher")

        Returns:
            List of Parameter objects
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 param list {node_name}"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0 or not result.stdout:
                logging.warning(f"Failed to list params for {node_name}: {result.stderr}")
                return []

            # Parse parameter names
            param_names = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]

            # Get values for each parameter
            parameters = []
            for param_name in param_names:
                param = self.get_param(node_name, param_name)
                if param:
                    parameters.append(param)

            logging.info(f"Found {len(parameters)} parameters for {node_name}")
            return parameters

        except Exception as e:
            logging.error(f"Error listing params for {node_name}: {e}")
            return []

    def get_param(self, node_name: str, param_name: str) -> Optional[Parameter]:
        """
        Get a parameter value

        Args:
            node_name: Node name
            param_name: Parameter name

        Returns:
            Parameter object or None if error
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 param get {node_name} {param_name}"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0 or not result.stdout:
                logging.warning(f"Failed to get param {node_name}::{param_name}: {result.stderr}")
                return None

            # Parse output - format is typically "Type: value"
            output = result.stdout.strip()

            # Extract type and value
            type_match = re.match(r'(\w+)\s+value\s+is:\s*(.+)', output, re.IGNORECASE)
            if type_match:
                param_type = type_match.group(1).lower()
                value_str = type_match.group(2).strip()

                # Convert value to appropriate Python type
                value = self._parse_value(value_str, param_type)

                param = Parameter(
                    node_name=node_name,
                    param_name=param_name,
                    value=value,
                    param_type=param_type
                )

                logging.debug(f"Got param: {param}")
                return param
            else:
                logging.warning(f"Could not parse param output: {output}")
                return None

        except Exception as e:
            logging.error(f"Error getting param {node_name}::{param_name}: {e}")
            return None

    def set_param(
        self,
        node_name: str,
        param_name: str,
        value: Any
    ) -> bool:
        """
        Set a parameter value

        Args:
            node_name: Node name
            param_name: Parameter name
            value: New value

        Returns:
            True if successful, False otherwise
        """
        try:
            # Format value for command
            value_str = self._format_value(value)

            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && "
            cmd += f"ros2 param set {node_name} {param_name} {value_str}"

            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0:
                logging.info(f"Set param {node_name}::{param_name} = {value}")
                return True
            else:
                logging.error(f"Failed to set param {node_name}::{param_name}: {result.stderr}")
                return False

        except Exception as e:
            logging.error(f"Error setting param {node_name}::{param_name}: {e}")
            return False

    def dump_params(self, node_name: str, output_path: str) -> bool:
        """
        Dump all parameters for a node to a YAML file

        Args:
            node_name: Node name
            output_path: Output file path on remote system

        Returns:
            True if successful, False otherwise
        """
        try:
            # Ensure output directory exists
            output_dir = str(Path(output_path).parent)
            self.ssh.execute_command(f"mkdir -p {output_dir}")

            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && "
            cmd += f"ros2 param dump {node_name} --output-dir {output_dir}"

            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0:
                logging.info(f"Dumped params for {node_name} to {output_path}")
                return True
            else:
                logging.error(f"Failed to dump params for {node_name}: {result.stderr}")
                return False

        except Exception as e:
            logging.error(f"Error dumping params for {node_name}: {e}")
            return False

    def load_params(self, node_name: str, param_file: str) -> bool:
        """
        Load parameters from a YAML file

        Args:
            node_name: Node name
            param_file: Parameter file path on remote system

        Returns:
            True if successful, False otherwise
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && "
            cmd += f"ros2 param load {node_name} {param_file}"

            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0:
                logging.info(f"Loaded params for {node_name} from {param_file}")
                return True
            else:
                logging.error(f"Failed to load params for {node_name}: {result.stderr}")
                return False

        except Exception as e:
            logging.error(f"Error loading params for {node_name}: {e}")
            return False

    def save_config(self, config_data: Dict[str, Dict[str, Any]], file_path: str) -> bool:
        """
        Save parameter configuration to a local YAML file

        Args:
            config_data: Dictionary of node_name -> {param_name: value}
            file_path: Local file path to save

        Returns:
            True if successful, False otherwise
        """
        try:
            # Format as ROS2 param file format
            ros_params = {}

            for node_name, params in config_data.items():
                # Remove leading slash for YAML format
                node_key = node_name.lstrip('/')

                ros_params[node_key] = {
                    'ros__parameters': params
                }

            # Write YAML
            with open(file_path, 'w') as f:
                yaml.dump(ros_params, f, default_flow_style=False)

            logging.info(f"Saved parameter config to {file_path}")
            return True

        except Exception as e:
            logging.error(f"Error saving config to {file_path}: {e}")
            return False

    def load_config(self, file_path: str) -> Optional[Dict[str, Dict[str, Any]]]:
        """
        Load parameter configuration from a local YAML file

        Args:
            file_path: Local file path to load

        Returns:
            Dictionary of node_name -> {param_name: value}, or None if error
        """
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)

            if not data:
                return None

            # Convert from ROS2 param format to simple dict
            config_data = {}

            for node_key, node_data in data.items():
                node_name = f"/{node_key}" if not node_key.startswith('/') else node_key

                if 'ros__parameters' in node_data:
                    config_data[node_name] = node_data['ros__parameters']
                else:
                    config_data[node_name] = node_data

            logging.info(f"Loaded parameter config from {file_path}")
            return config_data

        except Exception as e:
            logging.error(f"Error loading config from {file_path}: {e}")
            return None

    def _parse_value(self, value_str: str, param_type: str) -> Any:
        """
        Parse a parameter value string to Python type

        Args:
            value_str: Value string
            param_type: Parameter type

        Returns:
            Parsed value
        """
        value_str = value_str.strip()

        if param_type == 'boolean':
            return value_str.lower() in ('true', '1', 'yes')
        elif param_type == 'integer':
            try:
                return int(value_str)
            except ValueError:
                return 0
        elif param_type in ('double', 'float'):
            try:
                return float(value_str)
            except ValueError:
                return 0.0
        elif param_type == 'string':
            # Remove quotes if present
            if value_str.startswith('"') and value_str.endswith('"'):
                return value_str[1:-1]
            if value_str.startswith("'") and value_str.endswith("'"):
                return value_str[1:-1]
            return value_str
        elif 'array' in param_type:
            # Try to parse as list
            try:
                # Handle [1, 2, 3] format
                if value_str.startswith('[') and value_str.endswith(']'):
                    return eval(value_str)  # Safe here since we control the input
                else:
                    return [value_str]
            except:
                return [value_str]
        else:
            return value_str

    def _format_value(self, value: Any) -> str:
        """
        Format a Python value for ROS2 param set command

        Args:
            value: Python value

        Returns:
            Formatted string
        """
        if isinstance(value, bool):
            return 'true' if value else 'false'
        elif isinstance(value, str):
            # Quote strings
            return f'"{value}"'
        elif isinstance(value, (list, tuple)):
            # Format as [1, 2, 3]
            return str(list(value))
        else:
            return str(value)

    def get_all_params(self) -> Dict[str, List[Parameter]]:
        """
        Get all parameters for all nodes

        Returns:
            Dictionary of node_name -> List[Parameter]
        """
        all_params = {}

        nodes = self.list_nodes()
        for node_name in nodes:
            params = self.list_params(node_name)
            if params:
                all_params[node_name] = params

        logging.info(f"Got parameters for {len(all_params)} nodes")
        return all_params
