"""
Service Manager - Manage ROS2 services and actions

Features:
- List services and actions
- Get service/action types
- Call services
- Send action goals
- Parse responses
"""

import logging
import re
import json
from typing import List, Optional, Dict, Any
from dataclasses import dataclass

from roboshire.backend.ssh_subprocess import SSHManagerSubprocess


@dataclass
class ServiceInfo:
    """Information about a ROS2 service"""
    name: str
    service_type: str = "unknown"
    nodes: List[str] = None

    def __post_init__(self):
        if self.nodes is None:
            self.nodes = []


@dataclass
class ActionInfo:
    """Information about a ROS2 action"""
    name: str
    action_type: str = "unknown"
    action_servers: List[str] = None
    action_clients: List[str] = None

    def __post_init__(self):
        if self.action_servers is None:
            self.action_servers = []
        if self.action_clients is None:
            self.action_clients = []


@dataclass
class ServiceCallResult:
    """Result of a service call"""
    success: bool
    response: Optional[Dict[str, Any]] = None
    error_message: str = ""


class ServiceManager:
    """
    Manage ROS2 services and actions

    Features:
    - List services
    - Get service info
    - Call services
    - List actions
    - Send action goals
    """

    def __init__(self, ssh_manager: SSHManagerSubprocess, ros_distro: str = "humble"):
        """
        Initialize service manager

        Args:
            ssh_manager: SSH manager for remote execution
            ros_distro: ROS2 distribution name
        """
        self.ssh = ssh_manager
        self.ros_distro = ros_distro

        logging.info("ServiceManager initialized")

    def list_services(self) -> List[str]:
        """
        List all available services

        Returns:
            List of service names
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 service list"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0 and result.stdout:
                services = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
                logging.info(f"Found {len(services)} services")
                return services
            else:
                logging.warning(f"Failed to list services: {result.stderr}")
                return []

        except Exception as e:
            logging.error(f"Error listing services: {e}")
            return []

    def get_service_type(self, service_name: str) -> Optional[str]:
        """
        Get the type of a service

        Args:
            service_name: Service name

        Returns:
            Service type or None if error
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 service type {service_name}"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0 and result.stdout:
                service_type = result.stdout.strip()
                logging.info(f"Service {service_name} type: {service_type}")
                return service_type
            else:
                logging.warning(f"Failed to get service type for {service_name}: {result.stderr}")
                return None

        except Exception as e:
            logging.error(f"Error getting service type for {service_name}: {e}")
            return None

    def get_service_info(self, service_name: str) -> Optional[ServiceInfo]:
        """
        Get detailed information about a service

        Args:
            service_name: Service name

        Returns:
            ServiceInfo object or None if error
        """
        info = ServiceInfo(name=service_name)

        # Get service type
        service_type = self.get_service_type(service_name)
        if service_type:
            info.service_type = service_type

        return info

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request_data: str
    ) -> ServiceCallResult:
        """
        Call a ROS2 service

        Args:
            service_name: Service name (e.g., "/my_service")
            service_type: Service type (e.g., "std_srvs/srv/SetBool")
            request_data: Request data as YAML/JSON string (e.g., "{data: true}")

        Returns:
            ServiceCallResult object
        """
        try:
            # Build command
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && "
            cmd += f"ros2 service call {service_name} {service_type} '{request_data}'"

            result = self.ssh.execute_command(cmd, timeout=30.0)

            if result.exit_code == 0:
                # Parse response
                response_text = result.stdout.strip()

                # Try to extract response fields
                response_dict = {}

                # Simple parsing - look for "field: value" patterns
                for line in response_text.split('\n'):
                    match = re.match(r'\s*(\w+):\s*(.+)', line)
                    if match:
                        field = match.group(1)
                        value = match.group(2).strip()

                        # Try to convert to appropriate type
                        if value.lower() == 'true':
                            value = True
                        elif value.lower() == 'false':
                            value = False
                        elif value.isdigit():
                            value = int(value)
                        elif re.match(r'^-?\d+\.\d+$', value):
                            value = float(value)

                        response_dict[field] = value

                logging.info(f"Service call successful: {service_name}")
                return ServiceCallResult(
                    success=True,
                    response=response_dict if response_dict else {"raw": response_text}
                )
            else:
                error_msg = result.stderr or "Unknown error"
                logging.error(f"Service call failed for {service_name}: {error_msg}")
                return ServiceCallResult(
                    success=False,
                    error_message=error_msg
                )

        except Exception as e:
            error_msg = str(e)
            logging.error(f"Error calling service {service_name}: {e}")
            return ServiceCallResult(
                success=False,
                error_message=error_msg
            )

    def list_actions(self) -> List[str]:
        """
        List all available actions

        Returns:
            List of action names
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 action list"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0 and result.stdout:
                actions = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
                logging.info(f"Found {len(actions)} actions")
                return actions
            else:
                logging.warning(f"Failed to list actions: {result.stderr}")
                return []

        except Exception as e:
            logging.error(f"Error listing actions: {e}")
            return []

    def get_action_type(self, action_name: str) -> Optional[str]:
        """
        Get the type of an action

        Args:
            action_name: Action name

        Returns:
            Action type or None if error
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 action type {action_name}"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0 and result.stdout:
                action_type = result.stdout.strip()
                logging.info(f"Action {action_name} type: {action_type}")
                return action_type
            else:
                logging.warning(f"Failed to get action type for {action_name}: {result.stderr}")
                return None

        except Exception as e:
            logging.error(f"Error getting action type for {action_name}: {e}")
            return None

    def get_action_info(self, action_name: str) -> Optional[ActionInfo]:
        """
        Get detailed information about an action

        Args:
            action_name: Action name

        Returns:
            ActionInfo object or None if error
        """
        try:
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 action info {action_name}"
            result = self.ssh.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0:
                logging.warning(f"Failed to get action info for {action_name}: {result.stderr}")
                return None

            info = ActionInfo(name=action_name)

            # Get action type
            action_type = self.get_action_type(action_name)
            if action_type:
                info.action_type = action_type

            # Parse info output for servers/clients
            output = result.stdout

            # Extract action servers
            servers_match = re.search(r'Action servers:\s*(.*?)(?=Action clients|$)', output, re.DOTALL)
            if servers_match:
                servers_text = servers_match.group(1).strip()
                info.action_servers = [line.strip() for line in servers_text.split('\n') if line.strip()]

            # Extract action clients
            clients_match = re.search(r'Action clients:\s*(.*?)$', output, re.DOTALL)
            if clients_match:
                clients_text = clients_match.group(1).strip()
                info.action_clients = [line.strip() for line in clients_text.split('\n') if line.strip()]

            logging.info(f"Got action info for {action_name}")
            return info

        except Exception as e:
            logging.error(f"Error getting action info for {action_name}: {e}")
            return None

    def send_action_goal(
        self,
        action_name: str,
        action_type: str,
        goal_data: str
    ) -> ServiceCallResult:
        """
        Send a goal to an action server

        Args:
            action_name: Action name
            action_type: Action type
            goal_data: Goal data as YAML/JSON string

        Returns:
            ServiceCallResult with action result
        """
        try:
            # Build command
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && "
            cmd += f"ros2 action send_goal {action_name} {action_type} '{goal_data}'"

            result = self.ssh.execute_command(cmd, timeout=60.0)  # Actions can take longer

            if result.exit_code == 0:
                response_text = result.stdout.strip()
                logging.info(f"Action goal sent successfully: {action_name}")
                return ServiceCallResult(
                    success=True,
                    response={"raw": response_text}
                )
            else:
                error_msg = result.stderr or "Unknown error"
                logging.error(f"Action goal failed for {action_name}: {error_msg}")
                return ServiceCallResult(
                    success=False,
                    error_message=error_msg
                )

        except Exception as e:
            error_msg = str(e)
            logging.error(f"Error sending action goal to {action_name}: {e}")
            return ServiceCallResult(
                success=False,
                error_message=error_msg
            )
