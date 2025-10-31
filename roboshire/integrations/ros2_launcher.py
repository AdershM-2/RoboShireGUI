"""
ROS2 Launcher - Launch and manage ROS2 nodes

Handles launching ROS2 nodes and launch files, tracking running processes,
and providing lifecycle management.

Updated for v2.3.0 Ubuntu Standalone - uses ExecutionManager for local/remote execution
"""

import logging
import re
import time
import subprocess
import threading
from typing import List, Optional, Callable, Dict
from dataclasses import dataclass
from enum import Enum

from roboshire.backend.execution_manager import ExecutionManager


class NodeStatus(Enum):
    """Node status enum"""
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    FAILED = "failed"


@dataclass
class LaunchResult:
    """Result of launching a node or launch file"""
    success: bool
    pid: Optional[int] = None
    node_name: str = ""
    error_message: str = ""
    process_handle: Optional[subprocess.Popen] = None


@dataclass
class NodeInfo:
    """Information about a running node"""
    name: str
    namespace: str = "/"
    pid: Optional[int] = None
    status: NodeStatus = NodeStatus.RUNNING


class ROS2Launcher:
    """
    Launch and manage ROS2 nodes.

    Uses ExecutionManager to execute ros2 run/launch commands (local or remote).
    Tracks running processes and provides lifecycle management.

    v2.3.0: Updated to support both local Ubuntu execution and remote SSH execution.
    """

    def __init__(
        self,
        workspace_path: str,
        ros_distro: str = "humble"
    ):
        """
        Initialize ROS2 launcher

        Args:
            workspace_path: Path to workspace (local or remote depending on execution mode)
            ros_distro: ROS2 distribution name
        """
        self.execution_manager = ExecutionManager.get_instance()
        self.workspace_path = workspace_path
        self.ros_distro = ros_distro
        self.running_processes: Dict[int, LaunchResult] = {}  # pid -> LaunchResult
        self.active_streams: List[threading.Thread] = []  # Active output streaming threads

        logging.info(f"ROS2Launcher initialized for workspace: {workspace_path}")

    def source_workspace(self) -> str:
        """
        Get command to source ROS2 and workspace

        Returns:
            Command string to source environment
        """
        return (
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"source {self.workspace_path}/install/setup.bash"
        )

    def launch_node(
        self,
        package_name: str,
        executable: str,
        node_name: Optional[str] = None,
        namespace: Optional[str] = None,
        parameters: Optional[dict] = None,
        remappings: Optional[dict] = None,
        output_callback: Optional[Callable[[str, str], None]] = None
    ) -> LaunchResult:
        """
        Launch a ROS2 node in the background

        Args:
            package_name: ROS2 package name
            executable: Node executable name
            node_name: Optional node name (default: executable name)
            namespace: Optional namespace
            parameters: Optional parameters dict
            remappings: Optional topic remappings dict
            output_callback: Callback for node output (line, stream)

        Returns:
            LaunchResult with PID if successful
        """
        try:
            # Build ros2 run command
            cmd_parts = ["ros2", "run", package_name, executable]

            # Add node name remapping
            if node_name:
                cmd_parts.append(f"__node:={node_name}")

            # Add namespace remapping
            if namespace:
                cmd_parts.append(f"__ns:={namespace}")

            # Add parameters
            if parameters:
                for key, value in parameters.items():
                    cmd_parts.append(f"-p {key}:={value}")

            # Add remappings
            if remappings:
                for old_name, new_name in remappings.items():
                    cmd_parts.append(f"{old_name}:={new_name}")

            ros2_cmd = " ".join(cmd_parts)

            # Build full command with sourcing
            full_cmd = f"{self.source_workspace()} && {ros2_cmd}"

            logging.info(f"Launching node: {package_name}/{executable}")
            logging.debug(f"Command: {full_cmd}")

            # Get executor
            executor = self.execution_manager.get_executor()
            logging.info(f"Launching with {executor.get_display_name()}")

            # Launch node in background using executor
            async_process = executor.execute_command_async(
                full_cmd,
                working_dir=self.workspace_path
            )

            # Get PID from async process
            pid = async_process.pid
            if pid:
                logging.info(f"Node launched with PID: {pid}")
            else:
                logging.warning("Could not get PID from async process")

            # Build the FULL node name as ROS2 will report it in 'ros2 node list'
            # This ensures the name matches what the node monitor will see
            full_node_name = node_name or executable

            # Add namespace if provided
            if namespace:
                # Remove leading slash from namespace if present
                clean_namespace = namespace.strip('/')
                if clean_namespace:
                    full_node_name = f"/{clean_namespace}/{full_node_name}"
                else:
                    full_node_name = f"/{full_node_name}"
            else:
                # No namespace - just add leading slash
                if not full_node_name.startswith('/'):
                    full_node_name = f"/{full_node_name}"

            logging.debug(f"Node full name for monitoring: {full_node_name}")

            # Create launch result
            result = LaunchResult(
                success=True,
                pid=pid,
                node_name=full_node_name,  # Return FULL name with slash for accurate monitoring
                process_handle=async_process  # Store AsyncProcess instead of Popen
            )

            # Store process
            if pid:
                self.running_processes[pid] = result

            # Start output streaming in background thread if callback provided
            if output_callback:
                stream_thread = threading.Thread(
                    target=self._stream_async_output,
                    args=(async_process, node_name or executable, output_callback),
                    daemon=True
                )
                stream_thread.start()
                self.active_streams.append(stream_thread)

            return result

        except Exception as e:
            error_msg = f"Failed to launch node: {e}"
            logging.error(error_msg)
            return LaunchResult(
                success=False,
                error_message=error_msg
            )

    def _stream_async_output(
        self,
        async_process,
        node_name: str,
        callback: Callable[[str, str], None]
    ):
        """
        Stream output from an async process

        Args:
            async_process: AsyncProcess handle (from ExecutionManager)
            node_name: Name of node (for logging)
            callback: Callback function(line, stream)
        """
        try:
            # Poll for output while process is running
            while async_process.is_running:
                stdout_lines, stderr_lines = async_process.get_output(timeout=0.1)

                # Forward stdout
                for line in stdout_lines:
                    callback(line, 'stdout')

                # Forward stderr
                for line in stderr_lines:
                    callback(line, 'stderr')

                time.sleep(0.05)  # Small delay to avoid busy waiting

            # Get any remaining output
            stdout_lines, stderr_lines = async_process.get_output(timeout=0.1)
            for line in stdout_lines:
                callback(line, 'stdout')
            for line in stderr_lines:
                callback(line, 'stderr')

        except Exception as e:
            logging.error(f"Error streaming output for {node_name}: {e}")

    def launch_file(
        self,
        package_name: str,
        launch_file: str,
        arguments: Optional[dict] = None,
        output_callback: Optional[Callable[[str, str], None]] = None
    ) -> LaunchResult:
        """
        Launch a ROS2 launch file

        Args:
            package_name: Package containing launch file
            launch_file: Launch file name (e.g., "bringup.launch.py")
            arguments: Optional launch arguments
            output_callback: Callback for launch output

        Returns:
            LaunchResult with PID if successful
        """
        try:
            # Build ros2 launch command
            cmd_parts = ["ros2", "launch", package_name, launch_file]

            # Add arguments
            if arguments:
                for key, value in arguments.items():
                    cmd_parts.append(f"{key}:={value}")

            ros2_cmd = " ".join(cmd_parts)

            # Build full command with sourcing
            full_cmd = f"{self.source_workspace()} && {ros2_cmd}"

            logging.info(f"Launching file: {package_name}/{launch_file}")
            logging.debug(f"Command: {full_cmd}")

            # Get executor
            executor = self.execution_manager.get_executor()
            logging.info(f"Launching with {executor.get_display_name()}")

            # Launch file in background using executor
            async_process = executor.execute_command_async(
                full_cmd,
                working_dir=self.workspace_path
            )

            # Get PID from async process
            pid = async_process.pid
            if pid:
                logging.info(f"Launch file started with PID: {pid}")
            else:
                logging.warning("Could not get PID from async process")

            # Create launch result
            result = LaunchResult(
                success=True,
                pid=pid,
                node_name=f"{package_name}/{launch_file}",
                process_handle=async_process  # Store AsyncProcess instead of Popen
            )

            # Store process
            if pid:
                self.running_processes[pid] = result

            # Start output streaming if callback provided
            if output_callback:
                stream_thread = threading.Thread(
                    target=self._stream_async_output,
                    args=(async_process, launch_file, output_callback),
                    daemon=True
                )
                stream_thread.start()
                self.active_streams.append(stream_thread)

            return result

        except Exception as e:
            error_msg = f"Failed to launch file: {e}"
            logging.error(error_msg)
            return LaunchResult(
                success=False,
                error_message=error_msg
            )

    def list_nodes(self) -> List[NodeInfo]:
        """
        List all running ROS2 nodes

        Returns:
            List of NodeInfo objects
        """
        try:
            # Source environment and run ros2 node list
            cmd = f"{self.source_workspace()} && ros2 node list"
            executor = self.execution_manager.get_executor()
            result = executor.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0:
                logging.error(f"Failed to list nodes: {result.stderr}")
                return []

            # Parse output - each line is a node name
            nodes = []
            seen_nodes = set()  # Track seen nodes to avoid duplicates

            for line in result.stdout.strip().split('\n'):
                line = line.strip()
                if line and line.startswith('/') and line not in seen_nodes:
                    seen_nodes.add(line)

                    # Extract namespace and name
                    parts = line.split('/')
                    namespace = '/' + '/'.join(parts[1:-1]) if len(parts) > 2 else '/'
                    name = parts[-1] if len(parts) > 1 else line

                    nodes.append(NodeInfo(
                        name=line,  # Full name with namespace
                        namespace=namespace,
                        status=NodeStatus.RUNNING
                    ))

            logging.info(f"Found {len(nodes)} running nodes")
            return nodes

        except Exception as e:
            logging.error(f"Error listing nodes: {e}")
            return []

    def stop_node(self, node_name: str, force: bool = False) -> bool:
        """
        Stop a running node by name

        Args:
            node_name: Node name (e.g., "/velocity_publisher")
            force: If True, use SIGKILL instead of SIGTERM

        Returns:
            True if successful
        """
        try:
            # Extract just the node name without leading slash
            node_simple_name = node_name.lstrip('/')

            # Strategy: Find PID using pgrep, then kill
            # Use more specific pattern to find ROS2 nodes
            find_cmd = f"pgrep -f 'python3.*{node_simple_name}|ros2.*{node_simple_name}'"
            executor = self.execution_manager.get_executor()
            result = executor.execute_command(find_cmd, timeout=5.0)

            if result.exit_code == 0 and result.stdout.strip():
                # Found PIDs - kill them
                pids = [pid.strip() for pid in result.stdout.strip().split('\n') if pid.strip()]
                signal = "SIGKILL" if force else "SIGTERM"

                killed_count = 0
                for pid in pids:
                    kill_cmd = f"kill -{signal} {pid}"
                    kill_result = executor.execute_command(kill_cmd, timeout=5.0)
                    if kill_result.exit_code == 0:
                        logging.info(f"Killed process {pid} for node {node_name}")
                        killed_count += 1
                        # Remove from our tracking dict if present
                        if int(pid) in self.running_processes:
                            del self.running_processes[int(pid)]

                return killed_count > 0
            else:
                logging.warning(f"Node not found: {node_name}")
                return False

        except Exception as e:
            logging.error(f"Failed to stop node {node_name}: {e}")
            return False

    def stop_all_nodes(self, force: bool = False) -> int:
        """
        Stop all tracked running nodes

        Args:
            force: If True, use SIGKILL

        Returns:
            Number of nodes stopped
        """
        stopped_count = 0

        # Get list of running nodes
        nodes = self.list_nodes()

        for node in nodes:
            if self.stop_node(node.name, force):
                stopped_count += 1

        # Also kill any processes we're tracking by PID
        executor = self.execution_manager.get_executor()
        for pid in list(self.running_processes.keys()):
            try:
                signal = "SIGKILL" if force else "SIGTERM"
                kill_cmd = f"kill -{signal} {pid}"
                result = executor.execute_command(kill_cmd, timeout=5.0)
                if result.exit_code == 0:
                    del self.running_processes[pid]
                    stopped_count += 1
            except Exception as e:
                logging.error(f"Failed to kill process {pid}: {e}")

        logging.info(f"Stopped {stopped_count} nodes/processes")
        return stopped_count

    def get_node_info(self, node_name: str) -> Optional[NodeInfo]:
        """
        Get information about a node

        Args:
            node_name: Node name

        Returns:
            NodeInfo or None if not found
        """
        try:
            # Run ros2 node info
            cmd = f"{self.source_workspace()} && ros2 node info {node_name}"
            executor = self.execution_manager.get_executor()
            result = executor.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0:
                logging.warning(f"Node not found: {node_name}")
                return None

            # Parse basic info (we could parse subscribers, publishers, etc. later)
            return NodeInfo(
                name=node_name,
                namespace='/',  # TODO: Parse from output
                status=NodeStatus.RUNNING
            )

        except Exception as e:
            logging.error(f"Error getting node info for {node_name}: {e}")
            return None

    def list_topics(self) -> List[str]:
        """
        List all active ROS2 topics

        Returns:
            List of topic names
        """
        try:
            cmd = f"{self.source_workspace()} && ros2 topic list"
            executor = self.execution_manager.get_executor()
            result = executor.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0:
                logging.error(f"Failed to list topics: {result.stderr}")
                return []

            # Parse output - each line is a topic name
            topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
            logging.info(f"Found {len(topics)} topics")
            return topics

        except Exception as e:
            logging.error(f"Error listing topics: {e}")
            return []

    def echo_topic(
        self,
        topic_name: str,
        callback: Callable[[str], None],
        count: int = 0
    ):
        """
        Echo messages from a topic

        Args:
            topic_name: Topic to echo
            callback: Callback for each message line
            count: Number of messages to echo (0 = infinite)
        """
        try:
            # Build command
            cmd_parts = [f"{self.source_workspace()}", "&&", "ros2", "topic", "echo", topic_name]

            if count > 0:
                cmd_parts.extend(["--once" if count == 1 else f"--times {count}"])

            cmd = " ".join(cmd_parts)

            logging.info(f"Echoing topic: {topic_name}")

            # Stream output using executor
            executor = self.execution_manager.get_executor()
            async_process = executor.execute_command_async(cmd)

            # Stream output from async process
            while async_process.is_running:
                stdout_lines, stderr_lines = async_process.get_output(timeout=0.1)
                for line in stdout_lines:
                    callback(line)
                for line in stderr_lines:
                    callback(line)
                time.sleep(0.05)

            # Get any remaining output
            stdout_lines, stderr_lines = async_process.get_output(timeout=0.1)
            for line in stdout_lines:
                callback(line)
            for line in stderr_lines:
                callback(line)

        except Exception as e:
            logging.error(f"Error echoing topic {topic_name}: {e}")

    def cleanup(self):
        """
        Cleanup resources - stop all running processes
        """
        logging.info("Cleaning up ROS2 launcher...")
        self.stop_all_nodes(force=True)
        self.running_processes.clear()

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.cleanup()
