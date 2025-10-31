"""
Lifecycle Manager

Controls ROS2 lifecycle node state transitions via SSH.
Provides GUI interface for managing lifecycle node states.

Lifecycle States:
- UNCONFIGURED: Initial state after construction
- INACTIVE: Configured but not yet active
- ACTIVE: Fully operational
- FINALIZED: Shutdown complete

State Transitions:
- configure: UNCONFIGURED -> INACTIVE
- activate: INACTIVE -> ACTIVE
- deactivate: ACTIVE -> INACTIVE
- cleanup: INACTIVE -> UNCONFIGURED
- shutdown: ANY -> FINALIZED
"""

from typing import Optional, Dict, Callable, List
from enum import Enum
import logging
from roboshire.backend.ssh_subprocess import SSHManagerSubprocess


class LifecycleState(Enum):
    """Lifecycle node states"""
    UNKNOWN = "unknown"
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"


class LifecycleTransition(Enum):
    """Lifecycle state transitions"""
    CONFIGURE = "configure"
    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"
    CLEANUP = "cleanup"
    SHUTDOWN = "shutdown"


class LifecycleNodeInfo:
    """Information about a lifecycle node"""

    def __init__(self, name: str, state: LifecycleState = LifecycleState.UNKNOWN):
        self.name = name
        self.state = state
        self.last_transition = None
        self.error_message = ""

    def __repr__(self):
        return f"LifecycleNodeInfo(name={self.name}, state={self.state.value})"


class LifecycleManager:
    """
    Manages lifecycle node state transitions

    Uses ros2 lifecycle CLI commands via SSH to control remote nodes.
    """

    def __init__(self, ssh_manager: SSHManagerSubprocess):
        """
        Initialize Lifecycle Manager

        Args:
            ssh_manager: SSHManagerSubprocess instance for remote execution
        """
        self.ssh_manager = ssh_manager
        self.logger = logging.getLogger(__name__)

        # Track lifecycle nodes
        self.nodes: Dict[str, LifecycleNodeInfo] = {}

        # Callbacks for state changes
        self.state_change_callbacks: List[Callable] = []

    def add_state_change_callback(self, callback: Callable):
        """
        Add callback to be notified of state changes

        Callback signature: callback(node_name: str, new_state: LifecycleState)
        """
        self.state_change_callbacks.append(callback)

    def _notify_state_change(self, node_name: str, new_state: LifecycleState):
        """Notify all callbacks of state change"""
        for callback in self.state_change_callbacks:
            try:
                callback(node_name, new_state)
            except Exception as e:
                self.logger.error(f"Error in state change callback: {e}")

    def list_lifecycle_nodes(self) -> List[str]:
        """
        List all available lifecycle nodes

        Returns:
            List of lifecycle node names
        """
        try:
            # Source ROS2 and list lifecycle nodes
            cmd = """
            source /opt/ros/humble/setup.bash
            cd /mnt/hgfs/ROS2_PROJECT/workspace
            if [ -f install/setup.bash ]; then
                source install/setup.bash
            fi
            ros2 lifecycle nodes
            """

            result = self.ssh_manager.execute_command(cmd.strip())

            if result.returncode == 0 and result.stdout:
                nodes = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
                self.logger.info(f"Found {len(nodes)} lifecycle nodes: {nodes}")
                return nodes
            else:
                self.logger.warning("No lifecycle nodes found")
                return []

        except Exception as e:
            self.logger.error(f"Failed to list lifecycle nodes: {e}")
            return []

    def get_node_state(self, node_name: str) -> LifecycleState:
        """
        Get current state of a lifecycle node

        Args:
            node_name: Name of the lifecycle node (e.g., "/velocity_publisher")

        Returns:
            Current LifecycleState
        """
        try:
            # Source ROS2 and get node state
            cmd = f"""
            source /opt/ros/humble/setup.bash
            cd /mnt/hgfs/ROS2_PROJECT/workspace
            if [ -f install/setup.bash ]; then
                source install/setup.bash
            fi
            ros2 lifecycle get {node_name}
            """

            result = self.ssh_manager.execute_command(cmd.strip())

            if result.returncode == 0 and result.stdout:
                state_output = result.stdout.strip().lower()
                self.logger.debug(f"Node {node_name} state output: {state_output}")

                # Parse state from output
                if "unconfigured" in state_output:
                    state = LifecycleState.UNCONFIGURED
                elif "inactive" in state_output:
                    state = LifecycleState.INACTIVE
                elif "active" in state_output:
                    state = LifecycleState.ACTIVE
                elif "finalized" in state_output:
                    state = LifecycleState.FINALIZED
                else:
                    state = LifecycleState.UNKNOWN

                # Update cached state
                if node_name in self.nodes:
                    self.nodes[node_name].state = state
                else:
                    self.nodes[node_name] = LifecycleNodeInfo(node_name, state)

                return state
            else:
                self.logger.warning(f"Could not get state for {node_name}")
                return LifecycleState.UNKNOWN

        except Exception as e:
            self.logger.error(f"Failed to get node state for {node_name}: {e}")
            return LifecycleState.UNKNOWN

    def transition_node(
        self,
        node_name: str,
        transition: LifecycleTransition
    ) -> bool:
        """
        Trigger a state transition for a lifecycle node

        Args:
            node_name: Name of the lifecycle node
            transition: Desired transition

        Returns:
            True if transition successful, False otherwise
        """
        try:
            self.logger.info(f"Triggering {transition.value} on {node_name}")

            # Build transition command
            cmd = f"""
            source /opt/ros/humble/setup.bash
            cd /mnt/hgfs/ROS2_PROJECT/workspace
            if [ -f install/setup.bash ]; then
                source install/setup.bash
            fi
            ros2 lifecycle set {node_name} {transition.value}
            """

            result = self.ssh_manager.execute_command(cmd.strip())

            if result.returncode == 0:
                self.logger.info(f"Transition {transition.value} successful for {node_name}")

                # Update state after transition
                new_state = self.get_node_state(node_name)
                self._notify_state_change(node_name, new_state)

                return True
            else:
                error_msg = result.stderr or "Unknown error"
                self.logger.error(f"Transition {transition.value} failed for {node_name}: {error_msg}")

                if node_name in self.nodes:
                    self.nodes[node_name].error_message = error_msg

                return False

        except Exception as e:
            self.logger.error(f"Exception during transition {transition.value} for {node_name}: {e}")
            return False

    def configure_node(self, node_name: str) -> bool:
        """Transition node: UNCONFIGURED -> INACTIVE"""
        return self.transition_node(node_name, LifecycleTransition.CONFIGURE)

    def activate_node(self, node_name: str) -> bool:
        """Transition node: INACTIVE -> ACTIVE"""
        return self.transition_node(node_name, LifecycleTransition.ACTIVATE)

    def deactivate_node(self, node_name: str) -> bool:
        """Transition node: ACTIVE -> INACTIVE"""
        return self.transition_node(node_name, LifecycleTransition.DEACTIVATE)

    def cleanup_node(self, node_name: str) -> bool:
        """Transition node: INACTIVE -> UNCONFIGURED"""
        return self.transition_node(node_name, LifecycleTransition.CLEANUP)

    def shutdown_node(self, node_name: str) -> bool:
        """Transition node: ANY -> FINALIZED"""
        return self.transition_node(node_name, LifecycleTransition.SHUTDOWN)

    def auto_start_node(self, node_name: str) -> bool:
        """
        Automatically start a lifecycle node (configure + activate)

        Args:
            node_name: Name of the lifecycle node

        Returns:
            True if fully activated, False otherwise
        """
        try:
            # Get current state
            current_state = self.get_node_state(node_name)

            if current_state == LifecycleState.ACTIVE:
                self.logger.info(f"Node {node_name} already ACTIVE")
                return True

            elif current_state == LifecycleState.UNCONFIGURED:
                # Need to configure first
                if not self.configure_node(node_name):
                    return False

            # Should be INACTIVE now, activate it
            if self.activate_node(node_name):
                self.logger.info(f"Node {node_name} auto-started successfully")
                return True
            else:
                return False

        except Exception as e:
            self.logger.error(f"Failed to auto-start {node_name}: {e}")
            return False

    def get_available_transitions(self, node_name: str) -> List[str]:
        """
        Get list of available transitions for a node in its current state

        Args:
            node_name: Name of the lifecycle node

        Returns:
            List of available transition names
        """
        try:
            cmd = f"""
            source /opt/ros/humble/setup.bash
            cd /mnt/hgfs/ROS2_PROJECT/workspace
            if [ -f install/setup.bash ]; then
                source install/setup.bash
            fi
            ros2 lifecycle list {node_name}
            """

            result = self.ssh_manager.execute_command(cmd.strip())

            if result.returncode == 0 and result.stdout:
                # Parse transitions from output
                transitions = []
                for line in result.stdout.split('\n'):
                    line = line.strip().lower()
                    if any(t.value in line for t in LifecycleTransition):
                        # Extract transition name
                        for trans in LifecycleTransition:
                            if trans.value in line:
                                transitions.append(trans.value)

                return transitions
            else:
                return []

        except Exception as e:
            self.logger.error(f"Failed to get available transitions for {node_name}: {e}")
            return []

    def refresh_nodes(self):
        """Refresh the list of lifecycle nodes and their states"""
        try:
            node_names = self.list_lifecycle_nodes()

            for node_name in node_names:
                state = self.get_node_state(node_name)

                if node_name not in self.nodes:
                    self.nodes[node_name] = LifecycleNodeInfo(node_name, state)
                else:
                    old_state = self.nodes[node_name].state
                    if old_state != state:
                        self.nodes[node_name].state = state
                        self._notify_state_change(node_name, state)

            self.logger.info(f"Refreshed {len(self.nodes)} lifecycle nodes")

        except Exception as e:
            self.logger.error(f"Failed to refresh lifecycle nodes: {e}")

    def get_all_nodes(self) -> Dict[str, LifecycleNodeInfo]:
        """Get all tracked lifecycle nodes and their info"""
        return self.nodes.copy()
