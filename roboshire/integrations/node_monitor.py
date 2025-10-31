"""
Node Monitor - Monitor ROS2 node health and auto-restart crashed nodes

Provides periodic polling of running nodes, crash detection, uptime tracking,
and auto-restart capabilities.

Pattern: Similar to ROS2Launcher from Phase 4
"""

import logging
import time
import threading
from typing import Dict, List, Optional, Callable, Set
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum


class NodeHealth(Enum):
    """Node health status"""
    UNKNOWN = "unknown"
    HEALTHY = "healthy"
    WARNING = "warning"
    CRASHED = "crashed"
    RESTARTING = "restarting"


@dataclass
class NodeHealthStatus:
    """Health status of a monitored node"""
    node_name: str
    health: NodeHealth = NodeHealth.UNKNOWN
    last_seen: Optional[datetime] = None
    first_seen: Optional[datetime] = None
    registration_time: Optional[datetime] = None  # Track when node was registered
    crash_count: int = 0
    restart_count: int = 0
    auto_restart_enabled: bool = False
    launch_info: Optional[dict] = None  # Store launch params for restart

    @property
    def uptime_seconds(self) -> float:
        """Calculate uptime in seconds"""
        if self.first_seen and self.last_seen:
            return (self.last_seen - self.first_seen).total_seconds()
        return 0.0

    @property
    def is_healthy(self) -> bool:
        """Check if node is healthy"""
        return self.health == NodeHealth.HEALTHY

    @property
    def crashed(self) -> bool:
        """Check if node has crashed"""
        return self.health == NodeHealth.CRASHED


@dataclass
class MonitorStats:
    """Statistics about the monitoring system"""
    total_nodes_monitored: int = 0
    healthy_nodes: int = 0
    crashed_nodes: int = 0
    total_crashes_detected: int = 0
    total_restarts: int = 0
    monitoring_uptime_seconds: float = 0.0


class NodeMonitor:
    """
    Monitor ROS2 node health and provide auto-restart functionality.

    Features:
    - Periodic polling of running nodes
    - Crash detection (node disappeared from list)
    - Uptime tracking
    - Auto-restart crashed nodes
    - Status change notifications via callbacks
    """

    def __init__(
        self,
        ros2_launcher,
        poll_interval: float = 5.0,
        status_callback: Optional[Callable[[str, NodeHealth], None]] = None
    ):
        """
        Initialize node monitor

        Args:
            ros2_launcher: ROS2Launcher instance for node operations
            poll_interval: Seconds between health checks
            status_callback: Optional callback(node_name, health) for status changes
        """
        self.launcher = ros2_launcher
        self.poll_interval = poll_interval
        self.status_callback = status_callback

        # Monitored nodes: node_name -> NodeHealthStatus
        self.monitored_nodes: Dict[str, NodeHealthStatus] = {}

        # Track nodes being gracefully stopped (don't report as crashes)
        self.gracefully_stopping: Set[str] = set()

        # Monitoring state
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.monitor_start_time: Optional[datetime] = None

        # Thread safety
        self.lock = threading.Lock()

        logging.info(f"NodeMonitor initialized with poll_interval={poll_interval}s")

    def start_monitoring(self):
        """Start background monitoring thread"""
        if self.monitoring:
            logging.warning("Monitoring already started")
            return

        self.monitoring = True
        self.monitor_start_time = datetime.now()

        self.monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="NodeMonitorThread"
        )
        self.monitor_thread.start()

        logging.info("Node monitoring started")

    def stop_monitoring(self):
        """Stop monitoring thread"""
        if not self.monitoring:
            return

        self.monitoring = False

        if self.monitor_thread:
            self.monitor_thread.join(timeout=self.poll_interval + 2.0)

        logging.info("Node monitoring stopped")

    def register_node(
        self,
        node_name: str,
        auto_restart: bool = False,
        launch_info: Optional[dict] = None
    ):
        """
        Register a node for monitoring

        Args:
            node_name: Full node name (e.g., "/velocity_publisher")
            auto_restart: Enable auto-restart on crash
            launch_info: Launch parameters for restart (package, executable, etc.)
        """
        with self.lock:
            if node_name not in self.monitored_nodes:
                status = NodeHealthStatus(
                    node_name=node_name,
                    health=NodeHealth.UNKNOWN,
                    first_seen=None,  # Will be set when node is first discovered
                    last_seen=None,
                    registration_time=datetime.now(),  # Track registration time for grace period
                    auto_restart_enabled=auto_restart,
                    launch_info=launch_info
                )
                self.monitored_nodes[node_name] = status
                logging.info(f"Registered node for monitoring: {node_name} (auto_restart={auto_restart})")
            else:
                # Update existing
                status = self.monitored_nodes[node_name]
                status.auto_restart_enabled = auto_restart
                if launch_info:
                    status.launch_info = launch_info

    def unregister_node(self, node_name: str):
        """
        Unregister a node from monitoring

        Args:
            node_name: Node name to unregister
        """
        with self.lock:
            if node_name in self.monitored_nodes:
                del self.monitored_nodes[node_name]
                # Also remove from gracefully_stopping set
                self.gracefully_stopping.discard(node_name)
                logging.info(f"Unregistered node: {node_name}")

    def mark_node_stopping(self, node_name: str):
        """
        Mark a node as being gracefully stopped (don't report as crash)

        Args:
            node_name: Node name being stopped
        """
        with self.lock:
            self.gracefully_stopping.add(node_name)
            logging.debug(f"Marked node as gracefully stopping: {node_name}")

    def enable_auto_restart(self, node_name: str, launch_info: Optional[dict] = None):
        """
        Enable auto-restart for a node

        Args:
            node_name: Node name
            launch_info: Launch parameters (package, executable, etc.)
        """
        with self.lock:
            if node_name in self.monitored_nodes:
                status = self.monitored_nodes[node_name]
                status.auto_restart_enabled = True
                if launch_info:
                    status.launch_info = launch_info
                logging.info(f"Auto-restart enabled for: {node_name}")

    def disable_auto_restart(self, node_name: str):
        """
        Disable auto-restart for a node

        Args:
            node_name: Node name
        """
        with self.lock:
            if node_name in self.monitored_nodes:
                self.monitored_nodes[node_name].auto_restart_enabled = False
                logging.info(f"Auto-restart disabled for: {node_name}")

    def get_node_status(self, node_name: str) -> Optional[NodeHealthStatus]:
        """
        Get health status of a node

        Args:
            node_name: Node name

        Returns:
            NodeHealthStatus or None if not monitored
        """
        with self.lock:
            return self.monitored_nodes.get(node_name)

    def get_all_statuses(self) -> Dict[str, NodeHealthStatus]:
        """
        Get all node statuses

        Returns:
            Dict of node_name -> NodeHealthStatus
        """
        with self.lock:
            return self.monitored_nodes.copy()

    def get_stats(self) -> MonitorStats:
        """
        Get monitoring statistics

        Returns:
            MonitorStats object
        """
        with self.lock:
            stats = MonitorStats()
            stats.total_nodes_monitored = len(self.monitored_nodes)

            for status in self.monitored_nodes.values():
                if status.health == NodeHealth.HEALTHY:
                    stats.healthy_nodes += 1
                elif status.health == NodeHealth.CRASHED:
                    stats.crashed_nodes += 1

                stats.total_crashes_detected += status.crash_count
                stats.total_restarts += status.restart_count

            if self.monitor_start_time:
                stats.monitoring_uptime_seconds = (
                    datetime.now() - self.monitor_start_time
                ).total_seconds()

            return stats

    def _monitor_loop(self):
        """Main monitoring loop (runs in background thread)"""
        logging.info("Monitor loop started")

        while self.monitoring:
            try:
                self._check_node_health()
            except Exception as e:
                logging.error(f"Error in monitor loop: {e}")

            # Sleep until next poll
            time.sleep(self.poll_interval)

        logging.info("Monitor loop stopped")

    def _check_node_health(self):
        """Check health of all monitored nodes"""
        # Grace period for newly registered nodes (allows time for DDS discovery)
        DISCOVERY_GRACE_PERIOD = 15.0  # seconds

        try:
            # Get list of currently running nodes
            running_nodes = self.launcher.list_nodes()
            running_node_names = {node.name for node in running_nodes}

            logging.info(f"Found {len(running_node_names)} running nodes")

            with self.lock:
                # Check each monitored node
                for node_name, status in self.monitored_nodes.items():
                    previous_health = status.health

                    # Check if node is in grace period (recently registered)
                    in_grace_period = False
                    time_since_registration = 0.0

                    if status.registration_time:
                        time_since_registration = (datetime.now() - status.registration_time).total_seconds()
                        in_grace_period = time_since_registration < DISCOVERY_GRACE_PERIOD

                    # Create name variants to match different formats
                    # Handles: "node_name", "/node_name", "/namespace/node_name"
                    name_variants = {
                        node_name,
                        node_name.lstrip('/'),
                        f"/{node_name.lstrip('/')}"
                    }

                    # Check if node is found (in any name variant)
                    node_found = any(variant in running_node_names for variant in name_variants)

                    if node_found:
                        # Node is running
                        status.health = NodeHealth.HEALTHY
                        status.last_seen = datetime.now()

                        # Set first_seen on first discovery
                        if status.first_seen is None:
                            status.first_seen = datetime.now()
                            if in_grace_period:
                                logging.info(f"Node {node_name} discovered after {time_since_registration:.1f}s")

                        # If node was previously crashed, log recovery
                        if previous_health == NodeHealth.CRASHED:
                            logging.info(f"Node recovered: {node_name}")

                    else:
                        # Node not found in running list

                        # GRACE PERIOD: Don't report crashes for newly registered nodes
                        if in_grace_period:
                            # Still waiting for DDS discovery to complete
                            if previous_health == NodeHealth.UNKNOWN:
                                logging.debug(f"Node {node_name} still in discovery grace period ({time_since_registration:.1f}s/{DISCOVERY_GRACE_PERIOD}s)")
                            # If node was healthy but now missing during grace period, might be transient
                            continue  # Skip crash detection during grace period

                        # Past grace period - check if this is a crash
                        if previous_health in (NodeHealth.HEALTHY, NodeHealth.UNKNOWN):
                            # Check if this was a graceful shutdown
                            if node_name in self.gracefully_stopping:
                                # Graceful shutdown - not a crash
                                logging.info(f"Node stopped gracefully: {node_name}")
                                # Remove from gracefully_stopping set
                                self.gracefully_stopping.discard(node_name)
                                # Don't mark as crashed, just leave as UNKNOWN or remove from monitoring
                                # (The main window should unregister it)
                            else:
                                # Unexpected shutdown - likely a crash
                                status.health = NodeHealth.CRASHED
                                status.crash_count += 1
                                logging.warning(f"Node crashed unexpectedly: {node_name} (crash count: {status.crash_count})")

                                # Attempt auto-restart if enabled
                                if status.auto_restart_enabled:
                                    self._attempt_restart(node_name, status)

                    # Notify status change
                    if previous_health != status.health and self.status_callback:
                        try:
                            self.status_callback(node_name, status.health)
                        except Exception as e:
                            logging.error(f"Error in status callback: {e}")

        except Exception as e:
            logging.error(f"Error checking node health: {e}")

    def _attempt_restart(self, node_name: str, status: NodeHealthStatus):
        """
        Attempt to restart a crashed node

        Args:
            node_name: Node name
            status: NodeHealthStatus object
        """
        if not status.launch_info:
            logging.error(f"Cannot restart {node_name}: no launch info available")
            return

        try:
            launch_info = status.launch_info
            logging.info(f"Attempting to restart {node_name}...")

            status.health = NodeHealth.RESTARTING

            # Launch node using stored launch info
            result = self.launcher.launch_node(
                package_name=launch_info.get('package_name'),
                executable=launch_info.get('executable'),
                node_name=launch_info.get('node_name'),
                namespace=launch_info.get('namespace'),
                parameters=launch_info.get('parameters'),
                remappings=launch_info.get('remappings')
            )

            if result.success:
                status.restart_count += 1
                status.first_seen = datetime.now()  # Reset uptime
                logging.info(f"Successfully restarted {node_name} (restart count: {status.restart_count})")
            else:
                status.health = NodeHealth.CRASHED
                logging.error(f"Failed to restart {node_name}: {result.error_message}")

        except Exception as e:
            logging.error(f"Error restarting {node_name}: {e}")
            status.health = NodeHealth.CRASHED

    def clear_all_nodes(self):
        """Clear all monitored nodes"""
        with self.lock:
            self.monitored_nodes.clear()
            logging.info("Cleared all monitored nodes")

    def is_monitoring(self) -> bool:
        """Check if monitoring is active"""
        return self.monitoring

    def __enter__(self):
        """Context manager entry"""
        self.start_monitoring()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop_monitoring()
