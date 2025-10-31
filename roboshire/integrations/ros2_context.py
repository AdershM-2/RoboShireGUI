"""
ROS2 Context Manager - Singleton Pattern

Provides a single, shared ROS2 context and executor for the entire application.
This prevents multiple rclpy.init() calls which cause crashes.

Usage:
    from roboshire.integrations.ros2_context import ROS2ContextManager

    # In any widget
    context_mgr = ROS2ContextManager()
    node = context_mgr.create_node('my_node')

    # Cleanup (only call destroy_node, NOT rclpy.shutdown())
    context_mgr.destroy_node(node)

Author: RoboShire Team
License: Apache 2.0
Version: 2.4.1
"""

import threading
import logging
from typing import Optional, List
from contextlib import contextmanager

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class ROS2ContextManager:
    """
    Singleton manager for ROS2 context and executor

    Ensures only one rclpy.init() call across the entire application.
    Provides thread-safe node creation and management.
    """

    _instance: Optional['ROS2ContextManager'] = None
    _lock = threading.Lock()

    def __new__(cls):
        """Singleton pattern - only one instance"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """Initialize ROS2 context and executor (once)"""
        if self._initialized:
            return

        self.logger = logging.getLogger(__name__)

        if not ROS2_AVAILABLE:
            self.logger.warning("ROS2 not available - install rclpy")
            self._initialized = True
            return

        # Initialize context
        self._context: Optional[Context] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._executor_thread: Optional[threading.Thread] = None
        self._nodes: List[Node] = []
        self._node_lock = threading.Lock()
        self._running = False

        self._initialized = True
        self.logger.info("ROS2 Context Manager initialized")

    def is_available(self) -> bool:
        """Check if ROS2 is available"""
        return ROS2_AVAILABLE

    def get_context(self) -> Optional[Context]:
        """
        Get the shared ROS2 context

        Returns:
            Context object, or None if ROS2 not available
        """
        if not ROS2_AVAILABLE:
            return None

        if self._context is None:
            with self._lock:
                if self._context is None:
                    try:
                        self._context = Context()
                        rclpy.init(context=self._context)
                        self.logger.info("ROS2 context initialized")
                    except Exception as e:
                        self.logger.error(f"Failed to initialize ROS2 context: {e}")
                        return None

        return self._context

    def get_executor(self) -> Optional[MultiThreadedExecutor]:
        """
        Get the shared executor

        Returns:
            MultiThreadedExecutor, or None if ROS2 not available
        """
        if not ROS2_AVAILABLE:
            return None

        if self._executor is None:
            with self._lock:
                if self._executor is None:
                    context = self.get_context()
                    if context is None:
                        return None

                    try:
                        self._executor = MultiThreadedExecutor(context=context)
                        self._running = True

                        # Start executor in background thread
                        self._executor_thread = threading.Thread(
                            target=self._spin_executor,
                            daemon=True,
                            name="ROS2-Executor"
                        )
                        self._executor_thread.start()
                        self.logger.info("ROS2 executor started")
                    except Exception as e:
                        self.logger.error(f"Failed to start executor: {e}")
                        return None

        return self._executor

    def _spin_executor(self):
        """Background thread to spin the executor"""
        try:
            while self._running and self._executor:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.logger.error(f"Executor error: {e}")

    def create_node(self, node_name: str, **kwargs) -> Optional[Node]:
        """
        Create and register a ROS2 node

        Args:
            node_name: Name for the node
            **kwargs: Additional arguments for Node constructor

        Returns:
            Node object, or None on failure
        """
        if not ROS2_AVAILABLE:
            self.logger.warning(f"Cannot create node '{node_name}' - ROS2 not available")
            return None

        context = self.get_context()
        executor = self.get_executor()

        if context is None or executor is None:
            self.logger.error(f"Cannot create node '{node_name}' - context/executor not ready")
            return None

        try:
            # Create node with shared context
            node = Node(node_name, context=context, **kwargs)

            # Add to executor
            with self._node_lock:
                executor.add_node(node)
                self._nodes.append(node)

            self.logger.info(f"Created node: {node_name}")
            return node

        except Exception as e:
            self.logger.error(f"Failed to create node '{node_name}': {e}")
            return None

    def destroy_node(self, node: Node):
        """
        Destroy a node (remove from executor and cleanup)

        Args:
            node: Node to destroy
        """
        if not ROS2_AVAILABLE or node is None:
            return

        try:
            with self._node_lock:
                executor = self.get_executor()
                if executor and node in self._nodes:
                    # Remove from executor
                    executor.remove_node(node)
                    self._nodes.remove(node)

                # Destroy node
                node.destroy_node()

            self.logger.info(f"Destroyed node: {node.get_name()}")

        except Exception as e:
            self.logger.error(f"Error destroying node: {e}")

    def get_active_nodes(self) -> List[str]:
        """Get list of active node names"""
        with self._node_lock:
            return [node.get_name() for node in self._nodes]

    def shutdown(self):
        """
        Shutdown ROS2 context and executor

        WARNING: Should only be called by main application on exit!
        DO NOT call from individual widgets!
        """
        if not ROS2_AVAILABLE:
            return

        self.logger.info("Shutting down ROS2 Context Manager...")

        # Stop executor thread
        self._running = False

        # Destroy all nodes
        with self._node_lock:
            for node in self._nodes[:]:  # Copy list to avoid modification during iteration
                try:
                    self._executor.remove_node(node)
                    node.destroy_node()
                except Exception as e:
                    self.logger.error(f"Error destroying node: {e}")
            self._nodes.clear()

        # Shutdown executor
        if self._executor:
            try:
                self._executor.shutdown()
            except Exception as e:
                self.logger.error(f"Error shutting down executor: {e}")

        # Wait for executor thread to finish
        if self._executor_thread and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=2.0)

        # Shutdown context
        if self._context:
            try:
                rclpy.shutdown(context=self._context)
                self.logger.info("ROS2 context shutdown complete")
            except Exception as e:
                self.logger.error(f"Error shutting down context: {e}")

        # Reset state
        self._context = None
        self._executor = None
        self._executor_thread = None

    @contextmanager
    def managed_node(self, node_name: str, **kwargs):
        """
        Context manager for automatic node lifecycle

        Usage:
            with ROS2ContextManager().managed_node('my_node') as node:
                if node:
                    # Use node
                    node.get_logger().info("Hello!")
            # Node automatically destroyed on exit
        """
        node = self.create_node(node_name, **kwargs)
        try:
            yield node
        finally:
            if node:
                self.destroy_node(node)


# Convenience functions for quick access
def get_ros2_context_manager() -> ROS2ContextManager:
    """Get the singleton ROS2 Context Manager"""
    return ROS2ContextManager()


def create_ros2_node(node_name: str, **kwargs) -> Optional[Node]:
    """Convenience function to create a node"""
    return get_ros2_context_manager().create_node(node_name, **kwargs)


def destroy_ros2_node(node: Node):
    """Convenience function to destroy a node"""
    get_ros2_context_manager().destroy_node(node)


# Example usage
if __name__ == "__main__":
    # This is for testing only
    logging.basicConfig(level=logging.INFO)

    print("Testing ROS2 Context Manager...")

    mgr = ROS2ContextManager()

    if mgr.is_available():
        print("✓ ROS2 is available")

        # Create a test node
        node1 = mgr.create_node('test_node_1')
        if node1:
            print(f"✓ Created {node1.get_name()}")
            node1.get_logger().info("Hello from test node 1!")

        # Create another node (should share context)
        node2 = mgr.create_node('test_node_2')
        if node2:
            print(f"✓ Created {node2.get_name()}")
            node2.get_logger().info("Hello from test node 2!")

        # Show active nodes
        print(f"Active nodes: {mgr.get_active_nodes()}")

        # Cleanup
        import time
        time.sleep(2)  # Let nodes publish messages

        mgr.destroy_node(node1)
        print("✓ Destroyed test_node_1")

        mgr.destroy_node(node2)
        print("✓ Destroyed test_node_2")

        print(f"Active nodes: {mgr.get_active_nodes()}")

        # Shutdown (only for testing - main app does this)
        mgr.shutdown()
        print("✓ Shutdown complete")
    else:
        print("✗ ROS2 not available")
