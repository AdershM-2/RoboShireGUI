"""
Topic Inspector - Inspect ROS2 topics

Features:
- List all available topics
- Get topic info (type, publishers, subscribers)
- Echo topic messages
- Measure topic frequency (hz)
- Measure topic bandwidth (bw)
- Parse YAML message output
"""

import logging
import re
import yaml
import threading
from typing import List, Optional, Callable, Dict, Any
from dataclasses import dataclass

from roboshire.backend.execution_manager import ExecutionManager


@dataclass
class TopicInfo:
    """Information about a ROS2 topic"""
    name: str
    msg_type: str = "unknown"
    publisher_count: int = 0
    subscription_count: int = 0
    publishers: List[str] = None
    subscribers: List[str] = None

    def __post_init__(self):
        if self.publishers is None:
            self.publishers = []
        if self.subscribers is None:
            self.subscribers = []


@dataclass
class TopicStats:
    """Statistics for a topic"""
    topic_name: str
    frequency_hz: float = 0.0
    bandwidth_bps: float = 0.0
    message_count: int = 0


class TopicInspector:
    """
    Inspect ROS2 topics

    Features:
    - List topics
    - Get topic info
    - Echo messages
    - Measure frequency
    - Measure bandwidth
    """

    def __init__(self, execution_manager: ExecutionManager, ros_distro: str = "humble"):
        """
        Initialize topic inspector

        Args:
            execution_manager: Execution manager for local/remote execution
            ros_distro: ROS2 distribution name
        """
        self.execution_manager = execution_manager
        self.ros_distro = ros_distro

        # Echo streams
        self.echo_streams: Dict[str, threading.Thread] = {}
        self.echo_active: Dict[str, bool] = {}

        logging.info("TopicInspector initialized")

    def list_topics(self) -> List[str]:
        """
        List all available topics

        Returns:
            List of topic names
        """
        try:
            executor = self.execution_manager.get_executor()
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 topic list"
            result = executor.execute_command(cmd, timeout=10.0)

            if result.exit_code == 0 and result.stdout:
                topics = [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
                logging.info(f"Found {len(topics)} topics")
                return topics
            else:
                logging.warning(f"Failed to list topics: {result.stderr}")
                return []

        except Exception as e:
            logging.error(f"Error listing topics: {e}")
            return []

    def get_topic_info(self, topic_name: str) -> Optional[TopicInfo]:
        """
        Get detailed information about a topic

        Args:
            topic_name: Topic name (e.g., "/cmd_vel")

        Returns:
            TopicInfo object or None if error
        """
        try:
            executor = self.execution_manager.get_executor()
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 topic info {topic_name} -v"
            result = executor.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0:
                logging.warning(f"Failed to get topic info for {topic_name}: {result.stderr}")
                return None

            # Parse output
            info = TopicInfo(name=topic_name)
            output = result.stdout

            # Extract topic type
            type_match = re.search(r'Type:\s*(\S+)', output)
            if type_match:
                info.msg_type = type_match.group(1)

            # Extract publisher count
            pub_match = re.search(r'Publisher count:\s*(\d+)', output)
            if pub_match:
                info.publisher_count = int(pub_match.group(1))

            # Extract subscription count
            sub_match = re.search(r'Subscription count:\s*(\d+)', output)
            if sub_match:
                info.subscription_count = int(sub_match.group(1))

            # Extract publisher node names (if available)
            publishers_section = re.search(r'Publishers:\s*(.*?)(?=Subscription|$)', output, re.DOTALL)
            if publishers_section:
                pub_lines = publishers_section.group(1).strip().split('\n')
                for line in pub_lines:
                    node_match = re.search(r'Node name:\s*(\S+)', line)
                    if node_match:
                        info.publishers.append(node_match.group(1))

            # Extract subscriber node names (if available)
            subscribers_section = re.search(r'Subscription count:.*?\n(.*?)$', output, re.DOTALL)
            if subscribers_section:
                sub_lines = subscribers_section.group(1).strip().split('\n')
                for line in sub_lines:
                    node_match = re.search(r'Node name:\s*(\S+)', line)
                    if node_match:
                        info.subscribers.append(node_match.group(1))

            logging.info(f"Got topic info for {topic_name}: type={info.msg_type}, pubs={info.publisher_count}, subs={info.subscription_count}")
            return info

        except Exception as e:
            logging.error(f"Error getting topic info for {topic_name}: {e}")
            return None

    def echo_topic_once(self, topic_name: str) -> Optional[Dict[str, Any]]:
        """
        Echo a single message from a topic

        Args:
            topic_name: Topic name

        Returns:
            Parsed message as dictionary, or None if error
        """
        try:
            executor = self.execution_manager.get_executor()
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && timeout 5 ros2 topic echo {topic_name} --once"
            result = executor.execute_command(cmd, timeout=10.0)

            if result.exit_code != 0 or not result.stdout:
                logging.warning(f"Failed to echo topic {topic_name}: {result.stderr}")
                return None

            # Parse YAML output (handle multiple documents with ---)
            try:
                # ros2 topic echo may output multiple YAML documents separated by ---
                # Use safe_load_all and take the first non-empty document
                documents = list(yaml.safe_load_all(result.stdout))

                # Filter out None/empty documents and return the first valid one
                for doc in documents:
                    if doc is not None:
                        return doc

                logging.warning(f"No valid YAML document found in topic {topic_name} output")
                return None

            except yaml.YAMLError as e:
                logging.error(f"Failed to parse YAML from topic {topic_name}: {e}")
                return None

        except Exception as e:
            logging.error(f"Error echoing topic {topic_name}: {e}")
            return None

    def start_echo_stream(
        self,
        topic_name: str,
        callback: Callable[[Dict[str, Any]], None],
        max_messages: int = 0
    ):
        """
        Start streaming messages from a topic

        Args:
            topic_name: Topic name
            callback: Function to call with each message (dict)
            max_messages: Maximum messages to receive (0 = unlimited)
        """
        if topic_name in self.echo_streams and self.echo_active.get(topic_name, False):
            logging.warning(f"Echo stream already active for {topic_name}")
            return

        self.echo_active[topic_name] = True

        def echo_loop():
            message_count = 0
            while self.echo_active.get(topic_name, False):
                try:
                    message = self.echo_topic_once(topic_name)
                    if message:
                        callback(message)
                        message_count += 1

                        if max_messages > 0 and message_count >= max_messages:
                            break

                except Exception as e:
                    logging.error(f"Error in echo stream for {topic_name}: {e}")
                    break

            self.echo_active[topic_name] = False
            logging.info(f"Echo stream stopped for {topic_name} after {message_count} messages")

        thread = threading.Thread(target=echo_loop, daemon=True, name=f"EchoStream-{topic_name}")
        self.echo_streams[topic_name] = thread
        thread.start()

        logging.info(f"Started echo stream for {topic_name}")

    def stop_echo_stream(self, topic_name: str):
        """
        Stop streaming messages from a topic

        Args:
            topic_name: Topic name
        """
        if topic_name in self.echo_active:
            self.echo_active[topic_name] = False
            logging.info(f"Stopping echo stream for {topic_name}")

    def get_topic_hz(self, topic_name: str, duration: float = 5.0) -> Optional[float]:
        """
        Get topic publish frequency in Hz

        Args:
            topic_name: Topic name
            duration: Duration to measure (seconds)

        Returns:
            Frequency in Hz, or None if error
        """
        try:
            executor = self.execution_manager.get_executor()
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && timeout {duration + 2} ros2 topic hz {topic_name}"
            result = executor.execute_command(cmd, timeout=duration + 5.0)

            if result.exit_code not in (0, 124) or not result.stdout:  # 124 = timeout exit code
                logging.warning(f"Failed to measure hz for {topic_name}: {result.stderr}")
                return None

            # Parse output - look for "average rate: X.XXX"
            match = re.search(r'average rate:\s*([\d.]+)', result.stdout)
            if match:
                hz = float(match.group(1))
                logging.info(f"Topic {topic_name} frequency: {hz:.2f} Hz")
                return hz
            else:
                logging.warning(f"Could not parse hz from output: {result.stdout}")
                return None

        except Exception as e:
            logging.error(f"Error measuring hz for {topic_name}: {e}")
            return None

    def get_topic_bw(self, topic_name: str, duration: float = 5.0) -> Optional[float]:
        """
        Get topic bandwidth in bytes per second

        Args:
            topic_name: Topic name
            duration: Duration to measure (seconds)

        Returns:
            Bandwidth in bytes/sec, or None if error
        """
        try:
            executor = self.execution_manager.get_executor()
            cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && timeout {duration + 2} ros2 topic bw {topic_name}"
            result = executor.execute_command(cmd, timeout=duration + 5.0)

            if result.exit_code not in (0, 124) or not result.stdout:
                logging.warning(f"Failed to measure bw for {topic_name}: {result.stderr}")
                return None

            # Parse output - look for "average: X.XXX"
            match = re.search(r'average:\s*([\d.]+)\s*([KMG]?B/s)', result.stdout)
            if match:
                value = float(match.group(1))
                unit = match.group(2)

                # Convert to bytes/sec
                if unit.startswith('KB'):
                    value *= 1024
                elif unit.startswith('MB'):
                    value *= 1024 * 1024
                elif unit.startswith('GB'):
                    value *= 1024 * 1024 * 1024

                logging.info(f"Topic {topic_name} bandwidth: {value:.2f} bytes/sec")
                return value
            else:
                logging.warning(f"Could not parse bw from output: {result.stdout}")
                return None

        except Exception as e:
            logging.error(f"Error measuring bw for {topic_name}: {e}")
            return None

    def get_topic_stats(self, topic_name: str, duration: float = 5.0) -> Optional[TopicStats]:
        """
        Get comprehensive statistics for a topic

        Args:
            topic_name: Topic name
            duration: Duration to measure (seconds)

        Returns:
            TopicStats object or None if error
        """
        stats = TopicStats(topic_name=topic_name)

        # Get frequency
        hz = self.get_topic_hz(topic_name, duration)
        if hz:
            stats.frequency_hz = hz

        # Get bandwidth
        bw = self.get_topic_bw(topic_name, duration)
        if bw:
            stats.bandwidth_bps = bw

        return stats

    def format_message(self, message: Dict[str, Any], indent: int = 0) -> str:
        """
        Format a message dictionary as a human-readable string

        Args:
            message: Message dictionary
            indent: Indentation level

        Returns:
            Formatted string
        """
        lines = []
        indent_str = "  " * indent

        if isinstance(message, dict):
            for key, value in message.items():
                if isinstance(value, (dict, list)):
                    lines.append(f"{indent_str}{key}:")
                    lines.append(self.format_message(value, indent + 1))
                else:
                    lines.append(f"{indent_str}{key}: {value}")
        elif isinstance(message, list):
            for i, item in enumerate(message):
                if isinstance(item, (dict, list)):
                    lines.append(f"{indent_str}[{i}]:")
                    lines.append(self.format_message(item, indent + 1))
                else:
                    lines.append(f"{indent_str}[{i}]: {item}")
        else:
            lines.append(f"{indent_str}{message}")

        return "\n".join(lines)

    def stop_all_streams(self):
        """Stop all active echo streams"""
        for topic_name in list(self.echo_active.keys()):
            self.stop_echo_stream(topic_name)

        logging.info("All echo streams stopped")
