"""
Performance Profiler

Simple performance monitoring for ROS2 nodes via SSH.

Author: RoboShire Team
Phase: 6.5 (Performance Profiling)
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
import logging
import re


@dataclass
class NodePerformance:
    """Performance metrics for a node"""
    node_name: str
    cpu_percent: float = 0.0
    memory_mb: float = 0.0
    thread_count: int = 0
    status: str = "unknown"


@dataclass
class TopicPerformance:
    """Performance metrics for a topic"""
    topic_name: str
    message_rate: float = 0.0  # Hz
    bandwidth: float = 0.0  # KB/s
    publisher_count: int = 0
    subscriber_count: int = 0


class PerformanceProfiler:
    """
    Monitor ROS2 system performance via SSH

    Features:
    - Node CPU/memory usage
    - Topic bandwidth monitoring
    - Message rate tracking
    """

    def __init__(self, ssh_manager):
        """
        Initialize Performance Profiler

        Args:
            ssh_manager: SSHManagerSubprocess instance
        """
        self.ssh_manager = ssh_manager
        self.logger = logging.getLogger(__name__)

    def get_node_performance(self, node_name: str) -> Optional[NodePerformance]:
        """
        Get performance metrics for a specific node

        Args:
            node_name: ROS2 node name

        Returns:
            NodePerformance or None
        """
        try:
            # Use ps to get node process info
            # This is a simplified approach - finds process by name
            clean_name = node_name.split('/')[-1]  # Remove namespace

            cmd = f"ps aux | grep '{clean_name}' | grep -v grep | head -1"
            result = self.ssh_manager.execute_command(cmd)

            if not result:
                return NodePerformance(node_name=node_name, status="not_found")

            # Parse ps output: USER PID %CPU %MEM VSZ RSS TTY STAT START TIME COMMAND
            parts = result.split()
            if len(parts) >= 4:
                cpu = float(parts[2])
                mem_percent = float(parts[3])

                # Estimate memory in MB (rough approximation)
                # This would need total system memory for accuracy
                mem_mb = mem_percent * 16384 / 100  # Assume 16GB system

                return NodePerformance(
                    node_name=node_name,
                    cpu_percent=cpu,
                    memory_mb=mem_mb,
                    status="running"
                )

            return NodePerformance(node_name=node_name, status="error")

        except Exception as e:
            self.logger.error(f"Failed to get node performance: {e}")
            return None

    def get_topic_performance(self, topic_name: str) -> Optional[TopicPerformance]:
        """
        Get performance metrics for a topic

        Args:
            topic_name: Topic name

        Returns:
            TopicPerformance or None
        """
        try:
            # Get topic info
            cmd = f"ros2 topic info {topic_name} -v"
            result = self.ssh_manager.execute_command(cmd)

            if not result:
                return None

            # Parse publisher/subscriber counts
            pub_count = 0
            sub_count = 0

            for line in result.split('\n'):
                if "Publisher count:" in line:
                    match = re.search(r'(\d+)', line)
                    if match:
                        pub_count = int(match.group(1))
                elif "Subscription count:" in line:
                    match = re.search(r'(\d+)', line)
                    if match:
                        sub_count = int(match.group(1))

            # Get message rate using ros2 topic hz
            cmd = f"timeout 3 ros2 topic hz {topic_name} 2>/dev/null | grep 'average rate'"
            rate_result = self.ssh_manager.execute_command(cmd)

            message_rate = 0.0
            if rate_result:
                match = re.search(r'(\d+\.?\d*)', rate_result)
                if match:
                    message_rate = float(match.group(1))

            # Get bandwidth using ros2 topic bw
            cmd = f"timeout 3 ros2 topic bw {topic_name} 2>/dev/null | grep 'average'"
            bw_result = self.ssh_manager.execute_command(cmd)

            bandwidth = 0.0
            if bw_result:
                match = re.search(r'(\d+\.?\d*)', bw_result)
                if match:
                    bandwidth = float(match.group(1)) / 1024  # Convert to KB/s

            return TopicPerformance(
                topic_name=topic_name,
                message_rate=message_rate,
                bandwidth=bandwidth,
                publisher_count=pub_count,
                subscriber_count=sub_count
            )

        except Exception as e:
            self.logger.error(f"Failed to get topic performance: {e}")
            return None

    def get_system_summary(self) -> Dict[str, any]:
        """
        Get overall system performance summary

        Returns:
            Dictionary with system metrics
        """
        try:
            # Get total node count
            node_list = self.ssh_manager.execute_command("ros2 node list 2>/dev/null")
            node_count = len([n for n in node_list.split('\n') if n.strip()]) if node_list else 0

            # Get total topic count
            topic_list = self.ssh_manager.execute_command("ros2 topic list 2>/dev/null")
            topic_count = len([t for t in topic_list.split('\n') if t.strip()]) if topic_list else 0

            # Get system CPU and memory
            cpu_result = self.ssh_manager.execute_command("top -bn1 | grep 'Cpu(s)' | awk '{print $2}'")
            cpu_usage = float(cpu_result.strip().replace('%', '')) if cpu_result else 0.0

            mem_result = self.ssh_manager.execute_command("free -m | awk 'NR==2{printf \"%.1f\", $3*100/$2 }'")
            mem_usage = float(mem_result.strip()) if mem_result else 0.0

            return {
                "node_count": node_count,
                "topic_count": topic_count,
                "cpu_usage": cpu_usage,
                "memory_usage": mem_usage,
                "status": "ok"
            }

        except Exception as e:
            self.logger.error(f"Failed to get system summary: {e}")
            return {"status": "error", "error": str(e)}
