"""
Documentation Search Backend for RoboShire

Provides search functionality for ROS2 documentation, including:
- ROS2 concepts (nodes, topics, services, parameters, actions)
- Common packages (rclpy, rclcpp, std_msgs, geometry_msgs, etc.)
- Tools (colcon, ament, ros2cli)
- Best practices and patterns

Author: RoboShire Team
Phase: 12 (Discovery & Search)
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum
import json
from pathlib import Path


class DocCategory(Enum):
    """Documentation category"""
    CONCEPT = "concept"           # Core ROS2 concepts
    PACKAGE = "package"           # ROS2 packages
    TOOL = "tool"                 # Development tools
    MESSAGE = "message"           # Message types
    SERVICE = "service"           # Service types
    ACTION = "action"             # Action types
    TUTORIAL = "tutorial"         # Tutorials and guides
    API = "api"                   # API reference
    EXAMPLE = "example"           # Code examples
    TROUBLESHOOTING = "troubleshooting"  # Common issues


@dataclass
class DocEntry:
    """A documentation entry"""
    title: str
    category: DocCategory
    description: str
    url: str
    tags: List[str] = field(default_factory=list)
    keywords: List[str] = field(default_factory=list)
    package: Optional[str] = None
    related: List[str] = field(default_factory=list)
    code_example: Optional[str] = None

    def matches_search(self, query: str) -> bool:
        """Check if entry matches search query"""
        query_lower = query.lower()

        # Search in title
        if query_lower in self.title.lower():
            return True

        # Search in description
        if query_lower in self.description.lower():
            return True

        # Search in tags
        for tag in self.tags:
            if query_lower in tag.lower():
                return True

        # Search in keywords
        for keyword in self.keywords:
            if query_lower in keyword.lower():
                return True

        # Search in package name
        if self.package and query_lower in self.package.lower():
            return True

        return False

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            'title': self.title,
            'category': self.category.value,
            'description': self.description,
            'url': self.url,
            'tags': self.tags,
            'keywords': self.keywords,
            'package': self.package,
            'related': self.related,
            'code_example': self.code_example
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DocEntry':
        """Create from dictionary"""
        return cls(
            title=data['title'],
            category=DocCategory(data['category']),
            description=data['description'],
            url=data['url'],
            tags=data.get('tags', []),
            keywords=data.get('keywords', []),
            package=data.get('package'),
            related=data.get('related', []),
            code_example=data.get('code_example')
        )


class DocumentationSearch:
    """Documentation search engine for ROS2"""

    # ROS2 Documentation URLs
    ROS2_DOCS_BASE = "https://docs.ros.org/en/humble"
    ROS2_TUTORIALS_BASE = f"{ROS2_DOCS_BASE}/Tutorials"
    ROS2_CONCEPTS_BASE = f"{ROS2_DOCS_BASE}/Concepts"

    def __init__(self):
        """Initialize documentation search"""
        self.entries: List[DocEntry] = []
        self._load_builtin_docs()

    def _load_builtin_docs(self):
        """Load built-in documentation entries"""

        # Core Concepts
        self.entries.extend([
            DocEntry(
                title="ROS2 Nodes",
                category=DocCategory.CONCEPT,
                description="Nodes are fundamental building blocks in ROS2. Each node is an executable that uses ROS2 to communicate with other nodes.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Basic/About-Nodes.html",
                tags=["node", "basic", "fundamentals"],
                keywords=["node", "process", "executable", "rclpy.Node", "rclcpp::Node"],
                code_example="""import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()"""
            ),
            DocEntry(
                title="ROS2 Topics",
                category=DocCategory.CONCEPT,
                description="Topics are named buses over which nodes exchange messages. Topics use a publish-subscribe pattern.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Basic/About-Topics.html",
                tags=["topic", "publish", "subscribe", "message"],
                keywords=["topic", "publisher", "subscriber", "pub", "sub", "message"],
                code_example="""# Publisher
self.publisher = self.create_publisher(String, 'my_topic', 10)
self.publisher.publish(String(data='Hello'))

# Subscriber
self.subscription = self.create_subscription(
    String, 'my_topic', self.callback, 10)

def callback(self, msg):
    self.get_logger().info(f'Received: {msg.data}')"""
            ),
            DocEntry(
                title="ROS2 Services",
                category=DocCategory.CONCEPT,
                description="Services provide a request-reply pattern for synchronous communication between nodes.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Basic/About-Services.html",
                tags=["service", "request", "reply", "synchronous"],
                keywords=["service", "client", "server", "request", "response", "srv"],
                code_example="""# Service Server
self.service = self.create_service(
    AddTwoInts, 'add_two_ints', self.handle_service)

def handle_service(self, request, response):
    response.sum = request.a + request.b
    return response

# Service Client
client = self.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 1
request.b = 2
future = client.call_async(request)"""
            ),
            DocEntry(
                title="ROS2 Parameters",
                category=DocCategory.CONCEPT,
                description="Parameters are node configuration values that can be set at runtime and changed dynamically.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Basic/About-Parameters.html",
                tags=["parameter", "config", "dynamic"],
                keywords=["parameter", "param", "config", "declare_parameter", "get_parameter"],
                code_example="""# Declare parameter
self.declare_parameter('my_param', 'default_value')

# Get parameter
value = self.get_parameter('my_param').value

# Set parameter callback
self.add_on_set_parameters_callback(self.param_callback)

def param_callback(self, params):
    for param in params:
        self.get_logger().info(f'{param.name} = {param.value}')
    return SetParametersResult(successful=True)"""
            ),
            DocEntry(
                title="ROS2 Actions",
                category=DocCategory.CONCEPT,
                description="Actions provide a way to execute long-running tasks with feedback and the ability to cancel.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Basic/About-Actions.html",
                tags=["action", "feedback", "cancel", "long-running"],
                keywords=["action", "goal", "feedback", "result", "cancel"],
                code_example="""# Action Client
action_client = ActionClient(self, Fibonacci, 'fibonacci')
goal = Fibonacci.Goal()
goal.order = 10

send_goal_future = action_client.send_goal_async(
    goal, feedback_callback=self.feedback_callback)

def feedback_callback(self, feedback_msg):
    self.get_logger().info(f'Progress: {feedback_msg.feedback.sequence}')"""
            ),
            DocEntry(
                title="Launch Files",
                category=DocCategory.CONCEPT,
                description="Launch files allow you to start multiple nodes with a single command and configure them.",
                url=f"{self.ROS2_TUTORIALS_BASE}/Intermediate/Launch/Creating-Launch-Files.html",
                tags=["launch", "startup", "configuration"],
                keywords=["launch", "launch.py", "LaunchDescription", "Node", "ros2 launch"],
                code_example="""from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{'param1': 'value1'}],
            remappings=[('topic1', 'renamed_topic1')]
        )
    ])"""
            ),
            DocEntry(
                title="QoS (Quality of Service)",
                category=DocCategory.CONCEPT,
                description="QoS policies control how messages are delivered, including reliability, durability, and history.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Intermediate/About-Quality-of-Service-Settings.html",
                tags=["qos", "reliability", "durability", "history"],
                keywords=["qos", "reliability", "durability", "history", "liveliness", "deadline"],
                code_example="""from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

publisher = self.create_publisher(String, 'topic', qos)"""
            ),
            DocEntry(
                title="Lifecycle Nodes",
                category=DocCategory.CONCEPT,
                description="Lifecycle nodes provide managed states (unconfigured, inactive, active, finalized) for controlled startup and shutdown.",
                url=f"{self.ROS2_TUTORIALS_BASE}/Demos/Lifecycle-State-Management.html",
                tags=["lifecycle", "managed", "state", "startup"],
                keywords=["lifecycle", "managed node", "configure", "activate", "deactivate", "cleanup"],
                code_example="""from rclpy.lifecycle import LifecycleNode, LifecycleState

class MyLifecycleNode(LifecycleNode):
    def on_configure(self, state: LifecycleState):
        self.get_logger().info('Configuring...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('Activating...')
        return TransitionCallbackReturn.SUCCESS"""
            ),
        ])

        # Common Packages
        self.entries.extend([
            DocEntry(
                title="rclpy - Python Client Library",
                category=DocCategory.PACKAGE,
                description="Python client library for ROS2, providing core functionality for creating nodes, publishers, subscribers, etc.",
                url=f"{self.ROS2_DOCS_BASE}/p/rclpy/index.html",
                tags=["python", "client", "library", "api"],
                keywords=["rclpy", "python", "node", "publisher", "subscriber", "client library"],
                package="rclpy"
            ),
            DocEntry(
                title="std_msgs - Standard Messages",
                category=DocCategory.MESSAGE,
                description="Common message types like String, Int32, Float64, Bool, Header, etc.",
                url=f"{self.ROS2_DOCS_BASE}/p/std_msgs/index.html",
                tags=["message", "standard", "common"],
                keywords=["std_msgs", "String", "Int32", "Float64", "Bool", "Header"],
                package="std_msgs"
            ),
            DocEntry(
                title="geometry_msgs - Geometry Messages",
                category=DocCategory.MESSAGE,
                description="Messages for geometric primitives like Point, Pose, Twist, Transform, etc.",
                url=f"{self.ROS2_DOCS_BASE}/p/geometry_msgs/index.html",
                tags=["message", "geometry", "pose", "transform"],
                keywords=["geometry_msgs", "Point", "Pose", "Twist", "Transform", "Quaternion"],
                package="geometry_msgs"
            ),
            DocEntry(
                title="sensor_msgs - Sensor Messages",
                category=DocCategory.MESSAGE,
                description="Messages for sensor data like Image, LaserScan, PointCloud2, Imu, CameraInfo, etc.",
                url=f"{self.ROS2_DOCS_BASE}/p/sensor_msgs/index.html",
                tags=["message", "sensor", "image", "lidar"],
                keywords=["sensor_msgs", "Image", "LaserScan", "PointCloud2", "Imu", "CameraInfo"],
                package="sensor_msgs"
            ),
            DocEntry(
                title="nav_msgs - Navigation Messages",
                category=DocCategory.MESSAGE,
                description="Messages for navigation including Odometry, Path, MapMetaData, OccupancyGrid, etc.",
                url=f"{self.ROS2_DOCS_BASE}/p/nav_msgs/index.html",
                tags=["message", "navigation", "odometry", "path"],
                keywords=["nav_msgs", "Odometry", "Path", "OccupancyGrid", "MapMetaData"],
                package="nav_msgs"
            ),
        ])

        # Tools
        self.entries.extend([
            DocEntry(
                title="colcon - Build Tool",
                category=DocCategory.TOOL,
                description="Build tool for ROS2 packages, supporting multiple build types (ament_python, ament_cmake, etc.)",
                url="https://colcon.readthedocs.io/",
                tags=["build", "tool", "compile"],
                keywords=["colcon", "build", "colcon build", "compile", "workspace"],
                code_example="""# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with symlink install
colcon build --symlink-install

# Clean build
rm -rf build install log && colcon build"""
            ),
            DocEntry(
                title="ros2 CLI - Command Line Tools",
                category=DocCategory.TOOL,
                description="Command line tools for interacting with ROS2: node, topic, service, param, action, etc.",
                url=f"{self.ROS2_TUTORIALS_BASE}/Beginner-CLI-Tools.html",
                tags=["cli", "command", "tool"],
                keywords=["ros2", "cli", "node list", "topic echo", "service call", "param get"],
                code_example="""# List nodes
ros2 node list

# Echo topic
ros2 topic echo /my_topic

# Call service
ros2 service call /my_service std_srvs/srv/Trigger

# Get parameter
ros2 param get /my_node my_param

# Run node
ros2 run my_package my_node"""
            ),
            DocEntry(
                title="rviz2 - 3D Visualization",
                category=DocCategory.TOOL,
                description="3D visualization tool for ROS2, displaying robot models, sensor data, and transforms.",
                url=f"{self.ROS2_DOCS_BASE}/Tutorials/Intermediate/RViz/RViz-User-Guide.html",
                tags=["visualization", "3d", "rviz"],
                keywords=["rviz", "rviz2", "visualization", "3d", "urdf", "robot model", "tf"],
                code_example="""# Launch rviz2
ros2 run rviz2 rviz2

# Launch with config
ros2 run rviz2 rviz2 -d my_config.rviz

# View URDF
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf"""
            ),
            DocEntry(
                title="rqt - Qt-based GUI Tools",
                category=DocCategory.TOOL,
                description="Collection of Qt-based GUI tools for ROS2: rqt_graph, rqt_console, rqt_plot, etc.",
                url=f"{self.ROS2_DOCS_BASE}/Concepts/Intermediate/About-RQt.html",
                tags=["gui", "tool", "rqt", "debug"],
                keywords=["rqt", "rqt_graph", "rqt_console", "rqt_plot", "gui", "visualization"],
                code_example="""# Node graph
ros2 run rqt_graph rqt_graph

# Console (logs)
ros2 run rqt_console rqt_console

# Plot data
ros2 run rqt_plot rqt_plot

# Main rqt
rqt"""
            ),
        ])

        # Tutorials
        self.entries.extend([
            DocEntry(
                title="Creating a Package",
                category=DocCategory.TUTORIAL,
                description="Step-by-step tutorial for creating a new ROS2 package with dependencies and setup files.",
                url=f"{self.ROS2_TUTORIALS_BASE}/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html",
                tags=["tutorial", "beginner", "package"],
                keywords=["create package", "ros2 pkg create", "package.xml", "setup.py"],
                code_example="""# Python package
ros2 pkg create --build-type ament_python my_package --dependencies rclpy std_msgs

# C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package --dependencies rclcpp std_msgs"""
            ),
            DocEntry(
                title="Writing a Publisher and Subscriber",
                category=DocCategory.TUTORIAL,
                description="Tutorial for creating publisher and subscriber nodes in Python and C++.",
                url=f"{self.ROS2_TUTORIALS_BASE}/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html",
                tags=["tutorial", "beginner", "publisher", "subscriber"],
                keywords=["publisher", "subscriber", "topic", "tutorial", "pub/sub"],
            ),
            DocEntry(
                title="Creating Custom Messages",
                category=DocCategory.TUTORIAL,
                description="Tutorial for defining custom message, service, and action types.",
                url=f"{self.ROS2_TUTORIALS_BASE}/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html",
                tags=["tutorial", "intermediate", "message", "interface"],
                keywords=["custom message", "msg", "srv", "action", "interface"],
                code_example="""# Message definition (.msg)
string name
int32 age
float64 height

# Service definition (.srv)
string question
---
string answer

# Build package
colcon build --packages-select my_interfaces"""
            ),
        ])

        # Troubleshooting
        self.entries.extend([
            DocEntry(
                title="Node Not Found",
                category=DocCategory.TROUBLESHOOTING,
                description="Common issue where 'ros2 node list' doesn't show your node. Check if node is running, DDS discovery, and network settings.",
                url=f"{self.ROS2_DOCS_BASE}/How-To-Guides/Installation-Troubleshooting.html",
                tags=["troubleshooting", "node", "discovery"],
                keywords=["node not found", "discovery", "dds", "ros2 node list", "network"],
                related=["ROS2 Nodes", "QoS (Quality of Service)"]
            ),
            DocEntry(
                title="Message Type Mismatch",
                category=DocCategory.TROUBLESHOOTING,
                description="Error when publisher and subscriber use different message types on the same topic.",
                url=f"{self.ROS2_CONCEPTS_BASE}/Basic/About-Topics.html",
                tags=["troubleshooting", "message", "type"],
                keywords=["message type", "mismatch", "incompatible types", "topic"],
                related=["ROS2 Topics", "std_msgs - Standard Messages"]
            ),
            DocEntry(
                title="Build Errors",
                category=DocCategory.TROUBLESHOOTING,
                description="Common colcon build errors: missing dependencies, CMake errors, Python setup issues.",
                url="https://colcon.readthedocs.io/en/released/user/troubleshooting.html",
                tags=["troubleshooting", "build", "colcon"],
                keywords=["build error", "colcon", "cmake", "dependency", "package not found"],
                related=["colcon - Build Tool", "Creating a Package"]
            ),
        ])

    def search(self, query: str, category: Optional[DocCategory] = None) -> List[DocEntry]:
        """
        Search documentation entries

        Args:
            query: Search query string
            category: Optional category filter

        Returns:
            List of matching documentation entries
        """
        if not query and not category:
            return self.entries

        results = []
        for entry in self.entries:
            # Filter by category if specified
            if category and entry.category != category:
                continue

            # Search by query if specified
            if query:
                if entry.matches_search(query):
                    results.append(entry)
            else:
                # No query, just category filter
                results.append(entry)

        return results

    def get_by_category(self, category: DocCategory) -> List[DocEntry]:
        """Get all entries in a category"""
        return [entry for entry in self.entries if entry.category == category]

    def get_categories(self) -> List[DocCategory]:
        """Get all available categories"""
        return list(DocCategory)

    def get_entry_count(self) -> int:
        """Get total number of documentation entries"""
        return len(self.entries)

    def add_custom_entry(self, entry: DocEntry):
        """Add a custom documentation entry"""
        self.entries.append(entry)

    def save_to_file(self, filepath: Path):
        """Save documentation entries to JSON file"""
        data = [entry.to_dict() for entry in self.entries]
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)

    def load_from_file(self, filepath: Path):
        """Load documentation entries from JSON file"""
        if not filepath.exists():
            return

        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        for item in data:
            entry = DocEntry.from_dict(item)
            self.entries.append(entry)
