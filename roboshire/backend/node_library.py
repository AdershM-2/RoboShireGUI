"""
Node Library

Factory functions for creating common ROS2 node types with predefined
ports and properties.
"""

from roboshire.backend.node_graph import (
    Node, Port, NodeType, PortType, generate_id
)


# Common ROS2 message types
COMMON_MSG_TYPES = [
    "std_msgs/String",
    "std_msgs/Int32",
    "std_msgs/Float32",
    "std_msgs/Float64",
    "std_msgs/Bool",
    "geometry_msgs/Twist",
    "geometry_msgs/Pose",
    "geometry_msgs/PoseStamped",
    "sensor_msgs/LaserScan",
    "sensor_msgs/Image",
    "nav_msgs/Odometry",
]


def create_publisher_node(
    name: str = "publisher",
    topic: str = "/topic",
    msg_type: str = "std_msgs/String",
    rate: float = 1.0,
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Publisher node

    Args:
        name: Node name
        topic: ROS2 topic name
        msg_type: Message type (e.g., "std_msgs/String")
        rate: Publishing rate in Hz
        position: (x, y) position on canvas

    Returns:
        Node configured as a Publisher
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.PUBLISHER,
        position=position,
        properties={
            'topic': topic,
            'msg_type': msg_type,
            'rate': rate,
            'queue_size': 10
        }
    )

    # Add output port
    output_port = Port(
        id=generate_id("port"),
        name="output",
        port_type=PortType.OUTPUT,
        data_type=msg_type
    )
    node.add_port(output_port)

    return node


def create_subscriber_node(
    name: str = "subscriber",
    topic: str = "/topic",
    msg_type: str = "std_msgs/String",
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Subscriber node

    Args:
        name: Node name
        topic: ROS2 topic name
        msg_type: Message type (e.g., "std_msgs/String")
        position: (x, y) position on canvas

    Returns:
        Node configured as a Subscriber
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.SUBSCRIBER,
        position=position,
        properties={
            'topic': topic,
            'msg_type': msg_type,
            'queue_size': 10
        }
    )

    # Add input port
    input_port = Port(
        id=generate_id("port"),
        name="input",
        port_type=PortType.INPUT,
        data_type=msg_type
    )
    node.add_port(input_port)

    # Optional: Add output port for processed data
    output_port = Port(
        id=generate_id("port"),
        name="output",
        port_type=PortType.OUTPUT,
        data_type=msg_type
    )
    node.add_port(output_port)

    return node


def create_timer_node(
    name: str = "timer",
    period: float = 1.0,
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Timer node

    Executes a callback periodically.

    Args:
        name: Node name
        period: Timer period in seconds
        position: (x, y) position on canvas

    Returns:
        Node configured as a Timer
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.TIMER,
        position=position,
        properties={
            'period': period
        }
    )

    # Add output port for timer trigger signal
    trigger_port = Port(
        id=generate_id("port"),
        name="trigger",
        port_type=PortType.OUTPUT,
        data_type="trigger"  # Special type for timer events
    )
    node.add_port(trigger_port)

    return node


def create_service_server_node(
    name: str = "service_server",
    service_name: str = "/service",
    service_type: str = "std_srvs/Trigger",
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Service Server node

    Args:
        name: Node name
        service_name: ROS2 service name
        service_type: Service type (e.g., "std_srvs/Trigger")
        position: (x, y) position on canvas

    Returns:
        Node configured as a Service Server
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.SERVICE_SERVER,
        position=position,
        properties={
            'service': service_name,
            'service_type': service_type
        }
    )

    # Add input port for request
    request_port = Port(
        id=generate_id("port"),
        name="request",
        port_type=PortType.INPUT,
        data_type=f"{service_type}/Request"
    )
    node.add_port(request_port)

    # Add output port for response
    response_port = Port(
        id=generate_id("port"),
        name="response",
        port_type=PortType.OUTPUT,
        data_type=f"{service_type}/Response"
    )
    node.add_port(response_port)

    return node


def create_service_client_node(
    name: str = "service_client",
    service_name: str = "/service",
    service_type: str = "std_srvs/Trigger",
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Service Client node

    Args:
        name: Node name
        service_name: ROS2 service name
        service_type: Service type (e.g., "std_srvs/Trigger")
        position: (x, y) position on canvas

    Returns:
        Node configured as a Service Client
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.SERVICE_CLIENT,
        position=position,
        properties={
            'service': service_name,
            'service_type': service_type
        }
    )

    # Add output port for request
    request_port = Port(
        id=generate_id("port"),
        name="request",
        port_type=PortType.OUTPUT,
        data_type=f"{service_type}/Request"
    )
    node.add_port(request_port)

    # Add input port for response
    response_port = Port(
        id=generate_id("port"),
        name="response",
        port_type=PortType.INPUT,
        data_type=f"{service_type}/Response"
    )
    node.add_port(response_port)

    return node


def create_logic_node(
    name: str = "logic",
    code: str = "# Custom logic here\npass",
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Logic node (custom Python code)

    Args:
        name: Node name
        code: Python code to execute
        position: (x, y) position on canvas

    Returns:
        Node configured as a Logic node
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.LOGIC,
        position=position,
        properties={
            'code': code,
            'inputs': [],  # List of input variable names
            'outputs': []  # List of output variable names
        }
    )

    # Ports are added dynamically based on inputs/outputs

    return node


def create_parameter_node(
    name: str = "parameter",
    param_name: str = "param",
    param_type: str = "string",
    default_value: str = "",
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Parameter node (ROS2 parameter declaration)

    Args:
        name: Node name
        param_name: Parameter name
        param_type: Parameter type (string, int, double, bool)
        default_value: Default value
        position: (x, y) position on canvas

    Returns:
        Node configured as a Parameter node
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.PARAMETER,
        position=position,
        properties={
            'param_name': param_name,
            'param_type': param_type,
            'default_value': default_value,
            'description': ""
        }
    )

    # Add output port for parameter value
    param_port = Port(
        id=generate_id("port"),
        name="value",
        port_type=PortType.OUTPUT,
        data_type=f"param/{param_type}"
    )
    node.add_port(param_port)

    return node


def create_lifecycle_publisher_node(
    name: str = "lifecycle_publisher",
    topic: str = "/topic",
    msg_type: str = "std_msgs/String",
    rate: float = 1.0,
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Lifecycle Publisher node (Phase 6)

    Lifecycle nodes follow ROS2 lifecycle management with states:
    - Unconfigured: Initial state
    - Inactive: Configured but not yet active
    - Active: Fully operational
    - Finalized: Shutdown complete

    Args:
        name: Node name
        topic: ROS2 topic name
        msg_type: Message type (e.g., "std_msgs/String")
        rate: Publishing rate in Hz
        position: (x, y) position on canvas

    Returns:
        Node configured as a Lifecycle Publisher
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.LIFECYCLE,
        position=position,
        properties={
            'lifecycle_type': 'publisher',
            'topic': topic,
            'msg_type': msg_type,
            'rate': rate,
            'queue_size': 10,
            'auto_start': True,  # Auto-activate on configure
            'bond_timeout': 4.0  # Bond heartbeat timeout
        }
    )

    # Add output port
    output_port = Port(
        id=generate_id("port"),
        name="output",
        port_type=PortType.OUTPUT,
        data_type=msg_type
    )
    node.add_port(output_port)

    return node


def create_lifecycle_subscriber_node(
    name: str = "lifecycle_subscriber",
    topic: str = "/topic",
    msg_type: str = "std_msgs/String",
    position: tuple = (0, 0)
) -> Node:
    """
    Create a Lifecycle Subscriber node (Phase 6)

    Args:
        name: Node name
        topic: ROS2 topic name
        msg_type: Message type (e.g., "std_msgs/String")
        position: (x, y) position on canvas

    Returns:
        Node configured as a Lifecycle Subscriber
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.LIFECYCLE,
        position=position,
        properties={
            'lifecycle_type': 'subscriber',
            'topic': topic,
            'msg_type': msg_type,
            'queue_size': 10,
            'auto_start': True,
            'bond_timeout': 4.0
        }
    )

    # Add input port
    input_port = Port(
        id=generate_id("port"),
        name="input",
        port_type=PortType.INPUT,
        data_type=msg_type
    )
    node.add_port(input_port)

    # Optional: Add output port for processed data
    output_port = Port(
        id=generate_id("port"),
        name="output",
        port_type=PortType.OUTPUT,
        data_type=msg_type
    )
    node.add_port(output_port)

    return node


def create_action_server_node(
    name: str = "action_server",
    action_name: str = "/action",
    action_type: str = "example_interfaces/Fibonacci",
    position: tuple = (0, 0)
) -> Node:
    """
    Create an Action Server node (v1.0.0)

    Action servers handle long-running tasks with:
    - Goal: Request from client
    - Feedback: Periodic progress updates
    - Result: Final outcome

    Common use cases:
    - Navigation (nav2_msgs/NavigateToPose)
    - Motion planning (moveit_msgs/MoveGroup)
    - Long computations

    Args:
        name: Node name
        action_name: ROS2 action name
        action_type: Action type (e.g., "example_interfaces/Fibonacci")
        position: (x, y) position on canvas

    Returns:
        Node configured as an Action Server
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.ACTION_SERVER,
        position=position,
        properties={
            'action': action_name,
            'action_type': action_type,
            'result_timeout': 10.0,  # Timeout for result
            'feedback_period': 0.5   # How often to send feedback (seconds)
        }
    )

    # Add input port for goal
    goal_port = Port(
        id=generate_id("port"),
        name="goal",
        port_type=PortType.INPUT,
        data_type=f"{action_type}/Goal"
    )
    node.add_port(goal_port)

    # Add output port for feedback
    feedback_port = Port(
        id=generate_id("port"),
        name="feedback",
        port_type=PortType.OUTPUT,
        data_type=f"{action_type}/Feedback"
    )
    node.add_port(feedback_port)

    # Add output port for result
    result_port = Port(
        id=generate_id("port"),
        name="result",
        port_type=PortType.OUTPUT,
        data_type=f"{action_type}/Result"
    )
    node.add_port(result_port)

    return node


def create_action_client_node(
    name: str = "action_client",
    action_name: str = "/action",
    action_type: str = "example_interfaces/Fibonacci",
    position: tuple = (0, 0)
) -> Node:
    """
    Create an Action Client node (v1.0.0)

    Action clients send goals to action servers and handle:
    - Goal sending
    - Feedback monitoring
    - Result processing
    - Cancellation

    Args:
        name: Node name
        action_name: ROS2 action name
        action_type: Action type (e.g., "example_interfaces/Fibonacci")
        position: (x, y) position on canvas

    Returns:
        Node configured as an Action Client
    """
    node = Node(
        id=generate_id("node"),
        name=name,
        node_type=NodeType.ACTION_CLIENT,
        position=position,
        properties={
            'action': action_name,
            'action_type': action_type,
            'wait_for_server': True,  # Wait for server before sending goal
            'server_timeout': 5.0     # Timeout for waiting for server
        }
    )

    # Add output port for goal
    goal_port = Port(
        id=generate_id("port"),
        name="goal",
        port_type=PortType.OUTPUT,
        data_type=f"{action_type}/Goal"
    )
    node.add_port(goal_port)

    # Add input port for feedback
    feedback_port = Port(
        id=generate_id("port"),
        name="feedback",
        port_type=PortType.INPUT,
        data_type=f"{action_type}/Feedback"
    )
    node.add_port(feedback_port)

    # Add input port for result
    result_port = Port(
        id=generate_id("port"),
        name="result",
        port_type=PortType.INPUT,
        data_type=f"{action_type}/Result"
    )
    node.add_port(result_port)

    return node


# Node type registry for GUI menus
NODE_LIBRARY = {
    "Publisher": {
        "factory": create_publisher_node,
        "description": "Publishes messages to a ROS2 topic",
        "category": "Communication",
        "icon": "📤"
    },
    "Subscriber": {
        "factory": create_subscriber_node,
        "description": "Subscribes to messages from a ROS2 topic",
        "category": "Communication",
        "icon": "📥"
    },
    "Timer": {
        "factory": create_timer_node,
        "description": "Executes callback periodically",
        "category": "Control",
        "icon": "⏰"
    },
    "Service Server": {
        "factory": create_service_server_node,
        "description": "Provides a ROS2 service",
        "category": "Communication",
        "icon": "🔧"
    },
    "Service Client": {
        "factory": create_service_client_node,
        "description": "Calls a ROS2 service",
        "category": "Communication",
        "icon": "📞"
    },
    "Action Server": {
        "factory": create_action_server_node,
        "description": "Handles long-running tasks (Nav2, MoveIt, etc.)",
        "category": "Communication",
        "icon": "🎯"
    },
    "Action Client": {
        "factory": create_action_client_node,
        "description": "Sends goals to action servers",
        "category": "Communication",
        "icon": "🎬"
    },
    "Logic": {
        "factory": create_logic_node,
        "description": "Custom Python code execution",
        "category": "Logic",
        "icon": "⚙️"
    },
    "Parameter": {
        "factory": create_parameter_node,
        "description": "ROS2 parameter declaration",
        "category": "Configuration",
        "icon": "📝"
    },
    "Lifecycle Publisher": {
        "factory": create_lifecycle_publisher_node,
        "description": "Lifecycle-managed publisher (production-grade)",
        "category": "Lifecycle",
        "icon": "🔄"
    },
    "Lifecycle Subscriber": {
        "factory": create_lifecycle_subscriber_node,
        "description": "Lifecycle-managed subscriber (production-grade)",
        "category": "Lifecycle",
        "icon": "🔄"
    }
}


def get_node_library() -> dict:
    """
    Get the complete node library

    Returns:
        Dictionary of node types with metadata
    """
    return NODE_LIBRARY


def get_node_categories() -> list:
    """
    Get list of node categories

    Returns:
        List of unique category names
    """
    categories = set()
    for info in NODE_LIBRARY.values():
        categories.add(info['category'])
    return sorted(categories)


def create_node_by_type(node_type_name: str, **kwargs) -> Node:
    """
    Create a node by type name

    Args:
        node_type_name: Name from NODE_LIBRARY (e.g., "Publisher")
        **kwargs: Arguments to pass to factory function

    Returns:
        Node instance

    Raises:
        ValueError: If node type not found
    """
    if node_type_name not in NODE_LIBRARY:
        raise ValueError(f"Unknown node type: {node_type_name}")

    factory = NODE_LIBRARY[node_type_name]['factory']
    return factory(**kwargs)
