"""
Node Graph Data Model

Pure Python data model for ROS2 node graphs.
No GUI dependencies - can be used for testing, serialization, and validation.
"""

from enum import Enum
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
import json
import uuid


class PortType(Enum):
    """Type of port (input or output)"""
    INPUT = "input"
    OUTPUT = "output"


class NodeType(Enum):
    """Type of ROS2 node"""
    PUBLISHER = "publisher"
    SUBSCRIBER = "subscriber"
    SERVICE_SERVER = "service_server"
    SERVICE_CLIENT = "service_client"
    ACTION_SERVER = "action_server"  # v1.0.0: Action server
    ACTION_CLIENT = "action_client"  # v1.0.0: Action client
    TIMER = "timer"
    LOGIC = "logic"
    PARAMETER = "parameter"
    TOPIC = "topic"  # v2.3.0: ROS2 topic (visualized as thin line)
    LIFECYCLE = "lifecycle"  # Phase 6: Lifecycle managed node


@dataclass
class Port:
    """Represents an input or output port on a node"""
    id: str
    name: str
    port_type: PortType
    data_type: str  # ROS2 message type like "std_msgs/String"
    connected_to: List[str] = field(default_factory=list)  # List of port IDs

    def can_connect_to(self, other: 'Port') -> Tuple[bool, str]:
        """
        Check if this port can connect to another port

        Returns:
            (can_connect, error_message)
        """
        # Can't connect to self
        if self.id == other.id:
            return False, "Cannot connect port to itself"

        # Must be opposite types
        if self.port_type == other.port_type:
            return False, f"Cannot connect {self.port_type.value} to {other.port_type.value}"

        # Check data type compatibility
        if not self._is_compatible_type(other.data_type):
            return False, f"Incompatible types: {self.data_type} and {other.data_type}"

        return True, ""

    def _is_compatible_type(self, other_type: str) -> bool:
        """Check if data types are compatible"""
        # Exact match
        if self.data_type == other_type:
            return True

        # TODO: Add more sophisticated type compatibility checking
        # For now, require exact match
        return False

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization"""
        return {
            'id': self.id,
            'name': self.name,
            'port_type': self.port_type.value,
            'data_type': self.data_type,
            'connected_to': self.connected_to
        }

    @staticmethod
    def from_dict(data: dict) -> 'Port':
        """Create Port from dictionary"""
        return Port(
            id=data['id'],
            name=data['name'],
            port_type=PortType(data['port_type']),
            data_type=data['data_type'],
            connected_to=data.get('connected_to', [])
        )


@dataclass
class Node:
    """Represents a ROS2 node in the graph"""
    id: str
    name: str
    node_type: NodeType
    position: Tuple[float, float] = (0.0, 0.0)
    ports: Dict[str, Port] = field(default_factory=dict)
    properties: Dict[str, Any] = field(default_factory=dict)

    def add_port(self, port: Port) -> None:
        """Add a port to this node"""
        self.ports[port.id] = port

    def remove_port(self, port_id: str) -> bool:
        """Remove a port from this node"""
        if port_id in self.ports:
            del self.ports[port_id]
            return True
        return False

    def get_port(self, port_id: str) -> Optional[Port]:
        """Get a port by ID"""
        return self.ports.get(port_id)

    def get_input_ports(self) -> List[Port]:
        """Get all input ports"""
        return [p for p in self.ports.values() if p.port_type == PortType.INPUT]

    def get_output_ports(self) -> List[Port]:
        """Get all output ports"""
        return [p for p in self.ports.values() if p.port_type == PortType.OUTPUT]

    def validate(self) -> Tuple[bool, List[str]]:
        """
        Validate this node

        Returns:
            (is_valid, error_messages)
        """
        errors = []

        # Check name is not empty
        if not self.name or not self.name.strip():
            errors.append(f"Node {self.id}: Name cannot be empty")

        # Check name is valid ROS2 name (alphanumeric + underscore)
        if self.name and not self._is_valid_ros2_name(self.name):
            errors.append(f"Node {self.id}: Invalid ROS2 name '{self.name}'")

        # Node-type specific validation
        if self.node_type == NodeType.PUBLISHER:
            if 'topic_name' not in self.properties or not self.properties['topic_name']:
                errors.append(f"Node {self.id}: Publisher must have a topic")

        elif self.node_type == NodeType.SUBSCRIBER:
            if 'topic_name' not in self.properties or not self.properties['topic_name']:
                errors.append(f"Node {self.id}: Subscriber must have a topic")

        elif self.node_type == NodeType.TIMER:
            if 'period' not in self.properties:
                errors.append(f"Node {self.id}: Timer must have a period")
            elif self.properties['period'] <= 0:
                errors.append(f"Node {self.id}: Timer period must be positive")

        return len(errors) == 0, errors

    def _is_valid_ros2_name(self, name: str) -> bool:
        """Check if name is valid for ROS2 (alphanumeric + underscore, no leading digit)"""
        if not name:
            return False
        if name[0].isdigit():
            return False
        return all(c.isalnum() or c == '_' for c in name)

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization"""
        return {
            'id': self.id,
            'name': self.name,
            'node_type': self.node_type.value,
            'position': list(self.position),
            'ports': {port_id: port.to_dict() for port_id, port in self.ports.items()},
            'properties': self.properties
        }

    @staticmethod
    def from_dict(data: dict) -> 'Node':
        """Create Node from dictionary"""
        node = Node(
            id=data['id'],
            name=data['name'],
            node_type=NodeType(data['node_type']),
            position=tuple(data.get('position', [0.0, 0.0])),
            properties=data.get('properties', {})
        )

        # Add ports
        for port_data in data.get('ports', {}).values():
            port = Port.from_dict(port_data)
            node.add_port(port)

        return node


@dataclass
class Connection:
    """Represents a connection between two ports"""
    id: str
    from_port_id: str  # Port ID (source)
    to_port_id: str    # Port ID (target)
    topic_name: str = ""  # ROS2 topic name (optional)

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization"""
        return {
            'id': self.id,
            'from_port_id': self.from_port_id,
            'to_port_id': self.to_port_id,
            'topic_name': self.topic_name
        }

    @staticmethod
    def from_dict(data: dict) -> 'Connection':
        """Create Connection from dictionary"""
        return Connection(
            id=data['id'],
            from_port_id=data['from_port_id'],
            to_port_id=data['to_port_id'],
            topic_name=data.get('topic_name', '')
        )


class NodeGraph:
    """Container for the entire node graph"""

    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self.connections: Dict[str, Connection] = {}

    def add_node(self, node: Node) -> bool:
        """
        Add a node to the graph

        Returns:
            True if added successfully, False if node ID already exists
        """
        if node.id in self.nodes:
            return False

        self.nodes[node.id] = node
        return True

    def remove_node(self, node_id: str) -> bool:
        """
        Remove a node from the graph

        Also removes all connections to/from this node.

        Returns:
            True if removed successfully, False if node not found
        """
        if node_id not in self.nodes:
            return False

        node = self.nodes[node_id]

        # Remove all connections involving this node's ports
        connections_to_remove = []
        for conn_id, conn in self.connections.items():
            # Get port IDs from this node
            node_port_ids = set(node.ports.keys())

            if conn.from_port_id in node_port_ids or conn.to_port_id in node_port_ids:
                connections_to_remove.append(conn_id)

        for conn_id in connections_to_remove:
            self.remove_connection(conn_id)

        # Remove the node
        del self.nodes[node_id]
        return True

    def get_node(self, node_id: str) -> Optional[Node]:
        """Get a node by ID"""
        return self.nodes.get(node_id)

    def add_connection(self, connection: Connection) -> Tuple[bool, str]:
        """
        Add a connection between two ports

        Returns:
            (success, error_message)
        """
        # Check if connection ID already exists
        if connection.id in self.connections:
            return False, "Connection ID already exists"

        # Find the ports
        from_port = self._find_port(connection.from_port_id)
        to_port = self._find_port(connection.to_port_id)

        if from_port is None:
            return False, f"Source port {connection.from_port_id} not found"
        if to_port is None:
            return False, f"Target port {connection.to_port_id} not found"

        # Check if ports can connect
        can_connect, error = from_port.can_connect_to(to_port)
        if not can_connect:
            return False, error

        # Add connection
        self.connections[connection.id] = connection

        # Update port connections
        from_port.connected_to.append(to_port.id)
        to_port.connected_to.append(from_port.id)

        return True, ""

    def remove_connection(self, connection_id: str) -> bool:
        """
        Remove a connection

        Returns:
            True if removed successfully, False if connection not found
        """
        if connection_id not in self.connections:
            return False

        connection = self.connections[connection_id]

        # Update port connections
        from_port = self._find_port(connection.from_port_id)
        to_port = self._find_port(connection.to_port_id)

        if from_port and to_port.id in from_port.connected_to:
            from_port.connected_to.remove(to_port.id)
        if to_port and from_port.id in to_port.connected_to:
            to_port.connected_to.remove(from_port.id)

        # Remove connection
        del self.connections[connection_id]
        return True

    def get_connection(self, connection_id: str) -> Optional[Connection]:
        """Get a connection by ID"""
        return self.connections.get(connection_id)

    def _find_port(self, port_id: str) -> Optional[Port]:
        """Find a port by ID across all nodes"""
        for node in self.nodes.values():
            port = node.get_port(port_id)
            if port:
                return port
        return None

    def _find_node_by_port(self, port_id: str) -> Optional[Node]:
        """Find the node that contains a given port"""
        for node in self.nodes.values():
            if port_id in node.ports:
                return node
        return None

    def validate(self) -> Tuple[bool, List[str]]:
        """
        Validate the entire graph

        Returns:
            (is_valid, error_messages)
        """
        errors = []

        # Validate each node
        for node in self.nodes.values():
            is_valid, node_errors = node.validate()
            errors.extend(node_errors)

        # Check for cycles (TODO: implement cycle detection)

        # Check that all connections are valid
        for connection in self.connections.values():
            from_port = self._find_port(connection.from_port_id)
            to_port = self._find_port(connection.to_port_id)

            if not from_port:
                errors.append(f"Connection {connection.id}: Source port not found")
            if not to_port:
                errors.append(f"Connection {connection.id}: Target port not found")

        # Check for duplicate node names
        node_names = [node.name for node in self.nodes.values()]
        duplicates = set([name for name in node_names if node_names.count(name) > 1])
        for dup in duplicates:
            errors.append(f"Duplicate node name: {dup}")

        return len(errors) == 0, errors

    def clear(self) -> None:
        """Clear all nodes and connections"""
        self.nodes.clear()
        self.connections.clear()

    def to_dict(self) -> dict:
        """
        Convert to dictionary for JSON serialization

        Returns:
            Dictionary representation of the graph
        """
        return {
            'nodes': {node_id: node.to_dict() for node_id, node in self.nodes.items()},
            'connections': {conn_id: conn.to_dict() for conn_id, conn in self.connections.items()}
        }

    def to_json(self, indent: int = 2) -> str:
        """Convert to JSON string"""
        return json.dumps(self.to_dict(), indent=indent)

    @staticmethod
    def from_dict(data: dict) -> 'NodeGraph':
        """
        Create NodeGraph from dictionary

        Args:
            data: Dictionary containing 'nodes' and 'connections'

        Returns:
            NodeGraph instance
        """
        graph = NodeGraph()

        # Add nodes
        for node_data in data.get('nodes', {}).values():
            node = Node.from_dict(node_data)
            graph.add_node(node)

        # Add connections
        for conn_data in data.get('connections', {}).values():
            connection = Connection.from_dict(conn_data)
            # Note: Validation happens in add_connection
            success, error = graph.add_connection(connection)
            if not success:
                print(f"Warning: Failed to add connection {connection.id}: {error}")

        return graph

    @staticmethod
    def from_json(json_str: str) -> 'NodeGraph':
        """Create NodeGraph from JSON string"""
        data = json.loads(json_str)
        return NodeGraph.from_dict(data)

    def get_stats(self) -> dict:
        """Get statistics about the graph"""
        return {
            'num_nodes': len(self.nodes),
            'num_connections': len(self.connections),
            'node_types': {
                node_type.value: sum(1 for n in self.nodes.values() if n.node_type == node_type)
                for node_type in NodeType
            }
        }


# Helper function to generate unique IDs
def generate_id(prefix: str = "") -> str:
    """Generate a unique ID with optional prefix"""
    unique_id = str(uuid.uuid4())[:8]
    return f"{prefix}_{unique_id}" if prefix else unique_id
