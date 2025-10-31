"""
Example Manager

Discovers, loads, and manages robot examples for RoboShire.

Examples provide users with ready-to-use robot templates including:
- URDF models
- Pre-configured node graphs
- Documentation
- Preview images
"""

from pathlib import Path
from typing import List, Dict, Optional
import json
import logging
from dataclasses import dataclass, field


@dataclass
class RobotExample:
    """Represents a robot example"""
    id: str
    name: str
    version: str
    category: str
    difficulty: str  # beginner, intermediate, advanced
    description: str
    author: str
    license: str
    tags: List[str] = field(default_factory=list)
    files: Dict[str, str] = field(default_factory=dict)
    features: List[str] = field(default_factory=list)
    requirements: Dict[str, List[str]] = field(default_factory=dict)
    nodes: List[Dict] = field(default_factory=list)
    use_cases: List[str] = field(default_factory=list)
    directory: Optional[Path] = None

    def __str__(self):
        return f"{self.name} ({self.category}, {self.difficulty})"


class ExampleManager:
    """
    Manages robot examples

    Features:
    - Discover examples in examples/ directory
    - Load example metadata
    - Load example files (URDF, node graph)
    - Validate example integrity
    - Filter and search examples
    """

    def __init__(self, examples_dir: Optional[Path] = None):
        """
        Initialize Example Manager

        Args:
            examples_dir: Directory containing examples
                         Default: roboshire/examples/
        """
        if examples_dir is None:
            # Default to roboshire/examples/
            module_dir = Path(__file__).parent.parent
            examples_dir = module_dir / "examples"

        self.examples_dir = Path(examples_dir)
        self.logger = logging.getLogger(__name__)
        self.examples: Dict[str, RobotExample] = {}

        # Discover examples on initialization
        if self.examples_dir.exists():
            self.discover_examples()
        else:
            self.logger.warning(f"Examples directory not found: {self.examples_dir}")

    def discover_examples(self) -> List[RobotExample]:
        """
        Discover all examples in examples directory

        Returns:
            List of discovered examples
        """
        self.examples.clear()

        if not self.examples_dir.exists():
            self.logger.warning(f"Examples directory not found: {self.examples_dir}")
            return []

        # Scan for example directories
        discovered_count = 0
        for example_dir in self.examples_dir.iterdir():
            if not example_dir.is_dir():
                continue

            # Ignore hidden directories and __pycache__
            if example_dir.name.startswith('.') or example_dir.name == '__pycache__':
                continue

            # Look for example.json
            metadata_file = example_dir / "example.json"
            if not metadata_file.exists():
                self.logger.debug(f"Skipping {example_dir.name} - no example.json")
                continue

            # Load example
            try:
                example = self._load_example(example_dir)
                self.examples[example.id] = example
                discovered_count += 1
                self.logger.info(f"Discovered example: {example.name}")
            except Exception as e:
                self.logger.error(f"Failed to load example from {example_dir}: {e}")

        self.logger.info(f"Discovered {discovered_count} example(s)")
        return list(self.examples.values())

    def _load_example(self, example_dir: Path) -> RobotExample:
        """
        Load example from directory

        Args:
            example_dir: Directory containing example

        Returns:
            RobotExample object

        Raises:
            FileNotFoundError: If example.json not found
            json.JSONDecodeError: If example.json is invalid
            KeyError: If required fields missing
        """
        metadata_file = example_dir / "example.json"

        with open(metadata_file, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # Create RobotExample with required fields
        example = RobotExample(
            id=data['id'],
            name=data['name'],
            version=data['version'],
            category=data['category'],
            difficulty=data['difficulty'],
            description=data['description'],
            author=data['author'],
            license=data['license'],
            tags=data.get('tags', []),
            files=data.get('files', {}),
            features=data.get('features', []),
            requirements=data.get('requirements', {}),
            nodes=data.get('nodes', []),
            use_cases=data.get('use_cases', []),
            directory=example_dir
        )

        return example

    def get_example(self, example_id: str) -> Optional[RobotExample]:
        """
        Get example by ID

        Args:
            example_id: Example identifier

        Returns:
            RobotExample if found, None otherwise
        """
        return self.examples.get(example_id)

    def get_all_examples(self) -> List[RobotExample]:
        """
        Get all examples

        Returns:
            List of all examples
        """
        return list(self.examples.values())

    def get_examples_by_category(self, category: str) -> List[RobotExample]:
        """
        Get examples by category

        Args:
            category: Category name (e.g., "Mobile Robots")

        Returns:
            List of examples in category
        """
        return [ex for ex in self.examples.values() if ex.category == category]

    def get_examples_by_difficulty(self, difficulty: str) -> List[RobotExample]:
        """
        Get examples by difficulty level

        Args:
            difficulty: Difficulty level (beginner, intermediate, advanced)

        Returns:
            List of examples at difficulty level
        """
        return [ex for ex in self.examples.values() if ex.difficulty == difficulty]

    def get_examples_by_tag(self, tag: str) -> List[RobotExample]:
        """
        Get examples by tag

        Args:
            tag: Tag to search for

        Returns:
            List of examples with tag
        """
        return [ex for ex in self.examples.values() if tag.lower() in [t.lower() for t in ex.tags]]

    def search_examples(self, query: str) -> List[RobotExample]:
        """
        Search examples by query

        Searches in name, description, tags, and use cases

        Args:
            query: Search query

        Returns:
            List of matching examples
        """
        query_lower = query.lower()
        results = []

        for example in self.examples.values():
            # Search in name
            if query_lower in example.name.lower():
                results.append(example)
                continue

            # Search in description
            if query_lower in example.description.lower():
                results.append(example)
                continue

            # Search in tags
            if any(query_lower in tag.lower() for tag in example.tags):
                results.append(example)
                continue

            # Search in use cases
            if any(query_lower in uc.lower() for uc in example.use_cases):
                results.append(example)
                continue

        return results

    def get_categories(self) -> List[str]:
        """
        Get all unique categories

        Returns:
            Sorted list of category names
        """
        categories = set(ex.category for ex in self.examples.values())
        return sorted(categories)

    def get_tags(self) -> List[str]:
        """
        Get all unique tags

        Returns:
            Sorted list of tags
        """
        tags = set()
        for example in self.examples.values():
            tags.update(example.tags)
        return sorted(tags)

    def load_example_urdf(self, example_id: str) -> Optional[str]:
        """
        Load URDF content from example

        Args:
            example_id: Example identifier

        Returns:
            URDF content as string, or None if not found
        """
        example = self.get_example(example_id)
        if not example or not example.directory:
            self.logger.error(f"Example not found: {example_id}")
            return None

        urdf_filename = example.files.get('urdf')
        if not urdf_filename:
            self.logger.error(f"No URDF file specified for example: {example_id}")
            return None

        urdf_file = example.directory / urdf_filename
        if not urdf_file.exists():
            self.logger.error(f"URDF file not found: {urdf_file}")
            return None

        try:
            return urdf_file.read_text(encoding='utf-8')
        except Exception as e:
            self.logger.error(f"Failed to read URDF file: {e}")
            return None

    def load_example_node_graph(self, example_id: str) -> Optional[Dict]:
        """
        Load node graph from example

        Args:
            example_id: Example identifier

        Returns:
            Node graph as dictionary, or None if not found
        """
        example = self.get_example(example_id)
        if not example or not example.directory:
            self.logger.error(f"Example not found: {example_id}")
            return None

        graph_filename = example.files.get('node_graph')
        if not graph_filename:
            self.logger.error(f"No node graph file specified for example: {example_id}")
            return None

        graph_file = example.directory / graph_filename
        if not graph_file.exists():
            self.logger.error(f"Node graph file not found: {graph_file}")
            return None

        try:
            with open(graph_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            self.logger.error(f"Failed to read node graph file: {e}")
            return None

    def get_example_preview_path(self, example_id: str) -> Optional[Path]:
        """
        Get path to example preview image

        Args:
            example_id: Example identifier

        Returns:
            Path to preview image, or None if not found
        """
        example = self.get_example(example_id)
        if not example or not example.directory:
            return None

        preview_filename = example.files.get('preview', 'robot.png')
        preview_file = example.directory / preview_filename

        return preview_file if preview_file.exists() else None

    def get_example_documentation_path(self, example_id: str) -> Optional[Path]:
        """
        Get path to example documentation

        Args:
            example_id: Example identifier

        Returns:
            Path to documentation file, or None if not found
        """
        example = self.get_example(example_id)
        if not example or not example.directory:
            return None

        doc_filename = example.files.get('documentation', 'README.md')
        doc_file = example.directory / doc_filename

        return doc_file if doc_file.exists() else None

    def load_example_documentation(self, example_id: str) -> Optional[str]:
        """
        Load documentation content from example

        Args:
            example_id: Example identifier

        Returns:
            Documentation content as string, or None if not found
        """
        doc_path = self.get_example_documentation_path(example_id)
        if not doc_path:
            return None

        try:
            return doc_path.read_text(encoding='utf-8')
        except Exception as e:
            self.logger.error(f"Failed to read documentation: {e}")
            return None

    def validate_example(self, example_id: str) -> tuple:
        """
        Validate example integrity

        Checks:
        - All required files exist
        - URDF is valid XML
        - Node graph is valid JSON
        - Preview image exists

        Args:
            example_id: Example identifier

        Returns:
            (is_valid, list_of_errors)
        """
        example = self.get_example(example_id)
        if not example:
            return False, [f"Example not found: {example_id}"]

        if not example.directory:
            return False, ["Example directory not set"]

        errors = []

        # Check required files exist
        required_files = ['urdf', 'node_graph']
        for file_type in required_files:
            filename = example.files.get(file_type)
            if not filename:
                errors.append(f"Missing {file_type} file specification")
                continue

            file_path = example.directory / filename
            if not file_path.exists():
                errors.append(f"Missing file: {filename} ({file_type})")

        # Validate URDF (basic XML check)
        urdf_content = self.load_example_urdf(example_id)
        if urdf_content:
            if '<robot' not in urdf_content:
                errors.append("URDF file doesn't contain <robot> tag")
        else:
            errors.append("Could not load URDF content")

        # Validate node graph (basic JSON check)
        node_graph = self.load_example_node_graph(example_id)
        if not node_graph:
            errors.append("Could not load node graph")
        elif not isinstance(node_graph, dict):
            errors.append("Node graph is not a dictionary")

        # Check preview image (optional)
        preview_path = self.get_example_preview_path(example_id)
        if not preview_path:
            # Preview is optional, just warn
            self.logger.warning(f"No preview image for example: {example_id}")

        is_valid = len(errors) == 0
        return is_valid, errors

    def get_example_count(self) -> int:
        """
        Get total number of examples

        Returns:
            Number of examples
        """
        return len(self.examples)

    def get_statistics(self) -> Dict:
        """
        Get statistics about examples

        Returns:
            Dictionary with statistics
        """
        total = len(self.examples)
        categories = {}
        difficulties = {'beginner': 0, 'intermediate': 0, 'advanced': 0}

        for example in self.examples.values():
            # Count by category
            categories[example.category] = categories.get(example.category, 0) + 1

            # Count by difficulty
            if example.difficulty in difficulties:
                difficulties[example.difficulty] += 1

        return {
            'total': total,
            'categories': categories,
            'difficulties': difficulties,
            'tags': len(self.get_tags())
        }
