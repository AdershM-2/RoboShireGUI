"""
Package Repository Manager for RoboShire

Fetches and manages ROS2 package information from rosdistro.
Provides search, filtering, and caching capabilities.
"""

import json
import logging
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Optional, Dict, Set
from enum import Enum

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False
    logging.warning("requests library not available - package repository will use cached data only")

import yaml


class PackageCategory(Enum):
    """Package categories for filtering"""
    ALL = "All"
    NAVIGATION = "Navigation"
    VISION = "Vision"
    CONTROL = "Control"
    SENSORS = "Sensors"
    MANIPULATION = "Manipulation"
    SIMULATION = "Simulation"
    COMMUNICATION = "Communication"
    TOOLS = "Tools"
    DRIVERS = "Drivers"
    OTHER = "Other"


@dataclass
class PackageInfo:
    """Information about a ROS2 package"""
    name: str
    description: str = ""
    version: str = ""
    license: str = ""
    maintainer: str = ""
    repository_url: str = ""
    dependencies: List[str] = field(default_factory=list)
    category: PackageCategory = PackageCategory.OTHER
    popularity: int = 0  # 0-5 stars

    def matches_search(self, query: str) -> bool:
        """Check if package matches search query"""
        query_lower = query.lower()
        return (
            query_lower in self.name.lower() or
            query_lower in self.description.lower() or
            query_lower in self.category.value.lower()
        )

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization"""
        data = asdict(self)
        data['category'] = self.category.value
        return data

    @classmethod
    def from_dict(cls, data: dict) -> 'PackageInfo':
        """Create from dictionary"""
        if 'category' in data and isinstance(data['category'], str):
            data['category'] = PackageCategory(data['category'])
        return cls(**data)


class PackageRepository:
    """
    Manages ROS2 package information from rosdistro

    Features:
    - Fetches package list from rosdistro repository
    - Caches data locally for offline use
    - Provides search and filtering
    - Categorizes packages automatically
    """

    # rosdistro URL template
    ROSDISTRO_URL_TEMPLATE = "https://raw.githubusercontent.com/ros/rosdistro/master/{distro}/distribution.yaml"

    # Cache settings
    CACHE_DIR = Path.home() / ".roboshire" / "cache"
    CACHE_DURATION = timedelta(days=7)  # Refresh weekly

    # Category keywords for auto-categorization
    CATEGORY_KEYWORDS = {
        PackageCategory.NAVIGATION: ['nav', 'navigation', 'slam', 'amcl', 'map', 'localization', 'path'],
        PackageCategory.VISION: ['vision', 'image', 'camera', 'opencv', 'cv', 'perception', 'detection'],
        PackageCategory.CONTROL: ['control', 'controller', 'moveit', 'trajectory', 'motion', 'planning'],
        PackageCategory.SENSORS: ['sensor', 'imu', 'lidar', 'laser', 'gps', 'ultrasonic'],
        PackageCategory.MANIPULATION: ['manipulation', 'grasp', 'pick', 'gripper', 'arm'],
        PackageCategory.SIMULATION: ['gazebo', 'simulation', 'sim', 'mujoco', 'simulator'],
        PackageCategory.COMMUNICATION: ['bridge', 'comm', 'serial', 'can', 'mqtt', 'socket'],
        PackageCategory.TOOLS: ['rqt', 'tool', 'rviz', 'visualiz', 'plot', 'monitor'],
        PackageCategory.DRIVERS: ['driver', 'hardware', 'device'],
    }

    # Popularity mapping (based on common packages)
    POPULAR_PACKAGES = {
        'nav2': 5, 'navigation2': 5, 'slam_toolbox': 5,
        'cv_bridge': 5, 'image_transport': 5,
        'moveit': 5, 'ros2_control': 5,
        'gazebo': 5, 'rviz2': 5,
        'tf2': 5, 'rclpy': 5, 'rclcpp': 5,
    }

    def __init__(self, ros_distro: str = "humble"):
        """
        Initialize package repository

        Args:
            ros_distro: ROS2 distribution name (e.g., 'humble', 'iron')
        """
        self.ros_distro = ros_distro
        self.logger = logging.getLogger(__name__)

        # Ensure cache directory exists
        self.CACHE_DIR.mkdir(parents=True, exist_ok=True)

        # Cache file path
        self.cache_file = self.CACHE_DIR / f"packages_{ros_distro}.json"

        # In-memory cache
        self._packages: Dict[str, PackageInfo] = {}
        self._cache_loaded = False
        self._last_update: Optional[datetime] = None

    def get_packages(self, force_refresh: bool = False) -> List[PackageInfo]:
        """
        Get all packages, using cache if available

        Args:
            force_refresh: Force fetch from internet even if cache is valid

        Returns:
            List of PackageInfo objects
        """
        # Check if cache is valid
        if not force_refresh and self._is_cache_valid():
            if not self._cache_loaded:
                self._load_from_cache()
            return list(self._packages.values())

        # Fetch from internet
        try:
            self.logger.info(f"Fetching package list from rosdistro ({self.ros_distro})...")
            packages = self._fetch_from_rosdistro()

            # Update cache
            self._packages = {pkg.name: pkg for pkg in packages}
            self._save_to_cache()
            self._cache_loaded = True
            self._last_update = datetime.now()

            self.logger.info(f"Fetched {len(packages)} packages from rosdistro")
            return packages

        except Exception as e:
            self.logger.error(f"Failed to fetch from rosdistro: {e}")

            # Fallback to cache
            if self.cache_file.exists():
                self.logger.info("Using cached package list")
                self._load_from_cache()
                return list(self._packages.values())

            # No cache available
            self.logger.warning("No cache available - returning empty list")
            return []

    def search_packages(self, query: str, category: Optional[PackageCategory] = None) -> List[PackageInfo]:
        """
        Search packages by query and optionally filter by category

        Args:
            query: Search string (searches name and description)
            category: Optional category filter

        Returns:
            List of matching PackageInfo objects
        """
        packages = self.get_packages()

        # Filter by category
        if category and category != PackageCategory.ALL:
            packages = [pkg for pkg in packages if pkg.category == category]

        # Search by query
        if query.strip():
            packages = [pkg for pkg in packages if pkg.matches_search(query)]

        # Sort by popularity (5 stars first), then alphabetically
        packages.sort(key=lambda p: (-p.popularity, p.name.lower()))

        return packages

    def get_package_details(self, package_name: str) -> Optional[PackageInfo]:
        """
        Get detailed information about a specific package

        Args:
            package_name: Name of the package

        Returns:
            PackageInfo object or None if not found
        """
        packages = self.get_packages()
        return self._packages.get(package_name)

    def get_categories(self) -> List[PackageCategory]:
        """Get all available categories"""
        return list(PackageCategory)

    def _is_cache_valid(self) -> bool:
        """Check if cache file exists and is recent enough"""
        if not self.cache_file.exists():
            return False

        # Check modification time
        mtime = datetime.fromtimestamp(self.cache_file.stat().st_mtime)
        age = datetime.now() - mtime

        return age < self.CACHE_DURATION

    def _load_from_cache(self):
        """Load packages from cache file"""
        try:
            with open(self.cache_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            self._packages = {
                name: PackageInfo.from_dict(pkg_data)
                for name, pkg_data in data['packages'].items()
            }
            self._last_update = datetime.fromisoformat(data.get('last_update', datetime.now().isoformat()))
            self._cache_loaded = True

            self.logger.info(f"Loaded {len(self._packages)} packages from cache")

        except Exception as e:
            self.logger.error(f"Failed to load cache: {e}")
            self._packages = {}

    def _save_to_cache(self):
        """Save packages to cache file"""
        try:
            data = {
                'last_update': datetime.now().isoformat(),
                'ros_distro': self.ros_distro,
                'packages': {
                    name: pkg.to_dict()
                    for name, pkg in self._packages.items()
                }
            }

            with open(self.cache_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)

            self.logger.info(f"Saved {len(self._packages)} packages to cache")

        except Exception as e:
            self.logger.error(f"Failed to save cache: {e}")

    def _fetch_from_rosdistro(self) -> List[PackageInfo]:
        """
        Fetch package list from rosdistro repository

        Returns:
            List of PackageInfo objects
        """
        if not REQUESTS_AVAILABLE:
            raise RuntimeError("requests library not available - cannot fetch from internet")

        # Build URL
        url = self.ROSDISTRO_URL_TEMPLATE.format(distro=self.ros_distro)

        # Fetch YAML
        response = requests.get(url, timeout=30)
        response.raise_for_status()

        # Parse YAML
        data = yaml.safe_load(response.text)

        # Extract packages
        packages = []
        repositories = data.get('repositories', {})

        for repo_name, repo_info in repositories.items():
            # Get release info
            release = repo_info.get('release', {})
            if not release:
                continue

            # Get package list
            repo_packages = release.get('packages', [repo_name])

            # Get common info
            version = release.get('version', '').split('-')[0]  # Remove debian revision
            repo_url = release.get('url', '')

            # Create PackageInfo for each package
            for pkg_name in repo_packages:
                pkg_info = PackageInfo(
                    name=pkg_name,
                    description=self._generate_description(pkg_name, repo_name),
                    version=version,
                    license="",  # Not available in distribution.yaml
                    maintainer="",  # Not available in distribution.yaml
                    repository_url=repo_url,
                    dependencies=[],  # Would need to fetch package.xml for this
                    category=self._categorize_package(pkg_name),
                    popularity=self._get_popularity(pkg_name)
                )
                packages.append(pkg_info)

        return packages

    def _categorize_package(self, package_name: str) -> PackageCategory:
        """
        Auto-categorize package based on name

        Args:
            package_name: Name of the package

        Returns:
            PackageCategory
        """
        name_lower = package_name.lower().replace('_', ' ').replace('-', ' ')

        # Check each category's keywords
        for category, keywords in self.CATEGORY_KEYWORDS.items():
            if any(keyword in name_lower for keyword in keywords):
                return category

        return PackageCategory.OTHER

    def _generate_description(self, package_name: str, repo_name: str) -> str:
        """
        Generate a description for package
        (In production, this would be fetched from package metadata)

        Args:
            package_name: Name of the package
            repo_name: Name of the repository

        Returns:
            Description string
        """
        # For now, generate based on name
        category = self._categorize_package(package_name)

        if package_name != repo_name:
            return f"Package from {repo_name} repository ({category.value})"
        else:
            return f"ROS2 {category.value.lower()} package"

    def _get_popularity(self, package_name: str) -> int:
        """
        Get popularity score for package (0-5 stars)

        Args:
            package_name: Name of the package

        Returns:
            Popularity score (0-5)
        """
        # Check if in popular packages list
        for popular_name, score in self.POPULAR_PACKAGES.items():
            if popular_name in package_name.lower():
                return score

        # Default popularity based on category
        category = self._categorize_package(package_name)
        if category in [PackageCategory.NAVIGATION, PackageCategory.VISION, PackageCategory.CONTROL]:
            return 3
        elif category == PackageCategory.TOOLS:
            return 4
        else:
            return 2

    def clear_cache(self):
        """Clear the local cache"""
        try:
            if self.cache_file.exists():
                self.cache_file.unlink()
                self.logger.info("Cache cleared")

            self._packages = {}
            self._cache_loaded = False
            self._last_update = None

        except Exception as e:
            self.logger.error(f"Failed to clear cache: {e}")

    def get_cache_info(self) -> Dict[str, any]:
        """
        Get information about cache status

        Returns:
            Dictionary with cache info
        """
        if not self.cache_file.exists():
            return {
                'exists': False,
                'package_count': 0,
                'last_update': None,
                'age_days': None,
            }

        mtime = datetime.fromtimestamp(self.cache_file.stat().st_mtime)
        age = datetime.now() - mtime

        return {
            'exists': True,
            'package_count': len(self._packages) if self._cache_loaded else 0,
            'last_update': mtime,
            'age_days': age.days,
            'is_valid': age < self.CACHE_DURATION,
        }


# Example usage
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO)

    # Create repository
    repo = PackageRepository("humble")

    # Get all packages
    print("Fetching packages...")
    packages = repo.get_packages()
    print(f"Found {len(packages)} packages")

    # Search for navigation packages
    print("\nSearching for 'navigation' packages:")
    nav_packages = repo.search_packages("navigation", PackageCategory.NAVIGATION)
    for pkg in nav_packages[:5]:
        print(f"  - {pkg.name}: {pkg.description} (★{pkg.popularity})")

    # Get cache info
    cache_info = repo.get_cache_info()
    print(f"\nCache info: {cache_info}")
