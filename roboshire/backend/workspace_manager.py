"""
Workspace Manager - ROS2 workspace operations

Handles workspace setup, package management, and build operations.

Updated for v2.3.0 Ubuntu Standalone - uses ExecutionManager for local/remote operations
"""

import logging
from typing import List, Optional, Tuple
from dataclasses import dataclass
from pathlib import Path

from roboshire.backend.execution_manager import ExecutionManager


@dataclass
class PackageInfo:
    """Information about a ROS2 package"""
    name: str
    path: str
    package_format: str = "unknown"  # 2 or 3
    build_type: str = "unknown"  # ament_python, ament_cmake, etc.
    dependencies: List[str] = None

    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []


@dataclass
class PackageStatus:
    """Build status of a package"""
    name: str
    exists: bool
    built: bool
    build_time: Optional[float] = None  # seconds
    install_dir: Optional[str] = None


class WorkspaceManager:
    """
    Manages ROS2 workspace operations.

    Handles workspace setup, package discovery, and build coordination.
    v2.3.0: Updated to support both local and remote execution via ExecutionManager.
    """

    def __init__(
        self,
        workspace_path: str,
        ros_distro: str = "humble"
    ):
        """
        Initialize workspace manager

        Args:
            workspace_path: Path to ROS2 workspace (local or remote)
            ros_distro: ROS2 distribution name (humble, iron, etc.)
        """
        self.execution_manager = ExecutionManager.get_instance()
        self.workspace_path = workspace_path
        self.ros_distro = ros_distro
        self.ros_setup_script = f"/opt/ros/{ros_distro}/setup.bash"

        # Legacy: Keep ssh reference for backward compatibility (will be None in local mode)
        try:
            executor = self.execution_manager.get_executor()
            if hasattr(executor, 'ssh'):
                self.ssh = executor.ssh
            else:
                self.ssh = None
        except:
            self.ssh = None

    def setup_workspace(self, create_if_missing: bool = True) -> bool:
        """
        Setup ROS2 workspace directory structure

        Args:
            create_if_missing: Create workspace if it doesn't exist

        Returns:
            True if workspace is ready, False otherwise
        """
        try:
            executor = self.execution_manager.get_executor()

            # Check if workspace exists
            result = executor.execute_command(f"test -d {self.workspace_path}")

            if result.exit_code != 0:
                if not create_if_missing:
                    logging.error(f"Workspace does not exist: {self.workspace_path}")
                    return False

                # Create workspace structure
                logging.info(f"Creating workspace: {self.workspace_path}")
                result = executor.execute_command(
                    f"mkdir -p {self.workspace_path}/src"
                )

                if result.exit_code != 0:
                    logging.error(f"Failed to create workspace: {result.stderr}")
                    return False

            # Verify src directory
            result = executor.execute_command(f"test -d {self.workspace_path}/src")
            if result.exit_code != 0:
                logging.error("Workspace missing src directory")
                return False

            logging.info(f"Workspace ready: {self.workspace_path}")
            return True

        except Exception as e:
            logging.error(f"Error setting up workspace: {e}")
            return False

    def source_workspace(self) -> str:
        """
        Get command to source ROS2 and workspace setup scripts

        Returns:
            Source command string
        """
        # Source ROS2
        source_cmd = f"source {self.ros_setup_script}"

        # Check if workspace is built (v2.3.0 - use ExecutionManager)
        install_setup = f"{self.workspace_path}/install/setup.bash"
        try:
            executor = self.execution_manager.get_executor()
            result = executor.execute_command(f"test -f {install_setup}")

            if result.exit_code == 0:
                # Workspace has been built, source it too
                source_cmd += f" && source {install_setup}"
        except Exception as e:
            # If executor not available or command fails, just source ROS2
            logging.debug(f"Could not check workspace install: {e}")

        return source_cmd

    def list_packages(self) -> List[PackageInfo]:
        """
        List all packages in workspace src directory

        Returns:
            List of PackageInfo objects
        """
        try:
            executor = self.execution_manager.get_executor()

            # Find all package.xml files
            result = executor.execute_command(
                f"find {self.workspace_path}/src -name 'package.xml' -type f"
            )

            if result.exit_code != 0:
                logging.warning("No packages found in workspace")
                return []

            packages = []
            for package_xml_path in result.stdout.strip().split('\n'):
                if not package_xml_path:
                    continue

                # Get package directory
                pkg_dir = str(Path(package_xml_path).parent)

                # Extract package name from package.xml
                result = executor.execute_command(
                    f"grep -oP '<name>\\K[^<]+' {package_xml_path} | head -1"
                )

                if result.exit_code == 0:
                    pkg_name = result.stdout.strip()

                    packages.append(PackageInfo(
                        name=pkg_name,
                        path=pkg_dir
                    ))

            logging.info(f"Found {len(packages)} packages in workspace")
            return packages

        except Exception as e:
            logging.error(f"Error listing packages: {e}")
            return []

    def get_package_status(self, package_name: str) -> PackageStatus:
        """
        Get build status of a package

        Args:
            package_name: Name of the package

        Returns:
            PackageStatus object
        """
        try:
            executor = self.execution_manager.get_executor()

            # Check if package exists in src
            result = executor.execute_command(
                f"find {self.workspace_path}/src -type d -name '{package_name}' | head -1"
            )

            pkg_exists = result.exit_code == 0 and bool(result.stdout.strip())

            if not pkg_exists:
                return PackageStatus(
                    name=package_name,
                    exists=False,
                    built=False
                )

            # Check if package is built
            install_dir = f"{self.workspace_path}/install/{package_name}"
            result = executor.execute_command(f"test -d {install_dir}")

            built = result.exit_code == 0

            return PackageStatus(
                name=package_name,
                exists=True,
                built=built,
                install_dir=install_dir if built else None
            )

        except Exception as e:
            logging.error(f"Error getting package status: {e}")
            return PackageStatus(
                name=package_name,
                exists=False,
                built=False
            )

    def clean(self, packages: Optional[List[str]] = None) -> bool:
        """
        Clean build artifacts

        Args:
            packages: Optional list of packages to clean (None = all)

        Returns:
            True if successful, False otherwise
        """
        try:
            executor = self.execution_manager.get_executor()

            if packages:
                logging.info(f"Cleaning packages: {', '.join(packages)}")
                # Clean specific packages
                for pkg in packages:
                    # Remove from build, install, and log
                    dirs_to_remove = [
                        f"{self.workspace_path}/build/{pkg}",
                        f"{self.workspace_path}/install/{pkg}",
                    ]

                    for dir_path in dirs_to_remove:
                        executor.execute_command(f"rm -rf {dir_path}")

            else:
                logging.info("Cleaning entire workspace")
                # Clean all build artifacts
                dirs_to_remove = [
                    f"{self.workspace_path}/build",
                    f"{self.workspace_path}/install",
                    f"{self.workspace_path}/log",
                ]

                for dir_path in dirs_to_remove:
                    result = executor.execute_command(f"rm -rf {dir_path}")
                    if result.exit_code != 0:
                        logging.warning(f"Failed to remove {dir_path}: {result.stderr}")

            logging.info("Clean completed")
            return True

        except Exception as e:
            logging.error(f"Error cleaning workspace: {e}")
            return False

    def check_dependencies(self, packages: Optional[List[str]] = None) -> Tuple[bool, List[str]]:
        """
        Check if package dependencies are installed

        Args:
            packages: Optional list of packages to check (None = all in workspace)

        Returns:
            Tuple of (all_satisfied, list_of_missing_dependencies)
        """
        try:
            executor = self.execution_manager.get_executor()

            # Build rosdep command
            if packages:
                pkg_paths = []
                for pkg in packages:
                    result = executor.execute_command(
                        f"find {self.workspace_path}/src -type d -name '{pkg}' | head -1"
                    )
                    if result.exit_code == 0 and result.stdout.strip():
                        pkg_paths.append(result.stdout.strip())

                if not pkg_paths:
                    return True, []

                paths_arg = " ".join(pkg_paths)
            else:
                paths_arg = f"{self.workspace_path}/src"

            # Run rosdep check
            source_cmd = self.source_workspace()
            result = executor.execute_command(
                f"{source_cmd} && rosdep check --from-paths {paths_arg} --ignore-src"
            )

            if result.exit_code == 0:
                logging.info("All dependencies satisfied")
                return True, []

            # Parse missing dependencies from output
            missing = []
            for line in result.stdout.split('\n'):
                if line.startswith('ERROR'):
                    # Extract package name from error line
                    # Format: "ERROR: Package 'foo' not found"
                    parts = line.split("'")
                    if len(parts) >= 2:
                        missing.append(parts[1])

            logging.warning(f"Missing dependencies: {', '.join(missing)}")
            return False, missing

        except Exception as e:
            logging.error(f"Error checking dependencies: {e}")
            return True, []  # Assume OK if check fails

    def install_dependencies(self, packages: Optional[List[str]] = None) -> bool:
        """
        Install missing dependencies using rosdep

        Args:
            packages: Optional list of packages (None = all in workspace)

        Returns:
            True if successful, False otherwise
        """
        try:
            executor = self.execution_manager.get_executor()
            logging.info("Installing dependencies with rosdep...")

            # Build paths argument
            if packages:
                pkg_paths = []
                for pkg in packages:
                    result = executor.execute_command(
                        f"find {self.workspace_path}/src -type d -name '{pkg}' | head -1"
                    )
                    if result.exit_code == 0 and result.stdout.strip():
                        pkg_paths.append(result.stdout.strip())

                paths_arg = " ".join(pkg_paths) if pkg_paths else f"{self.workspace_path}/src"
            else:
                paths_arg = f"{self.workspace_path}/src"

            # Run rosdep install
            source_cmd = self.source_workspace()
            result = executor.execute_command(
                f"{source_cmd} && "
                f"rosdep install --from-paths {paths_arg} --ignore-src -y",
                timeout=300.0  # 5 minutes for dependency installation
            )

            if result.exit_code == 0:
                logging.info("Dependencies installed successfully")
                return True
            else:
                logging.error(f"Failed to install dependencies: {result.stderr}")
                return False

        except Exception as e:
            logging.error(f"Error installing dependencies: {e}")
            return False

    def get_workspace_summary(self) -> str:
        """
        Get a summary of workspace status

        Returns:
            Summary string
        """
        try:
            packages = self.list_packages()
            built_packages = []

            for pkg in packages:
                status = self.get_package_status(pkg.name)
                if status.built:
                    built_packages.append(pkg.name)

            summary = f"""Workspace: {self.workspace_path}
ROS Distribution: {self.ros_distro}
Total Packages: {len(packages)}
Built Packages: {len(built_packages)}"""

            if packages:
                summary += "\n\nPackages:"
                for pkg in packages[:10]:  # Show first 10
                    status = self.get_package_status(pkg.name)
                    build_status = "[built]" if status.built else "[not built]"
                    summary += f"\n  - {pkg.name} {build_status}"

                if len(packages) > 10:
                    summary += f"\n  ... and {len(packages) - 10} more"

            return summary

        except Exception as e:
            return f"Error getting workspace summary: {e}"
