"""
Colcon Bridge - Integration with colcon build system

Handles building ROS2 packages using colcon, parsing build output,
and tracking build status.

Updated for v2.3.0 Ubuntu Standalone - uses ExecutionManager for local/remote builds
"""

import logging
import re
import time
from typing import List, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum

from roboshire.backend.workspace_manager import WorkspaceManager
from roboshire.backend.execution_manager import ExecutionManager


class BuildStatus(Enum):
    """Build status enum"""
    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    SUCCESS = "success"
    FAILED = "failed"
    ABORTED = "aborted"


@dataclass
class BuildOptions:
    """Options for colcon build"""
    symlink_install: bool = False  # VMware shared folders don't support symlinks
    continue_on_error: bool = False
    cmake_args: List[str] = field(default_factory=list)
    packages_select: List[str] = field(default_factory=list)
    packages_skip: List[str] = field(default_factory=list)
    parallel_workers: int = 4
    event_handlers: str = "console_direct+"  # Show output directly


@dataclass
class PackageBuildResult:
    """Result of building a single package"""
    package_name: str
    status: BuildStatus
    duration: float = 0.0  # seconds
    error_message: str = ""


@dataclass
class BuildResult:
    """Overall build result"""
    status: BuildStatus
    packages: List[PackageBuildResult] = field(default_factory=list)
    total_duration: float = 0.0
    stdout: str = ""
    stderr: str = ""
    exit_code: int = -1

    def get_failed_packages(self) -> List[str]:
        """Get list of failed package names"""
        return [p.package_name for p in self.packages if p.status == BuildStatus.FAILED]

    def get_successful_packages(self) -> List[str]:
        """Get list of successful package names"""
        return [p.package_name for p in self.packages if p.status == BuildStatus.SUCCESS]


class ColconBridge:
    """
    Bridge to colcon build system.

    Executes colcon build commands using ExecutionManager (local or remote).
    v2.3.0: Updated to support both local Ubuntu builds and remote SSH builds.
    """

    def __init__(self, workspace_manager: WorkspaceManager):
        """
        Initialize colcon bridge

        Args:
            workspace_manager: Workspace manager for workspace operations
        """
        self.workspace = workspace_manager
        self.execution_manager = ExecutionManager.get_instance()

        # Legacy: Keep reference to SSH manager for compatibility
        self.ssh = workspace_manager.ssh if hasattr(workspace_manager, 'ssh') else None

        # Regex patterns for parsing build output
        self.pattern_starting = re.compile(r"Starting\s+>>>\s+(\w+)")
        self.pattern_finished = re.compile(r"Finished\s+<<<\s+(\w+)\s+\[([0-9.]+)s\]")
        self.pattern_failed = re.compile(r"Failed\s+<<<\s+(\w+)\s+\[([0-9.]+)s\]")
        self.pattern_aborted = re.compile(r"Aborted\s+<<<\s+(\w+)")

    def build_package(
        self,
        package_name: str,
        options: Optional[BuildOptions] = None,
        output_callback: Optional[Callable[[str, str], None]] = None
    ) -> BuildResult:
        """
        Build a specific package

        Args:
            package_name: Name of package to build
            options: Build options
            output_callback: Optional callback(line, stream) for real-time output

        Returns:
            BuildResult with status and details
        """
        if options is None:
            options = BuildOptions()

        # Add package to packages_select
        options.packages_select = [package_name]

        return self.build_workspace(options, output_callback)

    def build_workspace(
        self,
        options: Optional[BuildOptions] = None,
        output_callback: Optional[Callable[[str, str], None]] = None
    ) -> BuildResult:
        """
        Build entire workspace or selected packages

        Args:
            options: Build options
            output_callback: Optional callback(line, stream) for real-time output

        Returns:
            BuildResult with status and details
        """
        if options is None:
            options = BuildOptions()

        try:
            # Build colcon command
            cmd = self._build_colcon_command(options)

            logging.info(f"Building workspace with command: {cmd}")

            # Execute build with streaming output
            start_time = time.time()
            build_result = BuildResult(status=BuildStatus.IN_PROGRESS)
            package_results = {}

            # Get current executor (local or remote)
            try:
                executor = self.execution_manager.get_executor()
            except RuntimeError as e:
                logging.error(f"No executor available: {e}")
                build_result.status = BuildStatus.FAILED
                build_result.duration = time.time() - start_time
                build_result.error_message = "Execution mode not configured. Please configure via Settings → Execution Mode."
                return build_result

            # Source ROS2 and workspace
            source_cmd = self.workspace.source_workspace()
            full_cmd = f"{source_cmd} && {cmd}"

            # Execute build command using executor
            # Note: For now, we'll use execute_command instead of streaming
            # TODO: Implement proper streaming in ExecutionManager
            logging.info(f"Executing build with {executor.get_display_name()}")

            result = executor.execute_command(
                full_cmd,
                working_dir=self.workspace.workspace_path,
                timeout=None  # No timeout for builds
            )

            # Process output line by line
            stdout_lines = result.stdout.splitlines()
            stderr_lines = result.stderr.splitlines()

            # Parse build output
            for line in stdout_lines:
                self._process_build_output(line, 'stdout', package_results, output_callback)
            for line in stderr_lines:
                self._process_build_output(line, 'stderr', package_results, output_callback)

            # Calculate total duration
            build_result.total_duration = result.duration
            build_result.exit_code = result.exit_code
            build_result.stdout = result.stdout
            build_result.stderr = result.stderr

            # Convert package_results to list
            build_result.packages = list(package_results.values())

            # Determine overall status
            if result.exit_code == 0:
                build_result.status = BuildStatus.SUCCESS
            else:
                build_result.status = BuildStatus.FAILED

            # Log summary
            if build_result.status == BuildStatus.SUCCESS:
                logging.info(f"Build succeeded in {build_result.total_duration:.1f}s")
            else:
                logging.error(f"Build failed after {build_result.total_duration:.1f}s")
                if build_result.get_failed_packages():
                    logging.error(f"Failed packages: {', '.join(build_result.get_failed_packages())}")

            return build_result

        except Exception as e:
            logging.error(f"Build error: {e}")
            return BuildResult(
                status=BuildStatus.FAILED,
                total_duration=time.time() - start_time if 'start_time' in locals() else 0.0,
                stderr=str(e)
            )

    def _build_colcon_command(self, options: BuildOptions) -> str:
        """
        Build colcon command string from options

        Args:
            options: Build options

        Returns:
            Command string
        """
        cmd_parts = ["colcon", "build"]

        # Symlink install
        if options.symlink_install:
            cmd_parts.append("--symlink-install")

        # Continue on error
        if options.continue_on_error:
            cmd_parts.append("--continue-on-error")

        # Event handlers (for output formatting)
        if options.event_handlers:
            cmd_parts.append(f"--event-handlers {options.event_handlers}")

        # Parallel workers
        if options.parallel_workers:
            cmd_parts.append(f"--parallel-workers {options.parallel_workers}")

        # Package selection
        if options.packages_select:
            cmd_parts.append(f"--packages-select {' '.join(options.packages_select)}")

        # Package skip
        if options.packages_skip:
            cmd_parts.append(f"--packages-skip {' '.join(options.packages_skip)}")

        # CMake args
        if options.cmake_args:
            cmake_args_str = " ".join(options.cmake_args)
            cmd_parts.append(f"--cmake-args {cmake_args_str}")

        return " ".join(cmd_parts)

    def _process_build_output(
        self,
        line: str,
        stream: str,
        package_results: dict,
        output_callback: Optional[Callable[[str, str], None]]
    ):
        """
        Process a line of build output

        Args:
            line: Output line
            stream: Stream name ('stdout' or 'stderr')
            package_results: Dict to update with package results
            output_callback: Optional callback for forwarding output
        """
        # Call output callback if provided
        if output_callback:
            output_callback(line, stream)

        # Parse build progress
        # Pattern: "Starting >>> package_name"
        match = self.pattern_starting.match(line)
        if match:
            pkg_name = match.group(1)
            package_results[pkg_name] = PackageBuildResult(
                package_name=pkg_name,
                status=BuildStatus.IN_PROGRESS
            )
            logging.info(f"Building {pkg_name}...")
            return

        # Pattern: "Finished <<< package_name [12.3s]"
        match = self.pattern_finished.match(line)
        if match:
            pkg_name = match.group(1)
            duration = float(match.group(2))
            if pkg_name in package_results:
                package_results[pkg_name].status = BuildStatus.SUCCESS
                package_results[pkg_name].duration = duration
            logging.info(f"Finished {pkg_name} in {duration:.1f}s")
            return

        # Pattern: "Failed <<< package_name [12.3s]"
        match = self.pattern_failed.match(line)
        if match:
            pkg_name = match.group(1)
            duration = float(match.group(2))
            if pkg_name in package_results:
                package_results[pkg_name].status = BuildStatus.FAILED
                package_results[pkg_name].duration = duration
            logging.error(f"Failed {pkg_name} after {duration:.1f}s")
            return

        # Pattern: "Aborted <<< package_name"
        match = self.pattern_aborted.match(line)
        if match:
            pkg_name = match.group(1)
            if pkg_name in package_results:
                package_results[pkg_name].status = BuildStatus.ABORTED
            logging.warning(f"Aborted {pkg_name}")
            return

    def clean_package(self, package_name: str) -> bool:
        """
        Clean build artifacts for a package

        Args:
            package_name: Name of package to clean

        Returns:
            True if successful
        """
        return self.workspace.clean(packages=[package_name])

    def clean_workspace(self) -> bool:
        """
        Clean all build artifacts in workspace

        Returns:
            True if successful
        """
        return self.workspace.clean()

    def get_build_status(self, package_name: str) -> BuildStatus:
        """
        Get current build status of a package

        Args:
            package_name: Package name

        Returns:
            BuildStatus enum
        """
        status = self.workspace.get_package_status(package_name)

        if not status.exists:
            return BuildStatus.NOT_STARTED

        if status.built:
            return BuildStatus.SUCCESS

        return BuildStatus.NOT_STARTED

    def list_buildable_packages(self) -> List[str]:
        """
        List all packages that can be built

        Returns:
            List of package names
        """
        packages = self.workspace.list_packages()
        return [pkg.name for pkg in packages]
