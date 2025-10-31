"""
Execution Manager - Unified interface for local and remote ROS2 execution

Provides a transparent abstraction layer that allows RoboShire to execute
ROS2 commands either locally (subprocess) or remotely (SSH) without changing
the calling code.

This enables RoboShire to:
- Run natively on Ubuntu for development
- Deploy to remote devices (Jetson, Raspberry Pi) via SSH
- Switch between modes seamlessly through GUI settings

Author: RoboShire Team
Version: 2.3.0 (Ubuntu Standalone)
"""

import subprocess
import logging
import threading
import queue
import time
import os
import sys
from typing import Optional, Callable, Generator, Tuple, List
from dataclasses import dataclass
from enum import Enum
from abc import ABC, abstractmethod
from pathlib import Path


class ExecutionMode(Enum):
    """Execution mode for ROS2 commands"""
    LOCAL = "local"      # Run on local Ubuntu machine (subprocess)
    REMOTE = "remote"    # Run on remote device via SSH (Jetson/RPi/Ubuntu)


@dataclass
class CommandResult:
    """Result of a command execution"""
    exit_code: int
    stdout: str
    stderr: str
    duration: float  # seconds


class AsyncProcess:
    """Represents an asynchronous process (local or remote)"""

    def __init__(self, process_id: str, command: str):
        self.process_id = process_id
        self.command = command
        self.pid: Optional[int] = None
        self.is_running = True
        self.stdout_queue = queue.Queue()
        self.stderr_queue = queue.Queue()

    def get_output(self, timeout: float = 0.1) -> Tuple[List[str], List[str]]:
        """
        Get accumulated output from stdout and stderr

        Args:
            timeout: Timeout for reading from queues

        Returns:
            Tuple of (stdout_lines, stderr_lines)
        """
        stdout_lines = []
        stderr_lines = []

        # Read all available stdout
        try:
            while True:
                line = self.stdout_queue.get(timeout=timeout)
                stdout_lines.append(line)
        except queue.Empty:
            pass

        # Read all available stderr
        try:
            while True:
                line = self.stderr_queue.get(timeout=timeout)
                stderr_lines.append(line)
        except queue.Empty:
            pass

        return stdout_lines, stderr_lines

    def wait(self, timeout: Optional[float] = None) -> int:
        """Wait for process to complete"""
        start_time = time.time()
        while self.is_running:
            if timeout and (time.time() - start_time) > timeout:
                raise TimeoutError(f"Process did not complete within {timeout} seconds")
            time.sleep(0.1)
        return 0  # Will be overridden by subclasses

    @abstractmethod
    def terminate(self):
        """Terminate the process (SIGTERM)"""
        pass

    @abstractmethod
    def kill(self):
        """Kill the process (SIGKILL)"""
        pass


class LocalAsyncProcess(AsyncProcess):
    """Asynchronous process running locally via subprocess"""

    def __init__(self, process: subprocess.Popen, command: str):
        super().__init__(f"local_{process.pid}", command)
        self.process = process
        self.pid = process.pid

        # Start output reader threads
        self.stdout_thread = threading.Thread(
            target=self._read_output,
            args=(process.stdout, self.stdout_queue),
            daemon=True
        )
        self.stderr_thread = threading.Thread(
            target=self._read_output,
            args=(process.stderr, self.stderr_queue),
            daemon=True
        )
        self.stdout_thread.start()
        self.stderr_thread.start()

    def _read_output(self, stream, output_queue):
        """Read output from stream and put into queue"""
        try:
            for line in stream:
                output_queue.put(line.rstrip('\n'))
        except Exception as e:
            logging.error(f"Error reading output: {e}")
        finally:
            self.is_running = False

    def wait(self, timeout: Optional[float] = None) -> int:
        """Wait for process to complete"""
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            raise TimeoutError(f"Process did not complete within {timeout} seconds")

    def terminate(self):
        """Terminate the process (SIGTERM)"""
        try:
            self.process.terminate()
        except Exception as e:
            logging.error(f"Failed to terminate process {self.pid}: {e}")

    def kill(self):
        """Kill the process (SIGKILL)"""
        try:
            self.process.kill()
        except Exception as e:
            logging.error(f"Failed to kill process {self.pid}: {e}")


class BaseExecutor(ABC):
    """
    Abstract base class for command executors

    Provides a unified interface for executing commands either locally
    or remotely via SSH.
    """

    @abstractmethod
    def execute_command(
        self,
        command: str,
        timeout: Optional[float] = None,
        working_dir: Optional[str] = None
    ) -> CommandResult:
        """
        Execute a command synchronously and wait for completion

        Args:
            command: Command to execute
            timeout: Optional timeout in seconds
            working_dir: Optional working directory

        Returns:
            CommandResult with exit code and output
        """
        pass

    @abstractmethod
    def execute_command_async(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> AsyncProcess:
        """
        Execute a command asynchronously (background process)

        Args:
            command: Command to execute
            working_dir: Optional working directory

        Returns:
            AsyncProcess object for monitoring
        """
        pass

    @abstractmethod
    def stream_command(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> Generator[Tuple[str, str], None, None]:
        """
        Execute a command and stream output in real-time

        Args:
            command: Command to execute
            working_dir: Optional working directory

        Yields:
            Tuples of (stdout_line, stderr_line) as they become available
        """
        pass

    @abstractmethod
    def is_available(self) -> bool:
        """
        Check if executor is available and ready to use

        Returns:
            True if available, False otherwise
        """
        pass

    @abstractmethod
    def get_display_name(self) -> str:
        """Get human-readable name for this executor"""
        pass


class LocalExecutor(BaseExecutor):
    """
    Local executor - runs commands directly on the Ubuntu machine

    Uses Python subprocess module to execute commands locally.
    Ideal for development and testing on the same machine.
    """

    def __init__(self, ros_distro: str = "humble"):
        """
        Initialize local executor

        Args:
            ros_distro: ROS2 distribution name (humble, iron, jazzy, rolling)
        """
        self.ros_distro = ros_distro
        self.ros_setup_path = f"/opt/ros/{ros_distro}/setup.bash"

        logging.info(f"LocalExecutor initialized for ROS2 {ros_distro}")

    def _prepare_environment(self, working_dir: Optional[str] = None) -> dict:
        """
        Prepare environment variables for ROS2 commands

        Args:
            working_dir: Optional workspace directory to source

        Returns:
            Environment dictionary
        """
        env = os.environ.copy()

        # Source ROS2 setup
        if os.path.exists(self.ros_setup_path):
            # We can't directly source bash files in subprocess, so we need to
            # execute commands in bash with sourcing
            pass  # Handled in _build_command

        return env

    def _build_command(self, command: str, working_dir: Optional[str] = None) -> str:
        """
        Build command string with ROS2 environment sourcing

        Args:
            command: Command to execute
            working_dir: Optional working directory

        Returns:
            Full command string ready for bash execution
        """
        parts = []

        # Source ROS2
        parts.append(f"source {self.ros_setup_path}")

        # Source workspace if it exists
        if working_dir:
            workspace_setup = os.path.join(working_dir, "install", "setup.bash")
            if os.path.exists(workspace_setup):
                parts.append(f"source {workspace_setup}")

            # Add cd command
            parts.append(f"cd {working_dir}")

        # Add the actual command
        parts.append(command)

        return " && ".join(parts)

    def execute_command(
        self,
        command: str,
        timeout: Optional[float] = None,
        working_dir: Optional[str] = None
    ) -> CommandResult:
        """Execute a command synchronously"""
        try:
            full_command = self._build_command(command, working_dir)

            start_time = time.time()

            # Execute in bash shell
            process = subprocess.run(
                ["bash", "-c", full_command],
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=working_dir
            )

            duration = time.time() - start_time

            logging.debug(f"Local command executed: {command} (exit={process.returncode}, duration={duration:.2f}s)")

            return CommandResult(
                exit_code=process.returncode,
                stdout=process.stdout,
                stderr=process.stderr,
                duration=duration
            )

        except subprocess.TimeoutExpired as e:
            logging.error(f"Command timed out: {command}")
            raise TimeoutError(f"Command timed out after {timeout} seconds")
        except Exception as e:
            logging.error(f"Error executing local command: {e}")
            raise

    def execute_command_async(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> AsyncProcess:
        """Execute a command asynchronously"""
        try:
            full_command = self._build_command(command, working_dir)

            # Start process
            process = subprocess.Popen(
                ["bash", "-c", full_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=working_dir,
                bufsize=1  # Line buffered
            )

            logging.info(f"Local async command started: {command} (PID {process.pid})")

            return LocalAsyncProcess(process, command)

        except Exception as e:
            logging.error(f"Failed to start async command: {e}")
            raise

    def stream_command(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> Generator[Tuple[str, str], None, None]:
        """Execute a command and stream output in real-time"""
        try:
            full_command = self._build_command(command, working_dir)

            process = subprocess.Popen(
                ["bash", "-c", full_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=working_dir,
                bufsize=1
            )

            # Read output line by line
            while True:
                stdout_line = ""
                stderr_line = ""

                # Check if process finished
                if process.poll() is not None:
                    # Read remaining output
                    remaining_stdout = process.stdout.read()
                    remaining_stderr = process.stderr.read()
                    if remaining_stdout or remaining_stderr:
                        for line in remaining_stdout.splitlines():
                            yield (line, "")
                        for line in remaining_stderr.splitlines():
                            yield ("", line)
                    break

                # Read available output
                if process.stdout:
                    stdout_line = process.stdout.readline().rstrip('\n')
                if process.stderr:
                    stderr_line = process.stderr.readline().rstrip('\n')

                if stdout_line or stderr_line:
                    yield (stdout_line, stderr_line)

                time.sleep(0.05)  # Small delay to avoid busy waiting

        except Exception as e:
            logging.error(f"Error streaming local command output: {e}")
            raise

    def is_available(self) -> bool:
        """Check if local ROS2 installation is available"""
        return os.path.exists(self.ros_setup_path)

    def get_display_name(self) -> str:
        """Get human-readable name"""
        return f"🖥️ Local (ROS2 {self.ros_distro})"


class RemoteExecutor(BaseExecutor):
    """
    Remote executor - runs commands on remote device via SSH

    Uses existing SSHManager to execute commands on remote devices
    (Jetson, Raspberry Pi, remote Ubuntu servers).
    Ideal for deploying to embedded systems and production hardware.
    """

    def __init__(self, ssh_manager, workspace_path: str, ros_distro: str = "humble"):
        """
        Initialize remote executor

        Args:
            ssh_manager: SSHManager instance for remote connection
            workspace_path: Workspace path on remote system
            ros_distro: ROS2 distribution name on remote system
        """
        from roboshire.backend.ssh_manager import SSHManager

        self.ssh: SSHManager = ssh_manager
        self.workspace_path = workspace_path
        self.ros_distro = ros_distro

        logging.info(f"RemoteExecutor initialized for {ssh_manager.host}")

    def execute_command(
        self,
        command: str,
        timeout: Optional[float] = None,
        working_dir: Optional[str] = None
    ) -> CommandResult:
        """Execute a command synchronously via SSH"""
        if not working_dir:
            working_dir = self.workspace_path

        return self.ssh.execute_command(command, timeout=timeout, working_dir=working_dir)

    def execute_command_async(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> AsyncProcess:
        """Execute a command asynchronously via SSH"""
        if not working_dir:
            working_dir = self.workspace_path

        return self.ssh.execute_command_async(command, working_dir=working_dir, get_pid=True)

    def stream_command(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> Generator[Tuple[str, str], None, None]:
        """Execute a command and stream output via SSH"""
        if not working_dir:
            working_dir = self.workspace_path

        return self.ssh.stream_command(command, working_dir=working_dir)

    def is_available(self) -> bool:
        """Check if SSH connection is available"""
        return self.ssh.is_connected()

    def get_display_name(self) -> str:
        """Get human-readable name"""
        return f"🌐 Remote ({self.ssh.user}@{self.ssh.host})"


class ExecutionManager:
    """
    Singleton manager for execution mode

    Maintains the current executor (Local or Remote) and provides a
    unified interface for all ROS2 command execution in RoboShire.

    This allows transparent switching between local and remote execution
    without modifying calling code.
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._initialized = True
        self.current_mode = ExecutionMode.LOCAL
        self.local_executor: Optional[LocalExecutor] = None
        self.remote_executor: Optional[RemoteExecutor] = None
        self.current_executor: Optional[BaseExecutor] = None

        logging.info("ExecutionManager initialized")

    @classmethod
    def get_instance(cls) -> 'ExecutionManager':
        """Get singleton instance"""
        return cls()

    def initialize_local(self, ros_distro: str = "humble"):
        """
        Initialize local executor

        Args:
            ros_distro: ROS2 distribution name
        """
        self.local_executor = LocalExecutor(ros_distro=ros_distro)
        logging.info(f"Local executor initialized with ROS2 {ros_distro}")

    def initialize_remote(self, ssh_manager, workspace_path: str, ros_distro: str = "humble"):
        """
        Initialize remote executor

        Args:
            ssh_manager: SSHManager instance
            workspace_path: Workspace path on remote system
            ros_distro: ROS2 distribution name on remote system
        """
        self.remote_executor = RemoteExecutor(
            ssh_manager=ssh_manager,
            workspace_path=workspace_path,
            ros_distro=ros_distro
        )
        logging.info(f"Remote executor initialized for {ssh_manager.host}")

    def set_mode(self, mode: ExecutionMode) -> bool:
        """
        Set execution mode

        Args:
            mode: ExecutionMode (LOCAL or REMOTE)

        Returns:
            True if mode was set successfully, False otherwise
        """
        if mode == ExecutionMode.LOCAL:
            if not self.local_executor:
                logging.error("Local executor not initialized")
                return False

            if not self.local_executor.is_available():
                logging.error("Local ROS2 installation not found")
                return False

            self.current_executor = self.local_executor
            self.current_mode = ExecutionMode.LOCAL
            logging.info("Switched to LOCAL execution mode")
            return True

        elif mode == ExecutionMode.REMOTE:
            if not self.remote_executor:
                logging.error("Remote executor not initialized")
                return False

            if not self.remote_executor.is_available():
                logging.error("SSH connection not available")
                return False

            self.current_executor = self.remote_executor
            self.current_mode = ExecutionMode.REMOTE
            logging.info("Switched to REMOTE execution mode")
            return True

        return False

    def get_mode(self) -> ExecutionMode:
        """Get current execution mode"""
        return self.current_mode

    def get_executor(self) -> BaseExecutor:
        """
        Get current executor

        Returns:
            Current BaseExecutor instance

        Raises:
            RuntimeError: If no executor is initialized
        """
        if not self.current_executor:
            raise RuntimeError("No executor initialized. Call initialize_local() or initialize_remote() first.")

        return self.current_executor

    def get_display_name(self) -> str:
        """Get human-readable name for current execution mode"""
        if self.current_executor:
            return self.current_executor.get_display_name()
        return "⚠️ Not configured"

    def is_ready(self) -> bool:
        """Check if execution manager is ready to execute commands"""
        return self.current_executor is not None and self.current_executor.is_available()
