"""
SSH Manager - Centralized SSH connection and command execution

Provides SSH connectivity to Ubuntu VM for remote operations.
"""

import paramiko
import logging
import threading
import queue
from typing import Optional, Callable, Generator, Tuple, List
from dataclasses import dataclass
from pathlib import Path
import time


@dataclass
class CommandResult:
    """Result of a command execution"""
    exit_code: int
    stdout: str
    stderr: str
    duration: float  # seconds


@dataclass
class ProcessInfo:
    """Information about a running process"""
    pid: int
    name: str
    command: str
    cpu_percent: float = 0.0
    memory_mb: float = 0.0


class AsyncProcess:
    """Represents an asynchronous process running via SSH"""

    def __init__(self, ssh_client: paramiko.SSHClient, channel: paramiko.Channel, command: str):
        self.ssh_client = ssh_client
        self.channel = channel
        self.command = command
        self.pid: Optional[int] = None
        self.is_running = True
        self.stdout_queue = queue.Queue()
        self.stderr_queue = queue.Queue()

        # Start output reader threads
        self.stdout_thread = threading.Thread(
            target=self._read_output,
            args=(channel.makefile('r'), self.stdout_queue),
            daemon=True
        )
        self.stderr_thread = threading.Thread(
            target=self._read_output,
            args=(channel.makefile_stderr('r'), self.stderr_queue),
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
        """
        Wait for process to complete

        Args:
            timeout: Optional timeout in seconds

        Returns:
            Exit code
        """
        start_time = time.time()
        while self.is_running:
            if timeout and (time.time() - start_time) > timeout:
                raise TimeoutError(f"Process did not complete within {timeout} seconds")
            time.sleep(0.1)

        return self.channel.recv_exit_status()

    def terminate(self):
        """Terminate the process (SIGTERM)"""
        if self.pid:
            # Send SIGTERM via SSH
            try:
                stdin, stdout, stderr = self.ssh_client.exec_command(f"kill {self.pid}")
                stdin.close()
            except Exception as e:
                logging.error(f"Failed to terminate process {self.pid}: {e}")

    def kill(self):
        """Kill the process (SIGKILL)"""
        if self.pid:
            try:
                stdin, stdout, stderr = self.ssh_client.exec_command(f"kill -9 {self.pid}")
                stdin.close()
            except Exception as e:
                logging.error(f"Failed to kill process {self.pid}: {e}")


class SSHManager:
    """
    Manages SSH connections and command execution to Ubuntu VM.

    This class provides a centralized interface for all SSH operations,
    including synchronous commands, asynchronous processes, and output streaming.
    """

    def __init__(
        self,
        host: str = "192.168.145.128",
        user: str = "adm20",
        password: Optional[str] = None,
        key_path: Optional[str] = None,
        port: int = 22,
        timeout: float = 30.0,
        keepalive_interval: int = 60
    ):
        """
        Initialize SSH manager

        Args:
            host: SSH host (IP or hostname)
            user: SSH username
            password: SSH password (if not using key)
            key_path: Path to SSH private key file
            port: SSH port (default 22)
            timeout: Connection timeout in seconds
            keepalive_interval: Keepalive interval in seconds
        """
        self.host = host
        self.user = user
        self.password = password
        self.key_path = key_path
        self.port = port
        self.timeout = timeout
        self.keepalive_interval = keepalive_interval

        self.client: Optional[paramiko.SSHClient] = None
        self.is_connected_flag = False

        # Track async processes
        self.async_processes: List[AsyncProcess] = []

    def connect(self) -> bool:
        """
        Establish SSH connection to VM

        Returns:
            True if connected successfully, False otherwise
        """
        if self.is_connected_flag:
            logging.info("Already connected to SSH")
            return True

        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Connection parameters
            connect_kwargs = {
                'hostname': self.host,
                'port': self.port,
                'username': self.user,
                'timeout': self.timeout,
            }

            # Use key or password authentication
            if self.key_path:
                connect_kwargs['key_filename'] = self.key_path
            elif self.password:
                connect_kwargs['password'] = self.password
            else:
                # Try to use default SSH keys
                connect_kwargs['look_for_keys'] = True

            self.client.connect(**connect_kwargs)

            # Enable keepalive
            transport = self.client.get_transport()
            if transport:
                transport.set_keepalive(self.keepalive_interval)

            self.is_connected_flag = True
            logging.info(f"SSH connected to {self.user}@{self.host}")
            return True

        except paramiko.AuthenticationException:
            logging.error("SSH authentication failed")
            return False
        except paramiko.SSHException as e:
            logging.error(f"SSH connection failed: {e}")
            return False
        except Exception as e:
            logging.error(f"Unexpected error connecting to SSH: {e}")
            return False

    def disconnect(self):
        """Close SSH connection"""
        if self.client:
            try:
                self.client.close()
                logging.info("SSH disconnected")
            except Exception as e:
                logging.error(f"Error closing SSH connection: {e}")
            finally:
                self.client = None
                self.is_connected_flag = False

    def is_connected(self) -> bool:
        """
        Check if SSH connection is active

        Returns:
            True if connected, False otherwise
        """
        if not self.is_connected_flag or not self.client:
            return False

        try:
            # Test connection with a simple command
            transport = self.client.get_transport()
            if transport and transport.is_active():
                return True
        except Exception:
            pass

        self.is_connected_flag = False
        return False

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

        Raises:
            RuntimeError: If not connected to SSH
            TimeoutError: If command times out
        """
        if not self.is_connected():
            raise RuntimeError("Not connected to SSH. Call connect() first.")

        try:
            # Prepend cd command if working_dir specified
            if working_dir:
                command = f"cd {working_dir} && {command}"

            start_time = time.time()

            stdin, stdout, stderr = self.client.exec_command(
                command,
                timeout=timeout
            )

            # Read output
            stdout_text = stdout.read().decode('utf-8')
            stderr_text = stderr.read().decode('utf-8')
            exit_code = stdout.channel.recv_exit_status()

            duration = time.time() - start_time

            stdin.close()

            logging.debug(f"Command executed: {command} (exit={exit_code}, duration={duration:.2f}s)")

            return CommandResult(
                exit_code=exit_code,
                stdout=stdout_text,
                stderr=stderr_text,
                duration=duration
            )

        except paramiko.SSHException as e:
            logging.error(f"SSH command execution failed: {e}")
            raise
        except Exception as e:
            logging.error(f"Unexpected error executing command: {e}")
            raise

    def execute_command_async(
        self,
        command: str,
        working_dir: Optional[str] = None,
        get_pid: bool = True
    ) -> AsyncProcess:
        """
        Execute a command asynchronously (background process)

        Args:
            command: Command to execute
            working_dir: Optional working directory
            get_pid: Whether to capture process PID

        Returns:
            AsyncProcess object for monitoring

        Raises:
            RuntimeError: If not connected to SSH
        """
        if not self.is_connected():
            raise RuntimeError("Not connected to SSH. Call connect() first.")

        try:
            # Prepend cd command if working_dir specified
            if working_dir:
                command = f"cd {working_dir} && {command}"

            # If get_pid, wrap command to output PID
            if get_pid:
                # Start process in background and echo PID
                command = f"({command}) & echo $!"

            # Execute command with get_pty to enable job control
            transport = self.client.get_transport()
            channel = transport.open_session()
            channel.exec_command(command)

            async_process = AsyncProcess(self.client, channel, command)

            # If get_pid, read PID from first line of output
            if get_pid:
                try:
                    # Wait a bit for PID to be echoed
                    time.sleep(0.1)
                    stdout_lines, _ = async_process.get_output(timeout=0.5)
                    if stdout_lines:
                        async_process.pid = int(stdout_lines[0])
                        logging.info(f"Async process started with PID {async_process.pid}")
                except Exception as e:
                    logging.warning(f"Could not get PID: {e}")

            self.async_processes.append(async_process)
            logging.info(f"Async command started: {command}")

            return async_process

        except Exception as e:
            logging.error(f"Failed to start async command: {e}")
            raise

    def stream_command(
        self,
        command: str,
        working_dir: Optional[str] = None,
        buffer_size: int = 1024
    ) -> Generator[Tuple[str, str], None, None]:
        """
        Execute a command and stream output in real-time

        Args:
            command: Command to execute
            working_dir: Optional working directory
            buffer_size: Buffer size for reading output

        Yields:
            Tuples of (stdout_line, stderr_line) as they become available

        Raises:
            RuntimeError: If not connected to SSH
        """
        if not self.is_connected():
            raise RuntimeError("Not connected to SSH. Call connect() first.")

        try:
            # Prepend cd command if working_dir specified
            if working_dir:
                command = f"cd {working_dir} && {command}"

            stdin, stdout, stderr = self.client.exec_command(command)

            # Read output line by line
            while True:
                # Check if command finished
                if stdout.channel.exit_status_ready():
                    # Read remaining output
                    stdout_line = stdout.readline()
                    stderr_line = stderr.readline()
                    if stdout_line or stderr_line:
                        yield (stdout_line.rstrip('\n'), stderr_line.rstrip('\n'))
                    break

                # Read available output
                stdout_line = ""
                stderr_line = ""

                if stdout.channel.recv_ready():
                    stdout_line = stdout.readline().rstrip('\n')

                if stderr.channel.recv_stderr_ready():
                    stderr_line = stderr.readline().rstrip('\n')

                if stdout_line or stderr_line:
                    yield (stdout_line, stderr_line)

                time.sleep(0.05)  # Small delay to avoid busy waiting

            stdin.close()

        except Exception as e:
            logging.error(f"Error streaming command output: {e}")
            raise

    def kill_process(self, pid: int, force: bool = False) -> bool:
        """
        Kill a process by PID

        Args:
            pid: Process ID to kill
            force: If True, use SIGKILL (-9), otherwise SIGTERM

        Returns:
            True if successful, False otherwise
        """
        try:
            signal = "-9" if force else ""
            command = f"kill {signal} {pid}"
            result = self.execute_command(command, timeout=5.0)

            if result.exit_code == 0:
                logging.info(f"Process {pid} killed successfully")
                return True
            else:
                logging.warning(f"Failed to kill process {pid}: {result.stderr}")
                return False

        except Exception as e:
            logging.error(f"Error killing process {pid}: {e}")
            return False

    def list_processes(self, pattern: Optional[str] = None) -> List[ProcessInfo]:
        """
        List running processes

        Args:
            pattern: Optional pattern to filter processes (grep)

        Returns:
            List of ProcessInfo objects
        """
        try:
            command = "ps aux"
            if pattern:
                command += f" | grep '{pattern}' | grep -v grep"

            result = self.execute_command(command, timeout=10.0)

            if result.exit_code != 0:
                return []

            processes = []
            for line in result.stdout.splitlines()[1:]:  # Skip header
                parts = line.split(None, 10)  # Split on whitespace, max 11 parts
                if len(parts) >= 11:
                    processes.append(ProcessInfo(
                        pid=int(parts[1]),
                        name=parts[10].split()[0],  # First part of command
                        command=parts[10],
                        cpu_percent=float(parts[2]),
                        memory_mb=float(parts[3])
                    ))

            return processes

        except Exception as e:
            logging.error(f"Error listing processes: {e}")
            return []

    def cleanup_async_processes(self):
        """Clean up completed async processes"""
        self.async_processes = [p for p in self.async_processes if p.is_running]

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()

    def __del__(self):
        """Cleanup on deletion"""
        self.disconnect()
