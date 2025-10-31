"""
SSH Manager using subprocess - Alternative implementation using Windows OpenSSH

This implementation uses subprocess to call the system SSH client,
which avoids authentication issues and leverages existing SSH configuration.
"""

import subprocess
import logging
import threading
import queue
import time
from typing import Optional, Generator, Tuple, List
from dataclasses import dataclass


class SSHConnectionError(Exception):
    """Raised when SSH connection fails"""
    pass


class SSHAuthenticationError(Exception):
    """Raised when SSH authentication fails"""
    pass


class SSHTimeoutError(Exception):
    """Raised when SSH command times out"""
    pass


class SSHNetworkError(Exception):
    """Raised when network is unreachable"""
    pass


@dataclass
class CommandResult:
    """Result of a command execution"""
    exit_code: int
    stdout: str
    stderr: str
    duration: float  # seconds


class SSHManagerSubprocess:
    """
    SSH Manager using subprocess calls to system SSH client.

    This implementation is simpler and more reliable for Windows,
    as it uses the system's SSH client and existing authentication.
    """

    def __init__(
        self,
        host: str = "192.168.145.128",
        user: str = "adm20",
        port: int = 22,
        timeout: float = 30.0,
        max_retries: int = 3,
        retry_delay: float = 2.0
    ):
        """
        Initialize SSH manager

        Args:
            host: SSH host (IP or hostname)
            user: SSH username
            port: SSH port (default 22)
            timeout: Connection timeout in seconds
            max_retries: Maximum number of retry attempts for failed commands
            retry_delay: Delay between retries in seconds
        """
        self.host = host
        self.user = user
        self.port = port
        self.timeout = timeout
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.ssh_target = f"{user}@{host}"
        self.logger = logging.getLogger(__name__)

    def is_connected(self) -> Tuple[bool, str]:
        """
        Check if SSH connection is working with detailed error information

        Returns:
            Tuple of (is_connected, error_message)
        """
        try:
            result = subprocess.run(
                ['ssh', '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes', self.ssh_target, 'exit'],
                capture_output=True,
                timeout=10,
                text=True
            )

            if result.returncode == 0:
                return (True, "")

            # Parse error message
            error_msg = result.stderr.strip()

            if "Connection refused" in error_msg:
                return (False, f"Connection refused. Is SSH server running on {self.host}?")
            elif "Connection timed out" in error_msg or "No route to host" in error_msg:
                return (False, f"Network unreachable. Check if {self.host} is online and accessible.")
            elif "Permission denied" in error_msg:
                return (False, f"Authentication failed. Check username '{self.user}' and SSH keys/password.")
            elif "Host key verification failed" in error_msg:
                return (False, f"Host key verification failed. Run 'ssh {self.ssh_target}' manually to accept key.")
            else:
                return (False, f"SSH connection failed: {error_msg or 'Unknown error'}")

        except subprocess.TimeoutExpired:
            return (False, f"Connection timeout. {self.host} is not responding.")
        except FileNotFoundError:
            return (False, "SSH client not found. Install OpenSSH client on Windows.")
        except Exception as e:
            return (False, f"Unexpected error: {str(e)}")

    def test_connection(self) -> dict:
        """
        Test SSH connection and return detailed diagnostics

        Returns:
            Dictionary with connection status and diagnostics
        """
        diagnostics = {
            'connected': False,
            'error': None,
            'host_reachable': False,
            'ssh_service_running': False,
            'authentication_ok': False,
            'latency_ms': None
        }

        # Test 1: Ping host
        try:
            ping_result = subprocess.run(
                ['ping', '-n', '1', '-w', '1000', self.host],
                capture_output=True,
                timeout=5
            )
            diagnostics['host_reachable'] = (ping_result.returncode == 0)
        except:
            diagnostics['host_reachable'] = False

        # Test 2: Check SSH connection
        start_time = time.time()
        is_connected, error_msg = self.is_connected()
        latency = (time.time() - start_time) * 1000

        diagnostics['connected'] = is_connected
        diagnostics['latency_ms'] = latency if is_connected else None

        if is_connected:
            diagnostics['ssh_service_running'] = True
            diagnostics['authentication_ok'] = True
        else:
            diagnostics['error'] = error_msg

            # Determine which step failed
            if "Connection refused" in error_msg:
                diagnostics['ssh_service_running'] = False
            elif "Permission denied" in error_msg or "Authentication failed" in error_msg:
                diagnostics['ssh_service_running'] = True
                diagnostics['authentication_ok'] = False

        return diagnostics

    def execute_command(
        self,
        command: str,
        timeout: Optional[float] = None,
        working_dir: Optional[str] = None,
        retry: bool = True
    ) -> CommandResult:
        """
        Execute a command synchronously and wait for completion with retry logic

        Args:
            command: Command to execute
            timeout: Optional timeout in seconds
            working_dir: Optional working directory
            retry: Whether to retry on connection failures

        Returns:
            CommandResult with exit code and output

        Raises:
            SSHConnectionError: If SSH connection fails
            SSHTimeoutError: If command times out
            SSHNetworkError: If network is unreachable
        """
        # Prepend cd command if working_dir specified
        if working_dir:
            command = f"cd {working_dir} && {command}"

        # Build SSH command with connection options
        ssh_cmd = [
            'ssh',
            '-o', 'ConnectTimeout=10',
            '-o', 'ServerAliveInterval=30',
            '-o', 'ServerAliveCountMax=3',
            self.ssh_target,
            command
        ]

        last_exception = None
        max_attempts = self.max_retries if retry else 1

        for attempt in range(1, max_attempts + 1):
            start_time = time.time()

            try:
                result = subprocess.run(
                    ssh_cmd,
                    capture_output=True,
                    timeout=timeout or self.timeout,
                    text=True
                )

                duration = time.time() - start_time

                # Check for SSH-specific errors even if returncode is non-zero
                stderr_lower = result.stderr.lower()

                if "connection refused" in stderr_lower:
                    raise SSHConnectionError(f"Connection refused by {self.host}. Is SSH server running?")
                elif "connection timed out" in stderr_lower or "no route to host" in stderr_lower:
                    raise SSHNetworkError(f"Network unreachable. Cannot reach {self.host}.")
                elif "permission denied" in stderr_lower:
                    raise SSHAuthenticationError(f"Authentication failed for {self.user}@{self.host}")
                elif "host key verification failed" in stderr_lower:
                    raise SSHAuthenticationError(f"Host key verification failed for {self.host}")

                self.logger.debug(f"Command executed: {command} (exit={result.returncode}, duration={duration:.2f}s)")

                return CommandResult(
                    exit_code=result.returncode,
                    stdout=result.stdout,
                    stderr=result.stderr,
                    duration=duration
                )

            except subprocess.TimeoutExpired:
                duration = time.time() - start_time
                self.logger.error(f"Command timed out after {duration:.2f}s: {command}")
                last_exception = SSHTimeoutError(f"Command timed out after {timeout or self.timeout}s")

                if attempt < max_attempts:
                    self.logger.info(f"Retrying command (attempt {attempt + 1}/{max_attempts})...")
                    time.sleep(self.retry_delay)
                    continue
                raise last_exception

            except (SSHConnectionError, SSHNetworkError, SSHAuthenticationError) as e:
                last_exception = e

                if attempt < max_attempts and retry:
                    self.logger.warning(f"SSH error: {e}. Retrying (attempt {attempt + 1}/{max_attempts})...")
                    time.sleep(self.retry_delay)
                    continue
                raise

            except FileNotFoundError:
                raise SSHConnectionError("SSH client not found. Install OpenSSH client on Windows.")

            except Exception as e:
                self.logger.error(f"Failed to execute command: {e}")
                last_exception = SSHConnectionError(f"Unexpected error: {str(e)}")

                if attempt < max_attempts and retry:
                    self.logger.info(f"Retrying command (attempt {attempt + 1}/{max_attempts})...")
                    time.sleep(self.retry_delay)
                    continue
                raise last_exception

        # Should not reach here, but just in case
        if last_exception:
            raise last_exception
        raise SSHConnectionError("Command failed after all retry attempts")

    def execute_command_streaming(
        self,
        command: str,
        working_dir: Optional[str] = None,
        callback: Optional[callable] = None
    ) -> CommandResult:
        """
        Execute command and stream output in real-time

        Args:
            command: Command to execute
            working_dir: Optional working directory
            callback: Optional callback(line) for each output line

        Returns:
            CommandResult after command completes
        """
        # Prepend cd command if working_dir specified
        if working_dir:
            command = f"cd {working_dir} && {command}"

        # Build SSH command
        ssh_cmd = ['ssh', self.ssh_target, command]

        start_time = time.time()
        stdout_lines = []
        stderr_lines = []

        try:
            process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1  # Line buffered
            )

            # Read stdout in real-time
            for line in iter(process.stdout.readline, ''):
                if line:
                    line = line.rstrip('\n')
                    stdout_lines.append(line)
                    if callback:
                        callback(line, 'stdout')

            # Wait for process to complete
            process.wait()

            # Read remaining stderr
            stderr_text = process.stderr.read()
            if stderr_text:
                for line in stderr_text.splitlines():
                    stderr_lines.append(line)
                    if callback:
                        callback(line, 'stderr')

            duration = time.time() - start_time

            return CommandResult(
                exit_code=process.returncode,
                stdout='\n'.join(stdout_lines),
                stderr='\n'.join(stderr_lines),
                duration=duration
            )

        except Exception as e:
            logging.error(f"Error executing streaming command: {e}")
            raise

    def stream_command(
        self,
        command: str,
        working_dir: Optional[str] = None
    ) -> Generator[Tuple[str, str], None, None]:
        """
        Execute command and yield output lines as they arrive

        Args:
            command: Command to execute
            working_dir: Optional working directory

        Yields:
            Tuples of (line, stream) where stream is 'stdout' or 'stderr'
        """
        # Prepend cd command if working_dir specified
        if working_dir:
            command = f"cd {working_dir} && {command}"

        # Build SSH command
        ssh_cmd = ['ssh', self.ssh_target, command]

        try:
            process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            # Use threads to read stdout and stderr concurrently
            stdout_queue = queue.Queue()
            stderr_queue = queue.Queue()

            def read_stream(stream, q, stream_name):
                for line in iter(stream.readline, ''):
                    if line:
                        q.put((line.rstrip('\n'), stream_name))
                stream.close()

            stdout_thread = threading.Thread(
                target=read_stream,
                args=(process.stdout, stdout_queue, 'stdout'),
                daemon=True
            )
            stderr_thread = threading.Thread(
                target=read_stream,
                args=(process.stderr, stderr_queue, 'stderr'),
                daemon=True
            )

            stdout_thread.start()
            stderr_thread.start()

            # Yield lines as they arrive
            while process.poll() is None or not stdout_queue.empty() or not stderr_queue.empty():
                # Check stdout queue
                try:
                    line, stream = stdout_queue.get(timeout=0.1)
                    yield (line, stream)
                except queue.Empty:
                    pass

                # Check stderr queue
                try:
                    line, stream = stderr_queue.get(timeout=0.1)
                    yield (line, stream)
                except queue.Empty:
                    pass

            # Read remaining output
            stdout_thread.join(timeout=1.0)
            stderr_thread.join(timeout=1.0)

        except Exception as e:
            logging.error(f"Error streaming command: {e}")
            raise

    def kill_process(self, pid: int, force: bool = False) -> bool:
        """
        Kill a remote process by PID

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
            return result.exit_code == 0
        except Exception as e:
            logging.error(f"Failed to kill process {pid}: {e}")
            return False

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        pass  # Nothing to cleanup for subprocess implementation
