"""
SSH Manager with Password Support - Uses paramiko for password authentication

This module provides password-based SSH authentication using paramiko,
with automatic credential storage and retrieval.
"""

import logging
import paramiko
import time
from typing import Optional
from dataclasses import dataclass
from roboshire.backend.credential_manager import CredentialManager


@dataclass
class CommandResult:
    """Result of a command execution"""
    exit_code: int
    stdout: str
    stderr: str
    duration: float  # seconds


class SSHManagerPassword:
    """
    SSH Manager with password authentication support

    Features:
    - Password-based authentication
    - Automatic credential storage/retrieval
    - Persistent SSH connection
    - Command execution with timeout
    """

    def __init__(
        self,
        host: str = "192.168.145.128",
        user: str = "adm20",
        port: int = 22,
        password: Optional[str] = None,
        timeout: float = 30.0
    ):
        """
        Initialize SSH manager with password support

        Args:
            host: SSH host (IP or hostname)
            user: SSH username
            port: SSH port (default 22)
            password: SSH password (if None, will try to retrieve from credential manager)
            timeout: Connection timeout in seconds
        """
        self.host = host
        self.user = user
        self.port = port
        self.timeout = timeout
        self.ssh_target = f"{user}@{host}"

        # Credential manager
        self.cred_manager = CredentialManager()

        # SSH client
        self.client: Optional[paramiko.SSHClient] = None
        self.connected = False

        # Store or retrieve password
        if password:
            self.password = password
            # Store it for future use
            self.cred_manager.store_password(host, user, password)
        else:
            # Try to retrieve stored password
            self.password = self.cred_manager.get_password(host, user)

        logging.info(f"SSHManagerPassword initialized for {self.ssh_target}")

    def connect(self, password: Optional[str] = None) -> bool:
        """
        Establish SSH connection

        Args:
            password: Optional password (uses stored if not provided)

        Returns:
            True if connected successfully
        """
        if self.connected and self.client:
            return True

        # Use provided password or stored password
        pwd = password or self.password

        if not pwd:
            logging.error("No password available for SSH connection")
            return False

        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            logging.info(f"Connecting to {self.ssh_target}...")

            self.client.connect(
                hostname=self.host,
                port=self.port,
                username=self.user,
                password=pwd,
                timeout=self.timeout,
                look_for_keys=False,  # Don't use SSH keys, only password
                allow_agent=False
            )

            self.connected = True
            self.password = pwd  # Store password for reconnection

            # Save password in credential manager
            self.cred_manager.store_password(self.host, self.user, pwd)

            logging.info(f"Successfully connected to {self.ssh_target}")
            return True

        except paramiko.AuthenticationException:
            logging.error("Authentication failed - incorrect password")
            self.connected = False
            return False

        except paramiko.SSHException as e:
            logging.error(f"SSH connection failed: {e}")
            self.connected = False
            return False

        except Exception as e:
            logging.error(f"Failed to connect: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close SSH connection"""
        if self.client:
            self.client.close()
            self.connected = False
            logging.info("SSH connection closed")

    def is_connected(self) -> bool:
        """
        Check if SSH connection is active

        Returns:
            True if connected
        """
        if not self.connected or not self.client:
            return False

        try:
            # Test connection by executing a simple command
            transport = self.client.get_transport()
            if transport and transport.is_active():
                return True
            else:
                self.connected = False
                return False

        except Exception:
            self.connected = False
            return False

    def ensure_connected(self) -> bool:
        """
        Ensure connection is established, reconnect if needed

        Returns:
            True if connected
        """
        if self.is_connected():
            return True

        return self.connect()

    def execute_command(
        self,
        command: str,
        timeout: Optional[float] = None,
        working_dir: Optional[str] = None
    ) -> CommandResult:
        """
        Execute a command synchronously

        Args:
            command: Command to execute
            timeout: Optional timeout in seconds
            working_dir: Optional working directory

        Returns:
            CommandResult with exit code and output
        """
        if not self.ensure_connected():
            raise ConnectionError("SSH connection not established")

        # Prepend cd command if working_dir specified
        if working_dir:
            command = f"cd {working_dir} && {command}"

        start_time = time.time()
        cmd_timeout = timeout or self.timeout

        try:
            stdin, stdout, stderr = self.client.exec_command(
                command,
                timeout=cmd_timeout
            )

            # Read output
            stdout_text = stdout.read().decode('utf-8', errors='replace')
            stderr_text = stderr.read().decode('utf-8', errors='replace')
            exit_code = stdout.channel.recv_exit_status()

            duration = time.time() - start_time

            logging.debug(f"Command executed: {command[:100]} (exit={exit_code}, duration={duration:.2f}s)")

            return CommandResult(
                exit_code=exit_code,
                stdout=stdout_text,
                stderr=stderr_text,
                duration=duration
            )

        except paramiko.SSHException as e:
            duration = time.time() - start_time
            logging.error(f"SSH command failed after {duration:.2f}s: {e}")
            raise

        except Exception as e:
            duration = time.time() - start_time
            logging.error(f"Command execution failed after {duration:.2f}s: {e}")
            raise

    def set_password(self, password: str):
        """
        Set and store password

        Args:
            password: New password
        """
        self.password = password
        self.cred_manager.store_password(self.host, self.user, password)
        logging.info("Password updated")

    def clear_password(self):
        """Clear stored password"""
        self.password = None
        self.cred_manager.delete_password(self.host, self.user)
        logging.info("Password cleared")

    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()
