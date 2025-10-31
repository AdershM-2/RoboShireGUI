"""
Credential Manager - Securely store and retrieve SSH credentials

Uses Windows Credential Manager via keyring library for secure storage.
Falls back to encrypted local file if keyring is not available.
"""

import logging
import base64
import json
from pathlib import Path
from typing import Optional
from cryptography.fernet import Fernet
import getpass


class CredentialManager:
    """
    Manage SSH credentials securely

    Features:
    - Store password after first prompt
    - Retrieve password automatically
    - Delete stored credentials
    - Uses Windows Credential Manager when available
    - Falls back to encrypted file storage
    """

    def __init__(self, config_dir: Optional[Path] = None):
        """
        Initialize credential manager

        Args:
            config_dir: Directory for config files (default: user home)
        """
        if config_dir is None:
            config_dir = Path.home() / ".roboshire"

        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)

        self.cred_file = self.config_dir / "credentials.enc"
        self.key_file = self.config_dir / "key.bin"

        # Try to use keyring (Windows Credential Manager)
        self.use_keyring = False
        try:
            import keyring
            self.keyring = keyring
            self.use_keyring = True
            logging.info("Using Windows Credential Manager for password storage")
        except ImportError:
            logging.warning("keyring not available, using encrypted file storage")

        # Generate encryption key if needed
        if not self.use_keyring and not self.key_file.exists():
            self._generate_key()

        logging.info("CredentialManager initialized")

    def _generate_key(self):
        """Generate and save encryption key"""
        key = Fernet.generate_key()
        self.key_file.write_bytes(key)
        logging.info("Generated new encryption key")

    def _get_cipher(self) -> Fernet:
        """Get Fernet cipher for encryption/decryption"""
        if not self.key_file.exists():
            self._generate_key()

        key = self.key_file.read_bytes()
        return Fernet(key)

    def store_password(self, host: str, user: str, password: str) -> bool:
        """
        Store SSH password securely

        Args:
            host: SSH host
            user: SSH username
            password: SSH password

        Returns:
            True if successful
        """
        try:
            service_name = f"roboshire_ssh_{host}"
            account_name = user

            if self.use_keyring:
                # Use Windows Credential Manager
                self.keyring.set_password(service_name, account_name, password)
                logging.info(f"Stored password in Windows Credential Manager for {user}@{host}")
            else:
                # Use encrypted file
                cipher = self._get_cipher()

                # Load existing credentials or create new
                if self.cred_file.exists():
                    encrypted_data = self.cred_file.read_bytes()
                    decrypted_data = cipher.decrypt(encrypted_data)
                    credentials = json.loads(decrypted_data.decode())
                else:
                    credentials = {}

                # Store new credential
                key = f"{user}@{host}"
                credentials[key] = password

                # Encrypt and save
                json_data = json.dumps(credentials)
                encrypted_data = cipher.encrypt(json_data.encode())
                self.cred_file.write_bytes(encrypted_data)

                logging.info(f"Stored password in encrypted file for {user}@{host}")

            return True

        except Exception as e:
            logging.error(f"Failed to store password: {e}")
            return False

    def get_password(self, host: str, user: str) -> Optional[str]:
        """
        Retrieve stored SSH password

        Args:
            host: SSH host
            user: SSH username

        Returns:
            Password if found, None otherwise
        """
        try:
            service_name = f"roboshire_ssh_{host}"
            account_name = user

            if self.use_keyring:
                # Retrieve from Windows Credential Manager
                password = self.keyring.get_password(service_name, account_name)
                if password:
                    logging.debug(f"Retrieved password from Windows Credential Manager for {user}@{host}")
                return password

            else:
                # Retrieve from encrypted file
                if not self.cred_file.exists():
                    return None

                cipher = self._get_cipher()
                encrypted_data = self.cred_file.read_bytes()
                decrypted_data = cipher.decrypt(encrypted_data)
                credentials = json.loads(decrypted_data.decode())

                key = f"{user}@{host}"
                password = credentials.get(key)

                if password:
                    logging.debug(f"Retrieved password from encrypted file for {user}@{host}")

                return password

        except Exception as e:
            logging.error(f"Failed to retrieve password: {e}")
            return None

    def delete_password(self, host: str, user: str) -> bool:
        """
        Delete stored SSH password

        Args:
            host: SSH host
            user: SSH username

        Returns:
            True if successful
        """
        try:
            service_name = f"roboshire_ssh_{host}"
            account_name = user

            if self.use_keyring:
                # Delete from Windows Credential Manager
                try:
                    self.keyring.delete_password(service_name, account_name)
                    logging.info(f"Deleted password from Windows Credential Manager for {user}@{host}")
                    return True
                except self.keyring.errors.PasswordDeleteError:
                    logging.warning(f"No password found to delete for {user}@{host}")
                    return False

            else:
                # Delete from encrypted file
                if not self.cred_file.exists():
                    return False

                cipher = self._get_cipher()
                encrypted_data = self.cred_file.read_bytes()
                decrypted_data = cipher.decrypt(encrypted_data)
                credentials = json.loads(decrypted_data.decode())

                key = f"{user}@{host}"
                if key in credentials:
                    del credentials[key]

                    # Save updated credentials
                    json_data = json.dumps(credentials)
                    encrypted_data = cipher.encrypt(json_data.encode())
                    self.cred_file.write_bytes(encrypted_data)

                    logging.info(f"Deleted password from encrypted file for {user}@{host}")
                    return True
                else:
                    logging.warning(f"No password found to delete for {user}@{host}")
                    return False

        except Exception as e:
            logging.error(f"Failed to delete password: {e}")
            return False

    def clear_all(self) -> bool:
        """
        Clear all stored credentials

        Returns:
            True if successful
        """
        try:
            if self.use_keyring:
                logging.warning("Cannot clear all keyring credentials automatically")
                return False
            else:
                if self.cred_file.exists():
                    self.cred_file.unlink()
                    logging.info("Cleared all stored credentials")
                return True

        except Exception as e:
            logging.error(f"Failed to clear credentials: {e}")
            return False

    def has_password(self, host: str, user: str) -> bool:
        """
        Check if password is stored

        Args:
            host: SSH host
            user: SSH username

        Returns:
            True if password exists
        """
        password = self.get_password(host, user)
        return password is not None

    def prompt_and_store(self, host: str, user: str, prompt_text: Optional[str] = None) -> Optional[str]:
        """
        Prompt for password and store it

        Args:
            host: SSH host
            user: SSH username
            prompt_text: Custom prompt text

        Returns:
            Password entered by user
        """
        if prompt_text is None:
            prompt_text = f"Enter password for {user}@{host}: "

        try:
            password = getpass.getpass(prompt_text)

            if password:
                self.store_password(host, user, password)
                return password
            else:
                logging.warning("Empty password entered")
                return None

        except Exception as e:
            logging.error(f"Failed to prompt for password: {e}")
            return None
