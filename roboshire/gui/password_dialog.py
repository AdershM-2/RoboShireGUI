"""
Password Dialog - GUI for entering SSH password

Features:
- Secure password entry
- Remember password checkbox
- Connection test
"""

import logging
from typing import Optional
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QCheckBox, QDialogButtonBox, QGroupBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont


class PasswordDialog(QDialog):
    """
    Dialog for entering SSH password

    Features:
    - Password field (masked)
    - Remember password checkbox
    - OK/Cancel buttons
    - User/host display
    """

    def __init__(self, host: str, user: str, parent=None):
        """
        Initialize password dialog

        Args:
            host: SSH host
            user: SSH username
            parent: Parent widget
        """
        super().__init__(parent)

        self.host = host
        self.user = user
        self.password: Optional[str] = None
        self.remember = True  # Default to remember

        self.setWindowTitle("SSH Password Required")
        self.setModal(True)
        self.resize(400, 200)

        self._init_ui()

        logging.info("PasswordDialog initialized")

    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()

        # Title
        title_label = QLabel("<h3>SSH Connection</h3>")
        layout.addWidget(title_label)

        # Connection info
        info_group = QGroupBox("Connection Details")
        info_layout = QVBoxLayout()

        host_label = QLabel(f"<b>Host:</b> {self.host}")
        info_layout.addWidget(host_label)

        user_label = QLabel(f"<b>User:</b> {self.user}")
        info_layout.addWidget(user_label)

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # Password field
        password_group = QGroupBox("Authentication")
        password_layout = QVBoxLayout()

        password_label = QLabel("Password:")
        password_layout.addWidget(password_label)

        self.password_field = QLineEdit()
        self.password_field.setEchoMode(QLineEdit.Password)
        self.password_field.setPlaceholderText("Enter your SSH password")
        password_layout.addWidget(self.password_field)

        # Remember checkbox
        self.remember_checkbox = QCheckBox("Remember password (stored securely)")
        self.remember_checkbox.setChecked(True)
        password_layout.addWidget(self.remember_checkbox)

        password_group.setLayout(password_layout)
        layout.addWidget(password_group)

        # Info label
        info_label = QLabel(
            "<i>Your password will be stored securely using Windows Credential Manager "
            "or encrypted file storage.</i>"
        )
        info_label.setWordWrap(True)
        font = QFont()
        font.setPointSize(8)
        info_label.setFont(font)
        layout.addWidget(info_label)

        layout.addStretch()

        # Buttons
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self._on_ok)
        button_box.rejected.connect(self.reject)

        layout.addWidget(button_box)

        self.setLayout(layout)

        # Set focus to password field
        self.password_field.setFocus()

    def _on_ok(self):
        """Handle OK button click"""
        password = self.password_field.text()

        if not password:
            logging.warning("Empty password entered")
            # Could show error message, but for now just return
            return

        self.password = password
        self.remember = self.remember_checkbox.isChecked()

        logging.info(f"Password entered (remember={self.remember})")
        self.accept()

    def get_password(self) -> Optional[str]:
        """
        Get entered password

        Returns:
            Password or None if cancelled
        """
        return self.password

    def should_remember(self) -> bool:
        """
        Check if password should be remembered

        Returns:
            True if remember checkbox was checked
        """
        return self.remember

    @staticmethod
    def get_password_from_user(
        host: str,
        user: str,
        parent=None
    ) -> tuple[Optional[str], bool]:
        """
        Static method to get password from user

        Args:
            host: SSH host
            user: SSH username
            parent: Parent widget

        Returns:
            Tuple of (password, remember)
        """
        dialog = PasswordDialog(host, user, parent)
        result = dialog.exec()

        if result == QDialog.Accepted:
            return dialog.get_password(), dialog.should_remember()
        else:
            return None, False
