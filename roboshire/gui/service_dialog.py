"""
Service Call Dialog - GUI for calling ROS2 services

Features:
- Service/action selector
- Request data editor (YAML/JSON)
- Call button
- Response viewer
- Call history
"""

import logging
import json
from typing import Optional
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QTextEdit, QGroupBox, QSplitter, QListWidget,
    QMessageBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont

from roboshire.integrations.service_manager import ServiceManager


class ServiceCallDialog(QDialog):
    """
    Dialog for calling ROS2 services and actions

    Features:
    - Service/action selection
    - Request editor
    - Response display
    - Call history
    """

    def __init__(self, service_manager: Optional[ServiceManager] = None, parent=None):
        """
        Initialize service call dialog

        Args:
            service_manager: ServiceManager instance
            parent: Parent widget
        """
        super().__init__(parent)

        self.service_manager = service_manager
        self.call_history = []

        self.setWindowTitle("Service/Action Call")
        self.resize(800, 600)

        self._init_ui()
        self._refresh_services()

        logging.info("ServiceCallDialog initialized")

    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()

        # Service/Action selector
        selector_group = QGroupBox("Service/Action Selection")
        selector_layout = QVBoxLayout()

        # Type selector (Service or Action)
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel("Type:"))
        self.type_combo = QComboBox()
        self.type_combo.addItems(["Service", "Action"])
        self.type_combo.currentTextChanged.connect(self._on_type_changed)
        type_layout.addWidget(self.type_combo)

        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self._refresh_services)
        type_layout.addWidget(refresh_button)
        type_layout.addStretch()

        selector_layout.addLayout(type_layout)

        # Service/Action name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Name:"))
        self.name_combo = QComboBox()
        self.name_combo.setEditable(True)
        self.name_combo.currentTextChanged.connect(self._on_service_selected)
        name_layout.addWidget(self.name_combo)
        selector_layout.addLayout(name_layout)

        # Service/Action type
        srv_type_layout = QHBoxLayout()
        srv_type_layout.addWidget(QLabel("Type:"))
        self.srv_type_label = QLabel("-")
        srv_type_layout.addWidget(self.srv_type_label)
        srv_type_layout.addStretch()
        selector_layout.addLayout(srv_type_layout)

        selector_group.setLayout(selector_layout)
        layout.addWidget(selector_group)

        # Main splitter - Request/Response
        splitter = QSplitter(Qt.Vertical)

        # Request editor
        request_group = QGroupBox("Request Data (YAML/JSON)")
        request_layout = QVBoxLayout()

        self.request_editor = QTextEdit()
        self.request_editor.setFont(QFont("Courier", 9))
        self.request_editor.setPlaceholderText('Enter request data in YAML or JSON format.\nExample: {data: true}')
        request_layout.addWidget(self.request_editor)

        # Example templates
        template_layout = QHBoxLayout()
        template_layout.addWidget(QLabel("Templates:"))

        bool_button = QPushButton("Bool")
        bool_button.clicked.connect(lambda: self.request_editor.setText("{data: true}"))
        template_layout.addWidget(bool_button)

        trigger_button = QPushButton("Trigger")
        trigger_button.clicked.connect(lambda: self.request_editor.setText("{}"))
        template_layout.addWidget(trigger_button)

        template_layout.addStretch()
        request_layout.addLayout(template_layout)

        request_group.setLayout(request_layout)
        splitter.addWidget(request_group)

        # Response viewer
        response_group = QGroupBox("Response")
        response_layout = QVBoxLayout()

        self.response_viewer = QTextEdit()
        self.response_viewer.setReadOnly(True)
        self.response_viewer.setFont(QFont("Courier", 9))
        response_layout.addWidget(self.response_viewer)

        response_group.setLayout(response_layout)
        splitter.addWidget(response_group)

        splitter.setSizes([200, 200])
        layout.addWidget(splitter)

        # Action buttons
        button_layout = QHBoxLayout()

        self.call_button = QPushButton("Call Service/Action")
        self.call_button.clicked.connect(self._call_service)
        button_layout.addWidget(self.call_button)

        clear_button = QPushButton("Clear Response")
        clear_button.clicked.connect(self.response_viewer.clear)
        button_layout.addWidget(clear_button)

        button_layout.addStretch()

        close_button = QPushButton("Close")
        close_button.clicked.connect(self.accept)
        button_layout.addWidget(close_button)

        layout.addLayout(button_layout)

        self.setLayout(layout)

    def set_service_manager(self, service_manager: ServiceManager):
        """
        Set the service manager instance

        Args:
            service_manager: ServiceManager instance
        """
        self.service_manager = service_manager
        self._refresh_services()

    def _refresh_services(self):
        """Refresh the list of services/actions"""
        if not self.service_manager:
            return

        try:
            self.name_combo.clear()

            if self.type_combo.currentText() == "Service":
                items = self.service_manager.list_services()
            else:
                items = self.service_manager.list_actions()

            self.name_combo.addItems(sorted(items))
            logging.info(f"Refreshed {len(items)} {self.type_combo.currentText().lower()}s")

        except Exception as e:
            logging.error(f"Error refreshing services: {e}")

    def _on_type_changed(self, type_name: str):
        """
        Handle type change (Service/Action)

        Args:
            type_name: Selected type
        """
        self._refresh_services()

    def _on_service_selected(self, service_name: str):
        """
        Handle service/action selection

        Args:
            service_name: Selected service/action name
        """
        if not self.service_manager or not service_name:
            return

        try:
            if self.type_combo.currentText() == "Service":
                srv_type = self.service_manager.get_service_type(service_name)
            else:
                srv_type = self.service_manager.get_action_type(service_name)

            if srv_type:
                self.srv_type_label.setText(srv_type)
            else:
                self.srv_type_label.setText("-")

        except Exception as e:
            logging.error(f"Error getting service type: {e}")

    def _call_service(self):
        """Call the selected service/action"""
        if not self.service_manager:
            QMessageBox.warning(self, "Error", "Service manager not initialized")
            return

        service_name = self.name_combo.currentText()
        service_type = self.srv_type_label.text()
        request_data = self.request_editor.toPlainText().strip()

        if not service_name:
            QMessageBox.warning(self, "Error", "Please select a service/action")
            return

        if not service_type or service_type == "-":
            QMessageBox.warning(self, "Error", "Service/action type unknown")
            return

        if not request_data:
            request_data = "{}"

        try:
            self.response_viewer.clear()
            self.response_viewer.append(f"<b>Calling {service_name}...</b>\n")

            # Call service or action
            if self.type_combo.currentText() == "Service":
                result = self.service_manager.call_service(
                    service_name,
                    service_type,
                    request_data
                )
            else:
                result = self.service_manager.send_action_goal(
                    service_name,
                    service_type,
                    request_data
                )

            # Display result
            if result.success:
                self.response_viewer.append("<span style='color:green;'><b>Success!</b></span>\n")

                if result.response:
                    self.response_viewer.append("<b>Response:</b>")
                    response_json = json.dumps(result.response, indent=2)
                    self.response_viewer.append(response_json)
                else:
                    self.response_viewer.append("No response data")

                # Add to history
                self.call_history.append({
                    'name': service_name,
                    'type': service_type,
                    'request': request_data,
                    'response': result.response
                })

            else:
                self.response_viewer.append("<span style='color:red;'><b>Failed!</b></span>\n")
                self.response_viewer.append(f"<b>Error:</b> {result.error_message}")

                QMessageBox.warning(
                    self,
                    "Call Failed",
                    f"Failed to call {service_name}:\n\n{result.error_message}"
                )

        except Exception as e:
            logging.error(f"Error calling service: {e}")
            self.response_viewer.append(f"<span style='color:red;'><b>Error:</b> {e}</span>")
            QMessageBox.critical(self, "Error", f"Error calling service:\n\n{e}")
