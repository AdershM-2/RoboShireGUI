"""
Serial Monitor Widget

Real-time serial port monitoring for Arduino/ESP32 debugging.
No separate tools needed - monitor serial output directly in RoboShire!

Features:
- Auto-detect available serial ports
- Multiple baud rates (9600, 115200, etc.)
- Send commands to serial device
- Autoscroll with pause button
- Clear/export logs
- Timestamp display
- Color-coded messages (errors in red, warnings in yellow)

Keyboard Shortcut: Ctrl+Shift+S
"""

import time
from typing import Optional
from datetime import datetime
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QTextEdit, QLineEdit, QCheckBox, QGroupBox,
    QSplitter, QMessageBox, QFileDialog
)
from PySide6.QtCore import Qt, Signal, QTimer, Slot, QThread
from PySide6.QtGui import QFont, QTextCursor, QTextCharFormat, QColor
import logging

# Import serial library
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class SerialReaderThread(QThread):
    """Background thread for reading serial data"""

    data_received = Signal(str)  # Emits received data
    error_occurred = Signal(str)  # Emits error messages

    def __init__(self, port: str, baudrate: int):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.running = False

    def run(self):
        """Main thread loop"""
        try:
            self.serial_conn = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1  # Non-blocking read with 100ms timeout
            )
            self.running = True

            while self.running:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    try:
                        data = self.serial_conn.readline().decode('utf-8', errors='ignore')
                        if data:
                            self.data_received.emit(data)
                    except (UnicodeDecodeError, serial.SerialException) as e:
                        self.error_occurred.emit(f"Read error: {e}")

                # Small sleep to prevent CPU spinning
                self.msleep(10)

        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open port {self.port}: {e}")
        finally:
            self.cleanup()

    def send_data(self, data: str):
        """Send data to serial port"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(data.encode('utf-8'))
            except serial.SerialException as e:
                self.error_occurred.emit(f"Write error: {e}")

    def cleanup(self):
        """Close serial connection"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except:
                pass

    def stop(self):
        """Stop the reader thread"""
        self.running = False


class SerialMonitorWidget(QWidget):
    """
    Serial Monitor Widget for Arduino/ESP32 debugging

    Provides real-time serial output monitoring with:
    - Auto-detection of serial ports
    - Multiple baud rates
    - Send commands to device
    - Color-coded output
    - Export logs
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.logger = logging.getLogger(__name__)
        self.serial_thread: Optional[SerialReaderThread] = None
        self.autoscroll = True
        self.message_count = 0

        self._setup_ui()

        # Auto-refresh ports every 2 seconds
        self.port_refresh_timer = QTimer()
        self.port_refresh_timer.timeout.connect(self._refresh_ports)
        self.port_refresh_timer.start(2000)

        # Initial port detection
        self._refresh_ports()

    def _setup_ui(self):
        """Setup the user interface"""
        layout = QVBoxLayout(self)

        # Title
        title = QLabel("Serial Monitor")
        title_font = QFont()
        title_font.setPointSize(12)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        # Connection controls
        conn_group = QGroupBox("Connection Settings")
        conn_layout = QHBoxLayout()

        # Port selection
        conn_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        conn_layout.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(self.refresh_btn)

        # Baud rate
        conn_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems([
            "9600", "19200", "38400", "57600", "115200", "230400", "250000"
        ])
        self.baud_combo.setCurrentText("115200")  # Most common for Arduino
        conn_layout.addWidget(self.baud_combo)

        # Connect/Disconnect buttons
        self.connect_btn = QPushButton("▶ Connect")
        self.connect_btn.clicked.connect(self._on_connect)
        self.connect_btn.setStyleSheet("""
            QPushButton {
                background-color: rgb(85, 107, 47);
                color: white;
                padding: 6px 12px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: rgb(95, 120, 50);
            }
        """)
        conn_layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("⏹ Disconnect")
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("""
            QPushButton {
                background-color: rgb(211, 47, 47);
                color: white;
                padding: 6px 12px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: rgb(198, 40, 40);
            }
        """)
        conn_layout.addWidget(self.disconnect_btn)

        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        # Output display
        output_group = QGroupBox("Serial Output")
        output_layout = QVBoxLayout()

        # Toolbar
        toolbar_layout = QHBoxLayout()

        self.autoscroll_check = QCheckBox("Autoscroll")
        self.autoscroll_check.setChecked(True)
        self.autoscroll_check.stateChanged.connect(
            lambda state: setattr(self, 'autoscroll', state == Qt.Checked)
        )
        toolbar_layout.addWidget(self.autoscroll_check)

        self.timestamp_check = QCheckBox("Show Timestamps")
        self.timestamp_check.setChecked(True)
        toolbar_layout.addWidget(self.timestamp_check)

        toolbar_layout.addStretch()

        self.message_count_label = QLabel("Messages: 0")
        toolbar_layout.addWidget(self.message_count_label)

        self.clear_btn = QPushButton("🗑 Clear")
        self.clear_btn.clicked.connect(self._clear_output)
        toolbar_layout.addWidget(self.clear_btn)

        self.export_btn = QPushButton("💾 Export")
        self.export_btn.clicked.connect(self._export_log)
        toolbar_layout.addWidget(self.export_btn)

        output_layout.addLayout(toolbar_layout)

        # Text display
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setFont(QFont("JetBrains Mono", 9))
        self.output_text.setMinimumHeight(300)
        output_layout.addWidget(self.output_text)

        output_group.setLayout(output_layout)
        layout.addWidget(output_group)

        # Send commands
        send_group = QGroupBox("Send to Device")
        send_layout = QHBoxLayout()

        self.send_input = QLineEdit()
        self.send_input.setPlaceholderText("Type command and press Enter...")
        self.send_input.returnPressed.connect(self._send_command)
        send_layout.addWidget(self.send_input)

        self.send_btn = QPushButton("📤 Send")
        self.send_btn.clicked.connect(self._send_command)
        self.send_btn.setEnabled(False)
        send_layout.addWidget(self.send_btn)

        self.newline_check = QCheckBox("Add newline (\\n)")
        self.newline_check.setChecked(True)
        send_layout.addWidget(self.newline_check)

        send_group.setLayout(send_layout)
        layout.addWidget(send_group)

        # Status bar
        status_layout = QHBoxLayout()

        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("QLabel { color: gray; }")
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()

        layout.addLayout(status_layout)

        # Warning if serial not available
        if not SERIAL_AVAILABLE:
            warning = QLabel("⚠️ PySerial not installed. Install with: pip install pyserial")
            warning.setStyleSheet("QLabel { color: orange; font-weight: bold; padding: 5px; }")
            layout.insertWidget(1, warning)
            self.connect_btn.setEnabled(False)

    def _refresh_ports(self):
        """Refresh available serial ports"""
        if not SERIAL_AVAILABLE:
            return

        current_port = self.port_combo.currentText()

        self.port_combo.clear()

        try:
            ports = serial.tools.list_ports.comports()

            for port in ports:
                # Format: "COM3 - Arduino Uno (USB Serial)"
                port_text = f"{port.device}"
                if port.description and port.description != port.device:
                    port_text += f" - {port.description}"

                self.port_combo.addItem(port_text, port.device)

            if not ports:
                self.port_combo.addItem("No serial ports found", "")
                self.connect_btn.setEnabled(False)
            else:
                self.connect_btn.setEnabled(True)

                # Try to restore previous selection
                index = self.port_combo.findText(current_port)
                if index >= 0:
                    self.port_combo.setCurrentIndex(index)

        except Exception as e:
            self.logger.error(f"Failed to refresh ports: {e}")

    def _on_connect(self):
        """Connect to serial port"""
        if not SERIAL_AVAILABLE:
            QMessageBox.warning(self, "PySerial Not Installed",
                              "Please install pyserial:\n\npip install pyserial")
            return

        port_data = self.port_combo.currentData()
        if not port_data:
            QMessageBox.warning(self, "No Port Selected",
                              "Please select a serial port first.")
            return

        baudrate = int(self.baud_combo.currentText())

        # Create and start serial reader thread
        self.serial_thread = SerialReaderThread(port_data, baudrate)
        self.serial_thread.data_received.connect(self._on_data_received)
        self.serial_thread.error_occurred.connect(self._on_error)
        self.serial_thread.start()

        # Update UI
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.send_btn.setEnabled(True)
        self.port_combo.setEnabled(False)
        self.baud_combo.setEnabled(False)

        self.status_label.setText(f"Status: Connected to {port_data} @ {baudrate} baud")
        self.status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")

        self._append_output(f"=== Connected to {port_data} @ {baudrate} baud ===\n", "system")

    def _on_disconnect(self):
        """Disconnect from serial port"""
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait(2000)  # Wait up to 2 seconds
            self.serial_thread = None

        # Update UI
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.send_btn.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)

        self.status_label.setText("Status: Disconnected")
        self.status_label.setStyleSheet("QLabel { color: gray; }")

        self._append_output("\n=== Disconnected ===\n", "system")

    @Slot(str)
    def _on_data_received(self, data: str):
        """Handle received serial data"""
        # Detect message type by keywords
        data_lower = data.lower()
        if 'error' in data_lower or 'fail' in data_lower or '❌' in data:
            msg_type = "error"
        elif 'warning' in data_lower or 'warn' in data_lower or '⚠' in data:
            msg_type = "warning"
        elif 'success' in data_lower or '✅' in data or '✓' in data:
            msg_type = "success"
        else:
            msg_type = "normal"

        self._append_output(data, msg_type)

    @Slot(str)
    def _on_error(self, error_msg: str):
        """Handle serial errors"""
        self._append_output(f"[ERROR] {error_msg}\n", "error")
        self.logger.error(error_msg)

        # Auto-disconnect on fatal errors
        if "Failed to open port" in error_msg:
            self._on_disconnect()

    def _append_output(self, text: str, msg_type: str = "normal"):
        """Append text to output with formatting"""
        self.message_count += 1
        self.message_count_label.setText(f"Messages: {self.message_count}")

        # Prepare timestamp
        timestamp = ""
        if self.timestamp_check.isChecked() and msg_type != "system":
            timestamp = f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] "

        # Set color based on message type
        cursor = self.output_text.textCursor()
        cursor.movePosition(QTextCursor.End)

        text_format = QTextCharFormat()

        if msg_type == "error":
            text_format.setForeground(QColor(211, 47, 47))  # Red
        elif msg_type == "warning":
            text_format.setForeground(QColor(237, 108, 2))  # Orange
        elif msg_type == "success":
            text_format.setForeground(QColor(46, 125, 50))  # Green
        elif msg_type == "system":
            text_format.setForeground(QColor(85, 107, 47))  # Olive
            text_format.setFontWeight(QFont.Bold)
        else:
            text_format.setForeground(QColor(51, 51, 51))  # Dark gray

        cursor.setCharFormat(text_format)
        cursor.insertText(timestamp + text)

        # Autoscroll
        if self.autoscroll:
            self.output_text.setTextCursor(cursor)
            self.output_text.ensureCursorVisible()

    def _send_command(self):
        """Send command to serial device"""
        command = self.send_input.text()
        if not command:
            return

        if self.newline_check.isChecked():
            command += "\n"

        if self.serial_thread:
            self.serial_thread.send_data(command)
            self._append_output(f">>> {command}", "system")

        self.send_input.clear()

    def _clear_output(self):
        """Clear output display"""
        self.output_text.clear()
        self.message_count = 0
        self.message_count_label.setText("Messages: 0")

    def _export_log(self):
        """Export serial log to file"""
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export Serial Log",
            f"serial_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt);;All Files (*)"
        )

        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(self.output_text.toPlainText())

                QMessageBox.information(self, "Export Successful",
                                      f"Log exported to:\n{filename}")

            except Exception as e:
                QMessageBox.critical(self, "Export Failed",
                                   f"Failed to export log:\n{e}")

    def cleanup(self):
        """Cleanup when widget is closed"""
        self.port_refresh_timer.stop()

        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait(2000)
            self.serial_thread = None


# Test code
if __name__ == '__main__':
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    widget = SerialMonitorWidget()
    widget.setWindowTitle("Serial Monitor - RoboShire")
    widget.resize(800, 600)
    widget.show()

    sys.exit(app.exec())
