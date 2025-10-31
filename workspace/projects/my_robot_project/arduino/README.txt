RoboShire Arduino Firmware
===========================

Upload this sketch to your Arduino/ESP32 board.

Requirements:
- Arduino Uno/Mega or ESP32
- Motor driver (L298N or similar)
- Ultrasonic sensor (HC-SR04)

Upload Instructions:
1. Open main.ino in Arduino IDE
2. Select your board (Tools -> Board)
3. Select serial port (Tools -> Port)
4. Click Upload button

The Arduino will communicate with ROS2 via the serial_bridge node.
