/*
 * Line Following Robot - Arduino Firmware
 *
 * Hardware:
 * - Arduino Mega 2560 or Arduino Due
 * - QTR-8A Reflectance Sensor Array (8 analog sensors)
 * - L298N Motor Driver
 * - 2x DC Motors with wheels
 * - 6V Battery pack
 *
 * Features:
 * - PID control for smooth line following
 * - Sensor calibration routine
 * - Serial debugging output
 * - Emergency stop handling
 * - Adjustable speed control
 *
 * Wiring:
 * - QTR-8A sensors: A0-A7 (analog inputs)
 * - Left motor: PWM Pin 5 (speed), Pins 8-9 (direction)
 * - Right motor: PWM Pin 6 (speed), Pins 10-11 (direction)
 * - Emergency stop: Pin 7 (button to GND)
 *
 * Author: RoboShire Team
 * License: Apache 2.0
 * Version: 1.0.0
 */

// ==================== PIN DEFINITIONS ====================
// QTR-8A Sensor Array (left to right)
#define SENSOR_1 A0  // Leftmost sensor
#define SENSOR_2 A1
#define SENSOR_3 A2
#define SENSOR_4 A3
#define SENSOR_5 A4
#define SENSOR_6 A5
#define SENSOR_7 A6
#define SENSOR_8 A7  // Rightmost sensor

// Motor Control Pins
#define LEFT_MOTOR_PWM 5    // PWM for left motor speed
#define LEFT_MOTOR_DIR1 8   // Direction pin 1
#define LEFT_MOTOR_DIR2 9   // Direction pin 2

#define RIGHT_MOTOR_PWM 6   // PWM for right motor speed
#define RIGHT_MOTOR_DIR1 10 // Direction pin 1
#define RIGHT_MOTOR_DIR2 11 // Direction pin 2

// Emergency Stop
#define EMERGENCY_STOP_PIN 7  // Button to GND

// Status LED
#define STATUS_LED_PIN 13

// ==================== CONSTANTS ====================
#define NUM_SENSORS 8
#define CALIBRATION_SAMPLES 100
#define BASE_SPEED 120      // Base motor speed (0-255)
#define MAX_SPEED 200       // Maximum motor speed
#define MIN_SPEED 50        // Minimum motor speed (deadzone)

// PID Constants (tune these for your robot!)
float Kp = 0.1;   // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 0.5;   // Derivative gain

// ==================== GLOBAL VARIABLES ====================
int sensorPins[NUM_SENSORS] = {SENSOR_1, SENSOR_2, SENSOR_3, SENSOR_4,
                                SENSOR_5, SENSOR_6, SENSOR_7, SENSOR_8};
int sensorValues[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
bool calibrated = false;

// PID variables
float lastError = 0;
float integral = 0;
float derivative = 0;
float error = 0;

// State variables
bool emergencyStop = false;
unsigned long lastPrintTime = 0;
#define PRINT_INTERVAL 200  // Print debug info every 200ms

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("===================================");
  Serial.println("Line Following Robot - Starting...");
  Serial.println("===================================");

  // Configure motor pins
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

  // Configure sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Configure emergency stop
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

  // Configure status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Initialize motors to stopped
  stopMotors();

  Serial.println("Hardware initialized");
  Serial.println("");
  Serial.println("CALIBRATION INSTRUCTIONS:");
  Serial.println("1. Place robot on WHITE surface");
  Serial.println("2. Wait 3 seconds...");
  delay(3000);

  // Calibrate sensors
  calibrateSensors();

  Serial.println("");
  Serial.println("Ready to follow line!");
  Serial.println("Place robot on track and press reset to start.");
  Serial.println("");

  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check emergency stop
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    if (!emergencyStop) {
      emergencyStop = true;
      stopMotors();
      Serial.println("❌ EMERGENCY STOP ACTIVATED!");
    }
    digitalWrite(STATUS_LED_PIN, LOW);
    return;
  } else {
    emergencyStop = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
  }

  // Read sensors
  readSensors();

  // Calculate line position
  float position = calculateLinePosition();

  // Calculate error (0 = center, negative = left, positive = right)
  error = position - 3.5;  // Center position is 3.5 (between sensor 3 and 4)

  // PID calculation
  integral += error;
  derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Limit integral windup
  integral = constrain(integral, -100, 100);

  // Calculate motor speeds
  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  // Apply speeds to motors
  setMotorSpeeds(leftSpeed, rightSpeed);

  // Debug output
  if (millis() - lastPrintTime > PRINT_INTERVAL) {
    printDebugInfo(position, error, correction, leftSpeed, rightSpeed);
    lastPrintTime = millis();
  }
}

// ==================== SENSOR FUNCTIONS ====================

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  Serial.println("Move robot left and right over line");

  // Initialize min/max arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  // Collect calibration data
  for (int sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
    // Read each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value = analogRead(sensorPins[i]);

      // Update min/max
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }

    // Visual feedback
    if (sample % 10 == 0) {
      Serial.print(".");
    }

    delay(20);
  }

  Serial.println("");
  Serial.println("✅ Calibration complete!");
  Serial.println("Sensor ranges:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorMin[i]);
    Serial.print(" - ");
    Serial.println(sensorMax[i]);
  }

  calibrated = true;
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);

    // Normalize to 0-1000 range
    if (calibrated) {
      sensorValues[i] = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
      sensorValues[i] = constrain(sensorValues[i], 0, 1000);
    } else {
      // If not calibrated, use raw values
      sensorValues[i] = raw;
    }
  }
}

float calculateLinePosition() {
  /*
   * Calculate weighted average of sensor positions
   * Returns value 0.0 to 7.0 where:
   *   0.0 = line at leftmost sensor
   *   3.5 = line centered
   *   7.0 = line at rightmost sensor
   */

  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (long)sensorValues[i] * i;
    sum += sensorValues[i];
  }

  if (sum == 0) {
    // No line detected - return last known position
    Serial.println("⚠️  WARNING: No line detected!");
    return 3.5;  // Assume center
  }

  float position = (float)weightedSum / (float)sum;
  return position;
}

// ==================== MOTOR FUNCTIONS ====================

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_MOTOR_DIR1, HIGH);
    digitalWrite(LEFT_MOTOR_DIR2, LOW);
    analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));
  } else {
    digitalWrite(LEFT_MOTOR_DIR1, LOW);
    digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));
  }

  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
    digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
  } else {
    digitalWrite(RIGHT_MOTOR_DIR1, LOW);
    digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
  }
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_DIR1, LOW);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);

  digitalWrite(RIGHT_MOTOR_DIR1, LOW);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

// ==================== DEBUG FUNCTIONS ====================

void printDebugInfo(float position, float error, float correction,
                   int leftSpeed, int rightSpeed) {
  Serial.print("Pos:");
  Serial.print(position, 2);
  Serial.print(" Err:");
  Serial.print(error, 2);
  Serial.print(" Corr:");
  Serial.print(correction, 2);
  Serial.print(" L:");
  Serial.print(leftSpeed);
  Serial.print(" R:");
  Serial.print(rightSpeed);
  Serial.print(" Sensors:[");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    if (i < NUM_SENSORS - 1) Serial.print(",");
  }
  Serial.println("]");
}

// ==================== SERIAL COMMANDS ====================
// Optional: Add serial commands for tuning PID

void serialEvent() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'p') {
      // Increase Kp
      Kp += 0.01;
      Serial.print("Kp = ");
      Serial.println(Kp, 3);
    }
    else if (cmd == 'P') {
      // Decrease Kp
      Kp -= 0.01;
      Serial.print("Kp = ");
      Serial.println(Kp, 3);
    }
    else if (cmd == 'i') {
      // Increase Ki
      Ki += 0.001;
      Serial.print("Ki = ");
      Serial.println(Ki, 4);
    }
    else if (cmd == 'I') {
      // Decrease Ki
      Ki -= 0.001;
      Serial.print("Ki = ");
      Serial.println(Ki, 4);
    }
    else if (cmd == 'd') {
      // Increase Kd
      Kd += 0.05;
      Serial.print("Kd = ");
      Serial.println(Kd, 3);
    }
    else if (cmd == 'D') {
      // Decrease Kd
      Kd -= 0.05;
      Serial.print("Kd = ");
      Serial.println(Kd, 3);
    }
    else if (cmd == 's') {
      // Print current PID values
      Serial.println("Current PID values:");
      Serial.print("  Kp = ");
      Serial.println(Kp, 3);
      Serial.print("  Ki = ");
      Serial.println(Ki, 4);
      Serial.print("  Kd = ");
      Serial.println(Kd, 3);
    }
    else if (cmd == 'c') {
      // Recalibrate
      calibrateSensors();
    }
    else if (cmd == 'h') {
      // Print help
      Serial.println("Serial Commands:");
      Serial.println("  p/P - Increase/Decrease Kp");
      Serial.println("  i/I - Increase/Decrease Ki");
      Serial.println("  d/D - Increase/Decrease Kd");
      Serial.println("  s   - Show current PID values");
      Serial.println("  c   - Recalibrate sensors");
      Serial.println("  h   - Show this help");
    }
  }
}
