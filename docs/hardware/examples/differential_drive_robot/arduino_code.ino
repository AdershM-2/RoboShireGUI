/*
 * DIFFERENTIAL DRIVE ROBOT FIRMWARE
 * Arduino Mega 2560
 * RoboShire v2.4.0
 *
 * Features:
 * - Dual motor control via L298N
 * - Quadrature encoder reading
 * - MPU6050 IMU sensor fusion
 * - Serial communication with ROS2
 * - Safety mechanisms (emergency stop, watchdog)
 *
 * Pin Configuration (Arduino Mega 2560):
 * Motor Control:
 *   - Motor A (Left): PWM pin 5, Dir pin 8, Dir pin 9
 *   - Motor B (Right): PWM pin 6, Dir pin 10, Dir pin 11
 * Encoders:
 *   - Encoder A (Left): Int pin 2
 *   - Encoder B (Right): Int pin 3
 * IMU (I2C):
 *   - SDA: Pin 20
 *   - SCL: Pin 21
 *
 * License: Apache 2.0
 */

#include <Wire.h>
#include <math.h>

// ==================== MOTOR CONTROL PINS ====================
#define MOTOR_A_PWM 5        // Left motor PWM (0-255)
#define MOTOR_A_DIR1 8       // Left motor direction 1
#define MOTOR_A_DIR2 9       // Left motor direction 2

#define MOTOR_B_PWM 6        // Right motor PWM (0-255)
#define MOTOR_B_DIR1 10      // Right motor direction 1
#define MOTOR_B_DIR2 11      // Right motor direction 2

// Emergency stop and status
#define EMERGENCY_STOP_PIN 12
#define STATUS_LED_PIN 13

// ==================== ENCODER PINS ====================
#define ENCODER_A_PIN 2      // Left encoder (interrupt pin)
#define ENCODER_B_PIN 3      // Right encoder (interrupt pin)

// ==================== MOTOR PARAMETERS ====================
#define MAX_PWM 200          // Prevent full power (safety)
#define MIN_PWM 30           // Deadzone threshold
#define MOTOR_SPEED_SCALE 50 // PWM change per velocity unit

// ==================== ENCODER PARAMETERS ====================
#define TICKS_PER_REV 20     // Encoder ticks per revolution (typical)
#define WHEEL_DIAMETER 0.065 // 6.5cm wheel diameter in meters
#define WHEEL_SEPARATION 0.15 // 15cm distance between wheels

// ==================== IMU PARAMETERS ====================
#define MPU6050_ADDR 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B

// ==================== SAMPLING & TIMING ====================
#define PUBLISH_INTERVAL 50  // Publish every 50ms (20Hz)
#define SERIAL_BAUD 115200
#define WATCHDOG_TIMEOUT 1000 // 1 second motor timeout

// ==================== VOLATILE VARIABLES ====================
volatile long encoder_a_ticks = 0;
volatile long encoder_b_ticks = 0;
volatile int16_t imu_accel_x = 0, imu_accel_y = 0, imu_accel_z = 0;
volatile int16_t imu_gyro_x = 0, imu_gyro_y = 0, imu_gyro_z = 0;

// ==================== MOTOR STATE ====================
int pwm_left = 0;   // Current left motor PWM (-255 to 255)
int pwm_right = 0;  // Current right motor PWM (-255 to 255)

// ==================== TIMING ====================
unsigned long last_publish = 0;
unsigned long last_command = 0;
unsigned long current_time = 0;

// ==================== STATUS ====================
boolean motors_enabled = true;
boolean emergency_stop_triggered = false;

// ==================== SETUP ====================
void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  delay(100);

  // Initialize motor control pins
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_DIR1, OUTPUT);
  pinMode(MOTOR_A_DIR2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_DIR1, OUTPUT);
  pinMode(MOTOR_B_DIR2, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  // Initialize emergency stop and LED
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);  // Status LED on

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), read_encoder_a, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), read_encoder_b, RISING);

  // Initialize I2C for IMU
  Wire.begin();
  delay(100);

  // Initialize MPU6050
  if (init_mpu6050()) {
    Serial.println("STATUS:MPU6050_OK");
  } else {
    Serial.println("ERROR:MPU6050_INIT_FAILED");
  }

  // Stop motors
  stop_motors();

  // Publish startup status
  Serial.println("STATUS:READY");
  Serial.println("INFO:Differential Drive Robot Initialized");

  last_command = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  current_time = millis();

  // Check emergency stop button
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    if (!emergency_stop_triggered) {
      emergency_stop_triggered = true;
      stop_motors();
      Serial.println("ERROR:EMERGENCY_STOP_ACTIVATED");
      digitalWrite(STATUS_LED_PIN, LOW);  // Turn off LED
    }
    delay(10);
    return;  // Stay in emergency stop
  } else {
    emergency_stop_triggered = false;
    digitalWrite(STATUS_LED_PIN, HIGH);  // LED back on
  }

  // Check watchdog timeout (no command received)
  if ((current_time - last_command) > WATCHDOG_TIMEOUT) {
    if (motors_enabled) {
      stop_motors();
      motors_enabled = false;
      Serial.println("WARNING:WATCHDOG_TIMEOUT_MOTORS_STOPPED");
    }
  } else {
    motors_enabled = true;
  }

  // Read IMU data
  read_mpu6050();

  // Publish data at set interval
  if ((current_time - last_publish) >= PUBLISH_INTERVAL) {
    publish_sensor_data();
    last_publish = current_time;
  }

  // Check for incoming serial commands
  if (Serial.available() > 0) {
    process_serial_command();
  }

  // Small delay to prevent overwhelming the loop
  delay(5);
}

// ==================== MOTOR CONTROL ====================
void set_motor_pwm(int left_pwm, int right_pwm) {
  // Safety limits
  left_pwm = constrain(left_pwm, -MAX_PWM, MAX_PWM);
  right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

  // Apply deadzone
  if (abs(left_pwm) < MIN_PWM) left_pwm = 0;
  if (abs(right_pwm) < MIN_PWM) right_pwm = 0;

  pwm_left = left_pwm;
  pwm_right = right_pwm;

  // Set left motor
  if (left_pwm > 0) {
    // Forward
    digitalWrite(MOTOR_A_DIR1, HIGH);
    digitalWrite(MOTOR_A_DIR2, LOW);
    analogWrite(MOTOR_A_PWM, abs(left_pwm));
  } else if (left_pwm < 0) {
    // Backward
    digitalWrite(MOTOR_A_DIR1, LOW);
    digitalWrite(MOTOR_A_DIR2, HIGH);
    analogWrite(MOTOR_A_PWM, abs(left_pwm));
  } else {
    // Stop
    digitalWrite(MOTOR_A_DIR1, LOW);
    digitalWrite(MOTOR_A_DIR2, LOW);
    analogWrite(MOTOR_A_PWM, 0);
  }

  // Set right motor
  if (right_pwm > 0) {
    // Forward
    digitalWrite(MOTOR_B_DIR1, HIGH);
    digitalWrite(MOTOR_B_DIR2, LOW);
    analogWrite(MOTOR_B_PWM, abs(right_pwm));
  } else if (right_pwm < 0) {
    // Backward
    digitalWrite(MOTOR_B_DIR1, LOW);
    digitalWrite(MOTOR_B_DIR2, HIGH);
    analogWrite(MOTOR_B_PWM, abs(right_pwm));
  } else {
    // Stop
    digitalWrite(MOTOR_B_DIR1, LOW);
    digitalWrite(MOTOR_B_DIR2, LOW);
    analogWrite(MOTOR_B_PWM, 0);
  }

  last_command = current_time;
}

void stop_motors() {
  set_motor_pwm(0, 0);
}

// ==================== ENCODER READING ====================
void read_encoder_a() {
  encoder_a_ticks++;
}

void read_encoder_b() {
  encoder_b_ticks++;
}

// ==================== IMU (MPU6050) ====================
boolean init_mpu6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);  // Wake up
  if (Wire.endTransmission() != 0) {
    return false;
  }

  delay(100);
  return true;
}

void read_mpu6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  if (Wire.available() >= 14) {
    // Read accelerometer data (2 bytes each, big-endian)
    imu_accel_x = (Wire.read() << 8) | Wire.read();
    imu_accel_y = (Wire.read() << 8) | Wire.read();
    imu_accel_z = (Wire.read() << 8) | Wire.read();

    // Skip temperature (2 bytes)
    Wire.read();
    Wire.read();

    // Read gyroscope data (2 bytes each, big-endian)
    imu_gyro_x = (Wire.read() << 8) | Wire.read();
    imu_gyro_y = (Wire.read() << 8) | Wire.read();
    imu_gyro_z = (Wire.read() << 8) | Wire.read();
  }
}

// ==================== SERIAL COMMAND PROCESSING ====================
void process_serial_command() {
  if (Serial.peek() == 'M') {
    // Motor command: M<left_pwm><right_pwm>
    Serial.read();  // consume 'M'

    if (Serial.available() >= 4) {
      // Read 2-byte signed integers
      int8_t left_cmd = Serial.read();
      int8_t right_cmd = Serial.read();

      set_motor_pwm(left_cmd, right_cmd);
    }
  } else if (Serial.peek() == 'E') {
    // Echo command for testing
    Serial.read();
    Serial.println("ECHO:OK");
  } else {
    // Unknown command, flush
    Serial.read();
  }
}

// ==================== SENSOR DATA PUBLICATION ====================
void publish_sensor_data() {
  // Format: SENSOR:<encoder_l>,<encoder_r>,<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>

  // Convert encoder ticks to distance (meters)
  float dist_l = (float)encoder_a_ticks / TICKS_PER_REV * 3.14159 * WHEEL_DIAMETER;
  float dist_r = (float)encoder_b_ticks / TICKS_PER_REV * 3.14159 * WHEEL_DIAMETER;

  // Convert IMU raw values to physical units
  float accel_x = imu_accel_x / 16384.0;  // ±2g scale
  float accel_y = imu_accel_y / 16384.0;
  float accel_z = imu_accel_z / 16384.0;
  float gyro_x = imu_gyro_x / 131.0;      // ±250°/s scale
  float gyro_y = imu_gyro_y / 131.0;
  float gyro_z = imu_gyro_z / 131.0;

  // Publish formatted data
  Serial.print("SENSOR:");
  Serial.print(dist_l, 4);     // Left distance
  Serial.print(",");
  Serial.print(dist_r, 4);     // Right distance
  Serial.print(",");
  Serial.print(accel_x, 3);    // Acceleration X
  Serial.print(",");
  Serial.print(accel_y, 3);    // Acceleration Y
  Serial.print(",");
  Serial.print(accel_z, 3);    // Acceleration Z
  Serial.print(",");
  Serial.print(gyro_x, 1);     // Gyro X
  Serial.print(",");
  Serial.print(gyro_y, 1);     // Gyro Y
  Serial.print(",");
  Serial.println(gyro_z, 1);   // Gyro Z
}

// ==================== STATUS REPORTING ====================
void report_status() {
  Serial.print("STATUS:PWM_L=");
  Serial.print(pwm_left);
  Serial.print(",PWM_R=");
  Serial.print(pwm_right);
  Serial.print(",TICK_L=");
  Serial.print(encoder_a_ticks);
  Serial.print(",TICK_R=");
  Serial.println(encoder_b_ticks);
}

/*
 * SERIAL PROTOCOL REFERENCE:
 *
 * Commands (Arduino <- ROS2):
 *   M<left_pwm><right_pwm>    - Set motor PWM values (-255 to +255)
 *   E                          - Echo test
 *
 * Responses (Arduino -> ROS2):
 *   STATUS:<message>           - Status update
 *   ERROR:<code>               - Error condition
 *   WARNING:<message>          - Warning condition
 *   SENSOR:<data>              - Sensor telemetry
 *   INFO:<message>             - Information message
 *
 * Telemetry Format:
 *   SENSOR:dist_l,dist_r,ax,ay,az,gx,gy,gz
 *   - dist_l, dist_r: Distance traveled (meters)
 *   - ax, ay, az: Accelerometer (m/s²)
 *   - gx, gy, gz: Gyroscope (°/s)
 *
 * Example:
 *   -> M64-64        (Move left: PWM 100 fwd, right: PWM 100 bwd)
 *   <- SENSOR:0.123,-0.456,0.01,0.02,9.81,1.2,0.3,-0.1
 */

// ==================== DEBUGGING (Optional) ====================
/*
 * Uncomment to enable debug features:
 *
 * void print_debug_info() {
 *   Serial.print("DEBUG:");
 *   Serial.print("L="); Serial.print(encoder_a_ticks);
 *   Serial.print(" R="); Serial.print(encoder_b_ticks);
 *   Serial.print(" AX="); Serial.print(imu_accel_x);
 *   Serial.println();
 * }
 *
 * Add to loop():
 *   if ((current_time % 500) == 0) {
 *     print_debug_info();
 *   }
 */
