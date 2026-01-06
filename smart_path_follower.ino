/*
 * Smart Path Follower - Arduino Code
 * ===================================
 * Advanced path following with IMU heading correction.
 * 
 * Features:
 * - MPU6050 IMU for heading tracking (yaw angle)
 * - Real-time heading correction during playback
 * - Ultrasonic obstacle detection
 * - Watchdog safety timer
 * 
 * Works with: smart_record.py and smart_playback.py
 * 
 * --- WIRING ---
 * L298N Motor Driver:
 *   ENA -> Pin 2 (PWM)
 *   IN1 -> Pin 22
 *   IN2 -> Pin 23
 *   IN3 -> Pin 24
 *   IN4 -> Pin 25
 *   ENB -> Pin 3 (PWM)
 * 
 * HC-SR04 Ultrasonic:
 *   Trig -> Pin 4
 *   Echo -> Pin 5
 * 
 * MPU6050 IMU (I2C):
 *   SDA -> Pin 20 (SDA)
 *   SCL -> Pin 21 (SCL)
 *   VCC -> 3.3V
 *   GND -> GND
 */

#include <Wire.h>

// --- I2C TIMEOUT ---
const unsigned long I2C_TIMEOUT = 1000;  // 1ms timeout for I2C operations
bool imuAvailable = true;                 // Track if IMU is responding
unsigned long lastIMUSuccess = 0;         // Last successful IMU read

// --- MOTOR PINS ---
const int ENA = 2;
const int IN1 = 22;
const int IN2 = 23;
const int IN3 = 24;
const int IN4 = 25;
const int ENB = 3;

// --- ULTRASONIC PINS ---
const int TRIG_PIN = 4;
const int ECHO_PIN = 5;

// --- MPU6050 ---
const int MPU_ADDR = 0x68;
float yaw = 0.0;
float gyroZOffset = 0.0;
unsigned long lastIMUTime = 0;

// --- CONFIGURATION ---
const int SAFETY_DISTANCE = 5;          // Emergency stop distance (cm)
const unsigned long CMD_TIMEOUT = 500;   // Stop motors if no command for 500ms
const unsigned long SENSOR_INTERVAL = 50; // Send sensor data every 50ms (20Hz)

// --- HEADING CORRECTION PID ---
float Kp = 2.0;   // Proportional gain for heading correction
float Ki = 0.0;   // Integral gain (usually 0 for simple correction)
float Kd = 0.5;   // Derivative gain for smoother correction

float lastHeadingError = 0;
float headingIntegral = 0;

// --- VARIABLES ---
char receivedChars[64];
byte charIndex = 0;
boolean receiving = false;
boolean newCommand = false;

unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;
unsigned long receiveStartTime = 0;  // Track when we started receiving
int currentDistance = 999;

// Receive timeout (if we start receiving but don't get '>' in 100ms, reset)
const unsigned long RECEIVE_TIMEOUT = 100;

// Target heading for correction mode
float targetHeading = 0.0;
bool headingCorrectionEnabled = false;
int baseLeftSpeed = 0;
int baseRightSpeed = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Motor pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Initialize MPU6050
    initMPU6050();
    
    // Calibrate gyro (robot must be stationary!)
    calibrateGyro();
    
    // Start with motors stopped
    stopMotors();
    
    Serial.println("SMART_PATH_FOLLOWER_READY");
}

void loop() {
    // 1. Update IMU (always running for heading tracking)
    updateIMU();
    
    // 2. Receive commands
    receiveCommand();
    
    if (newCommand) {
        parseAndExecute();
        newCommand = false;
        lastCommandTime = millis();
    }
    
    // 3. Heading correction (if enabled and moving)
    if (headingCorrectionEnabled && (baseLeftSpeed != 0 || baseRightSpeed != 0)) {
        applyHeadingCorrection();
    }
    
    // 4. Watchdog - stop if no commands received
    unsigned long timeSinceLastCmd = millis() - lastCommandTime;
    if (timeSinceLastCmd > CMD_TIMEOUT) {
        if (baseLeftSpeed != 0 || baseRightSpeed != 0) {
            // Only print once when stopping
            stopMotors();
            headingCorrectionEnabled = false;
        }
    }
    
    // 5. Try to recover I2C if IMU has been failing for too long
    if (!imuAvailable && (millis() - lastIMUSuccess > 5000)) {
        resetI2C();
        initMPU6050();
        lastIMUSuccess = millis();  // Reset timer to avoid constant resets
    }
    
    // 6. Read and send sensor data periodically
    if (millis() - lastSensorTime > SENSOR_INTERVAL) {
        readDistance();
        sendSensorData();
        lastSensorTime = millis();
    }
    
    // Small delay to prevent overwhelming the loop
    delayMicroseconds(100);
}

// ==================== MPU6050 FUNCTIONS ====================

void initMPU6050() {
    // Set I2C timeout (some Arduino cores support this)
    Wire.setWireTimeout(I2C_TIMEOUT, true);  // timeout in microseconds, reset on timeout
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Wake up
    byte error = Wire.endTransmission(true);
    
    if (error != 0) {
        Serial.println("WARNING: MPU6050 not found!");
        imuAvailable = false;
        return;
    }
    
    // Set gyro range to ±250°/s for better precision
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x00);  // ±250°/s
    Wire.endTransmission(true);
    
    imuAvailable = true;
    lastIMUSuccess = millis();
    delay(100);
    Serial.println("MPU6050 initialized OK");
}

void calibrateGyro() {
    Serial.println("Calibrating gyro... keep robot still!");
    
    float sum = 0;
    int samples = 500;
    int validSamples = 0;
    
    for (int i = 0; i < samples; i++) {
        int16_t gyroZ = readGyroZ();
        if (imuAvailable) {
            sum += gyroZ / 131.0;  // Convert to °/s
            validSamples++;
        }
        delay(2);
    }
    
    if (validSamples > 0) {
        gyroZOffset = sum / validSamples;
        Serial.print("Gyro Z offset: ");
        Serial.println(gyroZOffset);
        Serial.print("Valid samples: ");
        Serial.println(validSamples);
    } else {
        Serial.println("WARNING: IMU not responding!");
        gyroZOffset = 0;
        imuAvailable = false;
    }
    
    lastIMUTime = micros();
}

// Safe I2C read for gyro Z with timeout
int16_t readGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);  // GYRO_ZOUT_H
    byte error = Wire.endTransmission(false);
    
    if (error != 0) {
        // I2C transmission failed
        imuAvailable = false;
        return 0;
    }
    
    byte bytesReceived = Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
    
    if (bytesReceived != 2) {
        // Didn't receive expected bytes
        imuAvailable = false;
        // Flush any partial data
        while (Wire.available()) Wire.read();
        return 0;
    }
    
    // Wait for data with timeout
    unsigned long startWait = micros();
    while (Wire.available() < 2) {
        if (micros() - startWait > I2C_TIMEOUT) {
            imuAvailable = false;
            return 0;
        }
    }
    
    imuAvailable = true;
    lastIMUSuccess = millis();
    return Wire.read() << 8 | Wire.read();
}

void updateIMU() {
    // Skip if IMU has been failing (try again every 1 second)
    if (!imuAvailable && (millis() - lastIMUSuccess < 1000)) {
        return;
    }
    
    // Read gyroscope Z axis with timeout protection
    int16_t gyroZ = readGyroZ();
    
    if (!imuAvailable) {
        // IMU read failed, skip this update
        return;
    }
    
    float gyroZRate = (gyroZ / 131.0) - gyroZOffset;  // °/s
    
    // Integrate to get yaw angle
    unsigned long currentTime = micros();
    float dt = (currentTime - lastIMUTime) / 1000000.0;  // Convert to seconds
    lastIMUTime = currentTime;
    
    // Only integrate if rate is significant (reduce drift)
    if (abs(gyroZRate) > 0.5) {
        yaw += gyroZRate * dt;
    }
    
    // Normalize yaw to -180 to 180
    while (yaw > 180) yaw -= 360;
    while (yaw < -180) yaw += 360;
}

void resetYaw() {
    yaw = 0.0;
    headingIntegral = 0.0;
    lastHeadingError = 0.0;
}

// ==================== HEADING CORRECTION ====================

void applyHeadingCorrection() {
    // Calculate heading error
    float error = targetHeading - yaw;
    
    // Normalize error to -180 to 180
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    // PID calculation
    headingIntegral += error;
    headingIntegral = constrain(headingIntegral, -100, 100);  // Anti-windup
    
    float derivative = error - lastHeadingError;
    lastHeadingError = error;
    
    float correction = Kp * error + Ki * headingIntegral + Kd * derivative;
    correction = constrain(correction, -100, 100);
    
    // Apply correction to motors
    int leftSpeed = baseLeftSpeed + (int)correction;
    int rightSpeed = baseRightSpeed - (int)correction;
    
    // Only apply correction for forward/backward movement
    if ((baseLeftSpeed > 0 && baseRightSpeed > 0) || 
        (baseLeftSpeed < 0 && baseRightSpeed < 0)) {
        setMotors(leftSpeed, rightSpeed);
    }
}

// ==================== COMMAND HANDLING ====================

void receiveCommand() {
    // Protocol: <CMD,ARG1,ARG2,...>
    // Examples:
    //   <M,200,200>        - Motor command (left, right)
    //   <H,45.5>           - Set target heading for correction
    //   <C,1>              - Enable heading correction
    //   <C,0>              - Disable heading correction
    //   <R>                - Reset yaw to 0
    //   <P,2.0,0.0,0.5>    - Set PID gains
    
    // Check for receive timeout (parser stuck in receiving state)
    if (receiving && (millis() - receiveStartTime > RECEIVE_TIMEOUT)) {
        receiving = false;
        charIndex = 0;
        // Flush any remaining garbage
        while (Serial.available() > 0) {
            Serial.read();
        }
    }
    
    // Prevent buffer overflow - if too much data, flush it
    if (Serial.available() > 60) {
        while (Serial.available() > 0) {
            Serial.read();
        }
        receiving = false;
        charIndex = 0;
        return;
    }
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '<') {
            receiving = true;
            charIndex = 0;
            receiveStartTime = millis();  // Start timeout timer
        }
        else if (c == '>') {
            receiving = false;
            receivedChars[charIndex] = '\0';
            newCommand = true;
        }
        else if (receiving) {
            if (charIndex < 63) {
                receivedChars[charIndex] = c;
                charIndex++;
            } else {
                // Buffer overflow - reset
                receiving = false;
                charIndex = 0;
            }
        }
        // Ignore characters outside of < > brackets
    }
}

void parseAndExecute() {
    char cmdType = receivedChars[0];
    
    if (cmdType == 'M' || cmdType == 'm') {
        // Motor command: M,left,right
        parseMotorCommand();
    }
    else if (cmdType == 'H' || cmdType == 'h') {
        // Set target heading: H,degrees
        char* token = strtok(receivedChars + 2, ",");
        if (token != NULL) {
            targetHeading = atof(token);
        }
    }
    else if (cmdType == 'C' || cmdType == 'c') {
        // Enable/disable correction: C,1 or C,0
        char* token = strtok(receivedChars + 2, ",");
        if (token != NULL) {
            headingCorrectionEnabled = (atoi(token) == 1);
            if (headingCorrectionEnabled) {
                // Reset PID state when enabling
                headingIntegral = 0;
                lastHeadingError = 0;
            }
        }
    }
    else if (cmdType == 'R' || cmdType == 'r') {
        // Reset yaw
        resetYaw();
    }
    else if (cmdType == 'P' || cmdType == 'p') {
        // Set PID gains: P,Kp,Ki,Kd
        char* token = strtok(receivedChars + 2, ",");
        if (token != NULL) {
            Kp = atof(token);
            token = strtok(NULL, ",");
            if (token != NULL) {
                Ki = atof(token);
                token = strtok(NULL, ",");
                if (token != NULL) {
                    Kd = atof(token);
                }
            }
        }
    }
    else {
        // Legacy format: LEFT,RIGHT (for compatibility)
        parseLegacyCommand();
    }
}

void parseMotorCommand() {
    // Parse "M,LEFT,RIGHT"
    char* token = strtok(receivedChars + 2, ",");  // Skip "M,"
    if (token != NULL) {
        baseLeftSpeed = atoi(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
            baseRightSpeed = atoi(token);
        }
    }
    
    // Safety: Don't move forward if obstacle too close
    if (baseLeftSpeed > 0 && baseRightSpeed > 0 && currentDistance < SAFETY_DISTANCE) {
        stopMotors();
        return;
    }
    
    setMotors(baseLeftSpeed, baseRightSpeed);
}

void parseLegacyCommand() {
    // Parse "LEFT,RIGHT" (old format for compatibility)
    int leftSpeed = 0;
    int rightSpeed = 0;
    
    char* token = strtok(receivedChars, ",");
    if (token != NULL) {
        leftSpeed = atoi(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
            rightSpeed = atoi(token);
        }
    }
    
    baseLeftSpeed = leftSpeed;
    baseRightSpeed = rightSpeed;
    
    // Safety check
    if (leftSpeed > 0 && rightSpeed > 0 && currentDistance < SAFETY_DISTANCE) {
        stopMotors();
        return;
    }
    
    setMotors(leftSpeed, rightSpeed);
}

// ==================== MOTOR CONTROL ====================

void setMotors(int leftSpeed, int rightSpeed) {
    // LEFT MOTOR (Motor A)
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else if (leftSpeed < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        leftSpeed = -leftSpeed;
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
    
    // RIGHT MOTOR (Motor B)
    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else if (rightSpeed < 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        rightSpeed = -rightSpeed;
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
    baseLeftSpeed = 0;
    baseRightSpeed = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// ==================== SENSORS ====================

void readDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    
    if (duration > 0) {
        currentDistance = duration * 0.0343 / 2;
    } else {
        currentDistance = 999;
    }
}

void sendSensorData() {
    // Format: START,distance,yaw,imuOK,END
    Serial.print("START,");
    Serial.print(currentDistance);
    Serial.print(",");
    Serial.print(yaw, 2);
    Serial.print(",");
    Serial.print(imuAvailable ? 1 : 0);  // Report IMU status
    Serial.println(",END");
}

// Reset I2C bus if it gets stuck
void resetI2C() {
    Wire.end();
    delay(10);
    Wire.begin();
    Wire.setWireTimeout(I2C_TIMEOUT, true);
    Serial.println("I2C bus reset");
}
