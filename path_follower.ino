/*
 * Path Follower Arduino Code
 * ==========================
 * Optimized version for path recording and playback.
 * Works with: record_path.py and playback_path.py
 * 
 * Features:
 * - Fast motor response (no sensor delays during playback)
 * - Ultrasonic safety stop
 * - Watchdog timer (stops if no command received)
 * - Simple and reliable
 * 
 * --- WIRING (Same as data_collet.ino) ---
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
 */

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

// --- CONFIGURATION ---
const int SAFETY_DISTANCE = 10;        // Emergency stop distance (cm)
const unsigned long CMD_TIMEOUT = 500;  // Stop motors if no command for 500ms
const unsigned long SENSOR_INTERVAL = 100; // Send sensor data every 100ms

// --- VARIABLES ---
char receivedChars[32];
byte charIndex = 0;
boolean receiving = false;
boolean newCommand = false;

unsigned long lastCommandTime = 0;
unsigned long lastSensorTime = 0;
int currentDistance = 999;

void setup() {
    Serial.begin(115200);
    
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
    
    // Start with motors stopped
    stopMotors();
    
    Serial.println("PATH_FOLLOWER_READY");
}

void loop() {
    // 1. Receive commands
    receiveCommand();
    
    if (newCommand) {
        parseAndExecute();
        newCommand = false;
        lastCommandTime = millis();
    }
    
    // 2. Watchdog - stop if no commands received
    if (millis() - lastCommandTime > CMD_TIMEOUT) {
        stopMotors();
    }
    
    // 3. Read and send sensor data periodically
    if (millis() - lastSensorTime > SENSOR_INTERVAL) {
        readDistance();
        sendSensorData();
        lastSensorTime = millis();
        
        // Safety stop if too close (only when moving forward)
        // Note: This is handled in parseAndExecute too
    }
}

void receiveCommand() {
    // Protocol: <LEFT,RIGHT>
    // Example: <200,200> or <-150,150>
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '<') {
            receiving = true;
            charIndex = 0;
        }
        else if (c == '>') {
            receiving = false;
            receivedChars[charIndex] = '\0';
            newCommand = true;
        }
        else if (receiving) {
            if (charIndex < 31) {
                receivedChars[charIndex] = c;
                charIndex++;
            }
        }
    }
}

void parseAndExecute() {
    // Parse "LEFT,RIGHT" from receivedChars
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
    
    // Safety: Don't move forward if obstacle too close
    if (leftSpeed > 0 && rightSpeed > 0 && currentDistance < SAFETY_DISTANCE) {
        stopMotors();
        return;
    }
    
    setMotors(leftSpeed, rightSpeed);
}

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
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void readDistance() {
    // Send ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo (with timeout)
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);  // 20ms timeout
    
    if (duration > 0) {
        currentDistance = duration * 0.0343 / 2;  // Convert to cm
    } else {
        currentDistance = 999;  // No echo = no obstacle
    }
}

void sendSensorData() {
    // Simple format for playback script
    Serial.print("START,");
    Serial.print(currentDistance);
    Serial.println(",END");
}
