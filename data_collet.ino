/*
 * Paper Courier Bot - Data Collection Slave
 * -----------------------------------------
 * Role: 
 * 1. Listen: Accepts motor commands "<LEFT,RIGHT>" from Jetson
 * 2. Report: Sends Sensor Data "START,dist,lat,lon,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,END"
 * 3. Safety: Stops if Ultrasonic < 5cm
 * * --- WIRING PINOUT ---
 * 1. L298N Motor Driver:
 * - ENA -> Pin 2 (PWM)
 * - IN1 -> Pin 22, IN2 -> Pin 23
 * - IN3 -> Pin 24, IN4 -> Pin 25
 * - ENB -> Pin 3 (PWM)
 * * 2. HC-SR04 Ultrasonic:
 * - Trig -> Pin 4
 * - Echo -> Pin 5
 * * 3. GPS Module (NEO-6M):
 * - TX  -> Pin 19 (RX1)  [Hardwired for Serial1]
 * - RX  -> Pin 18 (TX1)  [Hardwired for Serial1]
 * * 4. IMU (MPU6050):
 * - SDA -> Pin 20        [Hardwired for I2C]
 * - SCL -> Pin 21        [Hardwired for I2C]
 */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <TinyGPS++.h>

// --- PINS ---
const int ENA = 2; const int IN1 = 22; const int IN2 = 23;
const int ENB = 3; const int IN3 = 24; const int IN4 = 25;
const int TRIG_PIN = 4; 
const int ECHO_PIN = 5;
#define GPS_BAUD 9600 

TinyGPSPlus gps;
MPU6050 mpu; 

// Variables
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
unsigned long lastSensorTime = 0;
const int SENSOR_INTERVAL = 50; // Send sensors every 50ms (20Hz) for real-time driving
int currentDistance = 999;
int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(115200); 
  Serial1.begin(GPS_BAUD);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  
  Wire.begin();
  mpu.initialize();
  Serial.println("SYSTEM READY");
}

void loop() {
  // 1. Receive Motor Commands
  recvWithStartEndMarkers();
  if (newData) {
    parseData();
    newData = false;
  }

  // 2. Read GPS
  while (Serial1.available() > 0) gps.encode(Serial1.read());

  // 3. Send Sensor Data (20Hz for real-time response)
  if (millis() - lastSensorTime > SENSOR_INTERVAL) {
    readAndReport();
    lastSensorTime = millis();
  }
}

void readAndReport() {
  // Ultrasonic
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 15000); 
  int dist = (dur > 0) ? (dur * 0.0343 / 2) : 999;
  currentDistance = dist;

  // GPS Data
  double lat_val = (gps.location.isValid()) ? gps.location.lat() : 0.0;
  double lon_val = (gps.location.isValid()) ? gps.location.lng() : 0.0;

  // IMU Data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to physics units (m/s^2 and deg/s)
  float acc_x = (ax / 16384.0) * 9.81;
  float acc_y = (ay / 16384.0) * 9.81;
  float acc_z = (az / 16384.0) * 9.81;
  float gyro_x = gx / 131.0;
  float gyro_y = gy / 131.0;
  float gyro_z = gz / 131.0;

  // Packet: START, dist, lat, lon, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, END
  Serial.print("START,");
  Serial.print(dist); Serial.print(",");
  Serial.print(lat_val, 6); Serial.print(",");
  Serial.print(lon_val, 6); Serial.print(",");
  Serial.print(acc_x); Serial.print(",");
  Serial.print(acc_y); Serial.print(",");
  Serial.print(acc_z); Serial.print(",");
  Serial.print(gyro_x); Serial.print(",");
  Serial.print(gyro_y); Serial.print(",");
  Serial.print(gyro_z); 
  Serial.println(",END");
}

void setMotors(int leftSpeed, int rightSpeed) {
  // SAFETY STOP (5cm)
  if (leftSpeed > 0 && rightSpeed > 0 && currentDistance < 5) {
    leftSpeed = 0; rightSpeed = 0;
  }

  // Drive Motors
  if (leftSpeed > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftSpeed = -leftSpeed; }
  analogWrite(ENA, constrain(leftSpeed, 0, 255));

  if (rightSpeed > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightSpeed = -rightSpeed; }
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

// --- PARSING ---
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<'; char endMarker = '>'; char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc; ndx++;
        if (ndx >= numChars) ndx = numChars - 1;
      } else { receivedChars[ndx] = '\0'; recvInProgress = false; newData = true; }
    } else if (rc == startMarker) { recvInProgress = true; ndx = 0; }
  }
}
void parseData() {
  char * strtokIndx; 
  strtokIndx = strtok(receivedChars, ","); int leftSpd = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); int rightSpd = atoi(strtokIndx);
  setMotors(leftSpd, rightSpd);
}