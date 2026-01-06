#!/usr/bin/env python3
"""
Command-Based Robot Driver
===========================
Define a sequence of simple commands to drive the robot.

Commands:
    ("forward", seconds)   - Drive forward for X seconds
    ("backward", seconds)  - Drive backward for X seconds
    ("left", degrees)      - Turn left X degrees (uses IMU)
    ("right", degrees)     - Turn right X degrees (uses IMU)
    ("wait", seconds)      - Wait/pause for X seconds

Example:
    COMMANDS = [
        ("forward", 2),      # Go forward 2 seconds
        ("left", 90),        # Turn left 90 degrees
        ("forward", 2),      # Go forward 2 seconds
        ("right", 90),       # Turn right 90 degrees
        ("forward", 2),      # Go forward 2 seconds
    ]

Run on Jetson Nano:
    python3 command_drive.py

Press Ctrl+C to stop immediately.
"""

import serial
import time
import os
import sys
import math

# ============================================================================
# CONFIGURATION
# ============================================================================

# Define your route here!
COMMANDS = [
    # === GO TO TARGET ===
    ("forward", 2),      # Go forward 2 seconds
    ("left", 90),        # Turn left 90 degrees
    ("forward", 2),      # Go forward 2 seconds
    ("right", 90),       # Turn right 90 degrees
    ("forward", 2),      # Go forward 2 seconds
    
    # === ARRIVED AT TARGET - WAIT ===
    ("wait", 10),        # Wait 10 seconds at target
    
    # === RETURN PATH ===
    ("right", 90),       # Turn right 90 degrees
    ("forward", 2),      # Move forward 2 seconds
    ("right", 90),       # Turn right 90 degrees again
    ("forward", 3),      # Move forward 3 seconds
]

# Speed settings
DRIVE_SPEED = 200        # Speed for forward/backward (0-255)
TURN_SPEED = 250       # Speed for turning (0-255)

# Safety
OBSTACLE_STOP_DIST = 15  # Stop if obstacle closer than this (cm)
CHECK_OBSTACLES = True   # Set False to disable obstacle checking

# Serial
SERIAL_PORT = None       # Auto-detect if None
BAUD_RATE = 115200

# Turn calibration (adjust if turns are inaccurate)
# Time to complete a 360-degree turn at TURN_SPEED (measure and adjust!)
TURN_360_TIME = 2.5      # Seconds for full 360° rotation (calibrate this!)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def find_serial_port():
    """Auto-detect Arduino serial port"""
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    for port in ports:
        if os.path.exists(port):
            return port
    return None

class RobotController:
    def __init__(self, serial_port):
        self.ser = serial_port
        self.distance = 999
        self.heading = 0  # Current heading from IMU (yaw)
        self.imu_data = [0, 0, 0, 0, 0, 0]  # ax, ay, az, gx, gy, gz
        
    def send_motors(self, left, right):
        """Send motor command"""
        cmd = f"<{int(left)},{int(right)}>"
        try:
            self.ser.write(cmd.encode('utf-8'))
        except:
            pass
    
    def stop(self):
        """Stop motors"""
        self.send_motors(0, 0)
        
    def read_sensors(self):
        """Read sensor data from Arduino"""
        try:
            if self.ser.in_waiting > 0:
                lines = self.ser.read_all().decode('utf-8', errors='ignore').split('\n')
                for line in lines:
                    line = line.strip()
                    if line.startswith("START") and line.endswith("END"):
                        parts = line.split(',')
                        if len(parts) >= 10:
                            self.distance = float(parts[1])
                            # IMU data: parts[4:10] = ax, ay, az, gx, gy, gz
                            self.imu_data = [float(parts[i]) for i in range(4, 10)]
        except Exception as e:
            pass
        
        # Handle invalid distance readings
        if self.distance <= 0 or self.distance >= 999:
            self.distance = 999
            
    def get_yaw_rate(self):
        """Get yaw rotation rate from gyroscope (degrees/second)"""
        # gz is the yaw rate (rotation around vertical axis)
        return self.imu_data[5] if len(self.imu_data) > 5 else 0
    
    def is_obstacle_ahead(self):
        """Check if obstacle is in front"""
        if not CHECK_OBSTACLES:
            return False
        return self.distance < OBSTACLE_STOP_DIST
    
    def forward(self, seconds):
        """Drive forward for specified seconds"""
        print(f"  ➡️  Forward for {seconds}s at speed {DRIVE_SPEED}")
        start = time.time()
        
        while time.time() - start < seconds:
            self.read_sensors()
            
            if self.is_obstacle_ahead():
                self.stop()
                print(f"  ⚠️  Obstacle detected at {self.distance:.0f}cm! Waiting...")
                while self.is_obstacle_ahead():
                    self.read_sensors()
                    time.sleep(0.1)
                print(f"  ✅ Path clear, resuming...")
                start = time.time() + (time.time() - start)  # Don't count wait time
            
            self.send_motors(DRIVE_SPEED, DRIVE_SPEED)
            time.sleep(0.05)
        
        self.stop()
        print(f"  ✅ Forward complete")
        
    def backward(self, seconds):
        """Drive backward for specified seconds"""
        print(f"  ⬅️  Backward for {seconds}s at speed {DRIVE_SPEED}")
        start = time.time()
        
        while time.time() - start < seconds:
            self.send_motors(-DRIVE_SPEED, -DRIVE_SPEED)
            time.sleep(0.05)
        
        self.stop()
        print(f"  ✅ Backward complete")
        
    def turn_left(self, degrees):
        """Turn left by specified degrees using time-based control"""
        print(f"  ↩️  Turn LEFT {degrees}°")
        self._turn(degrees, direction="left")
        
    def turn_right(self, degrees):
        """Turn right by specified degrees using time-based control"""
        print(f"  ↪️  Turn RIGHT {degrees}°")
        self._turn(degrees, direction="right")
        
    def _turn(self, target_degrees, direction):
        """Execute turn using time-based estimation"""
        # Calculate turn time based on calibration
        turn_time = (target_degrees / 360.0) * TURN_360_TIME
        
        print(f"      (turning for {turn_time:.2f}s)")
        
        # Set motor direction for pivot turn
        # Left turn: left wheel backward, right wheel forward
        # Right turn: left wheel forward, right wheel backward
        if direction == "left":
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED
        else:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED
        
        # Execute turn
        start = time.time()
        while time.time() - start < turn_time:
            self.send_motors(left_speed, right_speed)
            time.sleep(0.02)
        
        self.stop()
        print(f"  ✅ Turn complete")
        time.sleep(0.3)  # Brief pause to stabilize
        
    def wait(self, seconds):
        """Wait/pause for specified seconds"""
        print(f"  ⏸️  Waiting {seconds}s...")
        time.sleep(seconds)
        print(f"  ✅ Wait complete")

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("=" * 60)
    print("   COMMAND-BASED ROBOT DRIVER")
    print("=" * 60)
    
    # Find serial port
    port = SERIAL_PORT or find_serial_port()
    if not port:
        print("❌ ERROR: No Arduino found!")
        sys.exit(1)
    
    print(f"Connecting to Arduino on {port}...")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to reset
        print(f"✅ Connected to {port}")
    except Exception as e:
        print(f"❌ Serial error: {e}")
        sys.exit(1)
    
    # Create controller
    robot = RobotController(ser)
    
    # Clear serial buffer
    ser.read_all()
    
    print("\n" + "=" * 60)
    print("   EXECUTING COMMAND SEQUENCE")
    print(f"   Total commands: {len(COMMANDS)}")
    print("=" * 60 + "\n")
    
    try:
        for i, (cmd, value) in enumerate(COMMANDS):
            print(f"[{i+1}/{len(COMMANDS)}] Command: {cmd.upper()} {value}")
            
            if cmd == "forward":
                robot.forward(value)
            elif cmd == "backward":
                robot.backward(value)
            elif cmd == "left":
                robot.turn_left(value)
            elif cmd == "right":
                robot.turn_right(value)
            elif cmd == "wait":
                robot.wait(value)
            else:
                print(f"  ⚠️  Unknown command: {cmd}")
            
            time.sleep(0.3)  # Brief pause between commands
        
        print("\n" + "=" * 60)
        print("   ✅ ALL COMMANDS COMPLETED!")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user!")
    
    finally:
        robot.stop()
        ser.close()
        print("Motors stopped, serial closed.")

if __name__ == "__main__":
    main()
