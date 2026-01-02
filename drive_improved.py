#!/usr/bin/env python3
"""
Improved Autonomous Driving Script
===================================
This script fixes several issues from the original drive_sequenced.py:

1. Resolution mismatch: Uses 640x480 to match training (train.py)
2. Better error handling and debugging output
3. Smoother motor control with ramping
4. Configurable parameters
5. Visual feedback option

Run on Jetson Nano:
    python3 drive_improved.py

Press 'q' to quit (if display available) or Ctrl+C
"""

import cv2
import numpy as np
import serial
import time
import torch
import torch.nn as nn
import os
import sys
import argparse

# ============================================================================
# CONFIGURATION - ADJUST THESE VALUES
# ============================================================================

# Model and Scaler
MODEL_PATH = "vga_pilot.pth"  # Change to your model file

# Serial Connection
SERIAL_PORT = None  # Auto-detect if None
BAUD_RATE = 115200

# Driving Parameters
BASE_SPEED = 120      # Base motor speed (0-255). Start low!
STEER_GAIN = 150      # How much steering affects motors. Start low!
MAX_SPEED = 200       # Maximum motor speed
MIN_SPEED = 60        # Minimum motor speed (below this motors may stall)

# Safety
OBSTACLE_STOP_DIST = 20   # Stop if obstacle closer than this (cm)
OBSTACLE_SLOW_DIST = 50   # Slow down if obstacle closer than this (cm)

# Image Settings (MUST MATCH TRAINING!)
IMG_WIDTH = 640
IMG_HEIGHT = 480

# Debug
SHOW_PREVIEW = True   # Show camera preview (requires display)
VERBOSE = True        # Print debug info

# Scaler Values - Updated from scaler.py extraction
SCALER_MEAN = [148.32751091703057, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
SCALER_SCALE = [239.56219562991816, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# ============================================================================
# HELPER CLASSES
# ============================================================================

class ManualScaler:
    """Replicates sklearn StandardScaler transform without needing sklearn"""
    def __init__(self, mean, scale):
        self.mean = np.array(mean)
        self.scale = np.array(scale)
        
    def transform(self, X):
        return (X - self.mean) / self.scale

class MotorController:
    """Smooth motor control with ramping"""
    def __init__(self, serial_conn, ramp_rate=20):
        self.ser = serial_conn
        self.ramp_rate = ramp_rate
        self.current_left = 0
        self.current_right = 0
        
    def send_raw(self, left, right):
        """Send raw motor command"""
        cmd = f"<{int(left)},{int(right)}>"
        try:
            self.ser.write(cmd.encode('utf-8'))
        except:
            pass
    
    def set_motors(self, target_left, target_right, instant=False):
        """Set motor speeds with optional ramping"""
        if instant:
            self.current_left = target_left
            self.current_right = target_right
        else:
            # Ramp towards target
            if target_left > self.current_left:
                self.current_left = min(target_left, self.current_left + self.ramp_rate)
            else:
                self.current_left = max(target_left, self.current_left - self.ramp_rate)
                
            if target_right > self.current_right:
                self.current_right = min(target_right, self.current_right + self.ramp_rate)
            else:
                self.current_right = max(target_right, self.current_right - self.ramp_rate)
        
        self.send_raw(self.current_left, self.current_right)
        
    def stop(self):
        """Emergency stop"""
        self.current_left = 0
        self.current_right = 0
        self.send_raw(0, 0)

# ============================================================================
# MODEL ARCHITECTURE - MUST MATCH TRAINING!
# ============================================================================

class SequencedPilot(nn.Module):
    """
    Neural network for autonomous driving.
    Architecture must exactly match the training script (train.py).
    """
    def __init__(self):
        super(SequencedPilot, self).__init__()
        
        # Vision Branch (CNN) - For 640x480 input
        # Each Conv2d with stride=2 roughly halves dimensions
        self.cnn = nn.Sequential(
            # Input: 3 x 480 x 640
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),   # -> 24 x 238 x 318
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),  # -> 36 x 117 x 157
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),  # -> 48 x 57 x 77
            nn.Conv2d(48, 64, 3, stride=2), nn.ReLU(),  # -> 64 x 28 x 38
            nn.Conv2d(64, 64, 3, stride=2), nn.ReLU(),  # -> 64 x 13 x 18
            nn.Flatten()  # -> 14976
        )
        
        # Sensor Branch (MLP)
        self.mlp = nn.Sequential(
            nn.Linear(9, 64), nn.ReLU(),
            nn.Linear(64, 32), nn.ReLU()
        )
        
        # Fusion Layer
        # CNN output (14976) + MLP output (32) = 15008
        self.combined = nn.Sequential(
            nn.Linear(15008, 100), nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 1)  # Output: Steering value
        )

    def forward(self, img, sens):
        x1 = self.cnn(img)
        x2 = self.mlp(sens)
        x = torch.cat((x1, x2), dim=1)
        return self.combined(x)

# ============================================================================
# MAIN FUNCTIONS
# ============================================================================

def find_serial_port():
    """Auto-detect Arduino serial port"""
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    for port in ports:
        if os.path.exists(port):
            return port
    return None

def create_gstreamer_pipeline(width, height):
    """Create GStreamer pipeline for CSI camera"""
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        f"nvvidconv ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=true"
    )

def preprocess_image(frame):
    """Preprocess image for model input"""
    # Convert BGR to YUV (same as training)
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    # Normalize to 0-1
    img = img / 255.0
    # Transpose from HWC to CHW format
    img = img.transpose((2, 0, 1))
    return img

def main():
    parser = argparse.ArgumentParser(description='Autonomous Robot Driver')
    parser.add_argument('--model', default=MODEL_PATH, help='Path to model file')
    parser.add_argument('--port', default=SERIAL_PORT, help='Serial port')
    parser.add_argument('--speed', type=int, default=BASE_SPEED, help='Base speed')
    parser.add_argument('--gain', type=int, default=STEER_GAIN, help='Steering gain')
    parser.add_argument('--no-preview', action='store_true', help='Disable preview')
    args = parser.parse_args()
    
    # Update from args
    model_path = args.model
    base_speed = args.speed
    steer_gain = args.gain
    show_preview = not args.no_preview and SHOW_PREVIEW
    
    print("=" * 60)
    print("   AUTONOMOUS ROBOT - IMPROVED DRIVER")
    print("=" * 60)
    
    # Setup device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    
    # Load model
    print(f"Loading model: {model_path}")
    try:
        model = SequencedPilot().to(device)
        model.load_state_dict(torch.load(model_path, map_location=device))
        model.eval()
        print("✅ Model loaded successfully!")
    except FileNotFoundError:
        print(f"❌ ERROR: Model file '{model_path}' not found!")
        print("   Make sure you've copied the trained model to this directory.")
        sys.exit(1)
    except Exception as e:
        print(f"❌ ERROR loading model: {e}")
        print("   This usually means the model architecture doesn't match.")
        print("   Check that IMG_WIDTH and IMG_HEIGHT match your training.")
        sys.exit(1)
    
    # Initialize scaler
    scaler = ManualScaler(SCALER_MEAN, SCALER_SCALE)
    print("✅ Scaler initialized")
    
    # Setup serial connection
    port = args.port or find_serial_port()
    if not port:
        print("❌ ERROR: No Arduino found!")
        print("   Check USB connection and try: ls /dev/ttyUSB* /dev/ttyACM*")
        sys.exit(1)
    
    print(f"Connecting to Arduino on {port}...")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
        time.sleep(2)  # Wait for Arduino reset
        ser.reset_input_buffer()
        print(f"✅ Connected to {port}")
    except Exception as e:
        print(f"❌ Serial Error: {e}")
        sys.exit(1)
    
    # Initialize motor controller
    motors = MotorController(ser)
    
    # Setup camera
    print(f"Opening camera ({IMG_WIDTH}x{IMG_HEIGHT})...")
    pipeline = create_gstreamer_pipeline(IMG_WIDTH, IMG_HEIGHT)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("❌ ERROR: Could not open camera!")
        print("   Try running: sudo systemctl restart nvargus-daemon")
        ser.close()
        sys.exit(1)
    
    print("✅ Camera opened successfully!")
    
    # Sensor data storage
    sensor_data = np.zeros(9)
    
    def read_sensors():
        """Read latest sensor data from Arduino"""
        nonlocal sensor_data
        try:
            if ser.in_waiting > 0:
                lines = ser.read_all().decode('utf-8', errors='ignore').split('\n')
                for line in lines:
                    line = line.strip()
                    if line.startswith("START") and line.endswith("END"):
                        parts = line.split(',')
                        if len(parts) >= 10:
                            vals = [float(x) for x in parts[1:10]]
                            sensor_data = np.array(vals)
        except:
            pass
    
    # Main loop
    print("\n" + "=" * 60)
    print("   STARTING AUTONOMOUS DRIVING")
    print(f"   Base Speed: {base_speed} | Steer Gain: {steer_gain}")
    print("   Press 'q' to quit (if preview enabled)")
    print("   Press Ctrl+C to stop")
    print("=" * 60 + "\n")
    
    # Give camera time to warm up
    time.sleep(1)
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Read sensors
            read_sensors()
            distance = sensor_data[0]
            
            # Read camera frame
            ret, frame = cap.read()
            if not ret:
                print("⚠️  Camera read failed, retrying...")
                time.sleep(0.1)
                continue
            
            # Preprocess image
            img = preprocess_image(frame)
            img_tensor = torch.tensor(img, dtype=torch.float32).unsqueeze(0).to(device)
            
            # Preprocess sensors
            sens_batch = sensor_data.reshape(1, -1)
            sens_scaled = scaler.transform(sens_batch)
            sens_tensor = torch.tensor(sens_scaled, dtype=torch.float32).to(device)
            
            # Get model prediction
            with torch.no_grad():
                steering = float(model(img_tensor, sens_tensor).item())
            
            # Calculate motor speeds
            turn_val = steering * steer_gain
            left_motor = base_speed + turn_val
            right_motor = base_speed - turn_val
            
            # Apply speed limits
            left_motor = max(-MAX_SPEED, min(MAX_SPEED, left_motor))
            right_motor = max(-MAX_SPEED, min(MAX_SPEED, right_motor))
            
            # Safety checks
            if distance < OBSTACLE_STOP_DIST:
                # Emergency stop
                motors.stop()
                status = f"🛑 OBSTACLE STOP ({distance:.0f}cm)"
            elif distance < OBSTACLE_SLOW_DIST:
                # Slow down
                slow_factor = distance / OBSTACLE_SLOW_DIST
                motors.set_motors(left_motor * slow_factor, right_motor * slow_factor)
                status = f"⚠️  SLOW ({distance:.0f}cm) | Steer: {steering:+.2f}"
            else:
                # Normal driving
                motors.set_motors(left_motor, right_motor)
                status = f"🚗 DRIVING | Steer: {steering:+.2f} | L:{left_motor:.0f} R:{right_motor:.0f}"
            
            # Calculate FPS
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Print status
            if VERBOSE:
                print(f"\r{status} | FPS: {fps:.1f} | Dist: {distance:.0f}cm", end="")
            
            # Show preview if enabled
            if show_preview:
                # Add overlay text
                cv2.putText(frame, f"Steer: {steering:+.2f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {distance:.0f}cm", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Draw steering indicator
                center_x = IMG_WIDTH // 2
                steer_x = int(center_x + steering * 100)
                cv2.line(frame, (center_x, IMG_HEIGHT - 20), 
                        (steer_x, IMG_HEIGHT - 50), (0, 0, 255), 3)
                
                cv2.imshow('Autonomous Driver', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n\nQuitting...")
                    break
    
    except KeyboardInterrupt:
        print("\n\nStopping (Ctrl+C)...")
    
    finally:
        # Cleanup
        print("Cleaning up...")
        motors.stop()
        time.sleep(0.1)
        cap.release()
        ser.close()
        if show_preview:
            cv2.destroyAllWindows()
        print("Done!")

if __name__ == "__main__":
    main()
