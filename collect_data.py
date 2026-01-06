#!/usr/bin/env python3
"""
Training Data Collector - Tesla-Style Learning
===============================================
Drive the robot manually with WASD, and this script captures:
- Camera frames (640x480)
- Motor commands (what YOU pressed)
- Sensor data (distance, IMU, GPS)

Run multiple sessions to build a large dataset!

Usage:
    python3 collect_data.py              # Start new session
    python3 collect_data.py --session 5  # Label this as session 5

Controls:
    W - Forward
    S - Backward
    A - Turn Left  
    D - Turn Right
    SPACE - Stop
    Q - Quit and save

The data is APPENDED to dataset_vga/log.csv (won't overwrite!)
"""

import serial
import time
import os
import sys
import tty
import termios
import select
import cv2
import argparse
from datetime import datetime

# --- CONFIGURATION ---
BAUD_RATE = 115200
DATA_DIR = "dataset_vga"
LOG_FILE = os.path.join(DATA_DIR, "log.csv")

# Camera settings (MUST match train.py!)
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAPTURE_FPS = 10  # Frames per second to capture

# Motor speeds (swing turns)
SPEED_FWD = 150   # Match record_path.py
SPEED_TURN = 150
SPEED_STOP = 0

# --- GLOBALS ---
ser = None
cap = None
sensor_data = {
    'dist_cm': 999,
    'gps_lat': 0.0,
    'gps_lon': 0.0,
    'acc_x': 0.0, 'acc_y': 0.0, 'acc_z': 0.0,
    'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0
}

def connect_arduino():
    global ser
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyACM1']
    for p in ports:
        if os.path.exists(p):
            try:
                print(f"Connecting to {p}...", end="")
                ser = serial.Serial(p, BAUD_RATE, timeout=0.05)
                time.sleep(2)
                ser.reset_input_buffer()
                print(" OK!")
                return True
            except Exception as e:
                print(f" Failed: {e}")
    print("ERROR: Arduino NOT found!")
    return False

def connect_camera():
    global cap
    # Try CSI camera first (Jetson), then USB
    pipelines = [
        # CSI Camera (Raspberry Pi Camera on Jetson)
        f'nvarguscamerasrc ! video/x-raw(memory:NVMM), width={CAM_WIDTH}, height={CAM_HEIGHT}, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1',
        # USB Camera fallback
        0
    ]
    
    for pipeline in pipelines:
        try:
            if isinstance(pipeline, str):
                print("Trying CSI camera...")
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            else:
                print("Trying USB camera...")
                cap = cv2.VideoCapture(pipeline)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"Camera OK! Resolution: {frame.shape[1]}x{frame.shape[0]}")
                    return True
        except:
            continue
    
    print("ERROR: Camera NOT found!")
    return False

def send_cmd(left, right):
    """Send motor command to Arduino"""
    if ser:
        try:
            cmd = f"<{int(left)},{int(right)}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def read_sensors():
    """Read sensor data from Arduino"""
    global sensor_data
    if not ser:
        return
    try:
        if ser.in_waiting > 0:
            lines = ser.read_all().decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                if line.startswith("START") and "END" in line:
                    parts = line.replace("START,", "").replace(",END", "").split(',')
                    if len(parts) >= 9:
                        sensor_data['dist_cm'] = int(float(parts[0]))
                        sensor_data['gps_lat'] = float(parts[1])
                        sensor_data['gps_lon'] = float(parts[2])
                        sensor_data['acc_x'] = float(parts[3])
                        sensor_data['acc_y'] = float(parts[4])
                        sensor_data['acc_z'] = float(parts[5])
                        sensor_data['gyro_x'] = float(parts[6])
                        sensor_data['gyro_y'] = float(parts[7])
                        sensor_data['gyro_z'] = float(parts[8])
    except:
        pass

def get_key():
    """Non-blocking key read"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def count_existing_samples():
    """Count how many samples already exist"""
    if os.path.exists(LOG_FILE):
        with open(LOG_FILE, 'r') as f:
            return sum(1 for line in f) - 1  # Subtract header
    return 0

def main():
    parser = argparse.ArgumentParser(description='Collect training data')
    parser.add_argument('--session', '-s', type=int, default=0, help='Session number for labeling')
    args = parser.parse_args()
    
    # Create data directory
    os.makedirs(DATA_DIR, exist_ok=True)
    
    # Connect hardware
    if not connect_arduino():
        sys.exit(1)
    if not connect_camera():
        sys.exit(1)
    
    # Check existing data
    existing_samples = count_existing_samples()
    print(f"\n📊 Existing samples in dataset: {existing_samples}")
    
    # Create/append CSV
    write_header = not os.path.exists(LOG_FILE)
    log_file = open(LOG_FILE, 'a')
    if write_header:
        log_file.write("timestamp,image_path,cmd_left,cmd_right,dist_cm,gps_lat,gps_lon,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n")
    
    # Setup terminal for raw input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print("\n" + "="*60)
    print("   TRAINING DATA COLLECTION - Session", args.session)
    print("="*60)
    print("Controls:")
    print("  W = Forward    S = Backward")
    print("  A = Left       D = Right")
    print("  SPACE = Stop")
    print("  K = Switch to RETURN mode (after reaching target)")
    print("  Q = Quit & Save")
    print("="*60)
    print("\n📍 Recording TO TARGET... Drive to destination!\n")
    
    # Command mapping (swing turns)
    key_to_motors = {
        'w': (SPEED_FWD, SPEED_FWD, "FWD "),
        's': (-SPEED_FWD, -SPEED_FWD, "BACK"),
        'a': (0, SPEED_TURN, "LEFT"),
        'd': (SPEED_TURN, 0, "RIGHT"),
        ' ': (SPEED_STOP, SPEED_STOP, "STOP"),
    }
    
    current_left, current_right = 0, 0
    current_action = "STOP"
    frame_count = 0
    to_target_frames = 0
    return_frames = 0
    direction = "TO_TARGET"  # or "RETURN"
    session_start = time.time()
    last_capture = 0
    capture_interval = 1.0 / CAPTURE_FPS
    
    try:
        while True:
            key = get_key()
            
            if key == 'q':
                break
            
            # K = Switch to RETURN mode
            if key == 'k' and direction == "TO_TARGET":
                send_cmd(0, 0)
                current_left, current_right = 0, 0
                direction = "RETURN"
                print("\n" + "="*60)
                print("🔄 SWITCHED TO RETURN MODE!")
                print("   Turn around manually (A or D), then drive back.")
                print("   Press Q when you arrive at base.")
                print("="*60)
                print(f"\n📍 TO TARGET frames: {to_target_frames}")
                print("📍 Recording RETURN... Drive back to base!\n")
                continue
            
            if key in key_to_motors:
                current_left, current_right, current_action = key_to_motors[key]
                send_cmd(current_left, current_right)
            
            # Read sensors continuously
            read_sensors()
            
            # Capture frame at target FPS
            now = time.time()
            if now - last_capture >= capture_interval:
                ret, frame = cap.read()
                if ret and (current_left != 0 or current_right != 0):
                    # Only save when moving (not when stopped)
                    timestamp = time.time()
                    img_name = f"img_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{int((timestamp % 1) * 1000000):06d}.jpg"
                    img_path = os.path.join(DATA_DIR, img_name)
                    
                    # Save image
                    cv2.imwrite(img_path, frame)
                    
                    # Log data
                    log_line = f"{timestamp},{img_name},{current_left},{current_right},"
                    log_line += f"{sensor_data['dist_cm']},{sensor_data['gps_lat']},{sensor_data['gps_lon']},"
                    log_line += f"{sensor_data['acc_x']:.2f},{sensor_data['acc_y']:.2f},{sensor_data['acc_z']:.2f},"
                    log_line += f"{sensor_data['gyro_x']:.2f},{sensor_data['gyro_y']:.2f},{sensor_data['gyro_z']:.2f}\n"
                    log_file.write(log_line)
                    log_file.flush()
                    
                    frame_count += 1
                    if direction == "TO_TARGET":
                        to_target_frames += 1
                    else:
                        return_frames += 1
                    
                    elapsed = now - session_start
                    total = existing_samples + frame_count
                    dir_indicator = "→ TARGET" if direction == "TO_TARGET" else "← RETURN"
                    
                    # Print status every 10 frames
                    if frame_count % 10 == 0:
                        print(f"[{elapsed:5.1f}s] {dir_indicator} | {current_action} | Frames: {frame_count} | Total: {total} | Dist: {sensor_data['dist_cm']:3}cm")
                
                last_capture = now
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    
    finally:
        # Stop motors
        send_cmd(0, 0)
        
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Cleanup
        log_file.close()
        cap.release()
        
        # Summary
        elapsed = time.time() - session_start
        total_samples = existing_samples + frame_count
        
        print("\n" + "="*60)
        print("   SESSION COMPLETE!")
        print("="*60)
        print(f"   Session {args.session}:")
        print(f"   - Duration: {elapsed:.1f} seconds")
        print(f"   - TO TARGET:  {to_target_frames} frames")
        print(f"   - RETURN:     {return_frames} frames")
        print(f"   - Total this session: {frame_count} frames")
        print(f"   - Total dataset: {total_samples} samples")
        print("="*60)
        
        if return_frames == 0 and to_target_frames > 0:
            print("\n⚠️  No RETURN data! Next time press K at target, then drive back.")
        
        if total_samples < 5000:
            runs_needed = (5000 - total_samples) // max(frame_count, 1) + 1
            print(f"\n💡 Tip: You need ~{runs_needed} more runs like this for good results")
            print(f"   Target: 5,000+ samples (currently {total_samples})")
        else:
            print(f"\n✅ Great! You have enough data. Run 'python3 train.py' to train!")

if __name__ == "__main__":
    main()
