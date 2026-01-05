#!/usr/bin/env python3
"""
Smart Path Playback Script
===========================
Plays back recorded paths with IMU heading correction and optional landmark verification.

Features:
- IMU-based heading correction (keeps robot on track)
- Camera landmark verification (optional)
- Obstacle avoidance
- Speed control

Usage:
    python3 smart_playback.py                     # Full round trip with heading correction
    python3 smart_playback.py --loop              # Continuous loop
    python3 smart_playback.py --target-only       # Only go to target
    python3 smart_playback.py --no-correction     # Disable IMU heading correction
    python3 smart_playback.py --verify-landmarks  # Verify camera landmarks
    python3 smart_playback.py --file mypath.json  # Use specific path

Controls during playback:
    SPACE - Pause/Resume
    Q - Quit immediately
    + - Increase correction strength
    - - Decrease correction strength
"""

import serial
import time
import os
import sys
import tty
import termios
import select
import json
import argparse

# --- CONFIGURATION ---
BAUD_RATE = 115200
DEFAULT_PATH_FILE = "smart_path.json"

# Speed multiplier
SPEED_MULTIPLIER = 1.0

# Safety
OBSTACLE_STOP_DISTANCE = 5

# Heading correction
HEADING_CORRECTION_ENABLED = True
HEADING_TOLERANCE = 5.0  # Degrees of acceptable error

# --- GLOBALS ---
ser = None
current_yaw = 0.0
current_distance = 999
camera = None

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
                continue
    print("ERROR: Arduino NOT found!")
    return False

def init_camera():
    global camera
    try:
        import cv2
        pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=1"
        )
        camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if camera.isOpened():
            for _ in range(5):
                camera.read()
            print("Camera initialized!")
            return True
        return False
    except Exception as e:
        print(f"Camera init failed: {e}")
        return False

def send_motor_cmd(left, right):
    """Send motor command with speed multiplier"""
    global ser
    if ser:
        try:
            left = int(left * SPEED_MULTIPLIER)
            right = int(right * SPEED_MULTIPLIER)
            cmd = f"<M,{left},{right}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def send_heading_target(heading):
    """Set target heading for Arduino correction"""
    global ser
    if ser:
        try:
            cmd = f"<H,{heading:.2f}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def enable_correction(enabled):
    """Enable/disable Arduino-side heading correction"""
    global ser
    if ser:
        try:
            cmd = f"<C,{1 if enabled else 0}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def reset_yaw():
    """Reset Arduino yaw to 0"""
    global ser
    if ser:
        try:
            ser.write(b"<R>")
            ser.flush()
        except:
            pass

def set_pid(kp, ki, kd):
    """Set PID gains for heading correction"""
    global ser
    if ser:
        try:
            cmd = f"<P,{kp},{ki},{kd}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def read_sensors():
    """Read sensor data from Arduino"""
    global ser, current_yaw, current_distance
    if not ser:
        return
    try:
        if ser.in_waiting > 0:
            lines = ser.read_all().decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                if line.startswith("START") and "END" in line:
                    parts = line.replace("START,", "").replace(",END", "").strip().split(',')
                    if len(parts) >= 2:
                        current_distance = int(parts[0])
                        current_yaw = float(parts[1])
    except:
        pass

def get_key_nonblocking():
    """Non-blocking key read"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def load_path(filepath):
    """Load recorded path from JSON file"""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"ERROR: Path file '{filepath}' not found!")
        print("Run 'python3 smart_record.py' first to record a path.")
        return None
    except json.JSONDecodeError:
        print(f"ERROR: Invalid JSON in '{filepath}'")
        return None

def compare_landmark(landmark_data, landmarks_dir, threshold=0.7):
    """Compare current camera view to saved landmark"""
    global camera
    if camera is None or "image" not in landmark_data:
        return True, 1.0  # Skip if no camera or no saved image
    
    try:
        import cv2
        import numpy as np
        
        # Read saved landmark
        saved_path = os.path.join(landmarks_dir, landmark_data["image"])
        if not os.path.exists(saved_path):
            return True, 1.0
        
        saved_img = cv2.imread(saved_path)
        
        # Capture current frame
        ret, current_img = camera.read()
        if not ret:
            return True, 1.0
        
        # Simple histogram comparison
        saved_hist = cv2.calcHist([saved_img], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
        current_hist = cv2.calcHist([current_img], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
        
        cv2.normalize(saved_hist, saved_hist)
        cv2.normalize(current_hist, current_hist)
        
        similarity = cv2.compareHist(saved_hist, current_hist, cv2.HISTCMP_CORREL)
        
        return similarity >= threshold, similarity
    except Exception as e:
        print(f"Landmark comparison failed: {e}")
        return True, 1.0

def play_path_smart(commands, direction_name, use_correction=True, landmarks=None, landmarks_dir=None):
    """Execute commands with IMU heading correction"""
    global current_yaw, current_distance
    
    print(f"\n🚀 Playing path: {direction_name}")
    print(f"   Commands: {len(commands)}")
    print(f"   Heading correction: {'ON' if use_correction else 'OFF'}")
    print("   Press SPACE to pause, Q to quit, +/- to adjust correction\n")
    
    paused = False
    pid_kp = 2.0
    
    # Reset yaw at start
    reset_yaw()
    time.sleep(0.3)
    read_sensors()
    
    for i, cmd in enumerate(commands):
        # Check for user input
        key = get_key_nonblocking()
        if key == 'q':
            print("\n⏹️  Stopped by user")
            send_motor_cmd(0, 0)
            enable_correction(False)
            return False
        elif key == ' ':
            paused = not paused
            if paused:
                send_motor_cmd(0, 0)
                enable_correction(False)
                print("⏸️  PAUSED - Press SPACE to resume")
            else:
                print("▶️  Resumed")
        elif key == '+':
            pid_kp = min(pid_kp + 0.5, 5.0)
            set_pid(pid_kp, 0, 0.5)
            print(f"   Correction strength: {pid_kp}")
        elif key == '-':
            pid_kp = max(pid_kp - 0.5, 0.5)
            set_pid(pid_kp, 0, 0.5)
            print(f"   Correction strength: {pid_kp}")
        
        # Wait while paused
        while paused:
            key = get_key_nonblocking()
            if key == ' ':
                paused = False
                print("▶️  Resumed")
            elif key == 'q':
                send_motor_cmd(0, 0)
                return False
            time.sleep(0.1)
        
        # Read sensors
        read_sensors()
        
        # Safety stop for obstacles
        if current_distance < OBSTACLE_STOP_DISTANCE and cmd["left"] > 0 and cmd["right"] > 0:
            send_motor_cmd(0, 0)
            enable_correction(False)
            print(f"⚠️  OBSTACLE at {current_distance}cm! Waiting...")
            while current_distance < OBSTACLE_STOP_DISTANCE + 5:
                read_sensors()
                time.sleep(0.1)
                key = get_key_nonblocking()
                if key == 'q':
                    return False
            print("✅ Path clear, continuing...")
        
        # Get command parameters
        action = cmd["action"]
        left = cmd["left"]
        right = cmd["right"]
        duration = cmd["duration"]
        target_end_yaw = cmd.get("end_yaw", current_yaw)
        
        # For forward/backward, use heading correction
        is_straight = (action == "FORWARD" or action == "BACKWARD")
        
        if use_correction and is_straight:
            # Set target heading (maintain current heading during straight movement)
            target_heading = cmd.get("start_yaw", current_yaw)
            send_heading_target(target_heading)
            enable_correction(True)
        else:
            enable_correction(False)
        
        # Calculate expected yaw change for turns
        expected_yaw_change = target_end_yaw - cmd.get("start_yaw", current_yaw)
        
        print(f"  [{i+1}/{len(commands)}] {action:8} L:{left:4} R:{right:4} {duration:.2f}s | Yaw:{current_yaw:+.1f}°", end="")
        
        if is_straight and use_correction:
            print(f" [CORRECTING to {target_heading:.1f}°]")
        else:
            print(f" [Target Δ={expected_yaw_change:+.1f}°]")
        
        # Send motor command
        send_motor_cmd(left, right)
        
        # Execute for duration
        elapsed = 0
        step = 0.02  # 50Hz update rate
        
        while elapsed < duration:
            time.sleep(step)
            elapsed += step
            
            # Read sensors continuously
            read_sensors()
            
            # Check for user input
            key = get_key_nonblocking()
            if key == 'q':
                send_motor_cmd(0, 0)
                enable_correction(False)
                return False
            elif key == ' ':
                send_motor_cmd(0, 0)
                enable_correction(False)
                paused = True
                print("⏸️  PAUSED")
                while paused:
                    key = get_key_nonblocking()
                    if key == ' ':
                        paused = False
                        if use_correction and is_straight:
                            enable_correction(True)
                        send_motor_cmd(left, right)
                        print("▶️  Resumed")
                    elif key == 'q':
                        return False
                    time.sleep(0.1)
            
            # Safety check
            if current_distance < OBSTACLE_STOP_DISTANCE and left > 0 and right > 0:
                send_motor_cmd(0, 0)
                enable_correction(False)
                print(f"\n⚠️  OBSTACLE at {current_distance}cm!")
                while current_distance < OBSTACLE_STOP_DISTANCE + 5:
                    read_sensors()
                    time.sleep(0.1)
                    key = get_key_nonblocking()
                    if key == 'q':
                        return False
                print("✅ Continuing...")
                if use_correction and is_straight:
                    enable_correction(True)
                send_motor_cmd(left, right)
        
        # For turns, check if we reached target yaw
        if not is_straight and use_correction:
            # Wait a bit for turn to complete if needed
            target_yaw = target_end_yaw
            yaw_error = abs(current_yaw - target_yaw)
            
            if yaw_error > HEADING_TOLERANCE:
                print(f"      Adjusting turn (error={yaw_error:.1f}°)...")
                
                # Small correction turn
                correction_time = 0
                while yaw_error > HEADING_TOLERANCE and correction_time < 1.0:
                    if current_yaw < target_yaw:
                        send_motor_cmd(left // 2, right // 2)  # Continue turn direction
                    else:
                        send_motor_cmd(right // 2, left // 2)  # Reverse turn direction
                    
                    time.sleep(0.05)
                    correction_time += 0.05
                    read_sensors()
                    yaw_error = abs(current_yaw - target_yaw)
                
                send_motor_cmd(0, 0)
        
        # Check landmark if exists at this point
        if landmarks and landmarks_dir:
            for lm in landmarks:
                if lm.get("command_index") == i + 1 and lm.get("mode") == direction_name.split()[0]:
                    match, similarity = compare_landmark(lm, landmarks_dir)
                    if match:
                        print(f"      🏁 Landmark #{lm['id']} verified (similarity={similarity:.2f})")
                    else:
                        print(f"      ⚠️ Landmark #{lm['id']} mismatch (similarity={similarity:.2f})")
    
    send_motor_cmd(0, 0)
    enable_correction(False)
    print(f"\n✅ Path complete: {direction_name}")
    print(f"   Final yaw: {current_yaw:.1f}°")
    return True

def main():
    global SPEED_MULTIPLIER, HEADING_CORRECTION_ENABLED
    
    parser = argparse.ArgumentParser(description='Smart path playback with heading correction')
    parser.add_argument('--file', '-f', default=DEFAULT_PATH_FILE, help='Path file to use')
    parser.add_argument('--loop', '-l', action='store_true', help='Continuous loop mode')
    parser.add_argument('--target-only', '-t', action='store_true', help='Only go to target')
    parser.add_argument('--return-only', '-r', action='store_true', help='Only return to base')
    parser.add_argument('--speed', '-s', type=float, default=1.0, help='Speed multiplier (0.5-2.0)')
    parser.add_argument('--no-correction', action='store_true', help='Disable heading correction')
    parser.add_argument('--verify-landmarks', action='store_true', help='Verify camera landmarks')
    args = parser.parse_args()
    
    SPEED_MULTIPLIER = max(0.5, min(2.0, args.speed))
    HEADING_CORRECTION_ENABLED = not args.no_correction
    
    # Load path
    path_data = load_path(args.file)
    if not path_data:
        sys.exit(1)
    
    # Check path format
    if "to_target" in path_data:
        to_target_commands = path_data["to_target"]["commands"]
        return_commands = path_data["return"]["commands"]
        to_target_duration = path_data["to_target"]["duration"]
        return_duration = path_data["return"]["duration"]
    else:
        # Fallback to old format
        to_target_commands = path_data.get("commands", [])
        return_commands = []
        to_target_duration = path_data.get("total_duration", 0)
        return_duration = 0
    
    landmarks = path_data.get("landmarks", [])
    landmarks_dir = path_data.get("landmarks_dir", None)
    has_imu = path_data.get("has_imu", False)
    
    print("\n" + "="*60)
    print("   SMART PATH PLAYBACK MODE")
    print("="*60)
    print(f"Path file: {args.file}")
    print(f"Version: {path_data.get('version', 'legacy')}")
    print(f"Recorded: {path_data.get('recorded_at', 'Unknown')}")
    print(f"TO TARGET: {len(to_target_commands)} commands ({to_target_duration}s)")
    print(f"RETURN:    {len(return_commands)} commands ({return_duration}s)")
    print(f"Landmarks: {len(landmarks)}")
    print(f"Speed: {SPEED_MULTIPLIER:.1f}x")
    print(f"Heading Correction: {'ON' if HEADING_CORRECTION_ENABLED else 'OFF'}")
    
    if not has_imu and HEADING_CORRECTION_ENABLED:
        print("⚠️  Path was recorded without IMU data. Correction may be limited.")
    
    mode_str = 'Loop' if args.loop else 'Target only' if args.target_only else 'Return only' if args.return_only else 'Full round trip'
    print(f"Mode: {mode_str}")
    print("="*60)
    
    # Check if return path exists
    if not return_commands and not args.target_only:
        print("\n⚠️  No RETURN path recorded!")
        if not args.target_only and not args.return_only:
            print("   Continuing with TO TARGET only...\n")
            args.target_only = True
    
    # Connect to Arduino
    if not connect_arduino():
        sys.exit(1)
    
    # Initialize camera if verifying landmarks
    if args.verify_landmarks and landmarks:
        init_camera()
    
    # Setup terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        if args.return_only:
            if return_commands:
                play_path_smart(return_commands, "RETURNING TO BASE", 
                               HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir)
            else:
                print("ERROR: No return path recorded!")
        
        elif args.target_only:
            play_path_smart(to_target_commands, "TO TARGET", 
                           HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir)
        
        elif args.loop:
            loop_count = 0
            while True:
                loop_count += 1
                print(f"\n{'='*60}")
                print(f"   LOOP {loop_count}")
                print('='*60)
                
                # Reset yaw at start of each loop
                reset_yaw()
                time.sleep(0.3)
                
                if not play_path_smart(to_target_commands, "TO TARGET", 
                                       HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir):
                    break
                
                time.sleep(1)
                
                if return_commands:
                    if not play_path_smart(return_commands, "RETURNING TO BASE", 
                                          HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir):
                        break
                else:
                    print("⚠️  No return path - stopping loop")
                    break
                
                time.sleep(1)
        
        else:
            # Full round trip
            if not play_path_smart(to_target_commands, "TO TARGET", 
                                  HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir):
                pass
            elif return_commands:
                time.sleep(1)
                play_path_smart(return_commands, "RETURNING TO BASE", 
                               HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    
    finally:
        send_motor_cmd(0, 0)
        enable_correction(False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if camera is not None:
            camera.release()
        print("\n🏁 Playback ended. Motors stopped.")

if __name__ == "__main__":
    main()
