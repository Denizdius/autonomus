#!/usr/bin/env python3
"""
ArUco Marker Path Playback with Correction
===========================================
Plays back a recorded path and corrects position at each ArUco marker.

Usage:
    python3 aruco_playback.py                # Full round trip
    python3 aruco_playback.py --loop         # Continuous loop
    python3 aruco_playback.py --target-only  # Go to target only

The robot will:
1. Follow the recorded path
2. At each marker checkpoint, detect the marker
3. Correct its position/heading based on marker location
4. Continue to the next waypoint

Controls during playback:
    SPACE - Pause/Resume
    Q - Quit
"""

import serial
import time
import os
import sys
import tty
import termios
import select
import json
import cv2
import numpy as np
import argparse

# --- CONFIGURATION ---
BAUD_RATE = 115200
DEFAULT_PATH_FILE = "aruco_path.json"

# Motor Speeds
SPEED_MULTIPLIER = 1.0
CORRECTION_SPEED = 180  # Slower speed for corrections
TURN_SPEED = 255

# ArUco Settings
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Camera Settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FRAME_CENTER_X = CAMERA_WIDTH // 2

# Correction thresholds
CENTER_TOLERANCE = 50      # Pixels from center (acceptable range)
CORRECTION_TIMEOUT = 5.0   # Max seconds to try correction

# 180° Turn
TURN_180_DURATION = 6.0

# Safety
OBSTACLE_STOP_DISTANCE = 5

# --- GLOBALS ---
ser = None
cap = None
sensor_distance = 999

def connect_arduino():
    global ser
    ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyACM1']
    for p in ports:
        if os.path.exists(p):
            try:
                print(f"Connecting to Arduino on {p}...", end="")
                ser = serial.Serial(p, BAUD_RATE, timeout=0.05)
                time.sleep(2)
                ser.reset_input_buffer()
                print(" OK!")
                return True
            except Exception as e:
                print(f" Failed: {e}")
    print("ERROR: Arduino NOT found!")
    return False

def init_camera():
    global cap
    print("Initializing camera...", end="")
    
    gst_pipeline = (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={CAMERA_WIDTH}, height={CAMERA_HEIGHT}, "
        f"format=NV12, framerate=30/1 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )
    
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    
    if cap.isOpened():
        print(" OK!")
        return True
    else:
        print(" FAILED!")
        return False

def send_cmd(left, right):
    if ser:
        try:
            left = int(left * SPEED_MULTIPLIER)
            right = int(right * SPEED_MULTIPLIER)
            cmd = f"<{left},{right}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def read_sensors():
    global sensor_distance
    if not ser:
        return
    try:
        if ser.in_waiting > 0:
            lines = ser.read_all().decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                if line.startswith("START") and "END" in line:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        sensor_distance = int(parts[1])
    except:
        pass

def do_180_turn():
    print("\n🔄 Performing 180° turn...")
    start_time = time.time()
    while time.time() - start_time < TURN_180_DURATION:
        cmd = f"<{TURN_SPEED},0>"
        ser.write(cmd.encode('utf-8'))
        ser.flush()
        time.sleep(0.1)
    ser.write(b"<0,0>")
    ser.flush()
    print("✅ Turn complete!\n")
    time.sleep(0.3)

def detect_aruco(target_id=None):
    """Detect ArUco markers, optionally filter by target ID"""
    if cap is None:
        return None
    
    ret, frame = cap.read()
    if not ret:
        return None
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    corners, ids, rejected = detector.detectMarkers(gray)
    
    if ids is None:
        return None
    
    for i, marker_id in enumerate(ids):
        mid = int(marker_id[0])
        if target_id is None or mid == target_id:
            marker_corners = corners[i][0]
            center_x = np.mean(marker_corners[:, 0])
            center_y = np.mean(marker_corners[:, 1])
            width = np.linalg.norm(marker_corners[0] - marker_corners[1])
            
            return {
                'id': mid,
                'center_x': center_x,
                'center_y': center_y,
                'width': width
            }
    
    return None

def correct_at_marker(target_marker):
    """
    Correct robot position at a marker checkpoint.
    Centers the marker in the camera view (heading correction).
    """
    marker_id = target_marker['marker_id']
    expected_x = target_marker.get('center_x', FRAME_CENTER_X)
    
    print(f"\n🎯 Correction at Marker #{marker_id}...")
    
    start_time = time.time()
    
    while time.time() - start_time < CORRECTION_TIMEOUT:
        marker = detect_aruco(marker_id)
        
        if marker is None:
            # Marker not visible - try turning slowly to find it
            print(f"  👁️  Searching for Marker #{marker_id}...")
            send_cmd(CORRECTION_SPEED // 2, -CORRECTION_SPEED // 2)
            time.sleep(0.2)
            send_cmd(0, 0)
            continue
        
        # Calculate error from center
        error_x = marker['center_x'] - FRAME_CENTER_X
        
        print(f"  📍 Marker #{marker_id}: x_error={error_x:+.0f}px")
        
        if abs(error_x) < CENTER_TOLERANCE:
            # Marker is centered - correction complete!
            send_cmd(0, 0)
            print(f"  ✅ Centered! (error: {error_x:+.0f}px)")
            time.sleep(0.3)
            return True
        
        # Turn to center the marker
        if error_x > 0:
            # Marker is to the right, turn right
            send_cmd(CORRECTION_SPEED, 0)
        else:
            # Marker is to the left, turn left
            send_cmd(0, CORRECTION_SPEED)
        
        time.sleep(0.1)
        send_cmd(0, 0)
        time.sleep(0.1)
    
    print(f"  ⚠️  Correction timeout - continuing anyway")
    return False

def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def play_path(commands, markers, direction_name="TO TARGET"):
    """Execute path with marker corrections"""
    
    print(f"\n🚀 Playing: {direction_name}")
    print(f"   Commands: {len(commands)}, Markers: {len(markers)}")
    print("   Press SPACE to pause, Q to quit\n")
    
    # Create lookup: command_index -> marker
    marker_checkpoints = {m['command_index']: m for m in markers}
    
    paused = False
    
    for i, cmd in enumerate(commands):
        # Check for marker checkpoint BEFORE this command
        if i in marker_checkpoints:
            correct_at_marker(marker_checkpoints[i])
        
        # Check user input
        key = get_key()
        if key == 'q':
            send_cmd(0, 0)
            return False
        elif key == ' ':
            paused = not paused
            if paused:
                send_cmd(0, 0)
                print("⏸️  PAUSED")
            else:
                print("▶️  Resumed")
        
        while paused:
            key = get_key()
            if key == ' ':
                paused = False
                print("▶️  Resumed")
            elif key == 'q':
                return False
            time.sleep(0.1)
        
        # Safety check
        read_sensors()
        if sensor_distance < OBSTACLE_STOP_DISTANCE and cmd["left"] > 0 and cmd["right"] > 0:
            send_cmd(0, 0)
            print(f"⚠️  OBSTACLE at {sensor_distance}cm!")
            while sensor_distance < OBSTACLE_STOP_DISTANCE + 10:
                read_sensors()
                time.sleep(0.1)
                if get_key() == 'q':
                    return False
        
        # Execute command
        action = cmd["action"]
        left = cmd["left"]
        right = cmd["right"]
        duration = cmd["duration"]
        
        print(f"  [{i+1}/{len(commands)}] {action:8} for {duration:.2f}s")
        
        # Send commands continuously
        start_time = time.time()
        while time.time() - start_time < duration:
            send_cmd(left, right)
            time.sleep(0.1)
            
            key = get_key()
            if key == 'q':
                send_cmd(0, 0)
                return False
            elif key == ' ':
                send_cmd(0, 0)
                paused = True
                print("⏸️  PAUSED")
                while paused:
                    key = get_key()
                    if key == ' ':
                        paused = False
                        print("▶️  Resumed")
                    elif key == 'q':
                        return False
                    time.sleep(0.1)
            
            read_sensors()
            if sensor_distance < OBSTACLE_STOP_DISTANCE and left > 0 and right > 0:
                send_cmd(0, 0)
                break
    
    send_cmd(0, 0)
    
    # Check for final marker
    if len(commands) in marker_checkpoints:
        correct_at_marker(marker_checkpoints[len(commands)])
    
    print(f"\n✅ Path complete: {direction_name}")
    return True

def load_path(filepath):
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"ERROR: Path file '{filepath}' not found!")
        print("Run 'python3 aruco_record.py' first.")
        return None
    except json.JSONDecodeError:
        print(f"ERROR: Invalid JSON in '{filepath}'")
        return None

def main():
    parser = argparse.ArgumentParser(description='Playback ArUco path with correction')
    parser.add_argument('--file', '-f', default=DEFAULT_PATH_FILE)
    parser.add_argument('--loop', '-l', action='store_true')
    parser.add_argument('--target-only', '-t', action='store_true')
    parser.add_argument('--return-only', '-r', action='store_true')
    parser.add_argument('--speed', '-s', type=float, default=1.0)
    args = parser.parse_args()
    
    global SPEED_MULTIPLIER
    SPEED_MULTIPLIER = max(0.5, min(2.0, args.speed))
    
    # Load path
    path_data = load_path(args.file)
    if not path_data:
        sys.exit(1)
    
    to_target = path_data.get('to_target', {})
    return_path = path_data.get('return', {})
    
    to_target_commands = to_target.get('commands', [])
    to_target_markers = to_target.get('markers', [])
    return_commands = return_path.get('commands', [])
    return_markers = return_path.get('markers', [])
    
    print("\n" + "="*50)
    print("   ArUco PATH PLAYBACK")
    print("="*50)
    print(f"Path: {args.file}")
    print(f"TO TARGET: {len(to_target_commands)} cmds, {len(to_target_markers)} markers")
    print(f"RETURN:    {len(return_commands)} cmds, {len(return_markers)} markers")
    print(f"Speed: {SPEED_MULTIPLIER:.1f}x")
    print("="*50)
    
    if not connect_arduino():
        sys.exit(1)
    
    if not init_camera():
        print("WARNING: Camera not available. No marker corrections.")
    
    # Setup terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        if args.return_only:
            if return_commands:
                play_path(return_commands, return_markers, "RETURN")
            else:
                print("ERROR: No return path!")
        
        elif args.target_only:
            play_path(to_target_commands, to_target_markers, "TO TARGET")
        
        elif args.loop:
            loop_count = 0
            while True:
                loop_count += 1
                print(f"\n{'='*50}")
                print(f"   LOOP {loop_count}")
                print('='*50)
                
                if not play_path(to_target_commands, to_target_markers, "TO TARGET"):
                    break
                
                time.sleep(0.5)
                do_180_turn()
                
                if return_commands:
                    if not play_path(return_commands, return_markers, "RETURN"):
                        break
                else:
                    print("⚠️  No return path")
                    break
                
                time.sleep(0.5)
                do_180_turn()
        
        else:
            # Full round trip
            if not play_path(to_target_commands, to_target_markers, "TO TARGET"):
                pass
            elif return_commands:
                time.sleep(0.5)
                do_180_turn()
                play_path(return_commands, return_markers, "RETURN")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    
    finally:
        send_cmd(0, 0)
        if cap:
            cap.release()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n🏁 Playback ended.")

if __name__ == "__main__":
    main()
