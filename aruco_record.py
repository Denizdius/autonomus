#!/usr/bin/env python3
"""
ArUco Marker Path Recording
============================
Record a path with ArUco marker checkpoints.
At each marker, the robot saves its position for later correction.

Usage:
    python3 aruco_record.py

Controls:
    W - Forward
    S - Backward
    A - Turn Left
    D - Turn Right
    SPACE - Stop
    M - Mark current ArUco marker as checkpoint
    K - Switch to RETURN mode (auto 180° turn)
    Q - Quit and Save

Markers are automatically detected - press M when you see one to save it.
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
from datetime import datetime
from threading import Thread

# --- CONFIGURATION ---
BAUD_RATE = 115200
PATH_FILE = "aruco_path.json"

# Motor Speeds
SPEED_FWD = 255
SPEED_TURN = 255
SPEED_STOP = 0

# 180° Turn
TURN_180_DURATION = 6.0

# ArUco Settings
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Marker quality thresholds
MARKER_MIN_SIZE = 40     # Too far if marker smaller than this
MARKER_MAX_SIZE = 250    # Too close if marker bigger than this
MARKER_IDEAL_MIN = 80    # Ideal range
MARKER_IDEAL_MAX = 180   # Ideal range

def beep(count=1):
    """Print visual beep indicator to console"""
    if count == 1:
        print("\n  🔔 BEEP!")
    elif count == 2:
        print("\n  🔔🔔 BEEP BEEP! (GOOD!)")
    else:
        print("\n  🔔🔔🔔 BEEP BEEP BEEP! (ERROR)")

# Camera Settings (Raspberry Pi Camera V2)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# --- GLOBALS ---
ser = None
cap = None
current_marker_id = None
current_marker_corners = None
frame_lock = False

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
    
    # Try GStreamer pipeline for Raspberry Pi Camera
    gst_pipeline = (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), width={CAMERA_WIDTH}, height={CAMERA_HEIGHT}, "
        f"format=NV12, framerate=30/1 ! "
        f"nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
    )
    
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        # Fallback to USB camera
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
            cmd = f"<{int(left)},{int(right)}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def do_180_turn():
    """Perform 180° turn with continuous commands"""
    print("\n🔄 Performing 180° turn...")
    start_time = time.time()
    while time.time() - start_time < TURN_180_DURATION:
        send_cmd(SPEED_TURN, 0)
        time.sleep(0.1)
    send_cmd(0, 0)
    print("✅ Turn complete!\n")
    time.sleep(0.3)

def detect_aruco(frame):
    """Detect ArUco markers in frame"""
    global current_marker_id, current_marker_corners
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    corners, ids, rejected = detector.detectMarkers(gray)
    
    if ids is not None and len(ids) > 0:
        # Take the first detected marker
        current_marker_id = int(ids[0][0])
        current_marker_corners = corners[0][0]
        
        # Calculate marker center and size
        center_x = np.mean(current_marker_corners[:, 0])
        center_y = np.mean(current_marker_corners[:, 1])
        
        # Marker width (for distance estimation)
        width = np.linalg.norm(current_marker_corners[0] - current_marker_corners[1])
        
        return {
            'id': current_marker_id,
            'center_x': center_x,
            'center_y': center_y,
            'width': width,
            'corners': current_marker_corners.tolist()
        }
    else:
        current_marker_id = None
        current_marker_corners = None
        return None

def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def main():
    global cap
    
    if not connect_arduino():
        sys.exit(1)
    
    if not init_camera():
        print("WARNING: Camera not available. Markers won't be detected.")
        cap = None
    
    # Setup terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print("\n" + "="*50)
    print("   ArUco PATH RECORDING")
    print("="*50)
    print("Controls:")
    print("  W/S/A/D = Drive")
    print("  M = Save current marker as checkpoint")
    print("  K = Switch to RETURN mode (180° turn)")
    print("  Q = Quit & Save")
    print("="*50)
    print("\n📍 Recording TO TARGET path...\n")
    
    # Recording storage
    to_target_commands = []
    return_commands = []
    to_target_markers = []  # Marker checkpoints
    return_markers = []
    current_command = None
    command_start_time = None
    recording_mode = "TO_TARGET"
    
    key_to_motors = {
        'w': (SPEED_FWD, SPEED_FWD, "FORWARD"),
        's': (-SPEED_FWD, -SPEED_FWD, "BACKWARD"),
        'a': (0, SPEED_TURN, "LEFT"),
        'd': (SPEED_TURN, 0, "RIGHT"),
        ' ': (SPEED_STOP, SPEED_STOP, "STOP"),
    }
    
    recording_start = time.time()
    last_marker_info = None
    last_status_time = 0
    marker_visible = False
    
    try:
        while True:
            # Read camera and detect markers
            if cap is not None:
                ret, frame = cap.read()
                if ret:
                    marker_info = detect_aruco(frame)
                    
                    # Continuous status update every 0.5 seconds
                    if time.time() - last_status_time > 0.5:
                        last_status_time = time.time()
                        
                        if marker_info:
                            marker_visible = True
                            last_marker_info = marker_info
                            size = marker_info['width']
                            center_x = marker_info['center_x']
                            
                            # Determine quality
                            if size < MARKER_MIN_SIZE:
                                quality = "❌ TOO FAR"
                                beep(1)
                            elif size > MARKER_MAX_SIZE:
                                quality = "❌ TOO CLOSE"
                                beep(1)
                            elif center_x < 150:
                                quality = "⚠️  TURN RIGHT (marker on left)"
                            elif center_x > CAMERA_WIDTH - 150:
                                quality = "⚠️  TURN LEFT (marker on right)"
                            elif MARKER_IDEAL_MIN <= size <= MARKER_IDEAL_MAX:
                                quality = "✅ PERFECT! Press M"
                                beep(2)  # Double beep = good!
                            else:
                                quality = "✅ OK - Press M"
                            
                            # Position indicator
                            pos_bar = ""
                            pos = int((center_x / CAMERA_WIDTH) * 20)
                            pos_bar = "-" * pos + "|" + "-" * (20 - pos)
                            
                            print(f"\r  👁️  Marker #{marker_info['id']:2d} | Size:{size:3.0f}px | [{pos_bar}] | {quality}      ", end="")
                        else:
                            if marker_visible:
                                print(f"\r  🚫 No marker visible - move closer or point camera at marker          ", end="")
                                marker_visible = False
            
            key = get_key()
            
            if key == 'q':
                # Save final command
                if current_command and command_start_time:
                    duration = time.time() - command_start_time
                    if duration > 0.05:
                        cmd_data = {
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        }
                        if recording_mode == "TO_TARGET":
                            to_target_commands.append(cmd_data)
                        else:
                            return_commands.append(cmd_data)
                break
            
            if key == 'm':
                if last_marker_info:
                    # Check if marker is in good position
                    size = last_marker_info['width']
                    if size < MARKER_MIN_SIZE:
                        print(f"\n  ❌ Marker too far! Move closer and try again.")
                        beep(3)
                        continue
                    elif size > MARKER_MAX_SIZE:
                        print(f"\n  ❌ Marker too close! Move back and try again.")
                        beep(3)
                        continue
                    
                    # Save marker as checkpoint
                    elapsed = time.time() - recording_start
                    checkpoint = {
                        "marker_id": last_marker_info['id'],
                        "time": round(elapsed, 2),
                        "command_index": len(to_target_commands) if recording_mode == "TO_TARGET" else len(return_commands),
                        "center_x": round(last_marker_info['center_x'], 1),
                        "center_y": round(last_marker_info['center_y'], 1),
                        "width": round(last_marker_info['width'], 1)
                    }
                    
                    if recording_mode == "TO_TARGET":
                        to_target_markers.append(checkpoint)
                    else:
                        return_markers.append(checkpoint)
                    
                    # Success feedback
                    beep(2)
                    print(f"\n\n  ✅ CHECKPOINT SAVED: Marker #{last_marker_info['id']}")
                    print(f"     Position: center=({checkpoint['center_x']:.0f}, {checkpoint['center_y']:.0f}), size={checkpoint['width']:.0f}px\n")
                else:
                    print(f"\n  ⚠️  No marker visible! Point camera at a marker first.")
                    beep(3)
            
            if key == 'k' and recording_mode == "TO_TARGET":
                # Save current command
                if current_command and command_start_time:
                    duration = time.time() - command_start_time
                    if duration > 0.05:
                        to_target_commands.append({
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        })
                
                send_cmd(0, 0)
                current_command = None
                command_start_time = None
                
                # 180° turn
                do_180_turn()
                
                recording_mode = "RETURN"
                print("="*50)
                print("🔄 SWITCHED TO RETURN MODE!")
                print("   Drive FORWARD to return to base.")
                print("   Press M at markers to save checkpoints.")
                print("="*50 + "\n")
                print("📍 Recording RETURN path...\n")
                continue
            
            if key in key_to_motors:
                new_command = key_to_motors[key]
                
                # Save previous command
                if current_command and command_start_time:
                    duration = time.time() - command_start_time
                    if duration > 0.05:
                        cmd_data = {
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        }
                        if recording_mode == "TO_TARGET":
                            to_target_commands.append(cmd_data)
                        else:
                            return_commands.append(cmd_data)
                
                current_command = new_command
                command_start_time = time.time()
                
                left, right, action = new_command
                send_cmd(left, right)
                
                elapsed = time.time() - recording_start
                mode_str = "→ TARGET" if recording_mode == "TO_TARGET" else "← RETURN"
                cmd_count = len(to_target_commands) if recording_mode == "TO_TARGET" else len(return_commands)
                marker_count = len(to_target_markers) if recording_mode == "TO_TARGET" else len(return_markers)
                print(f"[{elapsed:6.1f}s] {mode_str} | {action:8} | Cmds:{cmd_count} | Markers:{marker_count}")
            
            time.sleep(0.02)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    
    finally:
        send_cmd(0, 0)
        if cap:
            cap.release()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Save path
        if to_target_commands or return_commands:
            path_data = {
                "recorded_at": datetime.now().isoformat(),
                "type": "aruco_path",
                "to_target": {
                    "command_count": len(to_target_commands),
                    "marker_count": len(to_target_markers),
                    "commands": to_target_commands,
                    "markers": to_target_markers
                },
                "return": {
                    "command_count": len(return_commands),
                    "marker_count": len(return_markers),
                    "commands": return_commands,
                    "markers": return_markers
                }
            }
            
            with open(PATH_FILE, 'w') as f:
                json.dump(path_data, f, indent=2)
            
            print("\n" + "="*50)
            print(f"✅ PATH SAVED: {PATH_FILE}")
            print(f"   TO TARGET: {len(to_target_commands)} commands, {len(to_target_markers)} markers")
            print(f"   RETURN:    {len(return_commands)} commands, {len(return_markers)} markers")
            print("="*50)
            
            if to_target_markers:
                print("\nTO TARGET markers:")
                for m in to_target_markers:
                    print(f"  Marker #{m['marker_id']} at command {m['command_index']}")
            
            if return_markers:
                print("\nRETURN markers:")
                for m in return_markers:
                    print(f"  Marker #{m['marker_id']} at command {m['command_index']}")
        else:
            print("\n⚠️  No path recorded!")

if __name__ == "__main__":
    main()
