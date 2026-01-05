#!/usr/bin/env python3
"""
Smart Path Recording Script
============================
Records your path with IMU heading data and optional camera landmarks.

This creates a smarter path that can correct for drift during playback.

Usage:
    python3 smart_record.py                    # Record with IMU
    python3 smart_record.py --landmarks        # Also save camera landmarks
    python3 smart_record.py --file mypath.json # Custom filename

Controls:
    W - Forward
    S - Backward  
    A - Turn Left
    D - Turn Right
    SPACE - Stop/Pause
    L - Save current position as LANDMARK (with camera)
    K - Switch to RETURN mode (after reaching target)
    Q - Quit and Save
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
from datetime import datetime

# --- CONFIGURATION ---
BAUD_RATE = 115200
DEFAULT_PATH_FILE = "smart_path.json"
LANDMARKS_DIR = "landmarks"

# Motor Speeds
SPEED_FWD = 255
SPEED_TURN = 255
SPEED_STOP = 0

# --- GLOBALS ---
ser = None
current_yaw = 0.0
current_distance = 999
camera = None
use_camera = False

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
    global camera, use_camera
    try:
        import cv2
        # GStreamer pipeline for Raspberry Pi Camera V2
        pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=1"
        )
        camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if camera.isOpened():
            # Warm up
            for _ in range(5):
                camera.read()
            use_camera = True
            print("Camera initialized!")
            return True
        else:
            print("Camera failed to open")
            return False
    except Exception as e:
        print(f"Camera init failed: {e}")
        return False

def capture_landmark(landmark_id, landmarks_dir):
    """Capture and save a camera frame as a landmark"""
    global camera
    if not use_camera or camera is None:
        return None
    
    try:
        import cv2
        ret, frame = camera.read()
        if ret:
            os.makedirs(landmarks_dir, exist_ok=True)
            filename = f"landmark_{landmark_id:03d}.jpg"
            filepath = os.path.join(landmarks_dir, filename)
            cv2.imwrite(filepath, frame)
            print(f"📸 Landmark saved: {filename}")
            return filename
        return None
    except Exception as e:
        print(f"Landmark capture failed: {e}")
        return None

def send_cmd(left, right):
    """Send motor command to Arduino"""
    global ser
    if ser:
        try:
            cmd = f"<M,{int(left)},{int(right)}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def reset_yaw():
    """Tell Arduino to reset yaw to 0"""
    global ser
    if ser:
        try:
            ser.write(b"<R>")
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
                    parts = line.replace("START,", "").replace(",END", "").split(',')
                    if len(parts) >= 2:
                        current_distance = int(parts[0])
                        current_yaw = float(parts[1])
    except:
        pass

def get_key():
    """Non-blocking key read"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def main():
    global use_camera
    
    parser = argparse.ArgumentParser(description='Smart path recording with IMU')
    parser.add_argument('--file', '-f', default=DEFAULT_PATH_FILE, help='Output file')
    parser.add_argument('--landmarks', '-l', action='store_true', help='Enable camera landmarks')
    args = parser.parse_args()
    
    if not connect_arduino():
        sys.exit(1)
    
    # Initialize camera if landmarks enabled
    if args.landmarks:
        if not init_camera():
            print("Continuing without camera landmarks...")
    
    # Reset Arduino yaw
    print("Resetting IMU yaw...")
    reset_yaw()
    time.sleep(0.5)
    
    # Setup terminal for raw input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print("\n" + "="*60)
    print("   SMART PATH RECORDING MODE")
    print("="*60)
    print("Controls:")
    print("  W = Forward    S = Backward")
    print("  A = Left       D = Right")
    print("  SPACE = Stop")
    print("  L = Save LANDMARK (camera snapshot)")
    print("  K = Switch to RETURN mode")
    print("  Q = Quit & Save")
    print("="*60)
    print("\n📍 Recording TO TARGET path...\n")
    
    # Recording storage
    to_target_commands = []
    return_commands = []
    landmarks = []
    current_command = None
    command_start_time = None
    command_start_yaw = 0.0
    recording_mode = "TO_TARGET"
    landmark_count = 0
    
    # Landmarks directory
    path_basename = os.path.splitext(args.file)[0]
    landmarks_dir = f"{path_basename}_landmarks"
    
    # Command mapping
    key_to_motors = {
        'w': (SPEED_FWD, SPEED_FWD, "FORWARD"),
        's': (-SPEED_FWD, -SPEED_FWD, "BACKWARD"),
        'a': (-SPEED_TURN, SPEED_TURN, "LEFT"),
        'd': (SPEED_TURN, -SPEED_TURN, "RIGHT"),
        ' ': (SPEED_STOP, SPEED_STOP, "STOP"),
    }
    
    recording_start = time.time()
    sensor_read_time = 0
    
    try:
        while True:
            # Read sensors periodically
            if time.time() - sensor_read_time > 0.05:
                read_sensors()
                sensor_read_time = time.time()
            
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
                            "duration": round(duration, 3),
                            "start_yaw": round(command_start_yaw, 2),
                            "end_yaw": round(current_yaw, 2)
                        }
                        if recording_mode == "TO_TARGET":
                            to_target_commands.append(cmd_data)
                        else:
                            return_commands.append(cmd_data)
                break
            
            if key == 'l':
                # Save landmark
                landmark_count += 1
                elapsed = time.time() - recording_start
                
                landmark_data = {
                    "id": landmark_count,
                    "time": round(elapsed, 2),
                    "yaw": round(current_yaw, 2),
                    "mode": recording_mode,
                    "command_index": len(to_target_commands) if recording_mode == "TO_TARGET" else len(return_commands)
                }
                
                if use_camera:
                    img_file = capture_landmark(landmark_count, landmarks_dir)
                    if img_file:
                        landmark_data["image"] = img_file
                
                landmarks.append(landmark_data)
                print(f"🏁 Landmark #{landmark_count} saved at yaw={current_yaw:.1f}°")
                continue
            
            if key == 'k':
                # Switch to RETURN mode
                if recording_mode == "TO_TARGET":
                    if current_command and command_start_time:
                        duration = time.time() - command_start_time
                        if duration > 0.05:
                            to_target_commands.append({
                                "action": current_command[2],
                                "left": current_command[0],
                                "right": current_command[1],
                                "duration": round(duration, 3),
                                "start_yaw": round(command_start_yaw, 2),
                                "end_yaw": round(current_yaw, 2)
                            })
                    
                    send_cmd(0, 0)
                    current_command = None
                    command_start_time = None
                    recording_mode = "RETURN"
                    
                    # Auto-save target landmark
                    landmark_count += 1
                    target_landmark = {
                        "id": landmark_count,
                        "time": round(time.time() - recording_start, 2),
                        "yaw": round(current_yaw, 2),
                        "mode": "TARGET",
                        "command_index": len(to_target_commands)
                    }
                    if use_camera:
                        img_file = capture_landmark(landmark_count, landmarks_dir)
                        if img_file:
                            target_landmark["image"] = img_file
                    landmarks.append(target_landmark)
                    
                    print("\n" + "="*60)
                    print("🔄 SWITCHED TO RETURN MODE!")
                    print(f"   Target yaw: {current_yaw:.1f}°")
                    print("   Now drive the robot back to base.")
                    print("   Press Q when you arrive at base.")
                    print("="*60 + "\n")
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
                            "duration": round(duration, 3),
                            "start_yaw": round(command_start_yaw, 2),
                            "end_yaw": round(current_yaw, 2)
                        }
                        if recording_mode == "TO_TARGET":
                            to_target_commands.append(cmd_data)
                        else:
                            return_commands.append(cmd_data)
                
                # Start new command
                current_command = new_command
                command_start_time = time.time()
                command_start_yaw = current_yaw
                
                left, right, action = new_command
                send_cmd(left, right)
                
                elapsed = time.time() - recording_start
                mode_indicator = "→ TARGET" if recording_mode == "TO_TARGET" else "← RETURN"
                cmd_count = len(to_target_commands) if recording_mode == "TO_TARGET" else len(return_commands)
                print(f"[{elapsed:6.1f}s] {mode_indicator} | {action:8} | Yaw:{current_yaw:6.1f}° | Cmds:{cmd_count}")
            
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\n\nRecording interrupted!")
    
    finally:
        send_cmd(0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        if use_camera and camera is not None:
            camera.release()
        
        # Save recorded path
        if to_target_commands or return_commands:
            to_target_time = sum(cmd["duration"] for cmd in to_target_commands)
            return_time = sum(cmd["duration"] for cmd in return_commands)
            
            path_data = {
                "version": "smart_path_v1",
                "recorded_at": datetime.now().isoformat(),
                "total_duration": round(to_target_time + return_time, 2),
                "has_imu": True,
                "has_landmarks": len(landmarks) > 0,
                "landmarks_dir": landmarks_dir if landmarks else None,
                "to_target": {
                    "command_count": len(to_target_commands),
                    "duration": round(to_target_time, 2),
                    "commands": to_target_commands
                },
                "return": {
                    "command_count": len(return_commands),
                    "duration": round(return_time, 2),
                    "commands": return_commands
                },
                "landmarks": landmarks
            }
            
            with open(args.file, 'w') as f:
                json.dump(path_data, f, indent=2)
            
            print("\n" + "="*60)
            print(f"✅ SMART PATH SAVED: {args.file}")
            print(f"   TO TARGET: {len(to_target_commands)} commands ({to_target_time:.1f}s)")
            print(f"   RETURN:    {len(return_commands)} commands ({return_time:.1f}s)")
            print(f"   LANDMARKS: {len(landmarks)}")
            print("="*60)
            
            if to_target_commands:
                print("\nTO TARGET heading changes:")
                for i, cmd in enumerate(to_target_commands[:5]):
                    delta = cmd['end_yaw'] - cmd['start_yaw']
                    print(f"  {i+1}. {cmd['action']:8} {cmd['duration']:.2f}s | Δyaw={delta:+.1f}°")
                if len(to_target_commands) > 5:
                    print(f"  ... and {len(to_target_commands)-5} more")
        else:
            print("\n⚠️  No commands recorded!")

if __name__ == "__main__":
    main()
