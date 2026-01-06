#!/usr/bin/env python3
"""
Video Landmark Recording Script
================================
Records paths with VIDEO CLIP landmarks that capture:
- 2-second video approaching the landmark
- Distance profile from ultrasonic sensor
- Yaw angle from IMU

This creates robust multi-signal landmarks for reliable position verification.

Usage:
    python3 video_record.py                    # Record with video landmarks
    python3 video_record.py --file mypath.json # Custom filename
    python3 video_record.py --clip-duration 3  # 3-second clips

Controls:
    W - Forward
    S - Backward  
    A - Turn Left
    D - Turn Right
    SPACE - Stop/Pause
    L - Start LANDMARK recording (continues moving for 2 seconds)
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
import threading
from datetime import datetime

# --- CONFIGURATION ---
BAUD_RATE = 115200
DEFAULT_PATH_FILE = "video_path.json"
LANDMARKS_DIR = "video_landmarks"

# Motor Speeds
SPEED_FWD = 255
SPEED_TURN = 255
SPEED_STOP = 0

# Landmark recording
CLIP_DURATION = 2.0  # seconds
CLIP_FPS = 30

# --- GLOBALS ---
ser = None
current_yaw = 0.0
current_distance = 999
camera = None
use_camera = False
cv2 = None

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
    global camera, use_camera, cv2
    try:
        import cv2 as cv2_module
        cv2 = cv2_module
        
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
            for _ in range(10):
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

def record_video_landmark(landmark_id, landmarks_dir, duration=2.0, keep_moving=True, left_speed=255, right_speed=255):
    """
    Record a video clip with synchronized distance profile.
    Keeps sending motor commands to prevent watchdog timeout.
    Returns: (video_filename, sensor_data_dict)
    """
    global camera, current_distance, current_yaw, cv2, ser
    
    if not use_camera or camera is None:
        return None, {"timestamps": [], "distances": [], "yaws": []}
    
    os.makedirs(landmarks_dir, exist_ok=True)
    
    # Video writer setup
    video_filename = f"landmark_{landmark_id:03d}.avi"
    video_path = os.path.join(landmarks_dir, video_filename)
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_path, fourcc, CLIP_FPS, (640, 480))
    
    # Data profiles
    distance_profile = []
    yaw_profile = []
    timestamps = []
    
    print(f"📹 Recording landmark #{landmark_id}...")
    
    start_time = time.time()
    frame_count = 0
    last_cmd_time = 0
    
    while time.time() - start_time < duration:
        # Capture frame
        ret, frame = camera.read()
        if ret:
            out.write(frame)
            frame_count += 1
        
        # Read sensors
        read_sensors()
        
        # IMPORTANT: Keep sending motor commands to prevent watchdog timeout
        # Send every 100ms to keep Arduino happy
        if keep_moving and time.time() - last_cmd_time > 0.1:
            send_cmd(left_speed, right_speed)
            last_cmd_time = time.time()
        
        # Record sensor data
        elapsed = time.time() - start_time
        timestamps.append(round(elapsed, 3))
        distance_profile.append(current_distance)
        yaw_profile.append(round(current_yaw, 2))
        
        # Target ~30fps
        time.sleep(0.033)
    
    out.release()
    
    print(f"📹 Recorded {frame_count} frames, {len(distance_profile)} sensor samples")
    
    return video_filename, {
        "timestamps": timestamps,
        "distances": distance_profile,
        "yaws": yaw_profile
    }

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
                    parts = line.replace("START,", "").replace(",END", "").strip().split(',')
                    if len(parts) >= 2:
                        try:
                            current_distance = int(parts[0])
                            current_yaw = float(parts[1])
                        except:
                            pass
    except:
        pass

def get_key():
    """Non-blocking key read"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def main():
    global use_camera, CLIP_DURATION
    
    parser = argparse.ArgumentParser(description='Video landmark path recording')
    parser.add_argument('--file', '-f', default=DEFAULT_PATH_FILE, help='Output file')
    parser.add_argument('--clip-duration', '-d', type=float, default=2.0, help='Landmark clip duration (seconds)')
    args = parser.parse_args()
    
    CLIP_DURATION = args.clip_duration
    
    if not connect_arduino():
        sys.exit(1)
    
    # Initialize camera
    if not init_camera():
        print("ERROR: Camera required for video landmarks!")
        sys.exit(1)
    
    # Reset Arduino yaw
    print("Resetting IMU yaw...")
    reset_yaw()
    time.sleep(0.5)
    
    # Setup terminal for raw input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print("\n" + "="*60)
    print("   VIDEO LANDMARK RECORDING MODE")
    print("="*60)
    print("Controls:")
    print("  W = Forward    S = Backward")
    print("  A = Left       D = Right")
    print("  SPACE = Stop")
    print(f"  L = Record VIDEO LANDMARK ({CLIP_DURATION}s clip while moving)")
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
    
    # Command mapping (Swing turns: one motor stopped, other moves)
    key_to_motors = {
        'w': (SPEED_FWD, SPEED_FWD, "FORWARD"),
        's': (-SPEED_FWD, -SPEED_FWD, "BACKWARD"),
        'a': (0, SPEED_TURN, "LEFT"),       # Stop left, right forward
        'd': (SPEED_TURN, 0, "RIGHT"),      # Left forward, stop right
        ' ': (SPEED_STOP, SPEED_STOP, "STOP"),
    }
    
    recording_start = time.time()
    sensor_read_time = 0
    recording_landmark = False
    
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
            
            if key == 'l' and not recording_landmark:
                # Start video landmark recording
                recording_landmark = True
                landmark_count += 1
                elapsed = time.time() - recording_start
                
                # Save current command before landmark
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
                
                # Record video clip (robot continues moving!)
                start_yaw = current_yaw
                # Pass current motor speeds so robot keeps moving during recording
                curr_left = current_command[0] if current_command else 0
                curr_right = current_command[1] if current_command else 0
                video_file, sensor_data = record_video_landmark(
                    landmark_count, landmarks_dir, CLIP_DURATION,
                    keep_moving=True, left_speed=curr_left, right_speed=curr_right
                )
                
                landmark_data = {
                    "id": landmark_count,
                    "type": "video_clip",
                    "time": round(elapsed, 2),
                    "start_yaw": round(start_yaw, 2),
                    "end_yaw": round(current_yaw, 2),
                    "mode": recording_mode,
                    "command_index": len(to_target_commands) if recording_mode == "TO_TARGET" else len(return_commands),
                    "video": video_file,
                    "sensor_profile": sensor_data,
                    "clip_duration": CLIP_DURATION
                }
                
                landmarks.append(landmark_data)
                print(f"🏁 Video Landmark #{landmark_count} saved!")
                print(f"   Distance range: {min(sensor_data['distances'])}cm - {max(sensor_data['distances'])}cm")
                print(f"   Yaw change: {start_yaw:.1f}° → {current_yaw:.1f}°")
                
                # Reset command timing
                command_start_time = time.time()
                command_start_yaw = current_yaw
                recording_landmark = False
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
                    
                    # Auto-record target landmark
                    landmark_count += 1
                    print("\n📹 Recording TARGET landmark...")
                    
                    # For target, record a stationary clip (robot stopped)
                    start_yaw = current_yaw
                    video_file, sensor_data = record_video_landmark(
                        landmark_count, landmarks_dir, CLIP_DURATION,
                        keep_moving=False, left_speed=0, right_speed=0
                    )
                    
                    target_landmark = {
                        "id": landmark_count,
                        "type": "target_location",
                        "time": round(time.time() - recording_start, 2),
                        "start_yaw": round(start_yaw, 2),
                        "end_yaw": round(current_yaw, 2),
                        "mode": "TARGET",
                        "command_index": len(to_target_commands),
                        "video": video_file,
                        "sensor_profile": sensor_data,
                        "clip_duration": CLIP_DURATION
                    }
                    landmarks.append(target_landmark)
                    
                    print("\n" + "="*60)
                    print("🔄 SWITCHED TO RETURN MODE!")
                    print(f"   Target yaw: {current_yaw:.1f}°")
                    print("   Now drive the robot back to base.")
                    print("   Press L to add landmarks on the way back.")
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
                print(f"[{elapsed:6.1f}s] {mode_indicator} | {action:8} | Yaw:{current_yaw:6.1f}° | Dist:{current_distance:3}cm | Cmds:{cmd_count}")
            
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
                "version": "video_path_v1",
                "recorded_at": datetime.now().isoformat(),
                "total_duration": round(to_target_time + return_time, 2),
                "has_imu": True,
                "has_video_landmarks": True,
                "landmarks_dir": landmarks_dir,
                "clip_duration": CLIP_DURATION,
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
            print(f"✅ VIDEO PATH SAVED: {args.file}")
            print(f"   TO TARGET: {len(to_target_commands)} commands ({to_target_time:.1f}s)")
            print(f"   RETURN:    {len(return_commands)} commands ({return_time:.1f}s)")
            print(f"   VIDEO LANDMARKS: {len(landmarks)}")
            print(f"   Landmarks saved to: {landmarks_dir}/")
            print("="*60)
            
            # Show landmark summary
            if landmarks:
                print("\nLandmarks recorded:")
                for lm in landmarks:
                    dist_range = f"{min(lm['sensor_profile']['distances'])}-{max(lm['sensor_profile']['distances'])}cm"
                    print(f"  #{lm['id']} [{lm['mode']}] yaw={lm['start_yaw']:.1f}° dist={dist_range}")
        else:
            print("\n⚠️  No commands recorded!")

if __name__ == "__main__":
    main()
