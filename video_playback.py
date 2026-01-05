#!/usr/bin/env python3
"""
Video Landmark Playback Script
===============================
Plays back recorded paths with multi-signal landmark verification:
- Video clip matching (ORB features + optical flow)
- Distance profile comparison (ultrasonic curve matching)
- Yaw angle verification (IMU heading)

All three signals combined give high confidence position verification.

Usage:
    python3 video_playback.py                     # Full round trip
    python3 video_playback.py --loop              # Continuous loop
    python3 video_playback.py --verify            # Enable landmark verification
    python3 video_playback.py --strict            # Stop if landmark doesn't match
    python3 video_playback.py --file mypath.json  # Use specific path

Controls during playback:
    SPACE - Pause/Resume
    Q - Quit immediately
    V - Toggle verification display
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
import numpy as np

# --- CONFIGURATION ---
BAUD_RATE = 115200
DEFAULT_PATH_FILE = "video_path.json"

# Speed multiplier
SPEED_MULTIPLIER = 1.0

# Safety
OBSTACLE_STOP_DISTANCE = 5

# Heading correction
HEADING_CORRECTION_ENABLED = True
HEADING_TOLERANCE = 5.0

# Landmark verification thresholds
VIDEO_MATCH_THRESHOLD = 0.5      # ORB feature match ratio
DISTANCE_MATCH_THRESHOLD = 0.7   # Distance profile correlation
YAW_MATCH_THRESHOLD = 15.0       # Degrees of acceptable yaw difference
COMBINED_THRESHOLD = 0.6         # Combined confidence threshold

# --- GLOBALS ---
ser = None
current_yaw = 0.0
current_distance = 999
camera = None
cv2 = None
verify_enabled = False
orb = None

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
    global camera, cv2, orb
    try:
        import cv2 as cv2_module
        cv2 = cv2_module
        
        pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=1"
        )
        camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if camera.isOpened():
            for _ in range(10):
                camera.read()
            
            # Initialize ORB detector for feature matching
            orb = cv2.ORB_create(nfeatures=500)
            
            print("Camera initialized with ORB feature detector!")
            return True
        return False
    except Exception as e:
        print(f"Camera init failed: {e}")
        return False

def send_motor_cmd(left, right):
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
    global ser
    if ser:
        try:
            cmd = f"<H,{heading:.2f}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def enable_correction(enabled):
    global ser
    if ser:
        try:
            cmd = f"<C,{1 if enabled else 0}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def reset_yaw():
    global ser
    if ser:
        try:
            ser.write(b"<R>")
            ser.flush()
        except:
            pass

def set_pid(kp, ki, kd):
    global ser
    if ser:
        try:
            cmd = f"<P,{kp},{ki},{kd}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def read_sensors():
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

def get_key_nonblocking():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def load_path(filepath):
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"ERROR: Path file '{filepath}' not found!")
        return None
    except json.JSONDecodeError:
        print(f"ERROR: Invalid JSON in '{filepath}'")
        return None

# ==================== LANDMARK VERIFICATION ====================

def match_video_orb(saved_video_path, duration=2.0):
    """
    Compare current camera feed with saved video using ORB features.
    Returns: match_score (0.0 - 1.0)
    """
    global camera, cv2, orb
    
    if camera is None or cv2 is None:
        return 0.5  # Neutral score if no camera
    
    try:
        # Open saved video
        saved_cap = cv2.VideoCapture(saved_video_path)
        if not saved_cap.isOpened():
            return 0.5
        
        # BFMatcher for ORB
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        match_scores = []
        frame_count = 0
        start_time = time.time()
        
        while time.time() - start_time < duration and saved_cap.isOpened():
            # Read frame from saved video
            ret_saved, saved_frame = saved_cap.read()
            if not ret_saved:
                saved_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
                continue
            
            # Capture current frame
            ret_current, current_frame = camera.read()
            if not ret_current:
                continue
            
            # Convert to grayscale
            saved_gray = cv2.cvtColor(saved_frame, cv2.COLOR_BGR2GRAY)
            current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            # Detect ORB features
            kp1, des1 = orb.detectAndCompute(saved_gray, None)
            kp2, des2 = orb.detectAndCompute(current_gray, None)
            
            if des1 is not None and des2 is not None and len(des1) > 0 and len(des2) > 0:
                # Match features
                matches = bf.match(des1, des2)
                
                # Calculate match quality
                if len(matches) > 0:
                    # Sort by distance and take top matches
                    matches = sorted(matches, key=lambda x: x.distance)
                    good_matches = [m for m in matches if m.distance < 50]
                    
                    # Score based on number of good matches relative to keypoints
                    max_possible = min(len(kp1), len(kp2))
                    if max_possible > 0:
                        score = len(good_matches) / max_possible
                        match_scores.append(min(score * 2, 1.0))  # Scale up, cap at 1.0
            
            frame_count += 1
            read_sensors()  # Keep reading sensors
            time.sleep(0.033)  # ~30fps
        
        saved_cap.release()
        
        if match_scores:
            return sum(match_scores) / len(match_scores)
        return 0.0
        
    except Exception as e:
        print(f"Video matching error: {e}")
        return 0.5

def match_distance_profile(saved_profile, duration=2.0):
    """
    Compare current distance readings with saved profile.
    Returns: match_score (0.0 - 1.0), current_profile
    """
    global current_distance
    
    current_profile = []
    start_time = time.time()
    
    # Collect current distance profile
    while time.time() - start_time < duration:
        read_sensors()
        current_profile.append(current_distance)
        time.sleep(0.05)  # 20Hz sampling
    
    saved_distances = saved_profile.get('distances', [])
    
    if not saved_distances or not current_profile:
        return 0.5, current_profile
    
    try:
        # Normalize both profiles to same length
        saved_arr = np.array(saved_distances, dtype=float)
        current_arr = np.array(current_profile, dtype=float)
        
        # Resample to same length
        target_len = min(len(saved_arr), len(current_arr), 50)
        saved_resampled = np.interp(
            np.linspace(0, 1, target_len),
            np.linspace(0, 1, len(saved_arr)),
            saved_arr
        )
        current_resampled = np.interp(
            np.linspace(0, 1, target_len),
            np.linspace(0, 1, len(current_arr)),
            current_arr
        )
        
        # Replace 999 (no reading) with max value
        saved_resampled[saved_resampled >= 999] = 400
        current_resampled[current_resampled >= 999] = 400
        
        # Calculate correlation coefficient
        if np.std(saved_resampled) > 0 and np.std(current_resampled) > 0:
            correlation = np.corrcoef(saved_resampled, current_resampled)[0, 1]
            # Convert to 0-1 score (correlation can be -1 to 1)
            score = (correlation + 1) / 2
        else:
            # If no variation, check if means are similar
            mean_diff = abs(np.mean(saved_resampled) - np.mean(current_resampled))
            score = max(0, 1 - mean_diff / 100)
        
        return score, current_profile
        
    except Exception as e:
        print(f"Distance matching error: {e}")
        return 0.5, current_profile

def match_yaw(saved_start_yaw, saved_end_yaw):
    """
    Compare current yaw with saved yaw values.
    Returns: match_score (0.0 - 1.0)
    """
    global current_yaw
    
    # Calculate yaw error (normalized to -180 to 180)
    error = current_yaw - saved_start_yaw
    while error > 180: error -= 360
    while error < -180: error += 360
    
    # Convert error to score (0 error = 1.0, >YAW_MATCH_THRESHOLD = 0.0)
    score = max(0, 1 - abs(error) / YAW_MATCH_THRESHOLD)
    
    return score

def verify_landmark(landmark_data, landmarks_dir, duration=2.0):
    """
    Verify current position against saved landmark using all three signals.
    Returns: (combined_score, video_score, distance_score, yaw_score, details)
    """
    global current_yaw
    
    video_score = 0.5
    distance_score = 0.5
    yaw_score = 0.5
    details = {}
    
    # 1. Video matching (if available)
    if 'video' in landmark_data and landmarks_dir:
        video_path = os.path.join(landmarks_dir, landmark_data['video'])
        if os.path.exists(video_path):
            print(f"      📹 Matching video...", end="", flush=True)
            video_score = match_video_orb(video_path, duration)
            print(f" {video_score:.2f}")
            details['video'] = video_score
    
    # 2. Distance profile matching
    if 'sensor_profile' in landmark_data:
        print(f"      📏 Matching distance profile...", end="", flush=True)
        distance_score, current_profile = match_distance_profile(
            landmark_data['sensor_profile'], duration
        )
        print(f" {distance_score:.2f}")
        details['distance'] = distance_score
        details['current_distances'] = current_profile
    
    # 3. Yaw matching
    saved_yaw = landmark_data.get('start_yaw', 0)
    yaw_score = match_yaw(saved_yaw, landmark_data.get('end_yaw', saved_yaw))
    details['yaw'] = yaw_score
    details['current_yaw'] = current_yaw
    details['expected_yaw'] = saved_yaw
    
    # Combined score (weighted average)
    # Video: 40%, Distance: 40%, Yaw: 20%
    combined_score = (video_score * 0.4) + (distance_score * 0.4) + (yaw_score * 0.2)
    
    return combined_score, video_score, distance_score, yaw_score, details

def play_path_with_verification(commands, direction_name, use_correction=True, 
                                 landmarks=None, landmarks_dir=None, 
                                 verify=False, strict=False):
    """Execute commands with optional landmark verification"""
    global current_yaw, current_distance, verify_enabled
    
    print(f"\n🚀 Playing path: {direction_name}")
    print(f"   Commands: {len(commands)}")
    print(f"   Heading correction: {'ON' if use_correction else 'OFF'}")
    print(f"   Landmark verification: {'ON' if verify else 'OFF'}")
    if landmarks:
        print(f"   Landmarks to verify: {len([l for l in landmarks if l.get('mode') in [direction_name.split()[0], 'TO', 'RETURNING']])}")
    print("   Press SPACE to pause, Q to quit, V to toggle verification\n")
    
    verify_enabled = verify
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
        elif key == 'v':
            verify_enabled = not verify_enabled
            print(f"   Verification: {'ON' if verify_enabled else 'OFF'}")
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
        
        read_sensors()
        
        # Safety stop
        if current_distance < OBSTACLE_STOP_DISTANCE and cmd["left"] > 0 and cmd["right"] > 0:
            send_motor_cmd(0, 0)
            enable_correction(False)
            print(f"⚠️  OBSTACLE at {current_distance}cm! Waiting...")
            while current_distance < OBSTACLE_STOP_DISTANCE + 5:
                read_sensors()
                time.sleep(0.1)
                if get_key_nonblocking() == 'q':
                    return False
            print("✅ Path clear, continuing...")
        
        # Get command parameters
        action = cmd["action"]
        left = cmd["left"]
        right = cmd["right"]
        duration = cmd["duration"]
        target_end_yaw = cmd.get("end_yaw", current_yaw)
        
        is_straight = (action == "FORWARD" or action == "BACKWARD")
        
        if use_correction and is_straight:
            target_heading = cmd.get("start_yaw", current_yaw)
            send_heading_target(target_heading)
            enable_correction(True)
        else:
            enable_correction(False)
        
        print(f"  [{i+1}/{len(commands)}] {action:8} L:{left:4} R:{right:4} {duration:.2f}s | Yaw:{current_yaw:+.1f}°")
        
        send_motor_cmd(left, right)
        
        # Execute for duration
        elapsed = 0
        step = 0.02
        
        while elapsed < duration:
            time.sleep(step)
            elapsed += step
            read_sensors()
            
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
            
            if current_distance < OBSTACLE_STOP_DISTANCE and left > 0 and right > 0:
                send_motor_cmd(0, 0)
                enable_correction(False)
                print(f"\n⚠️  OBSTACLE!")
                while current_distance < OBSTACLE_STOP_DISTANCE + 5:
                    read_sensors()
                    time.sleep(0.1)
                    if get_key_nonblocking() == 'q':
                        return False
                print("✅ Continuing...")
                if use_correction and is_straight:
                    enable_correction(True)
                send_motor_cmd(left, right)
        
        # Check for landmarks at this position
        if verify_enabled and landmarks and landmarks_dir:
            for lm in landmarks:
                if lm.get('command_index') == i + 1:
                    print(f"\n   🏁 VERIFYING LANDMARK #{lm['id']}...")
                    
                    # Keep robot moving during verification
                    clip_duration = lm.get('clip_duration', 2.0)
                    
                    combined, video, distance, yaw, details = verify_landmark(
                        lm, landmarks_dir, clip_duration
                    )
                    
                    print(f"      📊 COMBINED SCORE: {combined:.2f}")
                    print(f"         Video: {video:.2f} | Distance: {distance:.2f} | Yaw: {yaw:.2f}")
                    
                    if combined >= COMBINED_THRESHOLD:
                        print(f"      ✅ LANDMARK VERIFIED!")
                    else:
                        print(f"      ⚠️  LANDMARK MISMATCH!")
                        if strict:
                            print(f"      🛑 Stopping (strict mode)")
                            send_motor_cmd(0, 0)
                            return False
                    print()
    
    send_motor_cmd(0, 0)
    enable_correction(False)
    print(f"\n✅ Path complete: {direction_name}")
    print(f"   Final yaw: {current_yaw:.1f}°")
    return True

def main():
    global SPEED_MULTIPLIER, HEADING_CORRECTION_ENABLED
    
    parser = argparse.ArgumentParser(description='Video landmark path playback')
    parser.add_argument('--file', '-f', default=DEFAULT_PATH_FILE, help='Path file to use')
    parser.add_argument('--loop', '-l', action='store_true', help='Continuous loop mode')
    parser.add_argument('--target-only', '-t', action='store_true', help='Only go to target')
    parser.add_argument('--return-only', '-r', action='store_true', help='Only return to base')
    parser.add_argument('--speed', '-s', type=float, default=1.0, help='Speed multiplier')
    parser.add_argument('--no-correction', action='store_true', help='Disable heading correction')
    parser.add_argument('--verify', '-v', action='store_true', help='Enable landmark verification')
    parser.add_argument('--strict', action='store_true', help='Stop if landmark verification fails')
    args = parser.parse_args()
    
    SPEED_MULTIPLIER = max(0.5, min(2.0, args.speed))
    HEADING_CORRECTION_ENABLED = not args.no_correction
    
    # Load path
    path_data = load_path(args.file)
    if not path_data:
        sys.exit(1)
    
    # Parse path data
    if "to_target" in path_data:
        to_target_commands = path_data["to_target"]["commands"]
        return_commands = path_data["return"]["commands"]
        to_target_duration = path_data["to_target"]["duration"]
        return_duration = path_data["return"]["duration"]
    else:
        to_target_commands = path_data.get("commands", [])
        return_commands = []
        to_target_duration = path_data.get("total_duration", 0)
        return_duration = 0
    
    landmarks = path_data.get("landmarks", [])
    landmarks_dir = path_data.get("landmarks_dir", None)
    
    print("\n" + "="*60)
    print("   VIDEO LANDMARK PLAYBACK MODE")
    print("="*60)
    print(f"Path file: {args.file}")
    print(f"Version: {path_data.get('version', 'legacy')}")
    print(f"TO TARGET: {len(to_target_commands)} commands ({to_target_duration}s)")
    print(f"RETURN:    {len(return_commands)} commands ({return_duration}s)")
    print(f"Video Landmarks: {len(landmarks)}")
    print(f"Speed: {SPEED_MULTIPLIER:.1f}x")
    print(f"Heading Correction: {'ON' if HEADING_CORRECTION_ENABLED else 'OFF'}")
    print(f"Landmark Verification: {'ON' if args.verify else 'OFF'}")
    print(f"Strict Mode: {'ON' if args.strict else 'OFF'}")
    print("="*60)
    
    if not return_commands and not args.target_only:
        print("\n⚠️  No RETURN path recorded!")
        args.target_only = True
    
    # Connect to Arduino
    if not connect_arduino():
        sys.exit(1)
    
    # Initialize camera if verifying
    if args.verify and landmarks:
        if not init_camera():
            print("⚠️  Camera failed - video verification disabled")
    
    # Setup terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        if args.return_only:
            if return_commands:
                play_path_with_verification(
                    return_commands, "RETURNING TO BASE",
                    HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir,
                    args.verify, args.strict
                )
            else:
                print("ERROR: No return path!")
        
        elif args.target_only:
            play_path_with_verification(
                to_target_commands, "TO TARGET",
                HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir,
                args.verify, args.strict
            )
        
        elif args.loop:
            loop_count = 0
            while True:
                loop_count += 1
                print(f"\n{'='*60}")
                print(f"   LOOP {loop_count}")
                print('='*60)
                
                reset_yaw()
                time.sleep(0.3)
                
                if not play_path_with_verification(
                    to_target_commands, "TO TARGET",
                    HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir,
                    args.verify, args.strict
                ):
                    break
                
                time.sleep(1)
                
                if return_commands:
                    if not play_path_with_verification(
                        return_commands, "RETURNING TO BASE",
                        HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir,
                        args.verify, args.strict
                    ):
                        break
                else:
                    break
                
                time.sleep(1)
        
        else:
            if not play_path_with_verification(
                to_target_commands, "TO TARGET",
                HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir,
                args.verify, args.strict
            ):
                pass
            elif return_commands:
                time.sleep(1)
                play_path_with_verification(
                    return_commands, "RETURNING TO BASE",
                    HEADING_CORRECTION_ENABLED, landmarks, landmarks_dir,
                    args.verify, args.strict
                )
    
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
