#!/usr/bin/env python3
"""
Line Follower - Follow Colored Tape on Floor
=============================================
Robot follows a colored line (tape/band) on the floor.

Usage:
    python3 line_follower.py              # Use default (blue tape)
    python3 line_follower.py --color red  # Follow red line
    python3 line_follower.py --calibrate  # Calibrate colors first

Supported colors: red, blue, green, yellow, black, white

Controls:
    Q - Quit
    SPACE - Pause/Resume
    C - Calibrate (pick color from camera)
    +/- - Adjust speed

Tip: Use colored electrical tape or painter's tape on the floor.
"""

import serial
import time
import os
import sys
import tty
import termios
import select
import cv2
import numpy as np
import argparse

# --- CONFIGURATION ---
BAUD_RATE = 115200

# Motor speeds
BASE_SPEED = 180      # Normal forward speed
TURN_SPEED = 200      # Speed for turning
MIN_SPEED = 100       # Minimum speed when turning hard

# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Region of Interest - WHERE TO LOOK FOR LINE
# For FLOOR tape (camera tilted down):
#   ROI_TOP = 300, ROI_BOTTOM = 450  (bottom of image)
# For WALL tape (camera straight):
#   ROI_TOP = 150, ROI_BOTTOM = 350  (middle of image)
ROI_TOP = 150         # Adjust these based on where tape appears
ROI_BOTTOM = 350      # in camera view
ROI_BOTTOM = 450      # Region of interest for line detection

# Line detection
MIN_LINE_WIDTH = 20   # Minimum pixels to consider as line
MAX_LINE_WIDTH = 300  # Maximum pixels (to avoid detecting whole floor)

# PID Control for smooth following
KP = 0.5              # Proportional gain
KI = 0.0              # Integral gain
KD = 0.2              # Derivative gain

# Pre-defined color ranges (HSV)
COLOR_RANGES = {
    'red': {
        'lower1': np.array([0, 100, 100]),
        'upper1': np.array([10, 255, 255]),
        'lower2': np.array([160, 100, 100]),  # Red wraps around in HSV
        'upper2': np.array([180, 255, 255]),
    },
    'blue': {
        'lower': np.array([100, 100, 50]),
        'upper': np.array([130, 255, 255]),
    },
    'green': {
        'lower': np.array([40, 100, 50]),
        'upper': np.array([80, 255, 255]),
    },
    'yellow': {
        'lower': np.array([20, 100, 100]),
        'upper': np.array([40, 255, 255]),
    },
    'black': {
        'lower': np.array([0, 0, 0]),
        'upper': np.array([180, 255, 50]),
    },
    'white': {
        'lower': np.array([0, 0, 200]),
        'upper': np.array([180, 30, 255]),
    },
}

# Custom calibrated color (updated by calibration)
custom_color = None

# --- GLOBALS ---
ser = None
cap = None

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

def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def create_mask(hsv, color_name):
    """Create binary mask for specified color"""
    if color_name == 'custom' and custom_color:
        lower = custom_color['lower']
        upper = custom_color['upper']
        return cv2.inRange(hsv, lower, upper)
    
    color = COLOR_RANGES.get(color_name, COLOR_RANGES['blue'])
    
    if 'lower1' in color:
        # Red has two ranges (wraps around)
        mask1 = cv2.inRange(hsv, color['lower1'], color['upper1'])
        mask2 = cv2.inRange(hsv, color['lower2'], color['upper2'])
        return cv2.bitwise_or(mask1, mask2)
    else:
        return cv2.inRange(hsv, color['lower'], color['upper'])

def find_line_position(frame, color_name):
    """
    Find the center position of the line in the image.
    Returns: (position, width, found)
        position: -1.0 (far left) to 1.0 (far right), 0 = center
        width: width of detected line in pixels
        found: True if line was found
    """
    # Crop to region of interest (bottom of image = floor)
    roi = frame[ROI_TOP:ROI_BOTTOM, :]
    
    # Convert to HSV
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Create mask for line color
    mask = create_mask(hsv, color_name)
    
    # Clean up mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return 0, 0, False
    
    # Find largest contour (the line)
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    
    # Get bounding box
    x, y, w, h = cv2.boundingRect(largest)
    
    # Check if it's a valid line
    if w < MIN_LINE_WIDTH or w > MAX_LINE_WIDTH:
        return 0, 0, False
    
    # Calculate center of line
    line_center = x + w // 2
    frame_center = CAMERA_WIDTH // 2
    
    # Normalize to -1.0 to 1.0
    position = (line_center - frame_center) / frame_center
    
    return position, w, True

def calibrate_color():
    """Calibrate by sampling color from camera"""
    global custom_color
    
    print("\n" + "="*50)
    print("   COLOR CALIBRATION")
    print("="*50)
    print("1. Point camera at your tape/line")
    print("2. Press SPACE when tape fills the center of view")
    print("3. Press Q to cancel")
    print("="*50 + "\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Show what camera sees (in text)
        roi = frame[ROI_TOP:ROI_BOTTOM, :]
        center_region = roi[:, CAMERA_WIDTH//2-50:CAMERA_WIDTH//2+50]
        avg_color = np.mean(center_region, axis=(0,1))
        
        # Convert to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        center_hsv = hsv_frame[ROI_TOP:ROI_BOTTOM, CAMERA_WIDTH//2-50:CAMERA_WIDTH//2+50]
        avg_hsv = np.mean(center_hsv, axis=(0,1))
        
        print(f"\r  Center color - BGR:({avg_color[0]:.0f},{avg_color[1]:.0f},{avg_color[2]:.0f}) HSV:({avg_hsv[0]:.0f},{avg_hsv[1]:.0f},{avg_hsv[2]:.0f}) - Press SPACE to capture", end="")
        
        key = get_key()
        if key == ' ':
            # Create color range from sample
            h, s, v = avg_hsv
            custom_color = {
                'lower': np.array([max(0, h-15), max(0, s-50), max(0, v-50)]),
                'upper': np.array([min(180, h+15), min(255, s+50), min(255, v+50)])
            }
            print(f"\n\n✅ Color calibrated!")
            print(f"   HSV range: ({custom_color['lower'][0]}-{custom_color['upper'][0]}, "
                  f"{custom_color['lower'][1]}-{custom_color['upper'][1]}, "
                  f"{custom_color['lower'][2]}-{custom_color['upper'][2]})")
            return 'custom'
        elif key == 'q':
            print("\n\nCalibration cancelled.")
            return None
        
        time.sleep(0.1)

def main():
    global BASE_SPEED
    
    parser = argparse.ArgumentParser(description='Line follower robot')
    parser.add_argument('--color', '-c', default='blue', 
                        choices=['red', 'blue', 'green', 'yellow', 'black', 'white'],
                        help='Color of line to follow')
    parser.add_argument('--calibrate', action='store_true',
                        help='Calibrate color from camera')
    parser.add_argument('--speed', '-s', type=int, default=180,
                        help='Base speed (100-255)')
    args = parser.parse_args()
    
    BASE_SPEED = max(100, min(255, args.speed))
    color_name = args.color
    
    if not connect_arduino():
        sys.exit(1)
    
    if not init_camera():
        print("ERROR: Camera required for line following!")
        sys.exit(1)
    
    # Setup terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    # Calibrate if requested
    if args.calibrate:
        result = calibrate_color()
        if result:
            color_name = result
    
    print("\n" + "="*50)
    print("   LINE FOLLOWER")
    print("="*50)
    print(f"Following: {color_name.upper()} line")
    print(f"Speed: {BASE_SPEED}")
    print("\nControls:")
    print("  SPACE = Pause/Resume")
    print("  C = Calibrate color")
    print("  +/- = Adjust speed")
    print("  Q = Quit")
    print("="*50 + "\n")
    
    paused = True
    print("⏸️  PAUSED - Press SPACE to start following\n")
    
    # PID variables
    last_error = 0
    integral = 0
    last_line_time = time.time()
    
    try:
        while True:
            key = get_key()
            
            if key == 'q':
                break
            elif key == ' ':
                paused = not paused
                if paused:
                    send_cmd(0, 0)
                    print("\n⏸️  PAUSED")
                else:
                    print("\n▶️  Following line...")
            elif key == 'c':
                send_cmd(0, 0)
                paused = True
                result = calibrate_color()
                if result:
                    color_name = result
                print("\nPress SPACE to resume")
            elif key == '+' or key == '=':
                BASE_SPEED = min(255, BASE_SPEED + 20)
                print(f"\n  Speed: {BASE_SPEED}")
            elif key == '-':
                BASE_SPEED = max(100, BASE_SPEED - 20)
                print(f"\n  Speed: {BASE_SPEED}")
            
            if paused:
                time.sleep(0.05)
                continue
            
            # Read camera
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Find line
            position, width, found = find_line_position(frame, color_name)
            
            if found:
                last_line_time = time.time()
                
                # PID control
                error = position
                integral += error
                integral = max(-50, min(50, integral))  # Anti-windup
                derivative = error - last_error
                last_error = error
                
                correction = KP * error + KI * integral + KD * derivative
                
                # Calculate motor speeds
                left_speed = BASE_SPEED + int(correction * TURN_SPEED)
                right_speed = BASE_SPEED - int(correction * TURN_SPEED)
                
                # Clamp speeds
                left_speed = max(MIN_SPEED, min(255, left_speed))
                right_speed = max(MIN_SPEED, min(255, right_speed))
                
                send_cmd(left_speed, right_speed)
                
                # Status display
                pos_bar = ""
                pos_idx = int((position + 1) / 2 * 20)
                pos_bar = "-" * pos_idx + "|" + "-" * (20 - pos_idx)
                
                if abs(position) < 0.1:
                    status = "✅ CENTERED"
                elif position < 0:
                    status = "↩️  TURN LEFT"
                else:
                    status = "↪️  TURN RIGHT"
                
                print(f"\r  [{pos_bar}] {status:12} | L:{left_speed:3} R:{right_speed:3} | Width:{width:3}px   ", end="")
            
            else:
                # Line lost
                time_since_line = time.time() - last_line_time
                
                if time_since_line < 0.5:
                    # Recently lost - keep going, might find it
                    print(f"\r  🔍 Searching for line...                                    ", end="")
                elif time_since_line < 2.0:
                    # Lost for a bit - slow down and search
                    send_cmd(MIN_SPEED, MIN_SPEED)
                    print(f"\r  ⚠️  Line lost! Slowing down...                              ", end="")
                else:
                    # Lost for too long - stop
                    send_cmd(0, 0)
                    print(f"\r  ❌ LINE NOT FOUND - Move robot to line and press SPACE      ", end="")
                    paused = True
            
            time.sleep(0.03)  # ~30 FPS
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    
    finally:
        send_cmd(0, 0)
        if cap:
            cap.release()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n🏁 Line follower stopped.")

if __name__ == "__main__":
    main()
