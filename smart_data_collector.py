import cv2
import serial
import time
import os
import sys
import tty
import termios
import select
import csv
from datetime import datetime

# --- CONFIGURATION ---
BAUD_RATE = 115200
DATA_FOLDER = "dataset_vga" # High Res Data
SAMPLE_RATE = 0.1 

# Driving Speeds (MAX POWER)
SPEED_FWD = 255
SPEED_TURN = 255
KEY_TIMEOUT = 0.3

# Create Folder
if not os.path.exists(DATA_FOLDER):
    os.makedirs(DATA_FOLDER)

# Initialize CSV
csv_path = os.path.join(DATA_FOLDER, 'log.csv')
file_exists = os.path.isfile(csv_path)
csv_file = open(csv_path, 'a', newline='')
csv_writer = csv.writer(csv_file)

if not file_exists:
    csv_writer.writerow([
        "timestamp", "image_path", "cmd_left", "cmd_right", 
        "dist_cm", "gps_lat", "gps_lon", 
        "acc_x", "acc_y", "acc_z", 
        "gyro_x", "gyro_y", "gyro_z"
    ])

# --- ARDUINO CONNECTION ---
ser = None
s_dist = 999
s_vals = [0.0] * 9

def connect_arduino():
    global ser
    ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1', '/dev/ttyUSB1']
    for p in ports:
        if os.path.exists(p):
            try:
                print(f"Connecting to {p}...", end="")
                ser = serial.Serial(p, BAUD_RATE, timeout=0.05)
                time.sleep(2)
                ser.reset_input_buffer()
                print(" OK!")
                return True
            except:
                continue
    print(" Arduino NOT found!")
    return False

def send_cmd(left, right):
    if ser:
        try:
            cmd = f"<{int(left)},{int(right)}>"
            ser.write(cmd.encode('utf-8'))
        except: pass

def read_sensors():
    global s_dist, s_vals
    if not ser: return
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("START") and line.endswith("END"):
                parts = line.split(',')
                if len(parts) >= 10:
                    s_dist = int(parts[1])
                    s_vals = [float(x) for x in parts[2:11]]
    except: pass

# --- HIGH RES PIPELINE (640x480) ---
def gstreamer_pipeline():
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=640, height=480, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=true"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
if not cap.isOpened(): cap = cv2.VideoCapture(0)

# --- INPUT HELPERS ---
def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

if not connect_arduino(): sys.exit(1)

old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

print("\n--- VGA DATA COLLECTION (640x480) ---")
print("Controls: W, A, S, D. Press 'q' to Quit.")

count = 0
left_spd, right_spd = 0, 0
last_record_time = time.time()
last_key_time = time.time()

try:
    while True:
        key = get_key()
        
        if key in ['w', 's', 'a', 'd']:
            last_key_time = time.time()
            if key == 'w': left_spd, right_spd = SPEED_FWD, SPEED_FWD
            elif key == 's': left_spd, right_spd = -SPEED_FWD, -SPEED_FWD
            elif key == 'a': left_spd, right_spd = -SPEED_TURN, SPEED_TURN
            elif key == 'd': left_spd, right_spd = SPEED_TURN, -SPEED_TURN
        elif key == 'c': left_spd, right_spd = 0, 0
        elif key == 'q': break
        
        if time.time() - last_key_time > KEY_TIMEOUT:
            left_spd, right_spd = 0, 0

        send_cmd(left_spd, right_spd)
        read_sensors()
        ret, frame = cap.read()
        if not ret: break

        # RECORD
        if time.time() - last_record_time >= SAMPLE_RATE:
            if left_spd != 0 or right_spd != 0:
                now = datetime.now()
                ts_str = now.strftime("%Y%m%d_%H%M%S_%f")
                img_name = f"img_{ts_str}.jpg"
                cv2.imwrite(os.path.join(DATA_FOLDER, img_name), frame)
                row = [now.timestamp(), img_name, left_spd, right_spd, s_dist] + s_vals
                csv_writer.writerow(row)
                count += 1
                print(f"REC: {count} | D:{s_dist}cm", end='\r')
            last_record_time = time.time()

finally:
    send_cmd(0, 0)
    cap.release()
    csv_file.close()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    print(f"\nSaved {count} VGA samples.")
