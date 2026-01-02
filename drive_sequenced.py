import cv2
import numpy as np
import serial
import time
import torch
import torch.nn as nn

# --- CONFIGURATION ---
MODEL_PATH = "vga_pilot.pth"  # Updated to match train.py output
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
BASE_SPEED = 120  # Start conservative for 6V 620RPM motors
STEER_GAIN = 150  # Reduced for smoother control 

# --- MANUAL SCALER VALUES (From scaler.py extraction) ---
SCALER_MEAN = [610.9863945578231, 0.0, 0.0, 0.7589795918367351, -0.7293197278911563, 3.1393197278911606, -6.6284353741496576, -6.011950113378683, -6.999297052154189]
SCALER_SCALE = [425.87855549743256, 1.0, 1.0, 1.9902980330479252, 3.68364485890359, 5.623753073366564, 27.678314555711555, 29.323319206022408, 34.25960874804837]

# --- MANUAL SCALER CLASS ---
class ManualScaler:
    def __init__(self, mean, scale):
        self.mean = np.array(mean)
        self.scale = np.array(scale)
        
    def transform(self, X):
        return (X - self.mean) / self.scale

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Running on: {DEVICE}")

# --- MODEL ARCHITECTURE (Matches train.py - 640x480 VGA input) ---
class SequencedPilot(nn.Module):
    def __init__(self):
        super(SequencedPilot, self).__init__()
        
        # Vision Branch (CNN) - For 640x480 input
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),   # -> 238x318
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(), # -> 117x157
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(), # -> 57x77
            nn.Conv2d(48, 64, 3, stride=2), nn.ReLU(), # -> 28x38
            nn.Conv2d(64, 64, 3, stride=2), nn.ReLU(), # -> 13x18
            nn.Flatten()  # -> 14976
        )
        
        # Sensor Branch (MLP)
        self.mlp = nn.Sequential(
            nn.Linear(9, 64), nn.ReLU(),
            nn.Linear(64, 32), nn.ReLU()
        )
        
        # Fusion (14976 + 32 = 15008 inputs) - FIXED to match 640x480
        self.combined = nn.Sequential(
            nn.Linear(15008, 100), nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 1) # Output: Steering
        )

    def forward(self, img, sens):
        x1 = self.cnn(img)
        x2 = self.mlp(sens)
        x = torch.cat((x1, x2), dim=1)
        return self.combined(x)

# --- LOAD BRAIN ---
print("Loading AI...")
try:
    model = SequencedPilot().to(DEVICE)
    model.load_state_dict(torch.load(MODEL_PATH, map_location=DEVICE))
    model.eval() 
    print("Model loaded!")
except FileNotFoundError:
    print(f"ERROR: Could not find {MODEL_PATH}. Did you transfer it?")
    exit()
except Exception as e:
    print(f"ERROR loading model: {e}")
    exit()

# Initialize Manual Scaler
scaler = ManualScaler(SCALER_MEAN, SCALER_SCALE)

# --- SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Serial Error: {e}")
    # Try alternate ports
    for port in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
            time.sleep(2)
            ser.reset_input_buffer()
            print(f"Connected to {port}")
            break
        except:
            continue
    else:
        print("ERROR: No Arduino found!")
        exit()

# Sensor Store
sensor_data = np.zeros(9)
last_cmd_time = time.time()  # Track command timing

def send_cmd(left, right):
    global last_cmd_time
    cmd = f"<{int(left)},{int(right)}>"
    try:
        ser.write(cmd.encode('utf-8'))
        ser.flush()  # Ensure data is sent immediately
        last_cmd_time = time.time()
    except serial.SerialException as e:
        print(f"Serial write error: {e}")
    except:
        pass

def read_sensors():
    global sensor_data
    try:
        # Don't loop forever here, just grab what's available
        if ser.in_waiting > 0:
            # Read all lines to clear buffer and get latest
            lines = ser.read_all().decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                if line.startswith("START") and line.endswith("END"):
                    parts = line.split(',')
                    # Expecting: START, dist, lat, lon, ax, ay, az, gx, gy, gz, END
                    if len(parts) >= 10:
                        vals = [float(x) for x in parts[1:10]]
                        sensor_data = np.array(vals)
    except:
        pass

# --- CAMERA (Raspberry Pi Camera V2 on Jetson Nano) ---
def gstreamer_pipeline():
    # FIXED: Use 640x480 to match training resolution in train.py
    # Raspberry Pi Camera V2 connected via CSI
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)30/1 ! "
        "nvvidconv ! "
        "video/x-raw, width=640, height=480, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=true"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    print("Try running: sudo systemctl restart nvargus-daemon")
    exit()

print("--- STARTING DRIVE LOOP ---")
time.sleep(2) # Give camera time to warm up

# Warm up CUDA with a dummy inference
print("Warming up CUDA...")
dummy_img = torch.zeros(1, 3, 480, 640).to(DEVICE)
dummy_sens = torch.zeros(1, 9).to(DEVICE)
with torch.no_grad():
    _ = model(dummy_img, dummy_sens)
print("CUDA ready!")

frame_count = 0
start_time = time.time()

try:
    while True:
        read_sensors()
        
        ret, frame = cap.read()
        if not ret: 
            print("Camera Fail: No frame received from cap.read()")
            break
        
        frame_count += 1
        
        # 1. Preprocess Image
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        img = img / 255.0
        # Transpose to (C, H, W) and add Batch dim
        img = img.transpose((2, 0, 1))
        img_tensor = torch.tensor(img, dtype=torch.float32).unsqueeze(0).to(DEVICE)
        
        # 2. Preprocess Sensors
        sens_batch = sensor_data.reshape(1, -1)
        sens_scaled = scaler.transform(sens_batch)
        sens_tensor = torch.tensor(sens_scaled, dtype=torch.float32).to(DEVICE)
        
        # 3. Predict
        with torch.no_grad():
            steering = float(model(img_tensor, sens_tensor).item())
        
        # 4. Drive
        turn_val = steering * STEER_GAIN
        left_motor = BASE_SPEED + turn_val
        right_motor = BASE_SPEED - turn_val
        
        # Calculate FPS every 10 frames
        if frame_count % 10 == 0:
            fps = frame_count / (time.time() - start_time)
            print(f"FPS: {fps:.1f}")
        
        # Safety: Stop if wall < 20cm
        if sensor_data[0] < 20: 
            send_cmd(0, 0)
            print(f"OBSTACLE STOP ({sensor_data[0]:.0f}cm)")
        else:
            send_cmd(left_motor, right_motor)
            print(f"AI: {steering:.2f} | L:{left_motor:.0f} R:{right_motor:.0f} | D:{sensor_data[0]:.0f}cm")
            
        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    print("Cleaning up...")
    send_cmd(0, 0)
    cap.release()
