# 🤖 Autonomous Robot Project

An end-to-end autonomous driving robot that learns to navigate from human demonstrations using deep learning. The robot uses a camera and multiple sensors to drive autonomously after being trained on collected driving data.

![Architecture](https://img.shields.io/badge/Architecture-CNN%20+%20Sensor%20Fusion-blue)
![Platform](https://img.shields.io/badge/Platform-Jetson%20Nano-green)
![Status](https://img.shields.io/badge/Status-In%20Development-yellow)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Wiring Diagram](#wiring-diagram)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Setup Instructions](#setup-instructions)
- [Usage Guide](#usage-guide)
- [Troubleshooting](#troubleshooting)
- [Known Issues](#known-issues)
- [Contributing](#contributing)

---

## 🎯 Overview

This project implements a self-driving robot using **imitation learning** (behavioral cloning). The workflow is:

1. **Data Collection**: Drive the robot manually while recording camera images and sensor data
2. **Training**: Train a neural network on your computer to predict steering from images and sensors
3. **Deployment**: Load the trained model on the Jetson Nano and let the robot drive autonomously

### Key Features

- 🎥 Real-time camera-based navigation
- 📡 Multi-sensor fusion (Ultrasonic, IMU, GPS)
- 🧠 Deep CNN for visual processing
- 🛡️ Safety stop when obstacles detected
- 📊 Data collection with synchronized sensor logging

---

## 🔧 Hardware Requirements

### Main Components

| Component | Model/Specs | Purpose | Quantity |
|-----------|-------------|---------|----------|
| **NVIDIA Jetson Nano** | 2GB | Main brain / AI inference | 1 |
| **Arduino Mega 2560** | ATmega2560 | Sensor hub & motor control | 1 |
| **Raspberry Pi Camera V2** | IMX219 8MP, CSI | Vision input | 1 |
| **L298N Motor Driver** | Dual H-Bridge | Control 4 DC motors | 1 |
| **DC Gear Motors** | 6V 620RPM | Movement (4WD) | 4 |
| **HC-SR04 Ultrasonic** | 2cm-400cm range | Obstacle detection | 1 |
| **MPU6050 IMU** | 6-axis Gyro+Accel | Orientation & acceleration | 1 |
| **NEO-6M GPS Module** | u-blox | Location (outdoor only) | 1 |

### Power Supply

| Component | Power Source | Notes |
|-----------|--------------|-------|
| Jetson Nano | USB Power Bank (5V 3A minimum) | Must be stable 3A output |
| Motors | 2x 18650 Li-ion batteries (~7.4V) | Powers L298N & 6V motors |
| Arduino | Powered via USB from Jetson | No separate power needed |
| Sensors | Powered from Arduino 5V

---

## 🔌 Wiring Diagram

### Arduino Mega Connections

```
┌─────────────────────────────────────────────────────────────┐
│                     ARDUINO MEGA 2560                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  L298N MOTOR DRIVER:                                        │
│  ├── ENA (PWM)  →  Pin 2                                    │
│  ├── IN1        →  Pin 22                                   │
│  ├── IN2        →  Pin 23                                   │
│  ├── IN3        →  Pin 24                                   │
│  ├── IN4        →  Pin 25                                   │
│  └── ENB (PWM)  →  Pin 3                                    │
│                                                              │
│  HC-SR04 ULTRASONIC:                                        │
│  ├── Trig       →  Pin 4                                    │
│  ├── Echo       →  Pin 5                                    │
│  ├── VCC        →  5V                                       │
│  └── GND        →  GND                                      │
│                                                              │
│  NEO-6M GPS:                                                │
│  ├── TX         →  Pin 19 (RX1)                             │
│  ├── RX         →  Pin 18 (TX1)                             │
│  ├── VCC        →  5V                                       │
│  └── GND        →  GND                                      │
│                                                              │
│  MPU6050 IMU:                                               │
│  ├── SDA        →  Pin 20 (I2C SDA)                         │
│  ├── SCL        →  Pin 21 (I2C SCL)                         │
│  ├── VCC        →  3.3V                                     │
│  └── GND        →  GND                                      │
│                                                              │
│  SERIAL CONNECTION:                                          │
│  └── USB        →  Jetson Nano USB (/dev/ttyUSB0 or ACM0)   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Motor Wiring (4WD Configuration)

```
┌────────────────────────────────────┐
│           ROBOT TOP VIEW           │
├────────────────────────────────────┤
│                                    │
│   FRONT LEFT      FRONT RIGHT      │
│   ┌──────┐        ┌──────┐        │
│   │Motor1│        │Motor2│        │
│   └───┬──┘        └───┬──┘        │
│       │               │            │
│       └───────┬───────┘            │
│               │                    │
│         ┌─────┴─────┐              │
│         │  L298N    │              │
│         │ OUT1  OUT2│              │
│         │ OUT3  OUT4│              │
│         └─────┬─────┘              │
│               │                    │
│       ┌───────┴───────┐            │
│       │               │            │
│   ┌───┴──┐        ┌───┴──┐        │
│   │Motor3│        │Motor4│        │
│   └──────┘        └──────┘        │
│   REAR LEFT       REAR RIGHT       │
│                                    │
└────────────────────────────────────┘

Wire Motors in Parallel:
- LEFT SIDE (Motor1 + Motor3) → OUT1 & OUT2
- RIGHT SIDE (Motor2 + Motor4) → OUT3 & OUT4
```

---

## 💻 Software Requirements

### On Jetson Nano

```bash
# System packages
sudo apt update
sudo apt install python3-pip python3-opencv

# Python packages
pip3 install numpy pyserial torch torchvision

# For CSI Camera (should be pre-installed)
# GStreamer and nvarguscamerasrc
```

### On Training Computer

```bash
# Python 3.8+
pip install numpy pandas opencv-python torch torchvision scikit-learn joblib
```

### Arduino IDE Libraries

Install via Arduino Library Manager:
- `I2Cdev`
- `MPU6050`
- `TinyGPS++`

---

## 📁 Project Structure

```
autonomus/
├── data_collet.ino          # Arduino firmware for sensors & motors
├── path_follower.ino        # Arduino firmware for path playback (simpler)
├── smart_data_collector.py   # Data collection script (runs on Jetson)
├── train.py                  # Model training script (runs on PC)
├── drive_sequenced.py        # Autonomous driving with AI (runs on Jetson)
├── record_path.py            # Record robot path with WASD keys (NEW)
├── playback_path.py          # Playback recorded path autonomously (NEW)
├── scaler.py                 # Utility to extract scaler values
├── debug_autonomous.py       # Diagnostic tool
├── drive_improved.py         # Improved autonomous driver
├── dataset_vga/              # Collected training data
│   ├── log.csv               # Sensor & command log
│   └── img_*.jpg             # Training images
├── recorded_path.json        # Saved path for playback (generated)
├── vga_pilot.pth             # Trained model weights
├── vga_scaler.pkl            # Sensor normalization values
└── README.md                 # This file
```

---

## 🎮 Two Autonomous Modes

### Mode 1: AI Camera Navigation (Advanced)
Uses trained neural network + camera for intelligent navigation.
- Requires: Camera, trained model, lots of data
- Scripts: `smart_data_collector.py` → `train.py` → `drive_sequenced.py`

### Mode 2: Path Recording & Playback (Simple & Reliable)
Records your manual driving and replays it exactly.
- Requires: Just motors and ultrasonic sensor
- Scripts: `record_path.py` → `playback_path.py`
- **Perfect for indoor use or when camera has issues!**

---

## 🧠 How It Works

### Neural Network Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    SEQUENCED PILOT MODEL                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐     ┌─────────────────────────────────────┐   │
│  │ CAMERA IMAGE │     │         CNN (Vision Branch)          │   │
│  │  640 x 480   │ ──► │ Conv2d(3,24) → Conv2d(24,36) →       │   │
│  │    RGB/YUV   │     │ Conv2d(36,48) → Conv2d(48,64) →      │   │
│  └──────────────┘     │ Conv2d(64,64) → Flatten              │   │
│                       │ Output: 14976 features                │   │
│                       └───────────────────┬─────────────────┘   │
│                                           │                      │
│                                           ▼                      │
│                                    ┌──────────────┐              │
│  ┌──────────────┐                  │   CONCAT     │              │
│  │   SENSORS    │                  │  14976 + 32  │              │
│  │ [dist, gps,  │                  │   = 15008    │              │
│  │  imu x 6]    │                  └──────┬───────┘              │
│  └──────┬───────┘                         │                      │
│         │                                 ▼                      │
│         │           ┌─────────────────────────────────────┐     │
│         │           │      FUSION (Combined MLP)           │     │
│         ▼           │ Linear(15008,100) → Dropout(0.3) →   │     │
│  ┌──────────────┐   │ Linear(100,50) → Linear(50,1)        │     │
│  │ MLP (Sensor) │   │ Output: STEERING VALUE (-1 to +1)    │     │
│  │ 9→64→32      │   └─────────────────────────────────────┘     │
│  └──────┬───────┘                                                │
│         │                                                        │
│         └────────────────────────►  (to concat)                  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Control Flow

```
STEERING OUTPUT:
   -1.0 ◄────────── 0.0 ──────────► +1.0
   HARD LEFT       STRAIGHT        HARD RIGHT

MOTOR CALCULATION:
   left_motor  = BASE_SPEED + (steering × STEER_GAIN)
   right_motor = BASE_SPEED - (steering × STEER_GAIN)
```

---

## 🚀 Setup Instructions

### Step 1: Flash Arduino

1. Open `data_collet.ino` in Arduino IDE
2. Install required libraries (I2Cdev, MPU6050, TinyGPS++)
3. Select **Arduino Mega 2560** as board
4. Upload to Arduino

### Step 2: Test Hardware Connection

```bash
# On Jetson Nano, check Arduino connection
ls /dev/ttyUSB* /dev/ttyACM*

# Test serial communication
screen /dev/ttyUSB0 115200
# You should see: "START,999,0.000000,0.000000,..."
# Press Ctrl+A then K to exit
```

### Step 3: Test Camera

```bash
# On Jetson Nano
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink

# If camera doesn't work:
sudo systemctl restart nvargus-daemon
```

---

## 📖 Usage Guide

### 🎮 OPTION A: Path Recording (Simple & Reliable)

This method records your driving and replays it exactly - no camera or AI needed!

#### Step 1: Record a Path

```bash
cd ~/autonomus
python3 record_path.py
```

**Controls:**
| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `SPACE` | Stop |
| `Q` | Quit & Save |

Drive the robot from start to destination. Press Q when done.
This saves the path to `recorded_path.json`.

#### Step 2: Playback Autonomously

```bash
# Single run to target
python3 playback_path.py

# Go to target and return to base
python3 playback_path.py --reverse

# Continuous loop (target → base → target...)
python3 playback_path.py --loop

# Slower speed (0.5 = half speed)
python3 playback_path.py --speed 0.7
```

**Controls during playback:**
- `SPACE` = Pause/Resume
- `Q` = Stop immediately

---

### 🤖 OPTION B: AI Camera Navigation (Advanced)

#### Phase 1: Data Collection (On Jetson Nano)

```bash
# SSH into Jetson or use terminal
cd ~/autonomus
python3 smart_data_collector.py
```

**Controls:**
| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `C` | Stop |
| `Q` | Quit & Save |

**Tips for Good Data:**
- Drive smoothly, avoid jerky movements
- Cover the entire path multiple times
- Include recovery maneuvers (edge cases)
- Collect 500-2000+ samples for better results

### Phase 2: Training (On Your PC)

```bash
# Copy dataset from Jetson
scp -r jetson@<jetson-ip>:~/autonomus/dataset_vga ./

# Train the model
python train.py

# This creates:
# - vga_pilot.pth (model weights)
# - vga_scaler.pkl (sensor normalization)
```

### Phase 3: Extract Scaler Values

```bash
# On your PC, extract scaler for Jetson
python scaler.py

# Copy the output and update drive_sequenced.py:
# SCALER_MEAN = [...]
# SCALER_SCALE = [...]
```

### Phase 4: Deploy & Run (On Jetson Nano)

```bash
# Copy trained model to Jetson
scp vga_pilot.pth jetson@<jetson-ip>:~/autonomus/

# SSH to Jetson and run
python3 drive_sequenced.py

# Press Q to stop
```

---

## 🔍 Troubleshooting

### Camera Issues

| Problem | Solution |
|---------|----------|
| "Could not open camera" | Run `sudo systemctl restart nvargus-daemon` |
| Black/frozen image | Check CSI cable connection |
| Low FPS | Reduce resolution in GStreamer pipeline |

### Serial/Arduino Issues

| Problem | Solution |
|---------|----------|
| "Serial Error" | Check USB cable, try different port |
| No sensor data | Verify Arduino is uploaded correctly |
| Permission denied | Run `sudo chmod 666 /dev/ttyUSB0` |

### Motor Issues

| Problem | Solution |
|---------|----------|
| Motors don't move | Check battery voltage (should be > 7V) |
| Motors spin wrong way | Swap IN1/IN2 or IN3/IN4 wires |
| Only one side works | Check L298N connections |

### Autonomous Driving Issues

| Problem | Solution |
|---------|----------|
| Robot drives straight into wall | Check model loaded correctly, verify scaler values |
| Very jerky movement | Reduce STEER_GAIN, increase BASE_SPEED |
| Robot doesn't move | Check safety stop (distance < 20cm triggers stop) |
| Robot spins in circles | Training data may be imbalanced, collect more straight driving |

---

## ⚠️ Known Issues

### 1. GPS Doesn't Work Indoors
- **Problem**: GPS module requires clear sky view, won't get fix indoors
- **Impact**: GPS values will be 0.0, reducing sensor fusion effectiveness
- **Workaround**: Model relies primarily on camera; IMU and ultrasonic still work

### 2. Model Architecture Mismatch
- **Problem**: `drive_sequenced.py` uses 160x120 input, but `train.py` uses 640x480
- **Impact**: Model expects different image size than what's being fed
- **Solution**: Use the improved driving script that matches training resolution

### 3. Scaler Values May Be Outdated
- **Problem**: Hardcoded scaler values may not match your trained model
- **Solution**: Always run `scaler.py` after training and update values

### 4. Sensor Data Rate
- **Problem**: Arduino sends data every 1000ms (1Hz), which is slow
- **Impact**: Robot can't react quickly to sensor changes
- **Solution**: Modify Arduino code to send every 50ms (20Hz)

---

## 🤝 Contributing

### How to Help

1. **Test on your hardware** - Report what works/doesn't work
2. **Improve the model** - Experiment with different architectures
3. **Add features** - Lane detection, path planning, etc.
4. **Fix bugs** - Submit pull requests

### Development Setup

```bash
git clone https://github.com/YOUR_USERNAME/autonomus.git
cd autonomus
pip install -r requirements.txt  # Create this file with dependencies
```

### Code Style
- Python: Follow PEP 8
- Arduino: Use consistent indentation
- Comment complex logic

---

## 📊 Performance Tips

### For Better Autonomous Driving

1. **Collect More Data**: 1000+ samples recommended
2. **Balance Your Data**: Equal amounts of left/right/straight
3. **Train Longer**: Increase EPOCHS if validation loss still decreasing
4. **Tune Parameters**:
   - `BASE_SPEED`: Start low (100-140), increase gradually
   - `STEER_GAIN`: Start low (100-150), adjust for responsiveness

### For Faster Inference

1. Use Jetson's GPU (already configured with CUDA)
2. Reduce image resolution if needed
3. Consider TensorRT optimization

---

## 📝 License

MIT License - Feel free to use and modify!

---

## 🙏 Acknowledgments

- NVIDIA Jetson Community
- PyTorch Team
- OpenCV Contributors
- TinyGPS++ Library Authors

---

**Made with ❤️ for robotics enthusiasts**

*Last Updated: January 2026*
