# ✅ Issues Found & Fixed

This document explains the problems found in the autonomous driving setup and how they were fixed.

---

## ✅ FIXED: Resolution Mismatch

### The Problem
Your **training script** (`train.py`) uses **640x480** images, but `drive_sequenced.py` was using **160x120** images.

### ✅ The Fix Applied
Updated `drive_sequenced.py` GStreamer pipeline to use 640x480:
```python
"video/x-raw, width=640, height=480, format=(string)BGRx ! "
```

---

## ✅ FIXED: Model Architecture Mismatch

### The Problem
The CNN output size depends on input image dimensions. The fusion layer had wrong size.

### ✅ The Fix Applied
Updated `drive_sequenced.py` model architecture to match train.py:
- Added stride=2 to all conv layers
- Changed fusion layer from 6688 to 15008 inputs

---

## ✅ FIXED: Model File Name Mismatch

### The Problem
- `train.py` saves the model as `vga_pilot.pth`
- `drive_sequenced.py` looked for `sequenced_pilot.pth`

### ✅ The Fix Applied
Updated MODEL_PATH in `drive_sequenced.py` to `vga_pilot.pth`

---

## ✅ FIXED: Slow Sensor Update Rate

### The Problem
Arduino was sending sensors only once per second (1Hz) - too slow for real-time driving.

### ✅ The Fix Applied
Changed `data_collet.ino` to send every 50ms (20Hz):
```cpp
const int SENSOR_INTERVAL = 50; // 20Hz
```

**⚠️ IMPORTANT: You need to re-upload the Arduino code!**

---

## 🟢 GPS Won't Work Indoors (Expected)

### The Problem
GPS requires clear sky view. Indoors, you'll always get:
```
GPS: 0.000000, 0.000000
```

### Why It's Okay
- Your model primarily uses camera for navigation
- IMU and ultrasonic still work indoors
- The scaler handles zero values (they get normalized)

### For Better Results
When you train, most GPS values will be 0. The model learns to rely more on camera and IMU, which is fine.

---

## ✅ Quick Fix Checklist

1. [ ] Run `debug_autonomous.py` to check all components
2. [ ] Update Arduino code to send sensors at 20Hz instead of 1Hz
3. [ ] After training, run `scaler.py` and update values
4. [ ] Use `drive_improved.py` instead of `drive_sequenced.py`
5. [ ] Start with low BASE_SPEED (100-120) and STEER_GAIN (100-150)
6. [ ] Collect more training data (1000+ samples recommended)

---

## 📊 Recommended Parameter Tuning

### Start Conservative
```python
BASE_SPEED = 100   # Very slow to start
STEER_GAIN = 100   # Low sensitivity
```

### If Robot Goes Too Slow
```python
BASE_SPEED = 140   # Increase base speed
```

### If Robot Doesn't Turn Enough
```python
STEER_GAIN = 200   # Increase steering sensitivity
```

### If Robot Is Jerky/Unstable
```python
STEER_GAIN = 80    # Reduce steering sensitivity
# Also consider adding smoothing in the drive script
```

---

## 🔧 Testing Order

1. **Test Hardware**: Run `debug_autonomous.py`
2. **Test Manual Drive**: Run `smart_data_collector.py`, verify WASD works
3. **Test Camera Only**: Comment out motor commands, just check predictions
4. **Test at Low Speed**: Start with BASE_SPEED=80, don't go full throttle
5. **Tune Parameters**: Gradually increase speed and gain
