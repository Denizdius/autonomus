#!/usr/bin/env python3
"""
Autonomous Robot Diagnostic Tool
================================
This script helps debug issues with autonomous driving by testing
each component individually.

Run this on your Jetson Nano to identify what's working and what's not.

Usage:
    python3 debug_autonomous.py
"""

import sys
import time
import os

def print_header(title):
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)

def print_result(test_name, success, message=""):
    status = "✅ PASS" if success else "❌ FAIL"
    print(f"  {status} - {test_name}")
    if message:
        print(f"         {message}")

def test_imports():
    """Test if all required Python packages are installed"""
    print_header("TEST 1: Python Package Imports")
    
    packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy',
        'torch': 'torch',
        'serial': 'pyserial',
    }
    
    all_passed = True
    for pkg, install_name in packages.items():
        try:
            __import__(pkg)
            print_result(f"import {pkg}", True)
        except ImportError:
            print_result(f"import {pkg}", False, f"Install with: pip3 install {install_name}")
            all_passed = False
    
    return all_passed

def test_cuda():
    """Test if CUDA/GPU is available"""
    print_header("TEST 2: CUDA/GPU Availability")
    
    try:
        import torch
        cuda_available = torch.cuda.is_available()
        print_result("CUDA Available", cuda_available)
        
        if cuda_available:
            device_name = torch.cuda.get_device_name(0)
            print_result(f"GPU Device", True, device_name)
        else:
            print("         Running on CPU (slower but will work)")
        
        return True
    except Exception as e:
        print_result("CUDA Check", False, str(e))
        return False

def test_serial():
    """Test Arduino serial connection"""
    print_header("TEST 3: Arduino Serial Connection")
    
    import serial
    
    # Find available ports
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
    found_ports = [p for p in ports if os.path.exists(p)]
    
    if not found_ports:
        print_result("Find Serial Port", False, "No Arduino found! Check USB connection.")
        return False, None
    
    print_result("Find Serial Port", True, f"Found: {found_ports}")
    
    # Try to connect
    port = found_ports[0]
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Arduino reset time
        ser.reset_input_buffer()
        print_result(f"Connect to {port}", True)
        
        # Read sensor data
        print("\n  Reading sensor data for 3 seconds...")
        start_time = time.time()
        data_received = False
        
        while time.time() - start_time < 3:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("START") and line.endswith("END"):
                    print(f"  📡 Received: {line[:60]}...")
                    data_received = True
                    
                    # Parse and show sensor values
                    parts = line.split(',')
                    if len(parts) >= 10:
                        dist = parts[1]
                        lat = parts[2]
                        lon = parts[3]
                        print(f"      Distance: {dist} cm")
                        print(f"      GPS: {lat}, {lon}")
                        print(f"      IMU: {parts[4]}, {parts[5]}, {parts[6]} (acc)")
                    break
        
        print_result("Receive Sensor Data", data_received)
        
        # Test sending motor command
        print("\n  Testing motor command (brief pulse)...")
        ser.write(b"<50,50>")
        time.sleep(0.1)
        ser.write(b"<0,0>")
        print_result("Send Motor Command", True, "Sent <50,50> then <0,0>")
        
        ser.close()
        return True, port
        
    except Exception as e:
        print_result(f"Connect to {port}", False, str(e))
        return False, None

def test_camera():
    """Test CSI camera with GStreamer"""
    print_header("TEST 4: CSI Camera")
    
    import cv2
    
    # Test different pipeline configurations
    pipelines = [
        # Pipeline 1: Standard 640x480
        (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
            "nvvidconv ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink drop=true",
            "640x480 (Training Resolution)"
        ),
        # Pipeline 2: Lower resolution for faster processing
        (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
            "nvvidconv ! "
            "video/x-raw, width=160, height=120, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink drop=true",
            "160x120 (drive_sequenced.py Resolution)"
        ),
    ]
    
    working_pipeline = None
    
    for pipeline, name in pipelines:
        print(f"\n  Testing: {name}")
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print_result(f"Open Camera ({name})", False)
                continue
            
            # Try to read a frame
            ret, frame = cap.read()
            cap.release()
            
            if ret and frame is not None:
                h, w = frame.shape[:2]
                print_result(f"Open Camera ({name})", True, f"Frame size: {w}x{h}")
                working_pipeline = pipeline
                break
            else:
                print_result(f"Read Frame ({name})", False)
                
        except Exception as e:
            print_result(f"Camera ({name})", False, str(e))
    
    if not working_pipeline:
        print("\n  ⚠️  Camera not working! Try running:")
        print("      sudo systemctl restart nvargus-daemon")
        print("      Then run this script again.")
        return False, None
    
    return True, working_pipeline

def test_model():
    """Test if the trained model can be loaded"""
    print_header("TEST 5: Model Loading")
    
    import torch
    import torch.nn as nn
    
    # Check for model files
    model_files = ['vga_pilot.pth', 'sequenced_pilot.pth']
    found_model = None
    
    for mf in model_files:
        if os.path.exists(mf):
            found_model = mf
            break
    
    if not found_model:
        print_result("Find Model File", False, f"No model found! Expected: {model_files}")
        return False, None
    
    print_result("Find Model File", True, f"Found: {found_model}")
    
    # Check file size
    size_mb = os.path.getsize(found_model) / (1024 * 1024)
    print_result("Model File Size", True, f"{size_mb:.2f} MB")
    
    # Define model architecture (must match training)
    class SequencedPilot(nn.Module):
        def __init__(self, input_height=480, input_width=640):
            super(SequencedPilot, self).__init__()
            
            # Calculate output size after CNN layers
            # This is for 640x480 input
            if input_height == 480 and input_width == 640:
                # With stride=2 on all layers: 640x480 -> 14976
                self.cnn = nn.Sequential(
                    nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
                    nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
                    nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
                    nn.Conv2d(48, 64, 3, stride=2), nn.ReLU(),
                    nn.Conv2d(64, 64, 3, stride=2), nn.ReLU(),
                    nn.Flatten()
                )
                cnn_output = 14976
            else:
                # For 160x120 input
                self.cnn = nn.Sequential(
                    nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
                    nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
                    nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
                    nn.Conv2d(48, 64, 3), nn.ReLU(),
                    nn.Conv2d(64, 64, 3), nn.ReLU(),
                    nn.Flatten()
                )
                cnn_output = 6656
            
            self.mlp = nn.Sequential(
                nn.Linear(9, 64), nn.ReLU(),
                nn.Linear(64, 32), nn.ReLU()
            )
            
            self.combined = nn.Sequential(
                nn.Linear(cnn_output + 32, 100), nn.ReLU(),
                nn.Dropout(0.3),
                nn.Linear(100, 50), nn.ReLU(),
                nn.Linear(50, 1)
            )

        def forward(self, img, sens):
            x1 = self.cnn(img)
            x2 = self.mlp(sens)
            x = torch.cat((x1, x2), dim=1)
            return self.combined(x)
    
    # Try loading with VGA (640x480) architecture first
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    try:
        model = SequencedPilot(480, 640).to(device)
        model.load_state_dict(torch.load(found_model, map_location=device))
        model.eval()
        print_result("Load Model (640x480)", True)
        
        # Test inference
        dummy_img = torch.randn(1, 3, 480, 640).to(device)
        dummy_sens = torch.randn(1, 9).to(device)
        
        with torch.no_grad():
            output = model(dummy_img, dummy_sens)
        
        print_result("Model Inference", True, f"Output shape: {output.shape}, value: {output.item():.4f}")
        return True, (found_model, 480, 640)
        
    except Exception as e:
        print(f"         Error with 640x480: {e}")
        
        # Try 160x120
        try:
            model = SequencedPilot(120, 160).to(device)
            model.load_state_dict(torch.load(found_model, map_location=device))
            model.eval()
            print_result("Load Model (160x120)", True)
            return True, (found_model, 120, 160)
        except Exception as e2:
            print_result("Load Model", False, str(e2))
            return False, None

def test_scaler():
    """Test if scaler file exists and can be loaded"""
    print_header("TEST 6: Scaler File")
    
    scaler_files = ['vga_scaler.pkl', 'sequenced_scaler.pkl']
    found_scaler = None
    
    for sf in scaler_files:
        if os.path.exists(sf):
            found_scaler = sf
            break
    
    if not found_scaler:
        print_result("Find Scaler File", False, 
                    "No scaler found! You need to copy scaler file from training PC")
        print("\n  Alternative: Use hardcoded values in drive script")
        return False
    
    print_result("Find Scaler File", True, f"Found: {found_scaler}")
    
    try:
        import joblib
        scaler = joblib.load(found_scaler)
        
        mean_vals = list(scaler.mean_)
        scale_vals = list(scaler.scale_)
        
        print(f"  📊 Scaler Mean: {[f'{x:.2f}' for x in mean_vals[:3]]}...")
        print(f"  📊 Scaler Scale: {[f'{x:.2f}' for x in scale_vals[:3]]}...")
        
        print_result("Load Scaler", True)
        return True
        
    except Exception as e:
        print_result("Load Scaler", False, str(e))
        return False

def run_full_diagnostic():
    """Run all diagnostic tests"""
    print("\n" + "🔧" * 30)
    print("   AUTONOMOUS ROBOT DIAGNOSTIC TOOL")
    print("🔧" * 30)
    
    results = {}
    
    # Run all tests
    results['imports'] = test_imports()
    results['cuda'] = test_cuda()
    results['serial'], serial_port = test_serial()
    results['camera'], camera_pipeline = test_camera()
    results['model'], model_info = test_model()
    results['scaler'] = test_scaler()
    
    # Summary
    print_header("DIAGNOSTIC SUMMARY")
    
    passed = sum(results.values())
    total = len(results)
    
    print(f"\n  Tests Passed: {passed}/{total}")
    print()
    
    for test, result in results.items():
        status = "✅" if result else "❌"
        print(f"  {status} {test.upper()}")
    
    # Recommendations
    print_header("RECOMMENDATIONS")
    
    if not results['serial']:
        print("  🔴 CRITICAL: Arduino not connected")
        print("     - Check USB cable")
        print("     - Verify Arduino is powered")
        print("     - Run: ls /dev/ttyUSB* /dev/ttyACM*")
    
    if not results['camera']:
        print("  🔴 CRITICAL: Camera not working")
        print("     - Check CSI cable connection")
        print("     - Run: sudo systemctl restart nvargus-daemon")
    
    if not results['model']:
        print("  🔴 CRITICAL: Model not found/loadable")
        print("     - Copy trained model from training PC")
        print("     - Make sure architecture matches")
    
    if results['model'] and model_info:
        model_file, h, w = model_info
        print(f"\n  ℹ️  Model expects {w}x{h} images")
        print(f"     Make sure your drive script uses the same resolution!")
    
    if all(results.values()):
        print("\n  ✅ All tests passed! System should be ready for autonomous driving.")
        print("     If it still doesn't work, the issue might be:")
        print("     - Training data quality")
        print("     - Scaler values mismatch")
        print("     - STEER_GAIN / BASE_SPEED tuning")
    
    return results

if __name__ == "__main__":
    try:
        run_full_diagnostic()
    except KeyboardInterrupt:
        print("\n\nDiagnostic cancelled by user.")
    except Exception as e:
        print(f"\n\nDiagnostic error: {e}")
        import traceback
        traceback.print_exc()
