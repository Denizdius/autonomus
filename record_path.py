#!/usr/bin/env python3
"""
Path Recording Script - Teach the Robot a Route
================================================
This script records your WASD key presses with precise timing.
Drive the robot manually, and it saves the movement sequence.

Usage:
    python3 record_path.py

Controls:
    W - Forward
    S - Backward  
    A - Turn Left
    D - Turn Right
    SPACE - Stop/Pause
    Q - Quit and Save
    
The recorded path is saved to 'recorded_path.json'
"""

import serial
import time
import os
import sys
import tty
import termios
import select
import json
from datetime import datetime

# --- CONFIGURATION ---
BAUD_RATE = 115200
PATH_FILE = "recorded_path.json"

# Motor Speeds
SPEED_FWD = 200       # Forward/backward speed
SPEED_TURN = 180      # Turning speed
SPEED_STOP = 0

# --- ARDUINO CONNECTION ---
ser = None

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

def send_cmd(left, right):
    """Send motor command to Arduino"""
    if ser:
        try:
            cmd = f"<{int(left)},{int(right)}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def get_key():
    """Non-blocking key read"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def main():
    if not connect_arduino():
        sys.exit(1)
    
    # Setup terminal for raw input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print("\n" + "="*50)
    print("   PATH RECORDING MODE")
    print("="*50)
    print("Controls:")
    print("  W = Forward    S = Backward")
    print("  A = Left       D = Right")
    print("  SPACE = Stop   Q = Quit & Save")
    print("="*50)
    print("\nRecording started! Drive the robot...\n")
    
    # Recording storage
    recorded_commands = []
    current_command = None
    command_start_time = None
    
    # Command mapping
    key_to_motors = {
        'w': (SPEED_FWD, SPEED_FWD, "FORWARD"),
        's': (-SPEED_FWD, -SPEED_FWD, "BACKWARD"),
        'a': (-SPEED_TURN, SPEED_TURN, "LEFT"),
        'd': (SPEED_TURN, -SPEED_TURN, "RIGHT"),
        ' ': (SPEED_STOP, SPEED_STOP, "STOP"),
    }
    
    last_left, last_right = 0, 0
    recording_start = time.time()
    
    try:
        while True:
            key = get_key()
            
            if key == 'q':
                # Save final command if any
                if current_command and command_start_time:
                    duration = time.time() - command_start_time
                    if duration > 0.05:  # Ignore very short commands
                        recorded_commands.append({
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        })
                break
            
            if key in key_to_motors:
                new_command = key_to_motors[key]
                
                # Save previous command (even if same key - allows W, W, W)
                if current_command and command_start_time:
                    duration = time.time() - command_start_time
                    if duration > 0.05:  # Ignore very short commands
                        recorded_commands.append({
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        })
                
                # Start new command
                current_command = new_command
                command_start_time = time.time()
                
                left, right, action = new_command
                send_cmd(left, right)
                last_left, last_right = left, right
                
                elapsed = time.time() - recording_start
                print(f"[{elapsed:6.1f}s] {action:8} | L:{left:4} R:{right:4} | Commands: {len(recorded_commands)}")
            
            time.sleep(0.01)  # Small delay to prevent CPU overload
    
    except KeyboardInterrupt:
        print("\n\nRecording interrupted!")
    
    finally:
        # Stop motors
        send_cmd(0, 0)
        
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Save recorded path
        if recorded_commands:
            total_time = sum(cmd["duration"] for cmd in recorded_commands)
            
            path_data = {
                "recorded_at": datetime.now().isoformat(),
                "total_duration": round(total_time, 2),
                "command_count": len(recorded_commands),
                "commands": recorded_commands
            }
            
            with open(PATH_FILE, 'w') as f:
                json.dump(path_data, f, indent=2)
            
            print("\n" + "="*50)
            print(f"✅ PATH SAVED: {PATH_FILE}")
            print(f"   Total commands: {len(recorded_commands)}")
            print(f"   Total duration: {total_time:.1f} seconds")
            print("="*50)
            
            # Show summary
            print("\nRecorded sequence:")
            for i, cmd in enumerate(recorded_commands[:10]):  # Show first 10
                print(f"  {i+1}. {cmd['action']:8} for {cmd['duration']:.2f}s")
            if len(recorded_commands) > 10:
                print(f"  ... and {len(recorded_commands)-10} more commands")
        else:
            print("\n⚠️  No commands recorded!")

if __name__ == "__main__":
    main()
