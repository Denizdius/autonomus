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
    K - Switch to RETURN mode (record the way back)
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
SPEED_FWD = 255       # Maximum speed
SPEED_TURN = 255      # Maximum speed
SPEED_STOP = 0

# 180° Turn Configuration (adjust TURN_180_DURATION to match your robot)
# This is how long it takes your robot to turn 180° at full speed
TURN_180_DURATION = 1.0  # seconds - ADJUST THIS for your robot!

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

def do_180_turn():
    """Perform a 180° turn (swing turn to the right)"""
    print("\n🔄 Performing 180° turn...")
    # Swing turn right: left motor forward, right motor stopped
    send_cmd(SPEED_TURN, 0)
    time.sleep(TURN_180_DURATION)
    send_cmd(0, 0)
    print("✅ Turn complete!\n")
    time.sleep(0.3)  # Brief pause to stabilize

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
    print("  SPACE = Stop")
    print("  K = Switch to RETURN mode (after reaching target)")
    print("  Q = Quit & Save")
    print("="*50)
    print("\n📍 Recording TO TARGET path...\n")
    
    # Recording storage - TWO paths now
    to_target_commands = []
    return_commands = []
    current_command = None
    command_start_time = None
    recording_mode = "TO_TARGET"  # or "RETURN"
    
    # Command mapping (Swing turns: one motor stopped, other moves)
    key_to_motors = {
        'w': (SPEED_FWD, SPEED_FWD, "FORWARD"),
        's': (-SPEED_FWD, -SPEED_FWD, "BACKWARD"),
        'a': (0, SPEED_TURN, "LEFT"),       # Stop left, right forward
        'd': (SPEED_TURN, 0, "RIGHT"),      # Left forward, stop right
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
                        cmd_data = {
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        }
                        if recording_mode == "TO_TARGET":
                            to_target_commands.append(cmd_data)
                        else:
                            return_commands.append(cmd_data)
                break
            
            if key == 'k':
                # Switch to RETURN mode with 180° turn
                if recording_mode == "TO_TARGET":
                    # Save current command before switching
                    if current_command and command_start_time:
                        duration = time.time() - command_start_time
                        if duration > 0.05:
                            to_target_commands.append({
                                "action": current_command[2],
                                "left": current_command[0],
                                "right": current_command[1],
                                "duration": round(duration, 3)
                            })
                    
                    send_cmd(0, 0)  # Stop motors
                    current_command = None
                    command_start_time = None
                    
                    # Perform 180° turn
                    do_180_turn()
                    
                    recording_mode = "RETURN"
                    
                    print("="*50)
                    print("🔄 SWITCHED TO RETURN MODE!")
                    print("   Robot has turned 180°.")
                    print("   Now drive FORWARD to return to base.")
                    print("   Press Q when you arrive at base.")
                    print("="*50 + "\n")
                    print("📍 Recording RETURN path (drive forward!)...\n")
                continue
            
            if key in key_to_motors:
                new_command = key_to_motors[key]
                
                # Save previous command (even if same key - allows W, W, W)
                if current_command and command_start_time:
                    duration = time.time() - command_start_time
                    if duration > 0.05:  # Ignore very short commands
                        cmd_data = {
                            "action": current_command[2],
                            "left": current_command[0],
                            "right": current_command[1],
                            "duration": round(duration, 3)
                        }
                        if recording_mode == "TO_TARGET":
                            to_target_commands.append(cmd_data)
                        else:
                            return_commands.append(cmd_data)
                
                # Start new command
                current_command = new_command
                command_start_time = time.time()
                
                left, right, action = new_command
                send_cmd(left, right)
                last_left, last_right = left, right
                
                elapsed = time.time() - recording_start
                mode_indicator = "→ TARGET" if recording_mode == "TO_TARGET" else "← RETURN"
                cmd_count = len(to_target_commands) if recording_mode == "TO_TARGET" else len(return_commands)
                print(f"[{elapsed:6.1f}s] {mode_indicator} | {action:8} | L:{left:4} R:{right:4} | Cmds: {cmd_count}")
            
            time.sleep(0.01)  # Small delay to prevent CPU overload
    
    except KeyboardInterrupt:
        print("\n\nRecording interrupted!")
    
    finally:
        # Stop motors
        send_cmd(0, 0)
        
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Save recorded paths
        if to_target_commands or return_commands:
            to_target_time = sum(cmd["duration"] for cmd in to_target_commands)
            return_time = sum(cmd["duration"] for cmd in return_commands)
            total_time = to_target_time + return_time
            
            path_data = {
                "recorded_at": datetime.now().isoformat(),
                "total_duration": round(total_time, 2),
                "to_target": {
                    "command_count": len(to_target_commands),
                    "duration": round(to_target_time, 2),
                    "commands": to_target_commands
                },
                "return": {
                    "command_count": len(return_commands),
                    "duration": round(return_time, 2),
                    "commands": return_commands
                }
            }
            
            with open(PATH_FILE, 'w') as f:
                json.dump(path_data, f, indent=2)
            
            print("\n" + "="*50)
            print(f"✅ PATH SAVED: {PATH_FILE}")
            print(f"   TO TARGET: {len(to_target_commands)} commands ({to_target_time:.1f}s)")
            print(f"   RETURN:    {len(return_commands)} commands ({return_time:.1f}s)")
            print("="*50)
            
            # Show summary
            if to_target_commands:
                print("\nTO TARGET sequence:")
                for i, cmd in enumerate(to_target_commands[:5]):
                    print(f"  {i+1}. {cmd['action']:8} for {cmd['duration']:.2f}s")
                if len(to_target_commands) > 5:
                    print(f"  ... and {len(to_target_commands)-5} more")
            
            if return_commands:
                print("\nRETURN sequence:")
                for i, cmd in enumerate(return_commands[:5]):
                    print(f"  {i+1}. {cmd['action']:8} for {cmd['duration']:.2f}s")
                if len(return_commands) > 5:
                    print(f"  ... and {len(return_commands)-5} more")
            
            if not return_commands:
                print("\n⚠️  No RETURN path recorded!")
                print("   Next time, press K at the target to record the return path.")
        else:
            print("\n⚠️  No commands recorded!")

if __name__ == "__main__":
    main()
