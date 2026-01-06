#!/usr/bin/env python3
"""
Path Playback Script - Autonomous Movement from Recorded Path
==============================================================
This script plays back a recorded path to make the robot move autonomously.
It uses BOTH the "to_target" and "return" paths that you recorded manually.

Usage:
    python3 playback_path.py                    # Play once: go to target, then return
    python3 playback_path.py --loop             # Continuous loop (target -> base -> target...)
    python3 playback_path.py --target-only      # Only go to target (no return)
    python3 playback_path.py --return-only      # Only return to base
    python3 playback_path.py --file mypath.json # Use specific path file

Controls during playback:
    SPACE - Pause/Resume
    Q - Quit immediately
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

# --- CONFIGURATION ---
BAUD_RATE = 115200
DEFAULT_PATH_FILE = "recorded_path.json"

# Speed multiplier (1.0 = same as recorded, 0.8 = slower, 1.2 = faster)
SPEED_MULTIPLIER = 1.0

# Target speed (lower for ~1 m/s - adjust based on your motors)
# 255 = max, 150 = ~60% power, should be around 1 m/s
SPEED_NORMAL = 150

# Safety: Stop if obstacle detected (distance in cm)
OBSTACLE_STOP_DISTANCE = 15

# Obstacle avoidance: if blocked for this many seconds, back up and turn
OBSTACLE_TIMEOUT = 10.0
BACKUP_DURATION = 1.0    # Seconds to back up
TURN_DURATION = 1.5      # Seconds to turn right

# --- ARDUINO CONNECTION ---
ser = None
sensor_distance = 999

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
            # Apply speed multiplier
            left = int(left * SPEED_MULTIPLIER)
            right = int(right * SPEED_MULTIPLIER)
            cmd = f"<{left},{right}>"
            ser.write(cmd.encode('utf-8'))
            ser.flush()
        except:
            pass

def read_sensors():
    """Read sensor data from Arduino"""
    global sensor_distance
    if not ser:
        return
    try:
        if ser.in_waiting > 0:
            lines = ser.read_all().decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                if line.startswith("START") and "END" in line:
                    parts = line.split(',')
                    if len(parts) >= 2:
                        sensor_distance = int(parts[1])
    except:
        pass

def get_key_nonblocking():
    """Non-blocking key read"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def avoid_obstacle():
    """Back up and turn right to avoid obstacle"""
    print("🔄 Avoiding obstacle: backing up...")
    
    # Back up
    start = time.time()
    while time.time() - start < BACKUP_DURATION:
        send_cmd(-SPEED_NORMAL, -SPEED_NORMAL)
        time.sleep(0.1)
    
    # Turn right (swing turn: left forward, right stopped)
    print("🔄 Turning right to avoid...")
    start = time.time()
    while time.time() - start < TURN_DURATION:
        send_cmd(SPEED_NORMAL, 0)
        time.sleep(0.1)
    
    send_cmd(0, 0)
    print("✅ Avoidance complete, continuing path...")
    time.sleep(0.3)

def load_path(filepath):
    """Load recorded path from JSON file"""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"ERROR: Path file '{filepath}' not found!")
        print("Run 'python3 record_path.py' first to record a path.")
        return None
    except json.JSONDecodeError:
        print(f"ERROR: Invalid JSON in '{filepath}'")
        return None

def reverse_path(commands):
    """Reverse a path for return journey"""
    reversed_cmds = []
    for cmd in reversed(commands):
        new_cmd = cmd.copy()
        # Swap left/right for turns, negate for forward/backward
        if cmd["action"] == "FORWARD":
            new_cmd["action"] = "BACKWARD"
            new_cmd["left"] = -cmd["left"]
            new_cmd["right"] = -cmd["right"]
        elif cmd["action"] == "BACKWARD":
            new_cmd["action"] = "FORWARD"
            new_cmd["left"] = -cmd["left"]
            new_cmd["right"] = -cmd["right"]
        elif cmd["action"] == "LEFT":
            new_cmd["action"] = "RIGHT"
            new_cmd["left"] = -cmd["left"]
            new_cmd["right"] = -cmd["right"]
        elif cmd["action"] == "RIGHT":
            new_cmd["action"] = "LEFT"
            new_cmd["left"] = -cmd["left"]
            new_cmd["right"] = -cmd["right"]
        reversed_cmds.append(new_cmd)
    return reversed_cmds

def play_path(commands, direction_name="TO TARGET"):
    """Execute a sequence of commands"""
    global sensor_distance
    
    print(f"\n🚀 Playing path: {direction_name}")
    print(f"   Commands: {len(commands)}")
    print("   Press SPACE to pause, Q to quit\n")
    
    paused = False
    
    for i, cmd in enumerate(commands):
        # Check for user input
        key = get_key_nonblocking()
        if key == 'q':
            print("\n⏹️  Stopped by user")
            send_cmd(0, 0)
            return False
        elif key == ' ':
            paused = not paused
            if paused:
                send_cmd(0, 0)
                print("⏸️  PAUSED - Press SPACE to resume")
            else:
                print("▶️  Resumed")
        
        # Wait while paused
        while paused:
            key = get_key_nonblocking()
            if key == ' ':
                paused = False
                print("▶️  Resumed")
            elif key == 'q':
                print("\n⏹️  Stopped by user")
                send_cmd(0, 0)
                return False
            time.sleep(0.1)
        
        # Read sensors for safety
        read_sensors()
        
        # Safety stop for obstacles
        if sensor_distance < OBSTACLE_STOP_DISTANCE and cmd["left"] > 0 and cmd["right"] > 0:
            send_cmd(0, 0)
            print(f"⚠️  OBSTACLE at {sensor_distance}cm! Waiting...")
            obstacle_start = time.time()
            while sensor_distance < OBSTACLE_STOP_DISTANCE + 5:
                read_sensors()
                time.sleep(0.1)
                key = get_key_nonblocking()
                if key == 'q':
                    return False
                # Check if blocked for too long
                if time.time() - obstacle_start >= OBSTACLE_TIMEOUT:
                    print(f"⏰ Blocked for {OBSTACLE_TIMEOUT}s - avoiding obstacle!")
                    avoid_obstacle()
                    break
            if sensor_distance >= OBSTACLE_STOP_DISTANCE + 5:
                print("✅ Path clear, continuing...")
        
        # Execute command
        action = cmd["action"]
        left = cmd["left"]
        right = cmd["right"]
        duration = cmd["duration"]
        
        print(f"  [{i+1}/{len(commands)}] {action:8} L:{left:4} R:{right:4} for {duration:.2f}s")
        
        send_cmd(left, right)
        
        # Wait for duration (in small steps to allow interruption)
        elapsed = 0
        step = 0.05
        while elapsed < duration:
            time.sleep(step)
            elapsed += step
            
            # Check for user input during wait
            key = get_key_nonblocking()
            if key == 'q':
                send_cmd(0, 0)
                return False
            elif key == ' ':
                send_cmd(0, 0)
                paused = True
                print("⏸️  PAUSED")
                while paused:
                    key = get_key_nonblocking()
                    if key == ' ':
                        paused = False
                        send_cmd(left, right)  # Resume movement
                        print("▶️  Resumed")
                    elif key == 'q':
                        return False
                    time.sleep(0.1)
            
            # Safety check during movement
            read_sensors()
            if sensor_distance < OBSTACLE_STOP_DISTANCE and left > 0 and right > 0:
                send_cmd(0, 0)
                print(f"⚠️  OBSTACLE during movement!")
                obstacle_start = time.time()
                while sensor_distance < OBSTACLE_STOP_DISTANCE + 5:
                    read_sensors()
                    time.sleep(0.1)
                    key = get_key_nonblocking()
                    if key == 'q':
                        return False
                    if time.time() - obstacle_start >= OBSTACLE_TIMEOUT:
                        print(f"⏰ Blocked for {OBSTACLE_TIMEOUT}s - avoiding obstacle!")
                        avoid_obstacle()
                        break
                # Don't continue this command, move to next one
                break
    
    send_cmd(0, 0)
    print(f"\n✅ Path complete: {direction_name}")
    return True

def main():
    parser = argparse.ArgumentParser(description='Playback recorded robot path')
    parser.add_argument('--file', '-f', default=DEFAULT_PATH_FILE, help='Path file to use')
    parser.add_argument('--loop', '-l', action='store_true', help='Continuous loop mode')
    parser.add_argument('--target-only', '-t', action='store_true', help='Only go to target (no return)')
    parser.add_argument('--return-only', '-r', action='store_true', help='Only return to base')
    parser.add_argument('--speed', '-s', type=float, default=1.0, help='Speed multiplier (0.5-2.0)')
    args = parser.parse_args()
    
    global SPEED_MULTIPLIER
    SPEED_MULTIPLIER = max(0.5, min(2.0, args.speed))
    
    # Load path
    path_data = load_path(args.file)
    if not path_data:
        sys.exit(1)
    
    # Support both old format (single "commands") and new format ("to_target" + "return")
    if "to_target" in path_data:
        # New format with both paths
        to_target_commands = path_data["to_target"]["commands"]
        return_commands = path_data["return"]["commands"]
        to_target_duration = path_data["to_target"]["duration"]
        return_duration = path_data["return"]["duration"]
    else:
        # Old format - single path (fallback)
        to_target_commands = path_data.get("commands", [])
        return_commands = []
        to_target_duration = path_data.get("total_duration", 0)
        return_duration = 0
    
    print("\n" + "="*50)
    print("   PATH PLAYBACK MODE")
    print("="*50)
    print(f"Path file: {args.file}")
    print(f"Recorded: {path_data.get('recorded_at', 'Unknown')}")
    print(f"TO TARGET: {len(to_target_commands)} commands ({to_target_duration}s)")
    print(f"RETURN:    {len(return_commands)} commands ({return_duration}s)")
    print(f"Speed: {SPEED_MULTIPLIER:.1f}x")
    mode_str = 'Loop' if args.loop else 'Target only' if args.target_only else 'Return only' if args.return_only else 'Full round trip'
    print(f"Mode: {mode_str}")
    print("="*50)
    
    # Check if return path exists
    if not return_commands and not args.target_only:
        print("\n⚠️  No RETURN path recorded!")
        print("   Use --target-only flag, or re-record with K key for return path.")
        if not args.target_only and not args.return_only:
            print("   Continuing with TO TARGET only...\n")
            args.target_only = True
    
    # Connect to Arduino
    if not connect_arduino():
        sys.exit(1)
    
    # Setup terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    try:
        if args.return_only:
            # Just play return path
            if return_commands:
                play_path(return_commands, "RETURNING TO BASE")
            else:
                print("ERROR: No return path recorded!")
        
        elif args.target_only:
            # Just go to target
            play_path(to_target_commands, "TO TARGET")
        
        elif args.loop:
            # Continuous loop using BOTH recorded paths
            loop_count = 0
            while True:
                loop_count += 1
                print(f"\n{'='*50}")
                print(f"   LOOP {loop_count}")
                print('='*50)
                
                # Go to target (using recorded to_target path)
                if not play_path(to_target_commands, "TO TARGET"):
                    break
                
                time.sleep(1)  # Brief pause at target
                
                # Return to base (using recorded return path)
                if return_commands:
                    if not play_path(return_commands, "RETURNING TO BASE"):
                        break
                else:
                    print("⚠️  No return path - stopping loop")
                    break
                
                time.sleep(1)  # Brief pause at base
        
        else:
            # Full round trip: go to target, then return
            if not play_path(to_target_commands, "TO TARGET"):
                pass
            elif return_commands:
                time.sleep(1)
                play_path(return_commands, "RETURNING TO BASE")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    
    finally:
        send_cmd(0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n🏁 Playback ended. Motors stopped.")

if __name__ == "__main__":
    main()
