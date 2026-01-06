#!/usr/bin/env python3
"""
ArUco Marker Generator
======================
Generates printable ArUco markers for robot navigation.

Usage:
    python3 print_markers.py              # Generate markers 1-10
    python3 print_markers.py --ids 1 5 10 # Generate specific markers
    python3 print_markers.py --all        # Generate all markers 0-49

The markers will be saved as PNG files in 'markers/' folder.
Print them on A4 paper and tape to walls at key positions.
"""

import cv2
import numpy as np
import os
import argparse

# Marker settings
MARKER_SIZE = 200  # pixels (good for A4 printing)
BORDER_SIZE = 50   # white border around marker
ARUCO_DICT = cv2.aruco.DICT_4X4_50  # 4x4 markers, IDs 0-49

def generate_marker(marker_id, output_folder="markers"):
    """Generate a single ArUco marker"""
    
    # Create output folder
    os.makedirs(output_folder, exist_ok=True)
    
    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    
    # Generate marker
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE)
    
    # Add white border
    bordered = cv2.copyMakeBorder(
        marker_image, 
        BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE,
        cv2.BORDER_CONSTANT, 
        value=255
    )
    
    # Add ID text below marker
    final_height = bordered.shape[0] + 60
    final_image = np.ones((final_height, bordered.shape[1]), dtype=np.uint8) * 255
    final_image[:bordered.shape[0], :] = bordered
    
    # Add text
    text = f"Marker #{marker_id}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = (final_image.shape[1] - text_size[0]) // 2
    text_y = bordered.shape[0] + 40
    cv2.putText(final_image, text, (text_x, text_y), font, 1, 0, 2)
    
    # Save marker
    filename = os.path.join(output_folder, f"marker_{marker_id:02d}.png")
    cv2.imwrite(filename, final_image)
    
    return filename

def generate_marker_sheet(marker_ids, output_file="markers/marker_sheet.png"):
    """Generate a single sheet with multiple markers"""
    
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    
    # Settings for sheet
    marker_size = 150
    padding = 30
    cols = 5
    rows = (len(marker_ids) + cols - 1) // cols
    
    # Calculate sheet size
    cell_width = marker_size + padding * 2
    cell_height = marker_size + padding * 2 + 40  # Extra for text
    sheet_width = cols * cell_width
    sheet_height = rows * cell_height
    
    # Create white sheet
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
    
    for idx, marker_id in enumerate(marker_ids):
        row = idx // cols
        col = idx % cols
        
        # Generate marker
        marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        
        # Position on sheet
        x = col * cell_width + padding
        y = row * cell_height + padding
        
        # Place marker
        sheet[y:y+marker_size, x:x+marker_size] = marker
        
        # Add ID text
        text = f"#{marker_id}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_x = x + marker_size // 2 - 20
        text_y = y + marker_size + 25
        cv2.putText(sheet, text, (text_x, text_y), font, 0.7, 0, 2)
    
    cv2.imwrite(output_file, sheet)
    return output_file

def main():
    parser = argparse.ArgumentParser(description='Generate ArUco markers for navigation')
    parser.add_argument('--ids', nargs='+', type=int, help='Specific marker IDs to generate')
    parser.add_argument('--all', action='store_true', help='Generate all markers (0-49)')
    parser.add_argument('--sheet', action='store_true', help='Generate single sheet with all markers')
    args = parser.parse_args()
    
    if args.all:
        marker_ids = list(range(50))
    elif args.ids:
        marker_ids = args.ids
    else:
        # Default: generate markers 1-10
        marker_ids = list(range(1, 11))
    
    print("="*50)
    print("   ArUco Marker Generator")
    print("="*50)
    print(f"Generating markers: {marker_ids}")
    print()
    
    # Generate individual markers
    for marker_id in marker_ids:
        filename = generate_marker(marker_id)
        print(f"✅ Generated: {filename}")
    
    # Generate sheet if requested or by default
    if args.sheet or not args.ids:
        sheet_file = generate_marker_sheet(marker_ids)
        print(f"\n✅ Generated sheet: {sheet_file}")
    
    print()
    print("="*50)
    print("📝 Instructions:")
    print("="*50)
    print("1. Print the markers on regular paper")
    print("2. Cut them out (keep the white border!)")
    print("3. Tape them to walls at key positions:")
    print("   - Marker #1: Base station (starting point)")
    print("   - Marker #2-9: Corners, turns, waypoints")
    print("   - Marker #10: Destination")
    print("4. Place markers at robot camera height")
    print("5. Make sure markers are flat (not bent)")
    print("="*50)

if __name__ == "__main__":
    main()
