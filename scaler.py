import joblib
import numpy as np
import sys

# --- CONFIGURATION ---
# The name of the scaler file created by your training script
SCALER_FILENAME = "vga_scaler.pkl" 

print(f"--- Scaler Extraction Tool ---")
print(f"Looking for: {SCALER_FILENAME}")

try:
    # Load the scaler object
    scaler = joblib.load(SCALER_FILENAME)
    print("✅ Scaler loaded successfully!")
    
    # Extract the math values
    # mean_ = The average value for each sensor column
    # scale_ = The standard deviation value for each sensor column
    mean_values = list(scaler.mean_)
    scale_values = list(scaler.scale_)
    
    print("\n" + "="*60)
    print("COPY AND PASTE THE FOLLOWING LINES INTO YOUR JETSON SCRIPT")
    print("Replace the old SCALER_MEAN and SCALER_SCALE lines.")
    print("="*60 + "\n")
    
    print(f"SCALER_MEAN = {mean_values}")
    print(f"SCALER_SCALE = {scale_values}")
    
    print("\n" + "="*60 + "\n")

except FileNotFoundError:
    print(f"❌ ERROR: Could not find '{SCALER_FILENAME}'.")
    print("Make sure you run this script in the SAME FOLDER where you ran train_sequenced_torch.py")
except Exception as e:
    print(f"❌ ERROR: Failed to load scaler. {e}")
