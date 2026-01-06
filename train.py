#!/usr/bin/env python3
"""
Tesla-Style Training Script - Lazy Loading Version
===================================================
Loads images from disk on-demand to save RAM.
Uses GPU for training.
"""

import os
import pandas as pd
import numpy as np
import cv2
import joblib
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# --- CONFIGURATION ---
DATA_DIR = "dataset_vga"
LOG_FILE = os.path.join(DATA_DIR, "log.csv")
MODEL_NAME = "vga_pilot.pth"
SCALER_NAME = "vga_scaler.pkl"
BATCH_SIZE = 64       # Big batch for RTX 4080
EPOCHS = 50           # More epochs
LEARNING_RATE = 1e-4
NUM_WORKERS = 4       # Parallel data loading
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Data Augmentation
AUGMENT_DATA = True
BRIGHTNESS_RANGE = 0.3

print(f"🚀 Using device: {DEVICE}")
if torch.cuda.is_available():
    print(f"   GPU: {torch.cuda.get_device_name(0)}")
    print(f"   VRAM: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")

# --- LAZY LOADING DATASET ---
class LazyRobotDataset(Dataset):
    """Loads images from disk on-demand instead of all at once"""
    def __init__(self, image_paths, sensors, steering, data_dir, augment=False):
        self.image_paths = image_paths
        self.sensors = torch.tensor(sensors, dtype=torch.float32)
        self.steering = torch.tensor(steering, dtype=torch.float32)
        self.data_dir = data_dir
        self.augment = augment
        
    def __len__(self): 
        return len(self.image_paths)
    
    def __getitem__(self, idx):
        # Load image from disk
        img_path = os.path.join(self.data_dir, self.image_paths[idx])
        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV).astype(np.float32) / 255.0
        
        steer = self.steering[idx].item()
        
        # Data augmentation
        if self.augment:
            if np.random.rand() > 0.5:
                img = np.fliplr(img).copy()
                steer = -steer
            brightness = 1.0 + np.random.uniform(-BRIGHTNESS_RANGE, BRIGHTNESS_RANGE)
            img = np.clip(img * brightness, 0, 1)
        
        img = img.transpose((2, 0, 1))  # HWC -> CHW
        return (torch.tensor(img, dtype=torch.float32), 
                self.sensors[idx], 
                torch.tensor(steer, dtype=torch.float32))

# --- LOAD CSV DATA (not images yet!) ---
print(f"Loading {LOG_FILE}...")
data = pd.read_csv(LOG_FILE)
print(f"CSV has {len(data)} rows")

# Prepare data lists (no images loaded yet!)
image_paths = []
raw_sensors = []
raw_steering = []

print("Validating data...")
valid_count = 0
for index, row in data.iterrows():
    img_filename = str(row['image_path']).strip()
    
    if not img_filename.lower().endswith('.jpg'):
        continue
    
    img_path = os.path.join(DATA_DIR, img_filename)
    if not os.path.exists(img_path):
        continue
    
    image_paths.append(img_filename)
    
    sens = [
        row['dist_cm'], row['gps_lat'], row['gps_lon'],
        row['acc_x'], row['acc_y'], row['acc_z'],
        row['gyro_x'], row['gyro_y'], row['gyro_z']
    ]
    raw_sensors.append([0.0 if pd.isna(x) else float(x) for x in sens])
    raw_steering.append((float(row['cmd_left']) - float(row['cmd_right'])) / 510.0)
    valid_count += 1

print(f"✅ Found {valid_count} valid samples")

if valid_count == 0:
    print("ERROR: No valid data found!")
    exit(1)

# Convert to arrays
X_sens = np.array(raw_sensors)
y = np.array(raw_steering)
image_paths = np.array(image_paths)

# Scale sensors
scaler = StandardScaler()
X_sens = scaler.fit_transform(X_sens)
joblib.dump(scaler, SCALER_NAME)
print(f"Saved scaler to {SCALER_NAME}")

# Train/test split
indices = np.arange(len(image_paths))
train_idx, test_idx = train_test_split(indices, test_size=0.2, random_state=42)

train_paths = image_paths[train_idx]
test_paths = image_paths[test_idx]
train_sens = X_sens[train_idx]
test_sens = X_sens[test_idx]
train_steer = y[train_idx]
test_steer = y[test_idx]

print(f"Training: {len(train_paths)} samples")
print(f"Validation: {len(test_paths)} samples")

# Create datasets
train_dataset = LazyRobotDataset(train_paths, train_sens, train_steer, DATA_DIR, augment=AUGMENT_DATA)
val_dataset = LazyRobotDataset(test_paths, test_sens, test_steer, DATA_DIR, augment=False)

train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, 
                          num_workers=NUM_WORKERS, pin_memory=True)
val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False,
                        num_workers=NUM_WORKERS, pin_memory=True)

if AUGMENT_DATA:
    print(f"📈 Data augmentation ENABLED")

# --- MODEL ARCHITECTURE ---
class SequencedPilot(nn.Module):
    def __init__(self):
        super(SequencedPilot, self).__init__()
        # Input: 3 x 480 x 640
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),   # -> 238 x 318
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),  # -> 117 x 157
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),  # -> 57 x 77
            nn.Conv2d(48, 64, 3, stride=2), nn.ReLU(),  # -> 28 x 38
            nn.Conv2d(64, 64, 3, stride=2), nn.ReLU(),  # -> 13 x 18
            nn.Flatten()
        )
        # Flatten: 64 * 13 * 18 = 14976
        
        self.mlp = nn.Sequential(
            nn.Linear(9, 64), nn.ReLU(), 
            nn.Linear(64, 32), nn.ReLU()
        )
        
        # Fusion: 14976 + 32 = 15008
        self.combined = nn.Sequential(
            nn.Linear(15008, 100), nn.ReLU(),
            nn.Dropout(0.3),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 1)
        )

    def forward(self, img, sens):
        x = torch.cat((self.cnn(img), self.mlp(sens)), dim=1)
        return self.combined(x)

model = SequencedPilot().to(DEVICE)
optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
criterion = nn.MSELoss()

print(f"\n🏋️ Starting training for {EPOCHS} epochs...")
print("="*50)

# --- TRAINING LOOP ---
best_loss = float('inf')
for epoch in range(EPOCHS):
    # Train
    model.train()
    train_loss = 0.0
    for batch_idx, (img, sens, steer) in enumerate(train_loader):
        img, sens, steer = img.to(DEVICE), sens.to(DEVICE), steer.to(DEVICE)
        
        optimizer.zero_grad()
        pred = model(img, sens).squeeze()
        loss = criterion(pred, steer)
        loss.backward()
        optimizer.step()
        train_loss += loss.item()
    
    # Validate
    model.eval()
    val_loss = 0.0
    with torch.no_grad():
        for img, sens, steer in val_loader:
            img, sens, steer = img.to(DEVICE), sens.to(DEVICE), steer.to(DEVICE)
            pred = model(img, sens).squeeze()
            val_loss += criterion(pred, steer).item()
    
    avg_train = train_loss / len(train_loader)
    avg_val = val_loss / len(val_loader)
    
    # Save best model
    if avg_val < best_loss:
        best_loss = avg_val
        torch.save(model.state_dict(), MODEL_NAME)
        print(f"Epoch {epoch+1:3d}: Train {avg_train:.4f} | Val {avg_val:.4f} ⭐ (saved)")
    else:
        print(f"Epoch {epoch+1:3d}: Train {avg_train:.4f} | Val {avg_val:.4f}")

print("="*50)
print(f"✅ DONE! Best validation loss: {best_loss:.4f}")
print(f"   Model saved to: {MODEL_NAME}")
print(f"\n📦 Transfer {MODEL_NAME} to Jetson Nano to test!")
