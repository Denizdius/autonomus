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
DATA_DIR = "dataset_vga"  # Updated folder
LOG_FILE = os.path.join(DATA_DIR, "log.csv")
MODEL_NAME = "vga_pilot.pth"
SCALER_NAME = "vga_scaler.pkl"
BATCH_SIZE = 64  # Bigger batch for RTX 4080
EPOCHS = 50      # More epochs for better learning
LEARNING_RATE = 1e-4
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Data Augmentation - multiply your samples!
AUGMENT_DATA = True  # Set to False to disable
BRIGHTNESS_RANGE = 0.3  # Random brightness adjustment

print(f"🚀 Using device: {DEVICE}")
if torch.cuda.is_available():
    print(f"   GPU: {torch.cuda.get_device_name(0)}")
    print(f"   VRAM: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")

# --- DATASET WITH AUGMENTATION ---
class RobotDataset(Dataset):
    def __init__(self, images, sensors, steering, augment=False):
        self.images = images
        self.sensors = sensors
        self.steering = steering
        self.augment = augment
        
    def __len__(self): 
        return len(self.images)
    
    def __getitem__(self, idx):
        img = self.images[idx].copy()
        steer = self.steering[idx]
        
        # Data augmentation (only during training)
        if self.augment:
            # Random horizontal flip (and flip steering!)
            if np.random.rand() > 0.5:
                img = np.fliplr(img).copy()
                steer = -steer
            
            # Random brightness adjustment
            brightness = 1.0 + np.random.uniform(-BRIGHTNESS_RANGE, BRIGHTNESS_RANGE)
            img = np.clip(img * brightness, 0, 1)
        
        img = img.transpose((2, 0, 1))
        return (torch.tensor(img, dtype=torch.float32), 
                torch.tensor(self.sensors[idx], dtype=torch.float32), 
                torch.tensor(steer, dtype=torch.float32))

# --- LOAD DATA ---
print(f"Loading {LOG_FILE}...")
try:
    data = pd.read_csv(LOG_FILE)
    print("CSV Headers found:", data.columns.tolist()) 
    if 'image_path' not in data.columns:
        print("ERROR: 'image_path' column missing from CSV!")
except Exception as e:
    print(f"Error reading CSV: {e}")
    exit()

raw_images, raw_sensors, raw_steering = [], [], []

print(f"Processing {len(data)} rows...")

# Reset index to ensure we iterate cleanly 0..N
data = data.reset_index(drop=True)

for index, row in data.iterrows():
    # Force image_path to string to prevent TypeError
    # Using explicit column name 'image_path'
    img_filename = str(row['image_path']).strip() 
    
    if index < 3:
        print(f"Row {index} DEBUG: Read filename '{img_filename}'")

    # Check if filename looks valid
    if not img_filename.lower().endswith('.jpg'):
        if index < 5: 
            print(f"Skipping Row {index}: '{img_filename}' is not a .jpg file")
        continue

    img_path = os.path.join(DATA_DIR, img_filename)
    
    img = cv2.imread(img_path)
    if img is None: 
        if index < 5:
            print(f"Warning: cv2.imread failed for: {os.path.abspath(img_path)}")
        continue
    
    # 640x480 Input
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV) / 255.0
    raw_images.append(img)
    
    # Selecting sensors. You mentioned IMU is 0, so relying mostly on Distance/GPS 
    # and placeholder IMU data is fine. The scaler will handle the 0s.
    sens = [
        row['dist_cm'], 
        row['gps_lat'], 
        row['gps_lon'], 
        row['acc_x'], 
        row['acc_y'], 
        row['acc_z'], 
        row['gyro_x'], 
        row['gyro_y'], 
        row['gyro_z']
    ]
    raw_sensors.append([0.0 if np.isnan(x) else float(x) for x in sens])
    
    raw_steering.append((float(row['cmd_left']) - float(row['cmd_right'])) / 510.0)

if len(raw_images) == 0:
    print("\nCRITICAL ERROR: No valid images/data found.")
    print("Check the debug output above. Are the filenames correct in log.csv?")
    exit()

X_img = np.array(raw_images)
X_sens = np.array(raw_sensors)
y = np.array(raw_steering)

print(f"Successfully loaded {len(X_img)} samples.")

scaler = StandardScaler()
X_sens = scaler.fit_transform(X_sens)
joblib.dump(scaler, SCALER_NAME)

indices = np.arange(len(X_img))
X_img_train, X_img_test, y_train, y_test, idx_train, idx_test = train_test_split(X_img, y, indices, test_size=0.2, random_state=42)
X_sens_train, X_sens_test = X_sens[idx_train], X_sens[idx_test]

# Use augmentation for training, not for validation
train_loader = DataLoader(RobotDataset(X_img_train, X_sens_train, y_train, augment=AUGMENT_DATA), batch_size=BATCH_SIZE, shuffle=True)
val_loader = DataLoader(RobotDataset(X_img_test, X_sens_test, y_test, augment=False), batch_size=BATCH_SIZE)

if AUGMENT_DATA:
    print(f"📈 Data augmentation ENABLED (effectively 2x samples)")

# --- MODEL ARCHITECTURE (High Res VGA) ---
class SequencedPilot(nn.Module):
    def __init__(self):
        super(SequencedPilot, self).__init__()
        # Input: 3 x 480 x 640
        self.cnn = nn.Sequential(
            # Layer 1: 5x5, s2 -> 238 x 318
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(), 
            # Layer 2: 5x5, s2 -> 117 x 157
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            # Layer 3: 5x5, s2 -> 57 x 77
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
            # Layer 4: 3x3, s2 -> 28 x 38 (Added stride)
            nn.Conv2d(48, 64, 3, stride=2), nn.ReLU(),
            # Layer 5: 3x3, s2 -> 13 x 18 (Added stride)
            nn.Conv2d(64, 64, 3, stride=2), nn.ReLU(),
            nn.Flatten()
        )
        # Flatten Size: 64 * 13 * 18 = 14976
        
        self.mlp = nn.Sequential(
            nn.Linear(9, 64), nn.ReLU(), nn.Linear(64, 32), nn.ReLU()
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

# --- TRAINING ---
best_loss = float('inf')
for epoch in range(EPOCHS):
    model.train()
    train_loss = 0.0
    for i, s, t in train_loader:
        i, s, t = i.to(DEVICE), s.to(DEVICE), t.to(DEVICE)
        optimizer.zero_grad()
        loss = criterion(model(i, s).squeeze(), t)
        loss.backward()
        optimizer.step()
        train_loss += loss.item()
    
    model.eval()
    val_loss = 0.0
    with torch.no_grad():
        for i, s, t in val_loader:
            i, s, t = i.to(DEVICE), s.to(DEVICE), t.to(DEVICE)
            val_loss += criterion(model(i, s).squeeze(), t).item()
            
    print(f"Epoch {epoch+1}: Train {train_loss/len(train_loader):.4f} Val {val_loss/len(val_loader):.4f}")
    if (val_loss/len(val_loader)) < best_loss:
        best_loss = val_loss/len(val_loader)
        torch.save(model.state_dict(), MODEL_NAME)

print(f"DONE. Transfer {MODEL_NAME} to Jetson.")
