import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset
import numpy as np
import copy
from radar_augmentation import load_radar_pcd_to_numpy, RadarDataAugmentor
import glob

# ----------------- 1. DATASET -----------------
class RadarDINODataset(Dataset):
    def __init__(self, pcd_folder, is_training=True):
        self.files = glob.glob(f"{pcd_folder}/**/*.pcd", recursive=True)
        self.is_training = is_training
        
        # We need two augmentors for DINO:
        # Teacher: Gets a lightly augmented view ("Global Crop")
        self.teacher_aug = RadarDataAugmentor(doppler_std=0.05, rcs_scale_range=(0.9, 1.1))
        
        # Student: Gets heavily augmented views ("Local Crops" & Noise)
        self.student_aug = RadarDataAugmentor(doppler_std=0.25, rcs_scale_range=(0.7, 1.3))

    def __len__(self):
        return len(self.files)

    def pad_or_sample(self, pts, num_points=1024):
        if pts.shape[0] < num_points:
            padded = np.zeros((num_points, 5))
            padded[:pts.shape[0], :] = pts
            return padded
        else:
            indices = np.random.choice(pts.shape[0], num_points, replace=False)
            return pts[indices]

    def __getitem__(self, idx):
        # Load raw [x, y, z, rcs, doppler] (N, 5) array
        # Note: Set ran_rename_script=False if the PCD fields haven't been renamed
        raw_points = load_radar_pcd_to_numpy(self.files[idx], ran_rename_script=False)
            
        if not self.is_training:
            return torch.tensor(self.pad_or_sample(raw_points), dtype=torch.float32)
            
        # --- DINO MULTI-CROP ---
        # Teacher only gets 2 global views
        v_teacher_1 = self.pad_or_sample(self.teacher_aug.augment_pipeline(raw_points))
        v_teacher_2 = self.pad_or_sample(self.teacher_aug.augment_pipeline(raw_points))
        
        # Student gets global views AND heavy local views
        v_student_1 = self.pad_or_sample(self.student_aug.augment_pipeline(raw_points))
        v_student_2 = self.pad_or_sample(self.student_aug.augment_pipeline(raw_points))
        # Note: For full DINO, you'd add 4-6 more heavy "local" crops (dropping 50% points etc.)
        
        return {
            'teacher': [torch.tensor(v_teacher_1, dtype=torch.float32), torch.tensor(v_teacher_2, dtype=torch.float32)],
            'student': [torch.tensor(v_student_1, dtype=torch.float32), torch.tensor(v_student_2, dtype=torch.float32)]
        }

# ----------------- 2. POINT TRANSFORMER (Placeholder) -----------------
class PointTransformerBackbone(nn.Module):
    """
    Placeholder for a Point-MAE or Point-DINO backbone.
    Usually involves Farthest Point Sampling (FPS) -> kNN grouping -> Mini-PointNet -> Standard Transformer.
    """
    def __init__(self, embed_dim=256, out_dim=1024):
        super().__init__()
        self.fc1 = nn.Linear(5, 64)   # Input is 5D: X, Y, Z, RCS, Doppler
        self.fc2 = nn.Linear(64, embed_dim)
        
        # Standard Transformer Block
        encoder_layer = nn.TransformerEncoderLayer(d_model=embed_dim, nhead=8, batch_first=True)
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=4)
        
        # Projection Head for DINO
        self.head = nn.Sequential(
            nn.Linear(embed_dim, embed_dim),
            nn.GELU(),
            nn.Linear(embed_dim, out_dim)
        )

    def forward(self, x):
        # x is (B, N, 5)
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        features = self.transformer(x) # (B, N, embed_dim)
        
        # Max pool over points to mimic [CLS] token, or use a learned [CLS] token stringed to points
        global_feature = torch.max(features, dim=1)[0] # (B, embed_dim)
        
        # DINO requires projection head
        out = self.head(global_feature)
        return out

# ----------------- 3. DINO LOSS -----------------
class DINOLoss(nn.Module):
    def __init__(self, out_dim, warmup_teacher_temp, teacher_temp, warmup_teacher_temp_epochs, nepochs, student_temp=0.1):
        super().__init__()
        self.student_temp = student_temp
        self.teacher_temp_schedule = np.concatenate((
            np.linspace(warmup_teacher_temp, teacher_temp, warmup_teacher_temp_epochs),
            np.ones(nepochs - warmup_teacher_temp_epochs) * teacher_temp
        ))

    def forward(self, student_output, teacher_output, epoch):
        student_out = student_output / self.student_temp
        student_out = student_out.chunk(2) # We passed 2 global crops

        # Teacher centering and sharpening
        temp = self.teacher_temp_schedule[epoch]
        teacher_out = F.softmax(teacher_output / temp, dim=-1)
        teacher_out = teacher_out.detach().chunk(2)

        total_loss = 0
        n_loss_terms = 0
        
        for iq, q in enumerate(teacher_out):
            for iv, v in enumerate(student_out):
                if iv == iq:
                    continue # Skip matching the exact same view
                loss = torch.sum(-q * F.log_softmax(v, dim=-1), dim=-1)
                total_loss += loss.mean()
                n_loss_terms += 1
                
        total_loss /= n_loss_terms
        return total_loss

# ----------------- 4. TRAINING LOOP -----------------
def train_dino():
    # Setup
    # The external hard drive path containing the PCD dataset
    dataset = RadarDINODataset(pcd_folder='/media/minhduc/TOSHIBA EXT2/4dradarslam/NTU4Dradlm/cp/cp', is_training=True)
    
    # Quick sanity check
    if len(dataset) == 0:
        print("ERROR: No .pcd files found in the specified path! Please check the folder.")
        return
        
    print(f"Dataset successfully loaded. Found {len(dataset)} radar scans.")
    
    dataloader = DataLoader(dataset, batch_size=8, shuffle=True)
    
    # Models
    student = PointTransformerBackbone()
    teacher = PointTransformerBackbone()
    
    # Initialize teacher with student weights
    teacher.load_state_dict(student.state_dict())
    for param in teacher.parameters():
        param.requires_grad = False  # Teacher forms EMA, no gradients!
        
    optimizer = optim.AdamW(student.parameters(), lr=1e-3, weight_decay=0.04)
    dino_loss = DINOLoss(out_dim=1024, warmup_teacher_temp=0.04, teacher_temp=0.04, 
                         warmup_teacher_temp_epochs=0, nepochs=100)
    
    ema_momentum = 0.996
    
    print("Starting DINO 4D Radar Pre-training...")
    epochs = 2 # Testing for 2 epochs
    for epoch in range(epochs):
        for batch_idx, batch in enumerate(dataloader):
            s_views = batch['student'] # list of 2 tensors (B, N, 5)
            t_views = batch['teacher'] # list of 2 tensors (B, N, 5)
            
            # 1. Forward passes
            s_input = torch.cat(s_views)
            t_input = torch.cat(t_views)
            
            s_out = student(s_input)
            with torch.no_grad():
                t_out = teacher(t_input)
                
            # 2. Loss & Backprop
            loss = dino_loss(s_out, t_out, epoch)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            # 3. EMA update for Teacher
            with torch.no_grad():
                m = ema_momentum
                for param_q, param_k in zip(student.parameters(), teacher.parameters()):
                    param_k.data.mul_(m).add_((1 - m) * param_q.detach().data)
                    
            if batch_idx % 10 == 0:
                print(f"Epoch {epoch}/{epochs} | Batch {batch_idx}/{len(dataloader)} | Loss: {loss.item():.4f}")
        
    print("\nTraining Test Run Complete!")

if __name__ == "__main__":
    train_dino()
