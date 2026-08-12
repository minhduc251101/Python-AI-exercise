import numpy as np
from scipy.spatial.transform import Rotation
import open3d as o3d

# Load estimated pose from ICP
T_est = np.loadtxt("estimated_relative_pose.txt")
idx1 = 0
idx2 = 100
# Load your ground truth (example: from gt_odom_cp.txt)
# Replace these with your actual GT poses for pose1 and pose2
gt = np.loadtxt("gt_odom_cp.txt")
T1_gt = np.eye(4)
T1_gt[:3, 3] = gt[idx1, 1:4]  # row 0 = pose1
T1_gt[:3, :3] = Rotation.from_quat(gt[idx1, 4:8]).as_matrix()

T2_gt = np.eye(4)
T2_gt[:3, 3] = gt[idx2, 1:4]  # row 100 = pose2
T2_gt[:3, :3] = Rotation.from_quat(gt[idx2, 4:8]).as_matrix()

# Ground truth relative pose
T_gt = np.linalg.inv(T1_gt) @ T2_gt

# Translation error
trans_err = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])

# Rotation error
R_diff = T_est[:3, :3].T @ T_gt[:3, :3]
rot_err = np.degrees(np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1, 1)))
pcd = o3d.io.read_point_cloud("cp_output_point_cloud_map.pcd")
print("num points =", len(pcd.points))
print("T_est:\n", T_est)
print("T_gt:\n", T_gt)
print("valid point index range = 0 to", len(pcd.points) - 1)
print(f"Translation error: {trans_err:.4f} m")
print(f"Rotation error:    {rot_err:.4f} deg")
