# """
# Real Radar Scan Matching Evaluation
# ===================================
# Loads two actual radar point clouds captured at two frames,
# runs scan matching, and compares the estimated relative pose
# against ground truth relative pose.

# Two supported input modes:

# 1) Load scans by index from a directory:
#     python real_radar_scan_matching_eval.py \
#         --scan_dir ./radar_frames \
#         --ext pcd \
#         --idx1 0 --idx2 10 \
#         --gt_poses ground_truth.csv

# 2) Load two explicit scan files:
#     python real_radar_scan_matching_eval.py \
#         --scan1 frame_0001.pcd \
#         --scan2 frame_0010.pcd \
#         --idx1 0 --idx2 10 \
#         --gt_poses ground_truth.csv

# You can also specify poses directly instead of a GT file:
#     python real_radar_scan_matching_eval.py \
#         --scan1 frame_0001.pcd \
#         --scan2 frame_0010.pcd \
#         --pose1 1.0,2.0,0.0,0,0,45 \
#         --pose2 3.0,4.0,0.0,0,0,90

# Ground truth CSV/TXT formats supported:
#     timestamp, x, y, z, qx, qy, qz, qw
#     timestamp, x, y, z, roll, pitch, yaw
#     x, y, z, qx, qy, qz, qw
#     x, y, z, roll, pitch, yaw
#     x, y, z, yaw
#     x, y, yaw
# """

# import numpy as np
# import open3d as o3d
# import argparse
# import os
# import sys
# import copy
# import glob
# import re
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation


# # ============================================================
# # 1. LOADING FUNCTIONS
# # ============================================================

# def natural_sort_key(path):
#     name = os.path.basename(path)
#     return [int(text) if text.isdigit() else text.lower()
#             for text in re.split(r'(\d+)', name)]


# def load_pcd(filepath):
#     """Load a point cloud file and return an Open3D point cloud."""
#     if not os.path.exists(filepath):
#         print(f"[ERROR] File not found: {filepath}")
#         sys.exit(1)

#     pcd = o3d.io.read_point_cloud(filepath)
#     print(f"  Loaded {filepath}: {len(pcd.points)} points")
#     return pcd


# def get_scan_file_list(scan_dir, ext="pcd"):
#     """Get a naturally sorted list of scan files."""
#     if not os.path.isdir(scan_dir):
#         print(f"[ERROR] Scan directory not found: {scan_dir}")
#         sys.exit(1)

#     pattern = os.path.join(scan_dir, f"*.{ext}")
#     files = glob.glob(pattern)
#     files = sorted(files, key=natural_sort_key)

#     if len(files) == 0:
#         print(f"[ERROR] No '.{ext}' files found in: {scan_dir}")
#         sys.exit(1)

#     return files


# def load_scan_by_index(scan_dir, idx, ext="pcd"):
#     files = get_scan_file_list(scan_dir, ext)
#     if idx < 0 or idx >= len(files):
#         print(f"[ERROR] Scan index out of range. Available: 0-{len(files)-1}")
#         sys.exit(1)
#     filepath = files[idx]
#     return load_pcd(filepath), filepath


# def parse_pose_to_T(row):
#     """
#     Convert a pose row to a 4x4 homogeneous matrix.
#     Handles:
#       [x,y,z,qx,qy,qz,qw]
#       [x,y,z,roll,pitch,yaw]   (radians)
#     """
#     T = np.eye(4)

#     if len(row) == 7:
#         T[:3, 3] = row[:3]
#         T[:3, :3] = Rotation.from_quat(row[3:7]).as_matrix()
#     elif len(row) == 6:
#         T[:3, 3] = row[:3]
#         T[:3, :3] = Rotation.from_euler('xyz', row[3:6]).as_matrix()
#     else:
#         print(f"[ERROR] Cannot parse pose with {len(row)} values. Expected 6 or 7.")
#         sys.exit(1)

#     return T


# def load_ground_truth(filepath):
#     """
#     Load ground truth poses from CSV/TXT.
#     Auto-detects format based on number of columns.
#     Returns: list of 4x4 homogeneous transformation matrices.
#     """
#     if not os.path.exists(filepath):
#         print(f"[ERROR] Ground truth file not found: {filepath}")
#         sys.exit(1)

#     for delim in [',', None]:
#         try:
#             data = np.loadtxt(filepath, delimiter=delim, comments='#')
#             if data.ndim == 1:
#                 data = data.reshape(1, -1)
#             break
#         except Exception:
#             continue
#     else:
#         print(f"[ERROR] Could not parse {filepath}")
#         sys.exit(1)

#     poses = []
#     ncols = data.shape[1]

#     for row in data:
#         T = np.eye(4)

#         if ncols == 8:
#             T = parse_pose_to_T(row[1:8])
#         elif ncols == 7:
#             if row[0] > 1e9:
#                 T[:3, 3] = row[1:4]
#                 T[:3, :3] = Rotation.from_euler('xyz', row[4:7]).as_matrix()
#             else:
#                 T = parse_pose_to_T(row[:7])
#         elif ncols == 6:
#             T = parse_pose_to_T(row[:6])
#         elif ncols == 4:
#             T[:3, 3] = [row[0], row[1], row[2]]
#             T[:3, :3] = Rotation.from_euler('z', row[3]).as_matrix()
#         elif ncols == 3:
#             T[:3, 3] = [row[0], row[1], 0.0]
#             T[:3, :3] = Rotation.from_euler('z', row[2]).as_matrix()
#         else:
#             print(f"[ERROR] Unexpected column count: {ncols}")
#             sys.exit(1)

#         poses.append(T)

#     print(f"  Loaded {len(poses)} ground truth poses from {filepath}")
#     return poses


# # ============================================================
# # 2. OVERLAP CHECK
# # ============================================================

# def check_overlap(scan1, scan2, T_relative, overlap_radius=1.0):
#     """
#     Check how much scan2 overlaps with scan1 after transforming scan2
#     using T_relative.
#     Returns overlap ratio (0.0 to 1.0).
#     """
#     scan2_transformed = copy.deepcopy(scan2)
#     scan2_transformed.transform(T_relative)

#     if len(scan1.points) == 0 or len(scan2_transformed.points) == 0:
#         return 0.0

#     tree = o3d.geometry.KDTreeFlann(scan1)

#     count = 0
#     for i in range(len(scan2_transformed.points)):
#         [k, _, _] = tree.search_radius_vector_3d(
#             scan2_transformed.points[i], overlap_radius
#         )
#         if k > 0:
#             count += 1

#     return count / len(scan2_transformed.points)


# # ============================================================
# # 3. SCAN MATCHING (ICP variants)
# # ============================================================

# def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
#     """Point-to-point ICP."""
#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )


# def run_icp_point_to_plane(source, target, init_T=np.eye(4), max_dist=2.0):
#     """Point-to-plane ICP (requires normals)."""
#     for pcd in [source, target]:
#         if not pcd.has_normals():
#             pcd.estimate_normals(
#                 search_param=o3d.geometry.KDTreeSearchParamHybrid(
#                     radius=1.0, max_nn=30
#                 )
#             )

#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )


# def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
#     """Generalized ICP."""
#     for pcd in [source, target]:
#         if not pcd.has_normals():
#             pcd.estimate_normals(
#                 search_param=o3d.geometry.KDTreeSearchParamHybrid(
#                     radius=1.0, max_nn=30
#                 )
#             )

#     return o3d.pipelines.registration.registration_generalized_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )


# # ============================================================
# # 4. EVALUATION
# # ============================================================

# def compute_relative_pose(T1, T2):
#     """T_1->2 = inv(T1) @ T2"""
#     return np.linalg.inv(T1) @ T2


# def pose_error(T_est, T_gt):
#     """Returns translation error (m) and rotation error (deg)."""
#     trans_error = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])

#     R_diff = T_est[:3, :3].T @ T_gt[:3, :3]
#     trace_val = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
#     rot_error = np.degrees(np.arccos(trace_val))

#     return trans_error, rot_error


# def print_transform(T, label=""):
#     """Pretty print a 4x4 transform."""
#     t = T[:3, 3]
#     r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
#     dist = np.linalg.norm(t)

#     print(f"  {label}")
#     print(f"    Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
#     print(f"    Rotation:    [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)")


# def overlap_label(overlap, threshold=0.30):
#     return "OVERLAPPING" if overlap >= threshold else "NON-OVERLAPPING / WEAK OVERLAP"


# # ============================================================
# # 5. VISUALIZATION
# # ============================================================

# def visualize_scans_3d(scan1, scan2, T_estimated, T_ground_truth):
#     """
#     3D Open3D interactive viewer:
#       Red:   Reference scan (scan 1 / target)
#       Green: Scan 2 aligned by estimated pose
#       Blue:  Scan 2 aligned by ground truth pose
#     """
#     s1 = copy.deepcopy(scan1)
#     s1.paint_uniform_color([1, 0, 0])

#     s2_est = copy.deepcopy(scan2)
#     s2_est.transform(T_estimated)
#     s2_est.paint_uniform_color([0, 1, 0])

#     s2_gt = copy.deepcopy(scan2)
#     s2_gt.transform(T_ground_truth)
#     s2_gt.paint_uniform_color([0, 0, 1])

#     print("\n  3D Visualization colors:")
#     print("    Red   = Reference scan (scan 1)")
#     print("    Green = Scan 2 aligned by ESTIMATED pose")
#     print("    Blue  = Scan 2 aligned by GROUND TRUTH pose")

#     o3d.visualization.draw_geometries(
#         [s1, s2_est, s2_gt],
#         window_name="Scan Matching: Red=ref, Green=estimated, Blue=ground truth",
#         width=1200, height=800
#     )


# def visualize_scans_2d(scan1, scan2, T_estimated, T_ground_truth, save=False):
#     """
#     2D matplotlib scatter plots:
#       Figure 1: Before alignment
#       Figure 2: Estimated alignment
#       Figure 3: Ground-truth alignment
#     """
#     pts1 = np.asarray(scan1.points)
#     pts2 = np.asarray(scan2.points)

#     s2_est = copy.deepcopy(scan2)
#     s2_est.transform(T_estimated)
#     pts2_est = np.asarray(s2_est.points)

#     s2_gt = copy.deepcopy(scan2)
#     s2_gt.transform(T_ground_truth)
#     pts2_gt = np.asarray(s2_gt.points)

#     fig1, ax1 = plt.subplots(figsize=(10, 8))
#     ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c='tab:orange', label='Current scan')
#     ax1.set_xlabel('X (m)')
#     ax1.set_ylabel('Y (m)')
#     ax1.set_title('Two Real Radar Scans (before alignment)')
#     ax1.legend(loc='upper left', markerscale=5)
#     ax1.set_aspect('equal')
#     ax1.grid(True, alpha=0.3)

#     fig2, ax2 = plt.subplots(figsize=(10, 8))
#     ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:orange', label='Current scan (estimated aligned)')
#     ax2.set_xlabel('X (m)')
#     ax2.set_ylabel('Y (m)')
#     ax2.set_title('Scan Matching Result (estimated pose)')
#     ax2.legend(loc='upper left', markerscale=5)
#     ax2.set_aspect('equal')
#     ax2.grid(True, alpha=0.3)

#     fig3, ax3 = plt.subplots(figsize=(10, 8))
#     ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:green', label='Current scan (ground-truth aligned)')
#     ax3.set_xlabel('X (m)')
#     ax3.set_ylabel('Y (m)')
#     ax3.set_title('Ground Truth Alignment')
#     ax3.legend(loc='upper left', markerscale=5)
#     ax3.set_aspect('equal')
#     ax3.grid(True, alpha=0.3)

#     if save:
#         fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches='tight')
#         fig2.savefig("plot_icp_aligned.png", dpi=150, bbox_inches='tight')
#         fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches='tight')
#         print("  Saved: plot_before_alignment.png, plot_icp_aligned.png, plot_gt_aligned.png")

#     plt.show()


# # ============================================================
# # 6. PREPROCESSING
# # ============================================================

# def preprocess(pcd, voxel_size=0.1):
#     """Downsample and remove outliers."""
#     pcd_down = pcd.voxel_down_sample(voxel_size)
#     _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
#     pcd_clean = pcd_down.select_by_index(ind)
#     print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
#     return pcd_clean


# # ============================================================
# # 7. ARGUMENT VALIDATION
# # ============================================================

# def validate_inputs(args):
#     using_pair_files = (args.scan1 is not None) or (args.scan2 is not None)
#     using_scan_dir = args.scan_dir is not None

#     if using_pair_files and using_scan_dir:
#         print("[ERROR] Use either (--scan1 and --scan2) OR --scan_dir, not both.")
#         sys.exit(1)

#     if not using_pair_files and not using_scan_dir:
#         print("[ERROR] Provide either (--scan1 and --scan2) OR --scan_dir.")
#         sys.exit(1)

#     if using_pair_files and (args.scan1 is None or args.scan2 is None):
#         print("[ERROR] If using direct file mode, both --scan1 and --scan2 are required.")
#         sys.exit(1)

#     if (args.pose1 is None) ^ (args.pose2 is None):
#         print("[ERROR] Provide both --pose1 and --pose2 together.")
#         sys.exit(1)


# # ============================================================
# # 8. MAIN
# # ============================================================

# def main():
#     parser = argparse.ArgumentParser(
#         description="Evaluate scan matching between two real radar point clouds"
#     )

#     # Input mode A: explicit files
#     parser.add_argument("--scan1", default=None,
#                         help="Path to radar scan 1 (reference / target)")
#     parser.add_argument("--scan2", default=None,
#                         help="Path to radar scan 2 (current / source)")

#     # Input mode B: directory + indices
#     parser.add_argument("--scan_dir", default=None,
#                         help="Directory containing scan files")
#     parser.add_argument("--ext", default="pcd",
#                         help="Scan file extension in --scan_dir (default: pcd)")

#     # Frame indices
#     parser.add_argument("--idx1", type=int, default=0,
#                         help="Frame index / GT pose index for scan 1")
#     parser.add_argument("--idx2", type=int, default=1,
#                         help="Frame index / GT pose index for scan 2")

#     # GT or manual poses
#     parser.add_argument("--gt_poses", default=None,
#                         help="Ground truth poses CSV/TXT file")
#     parser.add_argument("--pose1", default=None,
#                         help="Manual pose 1: x,y,z,roll,pitch,yaw (degrees)")
#     parser.add_argument("--pose2", default=None,
#                         help="Manual pose 2: x,y,z,roll,pitch,yaw (degrees)")

#     parser.add_argument("--use_gt_init", action="store_true",
#                         help="Use GT relative pose as ICP initialization")

#     # Processing and evaluation
#     parser.add_argument("--voxel_size", type=float, default=0.1,
#                         help="Voxel downsampling size (default: 0.1m)")
#     parser.add_argument("--icp_max_dist", type=float, default=2.0,
#                         help="ICP max correspondence distance (default: 2.0m)")
#     parser.add_argument("--overlap_radius", type=float, default=1.0,
#                         help="Radius for overlap check (default: 1.0m)")
#     parser.add_argument("--overlap_threshold", type=float, default=0.30,
#                         help="Threshold used to label pair as overlapping (default: 0.30)")
#     parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "all"],
#                         default="all", help="Registration method")

#     # Output
#     parser.add_argument("--no_vis", action="store_true",
#                         help="Skip visualization")
#     parser.add_argument("--save_plots", action="store_true",
#                         help="Save matplotlib plots as PNG files")
#     parser.add_argument("--save_pose", action="store_true",
#                         help="Save best estimated relative pose to estimated_relative_pose.txt")

#     args = parser.parse_args()
#     validate_inputs(args)

#     print("=" * 72)
#     print("  REAL RADAR SCAN MATCHING EVALUATION")
#     print("=" * 72)

#     # --- Load scans ---
#     print("\n[1] Loading radar scans...")
#     if args.scan_dir is not None:
#         scan1, scan1_path = load_scan_by_index(args.scan_dir, args.idx1, ext=args.ext)
#         scan2, scan2_path = load_scan_by_index(args.scan_dir, args.idx2, ext=args.ext)
#         print(f"  Scan 1 file: {scan1_path}")
#         print(f"  Scan 2 file: {scan2_path}")
#     else:
#         scan1 = load_pcd(args.scan1)
#         scan2 = load_pcd(args.scan2)
#         scan1_path = args.scan1
#         scan2_path = args.scan2

#     if len(scan1.points) == 0 or len(scan2.points) == 0:
#         print("\n[ERROR] One or both scans are empty.")
#         sys.exit(1)

#     # --- Get poses ---
#     print("\n[2] Getting poses...")
#     has_gt = False
#     T_relative_gt = None
#     gt_poses = None

#     if args.pose1 and args.pose2:
#         vals1 = [float(v) for v in args.pose1.split(',')]
#         vals2 = [float(v) for v in args.pose2.split(',')]

#         if len(vals1) != 6 or len(vals2) != 6:
#             print("[ERROR] --pose1 and --pose2 must each have 6 values: x,y,z,roll,pitch,yaw")
#             sys.exit(1)

#         vals1[3:] = np.radians(vals1[3:]).tolist()
#         vals2[3:] = np.radians(vals2[3:]).tolist()

#         T1 = parse_pose_to_T(np.array(vals1))
#         T2 = parse_pose_to_T(np.array(vals2))
#         has_gt = True
#         print("  Using manual poses")

#     elif args.gt_poses:
#         gt_poses = load_ground_truth(args.gt_poses)

#         if args.idx1 >= len(gt_poses) or args.idx2 >= len(gt_poses):
#             print(f"[ERROR] GT index out of range. Available: 0-{len(gt_poses)-1}")
#             sys.exit(1)

#         T1 = gt_poses[args.idx1]
#         T2 = gt_poses[args.idx2]
#         has_gt = True
#         print(f"  Using GT poses at indices {args.idx1} and {args.idx2}")

#         if args.scan_dir is not None:
#             scan_files = get_scan_file_list(args.scan_dir, args.ext)
#             print(f"  Scan files found: {len(scan_files)}")
#             if len(scan_files) != len(gt_poses):
#                 print("  [WARNING] Number of scan files and GT poses differ.")
#                 print("  [WARNING] Index-based pairing is only valid if their ordering still matches.")

#     else:
#         print("  [WARNING] No GT or manual poses provided.")
#         print("  [WARNING] Pose errors and GT-overlap comparison will be skipped.")

#     if has_gt:
#         print_transform(T1, "Pose 1 (world/map frame):")
#         print_transform(T2, "Pose 2 (world/map frame):")

#         pose_dist = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
#         print(f"\n  Distance between poses: {pose_dist:.4f} m")

#         T_relative_gt = compute_relative_pose(T1, T2)
#         print_transform(T_relative_gt, "Ground truth relative pose (1 -> 2):")
#     else:
#         pose_dist = None

#     # --- Preprocess ---
#     print(f"\n[3] Preprocessing (voxel_size={args.voxel_size}m)...")
#     print("  Scan 1:")
#     scan1_proc = preprocess(scan1, args.voxel_size)
#     print("  Scan 2:")
#     scan2_proc = preprocess(scan2, args.voxel_size)

#     if len(scan1_proc.points) == 0 or len(scan2_proc.points) == 0:
#         print("\n[ERROR] One or both processed scans are empty.")
#         sys.exit(1)

#     # --- Overlap baseline ---
#     print(f"\n[4] Checking overlap...")
#     if has_gt:
#         overlap_gt = check_overlap(
#             scan1_proc, scan2_proc, T_relative_gt,
#             overlap_radius=args.overlap_radius
#         )
#         print("  Using ground-truth relative pose for overlap check")
#         print(f"  Overlap using GT pose: {overlap_gt:.2%}")
#         print(f"  GT overlap decision:   {overlap_label(overlap_gt, args.overlap_threshold)}")

#         T_init = T_relative_gt if args.use_gt_init else np.eye(4)
#         overlap_init = check_overlap(
#             scan1_proc, scan2_proc, T_init,
#             overlap_radius=args.overlap_radius
#         )
#         init_name = "ground truth" if args.use_gt_init else "identity"
#         print(f"  ICP initial transform: {init_name}")
#         print(f"  Overlap using ICP init: {overlap_init:.2%}")
#     else:
#         overlap_gt = None
#         T_init = np.eye(4)
#         overlap_init = check_overlap(
#             scan1_proc, scan2_proc, T_init,
#             overlap_radius=args.overlap_radius
#         )
#         print("  No GT available, using identity init")
#         print(f"  Overlap using ICP init: {overlap_init:.2%}")

#     if has_gt and overlap_gt < args.overlap_threshold:
#         print("  [WARNING] GT says this pair has weak overlap. Registration may fail even if code is correct.")
#     elif overlap_init < args.overlap_threshold:
#         print("  [WARNING] Initialization overlap is weak. ICP may be unreliable.")
#     else:
#         print("  Sufficient overlap for registration.")

#     # --- Registration ---
#     print(f"\n[5] Running scan matching...")
#     methods = {
#         "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
#         "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
#         "gicp":        ("Generalized ICP",    run_generalized_icp),
#     }

#     to_run = methods if args.method == "all" else {args.method: methods[args.method]}

#     results = {}
#     best_key = None
#     best_fitness = -1.0

#     for key, (name, func) in to_run.items():
#         print(f"\n  --- {name} ---")

#         src = copy.deepcopy(scan2_proc)
#         tgt = copy.deepcopy(scan1_proc)

#         result = func(src, tgt, init_T=T_init, max_dist=args.icp_max_dist)
#         T_est = result.transformation

#         overlap_est = check_overlap(
#             scan1_proc, scan2_proc, T_est,
#             overlap_radius=args.overlap_radius
#         )

#         print(f"    Fitness:           {result.fitness:.4f}")
#         print(f"    Inlier RMSE:       {result.inlier_rmse:.4f}")
#         print_transform(T_est, "Estimated relative pose:")
#         print(f"    Overlap (est):     {overlap_est:.2%}")

#         entry = {
#             "name": name,
#             "transform": T_est,
#             "fitness": result.fitness,
#             "rmse": result.inlier_rmse,
#             "overlap_est": overlap_est,
#         }

#         if has_gt:
#             t_err, r_err = pose_error(T_est, T_relative_gt)
#             print(f"    Translation error: {t_err:.4f} m")
#             print(f"    Rotation error:    {r_err:.4f} deg")
#             entry["trans_error"] = t_err
#             entry["rot_error"] = r_err

#         results[key] = entry

#         if result.fitness > best_fitness:
#             best_fitness = result.fitness
#             best_key = key

#     # --- Summary ---
#     print("\n" + "=" * 72)
#     print("  RESULTS SUMMARY")
#     print("=" * 72)

#     print(f"\n  Scan 1: {scan1_path}")
#     print(f"  Scan 2: {scan2_path}")

#     if has_gt:
#         print(f"  Distance between poses:          {pose_dist:.4f} m")
#         print(f"  GT relative distance:            {np.linalg.norm(T_relative_gt[:3, 3]):.4f} m")
#         print(f"  Overlap before ICP (GT pose):    {overlap_gt:.2%}")
#         print(f"  Overlap before ICP (init pose):  {overlap_init:.2%}")
#         print(f"  GT overlap label:                {overlap_label(overlap_gt, args.overlap_threshold)}")
#         print(f"\n  {'Method':<25} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 96)

#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(f"  {r['name']:<25} {r['trans_error']:<14.4f} {r['rot_error']:<14.4f} "
#                   f"{r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")
#     else:
#         print(f"  Overlap before ICP (init pose):  {overlap_init:.2%}")
#         print(f"\n  {'Method':<25} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 64)

#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(f"  {r['name']:<25} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")

#     best_T = results[best_key]["transform"]

#     print(f"\n  Best method: {results[best_key]['name']}")
#     print_transform(best_T, "Estimated relative pose:")

#     if has_gt:
#         print_transform(T_relative_gt, "Ground truth relative pose:")
#         best_overlap_est = results[best_key]["overlap_est"]
#         print(f"\n  Overlap after ESTIMATED alignment:    {best_overlap_est:.2%}")
#         print(f"  Overlap after GROUND-TRUTH alignment: {overlap_gt:.2%}")
#         print(f"  Overlap difference (est - gt):        {(best_overlap_est - overlap_gt):.2%}")

#     # --- Save pose ---
#     if args.save_pose:
#         np.savetxt("estimated_relative_pose.txt", best_T, fmt="%.6f")
#         print("\n  Saved estimated pose: estimated_relative_pose.txt")

#     # --- Visualization ---
#     if not args.no_vis and best_key:
#         print("\n[6] Visualization...")
#         T_vis_gt = T_relative_gt if has_gt else best_T
#         visualize_scans_2d(
#             scan1_proc, scan2_proc,
#             best_T, T_vis_gt,
#             save=args.save_plots
#         )
#         visualize_scans_3d(
#             scan1_proc, scan2_proc,
#             best_T, T_vis_gt
#         )

#     print("\nDone.")


# if __name__ == "__main__":
#     main()

# #!/usr/bin/env python3
# """
# Real Radar Scan Matching Evaluation
# ===================================

# Supports two input modes:

# A) Nested frame folders (recommended for your dataset):
#    scan_root/
#       000000/
#          cloud.pcd
#          data
#       000001/
#          cloud.pcd
#          data
#       ...

# Example:
#     python real_radar_scan_matching_eval.py \
#         --scan_root /path/to/loop_true_fullpose \
#         --idx1 0 --idx2 10 \
#         --gt_poses ground_truth.txt

# B) Two explicit scan files:
#     python real_radar_scan_matching_eval.py \
#         --scan1 frame_0001.pcd \
#         --scan2 frame_0010.pcd \
#         --data1 frame_0001_data.txt \
#         --data2 frame_0010_data.txt \
#         --gt_poses ground_truth.txt

# You can also specify poses directly:
#     python real_radar_scan_matching_eval.py \
#         --scan1 frame_0001.pcd \
#         --scan2 frame_0010.pcd \
#         --pose1 1.0,2.0,0.0,0,0,45 \
#         --pose2 3.0,4.0,0.0,0,0,90

# Ground truth formats supported:
#     timestamp, x, y, z, qx, qy, qz, qw
#     timestamp, x, y, z, roll, pitch, yaw
#     timestamp, x, y, z, yaw
#     timestamp, x, y, yaw
#     x, y, z, qx, qy, qz, qw
#     x, y, z, roll, pitch, yaw
#     x, y, z, yaw
#     x, y, yaw

# Important:
# - For ICP, source = scan2, target = scan1.
# - Therefore GT alignment must also map scan2 -> scan1.
# - This script fixes that direction consistently.
# """

# import argparse
# import copy
# import glob
# import os
# import re
# import sys

# import matplotlib.pyplot as plt
# import numpy as np
# import open3d as o3d
# from scipy.spatial.transform import Rotation


# # ============================================================
# # 1. BASIC HELPERS
# # ============================================================

# def natural_sort_key(path):
#     name = os.path.basename(path)
#     return [int(text) if text.isdigit() else text.lower()
#             for text in re.split(r'(\d+)', name)]


# def is_unix_timestamp(x):
#     return x > 1e9


# def load_pcd(filepath):
#     if not os.path.exists(filepath):
#         print(f"[ERROR] File not found: {filepath}")
#         sys.exit(1)

#     pcd = o3d.io.read_point_cloud(filepath)
#     print(f"  Loaded {filepath}: {len(pcd.points)} points")
#     return pcd


# def parse_numeric_tokens(line):
#     tokens = re.split(r'[,\s]+', line.strip())
#     vals = []
#     for tok in tokens:
#         if tok == "":
#             continue
#         try:
#             vals.append(float(tok))
#         except ValueError:
#             continue
#     return vals


# # ============================================================
# # 2. DATASET LAYOUT: scan_root/000000/{cloud.pcd,data}
# # ============================================================

# def get_frame_entries(scan_root, scan_name="cloud.pcd", meta_name="data"):
#     if not os.path.isdir(scan_root):
#         print(f"[ERROR] Scan root not found: {scan_root}")
#         sys.exit(1)

#     folders = sorted(
#         [d for d in glob.glob(os.path.join(scan_root, "*")) if os.path.isdir(d)],
#         key=natural_sort_key
#     )

#     entries = []
#     for folder in folders:
#         scan_path = os.path.join(folder, scan_name)
#         meta_path = os.path.join(folder, meta_name)

#         if os.path.exists(scan_path):
#             entries.append({
#                 "folder": folder,
#                 "frame_name": os.path.basename(folder),
#                 "scan_path": scan_path,
#                 "meta_path": meta_path if os.path.exists(meta_path) else None
#             })

#     if len(entries) == 0:
#         print(f"[ERROR] No frame folders containing {scan_name} found in: {scan_root}")
#         sys.exit(1)

#     return entries


# def load_scan_from_entries(entries, idx):
#     if idx < 0 or idx >= len(entries):
#         print(f"[ERROR] Frame index out of range. Available: 0-{len(entries)-1}")
#         sys.exit(1)

#     entry = entries[idx]
#     return load_pcd(entry["scan_path"]), entry


# # ============================================================
# # 3. TIMESTAMP PARSING
# # ============================================================

# def read_text_file(filepath):
#     if filepath is None or not os.path.exists(filepath):
#         return ""
#     with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
#         return f.read()


# def extract_timestamp_from_text(text):
#     """
#     Very tolerant parser for frame metadata files.
#     Tries, in order:
#       1) 'timestamp' or 'stamp' followed by sec+nsec
#       2) a unix float like 1654233181.775779453
#       3) a pair like 1654233181 775779453
#       4) first plausible unix timestamp token
#     """
#     if not text:
#         return None

#     # 1) labeled: timestamp: 1654233181 775779453
#     m = re.search(r'(?:timestamp|stamp)\D+(\d{10})\D+(\d{1,9})', text, re.IGNORECASE)
#     if m:
#         sec = int(m.group(1))
#         nsec = int(m.group(2))
#         return sec + nsec * 1e-9

#     # 2) labeled float: timestamp: 1654233181.775779453
#     m = re.search(r'(?:timestamp|stamp)\D+(\d{10}\.\d+)', text, re.IGNORECASE)
#     if m:
#         return float(m.group(1))

#     # 3) any float timestamp
#     m = re.search(r'(\d{10}\.\d+)', text)
#     if m:
#         return float(m.group(1))

#     # 4) sec nsec pair anywhere
#     m = re.search(r'(\d{10})\s+(\d{1,9})', text)
#     if m:
#         sec = int(m.group(1))
#         nsec = int(m.group(2))
#         return sec + nsec * 1e-9

#     # 5) first standalone unix timestamp token
#     nums = parse_numeric_tokens(text)
#     for v in nums:
#         if is_unix_timestamp(v):
#             return float(v)

#     return None


# def read_frame_timestamp(meta_path):
#     text = read_text_file(meta_path)
#     ts = extract_timestamp_from_text(text)
#     if ts is None:
#         raise ValueError(f"Could not parse frame timestamp from: {meta_path}")
#     return ts


# # ============================================================
# # 4. POSE PARSING
# # ============================================================

# def pose_xyz_quat_to_T(x, y, z, qx, qy, qz, qw):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
#     return T


# def pose_xyz_rpy_to_T(x, y, z, roll, pitch, yaw, degrees=False):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees).as_matrix()
#     return T


# def pose_xy_yaw_to_T(x, y, yaw, z=0.0, degrees=False):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     T[:3, :3] = Rotation.from_euler('z', yaw, degrees=degrees).as_matrix()
#     return T


# def parse_gt_row(values):
#     """
#     Returns:
#         timestamp_or_None, T
#     Supports:
#         [timestamp, x, y, z, qx, qy, qz, qw]  -> 8
#         [timestamp, x, y, z, roll, pitch, yaw] -> 7
#         [x, y, z, qx, qy, qz, qw] -> 7
#         [timestamp, x, y, z, yaw] -> 5
#         [timestamp, x, y, yaw] -> 4
#         [x, y, z, roll, pitch, yaw] -> 6
#         [x, y, z, yaw] -> 4
#         [x, y, yaw] -> 3
#     """
#     n = len(values)

#     if n == 8:
#         ts = values[0]
#         T = pose_xyz_quat_to_T(*values[1:8])
#         return ts, T

#     if n == 7:
#         if is_unix_timestamp(values[0]):
#             ts = values[0]
#             T = pose_xyz_rpy_to_T(*values[1:7], degrees=False)
#             return ts, T
#         else:
#             T = pose_xyz_quat_to_T(*values[:7])
#             return None, T

#     if n == 6:
#         T = pose_xyz_rpy_to_T(*values[:6], degrees=False)
#         return None, T

#     if n == 5 and is_unix_timestamp(values[0]):
#         ts = values[0]
#         T = pose_xy_yaw_to_T(values[1], values[2], values[4], z=values[3], degrees=False)
#         return ts, T

#     if n == 4:
#         if is_unix_timestamp(values[0]):
#             ts = values[0]
#             T = pose_xy_yaw_to_T(values[1], values[2], values[3], z=0.0, degrees=False)
#             return ts, T
#         else:
#             T = pose_xy_yaw_to_T(values[0], values[1], values[3], z=values[2], degrees=False)
#             return None, T

#     if n == 3:
#         T = pose_xy_yaw_to_T(values[0], values[1], values[2], z=0.0, degrees=False)
#         return None, T

#     raise ValueError(f"Unsupported pose row with {n} numeric values: {values}")


# def load_ground_truth(filepath):
#     if not os.path.exists(filepath):
#         print(f"[ERROR] Ground truth file not found: {filepath}")
#         sys.exit(1)

#     rows = []
#     with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
#         for line in f:
#             line = line.strip()
#             if line == "" or line.startswith("#"):
#                 continue

#             vals = parse_numeric_tokens(line)
#             if len(vals) == 0:
#                 continue

#             try:
#                 ts, T = parse_gt_row(vals)
#                 rows.append({
#                     "timestamp": ts,
#                     "T": T,
#                     "raw": vals
#                 })
#             except Exception:
#                 continue

#     if len(rows) == 0:
#         print(f"[ERROR] Could not parse any valid GT poses from: {filepath}")
#         sys.exit(1)

#     n_with_ts = sum(r["timestamp"] is not None for r in rows)
#     print(f"  Loaded {len(rows)} ground truth poses from {filepath} ({n_with_ts} rows have timestamps)")
#     return rows


# def nearest_gt_index(frame_ts, gt_rows):
#     gt_timestamps = np.array([
#         np.nan if r["timestamp"] is None else float(r["timestamp"])
#         for r in gt_rows
#     ])

#     valid = np.where(~np.isnan(gt_timestamps))[0]
#     if len(valid) == 0:
#         raise ValueError("GT file has no timestamps; cannot do timestamp-based matching.")

#     best = valid[np.argmin(np.abs(gt_timestamps[valid] - frame_ts))]
#     return int(best)


# # ============================================================
# # 5. TRANSFORMS / ERRORS
# # ============================================================

# def transform_from_A_to_B(T_A_world, T_B_world):
#     """
#     Given:
#         T_A_world : pose of frame A in world
#         T_B_world : pose of frame B in world

#     Returns:
#         T_A_to_B = inv(T_A_world) @ T_B_world
#     """
#     return np.linalg.inv(T_A_world) @ T_B_world


# def print_transform(T, label=""):
#     t = T[:3, 3]
#     r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
#     dist = np.linalg.norm(t)

#     print(f"  {label}")
#     print(f"    Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
#     print(f"    Rotation:    [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)")


# def pose_error(T_est, T_gt):
#     trans_error = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])

#     R_err = T_est[:3, :3].T @ T_gt[:3, :3]
#     val = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
#     rot_error_deg = np.degrees(np.arccos(val))

#     return trans_error, rot_error_deg


# # ============================================================
# # 6. PREPROCESSING
# # ============================================================

# def preprocess(pcd, voxel_size=0.1):
#     pcd_down = pcd.voxel_down_sample(voxel_size)
#     if len(pcd_down.points) == 0:
#         return pcd_down

#     _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
#     pcd_clean = pcd_down.select_by_index(ind)

#     print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
#     return pcd_clean


# # ============================================================
# # 7. OVERLAP
# # ============================================================

# def check_overlap(target_scan, source_scan, T_source_to_target, overlap_radius=1.0):
#     """
#     target_scan: reference scan (scan1)
#     source_scan: current scan (scan2)
#     T_source_to_target: transform that maps source -> target
#     """
#     src = copy.deepcopy(source_scan)
#     src.transform(T_source_to_target)

#     if len(target_scan.points) == 0 or len(src.points) == 0:
#         return 0.0

#     tree = o3d.geometry.KDTreeFlann(target_scan)

#     count = 0
#     for p in src.points:
#         k, _, _ = tree.search_radius_vector_3d(p, overlap_radius)
#         if k > 0:
#             count += 1

#     return count / len(src.points)


# def overlap_label(overlap, threshold=0.30):
#     return "OVERLAPPING" if overlap >= threshold else "NON-OVERLAPPING / WEAK OVERLAP"


# # ============================================================
# # 8. FPFH PLOTTING
# # ============================================================

# # def plot_fpfh_bins(pcd, voxel_size=0.1, point_idx=0, prefix="scan"):
# #     """
# #     Plot FPFH descriptor bins for one point.
# #     FPFH has 33 bins = 11 alpha + 11 phi + 11 theta.
# #     """
# #     pcd_feat, fpfh = preprocess_for_fpfh(pcd, voxel_size)
# #     fpfh_array = np.asarray(fpfh.data)   # shape: (33, N)

# #     if fpfh_array.shape[1] == 0:
# #         print(f"  [WARNING] No FPFH descriptors available for {prefix}.")
# #         return

# #     if point_idx < 0 or point_idx >= fpfh_array.shape[1]:
# #         print(f"  [WARNING] point_idx={point_idx} out of range for {prefix}. Using point 0 instead.")
# #         point_idx = 0

# #     desc = fpfh_array[:, point_idx]

# #     alpha_bins = desc[0:11]
# #     phi_bins   = desc[11:22]
# #     theta_bins = desc[22:33]

# #     print(f"  {prefix} FPFH shape: {fpfh_array.shape}")
# #     print(f"  Plotting FPFH for {prefix}, point index {point_idx}")

# #     fig, axes = plt.subplots(1, 3, figsize=(15, 4))

# #     axes[0].bar(range(11), alpha_bins, color='steelblue')
# #     axes[0].set_title(r'$\alpha$ (normal vs connection vector)')
# #     axes[0].set_xlabel('Bin')
# #     axes[0].set_ylabel('Value')

# #     axes[1].bar(range(11), phi_bins, color='coral')
# #     axes[1].set_title(r'$\phi$ (normal vs normal)')
# #     axes[1].set_xlabel('Bin')

# #     axes[2].bar(range(11), theta_bins, color='seagreen')
# #     axes[2].set_title(r'$\theta$ (rotation around axis)')
# #     axes[2].set_xlabel('Bin')

# #     fig.suptitle(
# #         f'FPFH Descriptor for {prefix}, point {point_idx} (33 bins = 11 + 11 + 11)',
# #         fontsize=14, fontweight='bold'
# #     )
# #     plt.tight_layout()
# #     fig.savefig(f"{prefix}_fpfh_histogram_point_{point_idx}.png", dpi=150, bbox_inches='tight')
# #     print(f"  Saved: {prefix}_fpfh_histogram_point_{point_idx}.png")

# #     fig2, ax = plt.subplots(figsize=(12, 4))
# #     colors = ['steelblue'] * 11 + ['coral'] * 11 + ['seagreen'] * 11
# #     ax.bar(range(33), desc, color=colors)
# #     ax.axvline(x=10.5, color='black', linestyle='--', linewidth=0.8)
# #     ax.axvline(x=21.5, color='black', linestyle='--', linewidth=0.8)
# #     ax.set_xlabel('Bin Index')
# #     ax.set_ylabel('Value')
# #     ax.set_title(f'Full FPFH Descriptor for {prefix}, point {point_idx}')

# #     ymax = max(desc) if np.max(desc) > 0 else 1.0
# #     ax.text(5,  ymax * 0.9, r'$\alpha$', ha='center', fontsize=14)
# #     ax.text(16, ymax * 0.9, r'$\phi$',   ha='center', fontsize=14)
# #     ax.text(27, ymax * 0.9, r'$\theta$', ha='center', fontsize=14)

# #     plt.tight_layout()
# #     fig2.savefig(f"{prefix}_fpfh_full_point_{point_idx}.png", dpi=150, bbox_inches='tight')
# #     print(f"  Saved: {prefix}_fpfh_full_point_{point_idx}.png")

# #     plt.show()
# # ============================================================
# # FPFH UTILITIES
# # ============================================================

# def preprocess_for_fpfh(pcd, voxel_size=0.1, normal_radius=None, feature_radius=None,
#                         normal_max_nn=30, feature_max_nn=100):
#     """
#     Prepare a cloud for FPFH:
#       1) estimate normals
#       2) compute FPFH descriptors

#     Returns:
#         pcd_feat : point cloud with normals
#         fpfh     : Open3D Feature object, shape (33, N)
#     """
#     pcd_feat = copy.deepcopy(pcd)

#     if len(pcd_feat.points) == 0:
#         return pcd_feat, None

#     if normal_radius is None:
#         normal_radius = voxel_size * 2.0
#     if feature_radius is None:
#         feature_radius = voxel_size * 5.0

#     pcd_feat.estimate_normals(
#         o3d.geometry.KDTreeSearchParamHybrid(
#             radius=normal_radius,
#             max_nn=normal_max_nn
#         )
#     )

#     try:
#         pcd_feat.orient_normals_consistent_tangent_plane(10)
#     except Exception:
#         pass

#     fpfh = o3d.pipelines.registration.compute_fpfh_feature(
#         pcd_feat,
#         o3d.geometry.KDTreeSearchParamHybrid(
#             radius=feature_radius,
#             max_nn=feature_max_nn
#         )
#     )

#     return pcd_feat, fpfh


# def plot_fpfh_bins_exact(pcd, voxel_size=0.1, point_idx=0, prefix="scan",
#                          normal_radius=None, feature_radius=None,
#                          normal_max_nn=30, feature_max_nn=100,
#                          save=True, show=True):
#     """
#     Plot the exact FPFH descriptor of one selected point.
#     No fallback, no strongest-descriptor replacement, no top-k logic.
#     """
#     pcd_feat, fpfh = preprocess_for_fpfh(
#         pcd,
#         voxel_size=voxel_size,
#         normal_radius=normal_radius,
#         feature_radius=feature_radius,
#         normal_max_nn=normal_max_nn,
#         feature_max_nn=feature_max_nn
#     )

#     if fpfh is None:
#         print(f"[WARNING] No FPFH descriptors available for {prefix}.")
#         return

#     fpfh_array = np.asarray(fpfh.data)   # shape: (33, N)

#     if fpfh_array.ndim != 2 or fpfh_array.shape[0] != 33:
#         print(f"[WARNING] Unexpected FPFH shape for {prefix}: {fpfh_array.shape}")
#         return

#     n_points = fpfh_array.shape[1]
#     if n_points == 0:
#         print(f"[WARNING] No FPFH descriptors available for {prefix}.")
#         return

#     if point_idx < 0 or point_idx >= n_points:
#         print(f"[ERROR] point_idx={point_idx} out of range for {prefix}. "
#               f"Valid range: [0, {n_points - 1}]")
#         return

#     desc = fpfh_array[:, point_idx]
#     desc_norm = np.linalg.norm(desc)

#     alpha_bins = desc[0:11]
#     phi_bins   = desc[11:22]
#     theta_bins = desc[22:33]

#     print(f"  {prefix} FPFH shape: {fpfh_array.shape}")
#     print(f"  Plotting EXACT FPFH for {prefix}, point index {point_idx}")
#     print(f"  Descriptor L2 norm: {desc_norm:.6f}")
#     print("  Descriptor:")
#     print(desc)

#     fig, axes = plt.subplots(1, 3, figsize=(15, 4))

#     axes[0].bar(range(11), alpha_bins, color='steelblue', edgecolor='black', linewidth=0.5)
#     axes[0].set_title(r'$\alpha$')
#     axes[0].set_xlabel('Bin')
#     axes[0].set_ylabel('Value')
#     axes[0].grid(True, alpha=0.3)

#     axes[1].bar(range(11), phi_bins, color='coral', edgecolor='black', linewidth=0.5)
#     axes[1].set_title(r'$\phi$')
#     axes[1].set_xlabel('Bin')
#     axes[1].grid(True, alpha=0.3)

#     axes[2].bar(range(11), theta_bins, color='seagreen', edgecolor='black', linewidth=0.5)
#     axes[2].set_title(r'$\theta$')
#     axes[2].set_xlabel('Bin')
#     axes[2].grid(True, alpha=0.3)

#     fig.suptitle(
#         f'{prefix} | point {point_idx} | norm={desc_norm:.3f}',
#         fontsize=13, fontweight='bold'
#     )
#     plt.subplots_adjust(top=0.82, wspace=0.28)

#     if save:
#         out1 = f"{prefix}_point_{point_idx}_split.png"
#         fig.savefig(out1, dpi=150, bbox_inches='tight')
#         print(f"  Saved: {out1}")

#     fig2, ax = plt.subplots(figsize=(16, 5))
#     colors = ['steelblue'] * 11 + ['coral'] * 11 + ['seagreen'] * 11
#     ax.bar(range(33), desc, color=colors, edgecolor='black', linewidth=0.5)
#     ax.axvline(x=10.5, color='black', linestyle='--', linewidth=1.0)
#     ax.axvline(x=21.5, color='black', linestyle='--', linewidth=1.0)
#     ax.set_xlabel('Bin Index')
#     ax.set_ylabel('Value')
#     ax.set_title(f'Full FPFH Descriptor for {prefix}, point {point_idx}')
#     ax.grid(True, alpha=0.3)

#     ymax = max(np.max(desc), 1.0)
#     ax.text(5,  ymax * 0.92, r'$\alpha$', ha='center', fontsize=16)
#     ax.text(16, ymax * 0.92, r'$\phi$',   ha='center', fontsize=16)
#     ax.text(27, ymax * 0.92, r'$\theta$', ha='center', fontsize=16)

#     plt.subplots_adjust(top=0.88, bottom=0.15)

#     if save:
#         out2 = f"{prefix}_point_{point_idx}_full.png"
#         fig2.savefig(out2, dpi=150, bbox_inches='tight')
#         print(f"  Saved: {out2}")

#     if show:
#         plt.show()
#     else:
#         plt.close(fig)
#         plt.close(fig2)


# def run_fpfh_ransac_icp(source, target, voxel_size=0.1, max_dist=2.0,
#                         normal_radius=None, feature_radius=None,
#                         normal_max_nn=30, feature_max_nn=100,
#                         ransac_distance_threshold=None,
#                         ransac_n=4,
#                         ransac_max_iteration=100000,
#                         ransac_confidence=500):
#     """
#     FPFH feature matching -> global RANSAC -> ICP refinement.

#     source = scan2 (current)
#     target = scan1 (reference)

#     Returns:
#         result_icp, result_ransac
#     """
#     if normal_radius is None:
#         normal_radius = voxel_size * 2.0
#     if feature_radius is None:
#         feature_radius = voxel_size * 5.0
#     if ransac_distance_threshold is None:
#         ransac_distance_threshold = voxel_size * 3.0

#     source_feat_cloud, source_fpfh = preprocess_for_fpfh(
#         source,
#         voxel_size=voxel_size,
#         normal_radius=normal_radius,
#         feature_radius=feature_radius,
#         normal_max_nn=normal_max_nn,
#         feature_max_nn=feature_max_nn
#     )

#     target_feat_cloud, target_fpfh = preprocess_for_fpfh(
#         target,
#         voxel_size=voxel_size,
#         normal_radius=normal_radius,
#         feature_radius=feature_radius,
#         normal_max_nn=normal_max_nn,
#         feature_max_nn=feature_max_nn
#     )

#     if source_fpfh is None or target_fpfh is None:
#         identity = np.eye(4)
#         empty_result = o3d.pipelines.registration.RegistrationResult()
#         empty_result.transformation = identity
#         return empty_result, empty_result

#     result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#         source_feat_cloud,
#         target_feat_cloud,
#         source_fpfh,
#         target_fpfh,
#         mutual_filter=True,
#         max_correspondence_distance=ransac_distance_threshold,
#         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#         ransac_n=ransac_n,
#         checkers=[
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(ransac_distance_threshold)
#         ],
#         criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
#             ransac_max_iteration,
#             ransac_confidence
#         )
#     )

#     src_icp = copy.deepcopy(source)
#     tgt_icp = copy.deepcopy(target)

#     src_icp.estimate_normals(
#         search_param=o3d.geometry.KDTreeSearchParamHybrid(
#             radius=normal_radius,
#             max_nn=normal_max_nn
#         )
#     )
#     tgt_icp.estimate_normals(
#         search_param=o3d.geometry.KDTreeSearchParamHybrid(
#             radius=normal_radius,
#             max_nn=normal_max_nn
#         )
#     )

#     result_icp = o3d.pipelines.registration.registration_icp(
#         src_icp,
#         tgt_icp,
#         max_dist,
#         result_ransac.transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )

#     return result_icp, result_ransac
# # ============================================================
# # 8. ICP + SCAN MATCHING
# # ============================================================

# def ensure_normals(pcd):
#     if not pcd.has_normals():
#         pcd.estimate_normals(
#             search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
#         )


# def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )


# def run_icp_point_to_plane(source, target, init_T=np.eye(4), max_dist=2.0):
#     ensure_normals(source)
#     ensure_normals(target)

#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )


# def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
#     ensure_normals(source)
#     ensure_normals(target)

#     return o3d.pipelines.registration.registration_generalized_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500,
#             relative_fitness=1e-6,
#             relative_rmse=1e-6
#         )
#     )



# # ============================================================
# # 9. VISUALIZATION
# # ============================================================

# def visualize_scans_3d(scan1, scan2, T_est_source_to_target, T_gt_source_to_target):
#     """
#     Red   : scan1 (reference / target)
#     Green : scan2 aligned by estimated transform (source -> target)
#     Blue  : scan2 aligned by GT transform (source -> target)
#     """
#     s1 = copy.deepcopy(scan1)
#     s1.paint_uniform_color([1, 0, 0])

#     s2_est = copy.deepcopy(scan2)
#     s2_est.transform(T_est_source_to_target)
#     s2_est.paint_uniform_color([0, 1, 0])

#     s2_gt = copy.deepcopy(scan2)
#     s2_gt.transform(T_gt_source_to_target)
#     s2_gt.paint_uniform_color([0, 0, 1])

#     print("\n  3D Visualization colors:")
#     print("    Red   = Reference scan (scan1)")
#     print("    Green = Scan2 aligned by ESTIMATED pose")
#     print("    Blue  = Scan2 aligned by GROUND TRUTH pose")

#     o3d.visualization.draw_geometries(
#         [s1, s2_est, s2_gt],
#         window_name="Scan Matching: Red=reference, Green=estimated, Blue=ground truth",
#         width=1200, height=800
#     )


# def visualize_scans_2d(scan1, scan2, T_est_source_to_target, T_gt_source_to_target, save=False):
#     pts1 = np.asarray(scan1.points)
#     pts2 = np.asarray(scan2.points)

#     s2_est = copy.deepcopy(scan2)
#     s2_est.transform(T_est_source_to_target)
#     pts2_est = np.asarray(s2_est.points)

#     s2_gt = copy.deepcopy(scan2)
#     s2_gt.transform(T_gt_source_to_target)
#     pts2_gt = np.asarray(s2_gt.points)

#     fig1, ax1 = plt.subplots(figsize=(10, 8))
#     ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c='tab:orange', label='Current scan')
#     ax1.set_title('Before alignment')
#     ax1.set_xlabel('X (m)')
#     ax1.set_ylabel('Y (m)')
#     ax1.set_aspect('equal')
#     ax1.grid(True, alpha=0.3)
#     ax1.legend(loc='upper left', markerscale=5)

#     fig2, ax2 = plt.subplots(figsize=(10, 8))
#     ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:orange', label='Scan2 aligned by estimated pose')
#     ax2.set_title('Estimated alignment')
#     ax2.set_xlabel('X (m)')
#     ax2.set_ylabel('Y (m)')
#     ax2.set_aspect('equal')
#     ax2.grid(True, alpha=0.3)
#     ax2.legend(loc='upper left', markerscale=5)

#     fig3, ax3 = plt.subplots(figsize=(10, 8))
#     ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:green', label='Scan2 aligned by GT pose')
#     ax3.set_title('Ground-truth alignment')
#     ax3.set_xlabel('X (m)')
#     ax3.set_ylabel('Y (m)')
#     ax3.set_aspect('equal')
#     ax3.grid(True, alpha=0.3)
#     ax3.legend(loc='upper left', markerscale=5)

#     if save:
#         fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches='tight')
#         fig2.savefig("plot_icp_aligned.png", dpi=150, bbox_inches='tight')
#         fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches='tight')
#         print("  Saved: plot_before_alignment.png, plot_icp_aligned.png, plot_gt_aligned.png")

#     plt.show()


# # ============================================================
# # 10. ARGUMENT VALIDATION
# # ============================================================

# def validate_inputs(args):
#     using_root = args.scan_root is not None
#     using_pair = (args.scan1 is not None) or (args.scan2 is not None)

#     if using_root and using_pair:
#         print("[ERROR] Use either --scan_root OR (--scan1 and --scan2), not both.")
#         sys.exit(1)

#     if not using_root and not using_pair:
#         print("[ERROR] Provide either --scan_root OR (--scan1 and --scan2).")
#         sys.exit(1)

#     if using_pair and (args.scan1 is None or args.scan2 is None):
#         print("[ERROR] If using explicit file mode, both --scan1 and --scan2 are required.")
#         sys.exit(1)

#     if (args.pose1 is None) ^ (args.pose2 is None):
#         print("[ERROR] Provide both --pose1 and --pose2 together.")
#         sys.exit(1)

#     if (args.stamp1 is None) ^ (args.stamp2 is None):
#         print("[ERROR] Provide both --stamp1 and --stamp2 together.")
#         sys.exit(1)

#     if (args.data1 is None) ^ (args.data2 is None):
#         print("[ERROR] Provide both --data1 and --data2 together.")
#         sys.exit(1)


# # ============================================================
# # 11. MAIN
# # ============================================================

# def main():
#     parser = argparse.ArgumentParser(
#         description="Evaluate scan matching between two real radar point clouds"
#     )

#     # Dataset folder mode
#     parser.add_argument("--scan_root", "--scan_dir", dest="scan_root", default=None,
#                         help="Root folder containing frame folders like 000000/000001/...")

#     parser.add_argument("--scan_name", default="cloud.pcd",
#                         help="Point cloud filename inside each frame folder (default: cloud.pcd)")
#     parser.add_argument("--meta_name", default="data",
#                         help="Metadata filename inside each frame folder (default: data)")

#     # Explicit file mode
#     parser.add_argument("--scan1", default=None,
#                         help="Path to scan1 (reference / target)")
#     parser.add_argument("--scan2", default=None,
#                         help="Path to scan2 (current / source)")
#     parser.add_argument("--data1", default=None,
#                         help="Metadata file for scan1 (used to extract timestamp)")
#     parser.add_argument("--data2", default=None,
#                         help="Metadata file for scan2 (used to extract timestamp)")
#     parser.add_argument("--stamp1", type=float, default=None,
#                         help="Explicit timestamp for scan1")
#     parser.add_argument("--stamp2", type=float, default=None,
#                         help="Explicit timestamp for scan2")

#     # Indices
#     parser.add_argument("--idx1", type=int, default=0,
#                         help="Index of frame1")
#     parser.add_argument("--idx2", type=int, default=1,
#                         help="Index of frame2")

#     # GT or manual poses
#     parser.add_argument("--gt_poses", default=None,
#                         help="Ground truth CSV/TXT file")
#     parser.add_argument("--pose1", default=None,
#                         help="Manual pose1: x,y,z,roll,pitch,yaw (degrees)")
#     parser.add_argument("--pose2", default=None,
#                         help="Manual pose2: x,y,z,roll,pitch,yaw (degrees)")
#     parser.add_argument("--use_gt_init", action="store_true",
#                         help="Use GT transform as ICP initialization")

#     # Processing
#     parser.add_argument("--voxel_size", type=float, default=0.1,
#                         help="Voxel size (default: 0.1 m)")
#     parser.add_argument("--icp_max_dist", type=float, default=2.0,
#                         help="ICP max correspondence distance (default: 2.0 m)")
#     parser.add_argument("--overlap_radius", type=float, default=1.0,
#                         help="Radius for overlap check (default: 1.0 m)")
#     parser.add_argument("--overlap_threshold", type=float, default=0.30,
#                         help="Overlap threshold used for labeling (default: 0.30)")
#     # parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "all"],
#     #                     default="all", help="Registration method")
#     parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "fpfh", "all"],
#                         default="all", help="Registration method")
#     parser.add_argument("--plot_fpfh", action="store_true",
#                         help="Plot FPFH descriptor bins for selected scan points")
#     parser.add_argument("--fpfh_point_idx1", type=int, default=0,
#                         help="Point index for FPFH plotting in scan 1")
#     parser.add_argument("--fpfh_point_idx2", type=int, default=100,
#                         help="Point index for FPFH plotting in scan 2")


#     # Output
#     parser.add_argument("--no_vis", action="store_true",
#                         help="Skip visualization")
#     parser.add_argument("--save_plots", action="store_true",
#                         help="Save matplotlib plots")
#     parser.add_argument("--save_pose", action="store_true",
#                         help="Save best estimated source->target transform")
#     parser.add_argument("--save_pose_name", default="estimated_relative_pose.txt",
#                         help="Output filename for saved pose")

#     args = parser.parse_args()
#     validate_inputs(args)

#     print("=" * 78)
#     print("  REAL RADAR SCAN MATCHING EVALUATION (TIMESTAMP-AWARE)")
#     print("=" * 78)

#     # --------------------------------------------------------
#     # [1] Load scans
#     # --------------------------------------------------------
#     print("\n[1] Loading scans...")

#     frame_ts1 = None
#     frame_ts2 = None
#     gt_match_mode = None

#     if args.scan_root is not None:
#         entries = get_frame_entries(args.scan_root, args.scan_name, args.meta_name)

#         scan1, entry1 = load_scan_from_entries(entries, args.idx1)
#         scan2, entry2 = load_scan_from_entries(entries, args.idx2)

#         scan1_path = entry1["scan_path"]
#         scan2_path = entry2["scan_path"]

#         print(f"  Frame 1 folder: {entry1['folder']}")
#         print(f"  Frame 2 folder: {entry2['folder']}")

#         if entry1["meta_path"] is not None:
#             frame_ts1 = read_frame_timestamp(entry1["meta_path"])
#         if entry2["meta_path"] is not None:
#             frame_ts2 = read_frame_timestamp(entry2["meta_path"])

#     else:
#         scan1 = load_pcd(args.scan1)
#         scan2 = load_pcd(args.scan2)
#         scan1_path = args.scan1
#         scan2_path = args.scan2

#         if args.stamp1 is not None and args.stamp2 is not None:
#             frame_ts1 = args.stamp1
#             frame_ts2 = args.stamp2
#         elif args.data1 is not None and args.data2 is not None:
#             frame_ts1 = read_frame_timestamp(args.data1)
#             frame_ts2 = read_frame_timestamp(args.data2)

#     if len(scan1.points) == 0 or len(scan2.points) == 0:
#         print("[ERROR] One or both scans are empty.")
#         sys.exit(1)

#     if frame_ts1 is not None:
#         print(f"  Frame1 timestamp: {frame_ts1:.9f}")
#     if frame_ts2 is not None:
#         print(f"  Frame2 timestamp: {frame_ts2:.9f}")

#     # --------------------------------------------------------
#     # [2] Get poses
#     # --------------------------------------------------------
#     print("\n[2] Resolving poses...")

#     has_gt = False
#     T1_world = None
#     T2_world = None
#     T_gt_source_to_target = None
#     matched_gt_idx1 = None
#     matched_gt_idx2 = None

#     if args.pose1 and args.pose2:
#         vals1 = [float(v) for v in args.pose1.split(',')]
#         vals2 = [float(v) for v in args.pose2.split(',')]

#         if len(vals1) != 6 or len(vals2) != 6:
#             print("[ERROR] --pose1 and --pose2 must have 6 values: x,y,z,roll,pitch,yaw")
#             sys.exit(1)

#         vals1[3:] = np.radians(vals1[3:]).tolist()
#         vals2[3:] = np.radians(vals2[3:]).tolist()

#         T1_world = pose_xyz_rpy_to_T(*vals1, degrees=False)
#         T2_world = pose_xyz_rpy_to_T(*vals2, degrees=False)
#         has_gt = True
#         gt_match_mode = "manual poses"
#         print("  Using manual poses")

#     elif args.gt_poses:
#         gt_rows = load_ground_truth(args.gt_poses)

#         # Prefer timestamp matching when possible
#         if frame_ts1 is not None and frame_ts2 is not None:
#             try:
#                 matched_gt_idx1 = nearest_gt_index(frame_ts1, gt_rows)
#                 matched_gt_idx2 = nearest_gt_index(frame_ts2, gt_rows)

#                 T1_world = gt_rows[matched_gt_idx1]["T"]
#                 T2_world = gt_rows[matched_gt_idx2]["T"]
#                 has_gt = True
#                 gt_match_mode = "timestamp"

#                 gt_ts1 = gt_rows[matched_gt_idx1]["timestamp"]
#                 gt_ts2 = gt_rows[matched_gt_idx2]["timestamp"]

#                 print("  Using timestamp-based GT matching")
#                 print(f"  Frame1 -> GT row {matched_gt_idx1}, GT timestamp {gt_ts1:.9f}, |dt| = {abs(frame_ts1 - gt_ts1):.9f}s")
#                 print(f"  Frame2 -> GT row {matched_gt_idx2}, GT timestamp {gt_ts2:.9f}, |dt| = {abs(frame_ts2 - gt_ts2):.9f}s")

#             except Exception as e:
#                 print(f"  [WARNING] Timestamp matching failed: {e}")
#                 print("  [WARNING] Falling back to index-based GT matching")

#         if not has_gt:
#             if args.idx1 >= len(gt_rows) or args.idx2 >= len(gt_rows):
#                 print(f"[ERROR] GT index out of range. Available: 0-{len(gt_rows)-1}")
#                 sys.exit(1)

#             matched_gt_idx1 = args.idx1
#             matched_gt_idx2 = args.idx2
#             T1_world = gt_rows[matched_gt_idx1]["T"]
#             T2_world = gt_rows[matched_gt_idx2]["T"]
#             has_gt = True
#             gt_match_mode = "index"
#             print(f"  Using index-based GT rows {matched_gt_idx1} and {matched_gt_idx2}")

#     else:
#         print("  [WARNING] No GT file and no manual poses provided.")
#         print("  [WARNING] Errors versus GT will be skipped.")

#     if has_gt:
#         print_transform(T1_world, "Pose of scan1 in world/map frame:")
#         print_transform(T2_world, "Pose of scan2 in world/map frame:")

#         scan_world_distance = np.linalg.norm(T1_world[:3, 3] - T2_world[:3, 3])
#         print(f"\n  Distance between scan1 and scan2 poses: {scan_world_distance:.4f} m")

#         # Important:
#         # scan1 = target/reference
#         # scan2 = source/current
#         # GT alignment should map scan2 -> scan1
#         T_gt_source_to_target = transform_from_A_to_B(T1_world, T2_world)
#         print_transform(T_gt_source_to_target, "Ground-truth transform (scan2 -> scan1):")

#     # --------------------------------------------------------
#     # [3] Preprocess
#     # --------------------------------------------------------
#     print(f"\n[3] Preprocessing (voxel_size={args.voxel_size:.3f} m)...")
#     print("  Scan1:")
#     scan1_proc = preprocess(scan1, args.voxel_size)
#     print("  Scan2:")
#     scan2_proc = preprocess(scan2, args.voxel_size)

#     if len(scan1_proc.points) == 0 or len(scan2_proc.points) == 0:
#         print("[ERROR] One or both processed scans are empty.")
#         sys.exit(1)
#     # --------------------------------------------------------
#     # [3.5] PLOTTING FPFH
#     # --------------------------------------------------------
#     if args.plot_fpfh:
#         print("\n[3.5] Plotting EXACT FPFH descriptor bins...")
#         prefix1 = f"scan1_poseidx_{args.idx1}"
#         prefix2 = f"scan2_poseidx_{args.idx2}"

#     plot_fpfh_bins_exact(
#         scan1_proc,
#         voxel_size=args.voxel_size,
#         point_idx=args.fpfh_point_idx1,
#         prefix=prefix1,
#         save=True,
#         show=True
#     )

#     plot_fpfh_bins_exact(
#         scan2_proc,
#         voxel_size=args.voxel_size,
#         point_idx=args.fpfh_point_idx2,
#         prefix=prefix2,
#         save=True,
#         show=True
#     )
#     # --------------------------------------------------------
#     # [4] Overlap
#     # --------------------------------------------------------
#     print(f"\n[4] Checking overlap...")

#     if has_gt:
#         overlap_gt = check_overlap(
#             scan1_proc, scan2_proc,
#             T_gt_source_to_target,
#             overlap_radius=args.overlap_radius
#         )
#         print("  Using GT transform (scan2 -> scan1) for overlap check")
#         print(f"  Overlap using GT pose: {overlap_gt:.2%}")
#         print(f"  GT overlap decision:   {overlap_label(overlap_gt, args.overlap_threshold)}")

#         T_init = T_gt_source_to_target if args.use_gt_init else np.eye(4)
#         init_name = "ground truth" if args.use_gt_init else "identity"

#         overlap_init = check_overlap(
#             scan1_proc, scan2_proc,
#             T_init,
#             overlap_radius=args.overlap_radius
#         )
#         print(f"  ICP initialization:   {init_name}")
#         print(f"  Overlap using init:   {overlap_init:.2%}")
#     else:
#         overlap_gt = None
#         T_init = np.eye(4)
#         overlap_init = check_overlap(
#             scan1_proc, scan2_proc,
#             T_init,
#             overlap_radius=args.overlap_radius
#         )
#         print("  No GT available, using identity initialization")
#         print(f"  Overlap using init:   {overlap_init:.2%}")

#     if has_gt and overlap_gt < args.overlap_threshold:
#         print("  [WARNING] GT says this pair has weak overlap. Registration may fail even if code is correct.")
#     elif overlap_init < args.overlap_threshold:
#         print("  [WARNING] Initialization overlap is weak. ICP may be unreliable.")
#     else:
#         print("  Sufficient overlap for registration.")

#     # --------------------------------------------------------
#     # [5] Registration
#     # --------------------------------------------------------
#     print(f"\n[5] Running scan matching...")

#     # methods = {
#     #     "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
#     #     "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
#     #     "gicp":        ("Generalized ICP",    run_generalized_icp),
#     # }
#     methods = {
#     "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
#     "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
#     "gicp":        ("Generalized ICP",    run_generalized_icp),
#     "fpfh":        ("FPFH + RANSAC + ICP", run_fpfh_ransac_icp),
#     }
#     to_run = methods if args.method == "all" else {args.method: methods[args.method]}

#     results = {}
#     # best_key = None
#     # best_fitness = -1.0

#     # for key, (name, func) in to_run.items():
#     #     print(f"\n  --- {name} ---")

#     #     src = copy.deepcopy(scan2_proc)  # source/current
#     #     tgt = copy.deepcopy(scan1_proc)  # target/reference

#     #     result = func(src, tgt, init_T=T_init, max_dist=args.icp_max_dist)
#     #     T_est_source_to_target = result.transformation

#     #     overlap_est = check_overlap(
#     #         scan1_proc, scan2_proc,
#     #         T_est_source_to_target,
#     #         overlap_radius=args.overlap_radius
#     #     )

#     #     print(f"    Fitness:           {result.fitness:.4f}")
#     #     print(f"    Inlier RMSE:       {result.inlier_rmse:.4f}")
#     #     print_transform(T_est_source_to_target, "Estimated transform (scan2 -> scan1):")
#     #     print(f"    Overlap (est):     {overlap_est:.2%}")

#     #     entry = {
#     #         "name": name,
#     #         "transform": T_est_source_to_target,
#     #         "fitness": result.fitness,
#     #         "rmse": result.inlier_rmse,
#     #         "overlap_est": overlap_est,
#     #     }

#     #     if has_gt:
#     #         t_err, r_err = pose_error(T_est_source_to_target, T_gt_source_to_target)
#     #         print(f"    Translation error: {t_err:.4f} m")
#     #         print(f"    Rotation error:    {r_err:.4f} deg")
#     #         entry["trans_error"] = t_err
#     #         entry["rot_error"] = r_err

#     #     results[key] = entry

#     #     if result.fitness > best_fitness:
#     #         best_fitness = result.fitness
#     #         best_key = key
    
#     # --------------------------------------------------------
#     # [6] Summary
#     # --------------------------------------------------------
#     # print("\n" + "=" * 78)
#     # print("  RESULTS SUMMARY")
#     # print("=" * 78)

#     # print(f"\n  Scan1 (target/reference): {scan1_path}")
#     # print(f"  Scan2 (source/current):   {scan2_path}")

#     # if has_gt:
#     #     print(f"  GT matching mode:                {gt_match_mode}")
#     #     if gt_match_mode == "timestamp":
#     #         print(f"  Matched GT rows:                 scan1->{matched_gt_idx1}, scan2->{matched_gt_idx2}")

#     #     print(f"  Overlap before ICP (GT pose):    {overlap_gt:.2%}")
#     #     print(f"  Overlap before ICP (init pose):  {overlap_init:.2%}")
#     #     print(f"  GT overlap label:                {overlap_label(overlap_gt, args.overlap_threshold)}")
#     #     print(f"\n  {'Method':<25} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#     #     print("  " + "-" * 96)

#     #     for key, r in results.items():
#     #         tag = " * best" if key == best_key else ""
#     #         print(f"  {r['name']:<25} {r['trans_error']:<14.4f} {r['rot_error']:<14.4f} "
#     #               f"{r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")
#     # else:
#     #     print(f"  Overlap before ICP (init pose):  {overlap_init:.2%}")
#     #     print(f"\n  {'Method':<25} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#     #     print("  " + "-" * 64)

#     #     for key, r in results.items():
#     #         tag = " * best" if key == best_key else ""
#     #         print(f"  {r['name']:<25} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")

#     # best_T = results[best_key]["transform"]

#     # print(f"\n  Best method: {results[best_key]['name']}")
#     # print_transform(best_T, "Best estimated transform (scan2 -> scan1):")

#     # if has_gt:
#     #     print_transform(T_gt_source_to_target, "Ground truth transform (scan2 -> scan1):")
#     #     print(f"\n  Overlap after ESTIMATED alignment:    {results[best_key]['overlap_est']:.2%}")
#     #     print(f"  Overlap after GROUND-TRUTH alignment: {overlap_gt:.2%}")
#     #     print(f"  Overlap difference (est - gt):        {(results[best_key]['overlap_est'] - overlap_gt):.2%}")

#     # # Save pose
#     # if args.save_pose:
#     #     np.savetxt(args.save_pose_name, best_T, fmt="%.6f")
#     #     print(f"\n  Saved estimated pose: {args.save_pose_name}")

#     best_key = None
#     best_score = -1e18

#     for key, (name, func) in to_run.items():
#         print(f"\n  --- {name} ---")
#         src = copy.deepcopy(scan2_proc)   # source/current
#         tgt = copy.deepcopy(scan1_proc)   # target/reference

#         if key == "fpfh":
#             result, result_ransac = func(
#                 src, tgt,
#                 voxel_size=args.voxel_size,
#                 max_dist=args.icp_max_dist
#             )
#             print(f"    RANSAC fitness:     {result_ransac.fitness:.4f}")
#             print(f"    RANSAC inlier RMSE: {result_ransac.inlier_rmse:.4f}")
#             print_transform(result_ransac.transformation, "    RANSAC initial pose:")
#         else:
#             result = func(src, tgt, init_T=T_init, max_dist=args.icp_max_dist)
#             result_ransac = None

#         T_est = result.transformation

#         # Re-evaluate consistently in Open3D
#         eval_reg = o3d.pipelines.registration.evaluate_registration(
#             src, tgt, args.icp_max_dist, T_est
#         )
#         fitness = eval_reg.fitness
#         rmse = eval_reg.inlier_rmse

#         overlap_est = check_overlap(
#             scan1_proc, scan2_proc, T_est,
#             overlap_radius=args.overlap_radius
#         )

#         valid = (fitness > 1e-6) and (overlap_est >= args.overlap_threshold)

#         entry = {
#             "name": name,
#             "transform": T_est,
#             "fitness": fitness,
#             "rmse": rmse,
#             "overlap_est": overlap_est,
#             "valid": valid,
#         }

#         if result_ransac is not None:
#             entry["ransac_fitness"] = result_ransac.fitness
#             entry["ransac_rmse"] = result_ransac.inlier_rmse
#             entry["ransac_transform"] = result_ransac.transformation

#         print(f"    Fitness:           {fitness:.4f}")
#         print(f"    Inlier RMSE:       {rmse:.4f}")
#         print_transform(T_est, "    Estimated transform scan2 -> scan1:")
#         print(f"    Overlap (est):     {overlap_est:.2%}")
#         print(f"    Valid candidate:   {valid}")

#         if has_gt:
#             t_err, r_err = pose_error(T_est, T_gt_source_to_target)
#             entry["trans_error"] = t_err
#             entry["rot_error"] = r_err
#             print(f"    Translation error: {t_err:.4f} m")
#             print(f"    Rotation error:    {r_err:.4f} deg")

#             # GT-based score: smaller is better, invalid => reject hard
#             score = -(t_err + 0.02 * r_err) if valid else -1e18
#         else:
#             # No GT: combine consistency metrics, invalid => reject hard
#             score = (2.0 * fitness + overlap_est - 0.1 * rmse) if valid else -1e18

#         entry["score"] = score
#         results[key] = entry

#         if score > best_score:
#             best_score = score
#             best_key = key

#     print("\n" + "=" * 78)
#     print("RESULTS SUMMARY")
#     print("=" * 78)

#     print(f"\n  Scan1 (target/reference): {scan1_path}")
#     print(f"  Scan2 (source/current):   {scan2_path}")

#     if has_gt:
#         print(f"  GT matching mode: {gt_match_mode}")
#         if gt_match_mode == "timestamp":
#             print(f"  Matched GT rows: scan1->{matched_gt_idx1}, scan2->{matched_gt_idx2}")
#         print(f"  Overlap before ICP (GT pose):   {overlap_gt:.2%}")
#         print(f"  Overlap before ICP (init pose): {overlap_init:.2%}")
#         print(f"  GT overlap label: {overlap_label(overlap_gt, args.overlap_threshold)}")

#         print(f"\n  {'Method':<25} {'Valid':<8} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 108)
#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(
#                 f"  {r['name']:<25} "
#                 f"{str(r['valid']):<8} "
#                 f"{r.get('trans_error', float('nan')):<14.4f} "
#                 f"{r.get('rot_error', float('nan')):<14.4f} "
#                 f"{r['fitness']:<10.4f} "
#                 f"{r['rmse']:<10.4f} "
#                 f"{r['overlap_est']:<10.4f}{tag}"
#             )
#     else:
#         print(f"  Overlap before ICP (init pose): {overlap_init:.2%}")

#         print(f"\n  {'Method':<25} {'Valid':<8} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 78)
#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(
#                 f"  {r['name']:<25} "
#                 f"{str(r['valid']):<8} "
#                 f"{r['fitness']:<10.4f} "
#                 f"{r['rmse']:<10.4f} "
#                 f"{r['overlap_est']:<10.4f}{tag}"
#             )

#     if best_key is None or results[best_key]["score"] <= -1e17:
#         print("\n  [ERROR] No valid registration result passed the overlap/fitness gate.")
#         best_T = None
#     else:
#         best_T = results[best_key]["transform"]
#         print(f"\n  Best method: {results[best_key]['name']}")
#         print_transform(best_T, "  Best estimated transform (scan2 -> scan1):")

#         if has_gt:
#             print_transform(T_gt_source_to_target, "  Ground truth transform (scan2 -> scan1):")
#             print(f"\n  Overlap after ESTIMATED alignment:    {results[best_key]['overlap_est']:.2%}")
#             print(f"  Overlap after GROUND-TRUTH alignment: {overlap_gt:.2%}")
#             print(f"  Overlap difference (est - gt):        {(results[best_key]['overlap_est'] - overlap_gt):.2%}")

#     if args.save_pose and best_T is not None:
#         np.savetxt(args.save_pose_name, best_T, fmt="%.6f")
#         print(f"\n  Saved estimated pose: {args.save_pose_name}")

#     # Visualization
#     if not args.no_vis and best_key is not None:
#         print("\n[7] Visualization...")
#         T_vis_gt = T_gt_source_to_target if has_gt else best_T

#         visualize_scans_2d(
#             scan1_proc, scan2_proc,
#             best_T, T_vis_gt,
#             save=args.save_plots
#         )

#         visualize_scans_3d(
#             scan1_proc, scan2_proc,
#             best_T, T_vis_gt
#         )

#     print("\nDone.")


# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3
"""
Real Radar Scan Matching Evaluation
===================================

Supports two input modes:

A) Nested frame folders (recommended for your dataset):
   scan_root/
      000000/
         cloud.pcd
         data
      ...

B) Two explicit scan files:
    python real_radar_scan_matching_eval.py \
        --scan1 frame_0001.pcd \
        --scan2 frame_0010.pcd \
        --gt_poses ground_truth.txt

Important:
- For ICP, source = scan2, target = scan1.
- Therefore GT alignment maps scan2 -> scan1.
"""

import argparse
import copy
import glob
import os
import re
import sys

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


# ============================================================
# 1. BASIC HELPERS
# ============================================================

def natural_sort_key(path):
    name = os.path.basename(path)
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split(r'(\d+)', name)]

def is_unix_timestamp(x):
    return x > 1e9

def load_pcd(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        sys.exit(1)
    pcd = o3d.io.read_point_cloud(filepath)
    print(f"  Loaded {filepath}: {len(pcd.points)} points")
    return pcd

def parse_numeric_tokens(line):
    tokens = re.split(r'[,\s]+', line.strip())
    vals = []
    for tok in tokens:
        if tok == "":
            continue
        try:
            vals.append(float(tok))
        except ValueError:
            continue
    return vals


# ============================================================
# 2. DATASET LAYOUT
# ============================================================

def get_frame_entries(scan_root, scan_name="cloud.pcd", meta_name="data"):
    if not os.path.isdir(scan_root):
        print(f"[ERROR] Scan root not found: {scan_root}")
        sys.exit(1)

    folders = sorted(
        [d for d in glob.glob(os.path.join(scan_root, "*")) if os.path.isdir(d)],
        key=natural_sort_key
    )

    entries = []
    for folder in folders:
        scan_path = os.path.join(folder, scan_name)
        meta_path = os.path.join(folder, meta_name)

        if os.path.exists(scan_path):
            entries.append({
                "folder": folder,
                "frame_name": os.path.basename(folder),
                "scan_path": scan_path,
                "meta_path": meta_path if os.path.exists(meta_path) else None
            })

    if len(entries) == 0:
        print(f"[ERROR] No frame folders containing {scan_name} found in: {scan_root}")
        sys.exit(1)

    return entries


def load_scan_from_entries(entries, idx):
    if idx < 0 or idx >= len(entries):
        print(f"[ERROR] Frame index out of range. Available: 0-{len(entries)-1}")
        sys.exit(1)

    entry = entries[idx]
    return load_pcd(entry["scan_path"]), entry


# ============================================================
# 3. TIMESTAMP PARSING
# ============================================================

def read_text_file(filepath):
    if filepath is None or not os.path.exists(filepath):
        return ""
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        return f.read()

def extract_timestamp_from_text(text):
    if not text:
        return None

    m = re.search(r'(?:timestamp|stamp)\D+(\d{10})\D+(\d{1,9})', text, re.IGNORECASE)
    if m:
        return int(m.group(1)) + int(m.group(2)) * 1e-9

    m = re.search(r'(?:timestamp|stamp)\D+(\d{10}\.\d+)', text, re.IGNORECASE)
    if m:
        return float(m.group(1))

    m = re.search(r'(\d{10}\.\d+)', text)
    if m:
        return float(m.group(1))

    m = re.search(r'(\d{10})\s+(\d{1,9})', text)
    if m:
        return int(m.group(1)) + int(m.group(2)) * 1e-9

    nums = parse_numeric_tokens(text)
    for v in nums:
        if is_unix_timestamp(v):
            return float(v)
    return None

def read_frame_timestamp(meta_path):
    text = read_text_file(meta_path)
    ts = extract_timestamp_from_text(text)
    if ts is None:
        raise ValueError(f"Could not parse frame timestamp from: {meta_path}")
    return ts


# ============================================================
# 4. POSE PARSING
# ============================================================

def pose_xyz_quat_to_T(x, y, z, qx, qy, qz, qw):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    return T

def pose_xyz_rpy_to_T(x, y, z, roll, pitch, yaw, degrees=False):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees).as_matrix()
    return T

def pose_xy_yaw_to_T(x, y, yaw, z=0.0, degrees=False):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    T[:3, :3] = Rotation.from_euler('z', yaw, degrees=degrees).as_matrix()
    return T

def parse_gt_row(values):
    n = len(values)
    if n == 8:
        return values[0], pose_xyz_quat_to_T(*values[1:8])
    if n == 7:
        if is_unix_timestamp(values[0]):
            return values[0], pose_xyz_rpy_to_T(*values[1:7], degrees=False)
        else:
            return None, pose_xyz_quat_to_T(*values[:7])
    if n == 6:
        return None, pose_xyz_rpy_to_T(*values[:6], degrees=False)
    if n == 5 and is_unix_timestamp(values[0]):
        return values[0], pose_xy_yaw_to_T(values[1], values[2], values[4], z=values[3], degrees=False)
    if n == 4:
        if is_unix_timestamp(values[0]):
            return values[0], pose_xy_yaw_to_T(values[1], values[2], values[3], z=0.0, degrees=False)
        else:
            return None, pose_xy_yaw_to_T(values[0], values[1], values[3], z=values[2], degrees=False)
    if n == 3:
        return None, pose_xy_yaw_to_T(values[0], values[1], values[2], z=0.0, degrees=False)
    raise ValueError(f"Unsupported pose row with {n} numeric values: {values}")

def load_ground_truth(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] Ground truth file not found: {filepath}")
        sys.exit(1)

    rows = []
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if line == "" or line.startswith("#"):
                continue
            vals = parse_numeric_tokens(line)
            if len(vals) == 0:
                continue
            try:
                ts, T = parse_gt_row(vals)
                rows.append({"timestamp": ts, "T": T, "raw": vals})
            except Exception:
                continue

    if len(rows) == 0:
        print(f"[ERROR] Could not parse any valid GT poses from: {filepath}")
        sys.exit(1)
    n_with_ts = sum(r["timestamp"] is not None for r in rows)
    print(f"  Loaded {len(rows)} ground truth poses from {filepath} ({n_with_ts} rows have timestamps)")
    return rows

def nearest_gt_index(frame_ts, gt_rows):
    gt_timestamps = np.array([
        np.nan if r["timestamp"] is None else float(r["timestamp"])
        for r in gt_rows
    ])
    valid = np.where(~np.isnan(gt_timestamps))[0]
    if len(valid) == 0:
        raise ValueError("GT file has no timestamps; cannot do timestamp-based matching.")
    return int(valid[np.argmin(np.abs(gt_timestamps[valid] - frame_ts))])


# ============================================================
# 5. TRANSFORMS / ERRORS
# ============================================================

def transform_from_A_to_B(T_A_world, T_B_world):
    return np.linalg.inv(T_A_world) @ T_B_world

def print_transform(T, label=""):
    t = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    dist = np.linalg.norm(t)
    print(f"  {label}")
    print(f"    Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
    print(f"    Rotation:    [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)")

def pose_error(T_est, T_gt):
    trans_error = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])
    R_err = T_est[:3, :3].T @ T_gt[:3, :3]
    val = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    rot_error_deg = np.degrees(np.arccos(val))
    return trans_error, rot_error_deg


# ============================================================
# 6. PREPROCESSING & OVERLAP
# ============================================================

def preprocess(pcd, voxel_size=0.1):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    if len(pcd_down.points) == 0:
        return pcd_down
    _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_clean = pcd_down.select_by_index(ind)
    print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
    return pcd_clean

def check_overlap(target_scan, source_scan, T_source_to_target, overlap_radius=1.0):
    src = copy.deepcopy(source_scan)
    src.transform(T_source_to_target)
    if len(target_scan.points) == 0 or len(src.points) == 0:
        return 0.0
    tree = o3d.geometry.KDTreeFlann(target_scan)
    count = 0
    for p in src.points:
        k, _, _ = tree.search_radius_vector_3d(p, overlap_radius)
        if k > 0: count += 1
    return count / len(src.points)

def overlap_label(overlap, threshold=0.30):
    return "OVERLAPPING" if overlap >= threshold else "NON-OVERLAPPING / WEAK OVERLAP"


# ============================================================
# 7. FPFH UTILITIES (EXACT SAC-IA MAPPING)
# ============================================================

def preprocess_for_fpfh(pcd, voxel_size=0.1, normal_radius=None, feature_radius=None,
                        normal_max_nn=30, feature_max_nn=100):
    """
    Prepare a cloud for FPFH. SAC-IA methodology step 1.
    """
    pcd_feat = copy.deepcopy(pcd)
    if len(pcd_feat.points) == 0:
        return pcd_feat, None

    if normal_radius is None: normal_radius = voxel_size * 2.0
    if feature_radius is None: feature_radius = voxel_size * 5.0

    pcd_feat.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn))
    try:
        pcd_feat.orient_normals_consistent_tangent_plane(10)
    except Exception:
        pass

    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_feat,
        o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=feature_max_nn)
    )
    return pcd_feat, fpfh


def plot_fpfh_bins_exact(pcd, voxel_size=0.1, point_idx=0, prefix="scan",
                         normal_radius=None, feature_radius=None,
                         normal_max_nn=30, feature_max_nn=100,
                         save=True, show=True):
    """
    Plot the EXACT FPFH descriptor of one selected point mapping correctly to Rusu (2009).
    """
    pcd_feat, fpfh = preprocess_for_fpfh(
        pcd, voxel_size=voxel_size, normal_radius=normal_radius,
        feature_radius=feature_radius, normal_max_nn=normal_max_nn, feature_max_nn=feature_max_nn
    )

    if fpfh is None:
        print(f"[WARNING] No FPFH descriptors available for {prefix}.")
        return

    fpfh_array = np.asarray(fpfh.data)
    if fpfh_array.ndim != 2 or fpfh_array.shape[0] != 33:
        print(f"[WARNING] Unexpected FPFH shape for {prefix}: {fpfh_array.shape}")
        return

    n_points = fpfh_array.shape[1]
    if n_points == 0:
        return
    if point_idx < 0 or point_idx >= n_points:
        print(f"[ERROR] point_idx={point_idx} out of range for {prefix}.")
        return

    desc = fpfh_array[:, point_idx]
    desc_norm = np.linalg.norm(desc)

    alpha_bins = desc[0:11]
    phi_bins   = desc[11:22]
    theta_bins = desc[22:33]

    print(f"  {prefix} FPFH shape: {fpfh_array.shape}")
    print(f"  Plotting EXACT FPFH for {prefix}, point index {point_idx}")
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    # CORRECTED LABELS MATCHING THE FPFH PAPER
    axes[0].bar(range(11), alpha_bins, color='steelblue', edgecolor='black', linewidth=0.5)
    axes[0].set_title(r'$\alpha$ (normal vs normal)')
    axes[0].set_xlabel('Bin')
    axes[0].set_ylabel('Value')
    axes[0].grid(True, alpha=0.3)

    axes[1].bar(range(11), phi_bins, color='coral', edgecolor='black', linewidth=0.5)
    axes[1].set_title(r'$\phi$ (normal vs connection vector)')
    axes[1].set_xlabel('Bin')
    axes[1].grid(True, alpha=0.3)

    axes[2].bar(range(11), theta_bins, color='seagreen', edgecolor='black', linewidth=0.5)
    axes[2].set_title(r'$\theta$ (rotation around axis)')
    axes[2].set_xlabel('Bin')
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(f'{prefix} | point {point_idx} | norm={desc_norm:.3f}', fontsize=13, fontweight='bold')
    plt.subplots_adjust(top=0.82, wspace=0.28)

    if save:
        out1 = f"{prefix}_point_{point_idx}_split.png"
        fig.savefig(out1, dpi=150, bbox_inches='tight')
        print(f"  Saved: {out1}")

    fig2, ax = plt.subplots(figsize=(16, 5))
    colors = ['steelblue'] * 11 + ['coral'] * 11 + ['seagreen'] * 11
    ax.bar(range(33), desc, color=colors, edgecolor='black', linewidth=0.5)
    ax.axvline(x=10.5, color='black', linestyle='--', linewidth=1.0)
    ax.axvline(x=21.5, color='black', linestyle='--', linewidth=1.0)
    ax.set_xlabel('Bin Index')
    ax.set_ylabel('Value')
    ax.set_title(f'Full FPFH Descriptor for {prefix}, point {point_idx}')
    ax.grid(True, alpha=0.3)

    ymax = max(np.max(desc), 1.0)
    ax.text(5,  ymax * 0.92, r'$\alpha$', ha='center', fontsize=16)
    ax.text(16, ymax * 0.92, r'$\phi$',   ha='center', fontsize=16)
    ax.text(27, ymax * 0.92, r'$\theta$', ha='center', fontsize=16)
    plt.subplots_adjust(top=0.88, bottom=0.15)

    if save:
        out2 = f"{prefix}_point_{point_idx}_full.png"
        fig2.savefig(out2, dpi=150, bbox_inches='tight')
        print(f"  Saved: {out2}")
    if show: plt.show()
    else:
        plt.close(fig)
        plt.close(fig2)


def run_fpfh_ransac_icp(source, target, voxel_size=0.1, max_dist=2.0,
                        normal_radius=None, feature_radius=None,
                        normal_max_nn=30, feature_max_nn=100,
                        ransac_distance_threshold=None,
                        ransac_n=4,
                        ransac_max_iteration=100000,
                        ransac_confidence=500):
    """
    SAC-IA Pipeline: FPFH computing -> Global RANSAC alignment -> Refinement
    """
    if normal_radius is None: normal_radius = voxel_size * 2.0
    if feature_radius is None: feature_radius = voxel_size * 5.0
    if ransac_distance_threshold is None: ransac_distance_threshold = voxel_size * 3.0

    source_feat_cloud, source_fpfh = preprocess_for_fpfh(
        source, voxel_size=voxel_size, normal_radius=normal_radius, feature_radius=feature_radius,
        normal_max_nn=normal_max_nn, feature_max_nn=feature_max_nn
    )
    target_feat_cloud, target_fpfh = preprocess_for_fpfh(
        target, voxel_size=voxel_size, normal_radius=normal_radius, feature_radius=feature_radius,
        normal_max_nn=normal_max_nn, feature_max_nn=feature_max_nn
    )

    if source_fpfh is None or target_fpfh is None:
        empty_result = o3d.pipelines.registration.RegistrationResult()
        empty_result.transformation = np.eye(4)
        return empty_result, empty_result

    # Sample Consensus Initial Alignment (SAC-IA)
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_feat_cloud, target_feat_cloud, source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=ransac_distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=ransac_n,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(ransac_distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(ransac_max_iteration, ransac_confidence)
    )

    src_icp = copy.deepcopy(source_feat_cloud)
    tgt_icp = copy.deepcopy(target_feat_cloud)

    result_icp = o3d.pipelines.registration.registration_icp(
        src_icp, tgt_icp, max_dist, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
    )
    return result_icp, result_ransac


# ============================================================
# 8. ICP + SCAN MATCHING (Point-to-Point / Point-to-Plane)
# ============================================================

def ensure_normals(pcd):
    if not pcd.has_normals():
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
    return o3d.pipelines.registration.registration_icp(
        source, target, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
    )

def run_icp_point_to_plane(source, target, init_T=np.eye(4), max_dist=2.0):
    ensure_normals(source)
    ensure_normals(target)
    return o3d.pipelines.registration.registration_icp(
        source, target, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
    )

def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
    ensure_normals(source)
    ensure_normals(target)
    return o3d.pipelines.registration.registration_generalized_icp(
        source, target, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
    )


# ============================================================
# 9. VISUALIZATION
# ============================================================

def visualize_scans_3d(scan1, scan2, T_est, T_gt):
    s1 = copy.deepcopy(scan1).paint_uniform_color([1, 0, 0])
    s2_est = copy.deepcopy(scan2).transform(T_est).paint_uniform_color([0, 1, 0])
    s2_gt = copy.deepcopy(scan2).transform(T_gt).paint_uniform_color([0, 0, 1])

    print("\n  3D Visualization colors:")
    print("    Red   = Reference scan (scan1)")
    print("    Green = Scan2 aligned by ESTIMATED pose")
    print("    Blue  = Scan2 aligned by GROUND TRUTH pose")

    o3d.visualization.draw_geometries([s1, s2_est, s2_gt], window_name="Scan Matching", width=1200, height=800)

def visualize_scans_2d(scan1, scan2, T_est, T_gt, save=False):
    pts1 = np.asarray(scan1.points)
    pts2 = np.asarray(scan2.points)
    pts2_est = np.asarray(copy.deepcopy(scan2).transform(T_est).points)
    pts2_gt = np.asarray(copy.deepcopy(scan2).transform(T_gt).points)

    fig1, ax1 = plt.subplots(figsize=(10, 8))
    ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
    ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c='tab:orange', label='Current scan')
    ax1.set_title('Before alignment'), ax1.set_aspect('equal'), ax1.grid(True, alpha=0.3), ax1.legend(loc='upper left', markerscale=5)

    fig2, ax2 = plt.subplots(figsize=(10, 8))
    ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
    ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:orange', label='Scan2 aligned by estimated pose')
    ax2.set_title('Estimated alignment'), ax2.set_aspect('equal'), ax2.grid(True, alpha=0.3), ax2.legend(loc='upper left', markerscale=5)

    fig3, ax3 = plt.subplots(figsize=(10, 8))
    ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
    ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:green', label='Scan2 aligned by GT pose')
    ax3.set_title('Ground-truth alignment'), ax3.set_aspect('equal'), ax3.grid(True, alpha=0.3), ax3.legend(loc='upper left', markerscale=5)

    if save:
        fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches='tight')
        fig2.savefig("plot_icp_aligned.png", dpi=150, bbox_inches='tight')
        fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches='tight')
    plt.show()


# ============================================================
# 10. ARGUMENT VALIDATION & MAIN
# ============================================================

def validate_inputs(args):
    using_root = args.scan_root is not None
    using_pair = (args.scan1 is not None) or (args.scan2 is not None)
    if using_root and using_pair:
        print("[ERROR] Use either --scan_root OR (--scan1 and --scan2), not both."); sys.exit(1)
    if not using_root and not using_pair:
        print("[ERROR] Provide either --scan_root OR (--scan1 and --scan2)."); sys.exit(1)
    if using_pair and (args.scan1 is None or args.scan2 is None):
        print("[ERROR] If using explicit file mode, both --scan1 and --scan2 are required."); sys.exit(1)
    if (args.pose1 is None) ^ (args.pose2 is None):
        print("[ERROR] Provide both --pose1 and --pose2 together."); sys.exit(1)
    if (args.stamp1 is None) ^ (args.stamp2 is None):
        print("[ERROR] Provide both --stamp1 and --stamp2 together."); sys.exit(1)
    if (args.data1 is None) ^ (args.data2 is None):
        print("[ERROR] Provide both --data1 and --data2 together."); sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="Evaluate scan matching between two real radar point clouds")
    parser.add_argument("--scan_root", "--scan_dir", dest="scan_root", default=None)
    parser.add_argument("--scan_name", default="cloud.pcd")
    parser.add_argument("--meta_name", default="data")

    parser.add_argument("--scan1", default=None)
    parser.add_argument("--scan2", default=None)
    parser.add_argument("--data1", default=None)
    parser.add_argument("--data2", default=None)
    parser.add_argument("--stamp1", type=float, default=None)
    parser.add_argument("--stamp2", type=float, default=None)

    parser.add_argument("--idx1", type=int, default=0)
    parser.add_argument("--idx2", type=int, default=1)

    parser.add_argument("--gt_poses", default=None)
    parser.add_argument("--pose1", default=None)
    parser.add_argument("--pose2", default=None)
    parser.add_argument("--use_gt_init", action="store_true")

    parser.add_argument("--voxel_size", type=float, default=0.1)
    parser.add_argument("--icp_max_dist", type=float, default=2.0)
    parser.add_argument("--overlap_radius", type=float, default=1.0)
    parser.add_argument("--overlap_threshold", type=float, default=0.30)
    parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "fpfh", "all"], default="all")
    
    parser.add_argument("--plot_fpfh", action="store_true")
    parser.add_argument("--fpfh_point_idx1", type=int, default=0)
    parser.add_argument("--fpfh_point_idx2", type=int, default=100)

    parser.add_argument("--no_vis", action="store_true")
    parser.add_argument("--save_plots", action="store_true")
    parser.add_argument("--save_pose", action="store_true")
    parser.add_argument("--save_pose_name", default="estimated_relative_pose.txt")

    args = parser.parse_args()
    validate_inputs(args)

    print("=" * 78)
    print("  REAL RADAR SCAN MATCHING EVALUATION (TIMESTAMP-AWARE)")
    print("=" * 78)

    # Loads scans
    print("\n[1] Loading scans...")
    frame_ts1 = frame_ts2 = gt_match_mode = None
    if args.scan_root is not None:
        entries = get_frame_entries(args.scan_root, args.scan_name, args.meta_name)
        scan1, entry1 = load_scan_from_entries(entries, args.idx1)
        scan2, entry2 = load_scan_from_entries(entries, args.idx2)
        scan1_path, scan2_path = entry1["scan_path"], entry2["scan_path"]
        if entry1["meta_path"]: frame_ts1 = read_frame_timestamp(entry1["meta_path"])
        if entry2["meta_path"]: frame_ts2 = read_frame_timestamp(entry2["meta_path"])
    else:
        scan1, scan2 = load_pcd(args.scan1), load_pcd(args.scan2)
        scan1_path, scan2_path = args.scan1, args.scan2
        if args.stamp1 and args.stamp2: frame_ts1, frame_ts2 = args.stamp1, args.stamp2
        elif args.data1 and args.data2: frame_ts1, frame_ts2 = read_frame_timestamp(args.data1), read_frame_timestamp(args.data2)

    if len(scan1.points) == 0 or len(scan2.points) == 0:
        print("[ERROR] One or both scans are empty."); sys.exit(1)

    print("\n[2] Resolving poses...")
    has_gt = False
    T1_world = T2_world = T_gt_source_to_target = matched_gt_idx1 = matched_gt_idx2 = None

    if args.pose1 and args.pose2:
        vals1, vals2 = [float(v) for v in args.pose1.split(',')], [float(v) for v in args.pose2.split(',')]
        vals1[3:], vals2[3:] = np.radians(vals1[3:]).tolist(), np.radians(vals2[3:]).tolist()
        T1_world, T2_world = pose_xyz_rpy_to_T(*vals1, degrees=False), pose_xyz_rpy_to_T(*vals2, degrees=False)
        has_gt, gt_match_mode = True, "manual poses"
    elif args.gt_poses:
        gt_rows = load_ground_truth(args.gt_poses)
        if frame_ts1 is not None and frame_ts2 is not None:
            try:
                matched_gt_idx1, matched_gt_idx2 = nearest_gt_index(frame_ts1, gt_rows), nearest_gt_index(frame_ts2, gt_rows)
                T1_world, T2_world = gt_rows[matched_gt_idx1]["T"], gt_rows[matched_gt_idx2]["T"]
                has_gt, gt_match_mode = True, "timestamp"
            except Exception:
                pass
        if not has_gt:
            matched_gt_idx1, matched_gt_idx2 = args.idx1, args.idx2
            T1_world, T2_world = gt_rows[matched_gt_idx1]["T"], gt_rows[matched_gt_idx2]["T"]
            has_gt, gt_match_mode = True, "index"

    if has_gt:
        T_gt_source_to_target = transform_from_A_to_B(T1_world, T2_world)

    print(f"\n[3] Preprocessing (voxel_size={args.voxel_size:.3f} m)...")
    scan1_proc, scan2_proc = preprocess(scan1, args.voxel_size), preprocess(scan2, args.voxel_size)

    if args.plot_fpfh:
        print("\n[3.5] Plotting EXACT FPFH descriptor bins...")
        plot_fpfh_bins_exact(scan1_proc, voxel_size=args.voxel_size, point_idx=args.fpfh_point_idx1, prefix=f"scan1_poseidx_{args.idx1}", save=args.save_plots)
        plot_fpfh_bins_exact(scan2_proc, voxel_size=args.voxel_size, point_idx=args.fpfh_point_idx2, prefix=f"scan2_poseidx_{args.idx2}", save=args.save_plots)

    print(f"\n[4] Checking overlap...")
    if has_gt:
        overlap_gt = check_overlap(scan1_proc, scan2_proc, T_gt_source_to_target, overlap_radius=args.overlap_radius)
        T_init = T_gt_source_to_target if args.use_gt_init else np.eye(4)
        overlap_init = check_overlap(scan1_proc, scan2_proc, T_init, overlap_radius=args.overlap_radius)
    else:
        overlap_gt = None
        T_init = np.eye(4)
        overlap_init = check_overlap(scan1_proc, scan2_proc, T_init, overlap_radius=args.overlap_radius)

    print(f"\n[5] Running scan matching...")
    methods = {
        "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
        "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
        "gicp":        ("Generalized ICP",    run_generalized_icp),
        "fpfh":        ("FPFH + RANSAC + ICP", run_fpfh_ransac_icp),
    }
    to_run = methods if args.method == "all" else {args.method: methods[args.method]}

    results = {}
    best_key, best_score = None, -1e18

    for key, (name, func) in to_run.items():
        print(f"\n  --- {name} ---")
        src, tgt = copy.deepcopy(scan2_proc), copy.deepcopy(scan1_proc)

        if key == "fpfh":
            result, result_ransac = func(src, tgt, voxel_size=args.voxel_size, max_dist=args.icp_max_dist)
        else:
            result = func(src, tgt, init_T=T_init, max_dist=args.icp_max_dist)
            result_ransac = None

        T_est = result.transformation
        eval_reg = o3d.pipelines.registration.evaluate_registration(src, tgt, args.icp_max_dist, T_est)
        fitness, rmse = eval_reg.fitness, eval_reg.inlier_rmse
        overlap_est = check_overlap(scan1_proc, scan2_proc, T_est, overlap_radius=args.overlap_radius)
        valid = (fitness > 1e-6) and (overlap_est >= args.overlap_threshold)

        entry = {"name": name, "transform": T_est, "fitness": fitness, "rmse": rmse, "overlap_est": overlap_est, "valid": valid}

        if has_gt:
            t_err, r_err = pose_error(T_est, T_gt_source_to_target)
            entry["trans_error"], entry["rot_error"] = t_err, r_err
            score = -(t_err + 0.02 * r_err) if valid else -1e18
        else:
            score = (2.0 * fitness + overlap_est - 0.1 * rmse) if valid else -1e18

        entry["score"] = score
        results[key] = entry
        if score > best_score:
            best_score, best_key = score, key

    print("\n" + "=" * 78)
    print("RESULTS SUMMARY")
    print("=" * 78)

    if has_gt:
        print(f"\n  {'Method':<25} {'Valid':<8} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
        print("  " + "-" * 108)
        for key, r in results.items():
            tag = " * best" if key == best_key else ""
            print(f"  {r['name']:<25} {str(r['valid']):<8} {r.get('trans_error', float('nan')):<14.4f} "
                  f"{r.get('rot_error', float('nan')):<14.4f} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")
    else:
        print(f"\n  {'Method':<25} {'Valid':<8} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
        print("  " + "-" * 78)
        for key, r in results.items():
            tag = " * best" if key == best_key else ""
            print(f"  {r['name']:<25} {str(r['valid']):<8} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")

    if best_key is None or results[best_key]["score"] <= -1e17:
        print("\n  [ERROR] No valid registration result passed.")
    else:
        best_T = results[best_key]["transform"]
        if args.save_pose:
            np.savetxt(args.save_pose_name, best_T, fmt="%.6f")

        if not args.no_vis:
            visualize_scans_2d(scan1_proc, scan2_proc, best_T, T_gt_source_to_target if has_gt else best_T, save=args.save_plots)
            visualize_scans_3d(scan1_proc, scan2_proc, best_T, T_gt_source_to_target if has_gt else best_T)

if __name__ == "__main__":
    main()
