# """
# Scan Matching Evaluation from a Single PCD Map
# ==============================================
# Extracts two simulated scans from a full PCD map at given poses,
# checks overlap, runs scan matching (ICP / GICP / FPFH+RANSAC+ICP),
# and compares against ground truth.

# Usage:
#     python scan_matching_eval.py \
#         --map map.pcd \
#         --gt_poses ground_truth.csv \
#         --idx1 0 --idx2 10

#     # Or specify poses directly (x,y,z,roll,pitch,yaw in degrees):
#     python scan_matching_eval.py \
#         --map map.pcd \
#         --pose1 1.0,2.0,0.0,0,0,45 \
#         --pose2 3.0,4.0,0.0,0,0,90
# """

# import numpy as np
# import open3d as o3d
# import argparse
# import os
# import sys
# import copy
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation


# # ============================================================
# # 1. LOADING FUNCTIONS
# # ============================================================

# def load_pcd(filepath):
#     """Load a PCD file and return an Open3D point cloud."""
#     if not os.path.exists(filepath):
#         print(f"[ERROR] File not found: {filepath}")
#         sys.exit(1)
#     pcd = o3d.io.read_point_cloud(filepath)
#     print(f"  Loaded {filepath}: {len(pcd.points)} points")
#     return pcd


# def parse_pose_to_T(row):
#     """
#     Convert a pose row to a 4x4 homogeneous matrix.
#     Handles: [x,y,z,qx,qy,qz,qw] or [x,y,z,roll,pitch,yaw]
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
#             T[:3, 3] = [row[0], row[1], 0]
#             T[:3, :3] = Rotation.from_euler('z', row[2]).as_matrix()
#         else:
#             print(f"[ERROR] Unexpected column count: {ncols}")
#             sys.exit(1)

#         poses.append(T)

#     print(f"  Loaded {len(poses)} ground truth poses from {filepath}")
#     return poses


# # ============================================================
# # 2. SCAN EXTRACTION FROM MAP
# # ============================================================

# def extract_scan_from_map(pcd_map, pose_T, radius=30.0):
#     """
#     Extract a local scan from the full map at a given pose.
#     Crops points within `radius` meters, transforms to sensor local frame.
#     """
#     position = pose_T[:3, 3]
#     map_points = np.asarray(pcd_map.points)

#     distances = np.linalg.norm(map_points - position, axis=1)
#     mask = distances <= radius

#     if np.sum(mask) == 0:
#         print(f"  [WARNING] No points found within {radius}m of pose {position}. "
#               f"Try increasing --scan_radius.")
#         return o3d.geometry.PointCloud()

#     cropped_points = map_points[mask]

#     T_inv = np.linalg.inv(pose_T)
#     ones = np.ones((cropped_points.shape[0], 1))
#     points_homo = np.hstack([cropped_points, ones])
#     local_points = (T_inv @ points_homo.T).T[:, :3]

#     local_scan = o3d.geometry.PointCloud()
#     local_scan.points = o3d.utility.Vector3dVector(local_points)

#     if pcd_map.has_colors():
#         colors = np.asarray(pcd_map.colors)[mask]
#         local_scan.colors = o3d.utility.Vector3dVector(colors)

#     return local_scan


# # ============================================================
# # 3. OVERLAP CHECK
# # ============================================================

# def check_overlap(scan1, scan2, T_relative, search_radius=1.0):
#     """
#     Check how much two scans overlap when aligned using the relative pose.
#     Returns overlap ratio (0.0 to 1.0).
#     """
#     scan2_transformed = copy.deepcopy(scan2)
#     scan2_transformed.transform(T_relative)

#     if len(scan2_transformed.points) == 0 or len(scan1.points) == 0:
#         return 0.0

#     tree = o3d.geometry.KDTreeFlann(scan1)

#     count = 0
#     for i in range(len(scan2_transformed.points)):
#         k, _, _ = tree.search_radius_vector_3d(
#             scan2_transformed.points[i], search_radius
#         )
#         if k > 0:
#             count += 1

#     return count / len(scan2_transformed.points)


# # ============================================================
# # 4. SCAN MATCHING (ICP variants + FPFH)
# # ============================================================

# def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
#     """Point-to-point ICP."""
#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
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
#             max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
#         )
#     )


# def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
#     """Generalized ICP (plane-to-plane)."""
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
#             max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
#         )
#     )


# def preprocess_for_fpfh(pcd, voxel_size=0.1):
#     """
#     Prepare an already-cleaned/downsampled cloud for FPFH:
#     estimate normals and compute FPFH descriptors.
#     """
#     pcd_feat = copy.deepcopy(pcd)

#     if not pcd_feat.has_normals():
#         pcd_feat.estimate_normals(
#             o3d.geometry.KDTreeSearchParamHybrid(
#                 radius=voxel_size * 2.0,
#                 max_nn=30
#             )
#         )

#     fpfh = o3d.pipelines.registration.compute_fpfh_feature(
#         pcd_feat,
#         o3d.geometry.KDTreeSearchParamHybrid(
#             radius=voxel_size * 5.0,
#             max_nn=100
#         )
#     )

#     return pcd_feat, fpfh


# def run_fpfh_ransac_icp(source, target, voxel_size=0.1, max_dist=2.0):
#     """
#     FPFH feature matching -> global RANSAC -> ICP refinement.
#     Returns:
#         result_icp, result_ransac
#     """
#     source_feat, source_fpfh = preprocess_for_fpfh(source, voxel_size)
#     target_feat, target_fpfh = preprocess_for_fpfh(target, voxel_size)

#     distance_threshold = voxel_size * 3.0

#     result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#         source_feat,
#         target_feat,
#         source_fpfh,
#         target_fpfh,
#         mutual_filter=True,
#         max_correspondence_distance=distance_threshold,
#         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#         ransac_n=4,
#         checkers=[
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
#         ],
#         criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 500)
#     )

#     src_icp = copy.deepcopy(source)
#     tgt_icp = copy.deepcopy(target)

#     for pcd in [src_icp, tgt_icp]:
#         if not pcd.has_normals():
#             pcd.estimate_normals(
#                 search_param=o3d.geometry.KDTreeSearchParamHybrid(
#                     radius=1.0, max_nn=30
#                 )
#             )

#     result_icp = o3d.pipelines.registration.registration_icp(
#         src_icp,
#         tgt_icp,
#         max_dist,
#         result_ransac.transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(
#             max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
#         )
#     )

#     return result_icp, result_ransac


# # ============================================================
# # 5. EVALUATION
# # ============================================================

# def compute_relative_pose(T1, T2):
#     """T_1->2 = T1^{-1} * T2"""
#     return np.linalg.inv(T1) @ T2


# # def pose_error(T_est, T_gt):
# #     """
# #     Returns:
# #         trans_error (m), rot_error (deg)
# #     """
# #     trans_error = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])

# #     R_diff = T_est[:3, :3].T @ T_gt[:3, :3]
# #     trace_val = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
# #     rot_error = np.degrees(np.arccos(trace_val))

# #     return trans_error, rot_error

# def pose_error(T_est, T_gt):
#     """
#     MATLAB-style absolute pose error:
#         AbsoluteError_i = P_gt_i * inv(P_est_i)

#     Returns:
#         trans_error (m), rot_error (deg)
#     """
#     T_err = T_gt @ np.linalg.inv(T_est)

#     trans_error = np.linalg.norm(T_err[:3, 3])

#     trace_val = np.clip((np.trace(T_err[:3, :3]) - 1) / 2, -1.0, 1.0)
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


# # ============================================================
# # 6. VISUALIZATION
# # ============================================================

# def visualize_scans_3d(scan1, scan2, T_estimated, T_ground_truth):
#     """
#     3D Open3D interactive viewer:
#       Red:   Reference scan (scan 1)
#       Green: Scan 2 aligned by estimated pose (ICP result)
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
#       Figure 1: Reference scan vs Current scan (before alignment)
#       Figure 2: Reference scan vs Transformed current scan (after estimated alignment)
#       Figure 3: Reference scan vs Ground truth aligned scan
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
#     ax1.set_title('Two Scans (before alignment)')
#     ax1.legend(loc='upper left', markerscale=5)
#     ax1.set_aspect('equal')
#     ax1.grid(True, alpha=0.3)

#     fig2, ax2 = plt.subplots(figsize=(10, 8))
#     ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:orange',
#                 label='Transformed current scan (estimated)')
#     ax2.set_xlabel('X (m)')
#     ax2.set_ylabel('Y (m)')
#     ax2.set_title('Scan Matching Result (estimated alignment)')
#     ax2.legend(loc='upper left', markerscale=5)
#     ax2.set_aspect('equal')
#     ax2.grid(True, alpha=0.3)

#     fig3, ax3 = plt.subplots(figsize=(10, 8))
#     ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:green',
#                 label='Ground truth aligned scan')
#     ax3.set_xlabel('X (m)')
#     ax3.set_ylabel('Y (m)')
#     ax3.set_title('Ground Truth Alignment')
#     ax3.legend(loc='upper left', markerscale=5)
#     ax3.set_aspect('equal')
#     ax3.grid(True, alpha=0.3)

#     if save:
#         fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches='tight')
#         fig2.savefig("plot_est_aligned.png", dpi=150, bbox_inches='tight')
#         fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches='tight')
#         print("  Saved: plot_before_alignment.png, plot_est_aligned.png, plot_gt_aligned.png")

#     plt.show()


# # ============================================================
# # 7. PREPROCESSING
# # ============================================================

# def preprocess(pcd, voxel_size=0.1):
#     """Downsample and remove outliers."""
#     pcd_down = pcd.voxel_down_sample(voxel_size)
#     _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
#     pcd_clean = pcd_down.select_by_index(ind)
#     print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
#     return pcd_clean


# # ============================================================
# # 8. MAIN
# # ============================================================

# def main():
#     parser = argparse.ArgumentParser(
#         description="Extract scans from PCD map, run scan matching, compare vs ground truth"
#     )
#     parser.add_argument("--map", required=True,
#                         help="Path to the full PCD map file")
#     parser.add_argument("--gt_poses", default=None,
#                         help="Ground truth poses CSV file")
#     parser.add_argument("--idx1", type=int, default=0,
#                         help="Pose index for scan 1 (default: 0)")
#     parser.add_argument("--idx2", type=int, default=10,
#                         help="Pose index for scan 2 (default: 10)")
#     parser.add_argument("--pose1", default=None,
#                         help="Manual pose 1: x,y,z,roll,pitch,yaw (degrees)")
#     parser.add_argument("--pose2", default=None,
#                         help="Manual pose 2: x,y,z,roll,pitch,yaw (degrees)")
#     parser.add_argument("--scan_radius", type=float, default=5.0,
#                         help="Radius (m) to crop scan from map (default: 5m)")
#     parser.add_argument("--voxel_size", type=float, default=0.01,
#                         help="Voxel downsampling size (default: 0.01m)")
#     parser.add_argument("--icp_max_dist", type=float, default=2.0,
#                         help="ICP max correspondence distance (default: 2.0m)")
#     parser.add_argument("--overlap_radius", type=float, default=1.0,
#                         help="Radius to check point overlap (default: 1.0m)")
#     parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "fpfh", "all"],
#                         default="all", help="Scan matching method (default: all)")
#     parser.add_argument("--no_vis", action="store_true",
#                         help="Skip visualization")
#     parser.add_argument("--save_scans", action="store_true",
#                         help="Save extracted scans as PCD files")
#     parser.add_argument("--save_plots", action="store_true",
#                         help="Save matplotlib plots as PNG files")
#     parser.add_argument("--save_pose", action="store_true",
#                         help="Save estimated relative pose to estimated_relative_pose.txt")

#     args = parser.parse_args()

#     print("=" * 72)
#     print("  SCAN MATCHING EVALUATION (Extract from Map)")
#     print("=" * 72)

#     # --- Load map ---
#     print("\n[1] Loading PCD map...")
#     pcd_map = load_pcd(args.map)

#     # --- Get poses ---
#     print("\n[2] Getting poses...")
#     has_gt = False
#     T_relative_gt = None

#     if args.pose1 and args.pose2:
#         vals1 = [float(v) for v in args.pose1.split(',')]
#         vals2 = [float(v) for v in args.pose2.split(',')]
#         vals1[3:] = np.radians(vals1[3:]).tolist()
#         vals2[3:] = np.radians(vals2[3:]).tolist()
#         T1 = parse_pose_to_T(np.array(vals1))
#         T2 = parse_pose_to_T(np.array(vals2))
#         has_gt = True
#         print("  Using manual poses")
#     elif args.gt_poses:
#         has_gt = True
#         gt_poses = load_ground_truth(args.gt_poses)
#         if args.idx1 >= len(gt_poses) or args.idx2 >= len(gt_poses):
#             print(f"  [ERROR] Index out of range. Available: 0-{len(gt_poses)-1}")
#             sys.exit(1)
#         T1 = gt_poses[args.idx1]
#         T2 = gt_poses[args.idx2]
#         print(f"  GT pose count: {len(gt_poses)}")
#         print(f"  Using poses at indices {args.idx1} and {args.idx2}")
#     else:
#         print("  [ERROR] Provide either --gt_poses with --idx1/--idx2, or --pose1/--pose2")
#         sys.exit(1)

#     print_transform(T1, "Pose 1 (map frame):")
#     print_transform(T2, "Pose 2 (map frame):")

#     pose_dist = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
#     print(f"\n  Distance between poses: {pose_dist:.4f} m")

#     if has_gt:
#         T_relative_gt = compute_relative_pose(T1, T2)
#         print_transform(T_relative_gt, "Ground truth relative pose (1 -> 2):")

#     # --- Extract scans ---
#     print(f"\n[3] Extracting scans from map (radius={args.scan_radius}m)...")
#     scan1 = extract_scan_from_map(pcd_map, T1, radius=args.scan_radius)
#     print(f"  Scan 1: {len(scan1.points)} points")
#     scan2 = extract_scan_from_map(pcd_map, T2, radius=args.scan_radius)
#     print(f"  Scan 2: {len(scan2.points)} points")

#     if len(scan1.points) == 0 or len(scan2.points) == 0:
#         print("\n  [ERROR] One or both scans are empty. Check your poses and map.")
#         sys.exit(1)

#     if args.save_scans:
#         o3d.io.write_point_cloud("scan1_extracted.pcd", scan1)
#         o3d.io.write_point_cloud("scan2_extracted.pcd", scan2)
#         print("  Saved: scan1_extracted.pcd, scan2_extracted.pcd")

#     # --- Plot original scans overlaid (before any processing) ---
#     if not args.no_vis:
#         print(f"\n[3.5] Plotting original extracted scans (2D)...")
#         scan1_map = copy.deepcopy(scan1)
#         scan1_map.transform(T1)
#         scan2_map = copy.deepcopy(scan2)
#         scan2_map.transform(T2)

#         pts1_map = np.asarray(scan1_map.points)
#         pts2_map = np.asarray(scan2_map.points)

#         fig, ax = plt.subplots(figsize=(12, 9))
#         ax.scatter(pts1_map[:, 0], pts1_map[:, 1], s=1, c='tab:blue', label='Reference scan')
#         ax.scatter(pts2_map[:, 0], pts2_map[:, 1], s=1, c='tab:orange', label='Current scan')
#         ax.set_xlabel('X (m)')
#         ax.set_ylabel('Y (m)')
#         ax.set_title('Two Scans (before scan matching)')
#         ax.legend(loc='upper left', markerscale=5)
#         ax.set_aspect('equal')
#         ax.grid(True, alpha=0.3)

#         if args.save_plots:
#             fig.savefig("plot_original_scans.png", dpi=150, bbox_inches='tight')
#             print("  Saved: plot_original_scans.png")

#         plt.show()

#     # --- Preprocess ---
#     print(f"\n[4] Preprocessing (voxel_size={args.voxel_size}m)...")
#     print("  Scan 1:")
#     scan1_proc = preprocess(scan1, args.voxel_size)
#     print("  Scan 2:")
#     scan2_proc = preprocess(scan2, args.voxel_size)

#     if len(scan1_proc.points) == 0 or len(scan2_proc.points) == 0:
#         print("\n  [ERROR] One or both processed scans are empty.")
#         sys.exit(1)

#     # --- Overlap before registration ---
#     print(f"\n[5] Checking overlap...")
#     if has_gt:
#         print("  (using ground truth alignment)")
#         overlap_before = check_overlap(
#             scan1_proc, scan2_proc, T_relative_gt,
#             search_radius=args.overlap_radius
#         )
#     else:
#         T_init = compute_relative_pose(T1, T2)
#         print("  (using pose-based alignment)")
#         overlap_before = check_overlap(
#             scan1_proc, scan2_proc, T_init,
#             search_radius=args.overlap_radius
#         )

#     print(f"  Overlap before registration: {overlap_before:.2%}")
#     if overlap_before < 0.3:
#         print("  [WARNING] Low overlap - scan matching may be unreliable.")
#     else:
#         print("  Sufficient overlap for scan matching.")

#     # --- Scan matching ---
#     print(f"\n[6] Running scan matching...")
#     methods = {
#         "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
#         "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
#         "gicp":        ("Generalized ICP",    run_generalized_icp),
#         "fpfh":        ("FPFH + RANSAC + ICP", run_fpfh_ransac_icp),
#     }

#     to_run = methods if args.method == "all" else {args.method: methods[args.method]}

#     results = {}
#     best_key = None
#     best_fitness = -1.0

#     for key, (name, func) in to_run.items():
#         print(f"\n  --- {name} ---")
#         src = copy.deepcopy(scan2_proc)
#         tgt = copy.deepcopy(scan1_proc)

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
#             result = func(src, tgt, max_dist=args.icp_max_dist)
#             result_ransac = None

#         T_est = result.transformation

#         overlap_est = check_overlap(
#             scan1_proc, scan2_proc, T_est,
#             search_radius=args.overlap_radius
#         )

#         print(f"    Fitness:           {result.fitness:.4f}")
#         print(f"    Inlier RMSE:       {result.inlier_rmse:.4f}")
#         print_transform(T_est, "    Estimated relative pose:")
#         print(f"    Overlap (est):     {overlap_est:.2%}")

#         entry = {
#             "name": name,
#             "transform": T_est,
#             "fitness": result.fitness,
#             "rmse": result.inlier_rmse,
#             "overlap_est": overlap_est,
#         }

#         if result_ransac is not None:
#             entry["ransac_fitness"] = result_ransac.fitness
#             entry["ransac_rmse"] = result_ransac.inlier_rmse
#             entry["ransac_transform"] = result_ransac.transformation

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
#     print(f"\n  Scan extraction radius: {args.scan_radius}m")
#     print(f"  Distance between poses: {pose_dist:.4f}m")
#     print(f"  Overlap before registration: {overlap_before:.2%}")

#     if has_gt:
#         gt_dist = np.linalg.norm(T_relative_gt[:3, 3])
#         print(f"  GT relative distance:   {gt_dist:.4f}m")
#         print(f"\n  {'Method':<25} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} "
#               f"{'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 90)
#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(f"  {r['name']:<25} {r['trans_error']:<14.4f} {r['rot_error']:<14.4f} "
#                   f"{r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")
#     else:
#         print(f"\n  {'Method':<25} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 58)
#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(f"  {r['name']:<25} {r['fitness']:<10.4f} {r['rmse']:<10.4f} "
#                   f"{r['overlap_est']:<10.4f}{tag}")

#     print(f"\n  Best method: {results[best_key]['name']}")
#     print_transform(results[best_key]['transform'], "Estimated relative pose:")

#     if has_gt:
#         print_transform(T_relative_gt, "Ground truth relative pose:")

#         overlap_est_best = check_overlap(
#             scan1_proc, scan2_proc,
#             results[best_key]['transform'],
#             search_radius=args.overlap_radius
#         )
#         overlap_gt_best = check_overlap(
#             scan1_proc, scan2_proc,
#             T_relative_gt,
#             search_radius=args.overlap_radius
#         )

#         print(f"\n  Overlap after ESTIMATED alignment:    {overlap_est_best:.2%}")
#         print(f"  Overlap after GROUND-TRUTH alignment: {overlap_gt_best:.2%}")
#         print(f"  Overlap difference (est - gt):        {(overlap_est_best - overlap_gt_best):.2%}")

#     # --- Save estimated pose ---
#     best_T = results[best_key]['transform']
#     if args.save_pose:
#         np.savetxt("estimated_relative_pose.txt", best_T, fmt="%.6f")
#         print(f"\n  Saved estimated pose (4x4 matrix): estimated_relative_pose.txt")
#         print(f"  You can compare this with ground truth later.")

#     # --- Visualization ---
#     if not args.no_vis and best_key:
#         print(f"\n[7] Visualization...")
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


"""
Scan Matching Evaluation from a Single PCD Map
==============================================
Extracts two simulated scans from a full PCD map at given poses,
checks overlap, runs scan matching (ICP / GICP / FPFH+RANSAC+ICP),
compares against ground truth, and optionally plots FPFH descriptor bins.

Usage:
    python scan_matching_eval.py \
        --map map.pcd \
        --gt_poses ground_truth.csv \
        --idx1 0 --idx2 10

    # Or specify poses directly (x,y,z,roll,pitch,yaw in degrees):
    python scan_matching_eval.py \
        --map map.pcd \
        --pose1 1.0,2.0,0.0,0,0,45 \
        --pose2 3.0,4.0,0.0,0,0,90
"""

import numpy as np
import open3d as o3d
import argparse
import os
import sys
import copy
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


# ============================================================
# 1. LOADING FUNCTIONS
# ============================================================


def load_pcd(filepath):
    """Load a PCD file and return an Open3D point cloud."""
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        sys.exit(1)
    pcd = o3d.io.read_point_cloud(filepath)
    print(f"  Loaded {filepath}: {len(pcd.points)} points")
    return pcd


def parse_pose_to_T(row):
    """
    Convert a pose row to a 4x4 homogeneous matrix.
    Handles: [x,y,z,qx,qy,qz,qw] or [x,y,z,roll,pitch,yaw]
    """
    T = np.eye(4)
    if len(row) == 7:
        T[:3, 3] = row[:3]
        T[:3, :3] = Rotation.from_quat(row[3:7]).as_matrix()
    elif len(row) == 6:
        T[:3, 3] = row[:3]
        T[:3, :3] = Rotation.from_euler("xyz", row[3:6]).as_matrix()
    else:
        print(f"[ERROR] Cannot parse pose with {len(row)} values. Expected 6 or 7.")
        sys.exit(1)
    return T


def load_ground_truth(filepath):
    """
    Load ground truth poses from CSV/TXT.
    Auto-detects format based on number of columns.
    Returns: list of 4x4 homogeneous transformation matrices.
    """
    if not os.path.exists(filepath):
        print(f"[ERROR] Ground truth file not found: {filepath}")
        sys.exit(1)

    for delim in [",", None]:
        try:
            data = np.loadtxt(filepath, delimiter=delim, comments="#")
            if data.ndim == 1:
                data = data.reshape(1, -1)
            break
        except Exception:
            continue
    else:
        print(f"[ERROR] Could not parse {filepath}")
        sys.exit(1)

    poses = []
    ncols = data.shape[1]

    for row in data:
        T = np.eye(4)

        if ncols == 8:
            T = parse_pose_to_T(row[1:8])
        elif ncols == 7:
            if row[0] > 1e9:
                T[:3, 3] = row[1:4]
                T[:3, :3] = Rotation.from_euler("xyz", row[4:7]).as_matrix()
            else:
                T = parse_pose_to_T(row[:7])
        elif ncols == 6:
            T = parse_pose_to_T(row[:6])
        elif ncols == 4:
            T[:3, 3] = [row[0], row[1], row[2]]
            T[:3, :3] = Rotation.from_euler("z", row[3]).as_matrix()
        elif ncols == 3:
            T[:3, 3] = [row[0], row[1], 0]
            T[:3, :3] = Rotation.from_euler("z", row[2]).as_matrix()
        else:
            print(f"[ERROR] Unexpected column count: {ncols}")
            sys.exit(1)

        poses.append(T)

    print(f"  Loaded {len(poses)} ground truth poses from {filepath}")
    return poses


# ============================================================
# 2. SCAN EXTRACTION FROM MAP
# ============================================================


def extract_scan_from_map(pcd_map, pose_T, radius=30.0):
    """
    Extract a local scan from the full map at a given pose.
    Crops points within `radius` meters, transforms to sensor local frame.
    """
    position = pose_T[:3, 3]
    map_points = np.asarray(pcd_map.points)

    distances = np.linalg.norm(map_points - position, axis=1)
    mask = distances <= radius

    if np.sum(mask) == 0:
        print(
            f"  [WARNING] No points found within {radius}m of pose {position}. "
            f"Try increasing --scan_radius."
        )
        return o3d.geometry.PointCloud()

    cropped_points = map_points[mask]

    T_inv = np.linalg.inv(pose_T)
    ones = np.ones((cropped_points.shape[0], 1))
    points_homo = np.hstack([cropped_points, ones])
    local_points = (T_inv @ points_homo.T).T[:, :3]

    local_scan = o3d.geometry.PointCloud()
    local_scan.points = o3d.utility.Vector3dVector(local_points)

    if pcd_map.has_colors():
        colors = np.asarray(pcd_map.colors)[mask]
        local_scan.colors = o3d.utility.Vector3dVector(colors)

    return local_scan


# ============================================================
# 3. OVERLAP CHECK
# ============================================================


def check_overlap(scan1, scan2, T_relative, search_radius=1.0):
    """
    Check how much two scans overlap when aligned using the relative pose.
    Returns overlap ratio (0.0 to 1.0).
    """
    scan2_transformed = copy.deepcopy(scan2)
    scan2_transformed.transform(T_relative)

    if len(scan2_transformed.points) == 0 or len(scan1.points) == 0:
        return 0.0

    tree = o3d.geometry.KDTreeFlann(scan1)

    count = 0
    for i in range(len(scan2_transformed.points)):
        k, _, _ = tree.search_radius_vector_3d(
            scan2_transformed.points[i], search_radius
        )
        if k > 0:
            count += 1

    return count / len(scan2_transformed.points)


# ============================================================
# 4. SCAN MATCHING
# ============================================================


def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
    """Point-to-point ICP."""
    return o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_dist,
        init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
        ),
    )


def run_icp_point_to_plane(source, target, init_T=np.eye(4), max_dist=2.0):
    """Point-to-plane ICP (requires normals)."""
    for pcd in [source, target]:
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
            )
    return o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_dist,
        init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
        ),
    )


def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
    """Generalized ICP (plane-to-plane)."""
    for pcd in [source, target]:
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
            )
    return o3d.pipelines.registration.registration_generalized_icp(
        source,
        target,
        max_dist,
        init_T,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
        ),
    )


def preprocess_for_fpfh(pcd, voxel_size=0.1):
    """
    Prepare a cloud for FPFH:
    estimate normals and compute FPFH descriptors.
    """
    pcd_feat = copy.deepcopy(pcd)

    if not pcd_feat.has_normals():
        pcd_feat.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
        )

    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_feat,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0, max_nn=100),
    )

    return pcd_feat, fpfh


def run_fpfh_ransac_icp(source, target, voxel_size=0.1, max_dist=2.0):
    """
    FPFH feature matching -> global RANSAC -> ICP refinement.
    Returns:
        result_icp, result_ransac
    """
    source_feat, source_fpfh = preprocess_for_fpfh(source, voxel_size)
    target_feat, target_fpfh = preprocess_for_fpfh(target, voxel_size)

    distance_threshold = voxel_size * 3.0

    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_feat,
        target_feat,
        source_fpfh,
        target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
            False
        ),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold
            ),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 500),
    )

    src_icp = copy.deepcopy(source)
    tgt_icp = copy.deepcopy(target)

    for pcd in [src_icp, tgt_icp]:
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
            )

    result_icp = o3d.pipelines.registration.registration_icp(
        src_icp,
        tgt_icp,
        max_dist,
        result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6
        ),
    )

    return result_icp, result_ransac


# ============================================================
# 5. EVALUATION
# ============================================================


def compute_relative_pose(T1, T2):
    """T_1->2 = T1^{-1} * T2"""
    return np.linalg.inv(T1) @ T2


def pose_error(T_est, T_gt):
    """
    MATLAB-style absolute pose error:
        AbsoluteError_i = P_gt_i * inv(P_est_i)

    Returns:
        trans_error (m), rot_error (deg)
    """
    T_err = T_gt @ np.linalg.inv(T_est)

    trans_error = np.linalg.norm(T_err[:3, 3])

    trace_val = np.clip((np.trace(T_err[:3, :3]) - 1) / 2, -1.0, 1.0)
    rot_error = np.degrees(np.arccos(trace_val))

    return trans_error, rot_error


def print_transform(T, label=""):
    """Pretty print a 4x4 transform."""
    t = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    dist = np.linalg.norm(t)
    print(f"  {label}")
    print(f"    Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
    print(
        f"    Rotation:    [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)"
    )


# ============================================================
# 6. FPFH PLOTTING
# ============================================================


def plot_fpfh_bins(pcd, voxel_size=0.1, point_idx=0, prefix="scan"):
    """
    Plot FPFH descriptor bins for one point.
    FPFH has 33 bins = 11 alpha + 11 phi + 11 theta.
    """
    pcd_feat, fpfh = preprocess_for_fpfh(pcd, voxel_size)
    fpfh_array = np.asarray(fpfh.data)  # shape: (33, N)

    if fpfh_array.shape[1] == 0:
        print(f"  [WARNING] No FPFH descriptors available for {prefix}.")
        return

    if point_idx < 0 or point_idx >= fpfh_array.shape[1]:
        print(
            f"  [WARNING] point_idx={point_idx} out of range for {prefix}. "
            f"Using point 0 instead."
        )
        point_idx = 0

    desc = fpfh_array[:, point_idx]

    alpha_bins = desc[0:11]
    phi_bins = desc[11:22]
    theta_bins = desc[22:33]

    print(f"  {prefix} FPFH shape: {fpfh_array.shape}")
    print(f"  Plotting FPFH for {prefix}, point index {point_idx}")

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    axes[0].bar(range(11), alpha_bins, color="steelblue")
    axes[0].set_title(r"$\alpha$ (normal vs connection vector)")
    axes[0].set_xlabel("Bin")
    axes[0].set_ylabel("Value")

    axes[1].bar(range(11), phi_bins, color="coral")
    axes[1].set_title(r"$\phi$ (normal vs normal)")
    axes[1].set_xlabel("Bin")

    axes[2].bar(range(11), theta_bins, color="seagreen")
    axes[2].set_title(r"$\theta$ (rotation around axis)")
    axes[2].set_xlabel("Bin")

    fig.suptitle(
        f"FPFH Descriptor for {prefix}, point {point_idx} (33 bins = 11 + 11 + 11)",
        fontsize=14,
        fontweight="bold",
    )
    plt.tight_layout()
    fig.savefig(
        f"{prefix}_fpfh_histogram_point_{point_idx}.png", dpi=150, bbox_inches="tight"
    )
    print(f"  Saved: {prefix}_fpfh_histogram_point_{point_idx}.png")

    fig2, ax = plt.subplots(figsize=(12, 4))
    colors = ["steelblue"] * 11 + ["coral"] * 11 + ["seagreen"] * 11
    ax.bar(range(33), desc, color=colors)
    ax.axvline(x=10.5, color="black", linestyle="--", linewidth=0.8)
    ax.axvline(x=21.5, color="black", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Bin Index")
    ax.set_ylabel("Value")
    ax.set_title(f"Full FPFH Descriptor for {prefix}, point {point_idx}")

    ymax = max(desc) if np.max(desc) > 0 else 1.0
    ax.text(5, ymax * 0.9, r"$\alpha$", ha="center", fontsize=14)
    ax.text(16, ymax * 0.9, r"$\phi$", ha="center", fontsize=14)
    ax.text(27, ymax * 0.9, r"$\theta$", ha="center", fontsize=14)

    plt.tight_layout()
    fig2.savefig(
        f"{prefix}_fpfh_full_point_{point_idx}.png", dpi=150, bbox_inches="tight"
    )
    print(f"  Saved: {prefix}_fpfh_full_point_{point_idx}.png")

    plt.show()


# ============================================================
# 7. VISUALIZATION
# ============================================================


def visualize_scans_3d(scan1, scan2, T_estimated, T_ground_truth):
    """
    3D Open3D interactive viewer:
      Red:   Reference scan (scan 1)
      Green: Scan 2 aligned by estimated pose
      Blue:  Scan 2 aligned by ground truth pose
    """
    s1 = copy.deepcopy(scan1)
    s1.paint_uniform_color([1, 0, 0])

    s2_est = copy.deepcopy(scan2)
    s2_est.transform(T_estimated)
    s2_est.paint_uniform_color([0, 1, 0])

    s2_gt = copy.deepcopy(scan2)
    s2_gt.transform(T_ground_truth)
    s2_gt.paint_uniform_color([0, 0, 1])

    print("\n  3D Visualization colors:")
    print("    Red   = Reference scan (scan 1)")
    print("    Green = Scan 2 aligned by ESTIMATED pose")
    print("    Blue  = Scan 2 aligned by GROUND TRUTH pose")

    o3d.visualization.draw_geometries(
        [s1, s2_est, s2_gt],
        window_name="Scan Matching: Red=ref, Green=estimated, Blue=ground truth",
        width=1200,
        height=800,
    )


def visualize_scans_2d(scan1, scan2, T_estimated, T_ground_truth, save=False):
    """
    2D matplotlib scatter plots:
      Figure 1: Reference scan vs Current scan (before alignment)
      Figure 2: Reference scan vs Transformed current scan (after estimated alignment)
      Figure 3: Reference scan vs Ground truth aligned scan
    """
    pts1 = np.asarray(scan1.points)
    pts2 = np.asarray(scan2.points)

    s2_est = copy.deepcopy(scan2)
    s2_est.transform(T_estimated)
    pts2_est = np.asarray(s2_est.points)

    s2_gt = copy.deepcopy(scan2)
    s2_gt.transform(T_ground_truth)
    pts2_gt = np.asarray(s2_gt.points)

    fig1, ax1 = plt.subplots(figsize=(10, 8))
    ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c="tab:blue", label="Reference scan")
    ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c="tab:orange", label="Current scan")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Two Scans (before alignment)")
    ax1.legend(loc="upper left", markerscale=5)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)

    fig2, ax2 = plt.subplots(figsize=(10, 8))
    ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c="tab:blue", label="Reference scan")
    ax2.scatter(
        pts2_est[:, 0],
        pts2_est[:, 1],
        s=1,
        c="tab:orange",
        label="Transformed current scan (estimated)",
    )
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title("Scan Matching Result (estimated alignment)")
    ax2.legend(loc="upper left", markerscale=5)
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)

    fig3, ax3 = plt.subplots(figsize=(10, 8))
    ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c="tab:blue", label="Reference scan")
    ax3.scatter(
        pts2_gt[:, 0],
        pts2_gt[:, 1],
        s=1,
        c="tab:green",
        label="Ground truth aligned scan",
    )
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("Ground Truth Alignment")
    ax3.legend(loc="upper left", markerscale=5)
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)

    if save:
        fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches="tight")
        fig2.savefig("plot_est_aligned.png", dpi=150, bbox_inches="tight")
        fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches="tight")
        print(
            "  Saved: plot_before_alignment.png, plot_est_aligned.png, plot_gt_aligned.png"
        )

    plt.show()


# ============================================================
# 8. PREPROCESSING
# ============================================================


def preprocess(pcd, voxel_size=0.1):
    """Downsample and remove outliers."""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_clean = pcd_down.select_by_index(ind)
    print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
    return pcd_clean


# ============================================================
# 9. MAIN
# ============================================================


def main():
    parser = argparse.ArgumentParser(
        description="Extract scans from PCD map, run scan matching, compare vs ground truth"
    )
    parser.add_argument("--map", required=True, help="Path to the full PCD map file")
    parser.add_argument("--gt_poses", default=None, help="Ground truth poses CSV file")
    parser.add_argument(
        "--idx1", type=int, default=0, help="Pose index for scan 1 (default: 0)"
    )
    parser.add_argument(
        "--idx2", type=int, default=10, help="Pose index for scan 2 (default: 10)"
    )
    parser.add_argument(
        "--pose1", default=None, help="Manual pose 1: x,y,z,roll,pitch,yaw (degrees)"
    )
    parser.add_argument(
        "--pose2", default=None, help="Manual pose 2: x,y,z,roll,pitch,yaw (degrees)"
    )
    parser.add_argument(
        "--scan_radius",
        type=float,
        default=10.0,
        help="Radius (m) to crop scan from map (default: 10m)",
    )
    parser.add_argument(
        "--voxel_size",
        type=float,
        default=0.01,
        help="Voxel downsampling size (default: 0.01m)",
    )
    parser.add_argument(
        "--icp_max_dist",
        type=float,
        default=2.0,
        help="ICP max correspondence distance (default: 2.0m)",
    )
    parser.add_argument(
        "--overlap_radius",
        type=float,
        default=1.0,
        help="Radius to check point overlap (default: 1.0m)",
    )
    parser.add_argument(
        "--method",
        choices=["point2point", "point2plane", "gicp", "fpfh", "all"],
        default="all",
        help="Scan matching method (default: all)",
    )
    parser.add_argument(
        "--plot_fpfh",
        action="store_true",
        help="Plot FPFH descriptor bins for selected scan points",
    )
    parser.add_argument(
        "--fpfh_point_idx1",
        type=int,
        default=0,
        help="Point index for FPFH plotting in scan 1",
    )
    parser.add_argument(
        "--fpfh_point_idx2",
        type=int,
        default=100,
        help="Point index for FPFH plotting in scan 2",
    )
    parser.add_argument("--no_vis", action="store_true", help="Skip visualization")
    parser.add_argument(
        "--save_scans", action="store_true", help="Save extracted scans as PCD files"
    )
    parser.add_argument(
        "--save_plots", action="store_true", help="Save matplotlib plots as PNG files"
    )
    parser.add_argument(
        "--save_pose",
        action="store_true",
        help="Save estimated relative pose to estimated_relative_pose.txt",
    )

    args = parser.parse_args()

    print("=" * 72)
    print("  SCAN MATCHING EVALUATION (Extract from Map)")
    print("=" * 72)

    # --- Load map ---
    print("\n[1] Loading PCD map...")
    pcd_map = load_pcd(args.map)

    # --- Get poses ---
    print("\n[2] Getting poses...")
    has_gt = False
    T_relative_gt = None

    if args.pose1 and args.pose2:
        vals1 = [float(v) for v in args.pose1.split(",")]
        vals2 = [float(v) for v in args.pose2.split(",")]
        vals1[3:] = np.radians(vals1[3:]).tolist()
        vals2[3:] = np.radians(vals2[3:]).tolist()
        T1 = parse_pose_to_T(np.array(vals1))
        T2 = parse_pose_to_T(np.array(vals2))
        has_gt = True
        print("  Using manual poses")
    elif args.gt_poses:
        has_gt = True
        gt_poses = load_ground_truth(args.gt_poses)
        if args.idx1 >= len(gt_poses) or args.idx2 >= len(gt_poses):
            print(f"  [ERROR] Index out of range. Available: 0-{len(gt_poses) - 1}")
            sys.exit(1)
        T1 = gt_poses[args.idx1]
        T2 = gt_poses[args.idx2]
        print(f"  GT pose count: {len(gt_poses)}")
        print(f"  Using poses at indices {args.idx1} and {args.idx2}")
    else:
        print(
            "  [ERROR] Provide either --gt_poses with --idx1/--idx2, or --pose1/--pose2"
        )
        sys.exit(1)

    print_transform(T1, "Pose 1 (map frame):")
    print_transform(T2, "Pose 2 (map frame):")

    pose_dist = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
    print(f"\n  Distance between poses: {pose_dist:.4f} m")

    if has_gt:
        T_relative_gt = compute_relative_pose(T1, T2)
        print_transform(T_relative_gt, "Ground truth relative pose (1 -> 2):")

    # --- Extract scans ---
    print(f"\n[3] Extracting scans from map (radius={args.scan_radius}m)...")
    scan1 = extract_scan_from_map(pcd_map, T1, radius=args.scan_radius)
    print(f"  Scan 1: {len(scan1.points)} points")
    scan2 = extract_scan_from_map(pcd_map, T2, radius=args.scan_radius)
    print(f"  Scan 2: {len(scan2.points)} points")

    if len(scan1.points) == 0 or len(scan2.points) == 0:
        print("\n  [ERROR] One or both scans are empty. Check your poses and map.")
        sys.exit(1)

    if args.save_scans:
        o3d.io.write_point_cloud("scan1_extracted.pcd", scan1)
        o3d.io.write_point_cloud("scan2_extracted.pcd", scan2)
        print("  Saved: scan1_extracted.pcd, scan2_extracted.pcd")

    # --- Plot original scans overlaid ---
    if not args.no_vis:
        print(f"\n[3.5] Plotting original extracted scans (2D)...")
        scan1_map = copy.deepcopy(scan1)
        scan1_map.transform(T1)
        scan2_map = copy.deepcopy(scan2)
        scan2_map.transform(T2)

        pts1_map = np.asarray(scan1_map.points)
        pts2_map = np.asarray(scan2_map.points)

        fig, ax = plt.subplots(figsize=(12, 9))
        ax.scatter(
            pts1_map[:, 0], pts1_map[:, 1], s=1, c="tab:blue", label="Reference scan"
        )
        ax.scatter(
            pts2_map[:, 0], pts2_map[:, 1], s=1, c="tab:orange", label="Current scan"
        )
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Two Scans (before scan matching)")
        ax.legend(loc="upper left", markerscale=5)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

        if args.save_plots:
            fig.savefig("plot_original_scans.png", dpi=150, bbox_inches="tight")
            print("  Saved: plot_original_scans.png")

        plt.show()

    # --- Preprocess ---
    print(f"\n[4] Preprocessing (voxel_size={args.voxel_size}m)...")
    print("  Scan 1:")
    scan1_proc = preprocess(scan1, args.voxel_size)
    print("  Scan 2:")
    scan2_proc = preprocess(scan2, args.voxel_size)

    if len(scan1_proc.points) == 0 or len(scan2_proc.points) == 0:
        print("\n  [ERROR] One or both processed scans are empty.")
        sys.exit(1)

    # --- Optional FPFH plotting ---
    if args.plot_fpfh:
        print(f"\n[4.5] Plotting FPFH descriptor bins...")
        prefix1 = f"scan1_poseidx_{args.idx1}"
        prefix2 = f"scan2_poseidx_{args.idx2}"

        plot_fpfh_bins(
            scan1_proc,
            voxel_size=args.voxel_size,
            point_idx=args.fpfh_point_idx1,
            prefix=prefix1,
        )
        plot_fpfh_bins(
            scan2_proc,
            voxel_size=args.voxel_size,
            point_idx=args.fpfh_point_idx2,
            prefix=prefix2,
        )

    # --- Overlap before registration ---
    print(f"\n[5] Checking overlap...")
    if has_gt:
        print("  (using ground truth alignment)")
        overlap_before = check_overlap(
            scan1_proc, scan2_proc, T_relative_gt, search_radius=args.overlap_radius
        )
    else:
        T_init = compute_relative_pose(T1, T2)
        print("  (using pose-based alignment)")
        overlap_before = check_overlap(
            scan1_proc, scan2_proc, T_init, search_radius=args.overlap_radius
        )

    print(f"  Overlap before registration: {overlap_before:.2%}")
    if overlap_before < 0.3:
        print("  [WARNING] Low overlap - scan matching may be unreliable.")
    else:
        print("  Sufficient overlap for scan matching.")

    # --- Scan matching ---
    print(f"\n[6] Running scan matching...")
    methods = {
        "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
        "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
        "gicp": ("Generalized ICP", run_generalized_icp),
        "fpfh": ("FPFH + RANSAC + ICP", run_fpfh_ransac_icp),
    }

    to_run = methods if args.method == "all" else {args.method: methods[args.method]}

    results = {}
    best_key = None
    best_fitness = -1.0

    for key, (name, func) in to_run.items():
        print(f"\n  --- {name} ---")
        src = copy.deepcopy(scan2_proc)
        tgt = copy.deepcopy(scan1_proc)

        if key == "fpfh":
            result, result_ransac = func(
                src, tgt, voxel_size=args.voxel_size, max_dist=args.icp_max_dist
            )

            print(f"    RANSAC fitness:     {result_ransac.fitness:.4f}")
            print(f"    RANSAC inlier RMSE: {result_ransac.inlier_rmse:.4f}")
            print_transform(result_ransac.transformation, "    RANSAC initial pose:")
        else:
            result = func(src, tgt, max_dist=args.icp_max_dist)
            result_ransac = None

        T_est = result.transformation

        overlap_est = check_overlap(
            scan1_proc, scan2_proc, T_est, search_radius=args.overlap_radius
        )

        print(f"    Fitness:           {result.fitness:.4f}")
        print(f"    Inlier RMSE:       {result.inlier_rmse:.4f}")
        print_transform(T_est, "    Estimated relative pose:")
        print(f"    Overlap (est):     {overlap_est:.2%}")

        entry = {
            "name": name,
            "transform": T_est,
            "fitness": result.fitness,
            "rmse": result.inlier_rmse,
            "overlap_est": overlap_est,
        }

        if result_ransac is not None:
            entry["ransac_fitness"] = result_ransac.fitness
            entry["ransac_rmse"] = result_ransac.inlier_rmse
            entry["ransac_transform"] = result_ransac.transformation

        if has_gt:
            t_err, r_err = pose_error(T_est, T_relative_gt)
            print(f"    Translation error: {t_err:.4f} m")
            print(f"    Rotation error:    {r_err:.4f} deg")
            entry["trans_error"] = t_err
            entry["rot_error"] = r_err

        results[key] = entry

        if result.fitness > best_fitness:
            best_fitness = result.fitness
            best_key = key

    # --- Summary ---
    print("\n" + "=" * 72)
    print("  RESULTS SUMMARY")
    print("=" * 72)
    print(f"\n  Scan extraction radius: {args.scan_radius}m")
    print(f"  Distance between poses: {pose_dist:.4f}m")
    print(f"  Overlap before registration: {overlap_before:.2%}")

    if has_gt:
        gt_dist = np.linalg.norm(T_relative_gt[:3, 3])
        print(f"  GT relative distance:   {gt_dist:.4f}m")
        print(
            f"\n  {'Method':<25} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} "
            f"{'Fitness':<10} {'RMSE':<10} {'Overlap':<10}"
        )
        print("  " + "-" * 90)
        for key, r in results.items():
            tag = " * best" if key == best_key else ""
            print(
                f"  {r['name']:<25} {r['trans_error']:<14.4f} {r['rot_error']:<14.4f} "
                f"{r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}"
            )
    else:
        print(f"\n  {'Method':<25} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
        print("  " + "-" * 58)
        for key, r in results.items():
            tag = " * best" if key == best_key else ""
            print(
                f"  {r['name']:<25} {r['fitness']:<10.4f} {r['rmse']:<10.4f} "
                f"{r['overlap_est']:<10.4f}{tag}"
            )

    print(f"\n  Best method: {results[best_key]['name']}")
    print_transform(results[best_key]["transform"], "Estimated relative pose:")

    if has_gt:
        print_transform(T_relative_gt, "Ground truth relative pose:")

        overlap_est_best = check_overlap(
            scan1_proc,
            scan2_proc,
            results[best_key]["transform"],
            search_radius=args.overlap_radius,
        )
        overlap_gt_best = check_overlap(
            scan1_proc, scan2_proc, T_relative_gt, search_radius=args.overlap_radius
        )

        print(f"\n  Overlap after ESTIMATED alignment:    {overlap_est_best:.2%}")
        print(f"  Overlap after GROUND-TRUTH alignment: {overlap_gt_best:.2%}")
        print(
            f"  Overlap difference (est - gt):        {(overlap_est_best - overlap_gt_best):.2%}"
        )

    # --- Save estimated pose ---
    best_T = results[best_key]["transform"]
    if args.save_pose:
        np.savetxt("estimated_relative_pose.txt", best_T, fmt="%.6f")
        print(f"\n  Saved estimated pose (4x4 matrix): estimated_relative_pose.txt")
        print("  You can compare this with ground truth later.")

    # --- Visualization ---
    if not args.no_vis and best_key:
        print(f"\n[7] Visualization...")
        T_vis_gt = T_relative_gt if has_gt else best_T
        visualize_scans_2d(
            scan1_proc, scan2_proc, best_T, T_vis_gt, save=args.save_plots
        )
        visualize_scans_3d(scan1_proc, scan2_proc, best_T, T_vis_gt)

    print("\nDone.")


if __name__ == "__main__":
    main()
