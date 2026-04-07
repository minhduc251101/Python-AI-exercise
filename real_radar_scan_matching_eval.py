"""
Real Radar Scan Matching Evaluation
===================================
Loads two actual radar point clouds captured at two frames,
runs scan matching, and compares the estimated relative pose
against ground truth relative pose.

Two supported input modes:

1) Load scans by index from a directory:
    python real_radar_scan_matching_eval.py \
        --scan_dir ./radar_frames \
        --ext pcd \
        --idx1 0 --idx2 10 \
        --gt_poses ground_truth.csv

2) Load two explicit scan files:
    python real_radar_scan_matching_eval.py \
        --scan1 frame_0001.pcd \
        --scan2 frame_0010.pcd \
        --idx1 0 --idx2 10 \
        --gt_poses ground_truth.csv

You can also specify poses directly instead of a GT file:
    python real_radar_scan_matching_eval.py \
        --scan1 frame_0001.pcd \
        --scan2 frame_0010.pcd \
        --pose1 1.0,2.0,0.0,0,0,45 \
        --pose2 3.0,4.0,0.0,0,0,90

Ground truth CSV/TXT formats supported:
    timestamp, x, y, z, qx, qy, qz, qw
    timestamp, x, y, z, roll, pitch, yaw
    x, y, z, qx, qy, qz, qw
    x, y, z, roll, pitch, yaw
    x, y, z, yaw
    x, y, yaw
"""

import numpy as np
import open3d as o3d
import argparse
import os
import sys
import copy
import glob
import re
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


# ============================================================
# 1. LOADING FUNCTIONS
# ============================================================

def natural_sort_key(path):
    name = os.path.basename(path)
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split(r'(\d+)', name)]


def load_pcd(filepath):
    """Load a point cloud file and return an Open3D point cloud."""
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        sys.exit(1)

    pcd = o3d.io.read_point_cloud(filepath)
    print(f"  Loaded {filepath}: {len(pcd.points)} points")
    return pcd


def get_scan_file_list(scan_dir, ext="pcd"):
    """Get a naturally sorted list of scan files."""
    if not os.path.isdir(scan_dir):
        print(f"[ERROR] Scan directory not found: {scan_dir}")
        sys.exit(1)

    pattern = os.path.join(scan_dir, f"*.{ext}")
    files = glob.glob(pattern)
    files = sorted(files, key=natural_sort_key)

    if len(files) == 0:
        print(f"[ERROR] No '.{ext}' files found in: {scan_dir}")
        sys.exit(1)

    return files


def load_scan_by_index(scan_dir, idx, ext="pcd"):
    files = get_scan_file_list(scan_dir, ext)
    if idx < 0 or idx >= len(files):
        print(f"[ERROR] Scan index out of range. Available: 0-{len(files)-1}")
        sys.exit(1)
    filepath = files[idx]
    return load_pcd(filepath), filepath


def parse_pose_to_T(row):
    """
    Convert a pose row to a 4x4 homogeneous matrix.
    Handles:
      [x,y,z,qx,qy,qz,qw]
      [x,y,z,roll,pitch,yaw]   (radians)
    """
    T = np.eye(4)

    if len(row) == 7:
        T[:3, 3] = row[:3]
        T[:3, :3] = Rotation.from_quat(row[3:7]).as_matrix()
    elif len(row) == 6:
        T[:3, 3] = row[:3]
        T[:3, :3] = Rotation.from_euler('xyz', row[3:6]).as_matrix()
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

    for delim in [',', None]:
        try:
            data = np.loadtxt(filepath, delimiter=delim, comments='#')
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
                T[:3, :3] = Rotation.from_euler('xyz', row[4:7]).as_matrix()
            else:
                T = parse_pose_to_T(row[:7])
        elif ncols == 6:
            T = parse_pose_to_T(row[:6])
        elif ncols == 4:
            T[:3, 3] = [row[0], row[1], row[2]]
            T[:3, :3] = Rotation.from_euler('z', row[3]).as_matrix()
        elif ncols == 3:
            T[:3, 3] = [row[0], row[1], 0.0]
            T[:3, :3] = Rotation.from_euler('z', row[2]).as_matrix()
        else:
            print(f"[ERROR] Unexpected column count: {ncols}")
            sys.exit(1)

        poses.append(T)

    print(f"  Loaded {len(poses)} ground truth poses from {filepath}")
    return poses


# ============================================================
# 2. OVERLAP CHECK
# ============================================================

def check_overlap(scan1, scan2, T_relative, overlap_radius=1.0):
    """
    Check how much scan2 overlaps with scan1 after transforming scan2
    using T_relative.
    Returns overlap ratio (0.0 to 1.0).
    """
    scan2_transformed = copy.deepcopy(scan2)
    scan2_transformed.transform(T_relative)

    if len(scan1.points) == 0 or len(scan2_transformed.points) == 0:
        return 0.0

    tree = o3d.geometry.KDTreeFlann(scan1)

    count = 0
    for i in range(len(scan2_transformed.points)):
        [k, _, _] = tree.search_radius_vector_3d(
            scan2_transformed.points[i], overlap_radius
        )
        if k > 0:
            count += 1

    return count / len(scan2_transformed.points)


# ============================================================
# 3. SCAN MATCHING (ICP variants)
# ============================================================

def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
    """Point-to-point ICP."""
    return o3d.pipelines.registration.registration_icp(
        source, target, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )


def run_icp_point_to_plane(source, target, init_T=np.eye(4), max_dist=2.0):
    """Point-to-plane ICP (requires normals)."""
    for pcd in [source, target]:
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=1.0, max_nn=30
                )
            )

    return o3d.pipelines.registration.registration_icp(
        source, target, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )


def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
    """Generalized ICP."""
    for pcd in [source, target]:
        if not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=1.0, max_nn=30
                )
            )

    return o3d.pipelines.registration.registration_generalized_icp(
        source, target, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )


# ============================================================
# 4. EVALUATION
# ============================================================

def compute_relative_pose(T1, T2):
    """T_1->2 = inv(T1) @ T2"""
    return np.linalg.inv(T1) @ T2


def pose_error(T_est, T_gt):
    """Returns translation error (m) and rotation error (deg)."""
    trans_error = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])

    R_diff = T_est[:3, :3].T @ T_gt[:3, :3]
    trace_val = np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0)
    rot_error = np.degrees(np.arccos(trace_val))

    return trans_error, rot_error


def print_transform(T, label=""):
    """Pretty print a 4x4 transform."""
    t = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    dist = np.linalg.norm(t)

    print(f"  {label}")
    print(f"    Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
    print(f"    Rotation:    [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)")


def overlap_label(overlap, threshold=0.30):
    return "OVERLAPPING" if overlap >= threshold else "NON-OVERLAPPING / WEAK OVERLAP"


# ============================================================
# 5. VISUALIZATION
# ============================================================

def visualize_scans_3d(scan1, scan2, T_estimated, T_ground_truth):
    """
    3D Open3D interactive viewer:
      Red:   Reference scan (scan 1 / target)
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
        width=1200, height=800
    )


def visualize_scans_2d(scan1, scan2, T_estimated, T_ground_truth, save=False):
    """
    2D matplotlib scatter plots:
      Figure 1: Before alignment
      Figure 2: Estimated alignment
      Figure 3: Ground-truth alignment
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
    ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
    ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c='tab:orange', label='Current scan')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Two Real Radar Scans (before alignment)')
    ax1.legend(loc='upper left', markerscale=5)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    fig2, ax2 = plt.subplots(figsize=(10, 8))
    ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
    ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:orange', label='Current scan (estimated aligned)')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Scan Matching Result (estimated pose)')
    ax2.legend(loc='upper left', markerscale=5)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    fig3, ax3 = plt.subplots(figsize=(10, 8))
    ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
    ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:green', label='Current scan (ground-truth aligned)')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.set_title('Ground Truth Alignment')
    ax3.legend(loc='upper left', markerscale=5)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)

    if save:
        fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches='tight')
        fig2.savefig("plot_icp_aligned.png", dpi=150, bbox_inches='tight')
        fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches='tight')
        print("  Saved: plot_before_alignment.png, plot_icp_aligned.png, plot_gt_aligned.png")

    plt.show()


# ============================================================
# 6. PREPROCESSING
# ============================================================

def preprocess(pcd, voxel_size=0.1):
    """Downsample and remove outliers."""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_clean = pcd_down.select_by_index(ind)
    print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
    return pcd_clean


# ============================================================
# 7. ARGUMENT VALIDATION
# ============================================================

def validate_inputs(args):
    using_pair_files = (args.scan1 is not None) or (args.scan2 is not None)
    using_scan_dir = args.scan_dir is not None

    if using_pair_files and using_scan_dir:
        print("[ERROR] Use either (--scan1 and --scan2) OR --scan_dir, not both.")
        sys.exit(1)

    if not using_pair_files and not using_scan_dir:
        print("[ERROR] Provide either (--scan1 and --scan2) OR --scan_dir.")
        sys.exit(1)

    if using_pair_files and (args.scan1 is None or args.scan2 is None):
        print("[ERROR] If using direct file mode, both --scan1 and --scan2 are required.")
        sys.exit(1)

    if (args.pose1 is None) ^ (args.pose2 is None):
        print("[ERROR] Provide both --pose1 and --pose2 together.")
        sys.exit(1)


# ============================================================
# 8. MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Evaluate scan matching between two real radar point clouds"
    )

    # Input mode A: explicit files
    parser.add_argument("--scan1", default=None,
                        help="Path to radar scan 1 (reference / target)")
    parser.add_argument("--scan2", default=None,
                        help="Path to radar scan 2 (current / source)")

    # Input mode B: directory + indices
    parser.add_argument("--scan_dir", default=None,
                        help="Directory containing scan files")
    parser.add_argument("--ext", default="pcd",
                        help="Scan file extension in --scan_dir (default: pcd)")

    # Frame indices
    parser.add_argument("--idx1", type=int, default=0,
                        help="Frame index / GT pose index for scan 1")
    parser.add_argument("--idx2", type=int, default=1,
                        help="Frame index / GT pose index for scan 2")

    # GT or manual poses
    parser.add_argument("--gt_poses", default=None,
                        help="Ground truth poses CSV/TXT file")
    parser.add_argument("--pose1", default=None,
                        help="Manual pose 1: x,y,z,roll,pitch,yaw (degrees)")
    parser.add_argument("--pose2", default=None,
                        help="Manual pose 2: x,y,z,roll,pitch,yaw (degrees)")

    parser.add_argument("--use_gt_init", action="store_true",
                        help="Use GT relative pose as ICP initialization")

    # Processing and evaluation
    parser.add_argument("--voxel_size", type=float, default=0.1,
                        help="Voxel downsampling size (default: 0.1m)")
    parser.add_argument("--icp_max_dist", type=float, default=2.0,
                        help="ICP max correspondence distance (default: 2.0m)")
    parser.add_argument("--overlap_radius", type=float, default=1.0,
                        help="Radius for overlap check (default: 1.0m)")
    parser.add_argument("--overlap_threshold", type=float, default=0.30,
                        help="Threshold used to label pair as overlapping (default: 0.30)")
    parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "all"],
                        default="all", help="Registration method")

    # Output
    parser.add_argument("--no_vis", action="store_true",
                        help="Skip visualization")
    parser.add_argument("--save_plots", action="store_true",
                        help="Save matplotlib plots as PNG files")
    parser.add_argument("--save_pose", action="store_true",
                        help="Save best estimated relative pose to estimated_relative_pose.txt")

    args = parser.parse_args()
    validate_inputs(args)

    print("=" * 72)
    print("  REAL RADAR SCAN MATCHING EVALUATION")
    print("=" * 72)

    # --- Load scans ---
    print("\n[1] Loading radar scans...")
    if args.scan_dir is not None:
        scan1, scan1_path = load_scan_by_index(args.scan_dir, args.idx1, ext=args.ext)
        scan2, scan2_path = load_scan_by_index(args.scan_dir, args.idx2, ext=args.ext)
        print(f"  Scan 1 file: {scan1_path}")
        print(f"  Scan 2 file: {scan2_path}")
    else:
        scan1 = load_pcd(args.scan1)
        scan2 = load_pcd(args.scan2)
        scan1_path = args.scan1
        scan2_path = args.scan2

    if len(scan1.points) == 0 or len(scan2.points) == 0:
        print("\n[ERROR] One or both scans are empty.")
        sys.exit(1)

    # --- Get poses ---
    print("\n[2] Getting poses...")
    has_gt = False
    T_relative_gt = None
    gt_poses = None

    if args.pose1 and args.pose2:
        vals1 = [float(v) for v in args.pose1.split(',')]
        vals2 = [float(v) for v in args.pose2.split(',')]

        if len(vals1) != 6 or len(vals2) != 6:
            print("[ERROR] --pose1 and --pose2 must each have 6 values: x,y,z,roll,pitch,yaw")
            sys.exit(1)

        vals1[3:] = np.radians(vals1[3:]).tolist()
        vals2[3:] = np.radians(vals2[3:]).tolist()

        T1 = parse_pose_to_T(np.array(vals1))
        T2 = parse_pose_to_T(np.array(vals2))
        has_gt = True
        print("  Using manual poses")

    elif args.gt_poses:
        gt_poses = load_ground_truth(args.gt_poses)

        if args.idx1 >= len(gt_poses) or args.idx2 >= len(gt_poses):
            print(f"[ERROR] GT index out of range. Available: 0-{len(gt_poses)-1}")
            sys.exit(1)

        T1 = gt_poses[args.idx1]
        T2 = gt_poses[args.idx2]
        has_gt = True
        print(f"  Using GT poses at indices {args.idx1} and {args.idx2}")

        if args.scan_dir is not None:
            scan_files = get_scan_file_list(args.scan_dir, args.ext)
            print(f"  Scan files found: {len(scan_files)}")
            if len(scan_files) != len(gt_poses):
                print("  [WARNING] Number of scan files and GT poses differ.")
                print("  [WARNING] Index-based pairing is only valid if their ordering still matches.")

    else:
        print("  [WARNING] No GT or manual poses provided.")
        print("  [WARNING] Pose errors and GT-overlap comparison will be skipped.")

    if has_gt:
        print_transform(T1, "Pose 1 (world/map frame):")
        print_transform(T2, "Pose 2 (world/map frame):")

        pose_dist = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
        print(f"\n  Distance between poses: {pose_dist:.4f} m")

        T_relative_gt = compute_relative_pose(T1, T2)
        print_transform(T_relative_gt, "Ground truth relative pose (1 -> 2):")
    else:
        pose_dist = None

    # --- Preprocess ---
    print(f"\n[3] Preprocessing (voxel_size={args.voxel_size}m)...")
    print("  Scan 1:")
    scan1_proc = preprocess(scan1, args.voxel_size)
    print("  Scan 2:")
    scan2_proc = preprocess(scan2, args.voxel_size)

    if len(scan1_proc.points) == 0 or len(scan2_proc.points) == 0:
        print("\n[ERROR] One or both processed scans are empty.")
        sys.exit(1)

    # --- Overlap baseline ---
    print(f"\n[4] Checking overlap...")
    if has_gt:
        overlap_gt = check_overlap(
            scan1_proc, scan2_proc, T_relative_gt,
            overlap_radius=args.overlap_radius
        )
        print("  Using ground-truth relative pose for overlap check")
        print(f"  Overlap using GT pose: {overlap_gt:.2%}")
        print(f"  GT overlap decision:   {overlap_label(overlap_gt, args.overlap_threshold)}")

        T_init = T_relative_gt if args.use_gt_init else np.eye(4)
        overlap_init = check_overlap(
            scan1_proc, scan2_proc, T_init,
            overlap_radius=args.overlap_radius
        )
        init_name = "ground truth" if args.use_gt_init else "identity"
        print(f"  ICP initial transform: {init_name}")
        print(f"  Overlap using ICP init: {overlap_init:.2%}")
    else:
        overlap_gt = None
        T_init = np.eye(4)
        overlap_init = check_overlap(
            scan1_proc, scan2_proc, T_init,
            overlap_radius=args.overlap_radius
        )
        print("  No GT available, using identity init")
        print(f"  Overlap using ICP init: {overlap_init:.2%}")

    if has_gt and overlap_gt < args.overlap_threshold:
        print("  [WARNING] GT says this pair has weak overlap. Registration may fail even if code is correct.")
    elif overlap_init < args.overlap_threshold:
        print("  [WARNING] Initialization overlap is weak. ICP may be unreliable.")
    else:
        print("  Sufficient overlap for registration.")

    # --- Registration ---
    print(f"\n[5] Running scan matching...")
    methods = {
        "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
        "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
        "gicp":        ("Generalized ICP",    run_generalized_icp),
    }

    to_run = methods if args.method == "all" else {args.method: methods[args.method]}

    results = {}
    best_key = None
    best_fitness = -1.0

    for key, (name, func) in to_run.items():
        print(f"\n  --- {name} ---")

        src = copy.deepcopy(scan2_proc)
        tgt = copy.deepcopy(scan1_proc)

        result = func(src, tgt, init_T=T_init, max_dist=args.icp_max_dist)
        T_est = result.transformation

        overlap_est = check_overlap(
            scan1_proc, scan2_proc, T_est,
            overlap_radius=args.overlap_radius
        )

        print(f"    Fitness:           {result.fitness:.4f}")
        print(f"    Inlier RMSE:       {result.inlier_rmse:.4f}")
        print_transform(T_est, "Estimated relative pose:")
        print(f"    Overlap (est):     {overlap_est:.2%}")

        entry = {
            "name": name,
            "transform": T_est,
            "fitness": result.fitness,
            "rmse": result.inlier_rmse,
            "overlap_est": overlap_est,
        }

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

    print(f"\n  Scan 1: {scan1_path}")
    print(f"  Scan 2: {scan2_path}")

    if has_gt:
        print(f"  Distance between poses:          {pose_dist:.4f} m")
        print(f"  GT relative distance:            {np.linalg.norm(T_relative_gt[:3, 3]):.4f} m")
        print(f"  Overlap before ICP (GT pose):    {overlap_gt:.2%}")
        print(f"  Overlap before ICP (init pose):  {overlap_init:.2%}")
        print(f"  GT overlap label:                {overlap_label(overlap_gt, args.overlap_threshold)}")
        print(f"\n  {'Method':<25} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
        print("  " + "-" * 96)

        for key, r in results.items():
            tag = " * best" if key == best_key else ""
            print(f"  {r['name']:<25} {r['trans_error']:<14.4f} {r['rot_error']:<14.4f} "
                  f"{r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")
    else:
        print(f"  Overlap before ICP (init pose):  {overlap_init:.2%}")
        print(f"\n  {'Method':<25} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
        print("  " + "-" * 64)

        for key, r in results.items():
            tag = " * best" if key == best_key else ""
            print(f"  {r['name']:<25} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")

    best_T = results[best_key]["transform"]

    print(f"\n  Best method: {results[best_key]['name']}")
    print_transform(best_T, "Estimated relative pose:")

    if has_gt:
        print_transform(T_relative_gt, "Ground truth relative pose:")
        best_overlap_est = results[best_key]["overlap_est"]
        print(f"\n  Overlap after ESTIMATED alignment:    {best_overlap_est:.2%}")
        print(f"  Overlap after GROUND-TRUTH alignment: {overlap_gt:.2%}")
        print(f"  Overlap difference (est - gt):        {(best_overlap_est - overlap_gt):.2%}")

    # --- Save pose ---
    if args.save_pose:
        np.savetxt("estimated_relative_pose.txt", best_T, fmt="%.6f")
        print("\n  Saved estimated pose: estimated_relative_pose.txt")

    # --- Visualization ---
    if not args.no_vis and best_key:
        print("\n[6] Visualization...")
        T_vis_gt = T_relative_gt if has_gt else best_T
        visualize_scans_2d(
            scan1_proc, scan2_proc,
            best_T, T_vis_gt,
            save=args.save_plots
        )
        visualize_scans_3d(
            scan1_proc, scan2_proc,
            best_T, T_vis_gt
        )

    print("\nDone.")


if __name__ == "__main__":
    main()