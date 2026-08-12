#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FPFH + pure Open3D RANSAC vs GT (no Arun refinement, no custom inlier checks).
source -> target
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
# 1. BASIC HELPERS (copied from your file, slightly trimmed)
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
    print(f" Loaded {filepath}: {len(pcd.points)} points")
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
        sec = int(m.group(1))
        nsec = int(m.group(2))
        return sec + nsec * 1e-9

    m = re.search(r'(?:timestamp|stamp)\D+(\d{10}\.\d+)', text, re.IGNORECASE)
    if m:
        return float(m.group(1))

    m = re.search(r'(\d{10}\.\d+)', text)
    if m:
        return float(m.group(1))

    m = re.search(r'(\d{10})\s+(\d{1,9})', text)
    if m:
        sec = int(m.group(1))
        nsec = int(m.group(2))
        return sec + nsec * 1e-9

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
# 4. POSE PARSING & GT
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
        ts = values[0]
        T = pose_xyz_quat_to_T(*values[1:8])
        return ts, T

    if n == 7:
        if is_unix_timestamp(values[0]):
            ts = values[0]
            T = pose_xyz_rpy_to_T(*values[1:7], degrees=False)
            return ts, T
        else:
            T = pose_xyz_quat_to_T(*values[:7])
            return None, T

    if n == 6:
        T = pose_xyz_rpy_to_T(*values[:6], degrees=False)
        return None, T

    if n == 5 and is_unix_timestamp(values[0]):
        ts = values[0]
        T = pose_xy_yaw_to_T(values[1], values[2], values[4], z=values[3], degrees=False)
        return ts, T

    if n == 4:
        if is_unix_timestamp(values[0]):
            ts = values[0]
            T = pose_xy_yaw_to_T(values[1], values[2], values[3], z=0.0, degrees=False)
            return ts, T
        else:
            T = pose_xy_yaw_to_T(values[0], values[1], values[3], z=values[2], degrees=False)
            return None, T

    if n == 3:
        T = pose_xy_yaw_to_T(values[0], values[1], values[2], z=0.0, degrees=False)
        return None, T

    raise ValueError(f"Unsupported pose row with {n} numeric values: {values}")

def load_ground_truth(filepath):
    print("[DEBUG] Entering load_ground_truth with:", filepath)

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

            # print("DEBUG GT vals:", vals)

            try:
                ts, T = parse_gt_row(vals)
                rows.append({"timestamp": ts, "T": T, "raw": vals})
            except Exception as e:
                print("  skipped line, error:", e)
                continue

    if len(rows) == 0:
        print(f"[ERROR] Could not parse any valid GT poses from: {filepath}")
        sys.exit(1)
    return rows
def nearest_gt_index(frame_ts, gt_rows):
    gt_timestamps = np.array([
        np.nan if r["timestamp"] is None else float(r["timestamp"])
        for r in gt_rows
    ])
    valid = np.where(~np.isnan(gt_timestamps))[0]
    if len(valid) == 0:
        raise ValueError("GT file has no timestamps; cannot do timestamp-based matching.")
    best = valid[np.argmin(np.abs(gt_timestamps[valid] - frame_ts))]
    return int(best)

def transform_from_A_to_B(T_A_world, T_B_world):
    return np.linalg.inv(T_A_world) @ T_B_world

def print_transform(T, label=""):
    t = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    dist = np.linalg.norm(t)
    print(f" {label}")
    print(f" Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
    print(f" Rotation: [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)")

def pose_error(T_est, T_gt):
    T_err = T_gt @ np.linalg.inv(T_est)
    trans_error = np.linalg.norm(T_err[:3, 3])
    R_err = T_err[:3, :3]
    val = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    rot_error_deg = np.degrees(np.arccos(val))
    return trans_error, rot_error_deg

# ============================================================
# 5. FPFH + PURE OPEN3D RANSAC
# ============================================================

def preprocess_for_fpfh(pcd, voxel_size=0.1, normal_radius=None, feature_radius=None,
                        normal_max_nn=30, feature_max_nn=100):
    pcd_down = copy.deepcopy(pcd).voxel_down_sample(voxel_size)
    if len(pcd_down.points) == 0:
        return pcd_down, None

    if normal_radius is None:
        normal_radius = voxel_size * 2.0
    if feature_radius is None:
        feature_radius = voxel_size * 5.0

    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn)
    )
    pcd_down.orient_normals_towards_camera_location(np.array([0.0, 0.0, 0.0]))

    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=feature_max_nn)
    )
    return pcd_down, fpfh

def run_pure_ransac(source, target,
                    voxel_size=0.1,
                    normal_radius=None,
                    feature_radius=None,
                    distance_threshold=None,
                    max_iterations=50000,
                    sample_size=3,
                    edge_ratio=0.9):
    """
    FPFH -> Open3D RANSAC (single call), no Arun, no custom inlier checks.
    source -> target
    """
    if distance_threshold is None:
        distance_threshold = voxel_size * 3.0

    source_down, source_fpfh = preprocess_for_fpfh(
        source, voxel_size=voxel_size,
        normal_radius=normal_radius,
        feature_radius=feature_radius,
    )
    target_down, target_fpfh = preprocess_for_fpfh(
        target, voxel_size=voxel_size,
        normal_radius=normal_radius,
        feature_radius=feature_radius,
    )

    if source_fpfh is None or target_fpfh is None:
        return None

    print(" Running Open3D RANSAC (pure, no refinement)...")
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=sample_size,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(edge_ratio),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(max_iterations, 0.999)
    )

    corr_pairs = np.asarray(ransac_result.correspondence_set, dtype=int)

    result = {
        "transformation": ransac_result.transformation,
        "fitness": float(ransac_result.fitness),
        "inlier_rmse": float(ransac_result.inlier_rmse),
        "corr_pairs": corr_pairs,
        "source_down": source_down,
        "target_down": target_down,
        "source_fpfh": source_fpfh,
        "target_fpfh": target_fpfh,
    }
    return result

# ============================================================
# 6. PLOT
# ============================================================

def plot_result_2d(scan1, scan2, T_est, T_gt=None, save_path=None, show=True):
    pts1 = np.asarray(scan1.points)
    pts2 = np.asarray(scan2.points)

    s2_est = copy.deepcopy(scan2)
    s2_est.transform(T_est)
    pts2_est = np.asarray(s2_est.points)

    if T_gt is not None:
        s2_gt = copy.deepcopy(scan2)
        s2_gt.transform(T_gt)
        pts2_gt = np.asarray(s2_gt.points)
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        ax1, ax2, ax3 = axes
    else:
        fig, axes = plt.subplots(1, 2, figsize=(12, 6))
        ax1, ax2 = axes
        pts2_gt = None

    ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Target')
    ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c='tab:orange', label='Source')
    ax1.set_title('Before alignment')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', markerscale=5)

    ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Target')
    ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:green', label='RANSAC aligned')
    ax2.set_title('RANSAC alignment')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left', markerscale=5)

    if pts2_gt is not None:
        ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Target')
        ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:red', label='GT aligned')
        ax3.set_title('GT alignment')
        ax3.set_aspect('equal')
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='upper left', markerscale=5)

    plt.tight_layout()
    if save_path is not None:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f" Saved plot: {save_path}")
    if show:
        plt.show()
    else:
        plt.close(fig)

# ============================================================
# 7. MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="FPFH + PURE Open3D RANSAC vs GT (no refinement)"
    )

    parser.add_argument("--scan_root", required=True,
                        help="Root folder containing frame folders like 000000/000001/...")
    parser.add_argument("--scan_name", default="cloud.pcd")
    parser.add_argument("--meta_name", default="data")

    parser.add_argument("--idx1", type=int, required=True,
                        help="Target frame index")
    parser.add_argument("--idx2", type=int, required=True,
                        help="Source frame index")

    parser.add_argument("--gt_poses", required=True,
                        help="GT odom file")

    parser.add_argument("--voxel_size", type=float, default=0.1)
    parser.add_argument("--normal_radius", type=float, default=None)
    parser.add_argument("--feature_radius", type=float, default=None)
    parser.add_argument("--distance_threshold", type=float, default=None)
    parser.add_argument("--max_iterations", type=int, default=50000)
    parser.add_argument("--sample_size", type=int, default=3)
    parser.add_argument("--edge_ratio", type=float, default=0.9)

    parser.add_argument("--result_dir", default="results_fpfh_pure_ransac")
    parser.add_argument("--plot_result", action="store_true")
    parser.add_argument("--plot_show", action="store_true")
    parser.add_argument("--plot_save", action="store_true")

    args = parser.parse_args()
    os.makedirs(args.result_dir, exist_ok=True)

    print("=" * 78)
    print(" FPFH + PURE OPEN3D RANSAC (NO REFINEMENT)")
    print("=" * 78)

    # [1] Load scans
    print("\n[1] Loading scans...")
    entries = get_frame_entries(args.scan_root, args.scan_name, args.meta_name)

    scan1, entry1 = load_scan_from_entries(entries, args.idx1)  # target
    scan2, entry2 = load_scan_from_entries(entries, args.idx2)  # source

    print(f" Target frame folder: {entry1['folder']}")
    print(f" Source frame folder: {entry2['folder']}")

    frame_ts1 = read_frame_timestamp(entry1["meta_path"]) if entry1["meta_path"] else None
    frame_ts2 = read_frame_timestamp(entry2["meta_path"]) if entry2["meta_path"] else None

    if frame_ts1 is not None:
        print(f" Target timestamp: {frame_ts1:.9f}")
    if frame_ts2 is not None:
        print(f" Source timestamp: {frame_ts2:.9f}")

    # [2] GT matching
    print("\n[2] Resolving GT odom...")
    gt_rows = load_ground_truth(args.gt_poses)

    if frame_ts1 is None or frame_ts2 is None:
        print("[ERROR] Missing timestamps in frame metadata, cannot do timestamp GT matching.")
        sys.exit(1)

    gt_idx1 = nearest_gt_index(frame_ts1, gt_rows)
    gt_idx2 = nearest_gt_index(frame_ts2, gt_rows)

    T1_world = gt_rows[gt_idx1]["T"]  # target pose in world
    T2_world = gt_rows[gt_idx2]["T"]  # source pose in world
    gt_ts1 = gt_rows[gt_idx1]["timestamp"]
    gt_ts2 = gt_rows[gt_idx2]["timestamp"]

    print(f" Frame1 -> GT row {gt_idx1}, GT timestamp {gt_ts1:.12f}, |dt| = {abs(frame_ts1 - gt_ts1):.9f}s")
    print(f" Frame2 -> GT row {gt_idx2}, GT timestamp {gt_ts2:.12f}, |dt| = {abs(frame_ts2 - gt_ts2):.9f}s")

    print_transform(T1_world, "Target pose in world:")
    print_transform(T2_world, "Source pose in world:")

    # source = scan2, target = scan1
    T_gt_source_to_target = transform_from_A_to_B(T1_world, T2_world)
    print_transform(T_gt_source_to_target, "GT transform (source -> target):")

    # [3] Pipeline: pure RANSAC
    print("\n[3] Running FPFH + pure Open3D RANSAC...")
    result = run_pure_ransac(
        source=scan2,
        target=scan1,
        voxel_size=args.voxel_size,
        normal_radius=args.normal_radius,
        feature_radius=args.feature_radius,
        distance_threshold=args.distance_threshold,
        max_iterations=args.max_iterations,
        sample_size=args.sample_size,
        edge_ratio=args.edge_ratio,
    )

    if result is None:
        print("[ERROR] RANSAC failed.")
        sys.exit(1)

    T_ransac = result["transformation"]
    print("\n[4] Pure Open3D RANSAC result")
    print(f" Num correspondences: {len(result['corr_pairs'])}")
    print(f" RANSAC fitness: {result['fitness']:.6f}")
    print(f" RANSAC inlier RMSE: {result['inlier_rmse']:.6f}")
    print_transform(T_ransac, "RANSAC transform (source -> target):")

    # [5] Compare with GT
    print("\n[5] Compare RANSAC with GT odom...")
    trans_error, rot_error = pose_error(T_ransac, T_gt_source_to_target)
    print(f" Translation error: {trans_error:.6f} m")
    print(f" Rotation error: {rot_error:.6f} deg")

    # [6] Save
    print("\n[6] Saving outputs...")
    np.savetxt(os.path.join(args.result_dir, "ransac_transform_pure.txt"), T_ransac, fmt="%.10f")
    np.savetxt(os.path.join(args.result_dir, "gt_transform.txt"), T_gt_source_to_target, fmt="%.10f")

    with open(os.path.join(args.result_dir, "summary_pure_ransac.txt"), "w", encoding="utf-8") as f:
        f.write(f"target_scan={entry1['scan_path']}\n")
        f.write(f"source_scan={entry2['scan_path']}\n")
        f.write(f"target_gt_row={gt_idx1}\n")
        f.write(f"source_gt_row={gt_idx2}\n")
        f.write(f"num_correspondences={len(result['corr_pairs'])}\n")
        f.write(f"ransac_fitness={result['fitness']:.10f}\n")
        f.write(f"ransac_inlier_rmse={result['inlier_rmse']:.10f}\n")
        f.write(f"translation_error_m={trans_error:.10f}\n")
        f.write(f"rotation_error_deg={rot_error:.10f}\n")

    print(" Saved RANSAC and GT transforms and summary.")

    # [7] Optional plot
    if args.plot_result:
        print("\n[7] Plotting RANSAC result...")
        save_path = os.path.join(args.result_dir, "alignment_ransac_only.png") if args.plot_save else None
        plot_result_2d(
            scan1=result["target_down"],
            scan2=result["source_down"],
            T_est=T_ransac,
            T_gt=T_gt_source_to_target,
            save_path=save_path,
            show=args.plot_show,
        )

    print("\nDone.")

if __name__ == "__main__":
    main()