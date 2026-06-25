#!/usr/bin/env python3
"""
verify_map_gt_coverage.py
=========================
Verify whether a global point-cloud map and a GT trajectory are spatially compatible.

Checks:
1. Load map point cloud from one PCD file.
2. Load GT poses from CSV/TXT.
3. Compute map axis-aligned bounds.
4. Check whether GT poses lie inside map bounds (with optional margin).
5. For selected indices, check how many map points exist within scan_radius.
6. Optionally check all GT poses and report coverage statistics.

Use this when you have:
- one global map file, e.g. cp_output_point_cloud_map.pcd
- one GT trajectory file, e.g. gt_odom_cp.txt

Example:
    python verify_map_gt_coverage.py \
        --map cp_output_point_cloud_map.pcd \
        --gt_poses gt_odom_cp.txt \
        --idx1 0 \
        --idx2 100 \
        --scan_radius 5.0 \
        --margin 0.0 \
        --check_all
"""

import os
import sys
import argparse
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


# ============================================================
# 1. LOADING
# ============================================================


def load_pcd(filepath):
    if not os.path.exists(filepath):
        print(f"[ERROR] File not found: {filepath}")
        sys.exit(1)

    pcd = o3d.io.read_point_cloud(filepath)
    print(f"  Loaded map: {filepath}")
    print(f"  Map points: {len(pcd.points)}")
    return pcd


def parse_pose_to_T(row):
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
            T[:3, 3] = [row[0], row[1], 0.0]
            T[:3, :3] = Rotation.from_euler("z", row[2]).as_matrix()
        else:
            print(f"[ERROR] Unexpected column count: {ncols}")
            sys.exit(1)

        poses.append(T)

    print(f"  Loaded {len(poses)} GT poses from {filepath}")
    return poses


# ============================================================
# 2. MAP / POSE CHECKS
# ============================================================


def point_inside_bounds(p, min_b, max_b, margin=0.0):
    return np.all(p >= (min_b - margin)) and np.all(p <= (max_b + margin))


def point_inside_bounds_xy(p, min_b, max_b, margin=0.0):
    return (min_b[0] - margin) <= p[0] <= (max_b[0] + margin) and (
        min_b[1] - margin
    ) <= p[1] <= (max_b[1] + margin)


def count_points_within_radius(tree, point, radius):
    k, _, _ = tree.search_radius_vector_3d(point, radius)
    return k


def summarize_pose(T, idx, min_b, max_b, tree, scan_radius, margin):
    p = T[:3, 3]
    inside_xyz = point_inside_bounds(p, min_b, max_b, margin)
    inside_xy = point_inside_bounds_xy(p, min_b, max_b, margin)
    nearby = count_points_within_radius(tree, p, scan_radius)

    print(f"\n  Pose idx {idx}")
    print(f"    Position: [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]")
    print(f"    Inside map bounds (XYZ): {inside_xyz}")
    print(f"    Inside map bounds (XY):  {inside_xy}")
    print(f"    Map points within scan_radius={scan_radius:.2f} m: {nearby}")

    if nearby == 0:
        print("    [WARNING] No nearby map points. Extracted scan would be empty.")
    elif nearby < 50:
        print("    [WARNING] Very few nearby map points. Extracted scan may be weak.")
    else:
        print("    Nearby map support looks reasonable.")

    return inside_xyz, inside_xy, nearby


# ============================================================
# 3. MAIN
# ============================================================


def main():
    parser = argparse.ArgumentParser(
        description="Verify compatibility between one global map PCD and GT trajectory"
    )
    parser.add_argument("--map", required=True, help="Global point cloud map PCD")
    parser.add_argument("--gt_poses", required=True, help="Ground truth poses CSV/TXT")
    parser.add_argument("--idx1", type=int, default=0, help="First GT index to inspect")
    parser.add_argument(
        "--idx2", type=int, default=100, help="Second GT index to inspect"
    )
    parser.add_argument(
        "--scan_radius",
        type=float,
        default=5.0,
        help="Radius used for local scan extraction check",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=0.0,
        help="Extra tolerance when checking if pose is inside map bounds",
    )
    parser.add_argument(
        "--check_all",
        action="store_true",
        help="Check all GT poses against map bounds and support",
    )
    args = parser.parse_args()

    print("=" * 72)
    print("  VERIFY MAP <-> GT COVERAGE")
    print("=" * 72)

    print("\n[1] Loading data...")
    pcd_map = load_pcd(args.map)
    gt_poses = load_ground_truth(args.gt_poses)

    if len(pcd_map.points) == 0:
        print("[ERROR] Map is empty.")
        sys.exit(1)

    if args.idx1 < 0 or args.idx1 >= len(gt_poses):
        print(f"[ERROR] idx1 out of range: 0-{len(gt_poses) - 1}")
        sys.exit(1)

    if args.idx2 < 0 or args.idx2 >= len(gt_poses):
        print(f"[ERROR] idx2 out of range: 0-{len(gt_poses) - 1}")
        sys.exit(1)

    print("\n[2] Computing map bounds...")
    aabb = pcd_map.get_axis_aligned_bounding_box()
    min_b = aabb.get_min_bound()
    max_b = aabb.get_max_bound()
    extent = max_b - min_b

    print(f"  Min bound: [{min_b[0]:.4f}, {min_b[1]:.4f}, {min_b[2]:.4f}]")
    print(f"  Max bound: [{max_b[0]:.4f}, {max_b[1]:.4f}, {max_b[2]:.4f}]")
    print(f"  Extent:    [{extent[0]:.4f}, {extent[1]:.4f}, {extent[2]:.4f}]")

    print("\n[3] Building KD-tree for neighborhood checks...")
    tree = o3d.geometry.KDTreeFlann(pcd_map)

    print("\n[4] Checking selected GT poses...")
    summarize_pose(
        gt_poses[args.idx1],
        args.idx1,
        min_b,
        max_b,
        tree,
        args.scan_radius,
        args.margin,
    )
    summarize_pose(
        gt_poses[args.idx2],
        args.idx2,
        min_b,
        max_b,
        tree,
        args.scan_radius,
        args.margin,
    )

    T1 = gt_poses[args.idx1]
    T2 = gt_poses[args.idx2]
    pose_dist = np.linalg.norm(T1[:3, 3] - T2[:3, 3])
    print(
        f"\n  Distance between idx {args.idx1} and idx {args.idx2}: {pose_dist:.4f} m"
    )

    if args.check_all:
        print("\n[5] Checking all GT poses...")
        inside_xyz_count = 0
        inside_xy_count = 0
        support_nonzero_count = 0
        support_good_count = 0

        for T in gt_poses:
            p = T[:3, 3]
            inside_xyz = point_inside_bounds(p, min_b, max_b, args.margin)
            inside_xy = point_inside_bounds_xy(p, min_b, max_b, args.margin)
            nearby = count_points_within_radius(tree, p, args.scan_radius)

            inside_xyz_count += int(inside_xyz)
            inside_xy_count += int(inside_xy)
            support_nonzero_count += int(nearby > 0)
            support_good_count += int(nearby >= 50)

        n = len(gt_poses)
        print(
            f"  GT poses inside map bounds (XYZ): {inside_xyz_count}/{n} = {inside_xyz_count / n:.2%}"
        )
        print(
            f"  GT poses inside map bounds (XY):  {inside_xy_count}/{n} = {inside_xy_count / n:.2%}"
        )
        print(
            f"  GT poses with >0 nearby map points within {args.scan_radius:.2f} m: {support_nonzero_count}/{n} = {support_nonzero_count / n:.2%}"
        )
        print(
            f"  GT poses with >=50 nearby map points within {args.scan_radius:.2f} m: {support_good_count}/{n} = {support_good_count / n:.2%}"
        )

    print("\nInterpretation:")
    print(
        "  - If a GT pose is outside map bounds, extracting a local scan there is suspicious."
    )
    print(
        "  - If a GT pose has 0 nearby map points within scan_radius, extracted scan will be empty."
    )
    print(
        "  - If both selected poses have nearby map support, your map-based scan extraction setup is plausible."
    )
    print(
        "  - This script does NOT prove frame-to-frame identity; it checks spatial compatibility between map and GT."
    )

    print("\nDone.")


if __name__ == "__main__":
    main()
