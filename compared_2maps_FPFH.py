import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import pyvista as pv
def tum_line_to_matrix(line):
    """Convert a TUM format line to 4x4 transformation matrix"""
    parts = line.strip().split()
    tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
    qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
    
    R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T

def load_tum_trajectory(filepath):
    poses = {}
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            timestamp = float(line.split()[0])
            poses[timestamp] = tum_line_to_matrix(line)
    return poses

def preprocess(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2.0,
            max_nn=30
        )
    )

    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 5.0,
            max_nn=100
        )
    )
    return pcd_down, fpfh


def draw_result(source, target, T, title):
    src = copy.deepcopy(source)
    tgt = copy.deepcopy(target)

    src.paint_uniform_color([0, 1, 0])   # green
    tgt.paint_uniform_color([1, 0, 0])   # red

    src.transform(T)

    o3d.visualization.draw_geometries(
        [tgt, src],
        window_name=title
    )


voxel_size = 0.00001

source = o3d.io.read_point_cloud("bun000.ply")
target = o3d.io.read_point_cloud("bun_zipper.ply")
#Using Pyvista to read  and plot ply and point cloud data
source_pv = pv.read("bun000.ply")
print("Source data only")
source_pv.plot(eye_dome_lighting = True)



print("source points:", len(source.points), flush=True)
print("target points:", len(target.points), flush=True)

print("start preprocess", flush=True)
source_down, source_fpfh = preprocess(source, voxel_size)
target_down, target_fpfh = preprocess(target, voxel_size)
print("done preprocess", flush=True)

print("source down points:", len(source_down.points), flush=True)
print("target down points:", len(target_down.points), flush=True)
fpfh_array_source = np.asarray(source_fpfh.data)
fpfh_array_target = np.asarray(target_fpfh.data)

print("source FPFH shape:", np.asarray(source_fpfh.data).shape, flush=True)
print("target FPFH shape:", np.asarray(target_fpfh.data).shape, flush=True)
# Print descriptor every 1000 point (33x1 vector)
for i in range(0,fpfh_array_source.shape[1],2000): # using tuple 33 (0)    ,    65553(1)
    print(f"FPFH of source point {i}:", fpfh_array_source[:, i])
print ("\n========Example=======")
for i in range (0, fpfh_array_target.shape[1], 20000):
    print(f"\nFPFH of target point {i}", fpfh_array_target[:,i])
print("start correspondences", flush=True)
corres = o3d.pipelines.registration.correspondences_from_features(
    source_fpfh,
    target_fpfh,
    mutual_filter=True
)
corres_np = np.asarray(corres)
print("done correspondences:", corres_np.shape, flush=True)

for i in range(0,corres_np.shape[0],1000):
    src_idx = corres_np[i,0] # first column, idx start from 0 and first column is from 0 and second is from 1
    tgt_idx = corres_np[i,1] # second column
    print(f"FPFH of matched correspondence pair {i}:source point[{src_idx}]<-> target point[{tgt_idx}]")


distance_threshold = voxel_size * 3.0
# Load GT trajectory
poses = load_tum_trajectory("gt_odom_cp.txt")
timestamps = sorted(poses.keys())

# GT relative pose = from FIRST pose to LAST pose
# (adjust these if your two maps correspond to different timestamps)
T_first = poses[timestamps[0]]   # pose at start
T_last  = poses[timestamps[-1]]  # pose at end

# Relative pose: T_rel = inv(T_first) * T_last
T_gt_rel = np.linalg.inv(T_first) @ T_last

print("GT relative pose (first → last):")
print(T_gt_rel)


print("start ransac", flush=True)
result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    source_down,
    target_down,
    source_fpfh,
    target_fpfh,
    mutual_filter=True,
    max_correspondence_distance=distance_threshold,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    ransac_n=4,
    checkers=[
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
    ],
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 500)
)
print("done ransac", flush=True)

print("\n===== RANSAC result =====")
print("fitness:", result_ransac.fitness)
print("inlier_rmse:", result_ransac.inlier_rmse)
print("transformation:\n", result_ransac.transformation)

# draw_result(source, target, result_ransac.transformation, "FPFH RANSAC alignment")
src = copy.deepcopy(source)
tgt = copy.deepcopy(target)
src.paint_uniform_color([1, 0, 0])   # red = SOURCE ,output cp map
tgt.paint_uniform_color([0, 1, 0])   # green = TARGET
src.transform(result_ransac.transformation)
o3d.visualization.draw_geometries([tgt, src], window_name="FPFH RANSAC alignment")

# ICP refinement on original clouds
print("estimate normals for ICP", flush=True)
source.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)
target.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)

icp_threshold = voxel_size * 0.4

print("start ICP", flush=True)
result_icp = o3d.pipelines.registration.registration_icp(
    source,
    target,
    icp_threshold,
    result_ransac.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=300)
)
print("done ICP", flush=True)

print("\n===== ICP result =====")
print("fitness:", result_icp.fitness)
print("inlier_rmse:", result_icp.inlier_rmse)
print("transformation:\n", result_icp.transformation)
# ── Compare with your estimated result ──────────────────────
# Paste your result_icp.transformation here:
T_estimated = result_icp.transformation  # from your ICP code

# Error between GT and estimated
T_error = np.linalg.inv(T_gt_rel) @ T_estimated

# Translation error (metres)
t_err = np.linalg.norm(T_error[:3, 3])

# Rotation error (degrees)
cos_angle = (np.trace(T_error[:3, :3]) - 1) / 2
cos_angle = np.clip(cos_angle, -1, 1)
r_err = np.degrees(np.arccos(cos_angle))


draw_result(source, target, result_icp.transformation, "ICP refined alignment")
print("\n")
print("\n======Compared with GT file========")
print(f"\nTranslation error: {t_err:.4f} m")
print(f"Rotation error:    {r_err:.4f} deg")


# Pick a point index to visualize
point_idx = 0
fpfh_vector = fpfh_array_source[:, point_idx]  # 33-dim vector

# Split into three angular features (11 bins each)
alpha_bins = fpfh_vector[0:11]    # α: normal vs connection vector
phi_bins   = fpfh_vector[11:22]   # ϕ: normal vs normal
theta_bins = fpfh_vector[22:33]   # θ: rotation around connection axis

fig, axes = plt.subplots(1, 3, figsize=(15, 4))

# α histogram
axes[0].bar(range(11), alpha_bins, color='steelblue')
axes[0].set_title(r'$\alpha$ (normal vs connection vector)')
axes[0].set_xlabel('Bin')
axes[0].set_ylabel('Value')

# ϕ histogram
axes[1].bar(range(11), phi_bins, color='coral')
axes[1].set_title(r'$\phi$ (normal vs normal)')
axes[1].set_xlabel('Bin')

# θ histogram
axes[2].bar(range(11), theta_bins, color='seagreen')
axes[2].set_title(r'$\theta$ (rotation around axis)')
axes[2].set_xlabel('Bin')

fig.suptitle(f'FPFH Descriptor for Point {point_idx} (33 bins = 11 + 11 + 11)',
             fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig('fpfh_histogram.png', dpi=150)
plt.show()

# Plot the full 33-bin descriptor as one bar chart
fig2, ax = plt.subplots(figsize=(12, 4))
colors = ['steelblue']*11 + ['coral']*11 + ['seagreen']*11
ax.bar(range(33), fpfh_vector, color=colors)
ax.axvline(x=10.5, color='black', linestyle='--', linewidth=0.8)
ax.axvline(x=21.5, color='black', linestyle='--', linewidth=0.8)
ax.set_xlabel('Bin Index')
ax.set_ylabel('Value')
ax.set_title(f'Full FPFH Descriptor for Point {point_idx}')

# Label the three sections
ax.text(5,  max(fpfh_vector)*0.9, r'$\alpha$', ha='center', fontsize=14)
ax.text(16, max(fpfh_vector)*0.9, r'$\phi$',   ha='center', fontsize=14)
ax.text(27, max(fpfh_vector)*0.9, r'$\theta$', ha='center', fontsize=14)

plt.tight_layout()
plt.savefig('fpfh_full_histogram.png', dpi=150)
plt.show()