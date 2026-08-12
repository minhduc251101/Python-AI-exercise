# import numpy as np
# import open3d as o3d

# # 读取点云
# pcd1 = o3d.io.read_point_cloud("final_map_fixed_0.01.pcd")
# pcd2 = o3d.io.read_point_cloud("bun000.ply")

# print("Original number of points in pcd1:", len(pcd1.points))
# print("Original number of points in pcd2:", len(pcd2.points))

# # 设置颜色区分两个点云
# pcd1.paint_uniform_color([1, 0, 0])   # 红
# pcd2.paint_uniform_color([0, 0, 1])   # 蓝

# # 可视化
# o3d.visualization.draw_geometries([pcd1, pcd2])

# voxel_size = 0.005   # 可调：0.005 ~ 0.01

# # 法向量估计
# pcd1.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
# )
# pcd2.estimate_normals(
#     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
# )

# # 计算 FPFH 特征
# fpfh1 = o3d.pipelines.registration.compute_fpfh_feature(
#     pcd1,
#     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
# )
# fpfh2 = o3d.pipelines.registration.compute_fpfh_feature(
#     pcd2,
#     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100)
# )

# print("FPFH feature size for pcd1:", fpfh1.data.shape)
# print("FPFH feature size for pcd2:", fpfh2.data.shape)

# # 基于 FPFH 的 RANSAC 特征匹配
# # distance_threshold = voxel_size * 1.5

# # result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
# #     source=pcd2,         # source
# #     target=pcd1,         # target
# #     source_feature=fpfh2,
# #     target_feature=fpfh1,
# #     mutual_filter=True,
# #     max_correspondence_distance=distance_threshold,
# #     estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
# #     ransac_n=4,
# #     checkers=[
# #         o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
# #         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
# #     ],
# #     criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 500)
# # )
# # print("\n===== RANSAC Matching Result =====")
# # print("Fitness:", result.fitness)
# # print("Inlier RMSE:", result.inlier_rmse)
# # print("Estimated transformation:\n", result.transformation)

# # 找潜在的匹配对
# corres = o3d.pipelines.registration.correspondences_from_features(
#     fpfh2,
#     fpfh1,
#     mutual_filter=True
# )

# corres_np = np.asarray(corres)
# print(corres_np.shape)   # (K, 2)
# print(corres_np[:10])    # 前10对匹配 [source_idx, target_idx]

# # 用 corres_np 中的所有对应点，通过 SVD求解 R, t

# # 取原始点坐标
# src_pts = np.asarray(pcd2.points)   # source
# tgt_pts = np.asarray(pcd1.points)   # target

# # 根据 correspondence 取匹配点
# P = src_pts[corres_np[:, 0], :]   # source matched points, shape = (K, 3)
# Q = tgt_pts[corres_np[:, 1], :]   # target matched points, shape = (K, 3)
# print("Matched source points shape:", P.shape)
# print("Matched target points shape:", Q.shape)

# # --------------------------------------------------------------------

# # 把 P 和 Q 转成 point cloud
# pcd_P = o3d.geometry.PointCloud()
# pcd_Q = o3d.geometry.PointCloud()

# pcd_P.points = o3d.utility.Vector3dVector(P)
# pcd_Q.points = o3d.utility.Vector3dVector(Q)

# # 设置不同颜色方便区分
# pcd_P.paint_uniform_color([0, 0, 1])   # 蓝色
# pcd_Q.paint_uniform_color([1, 0, 0])   # 红色

# # 可视化
# o3d.visualization.draw_geometries([pcd_P, pcd_Q])

# # --------------------------------------------------------------------

# # 拼成 [src_xyz , tgt_xyz]
# # data = np.hstack((P, Q))
# # # 保存 CSV
# # np.savetxt(
# #     "potential_correspondences.csv",
# #     data,
# #     delimiter=","
# # )
# # print("Saved potential_correspondences.csv")

# # 取对应点编号
# src_ids = corres_np[:, 0].reshape(-1,1)
# tgt_ids = corres_np[:, 1].reshape(-1,1)
# # 拼成 [src_id, tgt_id, src_xyz , tgt_xyz]
# data = np.hstack((src_ids, tgt_ids, P, Q))
# # 保存 CSV
# np.savetxt(
#     "potential_correspondences.csv",
#     data,
#     delimiter=",",
#     header="src_id,tgt_id,src_x,src_y,src_z,tgt_x,tgt_y,tgt_z",
#     comments=''
# )
# print("Saved potential_correspondences.csv with point indices")

# # ---------- SVD / Arun method ----------
# # 1. 计算质心
# centroid_P = np.mean(P, axis=0)
# centroid_Q = np.mean(Q, axis=0)
# # 2. 去中心化
# P_centered = P - centroid_P
# Q_centered = Q - centroid_Q
# # 3. 计算协方差矩阵
# H = P_centered.T @ Q_centered
# # 4. SVD
# U, S, Vt = np.linalg.svd(H)
# # 5. 计算旋转矩阵
# R = Vt.T @ U.T
# # 6. 处理反射情况
# if np.linalg.det(R) < 0:
#     Vt[2, :] *= -1
#     R = Vt.T @ U.T
# # 7. 计算平移向量
# t = centroid_Q - R @ centroid_P
# print("\n===== SVD Result from correspondences =====")
# print("Rotation R =\n", R)
# print("Translation t =\n", t)
# # 8. 组成 4x4 齐次变换矩阵
# T = np.eye(4)
# T[:3, :3] = R
# T[:3, 3] = t
# print("Transformation T =\n", T)
# pcd2_svd = o3d.geometry.PointCloud(pcd2)
# pcd2_svd.transform(T)
# o3d.visualization.draw_geometries([pcd1, pcd2_svd])


import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

pcd = o3d.io.read_point_cloud("final_map_fixed_0.01.pcd")
pts = np.asarray(pcd.points)
print("Original points N =", len(pts))
# print("Original points:", len(pcd.points))

# optional: downsample for speed; set to 0.0 to disable
voxel_size = 0.05
# if voxel_size > 0:
#     pcd = pcd.voxel_down_sample(voxel_size)
# print("Working points:", len(pcd.points))
for v in [0.05, 0.1, 0.2, 0.5]:
    down = pcd.voxel_down_sample(v)
    print(v, len(down.points), len(down.points) / len(pts))
pts = np.asarray(pcd.points)

# estimate normals
normal_radius = max(0.1, voxel_size * 2.0)
pcd.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30)
)

# compute FPFH
feature_radius = max(0.25, voxel_size * 5.0)
fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=100)
)

F = np.asarray(fpfh.data).T  # shape: (N, D)
print("FPFH shape:", F.shape)

# descriptor saliency: distance to mean descriptor
mu = F.mean(axis=0)
dist = np.linalg.norm(F - mu, axis=1)

print("Descriptor distance stats")
print("min   :", dist.min())
print("mean  :", dist.mean())
print("median:", np.median(dist))
print("p90   :", np.percentile(dist, 90))
print("max   :", dist.max())

nn_dist = np.asarray(pcd.compute_nearest_neighbor_distance())
print("NN mean   =", nn_dist.mean())
print("NN median =", np.median(nn_dist))
print("NN p90    =", np.percentile(nn_dist, 90))

# select salient points
alpha = 1.0
thr = dist.mean() + alpha * dist.std()
mask = dist > thr
print("Salient points:", mask.sum(), "/", len(mask))

# color by saliency
dist_norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-12)
colors = np.zeros((len(dist_norm), 3))
colors[:, 0] = dist_norm  # red = more distinctive
colors[:, 2] = 1.0 - dist_norm  # blue = less distinctive
pcd.colors = o3d.utility.Vector3dVector(colors)

# histogram
plt.figure(figsize=(8, 5))
plt.hist(nn_dist, bins=100, color="steelblue", edgecolor="black")
plt.axvline(
    nn_dist.mean(), color="red", linestyle="--", label=f"mean={nn_dist.mean():.3f} m"
)
plt.axvline(
    np.median(nn_dist),
    color="green",
    linestyle="--",
    label=f"median={np.median(nn_dist):.3f} m",
)
plt.xlabel("Nearest-neighbor distance (m)")
plt.ylabel("Number of points")
plt.title("Histogram of nearest-neighbor distances")
plt.legend()
plt.tight_layout()
plt.savefig("nn_histogram_cp_test1.png", dpi=200)
plt.show()
# visualize all points with saliency coloring
o3d.visualization.draw_geometries([pcd], window_name="FPFH saliency")

# visualize only salient points
salient_pts = pts[mask]
pcd_salient = o3d.geometry.PointCloud()
pcd_salient.points = o3d.utility.Vector3dVector(salient_pts)
pcd_salient.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([pcd_salient], window_name="Salient FPFH points")

# histogram
# plt.figure(figsize=(8, 5))
# plt.hist(nn_dist, bins=100, color="steelblue", edgecolor="black")
# plt.axvline(np.mean(nn_dist), color="red", linestyle="--", label=f"mean={nn_dist.mean():.3f} m")
# plt.axvline(np.median(nn_dist), color="green", linestyle="--", label=f"median={np.median(nn_dist):.3f} m")
# plt.xlabel("Nearest-neighbor distance (m)")
# plt.ylabel("Number of points")
# plt.title("Histogram of nearest-neighbor distances")
# plt.legend()
# plt.tight_layout()
# plt.show()
