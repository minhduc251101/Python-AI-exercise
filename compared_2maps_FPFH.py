import copy
import numpy as np
import open3d as o3d


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


voxel_size = 0.7

source = o3d.io.read_point_cloud("final_map_fixed.pcd")
target = o3d.io.read_point_cloud("final_map_fixed_0.01.pcd")

print("source points:", len(source.points), flush=True)
print("target points:", len(target.points), flush=True)

print("start preprocess", flush=True)
source_down, source_fpfh = preprocess(source, voxel_size)
target_down, target_fpfh = preprocess(target, voxel_size)
print("done preprocess", flush=True)

print("source down points:", len(source_down.points), flush=True)
print("target down points:", len(target_down.points), flush=True)
print("source FPFH shape:", np.asarray(source_fpfh.data).shape, flush=True)
print("target FPFH shape:", np.asarray(target_fpfh.data).shape, flush=True)

print("start correspondences", flush=True)
corres = o3d.pipelines.registration.correspondences_from_features(
    source_fpfh,
    target_fpfh,
    mutual_filter=True
)
corres_np = np.asarray(corres)
print("done correspondences:", corres_np.shape, flush=True)

distance_threshold = voxel_size * 1.5

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
    criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(5000, 200)
)
print("done ransac", flush=True)

print("\n===== RANSAC result =====")
print("fitness:", result_ransac.fitness)
print("inlier_rmse:", result_ransac.inlier_rmse)
print("transformation:\n", result_ransac.transformation)

draw_result(source, target, result_ransac.transformation, "FPFH RANSAC alignment")

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
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
)
print("done ICP", flush=True)

print("\n===== ICP result =====")
print("fitness:", result_icp.fitness)
print("inlier_rmse:", result_icp.inlier_rmse)
print("transformation:\n", result_icp.transformation)

draw_result(source, target, result_icp.transformation, "ICP refined alignment")