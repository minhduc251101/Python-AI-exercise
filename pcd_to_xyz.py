import open3d as o3d

# 1. Load the PLY file
# Replace 'input.ply' with the path to your file
pcd = o3d.io.read_point_cloud("/media/minhduc/TOSHIBA EXT1/4dradarslam/NTU4Dradlm/cp/cp/loop_true_fullpose/005032/cloud.pcd")

# 2. Save the point cloud as an XYZ file
# Open3D automatically formats it as space-separated coordinates
o3d.io.write_point_cloud("/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/output_005032.xyz", pcd)

print("Conversion complete!")