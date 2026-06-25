import open3d as o3d
import os
import shutil

# 1. Load the PLY file
# Replace 'input.ply' with the path to your file
pcd = o3d.io.read_point_cloud(
    "/media/minhduc/TOSHIBA EXT1/4dradarslam/NTU4Dradlm/cp/cp/loop_true_fullpose/000027/cloud.pcd"
)

# 2. Save the point cloud as an XYZ file
# Open3D automatically formats it as space-separated coordinates
o3d.io.write_point_cloud(
    "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/output_000027.xyz",
    pcd,
)

print("Conversion complete!")

# The path to your existing XYZ file
source_file = "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/output_000027.xyz"
# The path for your new TXT file
destination_file = "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/output_000027.txt"

# This perfectly copies the file and gives it the new extension
shutil.copy(source_file, destination_file)

print(f"Successfully saved as '{destination_file}'")
