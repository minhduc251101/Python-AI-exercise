import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("cp_output_point_cloud_map.pcd")
pts = np.asarray(pcd.points)
print("Min:", pts.min(axis=0))
print("Max:", pts.max(axis=0))
print("Center:", pts.mean(axis=0))