import trimesh
import open3d as o3d
# import numpy as np

# 加载STL文件
mesh = trimesh.load_mesh("/home/radar/Desktop/radar2025/sp_radar_25/src/main/relocalization/pcd/RMUC2025.STL")
mesh.show()

# # 获取网格的所有三角形面片
# faces = mesh.faces
# vertices = mesh.vertices

# # 使用三角形面片的中心点进行均匀采样
# sampled_points = mesh.sample(1000000)  # 采样更多的点

# # 删除 z 坐标大于 5 的点
# # filtered_points = sampled_points[sampled_points[:, 2] <= 3.0]

# # 创建Open3D点云对象
# pcd = o3d.geometry.PointCloud()

# # 将过滤后的点转换为Open3D格式
# pcd.points = o3d.utility.Vector3dVector(sampled_points)
# # 保存为PCD文件
# o3d.io.write_point_cloud("/home/radar/Desktop/radar2025/sp_radar_25/src/main/relocalization/pcd/RMUC.pcd", pcd)

# print("PCD file saved as output.pcd")
