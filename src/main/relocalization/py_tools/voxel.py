import open3d as o3d

# 读取PCD文件
pcd = o3d.io.read_point_cloud("/home/lp1/RM/radar/sp_radar_25/src/relocalization/pcd/RMUC.pcd")

# 应用体素网格滤波器，设置体素大小（例如0.05米）
voxel_size = 0.1
downsampled_pcd = pcd.voxel_down_sample(voxel_size)

# 保存采样后的点云为新的PCD文件
o3d.io.write_point_cloud("/home/lp1/RM/radar/sp_radar_25/src/relocalization/pcd/RMUC_voxel.pcd", downsampled_pcd)

print("downsampled")
