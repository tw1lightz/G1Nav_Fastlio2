import open3d as o3d
pcd = o3d.io.read_point_cloud("D:\G1_dev\HongTu\G1Nav2D\src\fastlio2\PCD\ground_map.pcd")
print(pcd)  # 看看点数、有没有颜色
o3d.visualization.draw_geometries([pcd])
