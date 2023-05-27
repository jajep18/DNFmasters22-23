import open3d as o3d

import numpy as np


# Create a plane mesh


robot_pos = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
robot_pos.translate((0, 0, 0))
robot_pos.rotate(o3d.geometry.get_rotation_matrix_from_xyz((0, 0, 0)), center=(0, 0, 0))


# Cam left
# 0.1 -0.18 0.7 0 1.3107 -1.5707
cam_left = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
# rpy_left = np.array([0, 1.3107, -1.5707])
# rpy_left = np.array([0, 0, -1.5707])
# cam_left.rotate(o3d.geometry.get_rotation_matrix_from_xyz(( rpy_left)), center=(0, 0, 0))
cam_left.translate((0.1, -0.18, 0.7))




# Cam right
# -0.1 -0.22 0.7 0 1.3107 -1.5707
cam_right = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.1)
# rpy_right = np.array([0, 1.3107, -1.5707])
# cam_right.rotate(o3d.geometry.get_rotation_matrix_from_xyz(( rpy_right)), center=(0, 0, 0))
# cam_right.translate((0.1, -0.22, 0.7))
cam_right.translate((0.0, -0.0, 0.7))

# convert to deg from rad


# Create a single point 


blue_ball = o3d.geometry.PointCloud()
blue_ball.points = o3d.utility.Vector3dVector(np.array([[-0.1, -0.2, 0.1]]))
blue_ball.paint_uniform_color([0, 0, 1])
# Make the point bigger
blue_ball.scale(1, center=blue_ball.get_center())

red_ball = o3d.geometry.PointCloud()
red_ball.points = o3d.utility.Vector3dVector(np.array([[0.0, -0.2, 0.1]]))
red_ball.paint_uniform_color([1, 0, 0])
# Make the point bigger
red_ball.scale(1, center=red_ball.get_center())

green_ball = o3d.geometry.PointCloud()  
green_ball.points = o3d.utility.Vector3dVector(np.array([[0.1, -0.2, 0.1]]))
green_ball.paint_uniform_color([0, 1, 0])
# Make the point bigger
green_ball.scale(1, center=green_ball.get_center())

# Create a large plane of points
plane_large = o3d.geometry.PointCloud()
point_arr = np.array([]) 
dims = 100

for x in range(0, dims):
        for z in range(0, dims):
                point_arr = np.append(point_arr, [x, 0, z])
point_arr.resize((dims * dims, 3))

plane_large.points = o3d.utility.Vector3dVector(point_arr)

plane_large.paint_uniform_color([0.5, 0.5, 0.5])
# print(plane_large)




o3d.visualization.draw_geometries([ cam_left, cam_right, red_ball, blue_ball, green_ball, robot_pos])


# Get translation from camera to robot
print("Translation from left camera to robot")
rel_pos = cam_left.get_center() - robot_pos.get_center()
print(rel_pos)
# print("Rotation from left camera to robot")
# rel_rot = cam_left.get_rotation_matrix_from_xyz() - robot_pos.get_rotation_matrix_from_xyz()
# print(rel_rot)

print("Translation from right camera to robot")
rel_pos = cam_right.get_center() - robot_pos.get_center()
print(rel_pos)
# print("Rotation from right camera to robot")
# rel_rot = cam_right.get_rotation_matrix_from_xyz() - robot_pos.get_rotation_matrix_from_xyz()





