import ros_numpy
import rospy
import pyransac3d as pyrsc
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from arc_utilities.tf2wrapper import TF2Wrapper
import tf.transformations

# Load saved point cloud and visualize it
from sensor_msgs.msg import PointCloud2

pcd = o3d.io.read_point_cloud("/home/miguel/catkin_ws/src/point_cloud_selector/pcs/current_selection.pcd")
# o3d.visualization.draw_geometries([pcd])

# Convert Open3D.o3d.geometry.PointCloud to numpy array
points = np.asarray(pcd.points)

# Print x, y, z coordinates
print("Points Coordinates")
print(points)

# open3d plane segmentation
plane_model, best_inliers = pcd.segment_plane(distance_threshold=0.0005,
                                              ransac_n=3,
                                              num_iterations=1000)
[a, b, c, d] = plane_model

# # Fit to a plane
# plane1 = pyrsc.Plane()
# # Get equation of plane
# best_eq, best_inliers = plane1.fit(points, 0.01)
#
#
# # Print length of vector
# print("----------------------------------")
# print("Length of vector: ", len(points))
#
# # Plane equation
# print("---------------------------------")
# print("Constants of plane equation: ", best_eq)
# print("")
#
# # Indices of the best inliers to plane
# print("---------------------------------")
# print("Indices of best inliers: ", best_inliers)
#
# # PLANE
#
# a, b, c, d = best_eq[0], best_eq[1], best_eq[2], best_eq[3]

normal_vec = np.array([a, b, c])

x = np.linspace(-1, 1, 100)
y = np.linspace(-1, 1, 100)

X, Y = np.meshgrid(x, y)
Z = (-d - a * X - b * Y) / c

# ax+by+cz+d=0
# ax+by+d=-cz
# -ax-by-d=cz
# (-ax-by-d)/z=c

inlier_points = points[best_inliers]


# fig = plt.figure()
# ax = fig.gca(projection='3d')
# surf = ax.plot_surface(X, Y, Z)
# ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c='black', alpha=0.001)
# ax.scatter(inlier_points[:, 0], inlier_points[:, 1], inlier_points[:, 2], s=4, label='inliers', c='red')

def plot_pointcloud_rviz(pub: rospy.Publisher, xs, ys, zs):
    list_of_tuples = [(x, y, z) for x, y, z in zip(xs, ys, zs)]
    dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
    np_record_array = np.array(list_of_tuples, dtype=dtype)
    msg = ros_numpy.msgify(PointCloud2, np_record_array, frame_id='camera_rviz', stamp=rospy.Time.now())
    pub.publish(msg)


rospy.init_node("test_ransac")
src_pub = rospy.Publisher("source", PointCloud2, queue_size=10)
inliers_pub = rospy.Publisher("inliers", PointCloud2, queue_size=10)
plane_pub = rospy.Publisher("plane", PointCloud2, queue_size=10)

# compute the centroid of the inliers to get a grasp position
# use the plane normal to define the grasp orientation
# hard code some offsets to make it look good???

inliers_centroid = np.mean(inlier_points, axis=0)

tfw = TF2Wrapper()
translation = inliers_centroid
rpy = [0, np.pi, 0]
camera2tool = tf.transformations.compose_matrix(translate=inliers_centroid, angles=rpy)
# quaternion = [0, 0, 0, 1]
tool2ee = tfw.get_transform(parent="left_tool", child="end_effector_left")
camera2ee = camera2tool @ tool2ee

while True:
    tfw.send_transform_matrix(camera2ee, parent='camera_rviz', child='end_effector_left')

    plot_pointcloud_rviz(src_pub, points[:, 0], points[:, 1], points[:, 2])
    plot_pointcloud_rviz(inliers_pub, inlier_points[:, 0], inlier_points[:, 1], inlier_points[:, 2])
    plot_pointcloud_rviz(plane_pub, X.flatten(), Y.flatten(), Z.flatten())
    rospy.sleep(0.1)
