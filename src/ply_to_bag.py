#!/usr/bin/env python
import numpy as np

import rospy
from sensor_msgs import point_cloud2

from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import rosbag

def points_to_pc2_msg(points, frame_id):
    """
    Args:
        points: an Nx3 array
        frame_id: the frame to publish in

    Returns: a PointCloud2 message ready to be published to rviz

    """
    header = Header(frame_id=frame_id)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)
              ]
    pc2_msg = point_cloud2.create_cloud(header, fields, points)
    return pc2_msg


def xyz_to_pc2msg(filepath):
    A = np.loadtxt(filepath)
    A = A[~np.all(A == 0, axis=1)]
    pc2_msg = points_to_pc2_msg(A, "camera_depth_optical_frame")
    return pc2_msg


def main():
    rospy.init_node("xyz_to_pointcloud")
    pc2_msg1 = xyz_to_pc2msg("/home/christianforeman/catkin_ws/src/point_cloud_selector/pcs/scan_1_cropped.xyz")
    pc2_msg2 = xyz_to_pc2msg("/home/christianforeman/catkin_ws/src/point_cloud_selector/pcs/scan_2_cropped.xyz")
    pc2_msg3 = xyz_to_pc2msg("/home/christianforeman/catkin_ws/src/point_cloud_selector/pcs/scan_3_cropped.xyz")

    bag = rosbag.Bag("High_res.bag", 'w')
    bag.write('/camera/depth/color/points', pc2_msg1)
    bag.write('/camera/depth/color/points', pc2_msg2)
    bag.write('/camera/depth/color/points', pc2_msg3)
    bag.close()

    # publish_time(pub, pc2_msg)

    
if __name__ == "__main__":
    main()
