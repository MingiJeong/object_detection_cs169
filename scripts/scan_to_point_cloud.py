#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

'''
This is the code from http://wiki.ros.org/laser_geometry
The package converts the 2D laserscan to 3D point cloud
Goal: we want to use 3D point cloud as an inpu for obstacle_anaylsis.launch

'''
rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("filtered_cloud", PointCloud2, queue_size=1)

def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)

    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)
    # we can access a generator in a loop
    sum = 0.0
    num = 0
    for point in point_generator:
        if not math.isnan(point[2]):
            sum += point[2]
            num += 1
    print(str(sum/num))
    point_list = pc2.read_points_list(pc2_msg)
    print(point_list[len(point_list)/2].x)



rospy.Subscriber("scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()
