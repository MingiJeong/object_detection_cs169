#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/obstacle_detection_cs169/scripts/obstacle_detection_directuse_3D.py

import rospy
import math
import numpy as np
import csv
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from timeit import default_timer as timer

RATE = 10 # rospy.rate
# =========== Obstacle detector =========== #
class Obstacle_detector():
    def __init__(self):
        # subscriber and publisher
        self.lidar_subscriber = rospy.Subscriber("velodyne_points", PointCloud2, self.lidar_callback)
        self.filtered_publisher = rospy.Publisher("filtered_cloud", PointCloud2, queue_size=10)
        self.rate = rospy.Rate(RATE)
        self.input_msg = None

    def lidar_callback(self, msg):
        self.input_msg = msg
        self.filtered_publisher.publish(self.input_msg)
        print("msg publishing test", self.input_msg.height)

    def spin(self):
        while not rospy.is_shutdown():
           self.rate.sleep()

def main():
    obstacle_detector = Obstacle_detector()
    obstacle_detector.spin()


if __name__ == "__main__" :
    rospy.init_node("obstacle_detector")
    main()
