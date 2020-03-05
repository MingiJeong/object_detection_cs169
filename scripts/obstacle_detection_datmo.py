#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/obstacle_detection_cs169/scripts/obstacle_detection_datmo.py

import rospy
import math
import numpy as np
import csv
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from timeit import default_timer as timer

RATE = 15 # rospy.rate
# =========== Obstacle detector =========== #
class Obstacle_detector():
    def __init__(self):
        # subscriber and publisher
        # self.lidar_subscriber = rospy.Subscriber("velodyne_points", PointCloud2, self.lidar_callback)
        # self.filtered_publisher = rospy.Publisher("filtered_cloud", PointCloud2, queue_size=10)
        self.datmo_ukf_subscriber = rospy.Subscriber("marker_array", MarkerArray, self.datmo_callback)
        self.rate = rospy.Rate(RATE)
        self.current_msg = None
        self.before_msg = None
        self.current_time = 0
        self.before_time = timer()
        self.initial_msg = None
        self.distance_list = []

    def initial_update_msg(self):
        self.initial_msg = rospy.wait_for_message("marker_array", MarkerArray)
        print("initial received")

    def datmo_callback(self, msg):
        if self.current_time - self.before_time > 23 and self.initial_msg is not None:
            self.current_msg = msg
            for marker in self.current_msg.markers:
                print("accessed")
                distance = math.sqrt((marker.pose.position.x - 0)**2 + (marker.pose.position.y - 0)**2)
                distance_list.append(distance_list)
            print("min", min(distance_list))

        ''' # it is not working properly
        #marker finder
        self.current_time = timer()
        # print(self.before_time)
        if self.current_time - self.before_time > 8 and self.initial_msg is not None:
            self.current_msg = msg
            for marker in self.current_msg.markers:
                for b4marker in self.initial_msg.markers:
                    if marker.id == b4marker.id:
                        print("accessed")
                        distance = math.sqrt((marker.pose.position.x - b4marker.pose.position.x)**2 + (marker.pose.position.y - b4marker.pose.position.y)**2)
                        if distance > 1:
                            print("this is distance", distance)
                            print("ID", marker.id)

        '''

    def lidar_callback(self, msg):
        self.input_msg = msg
        self.filtered_publisher.publish(self.input_msg)
        print("msg publishing test", self.input_msg.height)

    def spin(self):
        while not rospy.is_shutdown():
           self.rate.sleep()

def main():
    obstacle_detector = Obstacle_detector()
    obstacle_detector.initial_update_msg()
    obstacle_detector.spin()


if __name__ == "__main__" :
    rospy.init_node("obstacle_detector")
    main()
