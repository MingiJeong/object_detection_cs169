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

RATE = 20 # rospy.rate

CSV_SAVE_PATH_CASE1 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case1.csv'
CSV_SAVE_PATH_CASE2 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case2.csv'
CSV_SAVE_PATH_CASE3 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case3.csv'
CSV_SAVE_PATH_CASE4 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case4.csv'

CSV_SAVE_PATH_CASE1_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case1_CPA.csv'
CSV_SAVE_PATH_CASE2_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case2_CPA.csv'
CSV_SAVE_PATH_CASE3_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case3_CPA.csv'
CSV_SAVE_PATH_CASE4_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case4_CPA.csv'

def csv_data_saver(PATH, timelist, datalist):
    with open(PATH, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))

def csv_data_saver_CPA(PATH, timelist, datalist):
    with open(PATH, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))


# =========== Obstacle detector =========== #
class Obstacle_detector():
    def __init__(self):
        # subscriber and publisher
        # self.lidar_subscriber = rospy.Subscriber("velodyne_points", PointCloud2, self.lidar_callback)
        # self.filtered_publisher = rospy.Publisher("filtered_cloud", PointCloud2, queue_size=10)
        self.multi_subscriber = rospy.Subscriber("viz", MarkerArray, self.multi_callback)
        self.rate = rospy.Rate(RATE)
        self.current_msg = None
        self.before_msg = None
        self.current_time = 0
        self.before_time = timer()
        # self.initial_msg = None # not use now
        self.distance_list = []
        self.coordinate_list = []
        self.time_list = []
        self.velocity_list = []
        self.CPA_list = []

    def initial_update_msg(self):
        self.initial_msg = rospy.wait_for_message("viz", MarkerArray)
        print("initial received")

    def multi_callback(self, msg):
        self.current_time = timer()
        self.current_msg = msg

        # ID finder : from rosbag file it finds the id we want for a person
        # ID 3 is white or pink
        # for case 1 it is number 2 or 4 (d.s furst and green)
        # for case 2 it is number 0 (blak)
        # for case 3 it is number 4 (green)
        # for case 4 it is number 2 (dark sky blue)
        if self.before_msg is not None:
            if self.current_time - self.before_time > 0.1:
                for marker in self.current_msg.markers:
                    if int(marker.id) == 0:
                        distance = math.sqrt((marker.pose.position.x - 0)**2 + (marker.pose.position.y - 0)**2)
                        self.distance_list.append(distance)
                        self.time_list.append(self.current_time - self.before_time)
                        self.coordinate_list.append((marker.pose.position.x, marker.pose.position.y))
                        if len(self.distance_list) >=2:
                            self.velocity_list.append((self.distance_list[-1] - self.distance_list[-2])/(self.current_time - self.before_time))
                        print("id", marker.id, "distance", distance) # tester

                        # CPA calculator process
                        if len(self.coordinate_list) >= 2 and (self.coordinate_list[-1][0] - self.coordinate_list[-2][0]) != 0 :
                            M = (self.coordinate_list[-1][1] - self.coordinate_list[-2][1]) / ((self.coordinate_list[-1][0] - self.coordinate_list[-2][0]))
                            if M != 0:
                                M2 = -1/M
                                x_i = (self.coordinate_list[-2][1] + self.coordinate_list[-2][0]*M) / (M2-M)
                                y_i = (self.coordinate_list[-2][1]- M* self.coordinate_list[-2][0]) / (1-M/M2)
                                CPA = math.sqrt(x_i**2 + y_i**2)
                                self.CPA_list.append(CPA)
                                print("CPA", CPA)
                        csv_data_saver(CSV_SAVE_PATH_CASE2, self.time_list, self.distance_list)
                        csv_data_saver_CPA(CSV_SAVE_PATH_CASE2_CPA, self.time_list, self.CPA_list)

                        #csv_data_saver(CSV_SAVE_PATH_CASE4, self.time_list, self.distance_list, self.velocity_list, self.CPA_list)

                    '''
                    Goal : To find a marker which we want
                    e.g., Based on the all 6 markers distance change, I can find the marker index which the object is expressed for
                    '''
                    '''
                    for b4marker in self.before_msg.markers:
                        if marker.id == b4marker.id:
                            distance = math.sqrt((marker.pose.position.x - 0)**2 + (marker.pose.position.y - 0)**2)
                            print("id", marker.id, "distance", distance)
                            self.before_time = self.current_time
                    '''

        # before_msg is None (initial)
        else:
            self.before_msg = self.current_msg


    def lidar_callback(self, msg):
        self.input_msg = msg
        self.filtered_publisher.publish(self.input_msg)
        print("msg publishing test", self.input_msg.height)

    def spin(self):
        while not rospy.is_shutdown():
           self.rate.sleep()

def main():
    obstacle_detector = Obstacle_detector()
    # obstacle_detector.initial_update_msg()
    obstacle_detector.spin()


if __name__ == "__main__" :
    rospy.init_node("obstacle_detector")
    main()
