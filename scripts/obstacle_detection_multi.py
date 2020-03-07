#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/obstacle_detection_cs169/scripts/obstacle_detection_multi.py

import rospy
import math
import numpy as np
import csv
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from timeit import default_timer as timer

RATE = 20 # rospy.rate
EXPECTED_CPA_THRESHOLD = float(rospy.get_param("~threshold", default="0.4"))

CSV_SAVE_PATH_CASE1 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case1.csv'
CSV_SAVE_PATH_CASE2 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case2.csv'
CSV_SAVE_PATH_CASE3 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case3.csv'
CSV_SAVE_PATH_CASE4 = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case4.csv'

CSV_SAVE_PATH_CASE1_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case1_CPA.csv'
CSV_SAVE_PATH_CASE2_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case2_CPA.csv'
CSV_SAVE_PATH_CASE3_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case3_CPA.csv'
CSV_SAVE_PATH_CASE4_CPA = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case4_CPA.csv'

CSV_SAVE_PATH_CASE1_SPEED = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case1_speed.csv'
CSV_SAVE_PATH_CASE2_SPEED = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case2_speed.csv'
CSV_SAVE_PATH_CASE3_SPEED = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case3_speed.csv'
CSV_SAVE_PATH_CASE4_SPEED = '/home/mingi/catkin_ws/src/object_detection_cs169/csv/case4_speed.csv'

'''
data saving functions as csv for plotting
Args:
    path of csv file for the output
    timelist for the plot
    datalist such as CPA, distacne
Output:
    csv file saved in the ROS package csv folder
'''
def csv_data_saver(PATH, timelist, datalist):
    with open(PATH, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))

def csv_data_saver_CPA(PATH, timelist, datalist):
    with open(PATH, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))

def csv_data_saver_speed(PATH, timelist, datalist):
    with open(PATH, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))


# =========== Obstacle detector =========== #
class Obstacle_detector():
    def __init__(self):
        # subscriber for viz topic published by multi kf algorithm
        self.multi_subscriber = rospy.Subscriber("viz", MarkerArray, self.multi_callback)
        self.rate = rospy.Rate(RATE)
        self.current_msg = None
        self.before_msg = None
        self.current_time = 0
        self.initial_time = timer()
        self.before_time = self.initial_time
        self.distance_list = []
        self.coordinate_list = []
        self.time_list = []
        self.velocity_list = []
        self.CPA_list = []


    def initial_update_msg(self):
        '''
        data saving functions as csv for plotting
        Args:
            path of csv file for the output
            timelist for the plot
            datalist such as CPA, distacne
        Output:
            csv file saved in the ROS package csv folder
        '''
        self.initial_msg = rospy.wait_for_message("viz", MarkerArray)
        print("initial received")


    def multi_callback(self, msg):
        '''
        call back for multi kf algorithm
        Args:
            subscribed msg from vis topic
        Output:
            csv file saved in the ROS package csv folder
            Additionally, for the commented part in the bottom, it should be excuted first to figure out which object id is what we want to see.
        '''
        self.current_time = timer()
        self.current_msg = msg

        # ID finder : from rosbag file it finds the id we want for a person
        # for case 1 it is number 2 or 4 (dark skyblue first and green): not used for algorithm's poor performance
        # for case 2 it is number 0 (black)
        # for case 3 it is number 4 (green)
        # for case 4 it is number 2 (dark sky blue)
        if self.before_msg is not None:
            if self.current_time - self.before_time > 0.1:
                for marker in self.current_msg.markers:
                    # change marker id as per each case
                    if int(marker.id) == 2:
                        distance = math.sqrt((marker.pose.position.x - 0)**2 + (marker.pose.position.y - 0)**2)
                        self.distance_list.append(distance)
                        self.time_list.append(self.current_time - self.initial_time)
                        self.coordinate_list.append((marker.pose.position.x, marker.pose.position.y))

                        print("id", marker.id, "distance", distance) # tester

                        # CPA calculator and potential collision warning process
                        if len(self.coordinate_list) >= 2 and (self.coordinate_list[-1][0] - self.coordinate_list[-2][0]) != 0 :
                            M = (self.coordinate_list[-1][1] - self.coordinate_list[-2][1]) / ((self.coordinate_list[-1][0] - self.coordinate_list[-2][0]))
                            # excluding cases when denominator becomes 0
                            # the calculation of CPA prediction is based on the system of linear equation.
                            # details about mathmatical deriviation will be included in the final report.
                            if M != 0:
                                M2 = -1/M
                                x_i = (self.coordinate_list[-2][1] + self.coordinate_list[-2][0]*M) / (M2-M)
                                y_i = (self.coordinate_list[-2][1]- M* self.coordinate_list[-2][0]) / (1-M/M2)
                                CPA = math.sqrt(x_i**2 + y_i**2)
                                self.CPA_list.append(CPA)
                                if CPA <= EXPECTED_CPA_THRESHOLD:
                                    print("CPA warning for collision", CPA)

                        # speed calculator
                        if len(self.distance_list) >=2:
                            self.velocity_list.append((self.distance_list[-1] - self.distance_list[-2])/(self.current_time - self.before_time))
                        # csv file saver depending on the case please set up
                        # change path as per each case
                        csv_data_saver(CSV_SAVE_PATH_CASE4, self.time_list, self.distance_list)
                        csv_data_saver_CPA(CSV_SAVE_PATH_CASE4_CPA, self.time_list, self.CPA_list)
                        csv_data_saver_speed(CSV_SAVE_PATH_CASE4_SPEED, self.time_list, self.velocity_list)

                    '''
                    Goal : To find a marker which we want
                    e.g., Based on the all 6 markers distance change, I can find the marker index which the object is expressed for in each ros bag
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
        # time update for the before time
        self.before_time = self.current_time

    # rospy spin function for the loop
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
