#!/usr/bin/env python3

import rospy
import actionlib
import csv
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading

class MoveBaseWithLogging:
    def __init__(self):
        rospy.init_node('move_base_logger')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server!")

        self.mcl_subscriber = rospy.Subscriber('/mcl_pose', PoseStamped, self.mcl_callback)
        self.odom_subscriber = rospy.Subscriber('/velocity_controller/odom', Odometry, self.odom_callback)

        self.data_log = []
        self.logging = True

        self.csv_directory = os.path.expanduser("~/Project/src/ALS_Final_Project/als_mobile_robot/result/")
        self.csv_filename = os.path.join(self.csv_directory, "localization_data_mcl.csv")

        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)

        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["MCL_time", "MCL_X", "MCL_Y", "Odom_time", "Odom_X", "Odom_Y", "Distance"])

        self.latest_mcl = None
        self.latest_odom = None
        self.mcl_time = None
        self.odom_time = None

        self.logging_thread = threading.Thread(target=self.log_position)
        self.logging_thread.start()

    def mcl_callback(self, msg):
        self.latest_mcl = msg.pose
        self.mcl_time = msg.header.stamp.to_sec()
        rospy.loginfo(f"MCL Pose Updated: X={msg.pose.position.x}, Y={msg.pose.position.y}")

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose
        self.odom_time = msg.header.stamp.to_sec()
        rospy.loginfo(f"Odom Pose Updated: X={msg.pose.pose.position.x}, Y={msg.pose.pose.position.y}")

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return (dx**2 + dy**2) ** 0.5

    def log_position(self):
        rate = rospy.Rate(10)
        while self.logging and not rospy.is_shutdown():
            rospy.sleep(0.1)

            if self.latest_mcl and self.latest_odom:
                mcl_x = self.latest_mcl.position.x
                mcl_y = self.latest_mcl.position.y
                odom_x = self.latest_odom.position.x
                odom_y = self.latest_odom.position.y
                distance = self.calculate_distance(self.latest_mcl, self.latest_odom)

                self.data_log.append([self.mcl_time, mcl_x, mcl_y, self.odom_time, odom_x, odom_y, distance])
                rospy.loginfo(f"Logging Data: MCL=({mcl_x}, {mcl_y}) at {self.mcl_time} | ODOM=({odom_x}, {odom_y}) at {self.odom_time} | Distance={distance}")

                with open(self.csv_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([self.mcl_time, mcl_x, mcl_y, self.odom_time, odom_x, odom_y, distance])

            rate.sleep()

    def move_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: x={x}, y={y}, theta={theta}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach goal.")

        self.logging = False

if __name__ == '__main__':
    try:
        mover = MoveBaseWithLogging()
        mover.move_to_goal(5.0, 0.0, 0.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
