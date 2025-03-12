#!/usr/bin/env python3

import rospy
import actionlib
import csv
import os
import time
import psutil
import threading
from datetime import datetime
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

class MoveBaseWithLogging:
    def __init__(self):
        rospy.init_node('move_base_logger')

        # Publisher ควบคุมการหมุนของหุ่นยนต์
        self.cmd_vel_publisher = rospy.Publisher('/velocity_controller/cmd_vel', Twist, queue_size=10)
        
        # Subscriber เพื่อรับข้อมูลตำแหน่งจาก /mcl_pose และ /odom
        self.mcl_subscriber = rospy.Subscriber('/mcl_pose', PoseStamped, self.mcl_callback)
        self.odom_subscriber = rospy.Subscriber('/velocity_controller/odom', Odometry, self.odom_callback)

        # ตัวแปรเก็บค่าตำแหน่ง
        self.logging = True  # ควบคุมการบันทึก
        
        # กำหนดพาธไฟล์ CSV
        self.csv_directory = os.path.expanduser("/home/supannee/Project/src/ALS_Final_Project/als_mobile_robot/result/Narrow_Map/Rotation/")
        self.localization_filename = os.path.join(self.csv_directory, "localization_rotation_data.csv")
        self.system_usage_filename = os.path.join(self.csv_directory, "system_usage_rotation_data.csv")
        
        # ตรวจสอบว่าไดเรกทอรีมีอยู่หรือไม่ ถ้าไม่มีให้สร้างใหม่
        if not os.path.exists(self.csv_directory):
            os.makedirs(self.csv_directory)
        
        # สร้างไฟล์ CSV สำหรับตำแหน่ง
        with open(self.localization_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["MCL_Timestamp", "MCL_X", "MCL_Y", "Odom_Timestamp", "Odom_X", "Odom_Y"])

        # สร้างไฟล์ CSV สำหรับการใช้งานระบบ
        with open(self.system_usage_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'CPU Usage (%)', 'RAM Usage (%)'])
        
        # ตัวแปรเก็บค่าล่าสุด
        self.latest_mcl = None
        self.latest_odom = None
        self.latest_mcl_time = None
        self.latest_odom_time = None
        
        # เริ่ม Thread สำหรับการบันทึกข้อมูล
        self.logging_thread = threading.Thread(target=self.log_position)
        self.system_usage_thread = threading.Thread(target=self.log_system_usage)
        self.logging_thread.start()
        self.system_usage_thread.start()

    def mcl_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก mcl """
        self.latest_mcl = msg.pose  # ดึงค่า pose เท่านั้น
        self.latest_mcl_time = msg.header.stamp.to_sec()

    def odom_callback(self, msg):
        """ Callback สำหรับรับค่าตำแหน่งจาก Odom (Ground Truth) """
        self.latest_odom = msg.pose.pose  # ดึงค่า pose เท่านั้น
        self.latest_odom_time = msg.header.stamp.to_sec()

    def log_position(self):
        """ บันทึกค่าตำแหน่งทุก 0.1 วินาที """
        rospy.loginfo("Waiting for MCL and Odom data before logging...")
        while not self.latest_mcl or not self.latest_odom:
            rospy.sleep(0.1)
        
        rospy.loginfo("MCL and Odom data received. Starting logging...")
        rate = rospy.Rate(10)  # 10 Hz (0.1 วินาที)
        while self.logging and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
            if self.latest_mcl and self.latest_odom:
                mcl_time = self.latest_mcl_time
                mcl_x = self.latest_mcl.position.x
                mcl_y = self.latest_mcl.position.y

                odom_time = self.latest_odom_time
                odom_x = self.latest_odom.position.x
                odom_y = self.latest_odom.position.y

                with open(self.localization_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([mcl_time, mcl_x, mcl_y, odom_time, odom_x, odom_y])

            rate.sleep()

    def log_system_usage(self):
        """ บันทึกค่าการใช้งาน CPU และ RAM ทุก 0.1 วินาที """
        rospy.loginfo("Starting system usage logging...")
        while self.logging and not rospy.is_shutdown():
            start_time = time.time()
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            cpu_usage = psutil.cpu_percent(interval=None)
            ram_usage = psutil.virtual_memory().percent
            
            with open(self.system_usage_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, cpu_usage, ram_usage])
            
            elapsed = time.time() - start_time
            sleep_duration = max(0.1 - elapsed, 0)
            time.sleep(sleep_duration)

    def rotate_robot(self, duration=60):
        """ หมุนหุ่นยนต์อยู่กับที่เป็นเวลาที่กำหนด """
        rospy.loginfo("Starting rotation...")
        twist = Twist()
        twist.angular.z = 0.5  # Set angular speed
        
        rate = rospy.Rate(50)  # Publish more frequently
        start_time = rospy.Time.now().to_sec()
        
        while (rospy.Time.now().to_sec() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel_publisher.publish(twist)
            rate.sleep()
        
        # หยุดการหมุน
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        rospy.loginfo("Rotation complete.")
        
        # หยุดการบันทึก
        self.logging = False

if __name__ == '__main__':
    try:
        mover = MoveBaseWithLogging()
        mover.rotate_robot()  # หมุนอยู่กับที่เป็นเวลา 1 นาที
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
