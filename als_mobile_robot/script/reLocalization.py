#!/usr/bin/env python3

import rospy
import random
import csv
import time
import tf
import os
from threading import Event, Thread
from datetime import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import psutil

# Define CSV save path
CSV_PATH = "/home/supannee/Project/src/ALS_Final_Project/als_mobile_robot/result/Normal_Ob/Unmap_Relocalization/iter2/"

# Ensure directory exists
if not os.path.exists(CSV_PATH):
    os.makedirs(CSV_PATH)
    rospy.loginfo(f"📂 Created directory: {CSV_PATH}")

class FakePoseTester:
    def __init__(self):
        rospy.init_node("fake_pose_tester", anonymous=True)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber("/reliability", Vector3Stamped, self.reliability_callback)
        
        self.reliability = 1.0  # Initial reliability
        self.relocalized = Event()
        self.data = []  # Store test data
        self.system_data = []  # Store system usage data
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.loginfo("✅ Fake Pose Tester Initialized")

        # Start system monitoring thread
        self.monitoring = True
        self.monitor_thread = Thread(target=self.monitor_system_usage)
        self.monitor_thread.start()

    def reliability_callback(self, msg):
        self.reliability = msg.vector.x
        rospy.loginfo(f"🔹 Received Reliability: {self.reliability:.6f}")
        if self.reliability >= 0.9999:
            self.relocalized.set()

    def monitor_system_usage(self):
        interval = 0.1  # 100 milliseconds
        while self.monitoring:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            cpu_usage = psutil.cpu_percent()
            ram_usage = psutil.virtual_memory().percent
            self.system_data.append([timestamp, cpu_usage, ram_usage])
            rospy.loginfo(f"🖥️ CPU: {cpu_usage}%, RAM: {ram_usage}%")
            time.sleep(interval)

    def run_tests(self, num_tests=10):
        for _ in range(num_tests):
            x_fake = random.uniform(-1.0, 1.0)
            y_fake = random.uniform(-1.0, 1.0)
            
            rospy.loginfo(f"🚀 Running test with fake pose: X={x_fake}, Y={y_fake}")
            self.relocalized.clear()
            start_time = time.time()
            
            self.pose_pub.publish(self.create_pose_message(x_fake, y_fake))
            rospy.loginfo("📍 Published Fake Pose")
            
            # Wait for relocalization (max 5 sec timeout)
            if self.relocalized.wait(timeout=5):
                relocalization_time = (time.time() - start_time) * 1000  # Convert to milliseconds
                status = "Success"
            else:
                relocalization_time = "Timeout"
                status = "Failed"
            
            pose_diff = ((x_fake ** 2 + y_fake ** 2) ** 0.5)
            
            self.data.append([0, 0, x_fake, y_fake, relocalization_time, pose_diff, status])
            rospy.loginfo(f"✅ Test {status}: Time={relocalization_time} ms, Pose Diff={pose_diff}")
            
        self.save_data()
        self.monitoring = False  # Stop monitoring system usage
        self.monitor_thread.join()
        self.save_system_usage()

    def create_pose_message(self, x, y):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        return pose_msg

    def save_data(self):
        rospy.loginfo(f"🔄 Saving test results to {CSV_PATH}fake_pose_results.csv")
        with open(CSV_PATH + "fake_pose_results.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["X_true", "Y_true", "X_Fake", "Y_Fake", "Re-Localize_time (ms)", "Pose Diff", "Status"])
            for row in self.data:
                writer.writerow(row)
        rospy.loginfo("✅ Data saved successfully!")

    def save_system_usage(self):
        rospy.loginfo(f"🔄 Saving system usage data to {CSV_PATH}system_usage.csv")
        with open(CSV_PATH + "system_usage.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "CPU Usage (%)", "RAM Usage (%)"])
            for row in self.system_data:
                writer.writerow(row)
        rospy.loginfo("✅ System usage data saved successfully!")

if __name__ == "__main__":
    tester = FakePoseTester()
    tester.run_tests(num_tests=10)
