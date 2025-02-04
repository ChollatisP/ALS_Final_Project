#!/usr/bin/env python3

import rospy
import random
import csv
import time
import psutil
import tf
from threading import Event
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class FakePoseTester:
    def __init__(self):
        rospy.init_node("fake_pose_tester", anonymous=True)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        rospy.Subscriber("/reliability", Vector3Stamped, self.reliability_callback)
        
        self.reliability = 1.0  # Initial reliability
        self.relocalized = Event()
        self.data = []  # Store test data
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.loginfo("✅ Fake Pose Tester Initialized")

    def publish_fake_pose(self, x, y):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        
        self.pose_pub.publish(pose_msg)
        rospy.loginfo(f"📍 Published Fake Pose: X={x}, Y={y}")

    def move_robot_to_origin(self):
        state_msg = ModelState()
        state_msg.model_name = "robot"
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.orientation.w = 1.0
        self.set_model_state(state_msg)
        self.publish_fake_pose(0.0, 0.0)
        rospy.loginfo("🔄 Moved robot and pose estimate to (0,0)")
        time.sleep(5)

    def reliability_callback(self, msg):
        self.reliability = msg.vector.x
        rospy.loginfo(f"🔹 Received Reliability: {self.reliability:.6f}")
        if self.reliability >= 0.9999:
            self.relocalized.set()

    def wait_for_reliability(self, x_fake, y_fake, cpu_before, ram_before):
        timeout = 5  # 5 seconds timeout
        start_relocalization = time.time()
        while self.reliability < 0.9999:
            if time.time() - start_relocalization > timeout:
                rospy.logwarn(f"⚠️ Test Timeout: Marking as Fail.")
                pose_diff = ((x_fake ** 2 + y_fake ** 2) ** 0.5)
                self.data.append([0, 0, x_fake, y_fake, "Timeout", pose_diff, cpu_before, psutil.cpu_percent(), ram_before, psutil.virtual_memory().percent])
                return "Fail", "Timeout"
            rospy.loginfo(f"⏳ Waiting for reliability... (Current: {self.reliability:.6f})")
            time.sleep(0.1)
        return "Success", (time.time() - start_relocalization) * 1000  # Convert to milliseconds

    def run_tests(self, num_tests=10):
        for _ in range(num_tests):
            self.wait_for_reliability(0, 0, 0, 0)  # Ensure starting reliability
            
            x_fake = random.uniform(-1.0, 1.0)
            y_fake = random.uniform(-1.0, 1.0)
            
            cpu_before = psutil.cpu_percent()
            ram_before = psutil.virtual_memory().percent
            
            self.publish_fake_pose(x_fake, y_fake)
            self.relocalized.clear()
            
            status, relocalization_time = self.wait_for_reliability(x_fake, y_fake, cpu_before, ram_before)
            
            if status == "Success":
                pose_diff = ((x_fake ** 2 + y_fake ** 2) ** 0.5)
                self.data.append([0, 0, x_fake, y_fake, relocalization_time, pose_diff, cpu_before, psutil.cpu_percent(), ram_before, psutil.virtual_memory().percent])
                rospy.loginfo(f"✅ Test Complete - {status}: Time={relocalization_time:.2f} ms, CPU={psutil.cpu_percent()}%, RAM={psutil.virtual_memory().percent}%")
            
            time.sleep(1)
        
        self.save_data()

    def save_data(self):
        with open("fake_pose_results.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["X_true", "Y_true", "X_Fake", "Y_Fake", "Re-Localize_time (ms)", "Pose Diff", "CPU Before", "CPU After", "RAM Before", "RAM After"])
            for row in self.data:
                writer.writerow(row)
        rospy.loginfo("✅ Data saved to fake_pose_results.csv")

if __name__ == "__main__":
    tester = FakePoseTester()
    tester.run_tests(num_tests=10)
