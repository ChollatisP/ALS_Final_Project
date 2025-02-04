#!/usr/bin/env python3

import rospy
import random
import csv
import time
import tf
from threading import Event
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

class RelocalizationTest:
    def __init__(self):
        rospy.init_node("relocalization_test", anonymous=True)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")

        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        
        self.robot_name = self.get_robot_name()
        if not self.robot_name:
            rospy.logerr("❌ No valid robot found. Exiting...")
            exit()
        
        self.data = []
        self.reliability_data = []  # Store reliability changes
        self.relocalized = Event()
        self.reliability = 0.0  
        self.x_true = 0.0
        self.y_true = 0.0
        self.x_wrong = 0.0
        self.y_wrong = 0.0
        self.relocalization_time = 0.0

        rospy.Subscriber("/reliability", Vector3Stamped, self.reliability_callback)
        rospy.loginfo("✅ Subscribed to /reliability topic...")

    def get_robot_name(self):
        models = ["/"]
        for model in models:
            response = self.get_model_state(model, "world")
            if response.success:
                rospy.loginfo(f"✅ Robot detected in Gazebo: {model}")
                return model
        rospy.logwarn("⚠️ No valid robot found in Gazebo! Check model name.")
        return None

    def move_robot_gazebo(self, x, y, yaw=0.0):
        if not self.robot_name:
            rospy.logerr("❌ Robot name not found. Cannot move robot!")
            return
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        state_msg = ModelState()
        state_msg.model_name = self.robot_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]

        self.set_model_state(state_msg)
        rospy.loginfo(f"🚀 Robot moved to: X={x}, Y={y}, Yaw={yaw}")

        def publish_initial_pose(self, x, y, yaw=0.0):
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]

    def reliability_callback(self, msg):
        self.reliability = msg.vector.x
        rospy.loginfo(f"🔹 Received Reliability: {self.reliability:.6f}")
        
        if self.reliability >= 0.0001:
            timestamp = time.time()
            self.reliability_data.append([timestamp, self.x_true, self.y_true, self.x_wrong, self.y_wrong, self.relocalization_time])
            self.save_reliability_data()
        
        if self.reliability >= 0.9999:
            self.relocalized.set()

    def save_reliability_data(self):
        with open("reliability_log.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "True X", "True Y", "Wrong X", "Wrong Y", "Relocalization Time (s)"])
            for row in self.reliability_data:
                writer.writerow(row)
        rospy.loginfo("✅ Reliability log updated in reliability_log.csv")

    def test_relocalization(self, num_tests=5):
        if not self.robot_name:
            rospy.logerr("❌ No valid robot found. Test aborted!")
            return

        for i in range(num_tests):
            rospy.loginfo(f"🔹 Running Test {i+1}/{num_tests}")
            self.x_true, self.y_true = random.uniform(-0.4, 0.4), random.uniform(-0.4, 0.4)
            self.move_robot_gazebo(self.x_true, self.y_true)
            time.sleep(2)

            self.x_wrong, self.y_wrong = self.x_true + random.uniform(-0.4, 0.4), self.y_true + random.uniform(-0.4, 0.4)
            self.publish_initial_pose(self.x_wrong, self.y_wrong)

            self.relocalized.clear()
            start_time = time.time()

            while not self.relocalized.is_set():
                time.sleep(0.1)
            self.relocalization_time = (time.time() - start_time) * 1000
            self.data.append([i+1, self.x_true, self.y_true, self.x_wrong, self.y_wrong, self.relocalization_time])
            rospy.loginfo(f"✅ Test {i+1} completed. Relocalization Time: {self.relocalization_time:.4f} ms")
            time.sleep(1)
        self.save_data()

    def save_data(self):
        with open("relocalization_results.csv", "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Test #", "True X", "True Y", "Wrong X", "Wrong Y", "Relocalization Time (s)"])
            for row in self.data:
                writer.writerow(row)
        rospy.loginfo("✅ Data saved to relocalization_results.csv")

if __name__ == "__main__":
    tester = RelocalizationTest()
    tester.test_relocalization(num_tests=10)
