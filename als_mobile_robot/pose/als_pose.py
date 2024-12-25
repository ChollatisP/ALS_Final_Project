#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
import math

# Variables to store poses
mcl_pose = None
ground_truth_pose = None

def mcl_pose_callback(msg):
    global mcl_pose
    mcl_pose = msg.pose.pose
    rospy.loginfo("Received MCL Pose: [x: %f, y: %f, z: %f]", mcl_pose.position.x, mcl_pose.position.y, mcl_pose.position.z)

def ground_truth_callback(msg):
    global ground_truth_pose
    for i, name in enumerate(msg.name):
        rospy.loginfo("Model name found: %s", name)  # Debug each model name
        if name == "robot":  # Replace with correct robot name
            ground_truth_pose = msg.pose[i]
            rospy.log

# Function to calculate Euclidean distance
def calculate_distance(pose1, pose2):
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx**2 + dy**2)

def compare_poses():
    if mcl_pose and ground_truth_pose:
        distance = calculate_distance(mcl_pose, ground_truth_pose)
        rospy.loginfo("ALS Pose: [x: %f, y: %f, z: %f]", mcl_pose.position.x, mcl_pose.position.y, mcl_pose.position.z)
        rospy.loginfo("Ground Truth Pose: [x: %f, y: %f, z: %f]", ground_truth_pose.position.x, ground_truth_pose.position.y, ground_truth_pose.position.z)
        rospy.loginfo("ALS Pose vs Ground Truth Distance: %f", distance)
    else:
        rospy.loginfo("Waiting for poses to be initialized...")

def listener():
    rospy.init_node('pose_comparator', anonymous=True)

    # Subscribe to topics
    rospy.Subscriber("/mcl_pose", PoseWithCovarianceStamped, mcl_pose_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, ground_truth_callback)

    # Run comparison every 1 second
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        compare_poses()
        rate.sleep()

if __name__ == '__main__':
    listener()
