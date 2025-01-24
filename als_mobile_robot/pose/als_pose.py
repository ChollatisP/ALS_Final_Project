#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import math

# Variables to store poses
mcl_pose = None
base_link_pose = None

# Robot name in Gazebo
ROBOT_NAME = "/"  # Replace with your actual robot's name if changed

# Callback function to get MCL pose
def mcl_pose_callback(msg):
    global mcl_pose
    mcl_pose = msg.pose  # Adjusted for PoseStamped
    # No logging here, as per your requirement

# Callback function to get ground truth (base_link in Gazebo)
def ground_truth_callback(msg):
    global base_link_pose
    try:
        # Find index of the robot in the Gazebo model states
        index = msg.name.index(ROBOT_NAME)
        base_link_pose = msg.pose[index]
    except ValueError:
        rospy.logwarn(f"Model {ROBOT_NAME} not found in Gazebo model states.")

# Function to calculate Euclidean distance
def calculate_distance(pose1, pose2):
    dx = pose1.position.x - pose2.position.x
    dy = pose1.position.y - pose2.position.y
    return math.sqrt(dx**2 + dy**2)

# Compare MCL and ground truth poses
def compare_poses():
    if mcl_pose and base_link_pose:
        # Calculate distance
        distance = calculate_distance(mcl_pose, base_link_pose)

        # Calculate yaw for both poses
        yaw_mcl = euler_from_quaternion([
            mcl_pose.orientation.x,
            mcl_pose.orientation.y,
            mcl_pose.orientation.z,
            mcl_pose.orientation.w
        ])[2]
        yaw_base_link = euler_from_quaternion([
            base_link_pose.orientation.x,
            base_link_pose.orientation.y,
            base_link_pose.orientation.z,
            base_link_pose.orientation.w
        ])[2]

        # Log the results
        rospy.loginfo("MCL Pose: [x: %f, y: %f, yaw: %f]", mcl_pose.position.x, mcl_pose.position.y, yaw_mcl)
        rospy.loginfo("Base Link Pose: [x: %f, y: %f, yaw: %f]", base_link_pose.position.x, base_link_pose.position.y, yaw_base_link)
        rospy.loginfo("Distance between MCL and Base Link: %f", distance)
    else:
        rospy.loginfo("Waiting for both poses to be initialized...")

# Listener function to initialize the node and run comparisons
def listener():
    rospy.init_node('pose_comparator', anonymous=True)

    # Subscribe to topics
    rospy.Subscriber("/mcl_pose", PoseStamped, mcl_pose_callback)  # Adjusted for PoseStamped
    rospy.Subscriber("/gazebo/model_states", ModelStates, ground_truth_callback)

    # Run comparison at 1 Hz
    rospy.loginfo("Pose comparator is running...")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        compare_poses()
        rate.sleep()

if __name__ == '__main__':
    listener()
