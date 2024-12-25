#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # ดึงข้อมูลจาก msg
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    linear_velocity = msg.twist.twist.linear
    angular_velocity = msg.twist.twist.angular

    rospy.loginfo(f"Position: x={position.x}, y={position.y}, z={position.z}")
    rospy.loginfo(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
    rospy.loginfo(f"Linear Velocity: x={linear_velocity.x}, y={linear_velocity.y}, z={linear_velocity.z}")
    rospy.loginfo(f"Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")

def odom_listener():
    rospy.init_node('odom_listener', anonymous=True)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
