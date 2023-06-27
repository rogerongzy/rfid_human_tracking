#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def send_twist_message():
    # Initialize the ROS node
    rospy.init_node('twist_publisher', anonymous=True)

    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a new Twist message
    twist_msg = Twist()

    # Set the linear velocity (x-axis)
    twist_msg.linear.x = 0.5  # Modify this value as needed

    # Set the angular velocity (z-axis)
    twist_msg.angular.z = 0.2  # Modify this value as needed

    # Publish the Twist message repeatedly at a rate of 10 Hz
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        cmd_vel_pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_twist_message()
    except rospy.ROSInterruptException:
        pass
