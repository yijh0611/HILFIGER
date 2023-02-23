#!/usr/bin/env python

# 

import time
import rospy
from geometry_msgs.msg import PoseStamped

# Initialize the ROS node
rospy.init_node('control', anonymous = True)

# Create a publisher object
pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)

# Create a PoseStamped message
pose_msg = PoseStamped()
# pose_msg.header.frame_id = ""  # set the frame ID
pose_msg.pose.position.x = 2.0  # set the x position
pose_msg.pose.position.y = 2.0  # set the y position
pose_msg.pose.position.z = 10.0  # set the z position
pose_msg.pose.orientation.x = 0.0  # set the x orientation
pose_msg.pose.orientation.y = 0.0  # set the y orientation
pose_msg.pose.orientation.z = 0.0  # set the z orientation
pose_msg.pose.orientation.w = 1.0  # set the w orientation

# Publish the message
# rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    x, y, z = input('Type in x, y, z :')
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    print('move to : ', x, y, z)

    pose_publisher.publish(pose_msg)
    # print(time.time())
    # rate.sleep()