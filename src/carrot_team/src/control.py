#!/usr/bin/env python

import time
import rospy
from geometry_msgs.msg import PoseStamped


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

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
    x, y, z, yaw = map(int, input('Type in x, y, z, yaw:').split(' '))
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z

    # quaternion_list
    q = get_quaternion_from_euler(0, 0, yaw)
    pose_msg.pose.orientation.x = q[0]  # set the x orientation
    pose_msg.pose.orientation.y = q[1]  # set the y orientation
    pose_msg.pose.orientation.z = q[2]  # set the z orientation
    pose_msg.pose.orientation.w = q[3]  # set the w orientation

    print('move to : ', x, y, z, yaw)

    pose_publisher.publish(pose_msg)
    # print(time.time())
    # rate.sleep()