#!/usr/bin/env python
# # license removed for brevity
# import rospy
# # from std_msgs.msg import String
# from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# def talker():
#     pub = rospy.Publisher('/red/position_hold/trajectory', MultiDOFJointTrajectory, queue_size=10) # String 아닌거 같은데
#     rospy.init_node('control_pub_hak', anonymous=True)
#     rate = rospy.Rate(60) # 60hz
#     while not rospy.is_shutdown():
#         control_input = MultiDOFJointTrajectory()

#         control_input.joint_names = ["joint1", "joint2"]

#         point = MultiDOFJointTrajectoryPoint()
#         point.time_from_start = rospy.Duration(1.0)
#         point.transforms = [10, 2, 3]
#         control_input.points.append(point)

#         pub.publish(control_input)
#         rate.sleep()

# if __name__ == '__main__': 
#     try:
#         print(123123)
#         talker()
#     except rospy.ROSInterruptException:
#         pass

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
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    pose_publisher.publish(pose_msg)
    print(time.time())
    rate.sleep()