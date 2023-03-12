#!/usr/bin/env python

import numpy as np
import rospy
import threading
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

class Search:
    def __init__(self):
        self.poi = np.array([])

        rospy.init_node('search_poi', anonymous=True)
        rospy.Subscriber('/carrot_team/poi', Float32MultiArray, self.get_poi)

        self.pub_get_poi = rospy.Publisher('/carrot_team/req_poi', Int32, queue_size=10)
        # control message
        self.pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)

        # Control input
        self.pose_msg = PoseStamped()

        # initial pose
        self.pose_msg.pose.position.x = 10.0  # set the x position 범위 0 ~ 15
        self.pose_msg.pose.position.y = 2.0  # set the y position 범위 0 ~ 50
        self.pose_msg.pose.position.z = 2.0  # set the z position 범위 0 ~ 15
        self.pose_msg.pose.orientation.x = 0.0  # set the x orientation
        self.pose_msg.pose.orientation.y = 0.0  # set the y orientation
        self.pose_msg.pose.orientation.z = 0.0  # set the z orientation
        self.pose_msg.pose.orientation.w = 1.0  # set the w orientation

        self.yaw = 0

        t = threading.Thread(target = self.ros_spin)
        t.start

    
    def get_poi(self, msg):

        # get poi
        tmp = np.array([])
        tmp = np.append(tmp, msg.data[0])
        tmp = np.append(tmp, msg.data[1])
        tmp = np.append(tmp, msg.data[2])

        # save poi
        self.poi = np.append(self.poi, tmp)

        # # Print ("End message")
        # print(self.poi)
    
    def ros_spin(self):
        rospy.spin()
    
    def get_quaternion_from_euler(self, roll, pitch, yaw, is_send = True):
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
        
        if is_send:
            self.pose_msg.pose.orientation.x = qx
            self.pose_msg.pose.orientation.y = qy
            self.pose_msg.pose.orientation.z = qz
            self.pose_msg.pose.orientation.w = qw

        return [qx, qy, qz, qw]
    
if __name__ == "__main__":
    # call class
    ctrl = Search()

    # request poi
    get_poi = Int32()
    get_poi.data = 9

    for i in range(2):
        # 오래동안 publish 안했으면 처음거는 버려지기 때문에 2개를 Publish 해야 정보를 받을 수 있다.
        ctrl.pub_get_poi.publish(get_poi)
        time.sleep(0.1)

    # wait 10s
    time.sleep(3)

    print(np.shape(ctrl.poi))
    print(ctrl.poi)    

    print('Move to POI')
    ctrl.pose_msg.pose.position.x = ctrl.poi[0]
    ctrl.pose_msg.pose.position.y = ctrl.poi[1]
    ctrl.pose_msg.pose.position.z = ctrl.poi[2]
    
    ctrl.pose_publisher.publish(ctrl.pose_msg)
      