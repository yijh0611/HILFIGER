#!/usr/bin/env python

import cv2
import math
import numpy as np
import os
import rospy
import threading
import time

from cv_bridge import CvBridge # Change ros image into opencv
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray, MultiArrayDimension # Subscribe map in 3D Array
from std_msgs.msg import Float32MultiArray

class Search:
    def __init__(self):
        self.poi = np.array([])

        rospy.init_node('search_poi', anonymous=True)
        rospy.Subscriber('/carrot_team/poi', Float32MultiArray, self.get_poi)
        rospy.Subscriber('/red/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/carrot_team/is_poi_ready', Bool, self.is_poi_callback)
        # Get global map
        rospy.Subscriber('/carrot_team/map', Int32MultiArray, self.global_map_callback)

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

        # is poi ready
        self.is_poi = False
        self.is_get_poi_end = False

        # number of poi
        self.no_poi = 10

        # image
        self.bridge = CvBridge() # Get drone image
        self.src = '/root/pic'
        if not os.path.isdir(self.src):
            os.mkdir(self.src)
        self.count = 0

        t = threading.Thread(target = self.ros_spin)
        t.start

    def get_poi(self, msg):
        
        tmp = np.reshape(msg.data, (-1, 3))
        self.poi = tmp
        
        
        print(self.poi)
        print()
        print(np.shape(self.poi))

        if len(self.poi) > 9:
            self.is_get_poi_end = True
    
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
    
    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def is_poi_callback(self, msg):
        self.is_poi = msg.data
    
    def global_map_callback(self, msg):
        height = msg.layout.dim[0].size
        width = msg.layout.dim[1].size
        depth = msg.layout.dim[2].size

        array_flat = msg.data

        # array_3d = [msg.data[i:i + depth] for i in range(0, len(msg.data), depth)]
        # print(np.shape(array_3d))
        # array_3d = [[array_3d[i * height + j] for j in range(height)] for i in range(width)]
        # print(np.shape(array_3d))
        # print()
        # print('Height : ', height)
        # print('Width : ', width)
        # print('Depth : ', depth)

        # self.map_np = np.array(array_3d) # init에서 정의 안하고 여기서 해도 되는건가?
        # print(np.shape(array_3d))

        self.map_np = np.reshape(array_flat, (height, width, depth))

if __name__ == "__main__":
    # call class
    ctrl = Search()

    wait_time = 5
    time_start = time.time()
    # Wait until poi is recieved
    while ctrl.is_poi == False:
        time.sleep(0.1)
        if time.time() - time_start > 40:
            # When 40s is passed
            print("No POI!!! ERROR!!!")
            exit() # 이 기능은 최종적으로는 빼는게 좋을 듯 하다.
            # break

    # request poi
    get_poi = Int32()
    tmp = ctrl.no_poi + 1
    get_poi.data = int(tmp)

    # 오래동안 publish 안했으면 처음거는 버려지기 때문에 2개를 Publish 해야 정보를 받을 수 있다.
    ctrl.pub_get_poi.publish(get_poi)
    
    # wait until getting poi ends
    while ctrl.is_get_poi_end == False:
        time.sleep(1)
        print('Waiting')


    for i in range(len(ctrl.poi)):
        if ctrl.poi[i][1] == 11.0:
            first_idx = i # ctrl.poi[i]
            print(ctrl.poi[first_idx])
            break

    print('Move to POI')
    ctrl.pose_msg.pose.position.x = ctrl.poi[first_idx, 0]
    ctrl.pose_msg.pose.position.y = ctrl.poi[first_idx, 1]
    ctrl.pose_msg.pose.position.z = ctrl.poi[first_idx, 2]
    
    ctrl.pose_publisher.publish(ctrl.pose_msg)

    time.sleep(wait_time + 3)
    # print Global map
    # print(ctrl.map_np[int(ctrl.pose_msg.pose.position.z)])
    # print(ctrl.map_np[:,:,2])
    print(ctrl.map_np[5:10,5:15,2])
    # print(np.shape(ctrl.map_np[2]))
    # print(np.sum(ctrl.map_np[:,:,2]))
    print(np.shape(ctrl.map_np))

    # imshow
    cv2.imshow('POI', ctrl.img)
    cv2.imwrite(f'{ctrl.src}/img_{ctrl.count}.jpg', ctrl.img)
    ctrl.count += 1
    cv2.waitKey(25)

    res = math.radians(45)
    for i in range(8):
        yaw = res * i
        ctrl.get_quaternion_from_euler(0, 0, yaw)

        ctrl.pose_publisher.publish(ctrl.pose_msg)
        time.sleep(wait_time)

        # imshow
        cv2.imshow('POI', ctrl.img)
        cv2.imwrite(f'{ctrl.src}/img_{ctrl.count}.jpg', ctrl.img)
        ctrl.count += 1
        cv2.waitKey(25)
    
    if ctrl.poi[first_idx, 2] >= 2.5: # tmp -> ctrl.poi
        ctrl.pose_msg.pose.position.z = ctrl.poi[first_idx, 2] - 2

        for i in range(8):
            yaw = res * i
            ctrl.get_quaternion_from_euler(0, 0, yaw)

            ctrl.pose_publisher.publish(ctrl.pose_msg)
            time.sleep(wait_time)

            # imshow
            cv2.imshow('POI', ctrl.img)
            cv2.imwrite(f'{ctrl.src}/img_{ctrl.count}.jpg', ctrl.img)
            ctrl.count += 1
            cv2.waitKey(25)
        
    if ctrl.poi[first_idx, 2] <= 12.5:
        ctrl.pose_msg.pose.position.z = ctrl.poi[first_idx, 2] + 2

        for i in range(8):
            yaw = res * i
            ctrl.get_quaternion_from_euler(0, 0, yaw)

            ctrl.pose_publisher.publish(ctrl.pose_msg)
            time.sleep(3)

            # imshow
            cv2.imshow('POI', ctrl.img)
            cv2.imwrite(f'{ctrl.src}/img_{ctrl.count}.jpg', ctrl.img)
            ctrl.count += 1
            cv2.waitKey(25)
    
    if ctrl.poi[first_idx, 2] <= 10.5:
        ctrl.pose_msg.pose.position.z = ctrl.poi[first_idx, 2] + 4

        for i in range(8):
            yaw = res * i
            ctrl.get_quaternion_from_euler(0, 0, yaw)

            ctrl.pose_publisher.publish(ctrl.pose_msg)
            time.sleep(wait_time)

            # imshow
            cv2.imshow('POI', ctrl.img)
            cv2.imwrite(f'{ctrl.src}/img_{ctrl.count}.jpg', ctrl.img)
            ctrl.count += 1
            cv2.waitKey(25)
        
    cv2.destroyAllWindows()