#!/usr/bin/env python

import cv2
import math
import numpy as np
import rospy
import threading
import time

from cv_bridge import CvBridge # Change ros image into opencv
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension as Dimension

'''
Depth 이미지 수신해서 상하좌우로 장애물이 몇 미터에 있는지 알려주는 노드

지금은 Dpeth Image에서 프로펠러 위치 파악하는데 활용하는 것이 좋을 듯
'''

class DepthUDLR:
    def __init__(self):
        rospy.init_node('pub_depth_udlr', anonymous=True)

        
        self.h = 480
        self.w = 640

        self.h_half = self.h // 2
        self.w_half = self.w // 2

        self.h_rad = math.radians(58)
        self.w_rad = math.radians(87)
        

        # Calculate rad for image calibration
        self.w_list = np.array([])
        self.h_list = np.array([])

        for i in range(self.h):
            h = self.h_half - i
            if h <= 0:
                h -= 1
            
            rad_h = math.atan((math.tan(self.h_rad / 2) * h) / self.h_half)

            self.h_list = np.append(self.h_list, rad_h)
        
        for i in range(self.w):
            w = i - self.w_half
            if w >= 0:
                w += 1
            
            rad_w = math.atan((math.tan(self.w_rad / 2) * w) / self.w_half)

            self.w_list = np.append(self.w_list, rad_w)


        # Change image
        self.bridge = CvBridge()

        # Subscriber
        rospy.Subscriber('/red/camera/depth/image_raw', Image, self.image_callback_depth)

        # # Depth data publisher
        self.pub_udlr = rospy.Publisher('/carrot_team/udlr', UInt8MultiArray, queue_size = 10)

        rospy.spin()
        # t = threading.Thread(target = self.ros_spin)
        # t.start()
    
    def get_dist(self, d, w, h, w_all = 640, h_all = 480, rad_cam_w = math.radians(87), rad_cam_h = math.radians(58)): # 앞에거가 원래 58이었음.
        rad_w = self.w_list[w]
        rad_h = self.h_list[h]

        dist_x = d * math.tan(rad_w)
        dist_y = d
        dist_z = d * math.tan(rad_h)

        return dist_x, dist_y, dist_z
    
    def image_callback_depth(self, msg):
        # get depth image
        tmp = np.array(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')) * 1

        # Array for publishing
        bool_array = [1, 1, 1, 1, 1] # Up, Down, Left, Right, Front

        d_lim = 2
        d_lim_2 = 1.5
        lim = 1

        # print(np.shape(tmp)) # 480, 640
        for i in range(0, 479, 15):
            for j in range(0, 639, 16):
                if (i < 60 and j > 100 and j < 177) or (i > 580 and j > 100 and j < 177):
                    pass
                else:
                    w, d, h = self.get_dist(tmp[i][j], j, i)
                    if d <= d_lim:
                        if 0 <= h <= lim:
                            bool_array[0] = 0

                        if -1 * lim <= h <= -0:
                            bool_array[1] = 0

                        if -1 * lim <= w <= -0:
                            bool_array[2] = 0
                        
                        if 0 <= w <= lim:
                            bool_array[3] = 0
                        
                        if d <= d_lim_2:
                            # Front
                            if -0.5 <= h <= 0.5:
                                if -0.5 <= w <= 0.5:
                                    bool_array[4] = 0


        # Publish data
        bool_pub = UInt8MultiArray()
        bool_pub.data = bool_array

        bool_pub.layout.dim.append(Dimension(label="rows", size=1, stride=5))
        bool_pub.layout.dim.append(Dimension(label="cols", size=5, stride=1))

        print(bool_array)
        self.pub_udlr.publish(bool_pub)


        # For imshow        
        self.img_depth = tmp
        self.img_depth[np.isnan(tmp)] = 10.0
        self.img_depth /= 10
        self.img_depth[100, :] = 0
        # self.img_depth[145, :] = 0
        self.img_depth[177, :] = 0
        self.img_depth[:, 60] = 0
        self.img_depth[:, 580] = 0

        cv2.imshow('Depth image', self.img_depth)
        cv2.waitKey(25)
        
    def ros_spin(self):
        rospy.spin()

if __name__ == "__main__":
    dp = DepthUDLR()
