#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import threading
import time

from cv_bridge import CvBridge # Change ros image into opencv
from sensor_msgs.msg import Image # Subscribe image

'''
Depth 이미지 수신해서 상하좌우로 장애물이 몇 미터에 있는지 알려주는 노드

지금은 Dpeth Image에서 프로펠러 위치 파악하는데 활용하는 것이 좋을 듯
'''

class DepthUDLR:
    def __init__(self):
        rospy.init_node('pub_depth_udlr', anonymous=True)

        # Change image
        self.bridge = CvBridge()

        # Subscriber
        rospy.Subscriber('/red/camera/depth/image_raw', Image, self.image_callback_depth)

        # # Depth data publisher
        # self.pub_left = rospy.Publisher('/carrot_team/depth_l', DataType, queue_size = 10)
        # self.pub_right = rospy.Publisher('/carrot_team/depth_r', DataType, queue_size = 10)
        # self.pub_up = rospy.Publisher('/carrot_team/depth_u', DataType, queue_size = 10)
        # self.pub_down = rospy.Publisher('/carrot_team/depth_d', DataType, queue_size = 10)


        rospy.spin()
        # t = threading.Thread(target = self.ros_spin)
        # t.start()

        # print('Sleep 3s')
        # time.sleep(3)
        # cv2.imshow('Depth image', self.img_depth)
        # cv2.waitKey(25)
    
    def image_callback_depth(self, msg):
        # get depth image
        tmp = np.array(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')) * 1
        
        self.img_depth = tmp
        self.img_depth[np.isnan(tmp)] = 10.0
        self.img_depth /= 10

        # for i in range(len(self.img_depth)):
        #     for j in range(len(self.img_depth[i])):
        #         if np.isnan(self.img_depth[i][j]):
        #             print('NaN')
                    # self.img_depth[i][j] = 10.0
                # else:
                #     self.img_depth[i][j] /= 10

        cv2.imshow('Depth image', self.img_depth)
        cv2.waitKey(25)




    def ros_spin(self):
        rospy.spin()

if __name__ == "__main__":
    dp = DepthUDLR()
