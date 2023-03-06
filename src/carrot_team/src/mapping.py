#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image # Subscribe image

# # 시작할때는 초기값 설정
# map_np = np.zeros((26, 51)) # 드론이 처음에 바라 보는 방향이 x인지 y인지 확인 필요

# 맵의 모양
'''
25*50*15
드론이 바라보고 있는 방향이 x (25m)
드론의 왼쪽이 y 방향
높이가 15m이다.

초기 값은 10, 2, 2이다.

맵의 크기는 0도 포함해서 26 * 51 * 16으로
현재는 2D 이기 때문에 26 * 51으로 만들 계획이다.


'''

# 1차원 거리데이터가 제대로 나오는지 확인
# arr = np.zeros(640) # dist_mid
# print(arr)

def image_callback_depth(msg):

    # get depth image
    img_depth = bridge_2.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    h_half = 480 / 2
    dist_mid = img_depth_ori[h_half][:]

    for i, n in enumerate(dist_mid):
        isNaN = np.isnan(n)
        if isNaN:
            dist_mid[i] = 10
    
    plt.plot(dist_mid)
    plt.show()

rospy.init_node('Mapping node', anonymous=True)

rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback_depth)
rospy.spin()

# 드론의 현재 position 받기