#!/usr/bin/env python

import cv2
# import matplotlib
# matplotlib.use('Agg')
# https://devpress.csdn.net/python/63045c767e6682346619a830.html
import math
import matplotlib.pyplot as plt
# plt.switch_backend('agg')
import numpy as np
import rospy
import threading

from cv_bridge import CvBridge # Change ros image into opencv
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import Float64 # get yaw

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

bridge = CvBridge() # Get drone image
dist_mid = np.array([])

def image_callback_depth(msg):

    # get depth image
    img_depth = np.array(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
    
    h_half = 480 // 2
    global dist_mid
    dist_mid = img_depth[h_half, :]

    for i, n in enumerate(dist_mid):
        isNaN = np.isnan(n)
        if isNaN:
            dist_mid[i] = 10
    
    # global plt
    # plt.plot(dist_mid)
    # plt.show()
    # plt.savefig(f'./line_plot.jpg', dpi=300)
    # img = cv2.imread(f'./line_plot.jpg')
    # cv2.imshow('image', img)

def yaw_rad(msg):
    print(msg)

rospy.init_node('mapping_node', anonymous=True)

rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback_depth)
rospy.Subscriber('/red/carrot/yaw', Float64, yaw_rad)
def ros_spin():
    rospy.spin()

t = threading.Thread(target = ros_spin)
t.start()

while True:
    deg_cam = 58 # 가로 화각
    # rad_cam = deg_cam * math.pi / 180
    rad_cam = math.radians(deg_cam)
    rad_cam_half = rad_cam / 2
    w = 640
    w_half = 640 // 2

    arr_x = np.array([])
    arr_y = np.array([])

    for i, d in enumerate(dist_mid):
        n = i - 320
        
        dist_x = (d * n) / (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2)**0.5
        dist_y = (d * w_half) / (math.tan(rad_cam_half) * (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2) ** 0.5)

        # if d == 10:
        #     dist_y = 0

        arr_x = np.append(arr_x, dist_x)
        arr_y = np.append(arr_y, dist_y)

    # plt.subplot(2,1,1)
    # plt.plot(dist_mid)
    # plt.title('Original')

    # plt.subplot(2,1,2)
    # # plt.plot(arr_x, arr_y)
    # plt.scatter(arr_x, arr_y)
    # plt.title('Converted')

    
    plt.subplot(3,1,1)
    plt.plot(dist_mid)
    plt.title('Original')

    plt.subplot(3,1,2)
    # plt.plot(arr_x, arr_y)
    plt.plot(arr_x, arr_y)
    plt.title('Converted_line')

    plt.subplot(3,1,3)
    # plt.plot(arr_x, arr_y)
    plt.scatter(arr_x, arr_y)
    plt.title('Converted_dot')

    plt.show()

# 드론의 현재 position 받기
