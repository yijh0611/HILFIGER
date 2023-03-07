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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import Float64 # get yaw

# 할일 !!!!
# 클래스 형태로 바꿔서 전역변수 없애기

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
drone_yaw = 0
drone_pose = np.array([])

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
    # print(msg)
    # print(msg.data)
    global drone_yaw
    drone_yaw = msg.data

def get_pose(msg):
    global drone_pose
    drone_pose = msg.pose.position
    print(drone_pose)

def get_dist(d, n, w = 640, rad_cam = math.radians(58)):
    w_half = w // 2
    rad_cam_half = rad_cam / 2

    dist_x = (d * n) / (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2)**0.5
    dist_y = (d * w_half) / (math.tan(rad_cam_half) * (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2) ** 0.5)

    return dist_x, dist_y

def get_r(d, n, w = 640, rad_cam = math.radians(58)):
    w_half = w // 2
    rad_cam_half = rad_cam / 2

    r_x = (n) / (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2)**0.5
    r_y = (w_half) / (math.tan(rad_cam_half) * (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2) ** 0.5)

    return r_x, r_y

rospy.init_node('mapping_node', anonymous=True)

rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback_depth)
rospy.Subscriber('/red/carrot/yaw', Float64, yaw_rad) # /red/uav/yaw 도 있는데, 값이 크게 차이나지는 않는거 같아서 그냥 이거 썼다.
rospy.Subscriber('/red/carrot/pose', PoseStamped, get_pose)

def ros_spin():
    rospy.spin()

t = threading.Thread(target = ros_spin)
t.start()

while True:
    # deg_cam = 58 # 가로 화각
    # # rad_cam = deg_cam * math.pi / 180
    # rad_cam = math.radians(deg_cam)
    # rad_cam_half = rad_cam / 2
    w = 640
    w_half = 640 // 2

    wall_x = np.array([])
    wall_y = np.array([])

    open_x = np.array([])
    open_y = np.array([])

    # Rotation matrix
    rot = np.array([[math.cos(drone_yaw), -1 * math.sin(drone_yaw)],[math.sin(drone_yaw), math.cos(drone_yaw)]])
    # print(rot)

    for i, d in enumerate(dist_mid):
        if (64 < i) and (i < 576):
            n = i - (w_half - 1) # 처음 시작하는 값을 -319으로 만들기 위함.
            
            if d != 10: # 뚫려있을때는 안하기
                # dist_x = (d * n) / (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2)**0.5
                # dist_y = (d * w_half) / (math.tan(rad_cam_half) * (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2) ** 0.5)
                dist_x, dist_y = get_dist(d, n)

                dist_x_rot, dist_y_rot = rot.dot(np.array([dist_x, dist_y]).T) # 원래 매핑 상태와 맞게 매칭한 그래프

                wall_x = np.append(wall_x, dist_x_rot)
                wall_y = np.append(wall_y, dist_y_rot)
            else:
                dist_x, dist_y = get_dist(5, n)

            for j in range(1, 11):
                rx, ry = get_r(d, n)

                dx = rx * (d - j / ry)
                dy = ry * (d - j / ry)
                
                if dy <= 0:
                    break

                dist_x_rot, dist_y_rot = rot.dot(np.array([dx, dy]).T) # 원래 매핑 상태와 맞게 매칭한 그래프

                open_x = np.append(open_x, dist_x_rot)
                open_y = np.append(open_y, dist_y_rot)
            
            

    


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
    plt.plot(wall_x, wall_y)
    plt.title('Converted_line')

    plt.subplot(3,1,3)
    # plt.plot(arr_x, arr_y)
    plt.scatter(open_x, open_y)
    plt.scatter(wall_x, wall_y)
    plt.scatter(0, 0)
    plt.title('Converted_dot')

    plt.show()

# 드론의 현재 position 받기
