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
import time

from cv_bridge import CvBridge # Change ros image into opencv
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import Float64 # get yaw

# 할일 !!!!
# 클래스 형태로 바꿔서 전역변수 없애기

# 시작할때는 초기값 설정
# map_np = np.zeros((16, 51)) # 드론이 시작할때 바라보는 방향은 x이다. 드론의 왼쪽이 y이다.
map_np = np.zeros((16, 51, 26))
map_img = np.zeros((16, 51, 26, 3))

# 0은 미탐색, 1은 갈 수 있음, 2는 갈 수 없음
# 색칠할때는
# 검은색 미탐색, 흰색 갈 수 있음, 빨간색 갈 수 없음

# 맵의 모양
'''
15*50*25
드론이 바라보고 있는 방향이 x (15m)
드론의 왼쪽이 y 방향
높이가 25m이다.

초기 값은 10, 2, 2이다.

맵의 크기는 0도 포함해서 16 * 51 * 26으로
현재는 2D 이기 때문에 16 * 51으로 만들 계획이다.


'''

bridge = CvBridge() # Get drone image
# dist_mid = np.array([])
img_depth = np.zeros((480,640))
drone_yaw = 0
drone_pose = np.array([0, 0, 0])
time_is_map = time.time()

def image_callback_depth(msg):

    # get depth image
    tmp = np.array(bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')) * 1
    
    global img_depth
    img_depth = tmp
    img_depth[np.isnan(tmp)] = 10.0

def yaw_rad(msg):
    global drone_yaw
    if msg.data != drone_yaw:
        global time_is_map
        time_is_map = time.time()
    drone_yaw = msg.data

def get_pose(msg):
    global drone_pose
    tmp = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    check = False
    for i in range(3):
        if drone_pose[i] != tmp[i]:
            check = True

    if check == True:
        global time_is_map
        time_is_map = time.time()

    drone_pose = tmp

def get_dist(d, n, z, w = 640, h = 480, rad_cam_w = math.radians(87), rad_cam_h = math.radians(58)): # 앞에거가 원래 58이었음.

    w_half = w // 2
    h_half = h // 2

    rad_w = math.atan((math.tan(rad_cam_w / 2) * n) / w_half)
    rad_h = math.atan((math.tan(rad_cam_h / 2) * z) / h_half)

    dist_x = d * math.tan(rad_w)
    dist_y = d
    dist_z = d * math.tan(rad_h)

    return dist_x, dist_y, dist_z

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
    width = 640
    height = 480
    w_half = width // 2
    h_half = height // 2

    wall_x = np.array([])
    wall_y = np.array([])
    wall_z = np.array([])

    open_x = np.array([])
    open_y = np.array([])
    open_z = np.array([])

    # Rotation matrix
    rot = np.array([[math.cos(drone_yaw), -1 * math.sin(drone_yaw)],[math.sin(drone_yaw), math.cos(drone_yaw)]])
    
    # for i in range(60,420):
    for i in range(200, 300):
        for j in range(64, 576):
            h = h_half - i
            if h <= 0:
                h -= 1
            w = j - w_half
            if w >= 0:
                w += 1
            
            d = img_depth[i][j]

            dist_x, dist_y, dist_z = get_dist(img_depth[i][j], w, h)

            dist_x_rot, dist_y_rot = rot.dot(np.array([dist_x, dist_y]).T) # 원래 매핑 상태와 맞게 매칭한 그래프

            isNaN = np.isnan(d)
            if is_NaN:
                d = 10
                
            elif d != 10:
                wall_x = np.append(wall_x, dist_x_rot)
                wall_y = np.append(wall_y, dist_y_rot)
                wall_z = np.append(wall_z, dist_z)

            # Open space mapping
            if i == 240:
                n = 5 # Resolution (How many)
                for k in range(1,n + 1):
                    open_x = np.append(open_x, dist_x_rot * k / n)
                    open_y = np.append(open_y, dist_y_rot * k / n)
                    open_z = np.append(open_z, dist_z * k / n)
                    # 1m 간격으로 Plot 하는 방법 생각해보기


    # global mapping # 3D 형태로 수정 필요
    if time.time() - time_is_map > 0.5 and (len(wall_x) > 0 or len(open_x) > 0):
        # 0.5초 이상 yaw의 변화가 없었을 때 매핑을 한다.
        # 지금은 매핑 되어있지 않은 곳에만 매핑을 한다.
        for i in range(len(wall_x)):
            map_x = int(drone_pose[0] + wall_y[i]) # !! 드론에 더 가까운 쪽으로 벽을 만들 필요가 있기 때문에, 그냥 int를 쓰면 안되고 상황에 따라서 +- 1을 해야한다. - 일단 맵이 어떻게 되는지 확인 후 기능 추가
            map_y = int(drone_pose[1] - wall_x[i])
            map_z = int(drone_pose[2] + wall_z[i])
            
            try:
                if map_np[map_x, map_y, map_z] == 0:
                    map_np[map_x, map_y, map_z] = 2 # 갈 수 없음
                    map_img[16 - map_x, 51- map_y, map_z, 2] = 125 # 빨간색
            except:
                pass
        
        for i in range(len(open_x)):
            map_x = int(drone_pose[0] + open_y[i]) # !! 여기서도 문제가 있을 수도 있으니 결과 보고 수정 필요하면 수정하기.
            map_y = int(drone_pose[1] - open_x[i])

            try:
                if map_np[map_x, map_y] == 0:
                    map_np[map_x, map_y] = 1 # 갈 수 있음
                    map_img[16 - map_x, 51 - map_y, map_z, :] = 125
            except:
                pass

    mul = 20
    img = cv2.resize(map_img[:, :, int(drone_pose[2]), :], dsize = (51 * mul, 16 * mul))
    cv2.imshow('global map', img)
    key = cv2.waitKey(10)

    if key == ord('q'):
        break

    # # 갈 수 있는 곳과 갈 수 없는 곳 둘다 매핑해서 Plot 하는 부분

    # plt.subplot(2,1,1)
    # plt.plot(wall_x, wall_y)
    # plt.grid(True)
    # plt.title('Converted_line')
    plt.subplot(2,1,1)
    fig = plt.figure()
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(wall_x, wall_y, wall_z)
    # plt.grid(True)
    plt.title('3D')

    plt.subplot(2,1,2)
    plt.scatter(open_x, open_y)
    plt.scatter(wall_x, wall_y)
    plt.grid(True)
    plt.scatter(0, 0)
    plt.title('2D')

    plt.show()

cv2.destroyAllWindows()