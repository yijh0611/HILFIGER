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
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import Float64 # get yaw


class mapping:
    def __init__(self):
        # Set variables
        self.map_np = np.zeros((16, 51, 26)) # x : Initial drone's orientation, y : left
        self.map_img = np.zeros((16, 51, 26, 3))
        # 0 : Unknown, 1 : Open, 2 : Wall
        # Color :
        # Black : Unknown, White : Open, Red : Wall
                
        # Shape of map (Resolution 1m)
        '''
        15*50*25
        x : Front of drone
        y : left side of drone
        Height : 25m

        Initial position : 10, 2, 2

        맵의 크기는 0도 포함해서 16 * 51 * 26으로
        현재는 2D 이기 때문에 16 * 51으로 만들 계획이다.

        '''
        self.bridge = CvBridge() # Get drone image
        self.img_depth = np.zeros((480,640))
        self.drone_yaw = 0
        self.drone_pose = np.array([0, 0, 0])
        self.time_is_map = time.time()

        # get radian - To reduce calculation
        self.w = 640
        self.h = 480

        self.w_half = self.w // 2
        self.h_half = self.h // 2

        self.w_rad = math.radians(87)
        self.h_rad = math.radians(58)

        self.w_list = np.array([])
        self.h_list = np.array([])

        # rad_w = math.atan((math.tan(rad_cam_w / 2) * n) / w_half)
        # rad_h = math.atan((math.tan(rad_cam_h / 2) * z) / h_half)

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
        
        
        rospy.init_node('mapping_node', anonymous=True)

        rospy.Subscriber('/red/camera/depth/image_raw', Image, self.image_callback_depth)
        rospy.Subscriber('/red/carrot/yaw', Float64, self.yaw_rad) # /red/uav/yaw 도 있는데, 값이 크게 차이나지는 않는거 같아서 그냥 이거 썼다.
        rospy.Subscriber('/red/carrot/pose', PoseStamped, self.get_pose)

        t = threading.Thread(target = self.ros_spin)
        t.start()
    
    def ros_spin(self):
        rospy.spin()

    def image_callback_depth(self, msg):

        # get depth image
        tmp = np.array(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')) * 1
        
        self.img_depth = tmp
        self.img_depth[np.isnan(tmp)] = 10.0

    def yaw_rad(self, msg):
        if msg.data != self.drone_yaw:
            self.time_is_map = time.time()

        self.drone_yaw = msg.data

    def get_pose(self, msg):
        tmp = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        check = False
        for i in range(3):
            if self.drone_pose[i] != tmp[i]:
                check = True

        if check == True:
            self.time_is_map = time.time()

        self.drone_pose = tmp

    def get_dist(self, d, w, h, w_all = 640, h_all = 480, rad_cam_w = math.radians(87), rad_cam_h = math.radians(58)): # 앞에거가 원래 58이었음.

        # w_half = w // 2
        # h_half = h // 2

        # rad_w = math.atan((math.tan(rad_cam_w / 2) * n) / w_half)
        # rad_h = math.atan((math.tan(rad_cam_h / 2) * z) / h_half)

        rad_w = self.w_list[w]
        rad_h = self.h_list[h]

        dist_x = d * math.tan(rad_w)
        dist_y = d
        dist_z = d * math.tan(rad_h)

        return dist_x, dist_y, dist_z

    def get_r(self, d, n, w = 640, rad_cam = math.radians(58)):
        w_half = w // 2
        rad_cam_half = rad_cam / 2

        r_x = (n) / (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2)**0.5
        r_y = (w_half) / (math.tan(rad_cam_half) * (n ** 2 + (w_half / math.tan(rad_cam_half)) ** 2) ** 0.5)

        return r_x, r_y


if __name__ == "__main__" :
    mp = mapping()

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
        rot = np.array([[math.cos(mp.drone_yaw), -1 * math.sin(mp.drone_yaw)],[math.sin(mp.drone_yaw), math.cos(mp.drone_yaw)]])
        
        for i in range(60,420):
        # for i in range(200, 300):
            for j in range(64, 576):
                # h = h_half - i
                # if h <= 0:
                #     h -= 1
                # w = j - w_half
                # if w >= 0:
                #     w += 1
                
                d = mp.img_depth[i][j]
                isNaN = np.isnan(d)
                if isNaN:
                    print('isNaN')
                    d = 10

                # dist_x, dist_y, dist_z = get_dist(img_depth[i][j], w, h)
                dist_x, dist_y, dist_z = mp.get_dist(d, j, i) # w, h

                dist_x_rot, dist_y_rot = rot.dot(np.array([dist_x, dist_y]).T) # 원래 매핑 상태와 맞게 매칭한 그래프

                if d != 10:
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
        if time.time() - mp.time_is_map > 0.5 and (len(wall_x) > 0 or len(open_x) > 0):
            print('Global mapping')
            # 0.5초 이상 yaw의 변화가 없었을 때 매핑을 한다.
            # 지금은 매핑 되어있지 않은 곳에만 매핑을 한다.
            for i in range(len(wall_x)):
                map_x = int(mp.drone_pose[0] + wall_y[i]) # !! 드론에 더 가까운 쪽으로 벽을 만들 필요가 있기 때문에, 그냥 int를 쓰면 안되고 상황에 따라서 +- 1을 해야한다. - 일단 맵이 어떻게 되는지 확인 후 기능 추가
                map_y = int(mp.drone_pose[1] - wall_x[i])
                map_z = int(mp.drone_pose[2] + wall_z[i])
                
                try:
                    if mp.map_np[map_x, map_y, map_z] == 0:
                        mp.map_np[map_x, map_y, map_z] = 2 # 갈 수 없음
                        mp.map_img[16 - map_x, 51- map_y, map_z, 2] = 125 # 빨간색
                except:
                    pass
            
            for i in range(len(open_x)):
                map_x = int(mp.drone_pose[0] + open_y[i]) # !! 여기서도 문제가 있을 수도 있으니 결과 보고 수정 필요하면 수정하기.
                map_y = int(mp.drone_pose[1] - open_x[i])

                try:
                    if mp.map_np[map_x, map_y] == 0:
                        mp.map_np[map_x, map_y] = 1 # 갈 수 있음
                        mp.map_img[16 - map_x, 51 - map_y, map_z, :] = 125
                except:
                    pass

        mul = 20
        img = cv2.resize(mp.map_img[:, :, int(mp.drone_pose[2]), :], dsize = (51 * mul, 16 * mul))
        cv2.imshow('Global map', img)
        key = cv2.waitKey(10)

        if key == ord('q'):
            break

        # # 갈 수 있는 곳과 갈 수 없는 곳 둘다 매핑해서 Plot 하는 부분

        # plt.subplot(2,1,1)
        # plt.plot(wall_x, wall_y)
        # plt.grid(True)
        # plt.title('Converted_line')


        # plt.subplot(2,1,1)

        fig = plt.figure()
        ax = fig.add_subplot(1, 2, 1, projection = '3d')
        ax.scatter(wall_x, wall_y, wall_z, marker = '.')
        # plt.grid(True)
        # ax.title('3D')

        # plt.subplot(2,1,2)
        ax = fig.add_subplot(1, 2, 2)
        ax.scatter(open_x, open_y)
        ax.scatter(wall_x, wall_y)
        ax.grid(True)
        ax.scatter(0, 0)
        # ax.title('2D')

        plt.show()

    cv2.destroyAllWindows()