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
from std_msgs.msg import Bool # Is get poi ready
from std_msgs.msg import Float64 # get yaw
from std_msgs.msg import Int32MultiArray, MultiArrayDimension # Publish map in 3D Array


class Mapping:
    def __init__(self):
        # is get POI end
        self.is_poi = False

        # Set variables
        self.x_size = 21 # 15 in introduction
        self.y_size = 51 # 50 in introduction
        self.z_size = 26 # 25 in introduction
        
        self.map_np = np.zeros((self.x_size, self.y_size, self.z_size)) # x : Initial drone's orientation, y : left
        self.map_img = np.zeros((self.x_size, self.y_size, self.z_size, 3))
        # 0 : Unknown, 1 : Open, 2 : Wall
        # Color :
        # Black : Unknown, White : Open, Red : Wall

        # Check how many times detectes as wall or open space
        self.wall_np = np.zeros((self.x_size, self.y_size, self.z_size))
        self.open_np = np.zeros((self.x_size, self.y_size, self.z_size))

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
        self.w = 640
        self.h = 480

        self.bridge = CvBridge() # Get drone image
        self.img_depth = np.zeros((self.h, self.w))
        self.drone_yaw = 0
        self.drone_pose = np.array([0, 0, 0])
        self.time_is_map = time.time()

        # get radian - To reduce calculation
        self.w_half = self.w // 2
        self.h_half = self.h // 2

        self.w_rad = math.radians(87)
        self.h_rad = math.radians(58)

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
        
        self.time_total = time.time()

        self.pos_prev = np.zeros(4)
        self.is_global_mapping = False
        
        rospy.init_node('mapping_node', anonymous=True)

        rospy.Subscriber('/red/camera/depth/image_raw', Image, self.image_callback_depth)
        rospy.Subscriber('/red/carrot/yaw', Float64, self.yaw_rad) # /red/uav/yaw 도 있는데, 값이 크게 차이나지는 않는거 같아서 그냥 이거 썼다.
        rospy.Subscriber('/red/carrot/pose', PoseStamped, self.get_pose)
        rospy.Subscriber('/carrot_team/is_poi_ready', Bool, self.is_poi_callback)

        # Publish map
        self.pub_map = rospy.Publisher('/carrot_team/map', Int32MultiArray, queue_size = 10)

        t = threading.Thread(target = self.ros_spin)
        t_pub_map = threading.Thread(target = self.publish_map)
        
        t.start()
        t_pub_map.start()
    
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

    def is_poi_callback(self, msg):
        self.is_poi = msg.data
    
    def publish_map(self):
        while True:
            array = self.map_np * 1
            array_flat = np.reshape(self.map_np, -1).astype(np.int32)

            # print(np.shape(array))

            msg = Int32MultiArray()
            # msg.data = sum(array, []) # flatten the 3D array to 1D list
            msg.data = list(array_flat)

            dim_height = MultiArrayDimension()
            dim_height.label = "height"
            dim_height.size = len(array)
            dim_height.stride = len(array[0]) * len(array[0][0])
            msg.layout.dim.append(dim_height)

            dim_width = MultiArrayDimension()
            dim_width.label = "width"
            dim_width.size = len(array[0])
            dim_width.stride = len(array[0][0])
            msg.layout.dim.append(dim_width)

            dim_depth = MultiArrayDimension()
            dim_depth.label = "depth"
            dim_depth.size = len(array[0][0])
            dim_depth.stride = 1
            msg.layout.dim.append(dim_depth)
            
            self.pub_map.publish(msg)
            
            time.sleep(0.1)

    
if __name__ == "__main__" :
    mp = Mapping()

    print("Waiting 40s until POI is ready")
    # Wait until POI is recieved
    time_is_poi = time.time()
    while mp.is_poi == False:
        time.sleep(1)
        if time.time() - time_is_poi > 40:
            print('POI not ready; Error!')
            break

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
        
        for i in range(80, 420, 5): # 60 ~ 420
            # if i % 5 == 0:
                for j in range(64, 576, 5):
                    # 아직 수정안했는데, 맞는지 확인해보기
                    if (i < 60 and j > 100 and j < 177) or (i > 580 and j > 100 and j < 177): # 범위 잘못된거 같다. i가 세로이기 때문에 i랑 j 랑 수정 필요
                        pass
                    else:
                        d = mp.img_depth[i][j]
                        isNaN = np.isnan(d)
                        if isNaN:
                            # print('isNaN')
                            d = 10
                        dist_x, dist_y, dist_z = mp.get_dist(d, j, i) # w, h

                        dist_x_rot, dist_y_rot = rot.dot(np.array([dist_x, dist_y]).T) # 원래 매핑 상태와 맞게 매칭한 그래프

                        if d != 10:
                            wall_x = np.append(wall_x, dist_x_rot)
                            wall_y = np.append(wall_y, dist_y_rot)
                            wall_z = np.append(wall_z, dist_z)

                        # Open space mapping
                        n = 5 # Resolution (How many)
                        for k in range(1,n + 1):
                            open_x = np.append(open_x, dist_x_rot * k / n)
                            open_y = np.append(open_y, dist_y_rot * k / n)
                            open_z = np.append(open_z, dist_z * k / n)
                            # 1m 간격으로 Plot 하는 방법 생각해보기


        # Check position change
        mp.is_global_mapping = False
        for i in range(4):
            if i < 3:
                if mp.pos_prev[i] != mp.drone_pose[i]:
                    mp.is_global_mapping = True
                    break
            
            if i == 3:
                if mp.pos_prev[i] != mp.drone_yaw:
                    mp.is_global_mapping = True

        # global mapping
        if mp.is_global_mapping:
            if time.time() - mp.time_is_map > 0.2 and (len(wall_x) > 0 or len(open_x) > 0):
                # mapping when drone is still for more than 0.5s.
                for i in range(len(wall_x)):
                    map_x = int(mp.drone_pose[0] + wall_y[i]) # !! 드론에 더 가까운 쪽으로 벽을 만들 필요가 있기 때문에, 그냥 int를 쓰면 안되고 상황에 따라서 +- 1을 해야한다. - 일단 맵이 어떻게 되는지 확인 후 기능 추가
                    map_y = int(mp.drone_pose[1] - wall_x[i])
                    map_z = int(mp.drone_pose[2] + wall_z[i])
                    
                    try:
                        mp.wall_np[map_x, map_y, map_z] += 1
                        # if mp.map_np[map_x, map_y, map_z] == 0:
                        if mp.wall_np[map_x, map_y, map_z] * 10 >= mp.open_np[map_x, map_y, map_z]:
                            mp.map_np[map_x, map_y, map_z] = 2 # Wall
                            mp.map_img[mp.x_size - map_x, mp.y_size - map_y, map_z, 2] = 125 # 빨간색
                    except:
                        pass
                
                for i in range(len(open_x)):
                    map_x = int(mp.drone_pose[0] + open_y[i]) # !! 여기서도 문제가 있을 수도 있으니 결과 보고 수정 필요하면 수정하기.
                    map_y = int(mp.drone_pose[1] - open_x[i])
                    map_z = int(mp.drone_pose[2] + open_z[i])

                    try:
                        mp.open_np[map_x, map_y, map_z] += 1
                        # if mp.map_np[map_x, map_y, map_z] == 0:
                        if mp.open_np[map_x, map_y, map_z] > mp.wall_np[map_x, map_y, map_z] * 10:
                            mp.map_np[map_x, map_y, map_z] = 1 # Open space
                            mp.map_img[mp.x_size - map_x, mp.y_size - map_y, map_z, :] = 125
                            # print(map_x, map_y, map_z)
                    except:
                        pass

        mul = 20
        img = cv2.resize(mp.map_img[:, :, int(mp.drone_pose[2]), :], dsize = (mp.y_size * mul, mp.x_size * mul))
        cv2.imshow('Global map', img)
        key = cv2.waitKey(10)

        if key == ord('q'):
            break

        # # # 갈 수 있는 곳과 갈 수 없는 곳 둘다 매핑해서 Plot 하는 부분
        if len(wall_x) > 0:
            print(len(wall_x))
            fig = plt.figure()
            ax = fig.add_subplot(1, 2, 1, projection = '3d')
            ax.scatter(wall_x, wall_y, wall_z, marker = '.')
            # plt.grid(True)
            # ax.title('3D')

            # plt.subplot(2,1,2)
            # new map temp
            # h_tmp = int(mp.drone_pose[2])
            wall_x_tmp = np.array([])
            wall_y_tmp = np.array([])
            # print('wall_z :', len(wall_z))
            for i,n in enumerate(wall_z):
                if int(n) == 0:
                    wall_x_tmp = np.append(wall_x_tmp, wall_x[i])
                    wall_y_tmp = np.append(wall_y_tmp, wall_y[i])
                
            ax = fig.add_subplot(1, 2, 2)
            ax.scatter(open_x, open_y)
            ax.scatter(wall_x_tmp, wall_y_tmp)
            # ax.scatter(wall_x, wall_y)
            ax.grid(True)
            ax.scatter(0, 0)
            # ax.title('2D')
            
            # # plt.show()

            # 이미지 저장 후 다시 불러와서 imshow
            filename = 'plot.png'
            fig.savefig(filename)

            img = cv2.imread(filename)

            cv2.imshow("Local mapping", img)
            key = cv2.waitKey(10)

        print(time.time() - mp.time_total)
        mp.time_total = time.time()

    cv2.destroyAllWindows()