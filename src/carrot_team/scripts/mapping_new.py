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

        # Constant variable
        self.WIDTH = 640
        self.HEIGHT = 480
        
        self.W_HALF = self.WIDTH // 2
        self.H_HALF = self.HEIGHT // 2


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
        self.w = self.WIDTH
        self.h = self.HEIGHT

        self.bridge = CvBridge() # Get drone image
        self.img_depth = np.zeros((self.h, self.w))
        self.drone_yaw = 0
        self.drone_pose = np.array([0, 0, 0])
        self.time_is_map = time.time()
        
        # prop mask
        self.mask = np.ones((self.h, self.w))
        for i in range(100,177):
            for j in range(60):
                self.mask[i][j] = 0
            for j in range(580,640):
                self.mask[i][j] = 0


        ########## Radian array cal start ##########
        # get radian - To reduce calculation
        # Constant variable
        self.W_HALF = self.w // 2
        self.H_HALF = self.h // 2

        self.W_LIST = np.array([])
        self.H_LIST = np.array([])
        self.W_TAN = np.array([])
        self.H_TAN = np.array([])

        self.PROP_W_S = 60
        self.PROP_W_L = 580
        self.PROP_H_S = 100
        self.PROP_H_L = 180 # 177

        self.res = 10 # open space mapping resolution

        # Change if necessary
        self.W_RAD = math.radians(87)
        self.H_RAD = math.radians(58)

        self.W_SKIP = 5
        self.H_SKIP = 5

        # change prop range
        self.PROP_W_S = self.PROP_W_S // self.W_SKIP
        self.PROP_W_L = self.PROP_W_L // self.W_SKIP
        self.PROP_H_S = self.PROP_H_S // self.H_SKIP
        self.PROP_H_L = self.PROP_H_L // self.H_SKIP
        

        for i in range(self.h):
            h = self.H_HALF - i
            if h <= 0:
                h -= 1
            
            rad_h = math.atan((math.tan(self.H_RAD / 2) * h) / self.H_HALF)

            self.H_LIST = np.append(self.H_LIST, rad_h)
            self.H_TAN = np.append(self.H_TAN, math.tan(rad_h))
        
        for i in range(self.w):
            w = i - self.W_HALF
            if w >= 0:
                w += 1
            
            rad_w = math.atan((math.tan(self.W_RAD / 2) * w) / self.W_HALF)
            self.W_LIST = np.append(self.W_LIST, rad_w)
            self.W_TAN = np.append(self.W_TAN, math.tan(rad_w))

        # Skip array
        self.H_LIST = self.H_LIST[::self.H_SKIP]
        self.W_LIST = self.W_LIST[::self.W_SKIP]
        self.H_TAN = self.H_TAN[::self.H_SKIP]
        self.W_TAN = self.W_TAN[::self.W_SKIP]

        # Change shape to horizontal array
        self.H_LIST = np.reshape(self.H_LIST, (-1, 1))
        self.H_TAN = np.reshape(self.H_TAN, (-1, 1))

        ########## Radian array cal end ##########
        
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

    def is_poi_callback(self, msg):
        self.is_poi = msg.data
    
    def publish_map(self):
        while True:
            array = self.map_np * 1
            array_flat = np.reshape(self.map_np, -1).astype(np.int32)

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

    def del_idx(self, x, y, z, idx):
        x = np.delete(x, idx)
        y = np.delete(y, idx)
        z = np.delete(z, idx)

        return x, y, z

    def del_range(self, x, y, z):
        # x < 0
        idx = np.where(x < 0)[0]
        x, y, z = self.del_idx(x, y, z, idx)

        idx = np.where(x >= self.x_size)[0]
        x, y, z = self.del_idx(x, y, z, idx)

        # y < 0
        idx = np.where(y < 0)[0]
        x, y, z = self.del_idx(x, y, z, idx)

        idx = np.where(y >= self.y_size)[0]
        x, y, z = self.del_idx(x, y, z, idx)

        # z < 0
        idx = np.where(z < 0)[0]
        x, y, z = self.del_idx(x, y, z, idx)

        idx = np.where(z >= self.z_size)[0]
        x, y, z = self.del_idx(x, y, z, idx)

        return x, y, z


if __name__ == "__main__" :
    mp = Mapping()

    # Wait until POI is recieved
    print("Waiting 40s until POI is ready")
    time_is_poi = time.time()

    while mp.is_poi == False:
        time.sleep(1)
        if time.time() - time_is_poi > 40:
            print('POI not ready; Error!')
            print("Mapping Start")
            break

    while True:
        # Map when drone is still
        if time.time() - mp.time_is_map > 0.2:
            # Check position change
            mp.is_global_mapping = False
            for i in range(4):
                if i < 3:
                    if mp.pos_prev[i] != mp.drone_pose[i]:
                        mp.is_global_mapping = True
                    mp.pos_prev[i] = mp.drone_pose[i]
                else: # i == 3
                    if mp.pos_prev[i] != mp.drone_yaw:
                        mp.is_global_mapping = True
                    mp.pos_prev[i] = mp.drone_yaw
            
            if mp.is_global_mapping:
                mp.time_total = time.time()
                wall_x = np.array([])
                wall_y = np.array([])
                wall_z = np.array([])

                open_x = np.array([])
                open_y = np.array([])
                open_z = np.array([])

                # Rotate
                cy = math.cos(mp.drone_yaw)
                sy = math.sin(mp.drone_yaw)

                # change nan to 10
                img_d_y = np.nan_to_num(mp.img_depth, nan = 10) * mp.mask
                img_d_y = img_d_y[::mp.H_SKIP,::mp.W_SKIP]
                img_d_x = img_d_y * mp.W_TAN
                img_d_z = img_d_y * mp.H_TAN

                img_d_x_new = cy * img_d_x - sy * img_d_y # rotated
                img_d_y_new = cy * img_d_y + sy * img_d_x # rotated

                # flatten
                d_flat = np.reshape(img_d_y, (-1))
                d_y_flat = np.reshape(img_d_y_new, (-1))
                d_x_flat = np.reshape(img_d_x_new, (-1))
                d_z_flat = np.reshape(img_d_z, (-1))

                # remove propeller
                idx = np.array(np.nonzero(d_flat))[0]
                d_flat = d_flat[idx]
                d_y_flat = d_y_flat[idx]
                d_x_flat = d_x_flat[idx]
                d_z_flat = d_z_flat[idx]

                # Open space
                for i in range(1, mp.res):
                    open_x = np.append(open_x, d_x_flat * i / mp.res)
                    open_y = np.append(open_y, d_y_flat * i / mp.res)
                    open_z = np.append(open_z, d_z_flat * i / mp.res)

                # wall mapping
                idx = np.where(d_flat == 10)[0]
                wall_x = np.delete(d_x_flat, idx)
                wall_y = np.delete(d_y_flat, idx)
                wall_z = np.delete(d_z_flat, idx)


                # global mapping
                if (len(wall_x) > 0 or len(open_x) > 0):
                    map_x = (mp.drone_pose[0] + wall_y).astype(int)
                    map_y = (mp.drone_pose[1] - wall_x).astype(int)
                    map_z = (mp.drone_pose[2] + wall_z).astype(int)
                    
                    # Del values out of range
                    map_x, map_y, map_z = mp.del_range(map_x, map_y, map_z)

                    count = 0

                    # !! 이거 가능하면 for문 사용안하게 수정하기 
                    for i in range(len(map_x)):
                        mx = map_x[i]
                        my = map_y[i]
                        mz = map_z[i]
                        
                        try:
                            mp.wall_np[mx, my, mz] += 30

                            if mp.wall_np[mx, my, mz] >= mp.open_np[mx, my, mz]:
                                mp.map_np[mx, my, mz] = 2 # Wall
                                mp.map_img[mx, my, mz, 2] = 125 # 빨간색
                        except:
                            count += 1
                    
                    if count > 0:
                        print(f"Error from wall mapping : {count}/{len(map_x)}")

                    
                    # Open space
                    map_x = (mp.drone_pose[0] + open_y).astype(int)
                    map_y = (mp.drone_pose[1] - open_x).astype(int)
                    map_z = (mp.drone_pose[2] + open_z).astype(int)

                    # Del values out of range
                    map_x, map_y, map_z = mp.del_range(map_x, map_y, map_z)

                    count = 0

                    for i in range(len(map_x)):
                        mx = map_x[i]
                        my = map_y[i]
                        mz = map_z[i]

                        try:
                            mp.open_np[mx, my, mz] += 1
                            if mp.open_np[mx, my, mz] > mp.wall_np[mx, my, mz]:
                                mp.map_np[mx, my, mz] = 1 # Open space
                                mp.map_img[mx, my, mz, :] = 125
                        except:
                            count += 1
                    
                    if count > 0:
                        print(f"Error from open space mapping : {count}/{len(map_x)}")

                print(time.time() - mp.time_total)


                    # for i in range(len(open_x)):
                    #     map_x = int(mp.drone_pose[0] + open_y[i]) # !! 여기서도 문제가 있을 수도 있으니 결과 보고 수정 필요하면 수정하기.
                    #     map_y = int(mp.drone_pose[1] - open_x[i])
                    #     map_z = int(mp.drone_pose[2] + open_z[i])

                    #     try:
                    #         mp.open_np[map_x, map_y, map_z] += 1
                    #         # if mp.map_np[map_x, map_y, map_z] == 0:
                    #         if mp.open_np[map_x, map_y, map_z] > mp.wall_np[map_x, map_y, map_z] * 10:
                    #             mp.map_np[map_x, map_y, map_z] = 1 # Open space
                    #             mp.map_img[map_x, map_y, map_z, :] = 125
                    #             # mp.map_img[mp.x_size - map_x, mp.y_size - map_y, map_z, :] = 125
                    #             # print(map_x, map_y, map_z)
                    #     except:
                    #         pass

        mul = 20
        mp.map_img_tmp = mp.map_img * 1
        mp.map_img_tmp = np.flip(mp.map_img, (0,1))
        # print(np.shape(mp.map_img_tmp))

        img = cv2.resize(mp.map_img_tmp[:, :, int(mp.drone_pose[2]), :], dsize = (mp.y_size * mul, mp.x_size * mul))
        cv2.imwrite('/root/pic/global_map.png', img)
        cv2.imshow('Global map', img)
        key = cv2.waitKey(10)

        if key == ord('q'):
            break

        # # # # 갈 수 있는 곳과 갈 수 없는 곳 둘다 매핑해서 Plot 하는 부분
        # if len(wall_x) > 0:
        #     print(len(wall_x))
        #     fig = plt.figure()
        #     ax = fig.add_subplot(1, 2, 1, projection = '3d')
        #     ax.scatter(wall_x, wall_y, wall_z, marker = '.')
        #     # plt.grid(True)
        #     # ax.title('3D')

        #     # plt.subplot(2,1,2)
        #     # new map temp
        #     # h_tmp = int(mp.drone_pose[2])
        #     wall_x_tmp = np.array([])
        #     wall_y_tmp = np.array([])
        #     # print('wall_z :', len(wall_z))
        #     for i,n in enumerate(wall_z):
        #         if int(n) == 0:
        #             wall_x_tmp = np.append(wall_x_tmp, wall_x[i])
        #             wall_y_tmp = np.append(wall_y_tmp, wall_y[i])
                
        #     ax = fig.add_subplot(1, 2, 2)
        #     ax.scatter(open_x, open_y)
        #     ax.scatter(wall_x_tmp, wall_y_tmp)
        #     # ax.scatter(wall_x, wall_y)
        #     ax.grid(True)
        #     ax.scatter(0, 0)
        #     # ax.title('2D')
            
        #     # # plt.show()

        #     # 이미지 저장 후 다시 불러와서 imshow
        #     filename = 'plot.png'
        #     fig.savefig(filename)

        #     img = cv2.imread(filename)

        #     # cv2.imshow("Local mapping", img)
        #     # key = cv2.waitKey(10)

        # # print(time.time() - mp.time_total)
    cv2.destroyAllWindows()