#!/usr/bin/env python
'''
Check section.
Check which section is hard to pass.

0 : Unknown or need information
1 :  y < small is easy
2 : y > big is easy
'''

import cv2
import math
import matplotlib.pyplot as plt
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
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray, MultiArrayDimension # Publish map in 3D Array


class CheckArea:
    def __init__(self):
        # Variables
        self.map_data = np.array([])
        self.x_size = 21# 15 in introduction
        self.y_size = 51 # 50 in introduction
        self.z_size = 26 # 25 in introduction // look until 15 (0 ~ 15; 16 ea)

        self.is_sub_map = False
        self.pub_area = 0
        '''
        self.pub_area
        0 : Unknown
        1 : y < small is open
        2 : y > big is open
        '''

        # for debugging
        self.map_img = np.array([])
        self.drone_z = 0
        self.is_imshow = False

        # init rospy
        rospy.init_node('check_section', anonymous=True)

        # Subscriber
        rospy.Subscriber('/carrot_team/map', Int32MultiArray, self.sub_map)
        # For debugging
        rospy.Subscriber('/red/carrot/pose', PoseStamped, self.get_pose)

        # Publisher
        self.check_section = rospy.Publisher('/carrot_team/check_section', Int32, queue_size=10)
        
        # Start ros spin
        t = threading.Thread(target = self.ros_spin)
        t.start()

    def sub_map(self, msg):
        self.map_data = np.reshape(msg.data, (self.x_size, self.y_size, self.z_size))
        self.is_sub_map = True
        
        
    
    def get_pose(self, msg):
        self.drone_z = int(msg.pose.position.z)
        # print(msg.pose.position.y)

    def ros_spin(self):
        rospy.spin()
    
if __name__ == "__main__":
    CA = CheckArea()

    while not rospy.is_shutdown():
        # Check wall for section and send message
        if CA.is_sub_map:
            count_wall_17 = 0
            count_open_17 = 0
            count_wall_34 = 0
            count_open_34 = 0
            is_17 = False
            is_34 = False
            map_data = CA.map_data[:,:,:18] # 원래 15인데, 여유있게 18로 한듯
            for i in range(3,19): # x
                for j in range(2,14): # z
                    # count wall and openspace
                    if map_data[i][17][j] == 2 or map_data[i][18][j] == 2 or map_data[i][19][j] == 2:
                        count_wall_17 += 1
                    elif map_data[i][17][j] == 1 or map_data[i][18][j] == 1 or map_data[i][19][j] == 1:
                        count_open_17 += 1
                    
                    if map_data[i][32][j] == 2 or map_data[i][33][j] == 2 or map_data[i][34][j] == 2:
                        count_wall_34 += 1
                    elif map_data[i][32][j] == 1 or map_data[i][33][j] == 1 or map_data[i][34][j] == 1:
                        count_open_34 += 1

                    # Check is_over1/3
                    if count_wall_17 + count_open_17 >= 192/3:
                        is_17 = True
                    if count_wall_34 + count_open_34 >= 192/3:
                        is_34 = True

                    # CA.pub_area
                    if is_17 == True and is_34 == True:
                        if count_open_34 == 0:
                            # y > small is blocked
                            CA.pub_area = 1
                        elif count_open_17 == 0:
                            CA.pub_area = 2
                        else:
                            rat_17 = count_wall_17 / count_open_17
                            rat_34 = count_wall_34 / count_open_34

                            if rat_17 >= rat_34:
                                # 17 is blocked
                                CA.pub_area = 2
                            else:
                                CA.pub_area = 1

                    elif is_17 == False and is_34 == True:
                        if count_wall_34 >= count_open_34:
                            CA.pub_area = 1
                        else:
                            CA.pub_area = 2
                    
                    elif is_17 == True and is_34 == False:
                        if count_wall_17 >= count_open_17:
                            CA.pub_area = 2
                        else:
                            CA.pub_area = 1

                    CA.check_section.publish(CA.pub_area)
                    # time.sleep(0.02)



        # For debugging
        if CA.is_imshow:
            if CA.is_sub_map:
                # Check map
                img = np.zeros(shape = (CA.x_size, CA.y_size, 3))
                map_data = CA.map_data[:,:,:18]
                z = CA.drone_z
                for i in range(CA.x_size):
                    for j in range(CA.y_size):
                        if map_data[i][j][z] == 2: # Wall
                            img[i][j][2] = 250
                        elif map_data[i][j][z] == 1: # Open
                            img[i][j][:] = 250
                
                mul = 20
                img = np.flip(img, (0,1))
                img = cv2.resize(img, dsize = (CA.y_size * mul, CA.x_size * mul), interpolation = cv2.INTER_NEAREST)
                
                # Check map 1/3 point and 2/3 point
                img_18 = np.zeros(shape=(CA.x_size, 15, 3))
                img_33 = np.zeros(shape=(CA.x_size, 15, 3))                
                for i in range(CA.x_size):
                    for j in range(15):
                        if map_data[i][18][j] == 2 or map_data[i][17][j] == 2 or map_data[i][19][j] == 2: # Wall
                            img_18[i][j][2] = 250
                        elif map_data[i][18][j] == 1 or map_data[i][17][j] == 1 or map_data[i][19][j] == 1: # Open
                            img_18[i][j][:] = 250
                        
                        if map_data[i][33][j] == 2 or map_data[i][32][j] == 2 or map_data[i][34][j] == 2: # Wall
                            img_33[i][j][2] = 250
                        elif map_data[i][33][j] == 1 or map_data[i][32][j] == 1 or map_data[i][34][j] == 1: # Open
                            img_33[i][j][:] = 250

                img_18 = np.swapaxes(img_18, 0, 1)
                img_18 = np.flip(img_18, 0)
                img_33 = np.swapaxes(img_33, 0, 1)
                img_33 = np.flip(img_33, 0)
                img_18 = cv2.resize(img_18, dsize = (CA.x_size * mul, 15 * mul), interpolation = cv2.INTER_NEAREST)
                img_33 = cv2.resize(img_33, dsize = (CA.x_size * mul, 15 * mul), interpolation = cv2.INTER_NEAREST)
                cv2.imshow("Map_recieved", img)
                cv2.imshow("IMG_18", img_18)
                cv2.imshow("IMG_33", img_33)
                key = cv2.waitKey(10)
                
                if key == ord('q'):
                    break


