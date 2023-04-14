#!/usr/bin/env python

import cv2
import math
import numpy as np
import rospy
import threading
import time

from sensor_msgs.msg import Imu
from std_msgs.msg import Bool # Is get poi ready


class CheckStill:
    def __init__(self):
        '''
        /red/mavros/imu/data 에서 IMU 가속도랑 각속도 받아서 35회 Array 만들고
        Orientation은 표준편차 작으면s
        Linear acceleration, Angular velocity는 0에 가까우면; z는 
        특정 값 보다 작으면 정지, 크면 움직이는 중이다라고 판단
        하려고 했으나, 그냥 전부 다 표준편차로
        '''

        # is drone still
        self.is_still = False

        self.ori_x = np.array([])
        self.ori_y = np.array([])
        self.ori_z = np.array([])
        self.ori_w = np.array([])
        
        self.lin_x = np.array([])
        self.lin_y = np.array([])
        self.lin_z = np.array([])

        self.ang_vel_x = np.array([])
        self.ang_vel_y = np.array([])
        self.ang_vel_z = np.array([])


        rospy.init_node('check_still', anonymous=True)

        rospy.Subscriber('/red/mavros/imu/data', Imu, self.get_imu)

        # Publish is still
        self.is_still_pub = rospy.Publisher('/carrot_team/is_still', Bool, queue_size = 10)


        t = threading.Thread(target = self.ros_spin)        
        t.start()

        while not rospy.is_shutdown():
            time.sleep(0.1)

    def ros_spin(self):
        rospy.spin()
    
    def rmse(self, y1, y2):
        return np.sqrt(((y1-y2)**2).mean())

    def get_imu(self, msg):
        # get imu data
        # print(msg.orientation.x) # x, y, z, w
        # print(msg.linear_acceleration) # x, y, z
        # print(msg.angular_velocity) # x, y, z

        self.ori_x = np.append(self.ori_x, msg.orientation.x)
        self.ori_y = np.append(self.ori_y, msg.orientation.y)
        self.ori_z = np.append(self.ori_z, msg.orientation.z)
        self.ori_w = np.append(self.ori_w, msg.orientation.w)

        self.lin_x = np.append(self.lin_x, msg.linear_acceleration.x)
        self.lin_y = np.append(self.lin_y, msg.linear_acceleration.y)
        self.lin_z = np.append(self.lin_z, msg.linear_acceleration.z)

        self.ang_vel_x = np.append(self.ang_vel_x, msg.angular_velocity.x)
        self.ang_vel_y = np.append(self.ang_vel_y, msg.angular_velocity.y)
        self.ang_vel_z = np.append(self.ang_vel_z, msg.angular_velocity.z)

        if len(self.ori_x) > 35:
            self.ori_x = np.delete(self.ori_x, 0)
            self.ori_y = np.delete(self.ori_y, 0)
            self.ori_z = np.delete(self.ori_z, 0)
            self.ori_w = np.delete(self.ori_w, 0)

            self.lin_x = np.delete(self.lin_x, 0)
            self.lin_y = np.delete(self.lin_y, 0)
            self.lin_z = np.delete(self.lin_z, 0)

            self.ang_vel_x = np.delete(self.ang_vel_x, 0)
            self.ang_vel_y = np.delete(self.ang_vel_y, 0)
            self.ang_vel_z = np.delete(self.ang_vel_z, 0)

            std_np = np.array([])
            # 표준편차 구하기 - 일단 구하고 제일 작은 값 출력
            std_np = np.append(std_np, np.std(self.ori_x))
            std_np = np.append(std_np, np.std(self.ori_y))
            std_np = np.append(std_np, np.std(self.ori_z))
            std_np = np.append(std_np, np.std(self.ori_w))

            std_np = np.append(std_np, np.std(self.lin_x))
            std_np = np.append(std_np, np.std(self.lin_y))
            std_np = np.append(std_np, np.std(self.lin_z))

            std_np = np.append(std_np, np.std(self.ang_vel_x))
            std_np = np.append(std_np, np.std(self.ang_vel_y))
            std_np = np.append(std_np, np.std(self.ang_vel_z))

            # RMSE of latest value
            rmse_np = np.array([self.rmse(self.ori_x, msg.orientation.x)])
            rmse_np = np.append(rmse_np, self.rmse(self.ori_y, msg.orientation.y))
            rmse_np = np.append(rmse_np, self.rmse(self.ori_z, msg.orientation.z))
            rmse_np = np.append(rmse_np, self.rmse(self.ori_w, msg.orientation.w))

            rmse_np = np.append(rmse_np, self.rmse(self.lin_x, msg.linear_acceleration.x))
            rmse_np = np.append(rmse_np, self.rmse(self.lin_y, msg.linear_acceleration.y))
            rmse_np = np.append(rmse_np, self.rmse(self.lin_z, msg.linear_acceleration.z))

            rmse_np = np.append(rmse_np, self.rmse(self.ang_vel_x, msg.angular_velocity.x))
            rmse_np = np.append(rmse_np, self.rmse(self.ang_vel_y, msg.angular_velocity.y))
            rmse_np = np.append(rmse_np, self.rmse(self.ang_vel_z, msg.angular_velocity.z))

            # print(np.max(rmse_np))

            # print(np.max(std_np))
            if np.max(std_np) > 0.05: # Moving
                self.is_still = False
            else:
                self.is_still = True

            if np.max(rmse_np) > 0.03: # Moving
                self.is_still = False
            
            
            msg = Bool()
            msg.data = self.is_still

            self.is_still_pub.publish(msg)


if __name__ == "__main__" :
    mp = CheckStill()