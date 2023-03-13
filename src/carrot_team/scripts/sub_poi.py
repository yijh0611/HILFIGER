#!/usr/bin/env python

import numpy as np
import rospy
import time
import threading

from icuas23_competition.msg import poi
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

class GetPOI:
    def __init__(self):
        # save poi
        self.poi = np.array([])

        # ros subscriber
        rospy.init_node('sub_poi', anonymous=True)
        rospy.Subscriber('/red/poi', poi, self.get_poi)
        rospy.Subscriber('/carrot_team/req_poi', Int32, self.sub_poi_when_request)
        
        # poi publisher
        self.pub_poi = rospy.Publisher('/carrot_team/poi', Float32MultiArray, queue_size=10)

        rospy.spin()
        # t = threading.Thread(target = self.ros_spin)
        # t.start

    def get_poi(self, msg):
        for i in range(len(msg.poi)):
            tmp = np.array([])
            tmp = np.append(tmp, msg.poi[i].x)
            tmp = np.append(tmp, msg.poi[i].y)
            tmp = np.append(tmp, msg.poi[i].z)

            # 두번 보내야 함
            self.poi = np.append(self.poi, tmp)
            # time.sleep(0.1)
            # self.poi = np.append(self.poi, tmp)

        self.poi = np.reshape(self.poi, (-1,3))
        print(self.poi)
    
    def sub_poi_when_request(self, msg):
        

        if msg.data < len(self.poi):
            # data to publish
            poi_msg = Float32MultiArray()
            poi_msg.data = self.poi[msg.data]
            
            # publish data
            self.pub_poi.publish(poi_msg)
    
    def ros_spin(self):
        rospy.spin()

if __name__ == "__main__":
    poi = GetPOI()

# <<<<<<< hak

# # # 원래 코드
# import random
# import subprocess
# from gazebo_msgs.srv import DeleteModel, DeleteModelRequest

# from geometry_msgs.msg import Point
# from icuas23_competition.msg import poi
# import rospy
# import numpy as np

# def construct_poi(pose):
# 	d = 2
# 	r = d*1.25
# 	cc_x = pose[0] + d*np.cos(pose[3])
# 	cc_y = pose[1] + d*np.sin(pose[3])

# 	z_off = 2*np.random.random()-4

# 	return (cc_x, cc_y, pose[2]+z_off, r)



# tile_poses = [(7.56, 8.32, 9.1, 0.7854), (7.57, 14.60, 5.5, -0.7854), (3.00, 43.36, 11.1, -1.5707), 
# (9.3, 43.36, 4.8, -1.5707), (10.36, 45.3, 3.8, 0), (10.36, 46.2, 12.1, 0), 
# (10.36, 44.5, 8.35, 0), (16.2, 44.275, 7.3, -1.5707), (5.92, 12, 4.35, 3.1415),
# (6.34, 11, 4.35, 0)]

# rospy.init_node('spawner_deleter')
# # delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
# pub = rospy.Publisher('poi', poi, queue_size=10, latch=True)
# # for i in range(5):
# poses = random.sample(tile_poses, 10)

# command1 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_1 x:=%s y:=%s z:=%s yaw:=%s" % poses[0]
# command2 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_2 x:=%s y:=%s z:=%s yaw:=%s" % poses[1]
# command3 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_3 x:=%s y:=%s z:=%s yaw:=%s" % poses[2]
# command4 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_4 x:=%s y:=%s z:=%s yaw:=%s" % poses[3]
# command5 = "roslaunch icuas23_competition spawn_crack.launch name:=crack_5 x:=%s y:=%s z:=%s yaw:=%s" % poses[4]
# command6 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_1 x:=%s y:=%s z:=%s yaw:=%s" % poses[5]
# command7 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_2 x:=%s y:=%s z:=%s yaw:=%s" % poses[6]
# command8 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_3 x:=%s y:=%s z:=%s yaw:=%s" % poses[7]
# command9 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_4 x:=%s y:=%s z:=%s yaw:=%s" % poses[8]
# command10 = "roslaunch icuas23_competition spawn_crack.launch name:=noncrack_5 x:=%s y:=%s z:=%s yaw:=%s" % poses[9]
# roslaunch = subprocess.Popen(command1.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command2.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command3.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command4.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command5.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command6.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command7.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command8.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command9.split(" "), shell=False)
# rospy.sleep(2)
# roslaunch = subprocess.Popen(command10.split(" "), shell=False)
# rospy.sleep(2)

# print("Publishing pois")

# PoIArray = poi()
# point_list = []
# for pose in poses:
# 	point_of_interest = construct_poi(pose)

# 	msg = Point()

# 	msg.x = point_of_interest[0]
# 	msg.y = point_of_interest[1]
# 	msg.z = point_of_interest[2]

# 	point_list.append(msg)

# PoIArray.poi = point_list

# pub.publish(PoIArray)
# rospy.sleep(5)
# =======
# rospy.spin()
# >>>>>>> main