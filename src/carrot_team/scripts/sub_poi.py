#!/usr/bin/env python

import numpy as np
import rospy
import time
import threading

from icuas23_competition.msg import poi
from std_msgs.msg import Bool
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
        self.pub_is_poi_ready = rospy.Publisher('/carrot_team/is_poi_ready', Bool, queue_size=10)
        self.is_poi_ready = False

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

        self.is_poi_ready = True
    
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

    while True:
        msg = Bool()
        msg.data = poi.is_poi_ready

        poi.pub_is_poi_ready(msg)

        time.sleep(0.1)