#!/usr/bin/env python

import numpy as np
import rospy
import threading
import time

from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

class Search:
    def __init__(self):
        self.poi = np.array([])

        rospy.init_node('search_poi', anonymous=True)
        rospy.Subscriber('/carrot_team/req_poi', Float32MultiArray, self.get_poi)

        self.pub_get_poi = rospy.Publisher('/carrot_team/req_poi', Int32, queue_size=10)

        t = threading.Thread(target = self.ros_spin)
        t.start

    
    def get_poi(self, msg):

        # get poi
        tmp = np.array([])
        tmp = np.append(tmp, msg.data[0])
        tmp = np.append(tmp, msg.data[1])
        tmp = np.append(tmp, msg.data[2])

        # save poi
        self.poi = np.append(self.poi, tmp)

        # # Print ("End message")
        # print(self.poi)
    
    def ros_spin(self):
        rospy.spin()
    
if __name__ == "__main__":
    # call class
    ctrl = Search()

    # request poi
    get_poi = Int32()
    get_poi.data = 9

    for i in range(2):
        # 오래동안 publish 안했으면 처음거는 버려지기 때문에 2개를 Publish 해야 정보를 받을 수 있다.
        ctrl.pub_get_poi.publish(get_poi)
        time.sleep(0.1)

    # wait 10s
    time.sleep(3)

    print(ctrl.poi)
    print(np.shape(ctrl.poi))

