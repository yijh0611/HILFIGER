#!/usr/bin/env python

import numpy as np
import rospy
import threading
import time

from std_msgs.msg import Int32
from std_msgs.msg import MultiArrayFloat32

class Search:
    def __init__(self):
        self.poi = np.array([])

        rospy.init_node('search_poi', anonymous=True)
        rospy.subscriber('/carrot_team/req_poi', MultiArrayFloat32, self.get_poi)

        self.pub_get_poi = rospy.Publisher('/carrot_team/req_poi', Int32, queue_size=10)

        t = threading.Thread(target = self.ros_spin)
        t.start

    
    def get_poi(self, msg):

        # get poi
        tmp = np.array([])
        tmp = np.append(tmp, msg.x)
        tmp = np.append(tmp, msg.y)
        tmp = np.append(tmp, msg.z)

        # save poi
        self.poi = np.append(self.poi, tmp)
    
    def ros_spin(self):
        rospy.spin()
    
if __name__ == "__main__":
    # call class
    ctrl = Search()

    # request poi
    get_poi = Int32()
    get_poi.data = 9

    ctrl.pub_get_poi.publish(get_poi)

    # wait 10s
    time.sleep(3)

    print(ctrl.poi)
    print(np.shape(ctrl.poi))
    
