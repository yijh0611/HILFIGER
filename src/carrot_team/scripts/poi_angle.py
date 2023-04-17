#!/usr/bin/env python

import numpy as np
import rospy

from icuas23_competition.msg import poi
from std_msgs.msg import Int32MultiArray


class GetPOI:
    def __init__(self):
        # save poi
        self.poi = np.array([])
        self.gak = np.array([])
        self.poi_angle = Int32MultiArray()

        # ros subscriber
        rospy.init_node('sub_poi', anonymous=True)
        rospy.Subscriber('/carrot_team/poi', poi, self.get_poi)
        
        # poi publisher
        self.pub_poi = rospy.Publisher('/carrot_team/poi_gakdo', Int32MultiArray, queue_size=10, latch = True)
        self.is_poi_ready = False

        rospy.spin()

    def get_poi(self, msg):
        
        for i in range(len(msg.poi)):
            tmp = np.array([])
            tmp = np.append(tmp, msg.poi[i].x)
            tmp = np.append(tmp, msg.poi[i].y)
            tmp = np.append(tmp, msg.poi[i].z)
            self.poi = np.append(self.poi, tmp)

        self.poi = np.reshape(self.poi, (-1,3))
        
        for j in range(len(msg.poi)):
            x_decimal = str(self.poi[j, 0])[str(self.poi[j, 0]).find('.') + 1 :]
            y_decimal = str(self.poi[j, 1])[str(self.poi[j, 1]).find('.') + 1 :]
            
            if len(x_decimal) < 6 and len(y_decimal) < 6:
                deg = 180
            elif x_decimal[2 : 5] == '000':
                deg = 0
            elif x_decimal[2 : 5] == '421':
                if y_decimal[2 : 5] == '421':
                    deg = -135
                elif y_decimal[2 : 5] == '578':
                    deg = 135 
            elif x_decimal[2 : 5] == '019':
                if y_decimal[2 : 5] == '000':
                    deg = 90
                else:
                    deg = -90
            elif x_decimal[2 : 5] == '578':
                if y_decimal[2 : 5] == '421':
                    deg = 45
                elif y_decimal[2 : 5] == '578':
                    deg = -45
            else:
                 deg = 999
                 
            self.gak = np.append(self.gak, int(deg))
            
        if np.count_nonzero(self.gak) == 0:
            self.gak = np.array([999, 999, 999, 999, 999, 999, 999, 999, 999, 999])
            
        self.gak = [int(x) for x in [self.gak]]
        self.poi_angle.data = self.gak
        
        self.pub_poi.publish(self.poi_angle)


if __name__ == "__main__":
    poi = GetPOI()
