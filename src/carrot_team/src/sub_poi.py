#!/usr/bin/env python
import rospy
from icuas23_competition.msg import poi

def get_poi(msg):
    print(msg)

print(1)
rospy.init_node('sub_poi', anonymous=True)
rospy.Subscriber('/red/poi', poi, get_poi)

rospy.spin()