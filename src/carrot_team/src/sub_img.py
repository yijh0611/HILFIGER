#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import Image

def image_callback(msg):
    print(msg)
    
    pass

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.spin()