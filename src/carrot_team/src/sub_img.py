#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image

bridge = CvBridge()


def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    cv2.imshow('Drone image', img)

    cv2.waitKey()
    # cv2.destroyAllWindows()

    # pass

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.spin()

cv2.destroyAllWindows()