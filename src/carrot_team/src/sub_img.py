#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image

bridge = CvBridge()


def image_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    # print(msg)
    print(np.shape(msg))
    
    pass

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.spin()