#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2

from sensor_msgs.msg import Image

bridge = CvBridge()


def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    cv2.imshow('Drone image', img)

    cv2.waitKey(25)

def image_callback_depth(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    cv2.imshow('Depth image', img)

    cv2.waitKey(25)

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback)
rospy.spin()

cv2.destroyAllWindows()