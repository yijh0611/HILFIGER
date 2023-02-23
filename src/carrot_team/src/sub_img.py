#!/usr/bin/env python

import sys # 파이썬 버전 확인

import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2

from sensor_msgs.msg import Image

bridge = CvBridge()
bridge_2 = CvBridge()

img = np.array([])

vid_name = '/root/uav_ws/src/icuas23_competition/HILFIGER/src/carrot_team/drone_vid.avi'
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter(vid_name, fourcc, 30.0, (640, 480))

def image_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # cv2.imshow('Drone image', img)

    # cv2.waitKey(25)

def image_callback_depth(msg):
    global img
    
    # get depth image
    img_depth = bridge_2.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img_depth = img_depth / 10

    # Record
    global out
    out.write(img)

    # imshow
    cv2.imshow('Depth image', img_depth)
    cv2.imshow('Drone image', img)

    cv2.waitKey(25)

# roslaunch 할때랑 rosrun 할때랑 파이썬 버전이 다른거 같다.
sys.version
print('OpenCv version')
cv2.__version__

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback_depth)
rospy.spin()

cv2.destroyAllWindows()
out.release()