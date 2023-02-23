#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import numpy as np

def image_callback(msg):
    
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    ### yolo 코드 작성 ###
    # img : 이미지 배열 (480,640,3)
    

    ### yolo 코드 여기까지 ###

    # # imshow 주석처리 가능
    # cv2.imshow('Drone image', img)
    # cv2.waitKey(25)


if __name__ == '__main__':
    rospy.init_node('yolo_ros', anonymous=True)
    rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
    rospy.spin()
