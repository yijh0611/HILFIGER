#!/usr/bin/env python

import os # mkdir
import sys # check python version

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge # Change ros image into opencv
from sensor_msgs.msg import Image # Subscribe image

bridge = CvBridge() # Get drone image
bridge_2 = CvBridge() # Get drone depth map

img = np.array([]) # 전역변수

# 동영상 저장하는 폴더가 없으면 만들기
base_dir = '/root/video'

if not os.path.isdir(base_dir):
    os.mkdir(base_dir)

# 동영상 저장 경로
vid_name = base_dir + '/drone_vid.avi'
vid_name_d = base_dir + '/drone_vid_depth.avi'

# 동영상 코덱
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter(vid_name, fourcc, 30.0, (640, 480))
out_d = cv2.VideoWriter(vid_name_d, fourcc, 30.0, (640, 480))


def image_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # cv2.imshow('Drone image', img)

    # cv2.waitKey(25)


def image_callback_depth(msg):
    global img
    
    # get depth image
    img_depth_ori = bridge_2.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img_depth = img_depth_ori / 10 # convert into (유사) grayscale

    ### send distance
    # 전방에 얼마나 남았는지
    h, w = np.shape(img_depth_ori)
    w_half = w // 2
    h_half = h // 2
    
    print(img_depth_ori[h_half][w_half])

    # 어디로 이동할지

    ### send distance end

    # Record
    global out
    global out_d

    out.write(img)
    # out_d.write(img_depth) # 코덱이 안맞아서 저장이 안되는 것 같다.

    # imshow
    cv2.imshow('Depth image', img_depth)
    cv2.imshow('Drone image', img)

    cv2.waitKey(25)


# roslaunch 할때랑 rosrun 할때랑 파이썬 버전이 다른거 같아서 확인
sys.version
print('OpenCv version')
cv2.__version__

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback_depth)
rospy.spin()

# 이거는 안쓰일거 같다. - 수정
cv2.destroyAllWindows()
out.release()
out_d.release()