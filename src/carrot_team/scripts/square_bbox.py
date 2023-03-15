# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from std_msgs.msg import int32MultiArray
from cv_bridge import CvBridge
import numpy as np

from sensor_msgs.msg import Image
import cv2


class SquareDetector : 
    
    def __init__(self): 
        
        self.bridge = CvBridge()
        self.threshold_low = 190
        self.threshold_high = 255
        self.square_bbox = int32MultiArray()
        self.pub = rospy.Publisher('/carrot_team/square_bbox', int32MultiArray, queue_size = 10)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(img_gray, self.threshold_low, self.threshold_high, cv2.CHAIN_APPROX_NONE)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        xyxy = [0, 0, 0, 0]
        
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1] - 5
            
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                if aspect_ratio >= 0.70 and aspect_ratio <= 1.3 and w > 5 and w < 300:
                    xyxy = [int(x), int(y), int(x + w), int(y + h)]
                
        self.square_bbox.data = xyxy
        self.pub.publish(self.square_bbox)
        
if __name__ == '__main__':
    rospy.init_node('box_ros', anonymous=True)
    box = SquareDetector()
    rospy.Subscriber('/red/camera/color/image_raw', Image, box.image_callback)
    rospy.spin()