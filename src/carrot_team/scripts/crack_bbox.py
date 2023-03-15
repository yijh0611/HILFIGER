#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO

from sensor_msgs.msg import Image
import cv2


class CrackDetector : 
    
    def __init__(self): 
        
        self.weights = '../detector/l.pt'
        self.model = YOLO(self.weights)
        self.c_cut = 0.01
        self.bridge = CvBridge()
        self.crack_bbox = Int32MultiArray()
        self.pub = rospy.Publisher('/carrot_team/crack_bbox', Int32MultiArray, queue_size = 10)

    def image_callback(self, msg):
        
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img_display = img.copy()
        
        results = self.model.predict(source = img, save = False, show = False, verbose = False)
        xyxy = []
        
        for result in results: 
            
            conf_list = result.boxes.conf
            xyxy_list = result.boxes.xyxy
            
        conf = conf_list.cpu().detach().numpy()
        xyxy_list = xyxy_list.cpu().detach().numpy()
        
        if len(conf) == 0: 
            
            xyxy = np.concatenate((xyxy, [0]), axis = 0)
            
        else:
            
            idx = np.where(conf > self.c_cut)
            
            for xyxy in xyxy_list[idx]: 
                xyxy = np.concatenate((xyxy, [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])]), aixs = 0)
                cv2.rectangle(img_display, (xyxy[0], xyxy[1]), (xyxy[0], xyxy[1]), (0, 255, 0), 3)
        
        self.crack_bbox.data = xyxy
        self.pub.publish(self.crack_bbox)
        
        cv2.imshow('crack_bbox', img_display)
        cv2.waitKey(0)

if __name__ == '__main__':
    rospy.init_node('yolo_ros', anonymous=True)
    detector = CrackDetector()
    rospy.Subscriber('/red/camera/color/image_raw', Image, detector.image_callback)
    rospy.spin()
