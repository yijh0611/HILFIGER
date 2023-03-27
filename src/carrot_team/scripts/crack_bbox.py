#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Int16
from queue import Queue
import threading
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
        self.poi_queue = Queue()
        self.img_queue = Queue()
        self.data = {}
        self.pub = rospy.Publisher('/carrot_team/crack_bbox', Int32MultiArray, queue_size = 10)

    def poi_callback(self, msg):

        self.poi_queue.put(msg.data)

        try:
            self.img_queue.put(self.data['img'])

        except KeyError:
            pass

    def img_callback(self, msg):

        self.data['img'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detect_callback(self):

        if self.img_queue.qsize() == 0:
            pass

        else:
            img = self.img_queue.get()
            poi = self.poi_queue.get()
            img_display = img.copy()
            results = self.model.predict(source = img, device = 'cpu', save = False, show = False, verbose = False)
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

    def rospy_spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('crack_ros', anonymous=True)
    detector = CrackDetector()
    rospy.Subscriber('/carrot_team/poi_idx', Int16, detector.poi_callback)
    rospy.Subscriber('/red/camera/color/image_raw', Image, detector.img_callback)
    t = threading.Thread(target = detector.rospy_spin)
    t.start()
    
    while not rospy.is_shutdown():
        detector.detect_callback()
