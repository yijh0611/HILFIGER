#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Int16
from queue import Queue
import threading
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
import cv2

class SquareDetector : 
    
    def __init__(self): 
        
        self.bridge = CvBridge()
        self.threshold_low = 190
        self.threshold_high = 255
        self.square_bbox = Int32MultiArray()
        self.poi_queue = Queue()
        self.img_queue = Queue()
        self.data = {}
        self.pub = rospy.Publisher('/carrot_team/square_bbox', Int32MultiArray, queue_size = 10)

    def poi_callback(self, msg):

        self.poi_queue.put(msg.data)

        try:
            self.img_queue.put(self.data['img'])

        except KeyError:
            pass

    def img_callback(self, msg):

        self.data['img'] = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def square_callback(self):

        if self.img_queue.qsize() == 0:
            pass
        
        else:
            img = self.img_queue.get()
            poi = self.poi_queue.get()
            img_display = img.copy()
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
                        cv2.rectangle(img_display, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    
            self.square_bbox.data = xyxy
            self.pub.publish(self.square_bbox)

    def rospy_spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('square_ros', anonymous=True)
    detector = SquareDetector()
    rospy.Subscriber('/carrot_team/poi_idx', Int16, detector.poi_callback)
    rospy.Subscriber('/red/camera/color/image_raw', Image, detector.img_callback)
    t = threading.Thread(target = detector.rospy_spin)
    t.start()

    while not rospy.is_shutdown():
        detector.square_callback()