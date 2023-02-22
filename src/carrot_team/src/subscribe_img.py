#!/usr/bin/env python

import rospy
from mavros_msgs.msg import CameraImageCaptured

def camera_callback(msg):
    # img_data = msg.image_data
    # img_height = msg.height
    # img_width = msg.width
    # timestamp = msg.header.stamp
    # # Do something with the image data (e.g. save it to a file)
    # # ...
    print(msg)
    print(1)
    print()

print('test')

rospy.init_node('camera_subscriber', anonymous=True)
rospy.Subscriber("/red/mavros/camera/image_captured", CameraImageCaptured, camera_callback)
rospy.spin()