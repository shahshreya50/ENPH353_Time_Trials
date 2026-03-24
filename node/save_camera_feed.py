#! /usr/bin/env python3

"""@package docstring
This file is a node which subscribes to image data from a camera saves a video stream of the images
"""

import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os

#initialize objects/nodes
path = os.path.expanduser("~/Videos/RobotVideoFeed.mp4")
rospy.init_node('save_video', anonymous=True)
bridge = CvBridge()
videoWriter = cv2.VideoWriter(path, cv2.VideoWriter.fourcc('M', 'P', '4', 'V'), 15.00, [800,800],True)
print("Initialization of save_video node successful")

if not videoWriter.isOpened():
  print("Error: Could not open output video file")

#define callback behaviour when image data received
def callback(data):
    """This function saves each new image recieved to a video file
    """

    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print (e)

    #add image to video
    videoWriter.write(cv_image)


def shutdown():
    videoWriter.release()

#create subscriber node
image_sub = rospy.Subscriber("/B1/rrbot/camera1/image_raw", Image, callback)
rospy.on_shutdown(shutdown)


#keep python running until this node is killed so that callback can be triggered
rospy.spin()
