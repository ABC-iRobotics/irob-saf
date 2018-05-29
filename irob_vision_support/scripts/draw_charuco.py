#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This script creates and saves a charuco marker in the current directory

from __future__ import print_function

import roslib
roslib.load_manifest('irob_vision_support')
import sys
import rospy
import cv2
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
print(aruco_dict)

board = aruco.CharucoBoard_create(3,3,.025,.0125,aruco_dict)

img = board.draw((700,700))
cv2.imwrite("marker.jpg", img)
print("Marker saved to file.")
