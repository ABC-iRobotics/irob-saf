#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib
roslib.load_manifest('irob_vision_support')
import sys
import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from irob_msgs.msg import Point2D
from irob_msgs.msg import Marker
from irob_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError

class aruco_detector:

  # Constructor
  def __init__(self):
    # Declare aruco stuff
    self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    self.parameters =  aruco.DetectorParameters_create()

    self.image_pub = rospy.Publisher("image_markers",Image, queue_size=10)
    self.marker_pub = rospy.Publisher("markers",MarkerArray, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_input",Image,self.callback)



  # Callback for image topic
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
      #print(corners)

      #img = aruco.drawDetectedMarkers(cv_image, corners)

      marker_msg = MarkerArray()
      marker_msg.header = data.header
      marker_msg.markers = []
      #print(corners)
      if not ids is None:
        if len(ids) != 0:
          for i in range(len(ids)):
            marker_msg.markers.append(Marker())
            marker_msg.markers[i].id = int(ids[i])
            marker_msg.markers[i].corners = []
            for j in range((corners[i]).shape[1]):
              marker_msg.markers[i].corners.append(Point2D())
              marker_msg.markers[i].corners[j].x = corners[i][0,j,0]
              marker_msg.markers[i].corners[j].y = corners[i][0,j,1]

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.marker_pub.publish(marker_msg)
    except CvBridgeError as e:
      print(e)






# Main function
def main(args):
  print("Node started")
  #help(cv2.aruco)

  detector = aruco_detector()
  rospy.init_node('aruco_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

