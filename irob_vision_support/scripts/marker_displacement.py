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
import math
from sensor_msgs.msg import Image

from irob_msgs.msg import Point2D
from irob_msgs.msg import Marker
from irob_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class marker_displacement:

  # Constructor
  def __init__(self):
    # Declare aruco stuff
    self.bridge = CvBridge()
    self.pos_pub = rospy.Publisher("qrkam",Point, queue_size=10)
    self.marker_sub = rospy.Subscriber("vision/markers",MarkerArray,self.callback)
    self.image_pub = rospy.Subscriber("vision/image_markers",Image, self.callback_2)
    self.frame=None
    self.marker_size_ratio = rospy.get_param('~marker_size_ratio')


  # Callback for image topic
  def callback(self,data):
    if not self.frame is None:
      h,w = self.frame.shape[:2]
      if (len(data.markers) > 0):
          p1 = data.markers[0].corners[0]
          x1=p1.x
          y1=p1.y
          p2 = data.markers[0].corners[1]
          x2=p2.x
          y2=p2.y
          p3 = data.markers[0].corners[2]
          x3=p3.x
          y3=p3.y
          p4 = data.markers[0].corners[3]
          x4=p4.x
          y4=p4.y

          ter=math.sqrt(abs((x2-x1)*(y4-y1)/(w*h)))  #százalékban van


          kul= ( self.marker_size_ratio-ter)   #ha nagyobb, tehát közelebb van, akkor nagyobb, mint 1!!!
          kp=Point()
          kp.x=x1+(x3-x1)/2
          kp.y=y4+(y2-y4)/2
          kp.z=kul
          #kp=np.array([x1+(x3-x1)/2, y4+(y2-y4)/2])
          #print(kp.x, kp.y,kp.z)
          # pub.publish(kp)

          tav=Point()
          tav.x=(kp.x-(w/2))/(w/2)
          tav.y=(kp.y-(h/2))/(h/2)

          z_scale_factor=10
          tav.z=kul*z_scale_factor
          print(tav.x, tav.y, tav.z)
          self.pos_pub.publish(tav)

  # Callback for image topic
  def callback_2(self,data):
    try:
      self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)



# Main function
def main(args):
  print("Node started")
  #help(cv2.aruco)


  rospy.init_node('marker_displacement', anonymous=True)
  detector = marker_displacement()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

