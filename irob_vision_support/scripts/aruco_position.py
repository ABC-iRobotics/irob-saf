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

from irob_msgs.msg import Point2D
from irob_msgs.msg import Marker
from irob_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class aruco_position:

  # Constructor
  def __init__(self):
    # Declare aruco stuff

    self.pos_pub = rospy.Publisher("qrkam",Point, queue_size=10)
    self.marker_sub = rospy.Subscriber("vision/markers",MarkerArray,self.callback)



  # Callback for image topic
  def callback(self,data):

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

      ter=(x2-x1)*(y4-y1)
      kul= (110*110 - ter)   #ha nagyobb, tehát közelebb van, akkor negatív!!!
      kp=Point()
      kp.x=x1+(x3-x1)/2
      kp.y=y4+(y2-y4)/2
      kp.z=kul
      #kp=np.array([x1+(x3-x1)/2, y4+(y2-y4)/2])
      #print(kp.x, kp.y,kp.z)
      # pub.publish(kp)

      tav=Point()
      tav.x=-(kp.x-320)
      tav.y=(kp.y-(475/2))
      if kul>0:
         tav.z=math.sqrt(kul)

      else:
         tav.z=-math.sqrt(-kul)

      kp.z=tav.z
      print(tav.x, tav.y, tav.z)
      self.pos_pub.publish(tav)





# Main function
def main(args):
  print("Node started")
  #help(cv2.aruco)

  detector = aruco_position()
  rospy.init_node('aruco_position', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

