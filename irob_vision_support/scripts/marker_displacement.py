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
      
      if (len(data.markers) > 1):   #only works with 2 markers! or the first 2 detected
         
          #first marker corners and center
          x1 = data.markers[0].corners[0].x
          y1 = data.markers[0].corners[0].y

          x2 = data.markers[0].corners[1].x
          y2 = data.markers[0].corners[1].y

          x3 = data.markers[0].corners[2].x
          y3 = data.markers[0].corners[2].y

          x4 = data.markers[0].corners[3].x
          y4 = data.markers[0].corners[3].y

          kp1=Point()
          kp1.x=x1+(x3-x1)/2
          kp1.y=y4+(y2-y4)/2

          #second marker corners and center
          x1 = data.markers[1].corners[0].x
          y1 = data.markers[1].corners[0].y

          x2 = data.markers[1].corners[1].x
          y2 = data.markers[1].corners[1].y

          x3 = data.markers[1].corners[2].x
          y3 = data.markers[1].corners[2].y

          x4 = data.markers[1].corners[3].x
          y4 = data.markers[1].corners[3].y

          kp2=Point()
          kp2.x=x1+(x3-x1)/2
          kp2.y=y4+(y2-y4)/2
          
          print("kp1:", kp1, ", kp2:", kp2)


          # distance calculation from center; ratio
          dist=Point()
          dist.x=((kp1.x+kp2.x)/2-(w/2))/(w/2)
          dist.y=((kp1.y+kp2.y)/2-(h/2))/(h/2)
          

          #depth calculation from the markers distance
          if (h > w):
              k=w
          else:
              k=h
          #z-hez arány! nem előjeles! alsó és felső treshold kell
          markers_distance=math.sqrt((kp1.x-kp2.x)**2+(kp1.y-kp2.y)**2)/k 

          threshold_for_zoom=0.2  #0-1
          dist.z=markers_distance-threshold_for_zoom  # -0.2 and 0.8

      
                  

          #publicate to qrkam
          self.pos_pub.publish(dist)


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

