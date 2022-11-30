#!/usr/bin/env python

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

class image_subscriber:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("stereo/preprocessed/left/image_rect",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  print("Node started")
  #help(cv2.aruco)

  aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
  print(aruco_dict)
  # second parameter is id number
  # last parameter is total image size
  #img = aruco.drawMarker(aruco_dict, 2, 700)
  #cv2.imwrite("/home/tamas/Pictures/test_marker.jpg", img)
  #cv2.imwrite("test_marker.jpg", img)

  img = cv2.imread('/home/dvrk_nat/Pictures/marker_photo.jpg',0)
  #cv2.imshow(img)
  #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  #aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
  parameters =  aruco.DetectorParameters_create()
  print(parameters)
  corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
  print(corners)

  img = aruco.drawDetectedMarkers(img, corners)

  cv2.imshow('frame',img)
  cv2.waitKey()

  ic = image_subscriber()
  rospy.init_node('image_subscriber', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
