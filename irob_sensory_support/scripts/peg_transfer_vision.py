#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function







import roslib
roslib.load_manifest('irob_sensory_support')
import sys
import rospy
import cv2
import numpy as np
from threading import Lock, Thread
from std_msgs.msg import String
from sensor_msgs.msg import Image
from irob_msgs.msg import Environment
from cv_bridge import CvBridge, CvBridgeError

class peg_transfer_vision:

  def __init__(self):
    self.target_pub = rospy.Publisher("vision/target",Environment, queue_size=10)

    self.bridge = CvBridge()
    self.cv_image_left_lock = Lock()
    self.cv_image_right_lock = Lock()
    self.cv_image_left = np.zeros((576,720,1), np.uint8)
    self.cv_image_right = np.zeros((576,720,1), np.uint8)
    self.image_left_sub = rospy.Subscriber("stereo/preprocessed/left/image_rect",Image,self.callback_image_left)
    self.image_right_sub = rospy.Subscriber("stereo/preprocessed/right/image_rect",Image,self.callback_image_right)


  def callback_image_left(self,data):
    try:
      self.cv_image_left_lock.acquire()
      self.cv_image_left = self.bridge.imgmsg_to_cv2(data, "mono8")
      self.cv_image_left_lock.release()
    except CvBridgeError as e:
      print(e)


    #cv2.imshow("Image window left", cv_image)
    #cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

  def callback_image_right(self,data):
      try:
        self.cv_image_right_lock.acquire()
        self.cv_image_right = self.bridge.imgmsg_to_cv2(data, "mono8")
        self.cv_image_right_lock.release()
      except CvBridgeError as e:
        print(e)


      #cv2.imshow("Image window right", cv_image)
      #cv2.waitKey(3)



  def do_image_processing(self):

      self.cv_image_right_lock.acquire()
      self.cv_image_left_lock.acquire()

      img_right = cv2.equalizeHist(self.cv_image_right)
      img_left = cv2.equalizeHist(self.cv_image_left)

      edge_left = cv2.Canny(img_left,100,200)

      self.cv_image_left_lock.release()
      self.cv_image_right_lock.release()

      res = np.hstack((img_left,edge_left))
      cv2.imshow("Image window", res)
      cv2.waitKey(3)

      #try:
      #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      #except CvBridgeError as e:
      #  print(e)


def main(args):
  ptv = peg_transfer_vision()
  rospy.init_node('peg_transfer_vision', anonymous=True)

  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
       ptv.do_image_processing()
       r.sleep()


  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)





















