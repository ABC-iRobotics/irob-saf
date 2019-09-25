#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib
roslib.load_manifest('irob_vision_support')
import sys
import rospy
import numpy as np
import tf.transformations
import cv2
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from irob_msgs.msg import Point2D
from irob_msgs.msg import Marker
from irob_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError


class qr_code_position:

  # Constructor
  def __init__(self):


    self.marker_pub = rospy.Publisher('markers',geometry_msgs.msg.Point, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_input",Image,self.callback)



  # Callback for image topic
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


      # ha kiíratjuk háthatjuk, hogy a ret azt adja meg, hogy sikerült-e olvasni
      # print(ret)q

      # Múlt órai átalakítás
      qrDecoder = cv2.QRCodeDetector()
      width, height = cv.GetSize(cv_image)

      # Detect and decode the qrcode
      data,bbox,rectifiedImage = qrDecoder.detectAndDecode(cv_image)
      if len(data)>0:
          #print("Decoded Data : {}".format(data))
          n = len(bbox)
          for j in range(n):
              cv2.line(cv_image, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,0,0), 3)
          print(bbox)
          x1=bbox[0,0]
          x1=x1[0]
          x2=bbox[1,0]
          x2=x2[0]
          x3=bbox[2,0]
          x3=x3[0]
          y1=bbox[0,0]
          y1=y1[1]
          y2=bbox[1,0]
          y2=y2[1]
          y4=bbox[3,0]
          y4=y4[1]

          ter=(x2-x1)*(y4-y1)
          kul= (110*110 - ter)   #ha nagyobb, tehát közelebb van, akkor negatív!!!
          kp=geometry_msgs.msg.Point()
          kp.x=x1+(x3-x1)/2
          kp.y=y4+(y2-y4)/2
          kp.z=kul
          #kp=np.array([x1+(x3-x1)/2, y4+(y2-y4)/2])
          #print(kp.x, kp.y,kp.z)
         # pub.publish(kp)

          tav=geometry_msgs.msg.Point()
          tav.x=kp.x-(width/2)
          tav.y=kp.y-(height/2)
          if kul>0:
              tav.z=math.sqrt(kul)

          else:
              tav.z=-math.sqrt(-kul)

          kp.z=tav.z
          print(tav.x, tav.y, tav.z)



          #display(inputImage, bbox)
          #rectifiedImage = np.uint8(rectifiedImage);
          #cv2.imshow("Rectified QRCode", rectifiedImage);
          #ok = tracker.init(frame, bbox)
      cv2.imshow('frame',cv_image)

    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      marker_pub.publish(tav)
    except CvBridgeError as e:
      print(e)




# Main function
def main(args):
  print("Node started")
  rospy.init_node('qrkam', anonymous=True)
  detector = qr_code_position()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

