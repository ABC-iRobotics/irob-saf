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
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Transform, TransformStamped

from geometry_msgs.msg import Pose2D
from irob_msgs.msg import Point2D

import pyrealsense2 as rs
# pip3 install pyrealsense2

import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag
from irob_msgs.msg import Marker
from irob_msgs.msg import MarkerArray
from irob_msgs.msg import VisionObject
from irob_msgs.msg import VisionObjectArray



from scipy.spatial.transform import Rotation
import yaml


class aruco_detector_realsense:

    # Constructor
    def __init__(self):
        # Declare aruco stuff
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()

        self.image_pub = rospy.Publisher("image_markers",Image, queue_size=10)
        self.marker_pub = rospy.Publisher("markers",MarkerArray, queue_size=10)
        self.vision_objs_pub = rospy.Publisher("vision_objects",VisionObjectArray, queue_size=10)

        # Init RealSense
        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.30
        self.exposure = 150.0
        #self.exposure = 150.0 #1800.0 #1000.0

        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.width,
                                        self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width,
                                        self.height, rs.format.bgr8, self.fps)

        self.bridge = CvBridge()


    def start_and_process_stream(self):
         """Connect to a RealSense camera and finds
         the position and orientation of a fiducial.
         """

         # Start streaming
         self.profile = self.pipeline.start(self.config)
         self.set_exposure(self.exposure)

         # Getting the depth sensor's depth scale (see rs-align example for explanation)
         depth_sensor = self.profile.get_device().first_depth_sensor()
         depth_scale = depth_sensor.get_depth_scale()
         print("Depth Scale is: " , depth_scale)

         # We will be removing the background of objects more than
         #  clipping_distance_in_meters meters away

         clipping_distance = self.clipping_distance_in_meters / depth_scale

         # Create an align object
         # rs.align allows us to perform alignment of depth frames to others frames
         # The "align_to" is the stream type to which we plan to align depth frames.
         align_to = rs.stream.color
         align = rs.align(align_to)

         # Streaming loop
         try:
             while not rospy.is_shutdown():
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                intrinsics = (aligned_depth_frame.profile
                            .as_video_stream_profile().get_intrinsics())
                w_h = depth_image.shape


                (rows,cols,channels) = color_image.shape
                if cols > 60 and rows > 60 :
                    corners, ids, rejectedImgPoints = aruco.detectMarkers(color_image, self.aruco_dict,
                    parameters=self.parameters)
                    #print(corners)

                    output = aruco.drawDetectedMarkers(color_image, corners)

                    stamp = rospy.Time.now()

                    marker_msg = MarkerArray()
                    marker_msg.header.stamp = stamp
                    marker_msg.markers = []

                    vision_objs_msg = VisionObjectArray()
                    vision_objs_msg.header.stamp = stamp
                    vision_objs_msg.objects = []

                    #print(corners)
                    if not ids is None:
                        if len(ids) != 0:
                            for i in range(len(ids)):
                                marker_msg.markers.append(Marker())

                                marker_msg.markers[i].id = int(ids[i])

                                marker_msg.markers[i].corners = []
                                im_cx = 0.0
                                im_cy = 0.0
                                for j in range((corners[i]).shape[1]):
                                    marker_msg.markers[i].corners.append(Point2D())
                                    marker_msg.markers[i].corners[j].x = corners[i][0,j,0]
                                    marker_msg.markers[i].corners[j].y = corners[i][0,j,1]
                                    im_cx = im_cx + corners[i][0,j,0]
                                    im_cy = im_cy + corners[i][0,j,1]

                                im_cx = im_cx / 4.0
                                im_cy = im_cy / 4.0

                                pos, output = self.get_marker_position(
                                                        [im_cx, im_cy, 1.0], aligned_depth_frame,
                                                        w_h, intrinsics, output)

                                if pos is not None:
                                    vision_objs_msg.objects.append(VisionObject())
                                    vision_objs_msg.objects[i].id = int(ids[i])
                                    vision_objs_msg.objects[i].transform.translation.x = pos[0]
                                    vision_objs_msg.objects[i].transform.translation.y = pos[1]
                                    vision_objs_msg.objects[i].transform.translation.z = pos[2]



                    cv2.imshow("Image window", output)
                    cv2.waitKey(3)

                    try:
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
                        self.marker_pub.publish(marker_msg)
                        self.vision_objs_pub.publish(vision_objs_msg)
                    except CvBridgeError as e:
                        print(e)


         finally:
             self.pipeline.stop()

    def set_exposure(self, exposure):
        """Set the camera exposure manually."""
        rgb_cam_sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        rgb_cam_sensor.set_option(rs.option.exposure, exposure)

    def get_marker_position(self, img_coords, aligned_depth_frame, w_h, intrinsics, output):
        """Get the 3D position of an object from
        image coordinates. Return the 3D position and output image.

        Keyword arguments:
        img_coords -- image coordinates of the object as a tuple [x, y, r]
        aligned_depth_frame -- aligned depth frame from RealSense
        w_h -- width and height of the image as tuple
        intrinsics -- camera intrinsics
        output -- output image with 3D position written
        """
        x_c = int(round(img_coords[0]))
        y_c = int(round(img_coords[1]))
        r = int(round(img_coords[2]))


        N = 0
        pos = np.array([0.0, 0.0, 0.0])
        a = max(1, int(round(math.sqrt(2.0 * r * r) - 10)))
        #print("a: " + str(a))

        for i in range(2*a+1):
            for j in range(2*a+1):
                x = (x_c - a) + i
                y = (y_c - a) + j

                if (x >= 0 and x < (w_h[1]-1) and y >=0 and y < (w_h[0]-1)):
                    d = aligned_depth_frame.get_distance(x,y)
                    #print("x: "  + str(x) + ", y: " + str(y) + ", d: "  + str(d))

                    if (d != 0 and d < self.clipping_distance_in_meters):
                        res = rs.rs2_deproject_pixel_to_point(intrinsics, [x,y], d)
                        pos = pos + res
                        N = N + 1

        if N == 0:
            return None, output
        pos = (pos / N) - np.array([0.0,0.0,0.0])

        text = "(" + str(int(round(pos[0]*1000.0))) + "," + str(int(round(pos[1]*1000.0))) + "," + str(int(round(pos[2]*1000.0))) + ")"

        output = cv2.putText(output, text, (x_c,y_c), cv2.FONT_HERSHEY_SIMPLEX,
                           0.5, (255,255,255), 1, cv2.LINE_AA)

        return pos, output





# Main function
def main(args):
  print("Node started")
  #help(cv2.aruco)

  detector = aruco_detector_realsense()
  rospy.init_node('aruco_detector', anonymous=True)
  try:
    detector.start_and_process_stream()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

