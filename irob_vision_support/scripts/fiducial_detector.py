import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
import message_filters

from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge,  CvBridgeError
import cv2
import pyrealsense2 as rs2

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag
from dynamic_reconfigure.server import Server
from irob_vision_support.cfg import FiducialsConfig


class FiducialDetector:


    def __init__(self):

        print("Init")
        rospy.init_node('fiducial_detector', anonymous=True)
        srv = Server(FiducialsConfig, self.cb_config)
        self.bridge = CvBridge()

        self.lower_red = (150, 120, 100)
        self.upper_red = (180, 255, 255)
        self.lower_darkred = (0, 150, 110)
        self.upper_darkred = (0, 255, 250)
        self.lower_yellow = (0, 20, 150)
        self.upper_yellow = (80, 220, 255)
        self.lower_green = (80, 150, 50)
        self.upper_green = (100, 255, 200)
        self.lower_orange = (70, 0, 220)
        self.upper_orange = (100, 255, 255)
        self.lower_purple = (70, 0, 220)
        self.upper_purple = (100, 255, 255)
        #self.lower_background = (90, 0, 0)
        #self.upper_background = (180, 255, 255)

        image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)

        ts = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub], 10)
        ts.registerCallback(self.cb_images)


        #rospy.spin()

    def cb_config(self, config, level):
        self.lower_background = (config.bg_l_h, config.bg_l_s, config.bg_l_v)
        self.upper_background = (config.bg_u_h, config.bg_u_s, config.bg_u_v)
        return config


    # Synced callback function for images
    def cb_images(self,image_msg,depth_msg,camera_info):
       try:
           image = self.bridge.imgmsg_to_cv2(image_msg,
                            desired_encoding=image_msg.encoding)
           image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
           depth = self.bridge.imgmsg_to_cv2(depth_msg,
                            desired_encoding=depth_msg.encoding)
       except CvBridgeError as e:
           print(e)

       #cv2.imshow("Image", image)
       #cv2.waitKey(1)
       #print()
       #print("Image")
       #print(image_msg.header)
       #print(image_msg.encoding)
       #print("Depth")
       #print(depth_msg.header)
       #print(depth_msg.encoding)
       #print("Info")
       #print(camera_info)

       self.find_fiducials_locations(image)



    def mask_fiducial(self, image, color):

        if color == 'red':
            hsv_lower = self.lower_red
            hsv_upper = self.upper_red
            hsv_lower_2 = self.lower_darkred
            hsv_upper_2 = self.upper_darkred
            n = 1
        elif color == 'yellow':
            hsv_lower = self.lower_yellow
            hsv_upper = self.upper_yellow
            n = 2
        elif color == 'green':
            hsv_lower = self.lower_green
            hsv_upper = self.upper_green
            n = 1
        elif color == 'orange':
            hsv_lower = self.lower_orange
            hsv_upper = self.upper_orange
            n = 1
        elif color == 'purple':
            hsv_lower = self.lower_purple
            hsv_upper = self.upper_purple
            n = 1
        else:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)

        if color == 'red':
            mask_2 = cv2.inRange(hsv_img, hsv_lower_2, hsv_upper_2)
            mask = cv2.bitwise_or(mask, mask_2)

        mask_background = cv2.inRange(hsv_img, self.lower_background, self.upper_background)
        mask_background = 255 - mask_background
        mask = cv2.bitwise_and(mask, mask_background)

        kernel = np.ones((3,3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)



        result = cv2.bitwise_and(image, image, mask=mask_background)
        if color == 'red':
            cv2.imshow("Mask", result)
            cv2.waitKey(1)

        # apply connected component analysis to the thresholded image
        output = cv2.connectedComponentsWithStats(
                    mask, 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = output

        ret = []
        for i in range(numLabels):
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            x = stats[i, cv2.CC_STAT_LEFT] + (w/2.0)
            y = stats[i, cv2.CC_STAT_TOP] + (h/2.0)
            area = stats[i, cv2.CC_STAT_AREA]
            e = math.sqrt(abs((w**2)-(h**2)))/w
            r = (w+h)/4.0
            if area > 80 and area < 10000 and e < 0.7 and w < 150 and h < 150:
                ret.append([x,y,r])

        while len(ret) > n:
            min_r = float('inf')
            min_r_i = 0
            for i in range(len(ret)):
                if ret[i][2] < min_r:
                    min_r = ret[i][2]
                    min_r_i = i
            ret.pop(min_r_i)

        return ret




    def find_fiducials_locations(self, image):

        fiducial_colors = ['red', 'yellow', 'green', 'orange', 'purple']
        output = image.copy()
        fiducials = {}

        for color in fiducial_colors:
            fiducials[color] = self.mask_fiducial(image, color)
            for f in fiducials[color]:
                x = f[0]
                y = f[1]
                r = f[2]
                cv2.circle(output, (int(x), int(y)), int(r),
                                                (0, 255, 0), 4)
                cv2.rectangle(output, (int(x) - 5, int(y) - 5),
                                                (int(x) + 5, int(y) + 5),
                                                (0, 128, 255), -1)

        #print(fiducials)
        #cv2.imshow("Output",  output)
        #cv2.waitKey(1)
        return fiducials





if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = FiducialDetector()
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_104229.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")

    #for image in detector.cv_images:
    #image = detector.cv_images[2]

        #detector.find_fiducials_locations(image)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
