import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge
import cv2

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag


class FiducialDetector:
    def __init__(self):
        #rospy.init_node('fiducial_detector', anonymous=True)
        print("Init")
        #rospy.spin()


    def load_img(self, bagfile):
        print("Loading images...")
        self.bridge = CvBridge()
        bag = rosbag.Bag(bagfile)
        self.cv_images = []
        i = 0
        for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw',
                                        '/device_0/sensor_1/Color_0/image/data']):
            try:
                self.cv_images.append(self.bridge.imgmsg_to_cv2(msg,
                                                desired_encoding='bgr8'))
                #print("Load img")
            except CvBridgeError as e:
                print(e)
            i += 1
            if i > 50000:
                break
        print(f'Loaded {len(self.cv_images)} images.')
        #cv2.imshow("Image", self.cv_images[0])
        #cv2.waitKey(0)
        bag.close()


        self.lower_red = (0, 150, 110)
        self.upper_red = (12, 255, 250)
        self.lower_darkred = (170, 150, 110)
        self.upper_darkred = (180, 255, 250)
        self.lower_yellow = (20, 150, 150)
        self.upper_yellow = (30, 255, 255)
        self.lower_green = (50, 80, 50)
        self.upper_green = (70, 255, 255)
        self.lower_white = (20, 0, 200)
        self.upper_white = (40, 120, 255)




    def mask_fiducial(self, image, color):

        if color == 'red':
            hsv_lower = self.lower_red
            hsv_upper = self.upper_red
            hsv_lower_2 = self.lower_darkred
            hsv_upper_2 = self.upper_darkred
        elif color == 'yellow':
            hsv_lower = self.lower_yellow
            hsv_upper = self.upper_yellow
        elif color == 'green':
            hsv_lower = self.lower_green
            hsv_upper = self.upper_green
        elif color == 'white':
            hsv_lower = self.lower_white
            hsv_upper = self.upper_white
        else:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)

        if color == 'red':
            mask_2 = cv2.inRange(hsv_img, hsv_lower_2, hsv_upper_2)
            mask = cv2.bitwise_or(mask, mask_2)

        kernel = np.ones((3,3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        result = cv2.bitwise_and(image, image, mask=mask)
        #cv2.imshow("Mask", result)
        #cv2.waitKey(0)

        # apply connected component analysis to the thresholded image
        output = cv2.connectedComponentsWithStats(
                    mask, 4, cv2.CV_32S)

        return output




    def detect_circles(self, image):
        # detect circles in the image
        gray = cv2.Canny(image,0,250)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,50,
            param1=250,param2=10,minRadius=0,maxRadius=60)

        #output = image.copy()
        #for (x, y, r) in circles[0,:]:
            #cv2.circle(output, (int(x), int(y)), int(r),
             #                   (255, 255, 0), 4)
            #cv2.rectangle(output, (int(x) - 5, int(y) - 5),
             #                   (int(x) + 5, int(y) + 5),
             #                   (255, 128, 255), -1)
        #cv2.imshow("Circles", output)
        #cv2.waitKey(0)


        return circles




    def find_fiducials_locations(self, image):

        fiducial_colors = ['red', 'yellow', 'green', 'white']
        circles = self.detect_circles(image)
        valid_circles = {}
        dist_threshold = 10.0
        dist_threshold_sq = dist_threshold * dist_threshold
        output = image.copy()

        for color in fiducial_colors:
            valid_circles[color] = []
            cc = self.mask_fiducial(image, color)
            if cc:
                (numLabels, labels, stats, centroids) = cc
                for i in range(numLabels):
                    cc_x = stats[i, cv2.CC_STAT_LEFT]
                    cc_y = stats[i, cv2.CC_STAT_TOP]
                    cc_w = stats[i, cv2.CC_STAT_WIDTH]
                    cc_h = stats[i, cv2.CC_STAT_HEIGHT]
                    cc_area = stats[i, cv2.CC_STAT_AREA]


                    if cc_area > 10 and cc_area < 3000:
                        min_dist_sq = float('inf')
                        closest_circle = []
                        for circle in circles[0, :]:
                            circ_x, circ_y, circ_r = circle
                            dist_sq = (((cc_x + cc_w/2.0) - circ_x) ** 2) + \
                                        (((cc_y + cc_h/2.0) - circ_y) ** 2)
                            if dist_sq <= dist_threshold_sq and dist_sq <= min_dist_sq:
                                min_dist_sq = dist_sq
                                closest_circle = circle
                        if min_dist_sq < float('inf'):
                            valid_circles[color].append(closest_circle)
                            circ_x, circ_y, circ_r = closest_circle
                            cv2.circle(output, (int(circ_x), int(circ_y)), int(circ_r),
                                                (0, 255, 0), 4)
                            cv2.rectangle(output, (int(circ_x) - 5, int(circ_y) - 5),
                                                (int(circ_x) + 5, int(circ_y) + 5),
                                                (0, 128, 255), -1)

        #print(valid_circles)
        cv2.imshow("Output",  output)
        cv2.waitKey(0)
        return valid_circles





if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = FiducialDetector()
    detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_104229.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")

    for image in detector.cv_images:
    #image = detector.cv_images[2]

        detector.find_fiducials_locations(image)

    cv2.destroyAllWindows()
