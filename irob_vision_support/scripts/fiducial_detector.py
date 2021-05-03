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
        for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw', '/device_0/sensor_1/Color_0/image/data']):
            try:
                self.cv_images.append(self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8'))
                #print("Load img")
            except CvBridgeError as e:
                print(e)
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

        #x,y,w,h = cv2.boundingRect(mask)

        #cv2.imshow("Image", result)
        #cv2.waitKey(0)

        output = image.copy()
        # detect circles in the image
        gray = cv2.Canny(image,0,255)

        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Gray", gray)
        #cv2.waitKey(0)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1,50,
            param1=200,param2=15,minRadius=0,maxRadius=50)
        # ensure at least some circles were found
        if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.round(circles[0, :]).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                for (x, y, r) in circles:
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                        cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                # show the output image
                cv2.imshow("output",  output)
                cv2.waitKey(0)


        #cv2.imshow("Image window", image)

        return mask



if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = FiducialDetector()
    detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_104229.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")

    image = detector.cv_images[0]

    #cv2.imshow("Image window", image)
    #cv2.waitKey(0)

    mask_red = detector.mask_fiducial(image, 'red')
    #mask_yellow = detector.mask_fiducial(image, 'yellow')
    #mask_green = detector.mask_fiducial(image, 'green')
    #mask_white = detector.mask_fiducial(image, 'white')


    cv2.destroyAllWindows()
