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
            if i > 5000:
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
            n = 1
        elif color == 'yellow':
            hsv_lower = self.lower_yellow
            hsv_upper = self.upper_yellow
            n = 3
        elif color == 'green':
            hsv_lower = self.lower_green
            hsv_upper = self.upper_green
            n = 1
        elif color == 'white':
            hsv_lower = self.lower_white
            hsv_upper = self.upper_white
            n = 1
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

        fiducial_colors = ['red', 'yellow', 'green', 'white']
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
        cv2.imshow("Output",  output)
        cv2.waitKey(5)
        return fiducials





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
