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


    def segment_fiducials(self):

        pixel_colors = self.cv_images[0].reshape((
                    np.shape(self.cv_images[0])[0]*np.shape(self.cv_images[0])[1], 3))
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(pixel_colors)
        pixel_colors = norm(pixel_colors).tolist()

        hsv_img = cv2.cvtColor(self.cv_images[0], cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)
        fig = plt.figure()
        axis = fig.add_subplot(1, 1, 1, projection="3d")
        #cv2.imshow("Image window", self.cv_images[0])
        #cv2.waitKey(0)


        #axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
        #axis.set_xlabel("Hue")
        #axis.set_ylabel("Saturation")
        #axis.set_zlabel("Value")
        #plt.show()




        light_red = (0, 150, 110)
        dark_red = (12, 255, 250)
        mask = cv2.inRange(hsv_img, light_red, dark_red)
        light_red_2 = (170, 150, 110)
        dark_red_2 = (180, 255, 250)
        mask_2 = cv2.inRange(hsv_img, light_red_2, dark_red_2)
        mask = cv2.bitwise_or(mask, mask_2)
        result = cv2.bitwise_and(self.cv_images[0], self.cv_images[0], mask=mask)

        kernel = np.ones((3,3), np.uint8)

        # The first parameter is the original image,
        # kernel is the matrix with which image is
        # convolved and third parameter is the number
        # of iterations, which will determine how much
        # you want to erode/dilate a given image.
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        x,y,w,h = cv2.boundingRect(mask)

        cv2.imshow("Image window", self.cv_images[0])
        cv2.imshow("Image", result)
        cv2.waitKey(0)



if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = FiducialDetector()
    detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_104229.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")

    detector.segment_fiducials()

    cv2.destroyAllWindows()
