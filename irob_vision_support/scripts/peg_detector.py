import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import CameraInfo
import message_filters
from cv_bridge import CvBridge
import cv2

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag


class PegDetector:
    def __init__(self):
        rospy.init_node('peg_detector', anonymous=True)
        print("Init")
        self.lower_red = (150, 120, 100)
        self.upper_red = (180, 255, 255)
        self.lower_darkred = (0, 150, 110)
        self.upper_darkred = (0, 255, 250)
        self.lower_yellow = (0, 20, 150)
        self.upper_yellow = (80, 220, 255)
        self.lower_green = (80, 150, 50)
        self.upper_green = (100, 255, 200)
        self.lower_white = (70, 0, 220)
        self.upper_white = (100, 255, 255)

        image_sub = message_filters.Subscriber("/device_0/sensor_1/Color_0/image/data", Image)
        depth_sub = message_filters.Subscriber("/device_0/sensor_0/Depth_0/image/data", Image)
        info_sub = message_filters.Subscriber("/device_0/sensor_0/Depth_0/info/camera_info", CameraInfo)

        #image_sub = message_filters.Subscriber("/device_0/sensor_1/Color_0/image/data", Image)
        #depth_sub = message_filters.Subscriber("/device_0/sensor_0/Depth_0/image/data", Image)
        #info_sub = message_filters.Subscriber("/device_0/sensor_0/Depth_0/info/camera_info", CameraInfo)

        ts = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub], 10)
        ts.registerCallback(self.cb_images)



    def cb_images(self,image_msg,depth_msg,camera_info):
        print("in cb_images")
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg,
                             desired_encoding=image_msg.encoding)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            depth = self.bridge.imgmsg_to_cv2(depth_msg,
                             desired_encoding=depth_msg.encoding)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image", image)
        cv2.waitKey(1)
        print()
        print("Image")
        print(image_msg.header)
        print(image_msg.encoding)
        print("Depth")
        print(depth_msg.header)
        print(depth_msg.encoding)
        print("Info")
        print(camera_info)

    def load_img(self, bagfile):
        print("Loading images...")
        self.bridge = CvBridge()
        bag = rosbag.Bag(bagfile)
        self.cv_images = []
        for topic, msg, t in bag.read_messages(topics=['/device_0/sensor_0/Depth_0/image/data', '/device_0/sensor_1/Color_0/image/data']):
            try:
                self.cv_images.append(self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8'))
                print("Load img")
            except CvBridgeError as e:
                print(e)
            break
        print(f'Loaded {len(self.cv_images)} images.')


        print("ez?")
        ts = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub],10)

        ts.registerCallback(self.cb_images)
        #self.cb_images(ts.registerCallback, depth_sub)


        #cv2.imshow("Image", self.cv_images[0])
        #cv2.waitKey(0)

        bag.close()
        print("Bag closed")

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
        #cv2.imshow("Image window", self.cv_images[0])	#
        #cv2.waitKey(0)	#


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
        print("mask?")
        x,y,w,h = cv2.boundingRect(mask)

        cv2.imshow("Image window", self.cv_images[0])
        cv2.imshow("Image", result)
        fiducial = self.find_fiducials_locations(result)
        cv2.waitKey(0)

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

        print(fiducials)
        cv2.imshow("Output",  output)
        cv2.waitKey(1)
        return fiducials

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


if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = PegDetector()
    #detector.load_img("/home/benceubuntu/Documents/bags/20210713_141200.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")

    #detector.segment_fiducials()

    rospy.spin()
    cv2.destroyAllWindows()
