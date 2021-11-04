import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
import message_filters

from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge,  CvBridgeError
import cv2
import pyrealsense2 as rs
# pip3 install pyrealsense2

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag
from dynamic_reconfigure.server import Server
from irob_vision_support.cfg import FiducialsConfig

from irob_utils.rigid_transform_3D import rigid_transform_3D
#import irob_utils

class FiducialDetector:


    def __init__(self):

        print("Init")


        self.lower_red = (150, 100, 50)
        self.upper_red = (180, 255, 255)
        self.lower_darkred = (0, 100, 50)
        self.upper_darkred = (7, 255, 255)
        self.lower_yellow = (20, 100, 100)
        self.upper_yellow = (60, 255, 255)
        #self.lower_green = (40, 120, 0)
        #self.upper_green = (70, 255, 255)
        self.lower_green = (60, 120, 0)
        self.upper_green = (90, 255, 255)
        self.lower_orange = (8, 100, 100)
        self.upper_orange = (20, 255, 255)
        #self.lower_purple = (0, 50, 0)
        #self.upper_purple = (10, 170, 255)
        #self.lower_darkpurple = (150, 50, 0)
        #self.upper_darkpurple = (180, 170, 255)
        self.lower_purple = (105, 0, 0)
        self.upper_purple = (130, 255, 255)
        self.lower_darkpurple = (180, 170,255)
        self.upper_darkpurple = (180, 170, 255)
        #self.lower_background = (90, 0, 0)
        #self.upper_background = (180, 255, 255)

        self.fiducial_colors = ['red', 'green', 'orange', 'purple']
        #self.fiducial_colors = ['red', 'yellow', 'green', 'orange', 'purple']


        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.30
        #self.exposure = 1500.0
        self.exposure = 1800.0 #1000.0

        self.z_offset = 0.004   # m

        #image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        #depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        #info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)

        #ts = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub], 10)
        #ts.registerCallback(self.cb_images)


        # Init RealSense
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.width,
                                    self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width,
                                    self.height, rs.format.bgr8, self.fps)


        rospy.init_node('fiducial_detector', anonymous=True)
        srv = Server(FiducialsConfig, self.cb_config)
        self.bridge = CvBridge()

        #rospy.spin()



    # realsense
    def start_and_process_stream(self):

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

                # Remove background - Set pixels further than clipping_distance to grey
                grey_color = 0
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image))
                #depth image is 1 channel, color is 3 channels
                bg_removed = np.where((depth_image_3d > clipping_distance)
                                        | (depth_image_3d <= 0), grey_color, color_image)

                # Render images:
                #   depth align to color on left
                #   depth on right
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
                                                                    cv2.COLORMAP_JET)
                images = np.hstack((bg_removed, depth_colormap))

                #self.kmeans_segmentation(bg_removed)
                fiducials, output = self.find_fiducials_locations(bg_removed)
                #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
                #cv2.imshow('Align Example', images)
                #cv2.waitKey(1)

                intrinsics = (aligned_depth_frame.profile
                            .as_video_stream_profile().get_intrinsics())
                w_h = depth_image.shape

                #print("intrinsics")
                #print(intrinsics)

                #print("camera_info")
                #print(profile.camera_info)
                #break


                fid_position = {}
                for color in fiducials:
                    for i in range(len(fiducials[color])):
                        fid_position[color] = []
                        if (fiducials[color] and fiducials[color]):
                            res, output = self.get_marker_position(fiducials[color][i], aligned_depth_frame,
                                                                                w_h, intrinsics, output)
                            fid_position[color].append(res)

                            #print(color + ": " + str(fid_posistion[color]))


                self.calc_pose(fid_position, output)

                cv2.imshow("Output",  output)
                cv2.waitKey(1)


        finally:
            self.pipeline.stop()

    #
    def get_marker_position(self, img_coords, aligned_depth_frame, w_h, intrinsics, output):
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
        pos = (pos / N) - np.array([0.0,0.0,self.z_offset])

        text = "(" + str(int(round(pos[0]*1000.0))) + "," + str(int(round(pos[1]*1000.0))) + "," + str(int(round(pos[2]*1000.0))) + ")"

        output = cv2.putText(output, text, (x_c,y_c), cv2.FONT_HERSHEY_SIMPLEX,
                           0.5, (255,255,255), 1, cv2.LINE_AA)

        return pos, output


    #
    def calc_pose(self, fid_position, output):

        rigid_transform_3D([1,1,1], [0,0,0])


    #
    def set_exposure(self, exposure):
        rgb_cam_sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        rgb_cam_sensor.set_option(rs.option.exposure, exposure)

    #
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
       #self.kmeans_segmentation(image)

    def kmeans_segmentation(self, image):

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask_background = cv2.inRange(hsv_img, self.lower_background, self.upper_background)
        mask_background = 255 - mask_background

        hsv_img = cv2.bitwise_and(hsv_img, hsv_img, mask=mask_background)


        pixel_values = hsv_img.reshape((-1, 3))
        pixel_values = np.float32(pixel_values)

        # define stopping criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.2)

        k = 8
        _, labels, (centers) = cv2.kmeans(pixel_values, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        centers = np.uint8(centers)

        labels = labels.flatten()
        segmented_image = centers[labels.flatten()]
        # reshape back to the original image dimension
        segmented_image = segmented_image.reshape(image.shape)
        # show the image
        segmented_image_bgr = cv2.cvtColor(segmented_image, cv2.COLOR_HSV2BGR)
        cv2.imshow("KMeans", segmented_image_bgr)
        cv2.waitKey(1)


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
            hsv_lower_2 = self.lower_darkpurple
            hsv_upper_2 = self.upper_darkpurple
            n = 1
        else:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)

        #if color == 'red' or color == 'purple':
        #    mask_2 = cv2.inRange(hsv_img, hsv_lower_2, hsv_upper_2)
        #    mask = cv2.bitwise_or(mask, mask_2)

        mask_background = cv2.inRange(hsv_img, self.lower_background, self.upper_background)
        mask_background = 255 - mask_background
        #mask = cv2.bitwise_and(mask, mask_background)

        kernel = np.ones((3,3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)



        result = cv2.bitwise_and(image, image, mask=mask)
        #if color == 'purple':
        #    cv2.imshow("Mask", result)
        #    cv2.waitKey(1)

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

            if area > 80 and area < 10000 and e < 0.98and w < 150 and h < 150:
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


        output = image.copy()
        fiducials = {}

        for color in self.fiducial_colors:
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
        return fiducials, output





if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = FiducialDetector()
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_104229.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")

    #for image in detector.cv_images:
    #image = detector.cv_images[2]

        #detector.find_fiducials_locations(image)
    detector.start_and_process_stream()

    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    print("Shutting down")
    cv2.destroyAllWindows()
