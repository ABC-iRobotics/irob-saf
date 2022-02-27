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

from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter



#from irob_utils import rigid_transform_3D
#import irob_utils

class BlockDetector:


    def __init__(self, bagfile):

        print("Init")


        self.lower_red = (150, 100, 50)
        self.upper_red = (180, 255, 255)
        self.lower_darkred = (0, 100, 50)
        self.upper_darkred = (7, 255, 255)
        self.lower_yellow = (20, 0, 0)
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

        self.bagfile = bagfile

        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.50
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

        self.config.enable_device_from_file(self.bagfile)


        rospy.init_node('block_detector', anonymous=True)
        srv = Server(FiducialsConfig, self.cb_config)
        self.bridge = CvBridge()

        #rospy.spin()



    # realsense
    def start_and_process_stream(self):

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        #self.set_exposure(self.exposure)

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
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                cv2.namedWindow('Color image', cv2.WINDOW_NORMAL)
                cv2.imshow('Color image', color_image)
                cv2.waitKey(1)


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
                #block = self.mask_block(color_image, "yellow")

                segmented_blocks = self.segment_blocks(color_image, "yellow", 6)

                detected_blocks= []
                for i in range(len(segmented_blocks)):
                    detected_blocks.append(self.detect_block(segmented_blocks[i]))

                detected_blocks_stacked = cv2.hconcat(detected_blocks)

                cv2.namedWindow('Segmented', cv2.WINDOW_NORMAL)
                cv2.imshow('Segmented', detected_blocks_stacked)
                cv2.waitKey(1)

                intrinsics = (aligned_depth_frame.profile
                            .as_video_stream_profile().get_intrinsics())
                w_h = depth_image.shape

                #print("intrinsics")
                #print(intrinsics)

                #print("camera_info")
                #print(profile.camera_info)
                #break

                #print(color_image.shape)




                            #print(color + ": " + str(fid_posistion[color]))


                #self.calc_pose(fid_position, output)

                #cv2.imshow("Output",  output)
                #cv2.waitKey(1)


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








    def segment_blocks(self, image, col, n):


        if col == 'yellow':
            hsv_lower = self.lower_yellow
            hsv_upper = self.upper_yellow
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




        blobs = cv2.connectedComponentsWithStats(
                    mask, 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = blobs


        ret_r = []
        ret_im = []
        for i in range(numLabels):
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            x = stats[i, cv2.CC_STAT_LEFT] + (w/2.0)
            y = stats[i, cv2.CC_STAT_TOP] + (h/2.0)
            area = stats[i, cv2.CC_STAT_AREA]
            e = math.sqrt(abs((w**2)-(h**2)))/w
            r = (w+h)/4.0

            if area > 80 and area < 10000 and e < 0.98and w < 150 and h < 150:
                ret_r.append(r)
                mask_ret=np.where(labels == i, np.uint8(1), np.uint8(0))
                mask_ret = cv2.dilate(mask_ret, kernel, iterations=1)
                ret_im.append(cv2.bitwise_and(image, image, mask = mask_ret))


        while len(ret_r) > n:
            min_r = float('inf')
            min_r_i = 0
            for i in range(len(ret)):
                if ret_r[i] < min_r:
                    min_r = ret_r[i]
                    min_r_i = i
            ret_r.pop(min_r_i)
            ret_im.pop(min_r_i)

        return ret_im





    def detect_block(self, segmented_block):

        result_gray = cv2.cvtColor(segmented_block, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((5,5),np.float32)/25
        result_gray = cv2.filter2D(result_gray,-1,kernel)
        # Defining all the parameters
        t_lower = 100 # Lower Threshold
        t_upper = 230 # Upper threshold
        aperture_size = 5 # Aperture size
        L2Gradient = False # Boolean

        # Applying the Canny Edge filter
        # with Aperture Size and L2Gradient
        edge = cv2.Canny(result_gray, t_lower, t_upper,
                         apertureSize = aperture_size,
                         L2gradient = L2Gradient )




        lines = cv2.HoughLinesP(
                    edge, # Input edge image
                    1, # Distance resolution in pixels
                    np.pi/180, # Angle resolution in radians
                    threshold=18, # Min number of votes for valid line
                    minLineLength=5, # Min allowed length of line
                    maxLineGap=10 # Max allowed gap between line for joining them
                    )


        result = segmented_block.copy()

        # Iterate over points
        for points in lines:
              # Extracted points nested in the list
            x1,y1,x2,y2=points[0]
            # Draw the lines joing the points
            # On the original image
            cv2.line(result,(x1,y1),(x2,y2),(0,255,0),2)


        circles = cv2.HoughCircles(result_gray,
            cv2.HOUGH_GRADIENT, 1, 20,
            param1=100, param2=10, minRadius=0, maxRadius=10)

        # ensure at least some circles were found
        if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                print("Ciercles found")
                circles = np.round(circles[0, :]).astype("int")

                # loop over the (x, y) coordinates and radius of the circles
                for (x, y, r) in circles:
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        cv2.circle(result, (x, y), r, (0, 255, 0), 4)
                        cv2.rectangle(result, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)



        #cv2.imshow('result', result)
        #cv2.waitKey(1)



        return result




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

    detector = BlockDetector("/home/tamas/data/pegtransfer/highres_yellow_1.bag")

    detector.start_and_process_stream()

    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    print("Shutting down")
    cv2.destroyAllWindows()
