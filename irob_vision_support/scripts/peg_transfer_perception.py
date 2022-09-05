import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Transform, TransformStamped
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
from scipy.spatial.transform import Rotation
import yaml


class PegTransferPerception:


    def __init__(self):
        """Constructor."""

        print("Node started")
        rospy.init_node('peg_transfer_perception', anonymous=True)
        self.peg_pub = rospy.Publisher("peg_tf", TransformStamped,
                                                        queue_size=10)

        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.30
        self.exposure = 200.0
        #self.exposure = 150.0 #1800.0 #1000.0
        self.tr_seq = 0

        # Init RealSense
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.width,
                                    self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width,
                                    self.height, rs.format.bgr8, self.fps)

        self.bridge = CvBridge()


        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        #rospy.spin()



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


                intrinsics = (aligned_depth_frame.profile
                            .as_video_stream_profile().get_intrinsics())
                w_h = depth_image.shape


                print("running")
                #cv2.imshow("Output",  output)
                #cv2.waitKey(1)


        finally:
            self.pipeline.stop()




    def set_exposure(self, exposure):
        """Set the camera exposure manually."""

        rgb_cam_sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        rgb_cam_sensor.set_option(rs.option.exposure, exposure)







if __name__ == '__main__':
    perception = PegTransferPerception()
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_104229.bag")
    #detector.load_img("/home/tamas/data/realsense/Realsense_viewer_20210326_103332.bag")
    #for image in detector.cv_images:
    #image = detector.cv_images[2]
        #detector.find_fiducials_locations(image)
    perception.start_and_process_stream()
    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    print("Shutting down")
    cv2.destroyAllWindows()
