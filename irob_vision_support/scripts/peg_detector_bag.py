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
import pyrealsense2 as rs

import open3d as o3d
import copy


class PegDetector:
    def __init__(self, bagfile, model):
        rospy.init_node('peg_detector', anonymous=True)
        print("Init")

        self.bagfile = bagfile
        self.model = model

        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.30

        # Init RealSense
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_device_from_file(self.bagfile)



        # Open3d
        self.source = o3d.io.read_point_cloud(model)

        self.threshold = 0.02
        self.trans_init = np.asarray([  [1.0, 0.0, 0.0, 0.05],
                                        [0.0, 1.0, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.27],
                                        [0.0, 0.0, 0.0, 1.0]])
        #self.source.transform(self.trans_init)

        #target = o3d.io.read_point_cloud("../../test_data/ICP/cloud_bin_1.pcd")
        #threshold = 0.02
        #trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
         #                        [-0.139, 0.967, -0.215, 0.7],
         #                        [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
        #draw_registration_result(source, target, trans_init)




    # Realsense
    def start_and_process_stream(self):

        # Declare pointcloud object, for calculating pointclouds and texture mappings
        pc = rs.pointcloud()
        # We want the points object to be persistent so we can display the last cloud when a frame drops
        points = rs.points()

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
        colorizer = rs.colorizer()

        #vis = o3d.visualization.Visualizer()
        #vis.create_window("Tests", width = 600, height = 600, left = 200, top = 200)
        pcd = o3d.geometry.PointCloud()
        #vis.set_full_screen(False)
        inited = False

        # Streaming loop
        try:
            while not rospy.is_shutdown():
                # Get frameset of color and depth
                for i in range(100):
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

                pc.map_to(color_frame)
                points = pc.calculate(aligned_depth_frame)

                vtx = np.asanyarray(points.get_vertices(2))
                print(np.shape(vtx))
                pcd.points = o3d.utility.Vector3dVector(vtx)

                # Plane segmentation

                plane_model, inliers = pcd.segment_plane(distance_threshold=0.005,
                                                         ransac_n=3,
                                                         num_iterations=1000)
                [a, b, c, d] = plane_model
                print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

                inlier_cloud = pcd.select_by_index(inliers)
                inlier_cloud.paint_uniform_color([1.0, 0, 0])
                outlier_cloud = pcd.select_by_index(inliers, invert=True)

                plane_model_board, inliers_board = outlier_cloud.segment_plane(distance_threshold=0.005,
                                                         ransac_n=3,
                                                         num_iterations=1000)


                inlier_cloud_board = outlier_cloud.select_by_index(inliers_board)
                inlier_cloud_board.paint_uniform_color([0, 1.0, 0])
                outlier_cloud = outlier_cloud.select_by_index(inliers_board, invert=True)


                # CLustering

                #with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                #    labels = np.array(outlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

                #max_label = labels.max()
                #print(f"point cloud has {max_label + 1} clusters")
                #colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
                #colors[labels < 0] = 0
                #outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
                o3d.visualization.draw_geometries([inlier_cloud_board, outlier_cloud, inlier_cloud], zoom=0.8,
                                                  front=[0.0, -1.0,-1.0],
                                                  lookat=[0.0, 0.1, -0.1],
                                                  up=[0.0, 1.0, 0.0])

                break
                # Visualization


               # if not inited:
                    #vis.add_geometry(inlier_cloud)
                    #vis.add_geometry(outlier_cloud)
                   # inited = True
               # else:
                    #vis.update_geometry(inlier_cloud)
                   # vis.update_geometry(outlier_cloud)
               # if not vis.poll_events():
                 #   break
                #vis.update_renderer()
                #o3d.visualization.draw_geometries([pcd])


        finally:
            self.pipeline.stop()
            #vis.destroy_window()







    def load_bag_open3d(self, bagfile):
        print("Loading bag file...")
        bag_reader = o3d.t.io.RSBagReader()
        bag_reader.open(bagfile)
        im_rgbd = bag_reader.next_frame()
        while not bag_reader.is_eof():
            # process im_rgbd.depth and im_rgbd.color

            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                im_rgbd,
                o3d.camera.PinholeCameraIntrinsic(
                    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
            # Flip it, otherwise the pointcloud will be upside down
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            o3d.visualization.draw_geometries([pcd], zoom=0.5)

            #
            im_rgbd = bag_reader.next_frame()

        bag_reader.close()
        print("Bag closed")






    def draw_registration_result(source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4459,
                                          front=[0.9288, -0.2951, -0.2242],
                                          lookat=[1.6784, 2.0612, 1.4451],
                                          up=[-0.3402, -0.9189, -0.1996])


if __name__ == '__main__':
    print("Node started")
    detector = PegDetector("/home/tamas/data/pegtransfer/pegboard1.bag",
                            "/home/tamas/data/3d_models/block_small.pcd")

    detector.start_and_process_stream()

    #rospy.spin()
    #cv2.destroyAllWindows()