import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
import message_filters
from irob_msgs.msg import GraspObject, Environment
from geometry_msgs.msg import Point, Transform, Pose

from geometry_msgs.msg import Pose2D

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

from skimage import data
from skimage.util import img_as_float
from skimage.feature import (corner_harris, corner_subpix, corner_peaks, plot_matches)
from skimage.transform import warp, AffineTransform, EuclideanTransform
from skimage.exposure import rescale_intensity
from skimage.color import rgb2gray
from skimage.measure import ransac
from scipy.spatial.transform import Rotation as R

import open3d as o3d



#from irob_utils import rigid_transform_3D
#import irob_utils

def dist_line_from_point(x0, y0, xp1, yp1, xp2, yp2):
    return (abs(((xp2-xp1) * (yp1-y0)) - ((xp1-x0) * (yp2-yp1)))
            / math.sqrt(((xp2-xp1)**2) + ((yp2-yp1)**2)))


def angle_between_lines(xp1, yp1, xp2, yp2, xp3, yp3, xp4, yp4):
    if xp2 != xp1:
        m1 = float((yp2-yp1)) / float((xp2-xp1))
    else:
        m1 = (yp2-yp1) * 2.0

    if xp4 != xp3:
        m2 = float((yp4-yp3)) / float((xp4-xp3))
    else:
        m2 = (yp4-yp3) * 2.0

    try:
        ret = math.degrees(math.atan((m1-m2) / (1 + (m1 * m2))))
    except ZeroDivisionError:
        ret = math.degrees(math.atan((m1-m2) / (0.0000001)))
    return ret

def intersection_of_lines(xp1, yp1, xp2, yp2, xp3, yp3, xp4, yp4):

    t = (((xp1-xp3)*(yp3-yp4)) - ((yp1-yp3)*(xp3-xp4))) / (((xp1-xp2)*(yp3-yp4)) - ((yp1-yp2)*(xp3-xp4)))

    #u = (((xp1-xp3)*(yp1-yp2)) - ((yp1-yp3)*(xp1-xp2)))
    #    / (((xp1-xp2)*(yp3-yp4)) - ((yp1-yp2)*(xp3-xp4)))

    Px = xp1 + (t*(xp2-xp1))
    Py = yp1 + (t*(yp2-yp1))
    return Px, Py

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def point_distance_from_plane(x1, y1, z1, a, b, c, d):
    """ https://mathinsight.org/distance_point_plane
    """
    return abs((a*x1) + (b*y1) + (c*z1) + d) / math.sqrt((a*a) + (b*b) + (c*c))


def force_depth(point, plane, block_offset):
    [a,b,c,d] = plane
    dist = point_distance_from_plane(point[0], point[1], point[2],
                                     a, b, c, d)
    #print(dist)
    norm_ori = np.array([a,b,c])
    norm_unit = norm_ori / np.linalg.norm(norm_ori)
    corr_dist = block_offset - dist
    corr_point = point - (corr_dist * norm_unit)
    return corr_point




class BlockDetector:


    def __init__(self, offline = False, bagfile = ""):

        print("Init")



        self.lower_red = (0, 0, 150)
        self.upper_red = (80, 255, 255)
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

        self.offline = offline
        self.bagfile = bagfile

        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.50
        #self.exposure = 1500.0
        self.exposure = 600.0 #400.0 #600.0 #600.0

        self.z_offset = 0.004   # m

        self.plane_detect_frames_N = 100
        self.cv_frames_N = 10
        self.detect_plane = True
        self.board_offset = 0.014
        self.board_bb_offset = 0.005
        self.block_offset = 0.0285
        self.approach_offset = 0.05
        self.grasp_diameter = 0.0001
        self.plot_scatter = False
        self.grasp_idx = 1

        self.src_triangle_im_coords = np.array([[0.0, 0.866], [1.0, 0.866], [0.5, 0.0]])
        self.src_grasp_im_coords = np.array([[0.36, 0.78],
                                             [0.62, 0.78],
                                             [0.76, 0.58],
                                             [0.65, 0.38],
                                             [0.38, 0.38],
                                             [0.26, 0.58]])



        self.src_board_top_corner_coords = np.array([[0.00, 0.00, -0.005],
                                                     [0.1011, 0.00, -0.005],
                                                     [0.1011, 0.0628, -0.005],
                                                     [0.00, 0.0628, -0.005]])


        self.tf_phantom = Transform()

        #image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        #depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        #info_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo)

        #ts = message_filters.TimeSynchronizer([image_sub, depth_sub, info_sub], 10)
        #ts.registerCallback(self.cb_images)

        # ROS
        rospy.init_node('block_detector', anonymous=True)
        self.blocks_pub = rospy.Publisher("blocks_grasp", Environment, queue_size=10)


        # Init RealSense
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if (self.offline):
            self.config.enable_device_from_file(self.bagfile)
        else:
            self.config.enable_stream(rs.stream.depth, self.width,
                                        self.height, rs.format.z16, self.fps)
            self.config.enable_stream(rs.stream.color, self.width,
                                        self.height, rs.format.bgr8, self.fps)



        #rospy.spin()



    # realsense
    def start_and_process_stream(self):

        # Declare pointcloud object, for calculating pointclouds and texture mappings
        pc = rs.pointcloud()
        # We want the points object to be persistent so we can display the last cloud when a frame drops
        points = rs.points()

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        if not self.offline:
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
        colorizer = rs.colorizer()

        pcd = o3d.geometry.PointCloud()
        #vis.set_full_screen(False)
        inited = False

        plane_detect_frames_cnt = self.plane_detect_frames_N
        cv_frames_cnt = self.cv_frames_N
        plane_model = []
        seq = 0
        plane_normal = np.array([0.0, 0.0, 0.0])


        if self.plot_scatter:
            plt.ion()
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(projection='3d')


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
                if self.offline:
                    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                #cv2.namedWindow('Color image', cv2.WINDOW_NORMAL)
                #cv2.imshow('Color image', color_image)
                #cv2.waitKey(1)

                if not cv_frames_cnt >= self.cv_frames_N:
                    cv_frames_cnt += 1
                    plane_detect_frames_cnt = plane_detect_frames_cnt + 1
                    continue
                cv_frames_cnt = 1
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


                intrinsics = (aligned_depth_frame.profile
                            .as_video_stream_profile().get_intrinsics())
                w_h = depth_image.shape
                result = color_image.copy()


                # Point cloud
                if self.detect_plane and plane_detect_frames_cnt >= self.plane_detect_frames_N:
                    plane_detect_frames_cnt = 1
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
                    plane_normal = np.array([-a, -b, -c])

                    inlier_cloud = pcd.select_by_index(inliers)
                    inlier_cloud.paint_uniform_color([1.0, 0, 0])
                    outlier_cloud = pcd.select_by_index(inliers, invert=True)




                    plane_model_board, inliers_board = outlier_cloud.segment_plane(distance_threshold=0.005,
                                                         ransac_n=3,
                                                         num_iterations=1000)


                    inlier_cloud_board = outlier_cloud.select_by_index(inliers_board)
                    inlier_cloud_board.paint_uniform_color([1.0, 0.0, 0])
                    outlier_cloud = outlier_cloud.select_by_index(inliers_board, invert=True)

                    # Find board
                    #R_bb = rotation_matrix_from_vectors([0.0, 0.0, 1.0], [a,b,c])
                    #center_bb = np.array([0.0, 0.0, -(d + self.board_bb_offset)])
                    #extent_bb = np.array([1.0, 1.0, 0.005])

                    #bb = o3d.geometry.OrientedBoundingBox(center_bb, R_bb, extent_bb)
                    #pcd_board = pcd.crop(bb)
                    #pcd_board.paint_uniform_color([0, 1.0, 0])

                    #bb_small = o3d.geometry.OrientedBoundingBox.create_from_points(pcd_board.points)
                    #print(np.asarray(bb_small.get_box_points()))
                    #bb_points = np.asarray(bb_small.get_box_points())

                    # Could be the lower 4?
                    #dst_board_top_corner_coords = np.array([force_depth(bb_points[0], plane_model, self.board_offset),
                     #                                       force_depth(bb_points[1], plane_model, self.board_offset),
                     #                                       force_depth(bb_points[7], plane_model, self.board_offset),
                     #                                       force_depth(bb_points[2], plane_model, self.board_offset)])


                    #print(dst_board_top_corner_coords)
                else:
                    plane_detect_frames_cnt = plane_detect_frames_cnt + 1


                # Board detection
                segmented_board, mask = self.segment_board(color_image, "red", 6)
                #cv2.imshow("Board seg",  segmented_board)
                #cv2.waitKey(1)
                result, dst = self.detect_board(segmented_board, mask, result)


                board_dst_coords = []
                if dst is not None:
                    for i in range(dst.shape[0]):
                        p, result = self.get_pixel_position_search_depth(dst[i,:],
                                               aligned_depth_frame, w_h, intrinsics, result, 0)
                        p_b = force_depth(p, plane_model, self.board_offset)
                        board_dst_coords.append(p_b)
                    #print(board_dst_coords)
                    #cv2.imshow("Board",  result_board)
                    #cv2.waitKey(1)

                    #
                    dst_board_top_corner_coords = np.array(board_dst_coords)
                    board_T = EuclideanTransform()
                    board_T.estimate(self.src_board_top_corner_coords, dst_board_top_corner_coords)
                    board_residuals = board_T.residuals(self.src_board_top_corner_coords, dst_board_top_corner_coords)
                    print("Residuals:", board_residuals)

                    val_board_corners = board_T(self.src_board_top_corner_coords)
                    #print(val_board_corners)



                    # Draw plot
                    if self.plot_scatter:
                        self.ax.scatter(val_board_corners.T[0,:], val_board_corners.T[1,:],
                                                    val_board_corners.T[2,:], marker='o')
                        self.ax.scatter(dst_board_top_corner_coords.T[0,:], dst_board_top_corner_coords.T[1,:],
                                                   dst_board_top_corner_coords.T[2,:], marker='^')


                    if np.mean(board_residuals) < 0.005:
                        board_R_mat = R.from_matrix(board_T.params[0:3,0:3])
                        #board_R_helper = R.from_euler('x', 90, degrees=True)
                        #board_R_mat = board_R_helper * board_R_mat

                        #start_ori_vec = np.array([0.0, 0.0, -1.0])
                        #board_R_mat = R.from_matrix(rotation_matrix_from_vectors(start_ori_vec, plane_normal))
                        board_R_quat = board_R_mat.as_quat()
                        self.tf_phantom.rotation.x = board_R_quat[0]
                        self.tf_phantom.rotation.y = board_R_quat[1]
                        self.tf_phantom.rotation.z = board_R_quat[2]
                        self.tf_phantom.rotation.w = board_R_quat[3]
                        self.tf_phantom.translation.x = board_T.params[0,3]*1000.0
                        self.tf_phantom.translation.y = board_T.params[1,3]*1000.0
                        self.tf_phantom.translation.z = board_T.params[2,3]*1000.0
                        print(self.tf_phantom)




                    #o3d.visualization.draw_geometries([pcd,inlier_cloud, bb, pcd_board, bb_small])



                # Block detection
                segmented_blocks = self.segment_blocks(color_image, "yellow", 6)

                detected_blocks= []
                env_msg = Environment()
                env_msg.valid = Environment.VALID
                env_msg.header.stamp = rospy.Time.now()
                env_msg.header.seq = seq
                env_msg.header.frame_id = "camera"
                env_msg.tf_phantom = self.tf_phantom
                #result = color_image.copy()
                for i in range(len(segmented_blocks)):
                    result, grasp_im_coords, block_rotation = self.detect_block(segmented_blocks[i], result)
                    detected_blocks.append(result)
                    # Grasp orientation
                    start_ori_vec = np.array([0.0, 0.0, -1.0])
                    grasp_ori = R.from_matrix(rotation_matrix_from_vectors(start_ori_vec, plane_normal))
                    block_rot_in_img = R.from_euler('z', block_rotation, degrees=False)
                    grasp_ori = block_rot_in_img * grasp_ori
                    grasp_ori_quat = grasp_ori.as_quat()
                    #cv2.imshow("Output1",  segmented_blocks[i])
                    #cv2.waitKey(1)


                    j = 0
                    grasp_coords = []
                    approach_coords = []
                    block_pos = np.array([0.0, 0.0, 0.0])
                    for p_im in grasp_im_coords:
                        p, result = self.get_pixel_position_search_depth(p_im, aligned_depth_frame, w_h, intrinsics, result, j)
                        p_b = force_depth(p, plane_model, self.block_offset)
                        grasp_coords.append(p_b)

                        p_a = force_depth(p, plane_model, self.approach_offset)
                        approach_coords.append(p_a)
                        if self.plot_scatter:
                            self.ax.scatter(p_b[0], p_b[1], p_b[2], marker='o')

                        block_pos = block_pos + p_b
                        j = j+1

                    if (len(grasp_coords) > 0):
                        block_msg = GraspObject()
                        for k in range(6):
                            grasp_pose = Pose()
                            approach_pose = Pose()
                            grasp_pose.position.x = grasp_coords[k][0]*1000.0
                            grasp_pose.position.y = grasp_coords[k][1]*1000.0
                            grasp_pose.position.z = grasp_coords[k][2]*1000.0
                            grasp_pose.orientation.x = grasp_ori_quat[0]
                            grasp_pose.orientation.y = grasp_ori_quat[1]
                            grasp_pose.orientation.z = grasp_ori_quat[2]
                            grasp_pose.orientation.w = grasp_ori_quat[3]

                            approach_pose.position.x = approach_coords[k][0]*1000.0
                            approach_pose.position.y = approach_coords[k][1]*1000.0
                            approach_pose.position.z = approach_coords[k][2]*1000.0
                            approach_pose.orientation.x = grasp_ori_quat[0]
                            approach_pose.orientation.y = grasp_ori_quat[1]
                            approach_pose.orientation.z = grasp_ori_quat[2]
                            approach_pose.orientation.w = grasp_ori_quat[3]

                            block_msg.grasp_poses.append(grasp_pose)
                            block_msg.approach_poses.append(approach_pose)

                        block_pos = block_pos / j
                        block_msg.position.x = block_pos[0]*1000.0
                        block_msg.position.y = block_pos[1]*1000.0
                        block_msg.position.z = block_pos[2]*1000.0

                        block_msg.grasp_diameter = self.grasp_diameter

                        block_msg.id = i
                        block_msg.name = "block#"+str(i)
                        env_msg.objects.append(block_msg)




                # Send grasp positions
                self.blocks_pub.publish(env_msg)

                #detected_blocks_stacked = cv2.hconcat(detected_blocks)


                if self.plot_scatter:
                    self.ax.set_xlabel('X')
                    self.ax.set_ylabel('Y')
                    self.ax.set_zlabel('Z')
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()


                #cv2.namedWindow('Segmented', cv2.WINDOW_NORMAL)
                #cv2.imshow('Segmented', detected_blocks[1])
                #cv2.waitKey(1)
                seq = seq + 1


                cv2.imshow("Output",  result)
                cv2.waitKey(1)


        finally:
            self.pipeline.stop()

    #
    def get_pixel_position(self, img_coords, aligned_depth_frame, w_h, intrinsics, output):
        x = int(round(img_coords[0]))
        y = int(round(img_coords[1]))

        pos = np.array([0.0, 0.0, 0.0])

        d = aligned_depth_frame.get_distance(x,y)
                    #print("x: "  + str(x) + ", y: " + str(y) + ", d: "  + str(d))
        if (d != 0 and d < self.clipping_distance_in_meters):
            pos = rs.rs2_deproject_pixel_to_point(intrinsics, [x,y], d)
            #text = "(" + str(int(round(pos[0]*1000.0))) + "," + str(int(round(pos[1]*1000.0))) + "," + str(int(round(pos[2]*1000.0))) + ")"
            #output = cv2.putText(output, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX,
            #               0.5, (255,255,255), 1, cv2.LINE_AA)

            return pos, output
        return None, output



    #
    def get_pixel_position_search_depth(self, img_coords, aligned_depth_frame, w_h, intrinsics, output, text_i = 0):
        x = int(round(img_coords[0]))
        y = int(round(img_coords[1]))

        pos = np.array([0.0, 0.0, 0.0])

        d = aligned_depth_frame.get_distance(x,y)
        i = 1
        while (d == 0 or d >= self.clipping_distance_in_meters):
            for j in range(-i,i+1):
                for k in range(-i,i+1):
                    x_d = x+j
                    y_d = y+k
                    if (x_d >= 0 and x_d < (w_h[1]-1) and y_d >=0 and y_d < (w_h[0]-1)):
                        d = aligned_depth_frame.get_distance(x_d,y_d)
                        #print(d)
                        if (d != 0 and d < self.clipping_distance_in_meters):
                            #print("breaking")
                            break
            i = i + 1

        #print(d)
        if text_i == 0:
            title = "Block #1 grasp positions"
            #output = cv2.putText(output, title, (10,20), cv2.FONT_HERSHEY_SIMPLEX,
            #                   0.5, (255,255,255), 1, cv2.LINE_AA)
        pos = rs.rs2_deproject_pixel_to_point(intrinsics, [x,y], d)
        text = "(" + str(int(round(pos[0]*1000.0))) + "," + str(int(round(pos[1]*1000.0))) + "," + str(int(round(pos[2]*1000.0))) + ")"
        #output = cv2.putText(output, text, (10,20 + ((text_i+1) * 20)), cv2.FONT_HERSHEY_SIMPLEX,
        #                   0.5, (255,255,255), 1, cv2.LINE_AA)
        return pos, output


    #
    def calc_pose(self, fid_position, output):

        rigid_transform_3D([1,1,1], [0,0,0])


    #
    def set_exposure(self, exposure):
        rgb_cam_sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        rgb_cam_sensor.set_option(rs.option.exposure, exposure)



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

        #mask_background = cv2.inRange(hsv_img, self.lower_background, self.upper_background)
        #mask_background = 255 - mask_background
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
            for i in range(len(ret_r)):
                if ret_r[i] < min_r:
                    min_r = ret_r[i]
                    min_r_i = i
            ret_r.pop(min_r_i)
            ret_im.pop(min_r_i)

        return ret_im


    def segment_board(self, image, col, n):


        if col == 'red':
            hsv_lower = self.lower_red
            hsv_upper = self.upper_red
        else:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)

        #if color == 'red' or color == 'purple':
        #    mask_2 = cv2.inRange(hsv_img, hsv_lower_2, hsv_upper_2)
        #    mask = cv2.bitwise_or(mask, mask_2)

        #mask_background = cv2.inRange(hsv_img, self.lower_background, self.upper_background)
        #mask_background = 255 - mask_background
        #mask = cv2.bitwise_and(mask, mask_background)

        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)




        blobs = cv2.connectedComponentsWithStats(
                    mask, 4, cv2.CV_32S)
        (numLabels, labels, stats, centroids) = blobs


        max_area = 0
        max_area_i = 0
        for i in range(1, numLabels):
            area = stats[i, cv2.CC_STAT_AREA]
            #print(area)
            if area > max_area:
                max_area = area
                max_area_i = i

        #print(max_area_i)
        mask_ret = np.where(labels == max_area_i, np.uint8(1), np.uint8(0))
        kernel = np.ones((3,3),np.uint8)

        #mask_ret = cv2.dilate(mask_ret, kernel, iterations=1)
        #ret_im = image.copy()
        ret_im = np.zeros(image.shape,dtype=np.uint8)
        ret_im.fill(255)
        ret_im = cv2.bitwise_and(ret_im, ret_im, mask = mask_ret)
        return ret_im, mask_ret


    def detect_board(self, segmented_board, mask, result):

        result_gray = cv2.cvtColor(segmented_board, cv2.COLOR_BGR2GRAY)
        result_gray = np.zeros(result_gray.shape,dtype=np.uint8)
        kernel = np.ones((3,3),np.float32)/25
        result_gray = cv2.filter2D(result_gray,-1,kernel)
        grasp_im_coords_tfromed = []
        # Defining all the parameters
        t_lower = 2 # Lower Threshold
        t_upper = 255 # Upper threshold
        aperture_size = 7 # Aperture size
        L2Gradient = False # Boolean

        # Applying the Canny Edge filter
        # with Aperture Size and L2Gradient
        #edge = cv2.Canny(result_gray, t_lower, t_upper,
        #                 apertureSize = aperture_size,
        #                 L2gradient = L2Gradient )

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #cv2.imshow("Contours",  im2)
        #cv2.waitKey(0)
        cv2.drawContours(result_gray, contours, -1, (255), 1)
        #cv2.imshow("Contours",  result)
        #cv2.waitKey(1)


        lines = cv2.HoughLinesP(
                    result_gray, # Input edge image
                    1, # Distance resolution in pixels
                    np.pi/360, # Angle resolution in radians
                    threshold=50, # Min number of votes for valid line
                    minLineLength=50, # Min allowed length of line
                    maxLineGap=500 # Max allowed gap between line for joining them
                    )


        #result = segmented_block.copy()






        # ensure at least some circles were found
        cornerX = []
        cornerY = []
        if lines is not None:
            print(lines.shape[0])
            # Iterate over points
            #for points in lines:
                  # Extracted points nested in the list
             #   x1,y1,x2,y2=points[0]
                # Draw the lines joing the points
                # On the original image
             #   cv2.line(result,(x1,y1),(x2,y2),(255,0,0),1)
        if lines is not None and lines.shape[0] == 4:
            for i in range(lines.shape[0] - 1):
                for j in range(i + 1,lines.shape[0] ):
                    xp1,yp1,xp2,yp2=lines[i][0]
                    xp3,yp3,xp4,yp4=lines[j][0]
                    angle = abs(angle_between_lines(xp1, yp1, xp2, yp2, xp3, yp3, xp4, yp4))
                    #print(angle)
                    if angle > 70 and angle < 100:
                        Ax,Ay = intersection_of_lines(xp1, yp1, xp2, yp2, xp3, yp3, xp4, yp4)
                        print(Ax, Ay)
                        cornerX.append(int(round(Ax)))
                        cornerY.append(int(round(Ay)))


        #cv2.imshow("Lines",  result)
        #cv2.waitKey(1)

        pre = np.array(([cornerX, cornerY])).T
        dst = np.empty_like(pre)
        print(pre)

        left_i_1 = 0
        left_i_2 = 1
        min_avg_x = 10000
        if pre.shape[0] != 4:
            print("Board not found!")
            return result, None
        for i in range(3):
            for j in range(1,4):
                avg_x = (pre[i,0] + pre[j,0]) / 2
                if avg_x < min_avg_x and i != j:
                    left_i_1 = i
                    left_i_2 = j
                    min_avg_x = avg_x
        idxs = [0,1,2,3]
        idxs.remove(left_i_1)
        idxs.remove(left_i_2)

        if pre[left_i_1,1] > pre[left_i_2,1]:
            dst[0,:] = pre[left_i_1,:]
            dst[3,:] = pre[left_i_2,:]
        else:
            dst[3,:] = pre[left_i_1,:]
            dst[0,:] = pre[left_i_2,:]

        if pre[idxs[0],1] > pre[idxs[1],1]:
            dst[1,:] = pre[idxs[0],:]
            dst[2,:] = pre[idxs[1],:]
        else:
            dst[2,:] = pre[idxs[0],:]
            dst[1,:] = pre[idxs[1],:]

        for i in range(4):
            cv2.line(result,(dst[i,0],dst[i,1]),(dst[(i+1)%4,0],dst[(i+1)%4,1]),(0,255,0),2)





        return result, dst


    def detect_block(self, segmented_block, result):

        result_gray = cv2.cvtColor(segmented_block, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((3,3),np.float32)/25
        result_gray = cv2.filter2D(result_gray,-1,kernel)
        grasp_im_coords_tfromed = []
        # Defining all the parameters
        t_lower = 100 # Lower Threshold
        t_upper = 230 # Upper threshold
        aperture_size = 7 # Aperture size
        L2Gradient = False # Boolean

        # Applying the Canny Edge filter
        # with Aperture Size and L2Gradient
        edge = cv2.Canny(result_gray, t_lower, t_upper,
                         apertureSize = aperture_size,
                         L2gradient = L2Gradient )
        #cv2.imshow("Canny",  edge)
        #cv2.waitKey(0)

        lines = cv2.HoughLinesP(
                    edge, # Input edge image
                    1, # Distance resolution in pixels
                    np.pi/180, # Angle resolution in radians
                    threshold=16, # Min number of votes for valid line
                    minLineLength=5, # Min allowed length of line
                    maxLineGap=10 # Max allowed gap between line for joining them
                    )


        #result = segmented_block.copy()

        # Iterate over points
        #for points in lines:
              # Extracted points nested in the list
            #x1,y1,x2,y2=points[0]
            # Draw the lines joing the points
            # On the original image
            #cv2.line(result,(x1,y1),(x2,y2),(0,255,0),2)


        circles = cv2.HoughCircles(result_gray,
            cv2.HOUGH_GRADIENT, 1, 20,
            param1=100, param2=10, minRadius=0, maxRadius=10)

        # ensure at least some circles were found
        triangles = []
        if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                #print("Ciercles found")
                circles = np.round(circles[0, :]).astype("int")

                # loop over the (x, y) coordinates and radius of the circles

                for (x, y, r) in circles:
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        #cv2.circle(result, (x, y), r, (0, 255, 0), 4)
                        #cv2.rectangle(result, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                        #print("Dist")
                        distances = []
                        if lines is not None:
                            for i in range(len(lines)):
                                      # Extracted points nested in the list
                                    x1,y1,x2,y2=lines[i][0]
                                    distances.append((i, dist_line_from_point(x, y, x1, y1, x2, y2)))

                        #print(distances)

                        distances.sort(key=lambda distance: distance[1])
                        #print(distances)

                        dist_threshold = 5 #10
                        diff_threshold = 4.0 #5.0
                        angle_threshold = 55.0 #50.0
                        line_idxs = []
                        for j in range(len(distances) - 1):
                            xp1,yp1,xp2,yp2=lines[distances[j][0]][0]
                            xp3,yp3,xp4,yp4=lines[distances[j+1][0]][0]
                            angle = abs(angle_between_lines(xp1, yp1, xp2, yp2, xp3, yp3, xp4, yp4))
                            #print("angle")
                            #print(angle)
                            if (abs(distances[j][1] - distances[j+1][1]) <= diff_threshold
                                            and distances[j][1] >= dist_threshold
                                            and distances[j+1][1] >= dist_threshold
                                            and angle >= angle_threshold):

                                if len(line_idxs) < 2:
                                    line_idxs.append(j)
                                    line_idxs.append(j+1)
                                elif line_idxs[-1] == j:
                                    xp1,yp1,xp2,yp2=lines[distances[line_idxs[0]][0]][0]
                                    angle = abs(angle_between_lines(xp1, yp1, xp2, yp2, xp3, yp3, xp4, yp4))
                                    if (angle >= angle_threshold):
                                        line_idxs.append(j+1)
                                        break

                        if len(line_idxs) == 3:
                            xa1,ya1,xa2,ya2=lines[distances[line_idxs[0]][0]][0]
                            xb1,yb1,xb2,yb2=lines[distances[line_idxs[1]][0]][0]
                            xc1,yc1,xc2,yc2=lines[distances[line_idxs[2]][0]][0]
                            Ax,Ay = intersection_of_lines(xb1, yb1, xb2, yb2, xc1, yc1, xc2, yc2)
                            Bx,By = intersection_of_lines(xa1, ya1, xa2, ya2, xc1, yc1, xc2, yc2)
                            Cx,Cy = intersection_of_lines(xb1, yb1, xb2, yb2, xa1, ya1, xa2, ya2)
                            Ax = int(round(Ax))
                            Ay = int(round(Ay))
                            Bx = int(round(Bx))
                            By = int(round(By))
                            Cx = int(round(Cx))
                            Cy = int(round(Cy))
                            cv2.line(result,(Ax,Ay),(Bx,By),(0,255,0),2)
                            cv2.line(result,(Ax,Ay),(Cx,Cy),(0,255,0),2)
                            cv2.line(result,(Cx,Cy),(Bx,By),(0,255,0),2)
                            triangles.append(np.array([[Ax, Ay], [Bx, By], [Cx, Cy]]))


        # End triangles detection
        # Start fine grasp point estimation using RANSAC


        # find correspondences using simple weighted sum of squared differences
        block_rotation = 0.0
        if (len(triangles)) > 0:
            #print(triangles)


            dst = triangles[0]

            # Arrange points so the first will be with the smallest x value
            while dst[0][0] > dst[1][0] or dst[0][0] > dst[2][0]:
                swap = dst[0].copy()
                dst[0] = dst[1].copy()
                dst[1] = dst[2].copy()
                dst[2] = swap.copy()



            if dst[1][1] < dst[2][1]:
                swap = dst[1].copy()
                dst[1] = dst[2].copy()
                dst[2] = swap.copy()


            # estimate affine transform model using all coordinates
            model = AffineTransform()
            model.estimate(self.src_triangle_im_coords, dst)


            # compare "true" and estimated transform parameters

            if not np.isnan(model.translation[0]):

                #print("Affine transform:")
                #print(f'Scale: ({model.scale[0]:.4f}, {model.scale[1]:.4f}), '
                #    f'Translation: ({model.translation[0]:.4f}, '
                #    f'{model.translation[1]:.4f}), '
                #   f'Rotation: {model.rotation:.4f}')


                tform = AffineTransform(scale=(100, 100), rotation=0.0, translation=(0, 0))
                src_triangle_im_coords_tfromed = model(self.src_triangle_im_coords)
                grasp_im_coords_tfromed = model(self.src_grasp_im_coords)


                #print(src_triangle_im_coords_tfromed)
                for i in range(len(src_triangle_im_coords_tfromed)):
                    cv2.line(result,(int(round(src_triangle_im_coords_tfromed[i][0])),int(round(src_triangle_im_coords_tfromed[i][1]))),
                                     (int(round(src_triangle_im_coords_tfromed[(i+1)%3][0])),int(round(src_triangle_im_coords_tfromed[(i+1)%3][1]))),(0,0,255),2)



                for p in grasp_im_coords_tfromed:
                    cv2.circle(result, (int(round(p[0])), int(round(p[1]))), 3, (0, 255, 0), 2)

                cv2.line(result,(Ax,Ay),(Cx,Cy),(0,255,0),2)
                cv2.line(result,(Cx,Cy),(Bx,By),(0,255,0),2)
                block_rotation = model.rotation


        return result, grasp_im_coords_tfromed, block_rotation




    def find_fiducials_locations(self, image):


        output = image.copy()
        fiducials = {}

        for color in self.fiducial_colors:
            fiducials[color] = self.mask_fiducial(image, color)
            for f in fiducials[color]:
                x = f[0]
                y = f[1]
                r = f[2]
                #cv2.circle(output, (int(x), int(y)), int(r),
                #                                (0, 255, 0), 4)
                #cv2.rectangle(output, (int(x) - 5, int(y) - 5),
                #                                (int(x) + 5, int(y) + 5),
                #                                (0, 128, 255), -1)

        #print(fiducials)
        #cv2.imshow("Output",  output)
        #cv2.waitKey(1)
        return fiducials, output








if __name__ == '__main__':
    print("Node started")
    #help(cv2.aruco)

    detector = BlockDetector(offline = False,
                        bagfile="/home/tamas/data/pegtransfer/highres_yellow_1.bag")

    detector.start_and_process_stream()

    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    print("Shutting down")
    cv2.destroyAllWindows()
