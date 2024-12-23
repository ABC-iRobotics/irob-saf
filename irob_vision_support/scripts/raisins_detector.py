#!/usr/bin/env python

import rospy
import time
import cv2
import pyrealsense2 as rs
import numpy as np
import random as rng
import math

from geometry_msgs.msg import Point, Pose
from irob_msgs.msg import GraspObject, Environment
import open3d as o3d
from scipy.spatial.transform import Rotation as R


# Point distance from plane
def point_distance_from_plane(x1, y1, z1, a, b, c, d):
    """ https://mathinsight.org/distance_point_plane
    """
    return abs((a*x1) + (b*y1) + (c*z1) + d) / math.sqrt((a*a) + (b*b) + (c*c))

# Correct depth based on plane detection
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

# Rotation matrix that aligns two vectors
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


class MyDetector:
    def __init__(self) -> None:
       
        self.contourThreshold = 50
        self.color = (66, 236, 245)
    
        self.offline = bool(rospy.get_param('offline', True))
        self.width = 640
        self.height = 480
        self.fps = 30 #30
        self.clipping_distance_in_meters = 0.50
        self.exposure = 200.0 #400.0 #600.0 #600.0
        self.z_offset = 0.004   # m
        self.approach_offset = 0.05
        self.pcd = o3d.geometry.PointCloud()
        self.bagfile = str(rospy.get_param('bag_file', '/home/bence/Desktop/20231201_133036.bag'))

        rospy.init_node('raisins_detector', anonymous=True)
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
            
        #self.tf_phantom = Transform() # skip

    def set_exposure(self, exposure):
        rgb_cam_sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        rgb_cam_sensor.set_option(rs.option.exposure, exposure)

    def process_stream(self):
        self.profile = self.pipeline.start(self.config)
        if not self.offline:
            self.set_exposure(self.exposure)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)
        align_to = rs.stream.color
        align = rs.align(align_to)

        try:
            while not rospy.is_shutdown():
                frames = self.pipeline.wait_for_frames()
            
                aligned_frames = align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not aligned_depth_frame or not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

     

                plane_model, mask = self.calculate_plane_and_mask(color_frame=color_frame,
                                                                color_image=color_image,
                                                                aligned_depth_frame=aligned_depth_frame)
                
                if plane_model is None:
                    continue

                raisins = self.get_raisins_coordinates_v2(color_image)

                img = self.draw_red_points(coordinates=raisins, image=color_image)
                cv2.namedWindow('Color image', cv2.WINDOW_NORMAL)
                cv2.imshow('Color image', img)
                cv2.waitKey(1)

                [a, b, c, d] = plane_model
                print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                plane_normal = np.array([-a, -b, -c])

                start_ori_vec = np.array([0.0, 0.0, 1.0])
                grasp_ori = R.from_matrix(rotation_matrix_from_vectors(start_ori_vec, plane_normal))
                grasp_ori_quat = grasp_ori.as_quat()

                env_msg = Environment()
                env_msg.valid = Environment.VALID
                env_msg.header.stamp = rospy.Time.now()
                env_msg.header.frame_id = "camera"

                for i, raisin in enumerate(raisins):
                    grasp_msg = GraspObject()
                    p = self.get_pixel_position(raisin, aligned_depth_frame)
                    
                    if p is None:
                        continue

                    p_a = force_depth(p, plane_model, self.approach_offset)
                    #p_b = force_depth(p, plane_model, self.block_offset)
                    grasp_pose = Pose()
                    approach_pose = Pose()
                    grasp_pose.position.x = p[0]*1000.0
                    grasp_pose.position.y = p[1]*1000.0
                    grasp_pose.position.z = p[2]*1000.0 #negative conversion due to correct coordinate
                    grasp_pose.orientation.x = grasp_ori_quat[0]
                    grasp_pose.orientation.y = grasp_ori_quat[1]
                    grasp_pose.orientation.z = grasp_ori_quat[2]
                    grasp_pose.orientation.w = grasp_ori_quat[3]

                    approach_pose.position.x = p_a[0]*1000.0
                    approach_pose.position.y = p_a[1]*1000.0
                    approach_pose.position.z = p_a[2]*1000.0 #negative conversion due to correct coordinate
                    #print("approach position: ", approach_pose.position)
                    approach_pose.orientation.x = grasp_ori_quat[0]
                    approach_pose.orientation.y = grasp_ori_quat[1]
                    approach_pose.orientation.z = grasp_ori_quat[2]
                    approach_pose.orientation.w = grasp_ori_quat[3]

                    grasp_msg.grasp_poses.append(grasp_pose)
                    grasp_msg.approach_poses.append(approach_pose)

                    grasp_msg.position.x = p[0]*1000.0
                    grasp_msg.position.y = p[1]*1000.0
                    grasp_msg.position.z = p[2]*1000.0 #negative conversion due to correct coordinate

                    grasp_msg.id = i
                    grasp_msg.name = "block#"+str(i)
                    env_msg.objects.append(grasp_msg)
                    
                self.blocks_pub.publish(env_msg)

        except Exception as e:
            rospy.logerr(f"RealSense error: {e}")

        finally:
            self.pipeline.stop()

    def get_raisins_coordinates(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, dst = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        coords = []
        for i in range(len(contours)):
            contour = contours[i]
            if len(contour) > self.contourThreshold:
                contour = contour.reshape(contour.shape[0], contour.shape[2])
                x_coordinates, y_coordinates = zip(*contour)
                center_x = sum(x_coordinates) / len(contour)
                center_y = sum(y_coordinates) / len(contour)
                coords.append([int(center_x), int(center_y)])

        return coords
    

    def get_raisins_coordinates_v2(self, image, mask=None):
        # preprocessing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 13, 2)

        # using mask
        if mask is not None:
            thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

        # search contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        coords = []
        for contour in contours:
            # Contour filtering by size
            area = cv2.contourArea(contour)
            if area < 30 or area > 1000:  # 
                continue

            # Checking convexity and circularity
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.7:  # 
                continue

            # Calculating the center point using moments
            M = cv2.moments(contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                coords.append([center_x, center_y])

        return coords


    def draw_red_points(self, coordinates, image): 
        for coordinate in coordinates:
            x = coordinate[0]
            y = coordinate[1]
            cv2.circle(image, (int(x), int(y)), 3, (0,0,255), -1)

        return image
    

    def calculate_plane_and_mask(self, color_frame, color_image, aligned_depth_frame):
        intrinsics = (aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics())
        pc = rs.pointcloud()
        points = rs.points()
        pc.map_to(color_frame)
        points = pc.calculate(aligned_depth_frame)
        vtx = np.asanyarray(points.get_vertices(2))
        self.pcd.points = o3d.utility.Vector3dVector(vtx)
        plane_model, inliers = self.pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=3,
                                                 num_iterations=100)
    
        plane_cloud = self.pcd.select_by_index(inliers)
        points = np.asarray(plane_cloud.points)
        
        # create mask
        mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
        # Projecting points onto the image plane
        for point in points:
            pixel = rs.rs2_project_point_to_pixel(intrinsics, point)
            if math.isnan(pixel[0]):
                return None, None
            
            x, y = int(pixel[0]), int(pixel[1])
            if 0 <= x < color_image.shape[1] and 0 <= y < color_image.shape[0]:
                mask[y, x] = 255
        
        # Mask refinement with morphological operations
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # fill gaps
        mask_fill = mask.copy()

        h, w = mask.shape[:2]
        fill_mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(mask_fill, fill_mask, (0,0), 255)
        mask_filled = mask | cv2.bitwise_not(mask_fill)

        return plane_model, mask_filled

    def get_pixel_position(self, img_coords, aligned_depth_frame):
        intrinsics = (aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics())
        x = int(round(img_coords[0]))
        y = int(round(img_coords[1]))
        pos = np.array([0.0, 0.0, 0.0])

        d = aligned_depth_frame.get_distance(x,y)
        if (d != 0 and d < self.clipping_distance_in_meters):
            pos = rs.rs2_deproject_pixel_to_point(intrinsics, [x,y], d)

            return pos
        return None
    
    


if __name__ == '__main__':
    mydetector = MyDetector()
    try:
        mydetector.process_stream()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
      
