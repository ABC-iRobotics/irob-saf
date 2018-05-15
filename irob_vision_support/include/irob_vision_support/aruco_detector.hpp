/*
 * 	aruco_detector.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-05-15
 *
 *  Example of the image processor node, that can be
 *  used as a member of the generic vision server object.
 *  Able to detect ArUco markers in camera image.
 *
 */

#ifndef ARUCO_DETECTOR_HPP
#define ARUCO_DETECTOR_HPP

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "visualization_msgs/Marker.h"


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/aruco.hpp>

#include "irob_utils/pose.hpp"
#include "irob_utils/utils.hpp"
#include "irob_utils/abstract_directions.hpp"



namespace saf {

class ArucoDetector {

private:

  ros::NodeHandle nh;

  ros::Subscriber result_sub;

public:
  ArucoDetector(ros::NodeHandle);
  ~ArucoDetector();

  void subscribeTopics();

  void tragetCB(const visualization_msgs::MarkerConstPtr&);

  geometry_msgs::Pose processImages( const cv_bridge::CvImagePtr,
                                     const cv_bridge::CvImagePtr,
                                     const cv_bridge::CvImagePtr,
                                     const cv_bridge::CvImagePtr,
                                     const cv_bridge::CvImagePtr);


};

ArucoDetector::ArucoDetector(ros::NodeHandle nh):
  nh(nh)
{
  subscribeTopics();
}

ArucoDetector::~ArucoDetector() {}


void ArucoDetector::subscribeTopics()
{
  result_sub = nh.subscribe<visualization_msgs::Marker>(
        "marker", 1000,
        &ArucoDetector::tragetCB,this);
}

// Callbacks
void ArucoDetector::tragetCB(const visualization_msgs::MarkerConstPtr& msg)
{
  //dummy_location.x() = msg->pose.position.x * 1000.0;
  //dummy_location.y() = msg->pose.position.y * 1000.0;
  //dummy_location.z() = msg->pose.position.z * 1000.0;
}

geometry_msgs::Pose ArucoDetector::processImages(
    const  cv_bridge::CvImagePtr image_left_ptr,
    const cv_bridge::CvImagePtr image_right_ptr,
    const cv_bridge::CvImagePtr color_image_left_ptr,
    const cv_bridge::CvImagePtr color_image_right_ptr,
    const cv_bridge::CvImagePtr disparity_ptr)
{
  ros::spinOnce();

  geometry_msgs::Pose aruco_pose;

  // OpenCV conversion
  cv::Mat inputImage;
  //color_image_left_ptr->image.copyTo(inputImage);
  inputImage = cv::imread("/home/dvrk_nat/Pictures/aruco.png", CV_LOAD_IMAGE_COLOR);

  // Detect Aruco markers
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters;

  ROS_INFO_STREAM("Before params");

  parameters->adaptiveThreshConstant = 7;
  ROS_INFO_STREAM("After 1st param");

  parameters->adaptiveThreshWinSizeMin = 3;

  parameters->adaptiveThreshWinSizeMax = 53;

  parameters->adaptiveThreshWinSizeStep = 4;

  parameters->cornerRefinementMaxIterations = 30;

  parameters->cornerRefinementMinAccuracy = 0.01;

  parameters->cornerRefinementWinSize = 5;

  //parameters->doCornerRefinement = true;

  //parameters->cornerRefinementSubpix = true;

  parameters->errorCorrectionRate = 0.6;

  parameters->minCornerDistanceRate = 0.05;

  parameters->markerBorderBits = 1;

  parameters->maxErroneousBitsInBorderRate = 0.04;

  parameters->minDistanceToBorder = 3;

  parameters->minMarkerDistanceRate = 0.1;

  parameters->minMarkerPerimeterRate = 0.03;

  parameters->maxMarkerPerimeterRate = 4.0;

  parameters->minOtsuStdDev = 5.0;

  parameters-> perspectiveRemoveIgnoredMarginPerCell = 0.13;

  parameters->perspectiveRemovePixelPerCell = 8;

  parameters->polygonalApproxAccuracyRate = 0.01;


  ROS_INFO_STREAM("Before dict");
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
   ROS_INFO_STREAM("After dict");
  cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
   ROS_INFO_STREAM("After detect");
  /*
  // Display detected markers
  cv::Mat outputImage;
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
  cv::imshow("out", outputImage);
  */
  return aruco_pose;
}


}











#endif // ARUCO_DETECTOR_HPP
