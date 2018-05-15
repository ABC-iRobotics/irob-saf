/*
 * 	vision_server_aruco_detector.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-05-15
 *
 *  Example for the usage of the generic class
 *  VisionServer. The vision server is specified
 *  by the command:
 *     VisionServer<geometry_msgs::Pose, ArucoDetector> td(nh, rate);
 *
 */



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

#include <irob_vision_support/vision_server.hpp>
#include <irob_vision_support/aruco_detector.hpp>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/aruco.hpp>

using namespace saf;

/**
 * Vision server main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
    ros::init(argc, argv, "vision_server_aruco_detector");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

  double rate;
  priv_nh.getParam("rate", rate);


    // Start Vision server
    try {
      //VisionServer<geometry_msgs::Pose, ArucoDetector> td(nh, rate);

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

      ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
    }


    // Exit
    ros::shutdown();
  return 0;
}




























