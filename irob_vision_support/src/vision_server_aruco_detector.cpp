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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/aruco.hpp>

#include <irob_vision_support/vision_server.hpp>
#include <irob_vision_support/aruco_detector.hpp>

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
      VisionServer<geometry_msgs::Pose, ArucoDetector> td(nh, rate);


      ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

    } catch (const std::exception& e) {
      ROS_ERROR_STREAM(e.what());
      ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
    }


    // Exit
    ros::shutdown();
  return 0;
}




























