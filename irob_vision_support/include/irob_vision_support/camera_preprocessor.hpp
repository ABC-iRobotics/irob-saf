/*
 * 	camera_preprocessor.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2087-03-13
 *
 *  Node to do basic preprocessing algorithms
 *  on camera stream. Yet only averaging frames is
 *  implemented.
 */

#ifndef CAMERA_PREPROCESSOR_HPP_
#define CAMERA_PREPROCESSOR_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"


namespace saf {

class CameraPreprocessor {
public:

private:

  typedef enum Command {
    NONE,
    AVG_ADJACENT
  } Command;

  ros::NodeHandle nh;
  std::string camera;
  Command command;
  cv::Mat prev_image;


  // Subscribers
  ros::Subscriber image_sub;
  ros::Subscriber camera_info_sub;

  // Publishers
  ros::Publisher image_pub;
  ros::Publisher camera_info_pub;

  ros::ServiceServer cam_info_service;

  void subscribeTopics();
  void advertiseTopics();

public:
  CameraPreprocessor(ros::NodeHandle, std::string, std::string);
  ~CameraPreprocessor();



  // Callbacks
  void imageCB(
      const sensor_msgs::ImageConstPtr &);

  void cameraInfoCB(
      const sensor_msgs::CameraInfoConstPtr &);

  bool setCameraInfoCB(sensor_msgs::SetCameraInfo::Request& request, sensor_msgs::SetCameraInfo::Response& response);


};





}
#endif /* CAMERA_PREPROCESSOR_HPP_ */
