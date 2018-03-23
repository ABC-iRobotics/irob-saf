/*
 * 	camera_preprocessor.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-03-13
 *
 */



#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>



#include <irob_vision_support/camera_preprocessor.hpp>


namespace ias {

CameraPreprocessor::CameraPreprocessor(ros::NodeHandle nh,
                                       std::string camera, std::string command):
  nh(nh), camera(camera)

{

  if (!command.compare("none"))
  {
    this->command = Command::NONE;
  }
  else if (!command.compare("avg_adjacent"))
  {
    this->command = Command::AVG_ADJACENT;
  }

  subscribeTopics();
  advertiseTopics();
  cam_info_service = nh.advertiseService("preprocessed/" + camera + "/set_camera_info", &CameraPreprocessor::setCameraInfoCB, this);
}

CameraPreprocessor::~CameraPreprocessor() {}

void CameraPreprocessor::subscribeTopics()
{
  image_sub = nh.subscribe<sensor_msgs::Image>(
        camera + "/image_raw", 1000,
        &CameraPreprocessor::imageCB,this);
  camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
            camera + "/camera_info", 1000,
&CameraPreprocessor::cameraInfoCB,this);
}

void CameraPreprocessor::advertiseTopics()
{
  image_pub = nh.advertise<sensor_msgs::Image>("preprocessed/" + camera + "/image_raw", 1000);
  camera_info_pub =
 nh.advertise<sensor_msgs::CameraInfo>("preprocessed/" + camera + "/camera_info", 1);

}

void CameraPreprocessor::imageCB(
    const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr image_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat processed_image;
    image_ptr->image.copyTo(processed_image);

    switch(command)
    {
    case Command::NONE:
      break;
    case Command::AVG_ADJACENT:
      if(!prev_image.empty())
        processed_image = (processed_image + prev_image) / 2.0;
      break;
    }


    sensor_msgs::ImagePtr processed_msg =
        cv_bridge::CvImage(msg->header, "bgr8",
                           processed_image).toImageMsg();
    image_pub.publish(processed_msg);
    image_ptr->image.copyTo(prev_image);

  } catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void CameraPreprocessor::cameraInfoCB(
        const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_pub.publish(*msg);
}

bool CameraPreprocessor::setCameraInfoCB(sensor_msgs::SetCameraInfo::Request& request, sensor_msgs::SetCameraInfo::Response& response)
{
  ros::ServiceClient client = nh.serviceClient<sensor_msgs::SetCameraInfo>(camera +"/set_camera_info");

  sensor_msgs::SetCameraInfo srv;

  srv.request = request;
  srv.response = response;

  if (client.call(srv))
  {
    response = srv.response;
    return true;
  }

  response = srv.response;
  return false;
}




}




using namespace ias;

/**
 * Image preprocessor main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
  ros::init(argc, argv, "camera_preprocessor");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string command;
  priv_nh.getParam("command", command);

  std::string camera;
  priv_nh.getParam("camera", camera);

  std::string calibration;
  priv_nh.getParam("calibration", calibration);


  // Start Vision server
  try {
    CameraPreprocessor prep(nh, camera, command);

    ros::spin();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}




























