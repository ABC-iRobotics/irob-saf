/*
 * 	camera_info_publisher.cpp
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



#include <irob_vision_support/camera_info_publisher.hpp>


namespace ias {

CameraInfoPublisher::CameraInfoPublisher(ros::NodeHandle nh,
          std::string camera, std::string calibration):
          nh(nh), camera(camera),
          c_info_man(ros::NodeHandle(nh, camera + "/calibrated")
            , camera + "/calibrated"
            , calibration)
{
  if(!c_info_man.loadCameraInfo (calibration)){
    ROS_INFO_STREAM("Calibration file missing. Camera not calibrated");
    }
    else
    {
      c_info = c_info_man.getCameraInfo ();
    ROS_INFO_STREAM("Camera successfully calibrated");
  }

  subscribeTopics();
  advertiseTopics();
}

CameraInfoPublisher::~CameraInfoPublisher() {}

void CameraInfoPublisher::subscribeTopics()
{
  image_sub = nh.subscribe<sensor_msgs::Image>(
            camera + "/image_raw", 1000,
            &CameraInfoPublisher::imageCB,this);
    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
            camera + "/camera_info", 1000,
            &CameraInfoPublisher::cameraInfoCB,this);
}

void CameraInfoPublisher::advertiseTopics()
{
  image_pub = nh.advertise<sensor_msgs::Image>(camera + "/calibrated/image", 1000);
  camera_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>(camera + "/calibrated/camera_info", 1);
}

void CameraInfoPublisher::imageCB(
        const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::Image fwd_msg(*msg);

    image_pub.publish(fwd_msg);

    c_info.header.seq = fwd_msg.header.seq;
    c_info.header.stamp = fwd_msg.header.stamp;
    c_info.header.frame_id = fwd_msg.header.frame_id;
    camera_info_pub.publish(c_info);

}

void CameraInfoPublisher::cameraInfoCB(
        const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (c_info.width <= 0)
  {
    c_info = *msg;
    c_info.width = msg -> width;
    c_info.height = msg -> height;

  }
}



}




using namespace ias;

/**
 * Image rotator main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
    ros::init(argc, argv, "camera_info_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");


  std::string camera;
  priv_nh.getParam("camera", camera);

  std::string calibration;
  priv_nh.getParam("calibration", calibration);


    // Start Vision server
    try {
      CameraInfoPublisher cinf(nh, camera, calibration);

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




























