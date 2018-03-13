/*
 * 	camera_info_publisher.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-03-13
 *
 * This node loads the camera calibration file and
 * publishes the camera info topic. Necessary since the
 * camera driver will not always load the camera info
 * itself. Insert between the preprocessing and the stereo vision node.
 *
 */

#ifndef CAMERA_INFO_PUBLISHER_HPP
#define CAMERA_INFO_PUBLISHER_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"


namespace ias {

class CameraInfoPublisher {
public:


private:

    ros::NodeHandle nh;
    std::string camera;

    sensor_msgs::CameraInfo c_info;
    camera_info_manager::CameraInfoManager c_info_man;


    // Subscribers
    ros::Subscriber image_sub;
  ros::Subscriber camera_info_sub;

    // Publishers
    ros::Publisher image_pub;
    ros::Publisher camera_info_pub;

    void subscribeTopics();
    void advertiseTopics();

public:
  CameraInfoPublisher(ros::NodeHandle, std::string, std::string);
  ~CameraInfoPublisher();



    // Callbacks
    void imageCB(
        const sensor_msgs::ImageConstPtr &);

    void cameraInfoCB(
        const sensor_msgs::CameraInfoConstPtr &);



};





}


#endif // CAMERA_INFO_PUBLISHER_HPP
