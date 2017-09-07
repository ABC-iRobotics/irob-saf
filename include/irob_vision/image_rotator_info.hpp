/*
 * 	image_rotator.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-09-06
 *
 */

#ifndef IMAGE_ROTATOR_INFO_HPP_
#define IMAGE_ROTATOR_INFO_HPP_

#include <iostream>
#include <sstream>
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

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"


namespace ias {

class ImageRotatorInfo {
public:
  

private:
	
    ros::NodeHandle nh;
    int angle_code;
    
    sensor_msgs::CameraInfo c_info;
    camera_info_manager::CameraInfoManager c_info_man;
   	
    // Subscribers
    ros::Subscriber image_sub;


    // Publishers
    ros::Publisher image_pub;
     ros::Publisher camera_info_pub;
    
    void subscribeTopics();
    void advertiseTopics();

public:
	ImageRotatorInfo(ros::NodeHandle, int);
	~ImageRotatorInfo();
	


    // Callbacks
    void imageCB(
    		const sensor_msgs::ImageConstPtr &);
    
    		
	
};





}
#endif /* IMAGE_ROTATOR_INFO_HPP_ */
