/*
 * 	image_rotator.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-09-06
 *  
 */



#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>



#include "irob_vision/image_rotator_info.hpp"


namespace ias {

ImageRotatorInfo::ImageRotatorInfo(ros::NodeHandle nh, int angle): nh(nh),
	 c_info_man(nh, "rotated", "package://raspicam/calibrations/rotated.yaml")
{
	switch (angle)
	{
		case 90: 
			angle_code = cv::RotateFlags::ROTATE_90_CLOCKWISE;	// 0
			break;
		case 180: 
			angle_code = cv::RotateFlags::ROTATE_180;	// 1
			break;
		case 270: 
			angle_code = cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE;	// 2
			break;
		case -90: 
			angle_code = cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE;	// 2
			break;
		default: 
			angle_code = cv::RotateFlags::ROTATE_90_CLOCKWISE;	// 0
			break;	
	}
	
	if(!c_info_man.loadCameraInfo (	
			"package://raspicam/calibrations/rotated.yaml")){
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

ImageRotatorInfo::~ImageRotatorInfo() {}

void ImageRotatorInfo::subscribeTopics() 
{                 	            	
	image_sub = nh.subscribe<sensor_msgs::Image>(
   					"image", 1000, 
   					&ImageRotatorInfo::imageCB,this);
}

void ImageRotatorInfo::advertiseTopics() 
{
	image_pub = nh.advertise<sensor_msgs::Image>("rotated/image", 1000); 
	camera_info_pub = 
    	nh.advertise<sensor_msgs::CameraInfo>("rotated/camera_info", 1); 
}

void ImageRotatorInfo::imageCB(
    		const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
    	cv_bridge::CvImagePtr image_ptr = 
    		cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	
    	cv::Mat rotated_image;

    	cv::rotate(image_ptr->image, rotated_image, angle_code);

    	sensor_msgs::ImagePtr rotated_msg = 
    		cv_bridge::CvImage(msg->header, "bgr8", 
    				rotated_image).toImageMsg();			
    	image_pub.publish(rotated_msg);
    	
		c_info.header.seq = rotated_msg->header.seq;
		c_info.header.stamp = rotated_msg->header.stamp;
		c_info.header.frame_id = rotated_msg->header.frame_id;
		camera_info_pub.publish(c_info);
    				
    	
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
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
    ros::init(argc, argv, "image_rotator_info");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
	
	int angle;
	priv_nh.getParam("angle", angle);
	
    
    // Start Vision server
  	try {
    	ImageRotatorInfo rot(nh, angle);
    		
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




























