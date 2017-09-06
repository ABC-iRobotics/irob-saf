/*
 * 	vision_server_test_dummy.cpp
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



#include "irob_vision/image_rotator.hpp"


namespace ias {

ImageRotator::ImageRotator(ros::NodeHandle nh, int angle): nh(nh)
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

	subscribeTopics();
	advertiseTopics();
}

ImageRotator::~ImageRotator() {}

void ImageRotator::subscribeTopics() 
{                 	            	
	image_sub = nh.subscribe<sensor_msgs::Image>(
   					"image", 1000, 
   					&ImageRotator::imageCB,this);
}

void ImageRotator::advertiseTopics() 
{
	image_pub = nh.advertise<sensor_msgs::Image>("rotated/image", 1000);   
}

void ImageRotator::imageCB(
    		const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
    	cv_bridge::CvImagePtr image_ptr = 
    		cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	
    	cv::Mat rotated_image;

    	cv::rotate(image_ptr->image, rotated_image, angle_code);

    	sensor_msgs::ImagePtr rotated_msg = 
    		cv_bridge::CvImage(std_msgs::Header(), "bgr8", 
    				rotated_image).toImageMsg();			
    	image_pub.publish(rotated_msg);			
    	
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
    ros::init(argc, argv, "image_rotator");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
	
	int angle;
	priv_nh.getParam("angle", angle);
	
    
    // Start Vision server
  	try {
    	ImageRotator rot(nh, angle);
    		
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




























