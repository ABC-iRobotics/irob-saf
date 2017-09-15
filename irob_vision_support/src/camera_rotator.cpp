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



#include <irob_vision_support/camera_rotator.hpp>


namespace ias {

CameraRotator::CameraRotator(ros::NodeHandle nh, 
					std::string camera, std::string calibration, int angle):
					nh(nh), camera(camera),
					c_info_man(ros::NodeHandle(nh, camera + "/rotated")
						, camera + "/rotated"
						, calibration)
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

CameraRotator::~CameraRotator() {}

void CameraRotator::subscribeTopics() 
{                 	            	
	image_sub = nh.subscribe<sensor_msgs::Image>(
   					camera + "/image_raw", 1000, 
   					&CameraRotator::imageCB,this);
   	camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
   					camera + "/camera_info", 1000, 
   					&CameraRotator::cameraInfoCB,this);
}

void CameraRotator::advertiseTopics() 
{
	image_pub = nh.advertise<sensor_msgs::Image>(camera + "/rotated/image", 1000); 
	camera_info_pub = 
    	nh.advertise<sensor_msgs::CameraInfo>(camera + "/rotated/camera_info", 1); 
}

void CameraRotator::imageCB(
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

void CameraRotator::cameraInfoCB(
    		const sensor_msgs::CameraInfoConstPtr& msg)
{
	if (c_info.width <= 0)
	{
		c_info = *msg;
		if (angle_code == cv::RotateFlags::ROTATE_90_CLOCKWISE 
			|| angle_code == cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE)
		{
			c_info.width = msg -> height;
			c_info.height = msg -> width;
 		}
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
    ros::init(argc, argv, "camera_rotator");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
	
	int angle;
	priv_nh.getParam("angle", angle);
	
	std::string camera;
	priv_nh.getParam("camera", camera);
	
	std::string calibration;
	priv_nh.getParam("calibration", calibration);
	
    
    // Start Vision server
  	try {
    	CameraRotator rot(nh, camera, calibration, angle);
    		
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




























