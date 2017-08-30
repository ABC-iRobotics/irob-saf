/*
 * 	vision_server.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-08-30
 *
 */

#ifndef VISION_SERVER_HPP_
#define VISION_SERVER_HPP_

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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "irob_utils/pose.hpp"
#include "irob_utils/utils.hpp"


using namespace ias;

template <class MsgT>
class VisionServer {
public:
  

private:
	
    ros::NodeHandle nh;
    double rate;
    
    // States
    cv_bridge::CvImagePtr image_left_ptr;
    cv_bridge::CvImagePtr image_right_ptr;
    cv_bridge::CvImagePtr color_image_left_ptr;
    cv_bridge::CvImagePtr color_image_right_ptr;
   	cv_bridge::CvImagePtr disparity_ptr;
   	
    // Subscribers
    ros::Subscriber image_left_sub;
    ros::Subscriber image_right_sub;
    ros::Subscriber color_image_left_sub;
    ros::Subscriber color_image_right_sub;
    ros::Subscriber disparity_sub;


    // Publishers
    ros::Publisher result_pub;
    
    void subscribeTopics();
    void advertiseTopics();

public:
	VisionServer(ros::NodeHandle, double);
	~VisionServer();
	
	virtual MsgT processImages() = 0; // Makes class abstract
	
	void loopImageProcessing();


    // Callbacks
    void imageLeftCB(
    		const sensor_msgs::ImageConstPtr &);
    
    void imageRightCB(
    		const sensor_msgs::ImageConstPtr &);
    
    void colorImageLeftCB(
    		const sensor_msgs::ImageConstPtr &);
    
    void colorImageRightCB(
    		const sensor_msgs::ImageConstPtr &);
    		
   	void disparityCB(
    		const sensor_msgs::ImageConstPtr &);
    		
	
};

template <class MsgT>
VisionServer<MsgT>::VisionServer(ros::NodeHandle nh, double rate): 
										nh(nh), rate(rate)
{
	subscribeTopics();
	advertiseTopics();
	loopImageProcessing();
}

template <class MsgT>
VisionServer<MsgT>::~VisionServer() {}

template <class MsgT>
void VisionServer<MsgT>::subscribeTopics() 
{                 	            	
	image_left_sub = nh.subscribe<sensor_msgs::Image>(
   					"left/image", 1000, 
   					&VisionServer<MsgT>::imageLeftCB,this);
   					
   	image_right_sub = nh.subscribe<sensor_msgs::Image>(
   					"right/image", 1000, 
   					&VisionServer<MsgT>::imageRightCB,this);
   
   	color_image_left_sub = nh.subscribe<sensor_msgs::Image>(
   					"left/color_image", 1000, 
   					&VisionServer<MsgT>::colorImageLeftCB,this);
   					
   	color_image_right_sub = nh.subscribe<sensor_msgs::Image>(
   					"right/color_image", 1000, 
   					&VisionServer<MsgT>::colorImageRightCB,this);
   					
   	disparity_sub = nh.subscribe<sensor_msgs::DisparityImage>(
   					"disparity", 1000, 
   					&VisionServer<MsgT>::disparityCB,this);
}

template <class MsgT>
void VisionServer<MsgT>::advertiseTopics() 
{
	result_pub = nh.advertise<MsgT>("result", 1000);   
}

// Callbacks
template <class MsgT>
void VisionServer<MsgT>::imageLeftCB(
    		const sensor_msgs::ImageConstPtr& msg);
{
    try
    {
    	image_left_ptr = 
    		cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
}

template <class MsgT>
void VisionServer<MsgT>::imageRightCB(
    		const sensor_msgs::ImageConstPtr& msg);
{
    try
    {
    	image_right_ptr = 
    		cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
}

template <class MsgT>
void VisionServer<MsgT>::colorImageLeftCB(
    		const sensor_msgs::ImageConstPtr& msg);
{
    try
    {
    	clor_image_left_ptr = 
    		cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
}

template <class MsgT>
void VisionServer<MsgT>::colorImageRightCB(
    		const sensor_msgs::ImageConstPtr& msg);
{
    try
    {
    	color_image_right_ptr = 
    		cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
}

template <class MsgT>
void VisionServer<MsgT>::disparityCB(
    		const sensor_msgs::DisparityImageConstPtr& msg);
{
    try
    {
    	disparity_ptr = 
    		cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      	ROS_ERROR("cv_bridge exception: %s", e.what());
      	return;
    }
}

template <class MsgT>
void VisionServer<MsgT>::loopImageProcessing()
{
	ros::Rate loop_rate(rate);
	while (ros::ok())
  	{
  		ros::spinOnce(); 
  		// Virtual function
  		MsgT res = processImages();
  		
  		result_pub.publish(result);
  		ros::spinOnce(); 
  		
  		loop_rate.sleep();
  	}
}


































#endif /* VISION_SERVER_HPP_ */
