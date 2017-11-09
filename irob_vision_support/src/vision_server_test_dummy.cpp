/*
 * 	vision_server_test_dummy.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-08-30
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


#include <irob_vision_support/vision_server.hpp>
#include <irob_vision_support/dummy_image_processor.hpp>

using namespace ias;

/**
 * Vision server main 
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "vision_server_test_dummy");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
	
	double rate;
	priv_nh.getParam("rate", rate);
	
    
    // Start Vision server
  	try {
    	VisionServer<geometry_msgs::Pose, DummyImageProcessor> td(nh, rate);
    		
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




























