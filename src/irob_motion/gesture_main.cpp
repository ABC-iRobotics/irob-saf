/*
 *  gesture_main.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-21
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include "irob_motion/gesture_server.hpp"


using namespace ias;

int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "gesture_server");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	std::string arm_name;
	priv_nh.getParam("arm_name", arm_name);
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
    
    // StartGesture server
  	try {
    	GestureServer gesture(nh, arm_name, 1.0/rate_command);
    	
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




