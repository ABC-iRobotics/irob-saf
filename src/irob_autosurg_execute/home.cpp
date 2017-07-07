/*
 *  home.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-03-02
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include "irob_dvrk/arm.hpp"
#include "irob_dvrk/psm.hpp"
#include "irob_math/pose.hpp"
#include "irob_math/trajectory_factory.hpp"

using namespace irob_autosurg;

int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "irob_home_arm");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm_name;
	priv_nh.getParam("arm", arm_name);
	
    
    // Robot control
  	try {
    	Arm arm(nh, ArmTypes::typeForString(arm_name),
    									 Arm::ACTIVE);
    	//ros::Duration(0.5).sleep();
    	
		ROS_INFO_STREAM("Going to home position...");
   	
		arm.home();
		  	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




