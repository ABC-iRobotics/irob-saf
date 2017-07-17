/*
 *  dvrk_action_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-03-07-08
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
#include "irob_utils/pose.hpp"
#include "irob_utils/trajectory_factory.hpp"


using namespace irob_autosurg;

int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "irob_home_arm");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm_typ;
	priv_nh.getParam("arm_typ", arm_typ);

	std::string arm_name;
	priv_nh.getParam("arm_name", arm_name);
	


	
    
    // Robot control
  	try {
    	irob_dvrk::PSM psm(nh, irob_dvrk::ArmTypes::typeForString(arm_typ),
    	 arm_name, irob_dvrk::PSM::ACTIVE);
    	
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




