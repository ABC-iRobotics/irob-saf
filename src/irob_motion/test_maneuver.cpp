/*
 *  move_circles_action_client.cpp
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

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include "irob_motion/maneuver_client.hpp"


int main(int argc, char **argv)
{
	ROS_INFO_STREAM("Start test_maneuver");	
	// Initialize ros node
    ros::init(argc, argv, "test_maneuver");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	std::vector<std::string> arm_names;
	priv_nh.getParam("arm_names", arm_names);
	
	ROS_INFO_STREAM(arm_names);	
	
    
    // Robot control
  	try {	
  		
  		ros::Rate loop_rate(50.0);		
		ManeuverClient mc(nh, arm_names);
	
		
		Eigen::Vector3d offset(0.02, 0.02, -0.05);
		Eigen::Vector3d offset1(-0.02, 0.02, 0.0);
		Eigen::Vector3d offset2(0.02, -0.02, 0.0);
		//ROS_INFO_STREAM(mc.getPoseCurrent(arm_names[0]));
		//ROS_INFO_STREAM(mc.getPoseCurrent(arm_names[0])+offset);
		
		std::vector<Pose> wps;
		
		wps.push_back(mc.getPoseCurrent(arm_names[0])+offset1);
		wps.push_back(mc.getPoseCurrent(arm_names[0])+offset2);

		mc.dissect(arm_names[0], mc.getPoseCurrent(arm_names[0])+offset,
		0.02, 0.0, 30.0, wps);	
		while(!mc.isDissectDone() && ros::ok())
    	{
  			loop_rate.sleep();
		}
   	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    	} catch (const std::exception& e) {
  			ROS_ERROR_STREAM(e.what());
  			ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  		}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




