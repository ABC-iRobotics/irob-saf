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
#include <actionlib/server/simple_action_server.h>
#include <irob_autosurg/HomeAction.h>
#include <irob_autosurg/FollowTrajectoryAction.h>

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
    	irob_dvrk::Arm psm(nh, irob_dvrk::ArmTypes::typeForString(arm_name),
    	 irob_dvrk::Arm::ACTIVE);
    	 
    	/* Trajectory<Pose> circle_tr =
    	 	TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, -r, 0.0),
			3.0/speed, dt);*/

    	
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




