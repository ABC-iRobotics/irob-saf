/*
 *  insert_tool.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
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
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory_factory.hpp"



int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_move_test");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm;
	priv_nh.getParam("arm", arm);
	
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
	double speed;
	priv_nh.getParam("speed", speed);
	
	double depth;
	priv_nh.getParam("depth", depth);

		
	double dt = 1.0/ rate_command;
	dvrk::Trajectory<double> to_enable_cartesian;
	dvrk::Trajectory<dvrk::Pose> circle_tr;
    
    // Robot control
  	try {
    	dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(arm), dvrk::PSM::ACTIVE);
    	//ros::Duration(0.5).sleep();
    	
		ROS_INFO_STREAM("Inserting tool past cannula...");
   		ROS_INFO_STREAM("Loop rate:\t" << rate_command << " Hz");
   		ROS_INFO_STREAM("Speed:\t"<< speed);
   		ROS_INFO_STREAM("Insertion depth:\t"<< depth << " mm");
   	
		psm.setRobotState(dvrk::PSM::STATE_POSITION_JOINT);
		int init_joint_idx = 2;
		to_enable_cartesian = 
   			dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						psm.getJointStateCurrent(init_joint_idx), 
   						depth/1000.0,
   						1.0/speed, 0.3/speed, dt);
 			
		psm.playTrajectory(init_joint_idx, to_enable_cartesian);
   	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




