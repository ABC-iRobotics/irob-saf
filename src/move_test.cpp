/*
 *  dvrk_move_test.cpp
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
#include <numeric>
#include <chrono>
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
    
    double dt = 1.0/ rate_command;
    
    // Robot control
  	try {
    	dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(arm),
    		dvrk::PSM::ACTIVE);
    	ros::Duration(1.0).sleep();
    	   	
   		// Do preprogrammed movement
   		ROS_INFO_STREAM("Starting programmed movement...");
   		ROS_INFO_STREAM("Loop rate:\t" << rate_command << " Hz");
   		ROS_INFO_STREAM("Speed:\t"<< speed);
   	
   	 	psm.setRobotState(dvrk::PSM::STATE_POSITION_JOINT);

    	ros::Duration(0.5).sleep();
		
		// rotation
		int joint_idx = 3;
    	double x1 = -0.9;
    	double x2 = 2.5;
    	double T = 5.0/speed;
    	double Tacc = T*0.1;
    	
    	// translation
    	/*int joint_idx = 2;
    	double x1 = 0.145;
    	double x2 = 0.179;
    	double T = 5.0/speed;
    	double Tacc = T*0.1;*/
    	
    	dvrk::Trajectory<double> 
    		init_tr(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getJointStateCurrent(joint_idx), 
					x1,
					T, Tacc, dt)); 
 
    
    	dvrk::Trajectory<double> 
    		to_tr(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				x1, 
					x2,
					T, Tacc, dt)); 
	
		dvrk::Trajectory<double> 
			back_tr(dvrk::TrajectoryFactory::
				linearTrajectoryWithSmoothAcceleration(
					x2,
					x1,
					T, Tacc, dt)); 
    	
    	// Move
    	
    	ROS_INFO_STREAM("Going to start position...");	
    		
    	auto start = std::chrono::high_resolution_clock::now();
    	std::chrono::duration<double> elapsed;
    	std::chrono::duration<double> testT(30.0);
		    			
    	psm.playTrajectory(joint_idx, init_tr);
    	ros::Duration(0.5).sleep();
     
   	    while(ros::ok()) {
			//
			elapsed =
	 			std::chrono::high_resolution_clock::now()-start;
	 		ROS_INFO("Time elapsed: %f s", (elapsed));
	 		if (elapsed >= testT)
	 			break;		
   	    	ROS_INFO_STREAM("Going to position 1...");	
    		
    		psm.playTrajectory(joint_idx, to_tr);
    		//
    		elapsed =
	 			std::chrono::high_resolution_clock::now()-start;
	 		ROS_INFO("Time elapsed: %f s", (elapsed));
	 		if (elapsed >= testT)
	 			break;	
    		
    		ros::Duration(0.5).sleep();
    		//
    		elapsed =
	 			std::chrono::high_resolution_clock::now()-start;
	 		ROS_INFO("Time elapsed: %f s", (elapsed));
	 		if (elapsed >= testT)
	 			break;	
    		ROS_INFO_STREAM("Going to position 2...");	
    		
    		psm.playTrajectory(joint_idx, back_tr);
    		//
    		elapsed =
	 			std::chrono::high_resolution_clock::now()-start;
	 		ROS_INFO("Time elapsed: %f s", (elapsed));
	 		if (elapsed >= testT)
	 			break;	
    		
    		ros::Duration(0.5).sleep();
    	}	
    
    	//psm.home();
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




