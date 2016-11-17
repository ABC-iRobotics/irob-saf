/*
 * dvrk_move_test.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include "dvrk_arm.hpp"
#include "psm.hpp"
#include "pose.hpp"
#include "trajectory_factory.hpp"



int main(int argc, char **argv)
{

	// Check command line arguments
	if (argc < 5) 
	{
		std::cout << 
		"Use with params: PSM1/PSM2; init_pos/no_init_pos; rate; speed" 
		<< std::endl;
		return 1;
	}
	
	std::istringstream ss1(argv[3]);
	int rate_command;
	ss1 >> rate_command;
	
	std::istringstream ss2(argv[4]);
	double speed;
	ss2 >> speed;	
		
	double dt = 1.0/ rate_command;
	Trajectory<double>* to_enable_cartesian;
	Trajectory<Pose>* circle_tr;
	
	
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_move_test");
    ros::NodeHandle nh;
    
    // Robot control
  	try {
    	PSM psm(nh, DVRKArmTypes::typeForString(argv[1]), DVRKArm::ACTIVE);
    	ros::Duration(1.0).sleep();
    	//psm.home(); 
		// Init position if necessary
		if (std::string(argv[2]) == "init_pos")
		{
			// init
			ROS_INFO_STREAM("Going to init position...");
			psm.setRobotState(DVRKArm::STATE_POSITION_JOINT);
			int init_joint_idx = 2;
			to_enable_cartesian = 
   			TrajectoryFactory::linearTrajectoryWithAcc(
   				psm.getJointStateCurrent(init_joint_idx), 
   				0.07, 0.02*speed, 1.0, dt);
 			
			psm.playTrajectory(init_joint_idx, *to_enable_cartesian);
   	 		
   		}
  
   	
   		// Do preprogrammed movement
   		ROS_INFO_STREAM("Starting programmed movement...");
   		ROS_INFO_STREAM("Loop rate:\t" << rate_command << " Hz");
   		ROS_INFO_STREAM("Speed:\t"<< speed);
   	
   	 	psm.setRobotState(DVRKArm::STATE_POSITION_CARTESIAN);

    	ros::Duration(1.0).sleep();

    	double r = 0.02;
    	
    	Pose poseto(0.0271533,	0.028501,	-0.0355035,
    		-0.0590722,	0.65363,	0.540358,	0.526584,	0.689405); 
    
    	Trajectory<Pose>* to_tr =
    		TrajectoryFactory::linearTrajectoryWithAcc(
    		psm.getPoseCurrent(), 
			poseto,
			0.01*speed,1.0, dt); 
	
		Trajectory<Pose>* back_tr =
    		TrajectoryFactory::linearTrajectoryWithAcc(
			poseto, psm.getPoseCurrent(),
			0.01*speed,1.0, dt); 
    	
    	
    	/*Trajectory<Eigen::Vector3d>* circle_tr =
    		TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, -r, 0.0),
			3.0/speed, dt);*/   
   	    
    	while(ros::ok()) {
    		psm.playTrajectory(*to_tr);
    		ros::Duration(1.0).sleep();
    		psm.playTrajectory(*back_tr);
    		ros::Duration(1.0).sleep();
    	}	
    
    	//psm.home();
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    // Remove garbage
    delete(circle_tr);
    delete(to_enable_cartesian);
    
    // Exit
    ros::shutdown();
	return 0;
}




