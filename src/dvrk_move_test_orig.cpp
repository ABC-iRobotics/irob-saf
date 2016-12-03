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
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory_factory.hpp"



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
	dvrk::Trajectory<double> to_enable_cartesian;
	dvrk::Trajectory<dvrk::Pose> circle_tr;
	
	
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_move_test");
    ros::NodeHandle nh;
    
    // Robot control
  	try {
    	dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(argv[1]),
    	 dvrk::PSM::ACTIVE);
    	ros::Duration(1.0).sleep();
    	//psm.home(); 
		// Init position if necessary
		if (std::string(argv[2]) == "init_pos")
		{
			// init
			ROS_INFO_STREAM("Going to init position...");
			psm.setRobotState(dvrk::PSM::STATE_POSITION_JOINT);
			int init_joint_idx = 2;
			to_enable_cartesian = 
   				dvrk::TrajectoryFactory::
   					linearTrajectoryWithSmoothAcceleration(
   						psm.getJointStateCurrent(init_joint_idx), 
   						0.07,
   						1.0/speed, 0.3/speed, dt);
 			
			psm.playTrajectory(init_joint_idx, to_enable_cartesian);
   	 		
   		}
  
   	
   		// Do preprogrammed movement
   		ROS_INFO_STREAM("Starting programmed movement...");
   		ROS_INFO_STREAM("Loop rate:\t" << rate_command << " Hz");
   		ROS_INFO_STREAM("Speed:\t"<< speed);
   	
   	 	psm.setRobotState(dvrk::PSM::STATE_POSITION_CARTESIAN);

    	ros::Duration(1.0).sleep();

    	double r = 0.02;
    	
    	//dvrk::Pose poseto(0.0271533,	0.028501,	-0.0355035,
    	//	-0.0590722,	0.65363,	0.540358,	0.526584,	0.689405); 
    
    	/*dvrk::Trajectory<dvrk::Pose> 
    		to_tr(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					poseto,
					0.5,2.0, dt)); */
	
		/*dvrk::Trajectory<dvrk::Pose> 
			back_tr(dvrk::
				TrajectoryFactory::linearTrajectoryWithSmoothAcceleration(
					poseto, psm.getPoseCurrent(),
					0.5,2.0, dt)); */
    	
    	
    	dvrk::Trajectory<Eigen::Vector3d> circle_tr =
    		dvrk::TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, -r, 0.0),
			3.0/speed, dt); 
		
		dvrk::Trajectory<Eigen::Vector3d> 
			back_tr(dvrk::
				TrajectoryFactory::linearTrajectoryWithSmoothAcceleration(
					circle_tr[circle_tr.size()-1], circle_tr[0],
					0.5,2.0, dt));  
   	    
    	while(ros::ok()) {
    		psm.playTrajectory(circle_tr);
    		ros::Duration(1.0).sleep();
    		psm.playTrajectory(back_tr);
    		ros::Duration(1.0).sleep();
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




