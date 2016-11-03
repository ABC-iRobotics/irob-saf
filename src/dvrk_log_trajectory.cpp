/*
 * dvrk_log_trajectory.cpp
 *
 *  Created on: 2016. okt. 26.
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
#include "trajectory_factory.hpp"
#include <fstream>
#include <stdexcept>


int main(int argc, char **argv)
{
	if (argc < 4) {
		std::cout 
			<< "Use with params: PSM1/PSM2; rate; filename" 
			<< std::endl;
		return 1;
	}
	
	
    std::istringstream ss1(argv[2]);
	int rate_command;
	ss1 >> rate_command;	
		
	double dt = 1.0/ rate_command;
	
	
	// Initialize node
    ros::init(argc, argv, "irob_dvrk_log_trajectory");
    ros::NodeHandle nh;

    DVRKArm psm(nh, DVRKArmTypes::typeForString(argv[1]));
	
    psm.subscribe(DVRKArmTopics::GET_ROBOT_STATE);
    psm.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    
    // Record trajectory
	Trajectory<Pose> tr_to_log(dt);
	ROS_INFO("Start recording trajectory...");
	psm.recordTrajectory(tr_to_log);
	std::cout << std::endl << "Record stopped" << std::endl;
	
	try	{
		tr_to_log.writeToFile(argv[3]);
	} catch (const std::exception& e) {
  		std::cerr << e.what() << std::endl;
  	}
   	
   	std::cout << std::endl << "Stopping program..." << std::endl;
   	ros::shutdown();
	return 0;
}






