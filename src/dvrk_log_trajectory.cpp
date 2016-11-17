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
#include "dvrk/arm.hpp"
#include "dvrk/trajectory_factory.hpp"
#include <fstream>
#include <stdexcept>


int main(int argc, char **argv)
{
	// Check command line arguments
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

    dvrk::Arm psm(nh, dvrk::ArmTypes::typeForString(argv[1]), dvrk::Arm::PASSIVE);
    
    // Record trajectory
	dvrk::Trajectory<dvrk::Pose> tr_to_log(dt);
	ROS_INFO_STREAM("Start recording trajectory...");
	psm.recordTrajectory(tr_to_log);
	std::cout << std::endl << "Record stopped" << std::endl;
	
	try	{
		tr_to_log.writeToFile(argv[3]);
	} catch (const std::exception& e) {
  		std::cerr << e.what() << std::endl;
  	}
   	
   	std::cout << std::endl << "Program finished succesfully, shutting down ..."
   		 << std::endl;
   	
   	ros::shutdown();
	return 0;
}






