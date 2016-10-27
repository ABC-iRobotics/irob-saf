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


int main(int argc, char **argv)
{
	if (argc < 4) {
		std::cout 
			<< "Use with params: PSM1/PSM2; rate; filename" 
			<< std::endl;
		return 1;
	}
    ros::init(argc, argv, "irob_dvrk_log_trajectory");
    ros::NodeHandle nh;

    std::istringstream ss1(argv[2]);
	int rate_command;
	ss1 >> rate_command;	
		
	double dt = 1.0/ rate_command;
    ros::Rate loop_rate(rate_command);
	
    DVRKArm psm(nh, DVRKArmTypes::typeForString(argv[1]));

    psm.subscribe(DVRKArmTopics::GET_ROBOT_STATE);

    psm.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    
    std::ofstream logfile;
    logfile.open (argv[3], std::ofstream::out | std::ofstream::trunc);
    
    
    if (!logfile.is_open())
	{
		ROS_INFO("Cannot open file %s\n", argv[3]);
		return 1;
	}
	
	ROS_INFO("Start logging to %s\n", argv[3]);
		
  	int cnt = 0;
  	while (ros::ok() && psm.getPositionCartesianCurrent().length() < 0.001)
  	{
  		loop_rate.sleep();
  		cnt++;
  	}
  	while (ros::ok())
  	{
  		logfile << psm.getPositionCartesianCurrent() << std::endl;
  		//std::cout << psm2.getPositionCartesianCurrent() << std::endl;
  		loop_rate.sleep();
  		cnt++;
  	}
  	logfile.flush();
	logfile.close();
   	ROS_INFO("Stopping program...\n");
   	ros::shutdown();
	return 0;
}






