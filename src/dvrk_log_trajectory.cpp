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
	if (argc < 2) {
		std::cout << std::endl << "Give filename in command line argument!" << std::endl;
		return 0;
	}
    ros::init(argc, argv, "irob_dvrk_log_trajectory");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    DVRKArm psm2(nh, DVRKArmTypes::PSM2);

    psm2.subscribe(DVRKArmTopics::GET_ROBOT_STATE);

    psm2.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm2.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    
    std::ofstream logfile;
    logfile.open (argv[1], std::ofstream::out | std::ofstream::trunc);
    if (logfile.is_open())
		std::cout << std::endl << "Start logging to "<< argv[1]  << std::endl;
	else
		std::cout << std::endl << "Cannot open file  "<< argv[1]  << std::endl;
  	int cnt = 0;
  	while (ros::ok() && psm2.getPositionCartesianCurrent().length() < 0.001)
  	{
  		ros::Duration(0.1).sleep();
  		cnt++;
  	}
  	while (ros::ok())
  	{
  		logfile << psm2.getPositionCartesianCurrent() << std::endl;
  		//std::cout << psm2.getPositionCartesianCurrent() << std::endl;
  		ros::Duration(0.1).sleep();
  		cnt++;
  	}
  	logfile.flush();
	logfile.close();
    std::cout << std::endl << "Stopping program..." << std::endl;
	return 0;
}






