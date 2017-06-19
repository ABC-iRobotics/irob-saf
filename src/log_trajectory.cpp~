/*
 *  dvrk_log_trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-26
 *  
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
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
	
	std::string filename;
	priv_nh.getParam("filename", filename);

	double dt = 1.0/ rate_command;

    dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(arm), dvrk::PSM::PASSIVE);
    
    // Record trajectory
	dvrk::Trajectory<dvrk::Pose> tr_to_log(dt);
	ROS_INFO_STREAM("Start recording trajectory...");
	psm.recordTrajectory(tr_to_log);
	std::cout << std::endl << "Record stopped" << std::endl;
	
	try	{
		tr_to_log.writeToFile(filename);
	} catch (const std::exception& e) {
  		std::cerr << e.what() << std::endl;
  	}
   	
   	std::cout << std::endl << "Program finished succesfully, shutting down ..."
   		 << std::endl;
   	
   	ros::shutdown();
	return 0;
}






