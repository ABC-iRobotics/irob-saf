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
#include "irob_dvrk/arm.hpp"
#include "irob_dvrk/psm.hpp"
#include "irob_math/pose.hpp"
#include "irob_math/trajectory_factory.hpp"

using namespace irob_autosurg;

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

    PSM psm(nh, ArmTypes::typeForString(arm), PSM::PASSIVE);
    
    // Record trajectory
	Trajectory<Pose> tr_to_log(dt);
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






