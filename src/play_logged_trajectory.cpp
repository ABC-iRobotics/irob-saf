/*
 *  play_logged_trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-27
 *  
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/trajectory_factory.hpp"




int main(int argc, char **argv)
{
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_play_logged_trajectory");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm;
	priv_nh.getParam("arm", arm);
	
	std::string filename;
	priv_nh.getParam("filename", filename);
	
	double speed;
	priv_nh.getParam("speed", speed);

	// Robot control
	
  	try {
    	dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(arm),
    	 dvrk::PSM::ACTIVE);
    	//psm.home();
    
    	// Load trajectory from file
		dvrk::Trajectory<dvrk::Pose> tr(filename);
    	ROS_INFO_STREAM(
    	"Trajectory of "<< tr.size() << " points with "
    					<< 1.0/tr.dt << " sample rate succesfully loaded from "
    					<< filename);
    	

		// Go to the start point of loaded trajectory
		ROS_INFO("Going to start point of loaded trajectory...");
		psm.setRobotState(dvrk::Arm::STATE_POSITION_CARTESIAN);
		dvrk::Trajectory<dvrk::Pose> to_start = 
   			dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					tr[0],
					2.0, 0.2, tr.dt); 
					
		tr.dt /= speed;
		psm.playTrajectory(to_start);
   		ros::Duration(0.5).sleep();
   			
	
		// Play loaded trajectory
		ROS_INFO("Playing loaded trajectory...");

   		ROS_INFO_STREAM("Speed:\t"<< speed);
    	psm.playTrajectory(tr);
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    
    }catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
  	
    ros::shutdown();
	return 0;
}




