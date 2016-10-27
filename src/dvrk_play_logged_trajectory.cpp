/*
 * dvrk_move_test.cpp
 *
 *  Created on: 2016. okt. 27.
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
#include <string>
#include <stdexcept>


int main(int argc, char **argv)
{
	
	if (argc < 4) 
	{
		std::cout 
			<< "Use with params: PSM1/PSM2; init_pos/no_init_pos; filename" 
			<< std::endl;
		return 1;
	}
	
	// Initialize node
    ros::init(argc, argv, "irob_dvrk_play_logged_trajectory");
    ros::NodeHandle nh;

    DVRKArm psm(nh, DVRKArmTypes::typeForString(argv[1]));

    psm.subscribe(DVRKArmTopics::GET_ROBOT_STATE);
    psm.advertise(DVRKArmTopics::SET_ROBOT_STATE);

    psm.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm.advertise(DVRKArmTopics::SET_POSITION_JOINT);

    psm.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    psm.advertise(DVRKArmTopics::SET_POSITION_CARTESIAN);

    //psm.home();
    
    // Load trajectory from file
    try	{
		Trajectory<Vector3D> tr(argv[3]);
    	ROS_INFO(
    	"Trajectory of %d points with %f sample rate succesfully loaded from %s"
    		,tr.size(), tr.dt, argv[3]);
    
    	// Init position if necessary
    	if (std::string(argv[2]) == "init_pos")
		{
			ROS_INFO("Going to init position...");
			psm.setRobotState(DVRKArm::STATE_POSITION_JOINT);
			int init_joint_idx = 2;
			Trajectory<double>* to_enable_cartesian = 
   			TrajectoryFactory::linearTrajectory(
   				psm.getJointStateCurrent(init_joint_idx), 
   				0.07, 2.0, 0.05);
			psm.playTrajectory(init_joint_idx, *to_enable_cartesian);
   	 
   			delete(to_enable_cartesian);
   		}

		// Go to the start point of loaded trajectory
		ROS_INFO("Going to start point of loaded trajectory...");
		psm.setRobotState(DVRKArm::STATE_POSITION_CARTESIAN);
		Trajectory<Vector3D>* to_start = 
   			TrajectoryFactory::linearTrajectory(
   				psm.getPositionCartesianCurrent(), tr[0], 2.0, tr.dt);
		
		psm.playTrajectory(*to_start);
   		ros::Duration(0.5).sleep();
   		delete(to_start);	
	
		// Play loaded trajectory
		ROS_INFO("Playing loaded trajectory...");
    	psm.playTrajectory(tr);
    
    	std::cout << std::endl << "Stopping program..." << std::endl;
    
    } catch (const std::exception& e) {
  		ROS_ERROR("%s", e.what());
  	}
	return 0;
}




